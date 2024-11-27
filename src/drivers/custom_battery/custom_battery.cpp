// Implementation file for the I2C float reader driver (CustomBatteryDriver.cpp)
#include "custom_battery.hpp"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
//#include <uORB/topics/battery_status_2.h>
#include <string.h>

CustomBatteryDriver::CustomBatteryDriver(int instance_id, int bus, int address) :
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
    I2C(2, "/dev/i2c_float_reader", bus, address, 100000),
    ModuleParams(nullptr),
    _instance_id(instance_id),
    _sample_perf(perf_alloc(PC_ELAPSED, "i2c_float_reader: read"))
{
    memset(&_report, 0, sizeof(_report));
    memset(&_battery_status, 0, sizeof(_battery_status));
}

CustomBatteryDriver::~CustomBatteryDriver()
{
    perf_free(_sample_perf);
    orb_unadvertise(_battery_status_topic_pub);
}

int CustomBatteryDriver::sendInitBatteryStatus()
{
    // Initialize battery status with default values
    _battery_status.timestamp = hrt_absolute_time();
    _battery_status.voltage_v = 12.0f;
    _battery_status.current_a = 0.0f;
    _battery_status.remaining = 1;
    _battery_status.cell_count = 10;
    _battery_status.connected = true;
    _battery_status.cycle_count = 100;
    // Set unique parameters based on instance ID
    if (_instance_id == _bat_id_1.get()) {
        _battery_status.serial_number = 12345;
        _battery_status.source = 1;
    } else if (_instance_id == _bat_id_2.get()) {
        _battery_status.serial_number = 23456;
        _battery_status.source = 2;
    }

    _battery_status.state_of_health = 95; // Placeholder state of health (95%)
    _battery_status.warning = battery_status_s::BATTERY_WARNING_NONE; // Default warning state
    _battery_status.id = _instance_id; // Set the instance ID
    battery_id = _instance_id;

    // Spread the voltage across 10 cells (even distribution for now)
    setPerCellVoltage(_battery_status.voltage_v);

    // Advertise the battery status topic
    _battery_status_topic_pub = orb_advertise_multi(ORB_ID(battery_status), &_battery_status, &_instance_id);
    if (_battery_status_topic_pub == nullptr) {
        PX4_ERR("Failed to advertise battery_status topic for battery %d", _instance_id);
        return PX4_ERROR;
    }

    // Publish initial battery status to ensure it is properly initialized
    orb_publish(ORB_ID(battery_status), _battery_status_topic_pub, &_battery_status);

    return PX4_OK;
}

void CustomBatteryDriver::setPerCellVoltage(float voltage)
{
    float per_cell_voltage = voltage / 10;
    for (int i = 0; i < 10; ++i) {
        _battery_status.voltage_cell_v[i] = per_cell_voltage;
    }
}

void CustomBatteryDriver::parameters_update()
{
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        // If any parameter updated, call updateParams() to check if
        // this class attributes need updating (and do so).
        updateParams();
    }
}

int CustomBatteryDriver::init()
{
    usleep(200000);
    int ret = I2C::init();

    if (ret != PX4_OK) {
        PX4_ERR("I2C init failed");
        return ret;
    }

    ret = sendInitBatteryStatus();

    if (ret != PX4_OK) {
        PX4_ERR("Failed to send initialization message to battery_status");
        return ret;
    }

    return PX4_OK;
}

void CustomBatteryDriver::start()
{
    // Start collecting data
    ScheduleOnInterval(500000); // 2 Hz rate
}

void CustomBatteryDriver::triggerWarning(uint8_t warning_severity, uint8_t id, const char *cause)
{
    // Log the appropriate warning or error based on the severity
    if (cause != nullptr) {
        switch (warning_severity) {
            case battery_status_s::BATTERY_WARNING_NONE:
                PX4_INFO("%s: %d", cause, id);
                break;

            case battery_status_s::BATTERY_WARNING_LOW:
                PX4_WARN("%s: %d", cause, id);
                break;

            case battery_status_s::BATTERY_WARNING_CRITICAL:
                PX4_ERR("%s: %d", cause, id);
                break;

            default:
                PX4_DEBUG("Unrecognized warning severity");
                break;
        }
    }

    // Update the battery status warning level and publish
    _battery_status.warning = warning_severity;
    if (orb_publish(ORB_ID(battery_status), _battery_status_topic_pub, &_battery_status) != PX4_OK) {
        PX4_ERR("Failed to trigger warning");
    }
}

void CustomBatteryDriver::triggerWarningOnce(uint8_t warning_severity, uint8_t id, const char *cause)
{
    // Only escalate if the new warning is more severe than the current warning
    if (!triggered_warnings[id]) {
        triggerWarning(warning_severity, id, cause);
        triggered_warnings[id] = true;
    } else {
        PX4_DEBUG("Warning %d was already triggered before", id);
    }
}

int CustomBatteryDriver::collect()
{
    if (_emergency_triggered) {
        PX4_WARN("Collection stopped due to emergency being triggered");
        return PX4_ERROR;
    }

    perf_begin(_sample_perf);

    uint8_t buffer[12] = {0};
    int ret = 0;
    unsigned int attempts = 0;

    // Attempt to transfer up to 10 times if it fails
    while (attempts < 10 && ((ret = transfer(nullptr, 0, buffer, 12)) != PX4_OK)) {
        if (i2c_comm_state) {
            PX4_WARN("I2C transfer attempt %d failed", attempts + 1);
        }
        attempts++;
    }

    if (ret == PX4_OK) {
        if (!i2c_comm_state) {
            PX4_DEBUG("I2C connection regained");
        }
        i2c_comm_state = true;
    } else if (attempts == 10 || ret < 0) {
        if (i2c_comm_state) {
            PX4_ERR("I2C transfer failed after 10 attempts");
        }
        i2c_comm_state = false;
        triggerWarningOnce(battery_status_s::BATTERY_WARNING_CRITICAL, I2C_CONNECTION_LOST, "I2C connection lost");
    }

    // Unpack the received data into floats
    memcpy(&_report.current, &buffer[0], sizeof(float));
    memcpy(&_report.voltage1, &buffer[4], sizeof(float));
    memcpy(&_report.voltage2, &buffer[8], sizeof(float));
    _report.timestamp = hrt_absolute_time();

    // Log the data
    PX4_DEBUG("Collected data - Current: %.2f, Voltage1: %.2f, Voltage2: %.2f", static_cast<double>(_report.current), static_cast<double>(_report.voltage1), static_cast<double>(_report.voltage2));

    // Update battery status with collected values
    _battery_status.timestamp = _report.timestamp;
    _battery_status.voltage_v = 13;                         //Placeholder for "Battery not identified"
    //PX4_INFO("Battery id: %d", battery_id);
    if(battery_id == _bat_id_1.get())
    {
        _battery_status.voltage_v = _report.voltage1;
    }
    if(battery_id == _bat_id_2.get())
    {
        _battery_status.voltage_v = _report.voltage2;
    }                                                       //By this moment both batteries should get data from readings
    _battery_status.current_a = _report.current;
    _battery_status.current_average_a = _report.current;

    // Set per-cell voltage
    setPerCellVoltage(_battery_status.voltage_v);

    // Re-publish the battery status
    orb_publish(ORB_ID(battery_status), _battery_status_topic_pub, &_battery_status);

    // Check conditions for warnings
    if (_report.voltage2 < _min_volt.get()) {
        triggerWarningOnce(battery_status_s::BATTERY_WARNING_CRITICAL, INTERNAL_POWER_LOW, "Internal power failure");
        return PX4_ERROR;
    }

    current_time = hrt_absolute_time();

if (_report.current > _max_amp.get()) {
    if (!has_warning) {
        if (time_var == 0) {
            time_var = current_time; // Start timing
        }
        if (current_time - time_var > static_cast<uint64_t>(_amp_timeout.get()) * 1000) {
            triggerWarning(battery_status_s::BATTERY_WARNING_LOW, CURRENT_TOO_HIGH, "Current too high");
            has_warning = true;
            time_var = 0; // Reset time_var after triggering
        }
    } else {
        time_var = 0; // Reset if already warned
    }
} else if (_report.current < _max_amp.get()) {
    if (has_warning) {
        if (time_var == 0) {
            time_var = current_time; // Start timing
        }
        if (current_time - time_var > static_cast<uint64_t>(_amp_timeout.get()) * 1000) {
            triggerWarning(battery_status_s::BATTERY_WARNING_NONE, CURRENT_TOO_HIGH, "Current returned back to normal");
            has_warning = false;
            time_var = 0; // Reset time_var after clearing the warning
        }
    } else {
        time_var = 0; // Reset if already stable
    }
}


    switch (state) {
        case EXTERNAL_POWER:
            if (_report.voltage1 < _min_volt.get()) {
                triggerWarning(battery_status_s::BATTERY_WARNING_CRITICAL, EXTERNAL_POWER_LOST, "External power turned OFF, switching to reserve power");
                state = RESERVE_POWER;
            }
            break;

        case POST_INITIALIZATION:
            if (_report.voltage1 > _min_volt.get()) {
                triggerWarning(battery_status_s::BATTERY_WARNING_NONE, EXTERNAL_POWER_ON, "External power turned ON");
                state = EXTERNAL_POWER;
            } else {
                triggerWarningOnce(battery_status_s::BATTERY_WARNING_CRITICAL, NO_EXTERNAL_POWER, "External power not turned ON");
            }
            break;

        case RESERVE_POWER:
            break;
    }

    perf_end(_sample_perf);
    return PX4_OK;
}

void CustomBatteryDriver::Run()
{
    collect();
}

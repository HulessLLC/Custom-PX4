// Header file for the I2C float reader driver (CustomBatteryDriver.hpp)
#ifndef I2C_FLOAT_READER_HPP
#define I2C_FLOAT_READER_HPP

#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>

#define I2C_FLOAT_READER_DEFAULT_I2C_ADDRESS 0x1b
#define PX4_I2C_BUS_EXPANSION 1
#define MINIMUM_VOLTAGE static_cast<float>(32.0)
#define CURRENT_WARNING_TRESHOLD static_cast<float>(70.0)

#define INSTANCE_ID_1 1
#define INSTANCE_ID_2 2

enum PowerUnitState {
    POST_INITIALIZATION,
    EXTERNAL_POWER,
    RESERVE_POWER
};

enum WarningID
{
    I2C_CONNECTION_LOST = 0,
    NO_EXTERNAL_POWER = 1,
    EXTERNAL_POWER_ON = 2,
    EXTERNAL_POWER_LOST = 3,
    INTERNAL_POWER_LOW = 4,
    CURRENT_TOO_HIGH = 5,
    URECOGNIZED_WARNING = 6
};

class CustomBatteryDriver : public px4::ScheduledWorkItem, public device::I2C
{
public:
    CustomBatteryDriver(int instance_id, int bus, int address = I2C_FLOAT_READER_DEFAULT_I2C_ADDRESS);
    virtual ~CustomBatteryDriver();

    virtual int init();
    void start();
    void Run() override;

private:
    int collect();
    int sendInitBatteryStatus();
    void setPerCellVoltage(float voltage); // Helper function to set per-cell voltage
    void triggerWarning(uint8_t warning_severity, uint8_t id, const char *cause);
    void triggerWarningOnce(uint8_t warning_severity, uint8_t id, const char *cause);

    struct sensor_float_s {
        float current;
        float voltage1;
        float voltage2;
        uint64_t timestamp;
    } _report;

    bool triggered_warnings[7] {false};
    int _instance_id;
    bool _emergency_triggered = false;
    uint8_t _current_warning_severity = battery_status_s::BATTERY_WARNING_NONE; // To track the current warning level
    bool i2c_comm_state = true;
    PowerUnitState state = POST_INITIALIZATION;
    perf_counter_t _sample_perf;
    orb_advert_t _battery_status_topic_pub;
    battery_status_s _battery_status;
};

#endif // I2C_FLOAT_READER_HPP

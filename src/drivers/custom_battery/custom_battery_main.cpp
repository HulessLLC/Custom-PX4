// Main control file for the I2C float reader driver (custom_battery_driver_main.cpp)
#include "custom_battery.hpp"
#include <px4_platform_common/module.h>

extern "C" __EXPORT int CustomBatteryDriver_main(int argc, char *argv[]);

static CustomBatteryDriver *g_custom_battery_driver_1 = nullptr;
static CustomBatteryDriver *g_custom_battery_driver_2 = nullptr;

int CustomBatteryDriver_main(int argc, char *argv[])
{
    if (argc < 2) {
        PX4_ERR("usage: custom_battery_driver {start|stop|status}");
        return PX4_ERROR;
    }

    if (!strcmp(argv[1], "start")) {
        if (g_custom_battery_driver_1 != nullptr || g_custom_battery_driver_2 != nullptr) {
            PX4_WARN("already running");
            return PX4_ERROR;
        }

        g_custom_battery_driver_1 = new CustomBatteryDriver(INSTANCE_ID_1, PX4_I2C_BUS_EXPANSION);
        if (g_custom_battery_driver_1 == nullptr) {
            PX4_ERR("alloc failed");
            return PX4_ERROR;
        }

        if (g_custom_battery_driver_1->init() != PX4_OK) {
            delete g_custom_battery_driver_1;
            g_custom_battery_driver_1 = nullptr;
            PX4_ERR("driver init failed");
            return PX4_ERROR;
        }

        g_custom_battery_driver_1->start();

        g_custom_battery_driver_2 = new CustomBatteryDriver(INSTANCE_ID_2, PX4_I2C_BUS_EXPANSION);
        if (g_custom_battery_driver_2 == nullptr) {
            PX4_ERR("alloc failed");
            return PX4_ERROR;
        }

        if (g_custom_battery_driver_2->init() != PX4_OK) {
            delete g_custom_battery_driver_2;
            g_custom_battery_driver_2 = nullptr;
            PX4_ERR("driver init failed");
            return PX4_ERROR;
        }

        g_custom_battery_driver_2->start();
        return PX4_OK;
    }

    if (!strcmp(argv[1], "stop")) {
        if (g_custom_battery_driver_1 == nullptr && g_custom_battery_driver_2 == nullptr) {
            PX4_WARN("not running");
            return PX4_ERROR;
        }

        if (g_custom_battery_driver_1 != nullptr) {
            delete g_custom_battery_driver_1;
            g_custom_battery_driver_1 = nullptr;
        }

        if (g_custom_battery_driver_2 != nullptr) {
            delete g_custom_battery_driver_2;
            g_custom_battery_driver_2 = nullptr;
        }
        return PX4_OK;
    }

    if (!strcmp(argv[1], "status")) {
        if (g_custom_battery_driver_1 == nullptr && g_custom_battery_driver_2 == nullptr) {
            PX4_WARN("not running");
            return PX4_ERROR;
        }

        if (g_custom_battery_driver_1 != nullptr) {
            PX4_INFO("battery instance 1 running");
        }

        if (g_custom_battery_driver_2 != nullptr) {
            PX4_INFO("battery instance 2 running");
        }
        return PX4_OK;
    }

    PX4_ERR("unrecognized command");
    return PX4_ERROR;
}

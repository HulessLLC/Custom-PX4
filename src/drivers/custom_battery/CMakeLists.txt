px4_add_module(
    MODULE drivers_custom_battery
    MAIN CustomBatteryDriver
    SRCS
        custom_battery.hpp
        custom_battery.cpp
        custom_battery_main.cpp
    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/platforms/nuttx/NuttX/apps/graphics/pdcurs34/include
)

# PX4 Drone Autopilot (Custom)

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones, and includes custom uavcan parachute and battery monitor drivers.

## Custom drivers
### Custom battery monitor driver
[Driver location](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/custom_battery)

Driver that monitors power unit data in real-time for current, internal voltage and external voltage. Received values will be displayed as two PX4 batteries, one containing external voltage, and other | internal.

#### Warnings conditions:
* Power consumption is too high

#### Critical errors conditions:
* External power not turned on
* Internal power failure
* I2C communication failed

## Parameters:
| Parameter name | Datatype | Default value | Description |
| ------------ | :------: | :-----------: | ----------- |
| CBAT_ENABLE | Boolean | Disabled  | Enables/Disables driver |
| CBAT_ID_1 | Int32 | 7 | ID of the first battery |
| CBAT_ID_2 | Int32 | 8 | ID of the second battery |
| CBAT_I2C_BUS | Int32 | 4 | I2C bus of the power unit |
| CBAT_I2C_ADDR | Int32 | 27 | I2C address of the power unit |
| CBAT_MIN_VOLT | Float | 32.0 | Minimum voltage. If less is read, then driver considers device turned off |
| CBAT_MAX_AMP | Float | 70.0 | Maximum current. If more is read for more than CBAT_MAX_AMP_TIMEOUT, then warning is thrown. |
| CBAT_MAX_AMP_TIMEOUT | Int32 | 1000 | Time in ms, afer which amp warning is thrown |

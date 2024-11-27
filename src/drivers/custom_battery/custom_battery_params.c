#include <parameters/param.h>


/**
 * Custom Battery mode
 *
 *  0 - Custom battery disabled.
 *  1 - Custom battery enabled.
 *
 * @reboot_required true
 * @group Custom battery
 * @boolean
 */
PARAM_DEFINE_INT32(CBAT_ENABLE, 0);

/**
 * CBAT ID 1
 *
 *  Any other value - battery ID for CBAT
 *
 * @min 0
 * @max 14
 * @reboot_required true
 * @group Custom battery
 */
PARAM_DEFINE_INT32(CBAT_ID_1, 7);

/**
 * CBAT ID 2
 *
 *  Any other value - battery ID for CBAT
 *
 * @min 0
 * @max 14
 * @reboot_required true
 * @group Custom battery
 */
PARAM_DEFINE_INT32(CBAT_ID_2, 8);


/**
 * I2C read address
 *
 *  Any other value - I2C address of power supply unit
 *
 * @min -1
 * @max 200
 * @reboot_required true
 * @group Custom battery
 */
PARAM_DEFINE_INT32(CBAT_I2C_ADDR, 27);

/**
 * I2C read bus
 *
 *  Any other value - I2C bus of power supply unit
 *
 * @min -1
 * @max 4
 * @reboot_required true
 * @group Custom battery
 */
PARAM_DEFINE_INT32(CBAT_I2C_BUS, 1);

/**
 * CBAT minimum internal voltage
 *
 *  Any other value - minimum voltage, if lower voltage detected critical error will occur
 *
 * @min -1.0
 * @max 100.0
 * @decimal 1
 * @reboot_required true
 * @group Custom battery
 */
PARAM_DEFINE_FLOAT(CBAT_MIN_VOLT, 32.0f);

/**
 * CBAT maximum power
 *
 *  Any other value - maximum power, if higher power detected you will get warned
 *
 * @min -1.0
 * @max 150.0
 * @decimal 1
 * @reboot_required true
 * @group Custom battery
 */
PARAM_DEFINE_FLOAT(CBAT_MAX_AMP, 70.0f);

/**
 * CBAT max power timeout
 *
 *  Any other value - timeout for max power warning in msec
 *
 * @min -1
 * @max 10000
 * @reboot_required true
 * @group Custom battery
 */
PARAM_DEFINE_INT32(CBAT_AMP_TIMEOUT, 1000);

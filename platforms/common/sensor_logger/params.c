/**
 * Sensor Logging Mode
 *
 * Define mode for sensor logger
 *
 * @value -1 disabled
 * @value 0 the first instance choose mode(tcp/file) (default)
 * @value 1 start with the default tcp socket
 * @value 2 start with the default file file
 *
 * @min -1
 * @max 2
 * @reboot_required true
 * @group Sensor Logging
 */
PARAM_DEFINE_INT32(SL_MODE, 0);

/**
 * Sensor Logging capacity the ring buffer
 *
 * @min 50
 * @max 100
 * @reboot_required true
 * @group Sensor Logging
 */
PARAM_DEFINE_INT32(SL_MAX_MSG, 50);
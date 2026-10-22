/**
 * @file tdc_params.c
 *
 * Parameters for Time-Delay Control (TDC)
 *
 */


/**
 * TDC K1 gain for roll axis
 *
 * @group TDC
 */
PARAM_DEFINE_FLOAT(TDC_K1_R, 3f);

/**
 * TDC K1 gain for pitch axis
 *
 * @group TDC
 */
PARAM_DEFINE_FLOAT(TDC_K1_P, 3f);

/**
 * TDC K1 gain for yaw axis
 *
 * @group TDC
 */
PARAM_DEFINE_FLOAT(TDC_K1_Y, 3f);

/**
 * TDC K2 gain for roll axis
 *
 * @group TDC
 */
PARAM_DEFINE_FLOAT(TDC_K2_R, 5f);

/**
 * TDC K2 gain for pitch axis
 *
 * @group TDC
 */
PARAM_DEFINE_FLOAT(TDC_K2_P, 5f);

/**
 * TDC K2 gain for yaw axis
 *
 * @group TDC
 */
PARAM_DEFINE_FLOAT(TDC_K2_Y, 5f);

/**
 * TDC control effectiveness for roll axis
 *
 * @group TDC
 */
PARAM_DEFINE_FLOAT(TDC_GBAR_R, 400.0f);

/**
 * TDC control effectiveness for pitch axis
 *
 * @group TDC
 */
PARAM_DEFINE_FLOAT(TDC_GBAR_P, 400.0f);

/**
 * TDC control effectiveness for yaw axis
 *
 * @group TDC
 */
PARAM_DEFINE_FLOAT(TDC_GBAR_Y, 400.0f);

/**
 * TDC output limit
 *
 * @group TDC
 */
PARAM_DEFINE_FLOAT(TDC_LIMIT, 0.8f);

/**
 * Attitude control mode
 *
 * @value 0 PID (default)
 * @value 1 Time Delay Control (TDC)
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(TDC_RT_MODE, 1);

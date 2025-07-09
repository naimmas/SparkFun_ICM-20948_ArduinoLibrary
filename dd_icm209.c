#include "dd_icm209.h"
#include "util/helpers/icm209_helpers.h"
#include "su_common.h"
#include "util/invensense/ICM_20948_REGISTERS.h"
#include "util/invensense/AK09916_REGISTERS.h"
#include "ha_iic/ha_iic.h"
#include "ha_timer/ha_timer.h"

#include "ps_logger/ps_logger.h"

#define MAX_MAGNETOMETER_STARTS (10U)
#define DEFAULT_IIC_TIMEOUT (100U)

IIC_SETUP_PORT_CONNECTION(1, IIC_DEFINE_CONNECTION(IIC_PORT1, 0, 0x68))
// Debug Printing: based on gfvalvo's flash string helper code:
// https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809

typedef struct st_driver
{
  icm209_dev_t user_dev;
  ICM_20948_Device_t internal_dev;
  ICM_20948_Serif_t serif_hndlr;
  ICM_20948_AGMT_t agmt;
} driver_t;

static driver_t g_driver = { 0x00 };

/**
 * @brief This internal function writes data to a specific register of the
 * QMI8658 device over I2C.
 * @param[in,out] ppt_dev QMI8658 device instance.
 * @param[in] ppt_data Pointer to the data buffer to write.
 * @param[in] p_data_sz Size of the data to write in bytes.
 * @param[in] p_reg_addr The register address to write to in the sensor.
 * @return Result of the execution status.
 */
static ICM_20948_Status_e write_register(uint8_t p_reg_addr, uint8_t *ppt_data, uint32_t p_data_sz, void *user)
{
    response_status_t api_ret_val        = RET_OK;
    (void)user;
    api_ret_val =
      ha_iic_master_mem_write(IIC_GET_DEV_PORT(0),
                              IIC_GET_DEV_ADDRESS(0),
                              ppt_data,
                              p_data_sz,
                              p_reg_addr,
                              HW_IIC_MEM_SZ_8BIT,
                              DEFAULT_IIC_TIMEOUT);

    return api_ret_val == RET_OK ? ICM_20948_Stat_Ok : ICM_20948_Stat_Err;
}

/**
 * @brief This internal function reads data from a specific register of the
 * QMI8658 device over I2C.
 * @param[in,out] ppt_dev QMI8658 device instance.
 * @param[out] ppt_data Pointer to the data buffer to read into.
 * @param[in] p_data_sz Size of the data to read in bytes.
 * @param[in] p_reg_addr The register address to read from in the sensor.
 * @return Result of the execution status.
 */
static ICM_20948_Status_e read_register(uint8_t p_reg_addr, uint8_t *ppt_data, uint32_t p_data_sz, void *user)
{
    response_status_t api_ret_val        = RET_OK;
    (void)user;
    api_ret_val =
      ha_iic_master_mem_read(IIC_GET_DEV_PORT(0),
                             IIC_GET_DEV_ADDRESS(0),
                             ppt_data,
                             p_data_sz,
                             p_reg_addr,
                             HW_IIC_MEM_SZ_8BIT,
                             DEFAULT_IIC_TIMEOUT);

    return api_ret_val == RET_OK ? ICM_20948_Stat_Ok : ICM_20948_Stat_Err;
}

// Data Getters
ICM_20948_AGMT_t icm209_getAGMT(void)
{
  ICM_20948_get_agmt(&g_driver.internal_dev, &g_driver.agmt);

  return g_driver.agmt;
}
float icm209_get_temp(void)
{
  return getTempC(g_driver.agmt.tmp.val);
}
float icm209_get_mag(uint8_t axis)
{
  int16_t axis_val[] = {
    g_driver.agmt.mag.axes.x,
    g_driver.agmt.mag.axes.y,
    g_driver.agmt.mag.axes.z
  };
  return getMagUT(axis_val[axis]);
}
float icm209_get_gyro(uint8_t axis)
{
  int16_t axis_val[] = {
    g_driver.agmt.gyr.axes.x,
    g_driver.agmt.gyr.axes.y,
    g_driver.agmt.gyr.axes.z
  };
  return getGyrDPS(&g_driver.agmt, axis_val[axis]);
}
float icm209_get_acc(uint8_t axis)
{
  int16_t axis_val[] = {
    g_driver.agmt.acc.axes.x,
    g_driver.agmt.acc.axes.y,
    g_driver.agmt.acc.axes.z
  };
  return getAccMG(&g_driver.agmt, axis_val[axis]);
}

// Gyro Bias
ICM_20948_Status_e icm209_setBiasGyro(uint8_t axis, int32_t newValue)
{
    const uint32_t gyro_bias[] = {
    GYRO_BIAS_X, GYRO_BIAS_Y, GYRO_BIAS_Z
  };
  if (g_driver.internal_dev._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char gyro_bias_reg[4];
    gyro_bias_reg[0] = (unsigned char)(newValue >> 24);
    gyro_bias_reg[1] = (unsigned char)(newValue >> 16);
    gyro_bias_reg[2] = (unsigned char)(newValue >> 8);
    gyro_bias_reg[3] = (unsigned char)(newValue & 0xff);
    ICM_20948_Status_e retval = inv_icm20948_write_mems(&g_driver.internal_dev, gyro_bias[axis], 4, (const unsigned char*)&gyro_bias_reg);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}
ICM_20948_Status_e icm209_getBiasGyro(uint8_t axis, int32_t* bias)
{
  const uint32_t gyro_bias[] = {
    GYRO_BIAS_X, GYRO_BIAS_Y, GYRO_BIAS_Z
  };
  if (g_driver.internal_dev._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    ICM_20948_Status_e retval = inv_icm20948_read_mems(&g_driver.internal_dev, gyro_bias[axis], 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

// Accel Bias
ICM_20948_Status_e icm209_setBiasAccel(uint8_t axis, int32_t newValue)
{
    const uint32_t acc_bias[] = {
    ACCEL_BIAS_X, ACCEL_BIAS_Y, ACCEL_BIAS_Z
  };
  if (g_driver.internal_dev._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char accel_bias_reg[4];
    accel_bias_reg[0] = (unsigned char)(newValue >> 24);
    accel_bias_reg[1] = (unsigned char)(newValue >> 16);
    accel_bias_reg[2] = (unsigned char)(newValue >> 8);
    accel_bias_reg[3] = (unsigned char)(newValue & 0xff);
    ICM_20948_Status_e retval = inv_icm20948_write_mems(&g_driver.internal_dev, acc_bias[axis], 4, (const unsigned char*)&accel_bias_reg);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}
ICM_20948_Status_e icm209_getBiasAccel(uint8_t axis, int32_t* bias)
{
      const uint32_t acc_bias[] = {
    ACCEL_BIAS_X, ACCEL_BIAS_Y, ACCEL_BIAS_Z
  };
  if (g_driver.internal_dev._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    ICM_20948_Status_e retval = inv_icm20948_read_mems(&g_driver.internal_dev, acc_bias[axis], 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

// CPass Bias
ICM_20948_Status_e icm209_setBiasCPass(uint8_t axis, int32_t newValue)
{
  uint32_t cpass_bias[] = {
    CPASS_BIAS_X, CPASS_BIAS_Y, CPASS_BIAS_Z
  };

  if (g_driver.internal_dev._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char cpass_bias_reg[4];
    cpass_bias_reg[0] = (unsigned char)(newValue >> 24);
    cpass_bias_reg[1] = (unsigned char)(newValue >> 16);
    cpass_bias_reg[2] = (unsigned char)(newValue >> 8);
    cpass_bias_reg[3] = (unsigned char)(newValue & 0xff);
    ICM_20948_Status_e retval = inv_icm20948_write_mems(&g_driver.internal_dev, cpass_bias[axis], 4, (const unsigned char*)&cpass_bias_reg);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}
ICM_20948_Status_e icm209_getBiasCPass(uint8_t axis, int32_t* bias)
{
    uint32_t cpass_bias[] = {
    CPASS_BIAS_X, CPASS_BIAS_Y, CPASS_BIAS_Z
  };
  if (g_driver.internal_dev._dmp_firmware_available == true) // Is DMP supported?
  {
    unsigned char bias_data[4] = { 0 };
    ICM_20948_Status_e retval = inv_icm20948_read_mems(&g_driver.internal_dev, cpass_bias[axis], 4, bias_data);
    union {
      int32_t signed32;
      uint32_t unsigned32;
    } signedUnsigned32;
    signedUnsigned32.unsigned32 = (((uint32_t)bias_data[0]) << 24) | (((uint32_t)bias_data[1]) << 16) | (((uint32_t)bias_data[2]) << 8) | (bias_data[3]);
    *bias = signedUnsigned32.signed32; // Convert from unsigned to signed with no cast ambiguity
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

// Device settings
ICM_20948_Status_e icm209_reset(void)
{
  ICM_20948_Status_e retval = ICM_20948_sw_reset(&g_driver.internal_dev);
  return retval;
}
ICM_20948_Status_e icm209_sleep(bool on)
{
  ICM_20948_Status_e retval = ICM_20948_sleep(&g_driver.internal_dev, on);
  return retval;
}
ICM_20948_Status_e icm209_low_power(bool on)
{
  ICM_20948_Status_e retval = ICM_20948_low_power(&g_driver.internal_dev, on);
  return retval;
}
ICM_20948_Status_e icm209_set_clock_source(ICM_20948_PWR_MGMT_1_CLKSEL_e source)
{
  ICM_20948_Status_e retval = ICM_20948_set_clock_source(&g_driver.internal_dev, source);
  return retval;
}

// Internal Sensor Options
ICM_20948_Status_e icm209_set_sample_mode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode)
{
  ICM_20948_Status_e retval = ICM_20948_set_sample_mode(&g_driver.internal_dev, (ICM_20948_InternalSensorID_bm)sensor_id_bm, (ICM_20948_LP_CONFIG_CYCLE_e)lp_config_cycle_mode);
  ha_timer_hard_delay_ms(1);
  return retval;
}
ICM_20948_Status_e icm209_set_full_scale_range(uint8_t sensor_id_bm, ICM_20948_fss_t fss)
{
  ICM_20948_Status_e retval = ICM_20948_set_full_scale(&g_driver.internal_dev, (ICM_20948_InternalSensorID_bm)sensor_id_bm, fss);
  return retval;
}
ICM_20948_Status_e icm209_setDLPFcfg(uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg)
{
  ICM_20948_Status_e retval = ICM_20948_set_dlpf_cfg(&g_driver.internal_dev, (ICM_20948_InternalSensorID_bm)sensor_id_bm, cfg);
  return retval;
}
ICM_20948_Status_e icm209_enable_low_pass_filter(uint8_t sensor_id_bm, bool enable)
{
  ICM_20948_Status_e retval = ICM_20948_enable_dlpf(&g_driver.internal_dev, (ICM_20948_InternalSensorID_bm)sensor_id_bm, enable);
  return retval;
}
ICM_20948_Status_e icm209_set_sample_rate(uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt)
{
  ICM_20948_Status_e retval = ICM_20948_set_sample_rate(&g_driver.internal_dev, (ICM_20948_InternalSensorID_bm)sensor_id_bm, smplrt);
  return retval;
}

// Interrupt settings
ICM_20948_Status_e icm209_clearInterrupts(void)
{
  ICM_20948_INT_STATUS_t int_stat;
  ICM_20948_INT_STATUS_1_t int_stat_1;

  // read to clear interrupts
  ICM_20948_Status_e retval = ICM_20948_set_bank(&g_driver.internal_dev, 0);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  retval = ICM_20948_execute_r(&g_driver.internal_dev, AGB0_REG_INT_STATUS, (uint8_t *)&int_stat, sizeof(ICM_20948_INT_STATUS_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  retval = ICM_20948_execute_r(&g_driver.internal_dev, AGB0_REG_INT_STATUS_1, (uint8_t *)&int_stat_1, sizeof(ICM_20948_INT_STATUS_1_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  // todo: there may be additional interrupts that need to be cleared, like FIFO overflow/watermark

  return retval;
}
ICM_20948_Status_e icm209_cfgIntActiveLow(bool active_low)
{
  ICM_20948_INT_PIN_CFG_t reg;
  ICM_20948_Status_e retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, NULL, &reg); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  reg.INT1_ACTL = active_low;                           // set the setting
  retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, &reg, NULL); // write phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_cfgIntOpenDrain(bool open_drain)
{
  ICM_20948_INT_PIN_CFG_t reg;
  ICM_20948_Status_e retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, NULL, &reg); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  reg.INT1_OPEN = open_drain;                           // set the setting
  retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, &reg, NULL); // write phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_cfgIntLatch(bool latching)
{
  ICM_20948_INT_PIN_CFG_t reg;
  ICM_20948_Status_e retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, NULL, &reg); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  reg.INT1_LATCH_EN = latching;                         // set the setting
  retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, &reg, NULL); // write phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_cfgIntAnyReadToClear(bool enabled)
{
  ICM_20948_INT_PIN_CFG_t reg;
  ICM_20948_Status_e retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, NULL, &reg); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  reg.INT_ANYRD_2CLEAR = enabled;                       // set the setting
  retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, &reg, NULL); // write phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_cfgFsyncActiveLow(bool active_low)
{
  ICM_20948_INT_PIN_CFG_t reg;
  ICM_20948_Status_e retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, NULL, &reg); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  reg.ACTL_FSYNC = active_low;                          // set the setting
  retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, &reg, NULL); // write phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_cfgFsyncIntMode(bool interrupt_mode)
{
  ICM_20948_INT_PIN_CFG_t reg;
  ICM_20948_Status_e retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, NULL, &reg); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  reg.FSYNC_INT_MODE_EN = interrupt_mode;               // set the setting
  retval = ICM_20948_int_pin_cfg(&g_driver.internal_dev, &reg, NULL); // write phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_intEnableI2C(bool enable)
{
  ICM_20948_INT_enable_t en;                          // storage
  ICM_20948_Status_e retval = ICM_20948_int_enable(&g_driver.internal_dev, NULL, &en); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  en.I2C_MST_INT_EN = enable;                        // change the setting
  retval = ICM_20948_int_enable(&g_driver.internal_dev, &en, &en); // write phase w/ readback
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  if (en.I2C_MST_INT_EN != enable)
  {
    ICM_20948_Status_e retval = ICM_20948_Stat_Err;
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_intEnableDMP(bool enable)
{
  ICM_20948_INT_enable_t en;                          // storage
  ICM_20948_Status_e retval = ICM_20948_int_enable(&g_driver.internal_dev, NULL, &en); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  en.DMP_INT1_EN = enable;                           // change the setting
  retval = ICM_20948_int_enable(&g_driver.internal_dev, &en, &en); // write phase w/ readback
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  if (en.DMP_INT1_EN != enable)
  {
    ICM_20948_Status_e retval = ICM_20948_Stat_Err;
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_intEnablePLL(bool enable)
{
  ICM_20948_INT_enable_t en;                          // storage
  ICM_20948_Status_e retval = ICM_20948_int_enable(&g_driver.internal_dev, NULL, &en); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  en.PLL_RDY_EN = enable;                            // change the setting
  retval = ICM_20948_int_enable(&g_driver.internal_dev, &en, &en); // write phase w/ readback
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  if (en.PLL_RDY_EN != enable)
  {
    ICM_20948_Status_e retval = ICM_20948_Stat_Err;
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_intEnableWOF(bool enable)
{
  ICM_20948_INT_enable_t en;                          // storage
  ICM_20948_Status_e retval = ICM_20948_int_enable(&g_driver.internal_dev, NULL, &en); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  en.REG_WOF_EN = enable;                            // change the setting
  retval = ICM_20948_int_enable(&g_driver.internal_dev, &en, &en); // write phase w/ readback
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  if (en.REG_WOF_EN != enable)
  {
    retval = ICM_20948_Stat_Err;
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_intEnableRawDataReady(bool enable)
{
  ICM_20948_INT_enable_t en;                          // storage
  ICM_20948_Status_e retval = ICM_20948_int_enable(&g_driver.internal_dev, NULL, &en); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  en.RAW_DATA_0_RDY_EN = enable;                     // change the setting
  retval = ICM_20948_int_enable(&g_driver.internal_dev, &en, &en); // write phase w/ readback
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  if (en.RAW_DATA_0_RDY_EN != enable)
  {
    retval = ICM_20948_Stat_Err;
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_intEnableOverflowFIFO(uint8_t bm_enable)
{
  ICM_20948_INT_enable_t en;                          // storage
  ICM_20948_Status_e retval = ICM_20948_int_enable(&g_driver.internal_dev, NULL, &en); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  en.FIFO_OVERFLOW_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
  en.FIFO_OVERFLOW_EN_1 = ((bm_enable >> 1) & 0x01);
  en.FIFO_OVERFLOW_EN_2 = ((bm_enable >> 2) & 0x01);
  en.FIFO_OVERFLOW_EN_3 = ((bm_enable >> 3) & 0x01);
  en.FIFO_OVERFLOW_EN_4 = ((bm_enable >> 4) & 0x01);
  retval = ICM_20948_int_enable(&g_driver.internal_dev, &en, &en); // write phase w/ readback
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}
ICM_20948_Status_e icm209_intEnableWatermarkFIFO(uint8_t bm_enable)
{
  ICM_20948_INT_enable_t en;                          // storage
  ICM_20948_Status_e retval = ICM_20948_int_enable(&g_driver.internal_dev, NULL, &en); // read phase
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  en.FIFO_WM_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
  en.FIFO_WM_EN_1 = ((bm_enable >> 1) & 0x01);
  en.FIFO_WM_EN_2 = ((bm_enable >> 2) & 0x01);
  en.FIFO_WM_EN_3 = ((bm_enable >> 3) & 0x01);
  en.FIFO_WM_EN_4 = ((bm_enable >> 4) & 0x01);
  retval = ICM_20948_int_enable(&g_driver.internal_dev, &en, &en); // write phase w/ readback
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

// FIFO settings
ICM_20948_Status_e icm209_enableFIFO(bool enable)
{
  ICM_20948_Status_e retval = ICM_20948_enable_FIFO(&g_driver.internal_dev, enable);
  return retval;
}
ICM_20948_Status_e icm209_resetFIFO(void)
{
  ICM_20948_Status_e retval = ICM_20948_reset_FIFO(&g_driver.internal_dev);
  return retval;
}
ICM_20948_Status_e icm209_setFIFOmode(bool snapshot)
{
  // Default to Stream (non-Snapshot) mode
  ICM_20948_Status_e retval = ICM_20948_set_FIFO_mode(&g_driver.internal_dev, snapshot);
  return retval;
}
ICM_20948_Status_e icm209_getFIFOcount(uint16_t *count)
{
  ICM_20948_Status_e retval = ICM_20948_get_FIFO_count(&g_driver.internal_dev, count);
  return retval;
}
ICM_20948_Status_e icm209_readFIFO(uint8_t *data, uint8_t len)
{
  ICM_20948_Status_e retval = ICM_20948_read_FIFO(&g_driver.internal_dev, data, len);
  return retval;
}

// DMP
ICM_20948_Status_e icm209_resetDMP(void)
{
  ICM_20948_Status_e retval = ICM_20948_reset_DMP(&g_driver.internal_dev);
  return retval;
}
ICM_20948_Status_e icm209_enableDMPSensor(enum inv_icm20948_sensor sensor, bool enable)
{
  if (g_driver.internal_dev._dmp_firmware_available == true) // Should we attempt to enable the sensor?
  {
    ICM_20948_Status_e retval = inv_icm20948_enable_dmp_sensor(&g_driver.internal_dev, sensor, enable == true ? 1 : 0);
    LOG_DEBUG_P1("icm209_enableDMPSensor:  _enabled_Android_0: %d\n", (int)g_driver.internal_dev._enabled_Android_0);
    LOG_EXTEND_P1("  _enabled_Android_1: %d\n", (int)g_driver.internal_dev._enabled_Android_1);
    LOG_EXTEND_P1("  _dataOutCtl1: %d\n", (int)g_driver.internal_dev._dataOutCtl1);
    LOG_EXTEND_P1("  _dataOutCtl2: %d\n", (int)g_driver.internal_dev._dataOutCtl2);
    LOG_EXTEND_P1("  _dataRdyStatus: %d\n", (int)g_driver.internal_dev._dataRdyStatus);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}
ICM_20948_Status_e icm209_enableDMPSensorInt(enum inv_icm20948_sensor sensor, bool enable)
{
  if (g_driver.internal_dev._dmp_firmware_available == true) // Should we attempt to enable the sensor interrupt?
  {
    ICM_20948_Status_e retval = inv_icm20948_enable_dmp_sensor_int(&g_driver.internal_dev, sensor, enable == true ? 1 : 0);
    LOG_DEBUG_P1("icm209_enableDMPSensorInt:  _enabled_Android_intr_0: %d", (int)g_driver.internal_dev._enabled_Android_intr_0);
    LOG_EXTEND_P1("  _enabled_Android_intr_1: %d\n", (int)g_driver.internal_dev._enabled_Android_intr_1);
    LOG_EXTEND_P1("  _dataIntrCtl: %d\n", (int)g_driver.internal_dev._dataIntrCtl);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}
ICM_20948_Status_e icm209_setDMPODRrate(enum DMP_ODR_Registers odr_reg, int interval)
{
  if (g_driver.internal_dev._dmp_firmware_available == true) // Should we attempt to set the DMP ODR?
  {
    // In order to set an ODR for a given sensor data, write 2-byte value to DMP using key defined above for a particular sensor.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate (225Hz) / ODR ) - 1
    // E.g. For a 25Hz ODR rate, value= (225/25) - 1 = 8.

    ICM_20948_Status_e retval = inv_icm20948_set_dmp_sensor_period(&g_driver.internal_dev, odr_reg, interval);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}
ICM_20948_Status_e icm209_readDMPdataFromFIFO(icm_20948_DMP_data_t *data)
{
  if (g_driver.internal_dev._dmp_firmware_available == true) // Should we attempt to set the data from the FIFO?
  {
    ICM_20948_Status_e retval = inv_icm20948_read_dmp_data(&g_driver.internal_dev, data);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}
ICM_20948_Status_e icm209_enableDMP( bool enable)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  if (g_driver.internal_dev._dmp_firmware_available == true) // Should we attempt to enable the DMP?
  {
    retval = ICM_20948_enable_DMP(&g_driver.internal_dev, enable == true ? 1 : 0);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e icm209_initialize_dmp(void)
{
  // First, let's check if the DMP is available
  if (g_driver.internal_dev._dmp_firmware_available != true)
  {
    LOG_DEBUG("icm209_startupDMP: DMP is not available. Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...\n");
    return ICM_20948_Stat_DMPNotSupported;
  }

  ICM_20948_Status_e  worstResult = ICM_20948_Stat_Ok;

#if defined(ICM_20948_USE_DMP)

  // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
  // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

  ICM_20948_Status_e  result = ICM_20948_Stat_Ok; // Use result and worstResult to show if the configuration was successful

  // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
  // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
  // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
  // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
  //
  // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
  // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
  // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
  // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
  // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
  //
  // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
  // 0: use I2C_SLV0
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
  // 10: we read 10 bytes each cycle
  // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
  result = ICM_20948_i2c_controller_configure_peripheral(&g_driver.internal_dev, 0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true, 0); if (result > worstResult) worstResult = result;
  //
  // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
  // 1: use I2C_SLV1
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_CNTL2: we start writing here (0x31)
  // 1: not sure why, but the write does not happen if this is set to zero
  // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
  // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
  result = ICM_20948_i2c_controller_configure_peripheral(&g_driver.internal_dev, 1, MAG_AK09916_I2C_ADDR, M_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single); if (result > worstResult) worstResult = result;
  
  // Set the I2C Master ODR configuration
  // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
  // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
  //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
  //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
  //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
  // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
  // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
  result = setBank(&g_driver.internal_dev, 3); if (result > worstResult) worstResult = result; // Select Bank 3
  uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
  result = write(&g_driver.internal_dev, AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1); if (result > worstResult) worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register  

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  result = icm209_set_clock_source(ICM_20948_Clock_Auto); if (result > worstResult) worstResult = result; // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  result = setBank(&g_driver.internal_dev, 0); if (result > worstResult) worstResult = result;                               // Select Bank 0
  uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
  result = write(&g_driver.internal_dev, AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); if (result > worstResult) worstResult = result; // Write one byte to the PWR_MGMT_2 register

  // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
  // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
  result = icm209_set_sample_mode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled); if (result > worstResult) worstResult = result;

  // Disable the FIFO
  result = icm209_enableFIFO(false); if (result > worstResult) worstResult = result;

  // Disable the DMP
  result = icm209_enableDMP(false); if (result > worstResult) worstResult = result;

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                         // gpm2
                         // gpm4
                         // gpm8
                         // gpm16
  myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                         // dps250
                         // dps500
                         // dps1000
                         // dps2000
  result = icm209_set_full_scale_range((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS); if (result > worstResult) worstResult = result;

  // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
  // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
  // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
  result = ICM_20948_enable_dlpf(&g_driver.internal_dev, ICM_20948_Internal_Gyr, true); if (result > worstResult) worstResult = result;

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  result = setBank(&g_driver.internal_dev, 0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t zero = 0;
  result = write(&g_driver.internal_dev, AGB0_REG_FIFO_EN_1, &zero, 1); if (result > worstResult) worstResult = result;
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  result = write(&g_driver.internal_dev, AGB0_REG_FIFO_EN_2, &zero, 1); if (result > worstResult) worstResult = result;

  // Turn off data ready interrupt through INT_ENABLE_1
  result = icm209_intEnableRawDataReady(false); if (result > worstResult) worstResult = result;

  // Reset FIFO through FIFO_RST
  result = icm209_resetFIFO(); if (result > worstResult) worstResult = result;

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  //mySmplrt.g = 4; // 225Hz
  //mySmplrt.a = 4; // 225Hz
  //mySmplrt.g = 8; // 112Hz
  //mySmplrt.a = 8; // 112Hz
  result = icm209_set_sample_rate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt); if (result > worstResult) worstResult = result;

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(&g_driver.internal_dev, DMP_START_ADDRESS); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  result = loadDMPFirmware(&g_driver.internal_dev); if (result > worstResult) worstResult = result;

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(&g_driver.internal_dev, DMP_START_ADDRESS); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Set the Hardware Fix Disable register to 0x48
  result = setBank(&g_driver.internal_dev, 0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fix = 0x48;
  result = write(&g_driver.internal_dev, AGB0_REG_HW_FIX_DISABLE, &fix, 1); if (result > worstResult) worstResult = result;

  // Set the Single FIFO Priority Select register to 0xE4
  result = setBank(&g_driver.internal_dev, 0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fifoPrio = 0xE4;
  result = write(&g_driver.internal_dev, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); if (result > worstResult) worstResult = result;

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  result = writeDMPmems(&g_driver.internal_dev, ACC_SCALE, 4, &accScale[0]); if (result > worstResult) worstResult = result; // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  result = writeDMPmems(&g_driver.internal_dev, ACC_SCALE2, 4, &accScale2[0]); if (result > worstResult) worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(&g_driver.internal_dev, CPASS_MTX_00, 4, &mountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, CPASS_MTX_01, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, CPASS_MTX_02, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, CPASS_MTX_10, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, CPASS_MTX_11, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, CPASS_MTX_12, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, CPASS_MTX_20, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, CPASS_MTX_21, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, CPASS_MTX_22, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(&g_driver.internal_dev, B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(&g_driver.internal_dev, B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  result = setGyroSF(&g_driver.internal_dev, 19, 3); if (result > worstResult) worstResult = result; // 19 = 55Hz (see above), 3 = 2000dps (see above)

  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  result = writeDMPmems(&g_driver.internal_dev, GYRO_FULLSCALE, 4, &gyroFullScale[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  result = writeDMPmems(&g_driver.internal_dev, ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  //const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  result = writeDMPmems(&g_driver.internal_dev, ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  //const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  result = writeDMPmems(&g_driver.internal_dev, ACCEL_A_VAR, 4, &accelAVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(&g_driver.internal_dev, ACCEL_CAL_RATE, 2, &accelCalRate[0]); if (result > worstResult) worstResult = result;

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  result = writeDMPmems(&g_driver.internal_dev, CPASS_TIME_BUFFER, 2, &compassRate[0]); if (result > worstResult) worstResult = result;

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

#endif

  return worstResult;
}

ICM_20948_Status_e icm209_init(icm209_dev_t** ppt_dev)
{

  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  retval = ICM_20948_init_struct(&g_driver.internal_dev);

  // Set up the serif
  g_driver.serif_hndlr.write = write_register;
  g_driver.serif_hndlr.read = read_register;
  g_driver.serif_hndlr.user = (void *)&g_driver.user_dev; // refer to yourself in the user field

  // Link the serif
  g_driver.internal_dev._serif = &g_driver.serif_hndlr;

#if defined(ICM_20948_USE_DMP)
  g_driver.internal_dev._dmp_firmware_available = true; // Initialize _dmp_firmware_available
#else
  g_driver.internal_dev._dmp_firmware_available = false; // Initialize _dmp_firmware_available
#endif

  g_driver.internal_dev._firmware_loaded = false; // Initialize _firmware_loaded
  g_driver.internal_dev._last_bank = 255;         // Initialize _last_bank. Make it invalid. It will be set by the first call of ICM_20948_set_bank.
  g_driver.internal_dev._last_mems_bank = 255;    // Initialize _last_mems_bank. Make it invalid. It will be set by the first call of inv_icm20948_write_mems.
  g_driver.internal_dev._gyroSF = 0;              // Use this to record the GyroSF, calculated by inv_icm20948_set_gyro_sf
  g_driver.internal_dev._gyroSFpll = 0;
  g_driver.internal_dev._enabled_Android_0 = 0;      // Keep track of which Android sensors are enabled: 0-31
  g_driver.internal_dev._enabled_Android_1 = 0;      // Keep track of which Android sensors are enabled: 32-
  g_driver.internal_dev._enabled_Android_intr_0 = 0; // Keep track of which Android sensor interrupts are enabled: 0-31
  g_driver.internal_dev._enabled_Android_intr_1 = 0; // Keep track of which Android sensor interrupts are enabled: 32-

  // Perform default startup
  // Do a minimal startupDefault if using the DMP. User can always call startupDefault(false) manually if required.
  retval = startupDefault(&g_driver.internal_dev, g_driver.internal_dev._dmp_firmware_available);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("ICM_20948_I2C::begin: startupDefault returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_EXTEND("\n");
  }
  else
  {
    *ppt_dev = &(g_driver.user_dev);
  }
  return retval;
}

#ifndef DD_ICM209_H
#define DD_ICM209_H

#include "util/invensense/AK09916_REGISTERS.h"
#include "util/invensense/ICM_20948_C.h" // The C backbone. ICM_20948_USE_DMP is defined in here.

typedef struct
{
    icm_20948_DMP_data_t dmp_data;

} icm209_dev_t;
ICM_20948_AGMT_t   icm209_getAGMT(void);
float              icm209_get_temp(void);
float              icm209_get_mag(uint8_t axis);
float              icm209_get_gyro(uint8_t axis);
float              icm209_get_acc(uint8_t axis);
ICM_20948_Status_e icm209_setBiasGyro(uint8_t axis, int32_t newValue);
ICM_20948_Status_e icm209_getBiasGyro(uint8_t axis, int32_t* bias);
ICM_20948_Status_e icm209_setBiasAccel(uint8_t axis, int32_t newValue);
ICM_20948_Status_e icm209_getBiasAccel(uint8_t axis, int32_t* bias);
ICM_20948_Status_e icm209_setBiasCPass(uint8_t axis, int32_t newValue);
ICM_20948_Status_e icm209_getBiasCPass(uint8_t axis, int32_t* bias);
ICM_20948_Status_e icm209_reset(void);
ICM_20948_Status_e icm209_sleep(bool on);
ICM_20948_Status_e icm209_low_power(bool on);
ICM_20948_Status_e icm209_set_clock_source(
  ICM_20948_PWR_MGMT_1_CLKSEL_e source);
ICM_20948_Status_e icm209_set_sample_mode(uint8_t sensor_id_bm,
                                          uint8_t lp_config_cycle_mode);
ICM_20948_Status_e icm209_set_full_scale_range(uint8_t         sensor_id_bm,
                                               ICM_20948_fss_t fss);
ICM_20948_Status_e icm209_setDLPFcfg(uint8_t            sensor_id_bm,
                                     ICM_20948_dlpcfg_t cfg);
ICM_20948_Status_e icm209_enable_low_pass_filter(uint8_t sensor_id_bm,
                                                 bool    enable);
ICM_20948_Status_e icm209_set_sample_rate(uint8_t            sensor_id_bm,
                                          ICM_20948_smplrt_t smplrt);
ICM_20948_Status_e icm209_clearInterrupts(void);
ICM_20948_Status_e icm209_cfgIntActiveLow(bool active_low);
ICM_20948_Status_e icm209_cfgIntOpenDrain(bool open_drain);
ICM_20948_Status_e icm209_cfgIntLatch(bool latching);
ICM_20948_Status_e icm209_cfgIntAnyReadToClear(bool enabled);
ICM_20948_Status_e icm209_cfgFsyncActiveLow(bool active_low);
ICM_20948_Status_e icm209_cfgFsyncIntMode(bool interrupt_mode);
ICM_20948_Status_e icm209_intEnableI2C(bool enable);
ICM_20948_Status_e icm209_intEnableDMP(bool enable);
ICM_20948_Status_e icm209_intEnablePLL(bool enable);
ICM_20948_Status_e icm209_intEnableWOF(bool enable);
ICM_20948_Status_e icm209_intEnableRawDataReady(bool enable);
ICM_20948_Status_e icm209_intEnableOverflowFIFO(uint8_t bm_enable);
ICM_20948_Status_e icm209_intEnableWatermarkFIFO(uint8_t bm_enable);
ICM_20948_Status_e icm209_enableFIFO(bool enable);
ICM_20948_Status_e icm209_resetFIFO(void);
ICM_20948_Status_e icm209_setFIFOmode(bool snapshot);
ICM_20948_Status_e icm209_getFIFOcount(uint16_t* count);
ICM_20948_Status_e icm209_readFIFO(uint8_t* data, uint8_t len);
ICM_20948_Status_e icm209_resetDMP(void);
ICM_20948_Status_e icm209_enableDMPSensor(enum inv_icm20948_sensor sensor,
                                          bool                     enable);
ICM_20948_Status_e icm209_enableDMPSensorInt(enum inv_icm20948_sensor sensor,
                                             bool                     enable);
ICM_20948_Status_e icm209_setDMPODRrate(enum DMP_ODR_Registers odr_reg,
                                        int                    interval);
ICM_20948_Status_e icm209_readDMPdataFromFIFO(icm_20948_DMP_data_t* data);
ICM_20948_Status_e icm209_initialize_dmp(void);
ICM_20948_Status_e icm209_enableDMP(bool enable);
ICM_20948_Status_e icm209_init(icm209_dev_t** ppt_dev);
#endif /* DD_ICM209_H */

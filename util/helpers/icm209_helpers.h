#ifndef ICM209_HELPERS_H
#define ICM209_HELPERS_H

#include "../invensense/ICM_20948_C.h" // The C backbone. ICM_20948_USE_DMP is defined in here.
#include "../invensense/AK09916_REGISTERS.h"

float getMagUT(int16_t axis_val);
float getAccMG(ICM_20948_AGMT_t* agmt, int16_t axis_val);
float getGyrDPS(ICM_20948_AGMT_t* agmt, int16_t axis_val);
float getTempC(int16_t val);
const char *get_dev_status_str(ICM_20948_Status_e stat);
ICM_20948_Status_e setBank(ICM_20948_Device_t* _device, uint8_t bank);
ICM_20948_Status_e check_id(ICM_20948_Device_t* _device);
bool is_data_ready(ICM_20948_Device_t* _device);
uint8_t getWhoAmI(ICM_20948_Device_t* _device);
bool is_connected(ICM_20948_Device_t* _device);
ICM_20948_Status_e i2cMasterPassthrough(ICM_20948_Device_t* _device, bool passthrough);
ICM_20948_Status_e i2cMasterEnable(ICM_20948_Device_t* _device, bool enable);
ICM_20948_Status_e i2cMasterReset(ICM_20948_Device_t* _device);
ICM_20948_Status_e i2cMasterConfigureSlave(ICM_20948_Device_t* _device, uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap);
ICM_20948_Status_e i2cMasterSLV4Transaction(ICM_20948_Device_t* _device, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);
ICM_20948_Status_e i2cMasterSingleW(ICM_20948_Device_t* _device, uint8_t addr, uint8_t reg, uint8_t data);
uint8_t i2cMasterSingleR(ICM_20948_Device_t* _device, uint8_t addr, uint8_t reg);
ICM_20948_Status_e startupDefault(ICM_20948_Device_t* _device, bool minimal);
ICM_20948_Status_e read(ICM_20948_Device_t* _device, uint8_t reg, uint8_t *pdata, uint32_t len);
ICM_20948_Status_e write(ICM_20948_Device_t* _device, uint8_t reg, uint8_t *pdata, uint32_t len);
uint8_t readMag(ICM_20948_Device_t* _device, AK09916_Reg_Addr_e reg);
ICM_20948_Status_e writeMag(ICM_20948_Device_t* _device, AK09916_Reg_Addr_e reg, uint8_t *pdata);
ICM_20948_Status_e resetMag(ICM_20948_Device_t* _device);
ICM_20948_Status_e loadDMPFirmware(ICM_20948_Device_t* _device);
ICM_20948_Status_e setDMPstartAddress(ICM_20948_Device_t* _device, unsigned short address);
ICM_20948_Status_e writeDMPmems(ICM_20948_Device_t* _device, unsigned short reg, unsigned int length, const unsigned char *data);
ICM_20948_Status_e readDMPmems(ICM_20948_Device_t* _device, unsigned short reg, unsigned int length, unsigned char *data);
ICM_20948_Status_e startupMagnetometer(ICM_20948_Device_t* _device, bool minimal);
ICM_20948_Status_e magWhoIAm(ICM_20948_Device_t* _device);
ICM_20948_Status_e setGyroSF(ICM_20948_Device_t* _device, unsigned char div, int gyro_level);

#endif // ICM209_HELPERS_H

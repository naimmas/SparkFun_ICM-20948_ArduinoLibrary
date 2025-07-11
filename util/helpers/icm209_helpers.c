#include "icm209_helpers.h"
#include "ps_logger/ps_logger.h"
#include "ps_timer/ps_timer.h"

#define MAX_MAGNETOMETER_STARTS (10U)

float getMagUT(int16_t axis_val)
{
  return (((float)axis_val) * 0.15);
}

float getAccMG(ICM_20948_AGMT_t* agmt, int16_t axis_val)
{
  switch (agmt->fss.a)
  {
  case 0:
    return (((float)axis_val) / 16.384);
    break;
  case 1:
    return (((float)axis_val) / 8.192);
    break;
  case 2:
    return (((float)axis_val) / 4.096);
    break;
  case 3:
    return (((float)axis_val) / 2.048);
    break;
  default:
    return 0;
    break;
  }
}

float getGyrDPS(ICM_20948_AGMT_t* agmt, int16_t axis_val)
{
  switch (agmt->fss.g)
  {
  case 0:
    return (((float)axis_val) / 131);
    break;
  case 1:
    return (((float)axis_val) / 65.5);
    break;
  case 2:
    return (((float)axis_val) / 32.8);
    break;
  case 3:
    return (((float)axis_val) / 16.4);
    break;
  default:
    return 0;
    break;
  }
}

float getTempC(int16_t val)
{
  return (((float)val - 21) / 333.87) + 21;
}

const char *get_dev_status_str(ICM_20948_Status_e stat)
{
  switch (stat)
  {
  case ICM_20948_Stat_Ok:
    return "All is well.";
    break;
  case ICM_20948_Stat_Err:
    return "General Error";
    break;
  case ICM_20948_Stat_NotImpl:
    return "Not Implemented";
    break;
  case ICM_20948_Stat_ParamErr:
    return "Parameter Error";
    break;
  case ICM_20948_Stat_WrongID:
    return "Wrong ID";
    break;
  case ICM_20948_Stat_InvalSensor:
    return "Invalid Sensor";
    break;
  case ICM_20948_Stat_NoData:
    return "Data Underflow";
    break;
  case ICM_20948_Stat_SensorNotSupported:
    return "Sensor Not Supported";
    break;
  case ICM_20948_Stat_DMPNotSupported:
    return "DMP Firmware Not Supported. Is #define ICM_20948_USE_DMP commented in util/ICM_20948_C.h?";
    break;
  case ICM_20948_Stat_DMPVerifyFail:
    return "DMP Firmware Verification Failed";
    break;
  case ICM_20948_Stat_FIFONoDataAvail:
    return "No FIFO Data Available";
    break;
  case ICM_20948_Stat_FIFOIncompleteData:
    return "DMP data in FIFO was incomplete";
    break;
  case ICM_20948_Stat_FIFOMoreDataAvail:
    return "More FIFO Data Available";
    break;
  case ICM_20948_Stat_UnrecognisedDMPHeader:
    return "Unrecognised DMP Header";
    break;
  case ICM_20948_Stat_UnrecognisedDMPHeader2:
    return "Unrecognised DMP Header2";
    break;
  case ICM_20948_Stat_InvalDMPRegister:
    return "Invalid DMP Register";
    break;
  default:
    return "Unknown Status";
    break;
  }
  return "None";
}

// Device Level
ICM_20948_Status_e setBank(ICM_20948_Device_t* _device, uint8_t bank)
{
  ICM_20948_Status_e retval = ICM_20948_set_bank(_device, bank);
  return retval;
}

ICM_20948_Status_e check_id(ICM_20948_Device_t* _device)
{
  ICM_20948_Status_e retval = ICM_20948_check_id(_device);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_checkID: ICM_20948_check_id returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
  }
  return retval;
}

uint8_t getWhoAmI(ICM_20948_Device_t* _device)
{
  uint8_t retval = 0x00;
  ICM_20948_Status_e status = ICM_20948_get_who_am_i(_device, &retval);
  (void)status;
  return retval;
}

bool is_connected(ICM_20948_Device_t* _device)
{
  ICM_20948_Status_e status = check_id(_device);
  if (status == ICM_20948_Stat_Ok)
  {
    return true;
  }
  LOG_DEBUG("icm209_isConnected: checkID returned: ");
  LOG_EXTEND(get_dev_status_str(status));
  LOG_DEBUG("\n");
  return false;
}

// Interface Options
ICM_20948_Status_e i2cMasterPassthrough(ICM_20948_Device_t* _device, bool passthrough)
{
  ICM_20948_Status_e retval = ICM_20948_i2c_master_passthrough(_device, passthrough);
  return retval;
}

ICM_20948_Status_e i2cMasterEnable(ICM_20948_Device_t* _device, bool enable)
{
  ICM_20948_Status_e retval = ICM_20948_i2c_master_enable(_device, enable);
  return retval;
}

ICM_20948_Status_e i2cMasterReset(ICM_20948_Device_t* _device)
{
  ICM_20948_Status_e retval = ICM_20948_i2c_master_reset(_device);
  return retval;
}

ICM_20948_Status_e i2cMasterConfigureSlave(ICM_20948_Device_t* _device, uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap)
{
  return (ICM_20948_i2c_controller_configure_peripheral(_device, peripheral, addr, reg, len, Rw, enable, data_only, grp, swap, 0));
}

ICM_20948_Status_e i2cMasterSLV4Transaction(ICM_20948_Device_t* _device, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
  return ICM_20948_i2c_controller_periph4_txn(_device, addr, reg, data, len, Rw, send_reg_addr);
}

ICM_20948_Status_e i2cMasterSingleW(ICM_20948_Device_t* _device, uint8_t addr, uint8_t reg, uint8_t data)
{
  ICM_20948_Status_e retval = ICM_20948_i2c_master_single_w(_device, addr, reg, &data);
  return retval;
}

uint8_t i2cMasterSingleR(ICM_20948_Device_t* _device, uint8_t addr, uint8_t reg)
{
  uint8_t data = 0;
  ICM_20948_Status_e status = ICM_20948_i2c_master_single_r(_device, addr, reg, &data);
  if (status != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_i2cMasterSingleR: ICM_20948_i2c_master_single_r returned: ");
    LOG_EXTEND(get_dev_status_str(status));
    LOG_DEBUG("\n");
  }
  return data;
}

ICM_20948_Status_e startupDefault(ICM_20948_Device_t* _device, bool minimal)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  retval = check_id(_device);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupDefault: checkID returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }

  retval = ICM_20948_sw_reset(_device);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupDefault: swReset returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }
  ps_hard_delay_ms(50);

  retval = ICM_20948_sleep(_device, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupDefault: sleep returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }

  retval = ICM_20948_low_power(_device, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupDefault: lowPower returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }

  retval = startupMagnetometer(_device, minimal); // Pass the minimal startup flag to startupMagnetometer
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupDefault: startupMagnetometer returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }

  if (minimal) // Return now if minimal is true
  {
    LOG_DEBUG("icm209_startupDefault: minimal startup complete!\n");
    return retval;
  }

  retval = ICM_20948_set_sample_mode(_device, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled
  ps_hard_delay_ms(1); 
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupDefault: setSampleMode returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  } // sensors: 	ICM_20948_Internal_Acc, ICM_20948_Internal_Gyr, ICM_20948_Internal_Mst

  ICM_20948_fss_t FSS;
  FSS.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  FSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  retval = ICM_20948_set_full_scale(_device, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupDefault: setFullScale returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }

  ICM_20948_dlpcfg_t dlpcfg;
  dlpcfg.a = acc_d473bw_n499bw;
  dlpcfg.g = gyr_d361bw4_n376bw5;
  retval = ICM_20948_set_dlpf_cfg(_device, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupDefault: setDLPFcfg returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }

  retval = ICM_20948_enable_dlpf(_device, ICM_20948_Internal_Acc, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupDefault: enableDLPF (Acc) returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }

  retval = ICM_20948_enable_dlpf(_device, ICM_20948_Internal_Gyr, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupDefault: enableDLPF (Gyr) returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }

  return retval;
}

// direct read/write
ICM_20948_Status_e read(ICM_20948_Device_t* _device, uint8_t reg, uint8_t *pdata, uint32_t len)
{
  ICM_20948_Status_e retval = ICM_20948_execute_r(_device, reg, pdata, len);
  return retval;
}

ICM_20948_Status_e write(ICM_20948_Device_t* _device, uint8_t reg, uint8_t *pdata, uint32_t len)
{
  ICM_20948_Status_e retval = ICM_20948_execute_w(_device, reg, pdata, len);
  return retval;
}

uint8_t readMag(ICM_20948_Device_t* _device, AK09916_Reg_Addr_e reg)
{
  uint8_t data = i2cMasterSingleR(_device, MAG_AK09916_I2C_ADDR, reg); // i2cMasterSingleR updates status too
  return data;
}

ICM_20948_Status_e writeMag(ICM_20948_Device_t* _device, AK09916_Reg_Addr_e reg, uint8_t *pdata)
{
  ICM_20948_Status_e retval = i2cMasterSingleW(_device, MAG_AK09916_I2C_ADDR, reg, *pdata);
  return retval;
}

ICM_20948_Status_e resetMag(ICM_20948_Device_t* _device)
{
  uint8_t SRST = 1;
  // SRST: Soft reset
  // “0”: Normal
  // “1”: Reset
  // When “1” is set, all registers are initialized. After reset, SRST bit turns to “0” automatically.
  ICM_20948_Status_e retval = i2cMasterSingleW(_device, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, SRST);
  return retval;
}

ICM_20948_Status_e loadDMPFirmware(ICM_20948_Device_t* _device)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  if (_device->_dmp_firmware_available == true) // Should we attempt to load the DMP firmware?
  {
    retval = ICM_20948_firmware_load(_device);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e setDMPstartAddress(ICM_20948_Device_t* _device, unsigned short address)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  if (_device->_dmp_firmware_available == true) // Should we attempt to set the start address?
  {
    retval = ICM_20948_set_dmp_start_address(_device, address);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e writeDMPmems(ICM_20948_Device_t* _device, unsigned short reg, unsigned int length, const unsigned char *data)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  if (_device->_dmp_firmware_available == true) // Should we attempt to write to the DMP?
  {
    retval = inv_icm20948_write_mems(_device, reg, length, data);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e readDMPmems(ICM_20948_Device_t* _device, unsigned short reg, unsigned int length, unsigned char *data)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  if (_device->_dmp_firmware_available == true) // Should we attempt to read from the DMP?
  {
    retval = inv_icm20948_read_mems(_device, reg, length, data);
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

ICM_20948_Status_e startupMagnetometer(ICM_20948_Device_t* _device, bool minimal)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  ICM_20948_Status_e status = ICM_20948_Stat_Ok;

  i2cMasterPassthrough(_device, false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
  i2cMasterEnable(_device, true);

  resetMag(_device);

  //After a ICM reset the Mag sensor may stop responding over the I2C master
  //Reset the Master I2C until it responds
  uint8_t tries = 0;
  while (tries < MAX_MAGNETOMETER_STARTS)
  {
    tries++;

    //See if we can read the WhoIAm register correctly
    retval = magWhoIAm(_device);
    if (retval == ICM_20948_Stat_Ok)
      break; //WIA matched!

    i2cMasterReset(_device); //Otherwise, reset the master I2C and try again

    ps_hard_delay_ms(10);
  }

  if (tries == MAX_MAGNETOMETER_STARTS)
  {
    LOG_DEBUG("icm209_startupMagnetometer: reached MAX_MAGNETOMETER_STARTS (");
    LOG_EXTEND_P1("%d", MAX_MAGNETOMETER_STARTS);
    LOG_EXTEND("). Returning ICM_20948_Stat_WrongID\n");
    status = ICM_20948_Stat_WrongID;
    return status;
  }
  else
  {
    LOG_DEBUG("icm209_startupMagnetometer: successful magWhoIAm after ");
    LOG_EXTEND_P1("%d", (int)tries);
    if (tries == 1)
      LOG_EXTEND(" try\n");
    else
      LOG_EXTEND(" tries\n");
  }

  //Return now if minimal is true. The mag will be configured manually for the DMP
  if (minimal) // Return now if minimal is true
  {
    LOG_DEBUG("icm209_startupMagnetometer: minimal startup complete!\n");
    return retval;
  }

  //Set up magnetometer
  AK09916_CNTL2_Reg_t reg;
  reg.MODE = AK09916_mode_cont_100hz;
  reg.reserved_0 = 0; // Make sure the unused bits are clear. Probably redundant, but prevents confusion when looking at the I2C traffic
  retval = writeMag(_device, AK09916_REG_CNTL2, (uint8_t *)&reg);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupMagnetometer: writeMag returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }

  retval = ICM_20948_i2c_controller_configure_peripheral(_device, 0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false, 0);
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_startupMagnetometer: i2cMasterConfigurePeripheral returned: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }

  return retval;
}

ICM_20948_Status_e magWhoIAm(ICM_20948_Device_t* _device)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  uint8_t whoiam1, whoiam2;
  whoiam1 = readMag(_device, AK09916_REG_WIA1);
  // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
  // i2cMasterSingleR updates status so it is OK to set retval to status here
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_magWhoIAm: whoiam1: ");
    LOG_EXTEND_P1("%d", (int)whoiam1);
    LOG_EXTEND(" (should be 72) readMag set status to: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }
  whoiam2 = readMag(_device, AK09916_REG_WIA2);
  // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
  // i2cMasterSingleR updates status so it is OK to set retval to status here
  if (retval != ICM_20948_Stat_Ok)
  {
    LOG_DEBUG("icm209_magWhoIAm: whoiam1: ");
    LOG_EXTEND_P1("%d", (int)whoiam1);
    LOG_EXTEND(" (should be 72) whoiam2: ");
    LOG_EXTEND_P1("%d", (int)whoiam2);
    LOG_EXTEND(" (should be 9) readMag set status to: ");
    LOG_EXTEND(get_dev_status_str(retval));
    LOG_DEBUG("\n");
    return retval;
  }

  if ((whoiam1 == (MAG_AK09916_WHO_AM_I >> 8)) && (whoiam2 == (MAG_AK09916_WHO_AM_I & 0xFF)))
  {
    retval = ICM_20948_Stat_Ok;
    return retval;
  }

  LOG_DEBUG("icm209_magWhoIAm: whoiam1: ");
  LOG_EXTEND_P1("%d", (int)whoiam1);
  LOG_EXTEND(" (should be 72) whoiam2: ");
   LOG_EXTEND_P1("%d", (int)whoiam2);
  LOG_EXTEND(" (should be 9). Returning ICM_20948_Stat_WrongID\n");

  retval = ICM_20948_Stat_WrongID;
  return retval;
}

ICM_20948_Status_e setGyroSF(ICM_20948_Device_t* _device, unsigned char div, int gyro_level)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
  if (_device->_dmp_firmware_available == true) // Should we attempt to set the Gyro SF?
  {
    retval = inv_icm20948_set_gyro_sf(_device, div, gyro_level);
    LOG_DEBUG("icm209_setGyroSF:  pll: ");
    LOG_EXTEND_P1("%d", (int)_device->_gyroSFpll);
    LOG_EXTEND("  Gyro SF is: ");
    LOG_EXTEND_P1("%d", (int)_device->_gyroSF);
    LOG_EXTEND("\n");
    return retval;
  }
  return ICM_20948_Stat_DMPNotSupported;
}

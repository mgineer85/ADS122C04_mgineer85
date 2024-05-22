/*

*/
#include <limits.h>
#include "Ads122C04lib.h"

// Constructor
ADS122C04::ADS122C04() {}

// Sets up the ADS122C04 for basic function
// Returns true upon completion
bool ADS122C04::begin(uint8_t deviceAddress, int DRDY_pin, TwoWire &wirePort)
{
  _deviceAddress = deviceAddress; // if other than default address can be set here
  _DRDY = DRDY_pin;               // if <> -1 use _DRDY instead polling ready register.
  _i2cPort = &wirePort;

  if (_DRDY != -1)
  {
    // TODO: DRDY pin use not implemented!
    return false;

    // pinMode(_DRDY, INPUT_PULLUP);
  }

  // wait to ensure device is ready to communicate for sure, dev needs 500µs after power on
  delay(1);

  bool result = true; // Accumulate a result as we do the setup

  result &= cmdReset();    // Reset all registers
  result &= isConnected(); // check connection

  return result;
}

// Returns true if device is present
bool ADS122C04::isConnected()
{
  _i2cPort->beginTransmission(_deviceAddress);
  if (_i2cPort->endTransmission() != 0)
  {
    // log_e("device not detected, no ACK");
    return false; // Sensor did not ACK
  }
  else
  {
    // log_d("device detected, ACK");
    return true; // All good
  }
}

//
// ************** Register Functions
//
bool ADS122C04::setPgaBypass(ADS122C04_PGA_BYPASS_Values pga_bypass)
{
  setInternalCalibrationOffset(0);

  return setBit(ADS122C04_CTRL0_PGA_BYPASS, ADS122C04_CTRL0, pga_bypass);
}

bool ADS122C04::setGain(ADS122C04_GAIN_Values gain)
{
  // invalidate internal offset calibration because different for other gain setting
  setInternalCalibrationOffset(0);

  uint8_t value = readRegister(ADS122C04_CTRL0);
  value &= 0b11110001;                   // Clear gain bits
  value |= gain << ADS122C04_CTRL0_GAIN; // Mask in new bits
  return (writeRegister(ADS122C04_CTRL0, value));
}

ADS122C04_GAIN_Values ADS122C04::getGain()
{
  uint8_t value = readRegister(ADS122C04_CTRL0);
  value &= 0b00001110;                   // Clear gain bits
  value = value >> ADS122C04_CTRL0_GAIN; // Mask in new bits
  return (ADS122C04_GAIN_Values)value;
}

bool ADS122C04::setMux(ADS122C04_MUX_Values mux)
{
  uint8_t value = readRegister(ADS122C04_CTRL0);
  value &= 0b00001111;                 // Clear gain bits
  value |= mux << ADS122C04_CTRL0_MUX; // Mask in new bits
  return (writeRegister(ADS122C04_CTRL0, value));
}

ADS122C04_MUX_Values ADS122C04::getMux()
{
  uint8_t value = readRegister(ADS122C04_CTRL0);
  value &= 0b11110000;                  // Clear gain bits
  value = value >> ADS122C04_CTRL0_MUX; // Mask in new bits
  return (ADS122C04_MUX_Values)value;
}

bool ADS122C04::setTemperatureSensorMode(ADS122C04_TS_Values ts)
{
  return setBit(ADS122C04_CTRL1_TS, ADS122C04_CTRL1, ts);
}
bool ADS122C04::setVoltageReference(ADS122C04_VREF_Values vref)
{
  uint8_t value = readRegister(ADS122C04_CTRL1);
  value &= 0b11111001;                   // Clear gain bits
  value |= vref << ADS122C04_CTRL1_VREF; // Mask in new bits
  return (writeRegister(ADS122C04_CTRL1, value));
}
bool ADS122C04::setConversionMode(ADS122C04_CM_Values cm)
{
  return setBit(ADS122C04_CTRL1_CM, ADS122C04_CTRL1, cm);
}
bool ADS122C04::setOperatingMode(ADS122C04_MODE_Values mode)
{
  return setBit(ADS122C04_CTRL1_MODE, ADS122C04_CTRL1, mode);
}
ADS122C04_MODE_Values ADS122C04::getOperatingMode()
{
  return (ADS122C04_MODE_Values)getBit(ADS122C04_CTRL1_MODE, ADS122C04_CTRL1);
}
bool ADS122C04::setDataRate(ADS122C04_DR_Values dr)
{
  uint8_t value = readRegister(ADS122C04_CTRL1);
  value &= 0b00011111;               // Clear gain bits
  value |= dr << ADS122C04_CTRL1_DR; // Mask in new bits
  return (writeRegister(ADS122C04_CTRL1, value));
}
ADS122C04_DR_Values ADS122C04::getDataRate()
{
  uint8_t value = readRegister(ADS122C04_CTRL1);
  value &= 0b11100000;                 // Clear gain bits
  value = value >> ADS122C04_CTRL1_DR; // Mask in new bits
  return (ADS122C04_DR_Values)value;
}
uint16_t ADS122C04::getDataRateRealSamples()
{
  ADS122C04_MODE_Values mode = getOperatingMode();
  ADS122C04_DR_Values dr = getDataRate();

  if (mode == ADS122C04_MODE_NORMAL)
  {
    return dr_normal_mode[dr];
  }
  else if (mode == ADS122C04_MODE_TURBO)
  {
    return dr_normal_mode[dr] * 2;
  }
  else
  {
    return 0;
  }
}

bool ADS122C04::setIdacCurrent(ADS122C04_IDAC_Values idac)
{
  uint8_t value = readRegister(ADS122C04_CTRL2);
  value &= 0b11111000;                   // Clear gain bits
  value |= idac << ADS122C04_CTRL2_IDAC; // Mask in new bits
  return (writeRegister(ADS122C04_CTRL2, value));
}
bool ADS122C04::setBurnoutCurrentSource(ADS122C04_BCS_Values bo)
{
  return setBit(ADS122C04_CTRL2_BCS, ADS122C04_CTRL2, bo);
}
bool ADS122C04::setDataIntegrityCheck(ADS122C04_CRC_Values crc)
{
  return false; // TODO: not implemented yet, need to decode conversion result properly.
  uint8_t value = readRegister(ADS122C04_CTRL2);
  value &= 0b11001111;                 // Clear gain bits
  value |= crc << ADS122C04_CTRL2_CRC; // Mask in new bits
  return (writeRegister(ADS122C04_CTRL2, value));
}
bool ADS122C04::setDataCounter(ADS122C04_DCNT_Values datacounter)
{
  return false; // TODO: not implemented yet, need to decode conversion result properly.
  return setBit(ADS122C04_CTRL2_DCNT, ADS122C04_CTRL2, datacounter);
}
bool ADS122C04::getDataReadyFlag()
{
  return getBit(ADS122C04_CTRL2_DRDY, ADS122C04_CTRL2);
}

bool ADS122C04::getDataReadyPin()
{
  return digitalRead(_DRDY) == LOW;
}

bool ADS122C04::setIDAC2Routing(ADS122C04_I2MUX_Values i2mux)
{
  uint8_t value = readRegister(ADS122C04_CTRL3);
  value &= 0b11100011;                     // Clear gain bits
  value |= i2mux << ADS122C04_CTRL3_I1MUX; // Mask in new bits
  return (writeRegister(ADS122C04_CTRL3, value));
}
bool ADS122C04::setIDAC1Routing(ADS122C04_I1MUX_Values i1mux)
{
  uint8_t value = readRegister(ADS122C04_CTRL3);
  value &= 0b00011111;                     // Clear gain bits
  value |= i1mux << ADS122C04_CTRL3_I2MUX; // Mask in new bits
  return (writeRegister(ADS122C04_CTRL3, value));
}

//
// ************** COMMANDS
//
bool ADS122C04::cmdStartSync()
{
  return command(ADS122C04_CMD_STARTSYNC);
}
// Resets all registers to Power Of Defaults
bool ADS122C04::cmdReset()
{

  bool retval = command(ADS122C04_CMD_RESET);
  delay(1);

  // invalidate internal offset calibration
  setInternalCalibrationOffset(0);

  return retval;
}
bool ADS122C04::cmdPowerDown()
{
  return command(ADS122C04_CMD_PWRDOWN);
}

// Returns true if Cycle Ready bit is set (conversion is complete)
bool ADS122C04::available()
{
  // if DRDY pin mode, check that, otherwise register: //TODO:
  if (_DRDY == -1)
  {
    return getDataReadyFlag();
  }
  else
  {
    return getDataReadyPin();
  }
}

// Returns true if Cycle Ready bit is set (conversion is complete)
bool ADS122C04::waitUntilAvailable(uint16_t timeout_ms)
{
  uint32_t begin = millis();
  while (!available())
  {
    if (((millis() - begin) > timeout_ms))
    {
      return false;
    }
  }

  return true;
}

// Returns 24-bit reading
int32_t ADS122C04::getReadingRaw()
{

  if (getBit(ADS122C04_CTRL1_CM, ADS122C04_CTRL1) == ADS122C04_CM_SINGLE_SHOT_MODE)
  {
    cmdStartSync();
  }

  if (!waitUntilAvailable())
  {
    return INT_MAX;
  }

  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(ADS122C04_CMD_RDATA); // SEND COMMAND TO READ DATA
  if (_i2cPort->endTransmission() != 0)
    return INT_MAX; // Sensor did not ACK

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)3);

  if (_i2cPort->available())
  {
    uint32_t valueRaw = (uint32_t)_i2cPort->read() << 16; // MSB
    valueRaw |= (uint32_t)_i2cPort->read() << 8;          // MidSB
    valueRaw |= (uint32_t)_i2cPort->read();               // LSB

    int32_t valueShifted = (int32_t)(valueRaw << 8);
    int32_t value = (valueShifted >> 8);

    return value;
  }
  return INT_MAX;
}

int32_t ADS122C04::getAverageReadingRaw(uint8_t number)
{

  int64_t total = 0; // warning: can overflow in theory!

  for (int i = 0; i < number; i++)
  {
    total += getReadingRaw();
  }

  return total / number;
}

int32_t ADS122C04::getReading()
{
  return getReadingRaw() - getInternalCalibrationOffset();
}

int32_t ADS122C04::getAverageReading(uint8_t number)
{
  return getAverageReadingRaw(number) - getInternalCalibrationOffset();
}

/// @brief Recommended to perform internal calibration after gain changed
/// @return calibration successful
bool ADS122C04::internalCalibration(uint16_t time_ms)
{
  // helps improving accuracy by removing any offset from internal circuitry

  // account for different sample rates
  uint16_t samplesToRead = (uint16_t)(getDataRateRealSamples() * (time_ms / 1000.0));
  if (samplesToRead < 4)
  {
    samplesToRead = 4;
  }

  // remove pga offset by shorting mux and deduct offset afterwards from readings
  ADS122C04_MUX_Values mux_before_calibration = getMux();
  // internally short mux - so read output shall be 0
  setMux(ADS122C04_MUX_AINp_AINn_SHORTED_TO_AVDD_AVSS_DIV2);
  delay(10);
  // throw away to ensure stable results
  getAverageReadingRaw();
  // deviation from 0 is offset that is permanently removed from following readings
  setInternalCalibrationOffset(getAverageReadingRaw(samplesToRead));
  // restore former mux config
  setMux(mux_before_calibration);

  return true;
}

void ADS122C04::setInternalCalibrationOffset(int32_t val)
{
  _internal_calibration_offset = val;
}
int32_t ADS122C04::getInternalCalibrationOffset()
{
  return _internal_calibration_offset;
}

bool ADS122C04::sensorConnected()
{
  setBurnoutCurrentSource(ADS122C04_BCS_CURRENT_SOURCE_ON);
  int32_t reading = getReadingRaw();
  setBurnoutCurrentSource(ADS122C04_BCS_CURRENT_SOURCE_OFF);
  if (reading >= 0x7FFFFF)
    return false;
  else
    return true;
}

void ADS122C04::printRegisterValues()
{
  Serial.print("Config_Regs 0-3: ");
  Serial.print(readRegister(ADS122C04_CTRL0), BIN);
  Serial.print(" ");
  Serial.print(readRegister(ADS122C04_CTRL1), BIN);
  Serial.print(" ");
  Serial.print(readRegister(ADS122C04_CTRL2), BIN);
  Serial.print(" ");
  Serial.print(readRegister(ADS122C04_CTRL3), BIN);
  Serial.println("");
}

/* *********************************************************
 *  I2C communication methods
 ********************************************************* */

// Mask & set a given bit within a register
bool ADS122C04::setBit(uint8_t bitNumber, uint8_t registerAddress, bool value)
{
  if (value)
  {
    return setBit(bitNumber, registerAddress);
  }
  else
  {
    return clearBit(bitNumber, registerAddress);
  }
}

// Mask & set a given bit within a register
bool ADS122C04::setBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = readRegister(registerAddress);
  value |= (1 << bitNumber); // Set this bit
  return (writeRegister(registerAddress, value));
}

// Mask & clear a given bit within a register
bool ADS122C04::clearBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = readRegister(registerAddress);
  value &= ~(1 << bitNumber); // Set this bit
  return (writeRegister(registerAddress, value));
}

// Return a given bit within a register
bool ADS122C04::getBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = readRegister(registerAddress);
  value &= (1 << bitNumber); // Clear all but this bit
  return (value);
}

// Get contents of a register
uint8_t ADS122C04::readRegister(uint8_t reg)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(ADS122C04_CMD_RREG | (reg << 2));
  if (_i2cPort->endTransmission() != 0)
  {
    // log_e("readRegister failed, no ACK");
    return (-1); // Sensor did not ACK  //TODO: -1 cant return to uint8_t?
  }
  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)1);

  if (_i2cPort->available())
  {
    // log_d("readRegister success, return value");
    return (_i2cPort->read());
  }

  // log_e("readRegister failed, other error");
  return (-1); // Error
}

// Send a given value to be written to given address
// Return true if successful
bool ADS122C04::writeRegister(uint8_t reg, uint8_t val)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(ADS122C04_CMD_WREG | (reg << 2));
  _i2cPort->write(val);
  if (_i2cPort->endTransmission() != 0)
    return (false); // Sensor did not ACK
  return (true);
}

bool ADS122C04::command(uint8_t cmd)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(cmd);
  if (_i2cPort->endTransmission() != 0)
    return (false); // Sensor did not ACK
  return (true);
}

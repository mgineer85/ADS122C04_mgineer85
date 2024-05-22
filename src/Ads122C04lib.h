/*

*/

#pragma once

#include <Arduino.h>
#include <Wire.h>

#define ADS122C04_DRDY_PIN_DEFAULT -1  // 6 // data ready pin
#define ADS122C04_I2CADDR_DEFAULT 0x40 // I2C address default (A0 GND, A1 GND)
#define ADS122C04_TIMEOUT_DEFAULT 75   // default timeout to wait for any data (lowest DR 20SPS -> 50ms + some buffer)

// ADS122C04 commands
typedef enum
{
  ADS122C04_CMD_RESET = 0b00000110,     // Reset the device
  ADS122C04_CMD_STARTSYNC = 0b00001000, // Start or restart conversions
  ADS122C04_CMD_PWRDOWN = 0b00000010,   // Enter power-down mode
  ADS122C04_CMD_RDATA = 0b00010000,     // Read data by command
  ADS122C04_CMD_RREG = 0b00100000,      // Read nn registers starting at address rr
  ADS122C04_CMD_WREG = 0b01000000,      // Write nn registers starting at address rr, following dddddddd databyte
} ADS122C04_Commands;

// Register Map
typedef enum
{
  ADS122C04_CTRL0 = 0x00,
  ADS122C04_CTRL1,
  ADS122C04_CTRL2,
  ADS122C04_CTRL3,
} ADS122C04_Registers;

// Bits within the CTRL0 register
typedef enum
{
  ADS122C04_CTRL0_PGA_BYPASS = 0, // [0]
  ADS122C04_CTRL0_GAIN = 1,       // [3:1]
  ADS122C04_CTRL0_MUX = 4,        // [7:4]
} CTRL0_Bits;

// Bits within the CTRL1 register
typedef enum
{
  ADS122C04_CTRL1_TS = 0,   // [0]
  ADS122C04_CTRL1_VREF = 1, // [2:1]
  ADS122C04_CTRL1_CM = 3,   // [3]
  ADS122C04_CTRL1_MODE = 4, // [4]
  ADS122C04_CTRL1_DR = 5,   // [7:5]
} CTRL1_Bits;

// Bits within the CTRL2 register
typedef enum
{
  ADS122C04_CTRL2_IDAC = 0, // [2:0]
  ADS122C04_CTRL2_BCS = 3,  // [3]
  ADS122C04_CTRL2_CRC = 4,  // [5:4]
  ADS122C04_CTRL2_DCNT = 6, // [6]
  ADS122C04_CTRL2_DRDY = 7, // [7] (readonly)
} CTRL2_Bits;

// Bits within the CTRL3 register
typedef enum
{
  ADS122C04_CTRL3_RESERVED = 0, // [1:0]
  ADS122C04_CTRL3_I2MUX = 2,    // [4:2]
  ADS122C04_CTRL3_I1MUX = 5,    // [7:5]
} CTRL3_Bits;

// CTRL0[0] Disables and bypasses the internal low-noise PGA
typedef enum
{
  ADS122C04_PGA_ENABLED = 0, // default
  ADS122C04_PGA_DISABLED_BYPASSED = 1,
} ADS122C04_PGA_BYPASS_Values;

// CTRL0[3:1] Gain configuration
typedef enum
{
  ADS122C04_GAIN_1 = 0b000, // default
  ADS122C04_GAIN_2 = 0b001,
  ADS122C04_GAIN_4 = 0b010,
  ADS122C04_GAIN_8 = 0b011,
  ADS122C04_GAIN_16 = 0b100,
  ADS122C04_GAIN_32 = 0b101,
  ADS122C04_GAIN_64 = 0b110,
  ADS122C04_GAIN_128 = 0b111,
} ADS122C04_GAIN_Values;

// CTRL0[7:4] Input multiplexer configuration
typedef enum
{
  ADS122C04_MUX_pAIN0_nAIN1 = 0b0000, // default
  ADS122C04_MUX_pAIN0_nAIN2,
  ADS122C04_MUX_pAIN0_nAIN3,
  ADS122C04_MUX_pAIN1_nAIN0,
  ADS122C04_MUX_pAIN1_nAIN2,
  ADS122C04_MUX_pAIN1_nAIN3,
  ADS122C04_MUX_pAIN2_nAIN3,
  ADS122C04_MUX_pAIN3_nAIN2,
  ADS122C04_MUX_pAIN0_nAVSS,
  ADS122C04_MUX_pAIN1_nAVSS,
  ADS122C04_MUX_pAIN2_nAVSS,
  ADS122C04_MUX_pAIN3_nAVSS,
  ADS122C04_MUX_VREFP_VREFN_DIV4_MONITOR_PGA_BYPASSED,
  ADS122C04_MUX_AVDD_AVSS_DIV4_MONITOR_PGA_BYPASSED,
  ADS122C04_MUX_AINp_AINn_SHORTED_TO_AVDD_AVSS_DIV2,
  // ADS122C04_MUX_RESERVED,
} ADS122C04_MUX_Values;

// CTRL1[0] Temperature sensor mode
typedef enum
{
  ADS122C04_TS_TEMP_SENSOR_MODE_DISABLED = 0, // default
  ADS122C04_TS_TEMP_SENSOR_MODE_ENABLED = 1,
} ADS122C04_TS_Values;

// CTRL1[2:1] Voltage reference selection
typedef enum
{
  ADS122C04_VREF_INTERNAL_2048 = 0b00, // default
  ADS122C04_VREF_EXTERNAL_REFP_REFN = 0b01,
  ADS122C04_VREF_ANALOG_SUPPY_AVDD_AVSS = 0b10,
} ADS122C04_VREF_Values;

// CTRL1[3] Conversion mode
typedef enum
{
  ADS122C04_CM_SINGLE_SHOT_MODE = 0, // default
  ADS122C04_CM_CONTINUOUS_CONVERSION_MODE = 1,
} ADS122C04_CM_Values;

// CTRL1[4] Operating mode
typedef enum
{
  ADS122C04_MODE_NORMAL = 0b0, // default
  ADS122C04_MODE_TURBO = 0b1,
} ADS122C04_MODE_Values;

// CTRL1[7:5] Data rate depends on operating mode
typedef enum
{
  ADS122C04_DR_NORMAL20_TURBO40 = 0b000, // default
  ADS122C04_DR_NORMAL45_TURBO90 = 0b001,
  ADS122C04_DR_NORMAL90_TURBO180 = 0b010,
  ADS122C04_DR_NORMAL175_TURBO350 = 0b011,
  ADS122C04_DR_NORMAL330_TURBO660 = 0b100,
  ADS122C04_DR_NORMAL600_TURBO1200 = 0b101,
  ADS122C04_DR_NORMAL1000_TURBO2000 = 0b110,
  // ADS122C04_DR_RESERVED = 0b111,
} ADS122C04_DR_Values;
static const uint16_t dr_normal_mode[7] =
    {
        20,
        45,
        90,
        175,
        330,
        600,
        1000,
};

// CTRL2[2:0] IDAC current setting
typedef enum
{
  ADS122C04_IDAC_OFF = 0b000, // default
  ADS122C04_IDAC_10uA = 0b001,
  ADS122C04_IDAC_50uA = 0b010,
  ADS122C04_IDAC_100uA = 0b011,
  ADS122C04_IDAC_250uA = 0b100,
  ADS122C04_IDAC_500uA = 0b101,
  ADS122C04_IDAC_1000uA = 0b110,
  ADS122C04_IDAC_1500uA = 0b111,
} ADS122C04_IDAC_Values;

// CTRL2[3] Burn-out current sources
typedef enum
{
  ADS122C04_BCS_CURRENT_SOURCE_OFF = 0, // default
  ADS122C04_BCS_CURRENT_SOURCE_ON = 1,
} ADS122C04_BCS_Values;

// CTRL2[5:4] CRC
typedef enum
{
  ADS122C04_CRC_DISABLED = 0b00, // default
  ADS122C04_CRC_INVERTED_DATA_OUT = 0b01,
  ADS122C04_CRC_CRC16_EN = 0b10,

} ADS122C04_CRC_Values;

// CTRL2[6] DCNT Data counter enable
typedef enum
{
  ADS122C04_DCNT_DISABLED = 0b0, // default
  ADS122C04_DCNT_ENABLED = 0b1,
} ADS122C04_DCNT_Values;

// CTRL2[7] DRDY Conversion result ready flag
typedef enum
{
  ADS122C04_DRDY_NO_NEW_AVAIL = 0b0, // default
  ADS122C04_DRDY_NEW_AVAIL = 0b1,
} ADS122C04_DRDY_Values;

// CTRL3[1:0] Reserved

// CTRL3[4:2] IDAC2 routing configuration
typedef enum
{
  ADS122C04_I2MUX_DISABLED = 0b000, // default
  ADS122C04_I2MUX_AIN0 = 0b001,
  ADS122C04_I2MUX_AIN1 = 0b010,
  ADS122C04_I2MUX_AIN2 = 0b011,
  ADS122C04_I2MUX_AIN3 = 0b100,
  ADS122C04_I2MUX_REFP = 0b101,
  ADS122C04_I2MUX_REFN = 0b110,
  // ADS122C04_I2MUX_RESERVED = 0b111,
} ADS122C04_I2MUX_Values;

// CTRL3[7:5] IDAC1 routing configuration
typedef enum
{
  ADS122C04_I1MUX_DISABLED = 0b000, // default
  ADS122C04_I1MUX_AIN0 = 0b001,
  ADS122C04_I1MUX_AIN1 = 0b010,
  ADS122C04_I1MUX_AIN2 = 0b011,
  ADS122C04_I1MUX_AIN3 = 0b100,
  ADS122C04_I1MUX_REFP = 0b101,
  ADS122C04_I1MUX_REFN = 0b110,
  // ADS122C04_I2MUX_RESERVED = 0b111,
} ADS122C04_I1MUX_Values;

class ADS122C04
{
  /* *************************************************** PUBLIC *************************************************** */

public: // basic functions and communication related
  ADS122C04();
  bool begin(uint8_t deviceAddress = ADS122C04_I2CADDR_DEFAULT, int DRDY_pin = ADS122C04_DRDY_PIN_DEFAULT, TwoWire &wirePort = Wire);
  bool isConnected(); // Returns true if device is detected

public: // ADS122C04 related functions
  // configure register functions
  bool setPgaBypass(ADS122C04_PGA_BYPASS_Values pga_bypass); // CONF0:
  bool setGain(ADS122C04_GAIN_Values gain);                  // CONF0: Set the gain. x1, 2, 4, 8, 16, 32, 64, 128 are available
  ADS122C04_GAIN_Values getGain();                           // CONF0: Get the gain. x1, 2, 4, 8, 16, 32, 64, 128 are available
  bool setMux(ADS122C04_MUX_Values mux);                     // CONF0:
  ADS122C04_MUX_Values getMux();                             // CONF0:
  bool setTemperatureSensorMode(ADS122C04_TS_Values ts);     // CONF1: Temperature Sensor mode
  bool setVoltageReference(ADS122C04_VREF_Values vref);      // CONF1: Voltage reference selection
  bool setConversionMode(ADS122C04_CM_Values cm);            // CONF1: Conversion mode
  bool setOperatingMode(ADS122C04_MODE_Values mode);         // CONF1: set operating mode (normal/turbo)
  ADS122C04_MODE_Values getOperatingMode();                  // CONF1: get operating mode (normal/turbo)
  bool setDataRate(ADS122C04_DR_Values dr);                  // CONF1: set datarate
  ADS122C04_DR_Values getDataRate();                         // CONF1: get datarate
  uint16_t getDataRateRealSamples();                         // get real datarate in samples per second
  bool setIdacCurrent(ADS122C04_IDAC_Values idac);           // CONF2: IDAC current setting
  bool setBurnoutCurrentSource(ADS122C04_BCS_Values bo);     // CONF2: Burnout current sources
  bool setDataIntegrityCheck(ADS122C04_CRC_Values crc);      // CONF2: Data integrity check enable
  bool setDataCounter(ADS122C04_DCNT_Values datacounter);    // CONF2: Data counter enable
  bool setIDAC2Routing(ADS122C04_I2MUX_Values i2mux);        // CONF3: IDAC2 routing config
  bool setIDAC1Routing(ADS122C04_I1MUX_Values i1mux);        // CONF3: IDAC1 routing config

  // commands
  bool cmdStartSync();
  bool cmdReset();
  bool cmdPowerDown();

  // utility functions that complete the lib
  bool available();                                                         // Returns true if data is available (either using DRDY pin or flag if no pin given)
  bool waitUntilAvailable(uint16_t timeout_ms = ADS122C04_TIMEOUT_DEFAULT); // blocks until timeout (return false) or data avail within timeout (return true)
  int32_t getReading();                                                     // reading from adc includes offset from internal calibration
  int32_t getAverageReading(uint8_t number = 4);                            // reading from adc averaged and corrected offset from internal calibration
  bool internalCalibration(uint16_t time_ms = 200);                         // Call after mode changes, time_ms is over which time to average (accounts for different sps rates)
  bool sensorConnected();                                                   // Use burnout current to detect open connection (no sensor)
  void printRegisterValues();

public:
  // bool tara();

  /* *************************************************** PRIVATE *************************************************** */

private: // ADS122C04 related stuff
  int32_t _internal_calibration_offset = 0;
  int32_t getReadingRaw();                          // straight from adc
  int32_t getAverageReadingRaw(uint8_t number = 4); // straight from adc but averaged
  void setInternalCalibrationOffset(int32_t val);   // set internal offset value to compensate public getReading and getAverageReading for
  int32_t getInternalCalibrationOffset();           // get current internal offset value
  bool getDataReadyFlag();                          // Data ready flag, read externally via available() that would respect register or pin if configured.
  bool getDataReadyPin();                           // Data ready derived by pin DRDY

private: // I2C and communication related stuff
  TwoWire *_i2cPort;
  uint8_t _deviceAddress = ADS122C04_I2CADDR_DEFAULT; // Default unshifted 7-bit address of the ADS122C04
  int _DRDY = ADS122C04_DRDY_PIN_DEFAULT;             // use pin to check data avil - NOT IMPLEMENTED YET

  bool setBit(uint8_t bitNumber, uint8_t registerAddress);
  bool setBit(uint8_t bitNumber, uint8_t registerAddress, bool value);
  bool clearBit(uint8_t bitNumber, uint8_t registerAddress);
  bool getBit(uint8_t bitNumber, uint8_t registerAddress);
  uint8_t readRegister(uint8_t reg);
  bool writeRegister(uint8_t reg, uint8_t val);
  bool command(uint8_t cmd);
};

/*
  Strain Gauge Simulator Example

*/
#include <Arduino.h>
#include <Wire.h>
#include <ADS122C04lib.h>

/* Create your ADS122C04 object */
ADS122C04 ads = ADS122C04();
float sensor_scale_factor;
int32_t sensor_zero_balance_raw;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for user to open terminal

    Serial.println("starting");

    Wire.begin();
    delay(100);
    if (!ads.begin())
    {
        Serial.println("Error connecting to device! Freezing.");

        while (1)
            ;
    }
    else
    {
        Serial.println("Connected to device!");
    }

    // setup strain gauge config
    ads.setConversionMode(ADS122C04_CM_CONTINUOUS_CONVERSION_MODE);
    ads.setMux(ADS122C04_MUX_pAIN1_nAIN0);
    ads.setVoltageReference(ADS122C04_VREF_EXTERNAL_REFP_REFN);
    ads.setGain(ADS122C04_GAIN_128);
    ads.cmdStartSync();

    // optimized for lowest noise
    ads.setOperatingMode(ADS122C04_MODE_TURBO);
    ads.setDataRate(ADS122C04_DR_NORMAL20_TURBO40); // slowest turbo mode. turbo mode and average 4 is lower noise than normal mode average 2.

    // offset calibration
    ads.internalCalibration();

    // print register values
    ads.printRegisterValues();

    // onetime math to calculate mV/V
    uint8_t gain = 1 << ads.getGain();
    float sensitivity = 1.0;
    float zerobalance = 0.0;
    float cali_gain_factor = 1.0;
    float cali_offset = 0.0;
    float fullrange = 1.0;

    sensor_scale_factor = ((float)(1 << 24) * (float)(gain) * ((sensitivity * cali_gain_factor))) / (1000.0 * fullrange * 2.0);
    sensor_zero_balance_raw = (int)((zerobalance - cali_offset) * (float)(1 << 24) * (float)(gain) / 1000.0);
}

void loop()
{
    int32_t reading;
    float reading_mvPerV;

    reading = ads.getAverageReading(8);
    reading_mvPerV = ((reading - sensor_zero_balance_raw) / sensor_scale_factor);

    Serial.print(millis());
    Serial.print(" ");
    Serial.print(reading);
    Serial.print(" ");
    Serial.println(reading_mvPerV, 5);
}
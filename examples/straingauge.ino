/*
  Strain Gauge Optimized Example

*/
#include <Arduino.h>
#include <ADS122C04lib.h>

/* Create your ADS122C04 object */
ADS122C04 ads = ADS122C04();

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for user to open terminal

    Serial.println("starting");

    Wire.begin();

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

    // eco mode, turn off bridge excitation if no measurement
    ads.setConversionMode(ADS122C04_CM_SINGLE_SHOT_MODE);

    // setup strain gauge config
    ads.setMux(ADS122C04_MUX_pAIN1_nAIN0);
    ads.setVoltageReference(ADS122C04_VREF_EXTERNAL_REFP_REFN);
    ads.setGain(ADS122C04_GAIN_128);

    // optimized for lowest noise
    ads.setOperatingMode(ADS122C04_MODE_TURBO);
    ads.setDataRate(ADS122C04_DR_NORMAL20_TURBO40); // slowest turbo mode. turbo mode and average 4 is lower noise than normal mode average 2.

    // offset calibration
    ads.internalCalibration();
}

void loop()
{
    int32_t reading;

    if (!ads.sensorConnected())
    {
        Serial.println("no sensor detected!");
    }
    else
    {
        delay(500);
        reading = ads.getReading() / 128;
        ads.cmdPowerDown();
        Serial.print("reading: ");
        Serial.println(reading);
    }

    delay(1000);
}

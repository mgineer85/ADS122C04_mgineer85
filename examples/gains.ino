/*
  Demonstrate different gains used for strain gauge application
  confirm that the result is almost equal if internalCalibration is issued after setting gain

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

    // setup strain gauge
    ads.setMux(ADS122C04_MUX_AIN1_AIN2);
    ads.setVoltageReference(ADS122C04_VREF_EXTERNAL_REFP_REFN);
    ads.setGain(ADS122C04_GAIN_128);

    ads.internalCalibration();
}

void loop()
{
    int32_t reading;

    Serial.print("gain:128 ");
    ads.setGain(ADS122C04_GAIN_128);
    ads.internalCalibration();
    reading = ads.getReading() / 128;
    Serial.println(reading);
    delay(50);

    Serial.print("gain:64  ");
    ads.setGain(ADS122C04_GAIN_64);
    ads.internalCalibration();
    reading = ads.getReading() / 64;
    Serial.println(reading);
    delay(50);

    Serial.print("gain:32  ");
    ads.setGain(ADS122C04_GAIN_32);
    ads.internalCalibration();
    reading = ads.getReading() / 32;
    Serial.println(reading);
    delay(50);

    Serial.print("gain:16  ");
    ads.setGain(ADS122C04_GAIN_16);
    ads.internalCalibration();
    reading = ads.getReading() / 16;
    Serial.println(reading);
    delay(50);

    delay(1000);
}

/*
  Sensordetect example

  results should be equal for both, means that the burnout current does not affect the reading right after
  sensor detection test.

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

    ads.setConversionMode(ADS122C04_CM_SINGLE_SHOT_MODE);

    // setup strain gauge
    ads.setMux(ADS122C04_MUX_pAIN1_nAIN0);
    ads.setVoltageReference(ADS122C04_VREF_EXTERNAL_REFP_REFN);
    ads.setGain(ADS122C04_GAIN_128);
    ads.internalCalibration();
}

void loop()
{
    int32_t reading_with_previous_sensorcheck;
    int32_t reading_without_previous_sensorcheck;

    if (!ads.sensorConnected())
    {
        Serial.println("no sensor detected!");
    }
    reading_with_previous_sensorcheck = ads.getReading() / 128;
    delay(500);
    reading_without_previous_sensorcheck = ads.getReading() / 128;

    Serial.print("reading_with_previous_sensorcheck:    ");
    Serial.println(reading_with_previous_sensorcheck);
    Serial.print("reading_without_previous_sensorcheck: ");
    Serial.println(reading_without_previous_sensorcheck);
    Serial.print("if values different, the burnout current affects readings!");

    delay(500);
}

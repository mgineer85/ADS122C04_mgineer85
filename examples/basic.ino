/*
  Basic Example

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
}

void loop()
{
    int32_t reading = ads.getReading();
    Serial.println(reading);

    delay(1000);
}

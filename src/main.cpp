#include <Arduino.h>

int led = LED_BUILTIN;
#include "esp_clk.h"

void setup()
{
    // Some boards work best if we also make a serial connection
    Serial.begin(115200);

    // set LED to be an output pin
    pinMode(led, OUTPUT);
    digitalWrite(led, HIGH); // turn the LED on (HIGH is the voltage level)
}

void loop()
{
    uint32_t cpu_freq = esp_clk_cpu_freq();

    // Say hi!
    Serial.println(cpu_freq);

    delay(500); // wait for a half second
}

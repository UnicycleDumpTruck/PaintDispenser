/***************************************************
Implementing a multi-nozzle version, starting with
a struct for each nozzle, and a 2D array of sensor
samples.
****************************************************/

#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc_a;
Adafruit_MCP3008 adc_b;
#define NUMBER_OF_SAMPLES 10
samples[NUMBER_OF_SAMPLES][16];
current_sample = 0;

int count = 0; // Counter for serial monitor to show change between lines

// structures

struct nozzle
{
    int position;
    int leds[4];
    // TODO pump obj in nozzle
    int sensor_a; // position in 16 sensor sample arrays
    int sensor_b; // position in 16 sensor sample arrays
}

// prototypes

void
readSensors();

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;
    delay(1000);
    Serial.println("Multi-Nozzle Test");

    // Hardware SPI (specify CS, use any available digital)
    adc_a.begin(5);
    adc_b.begin(6);
}

void loop()
{
    readSensors();
    delay(500);
}

void readSensors() // replace next set of samples with fresh read
{
    Serial.println(current_sample);
    sample = 0;
    for (int chan = 0; chan < 8; chan++)
    {
        samples[current_sample][sample] = adc_a.readADC(chan);
        Serial.print(samples[current_sample][sample]);
        Serial.print("\t");
    }
    for (int chan = 0; chan < 8; chan++)
    {
        samples[current_sample][sample] = adc_b.readADC(chan);
        Serial.print(adc_b.readADC(chan));
        Serial.print("\t");
    }
    if (current_sample < NUMBER_OF_SAMPLES)
    {
        current_sample++;
    }
    else
    {
        current_sample = 0;
    }

    Serial.print("[");
    Serial.print(count);
    Serial.println("]");
    count++;
}
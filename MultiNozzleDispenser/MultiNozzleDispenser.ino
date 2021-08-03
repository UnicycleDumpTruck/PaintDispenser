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
} n0, n1, n2, n3, n4;

nozzles[] = [ n0, n1, n2, n3, n4 ];

n0.position = 0;
n0.leds[] = [ 0, 1, 2, 3 ];
// TODO ref pump in n0
n0.sensor_a = 0;
n0.sensor_b = 1;
n0.a_thresh = 120;
n0.b_thresh = 120;

n1.position = 0;
n1.leds[] = [ 4, 5, 6, 7 ];
// TODO ref pump in n0
n1.sensor_a = 2;
n1.sensor_b = 3;
n1.a_thresh = 120;
n1.b_thresh = 120;

n2.position = 0;
n2.leds[] = [ 8, 9, 10, 11 ];
// TODO ref pump in n0
n2.sensor_a = 4;
n2.sensor_b = 5;
n2.a_thresh = 120;
n2.b_thresh = 120;

n3.position = 0;
n3.leds[] = [ 12, 13, 14, 15 ];
// TODO ref pump in n0
n3.sensor_a = 6;
n3.sensor_b = 7;
n3.a_thresh = 120;
n3.b_thresh = 120;

n4.position = 0;
n4.leds[] = [ 16, 17, 18, 19 ];
// TODO ref pump in n0
n4.sensor_a = 8;
n4.sensor_b = 9;
n4.a_thresh = 120;
n4.b_thresh = 120;

// prototypes
void sort(int arr[], int size);
void readSensors();
bool detect(int sensor_pin, int threshold);

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
    for (int nozzle = 0; nozzle < 5; nozzle++)
    {
        if (detect(nozzles[nozzle].sensor_a, nozzles[nozzle].a_thresh) && detect(nozzles[nozzle].sensor_b, nozzles[nozzle].b_thresh))
        {
            Serial.println(nozzles[nozzle].position);
        }
    }
    delay(100);
}

// Sort an array
void sort(int arr[], int size)
{
    for (int i = 0; i < (size - 1); i++)
    {
        bool doneFlag = true;
        for (int o = 0; o < (size - (i + 1)); o++)
        {
            if (arr[o] > arr[o + 1])
            {
                int tmp = arr[o];
                arr[o] = arr[o + 1];
                arr[o + 1] = tmp;
                doneFlag = false;
            }
        }
        if (doneFlag)
            break;
    }
}

void readSensors() // replace next set of samples with fresh read
{
    Serial.println(current_sample);
    sample = 0;
    for (int chan = 0; chan < 8; chan++)
    {
        samples[current_sample][sample] = adc_a.readADC(chan);
        //Serial.print(samples[current_sample][sample]);
        //Serial.print("\t");
    }
    for (int chan = 0; chan < 8; chan++)
    {
        samples[current_sample][sample] = adc_b.readADC(chan);
        //Serial.print(adc_b.readADC(chan));
        //Serial.print("\t");
    }
    if (current_sample < NUMBER_OF_SAMPLES)
    {
        current_sample++;
    }
    else
    {
        current_sample = 0;
    }

    //Serial.print("[");
    //Serial.print(count);
    //Serial.println("]");
    //count++;
}

bool detect(int sensor_pin, int threshold)
{ // Read a sensor, return true if it's over the threshold
    int samples[NUMBER_OF_SAMPLES];

    for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
    {
        samples[i] = analogRead(sensor_pin);
        delay(1);
    }

    sort(samples, NUMBER_OF_SAMPLES);
    int medianReading = samples[((int)(NUMBER_OF_SAMPLES / 2))];

#ifdef DEBUG
    if (sensor_pin == LEFT_SENSOR_PIN)
    {
        Serial.print(sensor_pin);
        Serial.print("  ");
        Serial.print(medianReading);
        Serial.print("  ");
    }
    else if (sensor_pin == RIGHT_SENSOR_PIN)
    {
        Serial.print(sensor_pin);
        Serial.print("  ");
        Serial.println(medianReading);
    }
#endif

    if (medianReading > threshold)
    {
        return true;
    }
    return false;
}

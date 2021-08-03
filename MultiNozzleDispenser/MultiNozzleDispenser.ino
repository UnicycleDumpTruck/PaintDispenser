// ██████╗  █████╗ ██╗███╗   ██╗████████╗    ██████╗ ██╗███████╗██████╗ ███████╗███╗   ██╗███████╗███████╗██████╗
// ██╔══██╗██╔══██╗██║████╗  ██║╚══██╔══╝    ██╔══██╗██║██╔════╝██╔══██╗██╔════╝████╗  ██║██╔════╝██╔════╝██╔══██╗
// ██████╔╝███████║██║██╔██╗ ██║   ██║       ██║  ██║██║███████╗██████╔╝█████╗  ██╔██╗ ██║███████╗█████╗  ██████╔╝
// ██╔═══╝ ██╔══██║██║██║╚██╗██║   ██║       ██║  ██║██║╚════██║██╔═══╝ ██╔══╝  ██║╚██╗██║╚════██║██╔══╝  ██╔══██╗
// ██║     ██║  ██║██║██║ ╚████║   ██║       ██████╔╝██║███████║██║     ███████╗██║ ╚████║███████║███████╗██║  ██║
// ╚═╝     ╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝   ╚═╝       ╚═════╝ ╚═╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═══╝╚══════╝╚══════╝╚═╝  ╚═╝

/***************************************************
Implementing a multi-nozzle version, starting with
a struct for each nozzle, and a 2D array of sensor
samples.
****************************************************/

#include <Adafruit_SleepyDog.h>
#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc_a;
Adafruit_MCP3008 adc_b;
#define NUMBER_OF_SAMPLES 10
int all_samples[NUMBER_OF_SAMPLES][16];
int current_sample = 0;

int count = 0; // Counter for serial monitor to show change between lines

// ███████╗████████╗██████╗ ██╗   ██╗ ██████╗████████╗██╗   ██╗██████╗ ███████╗███████╗
// ██╔════╝╚══██╔══╝██╔══██╗██║   ██║██╔════╝╚══██╔══╝██║   ██║██╔══██╗██╔════╝██╔════╝
// ███████╗   ██║   ██████╔╝██║   ██║██║        ██║   ██║   ██║██████╔╝█████╗  ███████╗
// ╚════██║   ██║   ██╔══██╗██║   ██║██║        ██║   ██║   ██║██╔══██╗██╔══╝  ╚════██║
// ███████║   ██║   ██║  ██║╚██████╔╝╚██████╗   ██║   ╚██████╔╝██║  ██║███████╗███████║
// ╚══════╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝  ╚═════╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝╚══════╝╚══════╝

typedef struct
{
    int pos;
    int leds[4];
    // TODO pump obj in nozzle
    int sensor_a; // position in 16 sensor sample arrays
    int sensor_b; // position in 16 sensor sample arrays
    int a_thresh; // threshold for pallete detection
    int b_thresh;
} nozzle;

nozzle n0{0, {0, 1, 2, 3}, 0, 1, 120, 120};
nozzle n1{1, {4, 5, 6, 7}, 2, 3, 120, 120};
nozzle n2{2, {8, 9, 10, 11}, 4, 5, 120, 120};
nozzle n3{3, {12, 13, 14, 15}, 6, 7, 120, 120};
nozzle n4{4, {16, 17, 18, 19}, 8, 9, 120, 120};

nozzle nozzles[5] = {n0, n1, n2, n3, n4};

//  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗ ████████╗██╗   ██╗██████╗ ███████╗███████╗
// ██╔══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔═══██╗╚══██╔══╝╚██╗ ██╔╝██╔══██╗██╔════╝██╔════╝
// ██████╔╝██████╔╝██║   ██║   ██║   ██║   ██║   ██║    ╚████╔╝ ██████╔╝█████╗  ███████╗
// ██╔═══╝ ██╔══██╗██║   ██║   ██║   ██║   ██║   ██║     ╚██╔╝  ██╔═══╝ ██╔══╝  ╚════██║
// ██║     ██║  ██║╚██████╔╝   ██║   ╚██████╔╝   ██║      ██║   ██║     ███████╗███████║
// ╚═╝     ╚═╝  ╚═╝ ╚═════╝    ╚═╝    ╚═════╝    ╚═╝      ╚═╝   ╚═╝     ╚══════╝╚══════╝

void sort(int arr[], int size);
void readSensors();
bool detect(int sensor_pin, int threshold);

// ███████╗███████╗████████╗██╗   ██╗██████╗
// ██╔════╝██╔════╝╚══██╔══╝██║   ██║██╔══██╗
// ███████╗█████╗     ██║   ██║   ██║██████╔╝
// ╚════██║██╔══╝     ██║   ██║   ██║██╔═══╝
// ███████║███████╗   ██║   ╚██████╔╝██║
// ╚══════╝╚══════╝   ╚═╝    ╚═════╝ ╚═╝

void setup()
{

    Serial.begin(9600);
    while (!Serial)
        ;
    delay(1000);
    Serial.println("Multi-Nozzle Paint Dispenser");
    Watchdog.enable(4000);
    Serial.println("Watchdog enabled.");
    // Hardware SPI (specify CS, use any available digital)
    adc_a.begin(5);
    adc_b.begin(6);
}

// ██╗      ██████╗  ██████╗ ██████╗
// ██║     ██╔═══██╗██╔═══██╗██╔══██╗
// ██║     ██║   ██║██║   ██║██████╔╝
// ██║     ██║   ██║██║   ██║██╔═══╝
// ███████╗╚██████╔╝╚██████╔╝██║
// ╚══════╝ ╚═════╝  ╚═════╝ ╚═╝

void loop()
{
    readSensors();
    for (int nozzle = 0; nozzle < 5; nozzle++)
    {
        if (detect(nozzles[nozzle].sensor_a, nozzles[nozzle].a_thresh) && detect(nozzles[nozzle].sensor_b, nozzles[nozzle].b_thresh))
        {
            Serial.println(nozzles[nozzle].pos);
        }
    }
    delay(100);
    Watchdog.reset();
}

// ███████╗ ██████╗ ██████╗ ████████╗
// ██╔════╝██╔═══██╗██╔══██╗╚══██╔══╝
// ███████╗██║   ██║██████╔╝   ██║
// ╚════██║██║   ██║██╔══██╗   ██║
// ███████║╚██████╔╝██║  ██║   ██║
// ╚══════╝ ╚═════╝ ╚═╝  ╚═╝   ╚═╝

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

// ██████╗ ███████╗ █████╗ ██████╗ ███████╗███████╗███╗   ██╗███████╗ ██████╗ ██████╗ ███████╗
// ██╔══██╗██╔════╝██╔══██╗██╔══██╗██╔════╝██╔════╝████╗  ██║██╔════╝██╔═══██╗██╔══██╗██╔════╝
// ██████╔╝█████╗  ███████║██║  ██║███████╗█████╗  ██╔██╗ ██║███████╗██║   ██║██████╔╝███████╗
// ██╔══██╗██╔══╝  ██╔══██║██║  ██║╚════██║██╔══╝  ██║╚██╗██║╚════██║██║   ██║██╔══██╗╚════██║
// ██║  ██║███████╗██║  ██║██████╔╝███████║███████╗██║ ╚████║███████║╚██████╔╝██║  ██║███████║
// ╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝╚═════╝ ╚══════╝╚══════╝╚═╝  ╚═══╝╚══════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝

void readSensors() // replace next set of samples with fresh read
{
    //Serial.println(current_sample);
    int sample = 0;
    for (int chan = 0; chan < 8; chan++)
    {
        all_samples[current_sample][sample] = adc_a.readADC(chan);
        Serial.print(all_samples[current_sample][sample]);
        Serial.print("\t");
    }
    for (int chan = 0; chan < 8; chan++)
    {
        all_samples[current_sample][sample] = adc_b.readADC(chan);
        Serial.print(all_samples[current_sample][sample]);
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

// ██████╗ ███████╗████████╗███████╗ ██████╗████████╗
// ██╔══██╗██╔════╝╚══██╔══╝██╔════╝██╔════╝╚══██╔══╝
// ██║  ██║█████╗     ██║   █████╗  ██║        ██║
// ██║  ██║██╔══╝     ██║   ██╔══╝  ██║        ██║
// ██████╔╝███████╗   ██║   ███████╗╚██████╗   ██║
// ╚═════╝ ╚══════╝   ╚═╝   ╚══════╝ ╚═════╝   ╚═╝

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

// ███████╗███╗   ██╗██████╗
// ██╔════╝████╗  ██║██╔══██╗
// █████╗  ██╔██╗ ██║██║  ██║
// ██╔══╝  ██║╚██╗██║██║  ██║
// ███████╗██║ ╚████║██████╔╝
// ╚══════╝╚═╝  ╚═══╝╚═════╝

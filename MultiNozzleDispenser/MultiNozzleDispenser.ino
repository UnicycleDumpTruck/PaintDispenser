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
#include <JrkG2.h>
// #include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoPXL8.h>

Adafruit_MCP3008 adc_a;
Adafruit_MCP3008 adc_b;
#define NUMBER_OF_SAMPLES 10
int all_samples[NUMBER_OF_SAMPLES][16];
int current_sample = 0;

int count = 0; // Counter for serial monitor to show change between lines

// // Possible states for main loop:
// #define IDLE 0
// #define DETECTED 1
// #define DISPENSING 2
// int state = IDLE;

// Motor Speeds
#define DISPENSE_DURATION 2000
#define DISPENSE_SPEED 1448
#define STOP_SPEED 2048

// LED Constants
// #define LED_PIN 9
// #define LED_COUNT 8 // 8 LEDs per strand to neoPXL8 wing
#define LED_STRAND_SIZE 8
#define NUM_LED_STRANDS 5
#define PIXEL_BRIGHTNESS 20
#define FILL_DELAY 75  // Minimum delay between leds before dispense. Sensor sampling also delays.
#define UNFILL_DELAY 50 // Minimum delay between leds after dispense. Sensor sampling also delays.
#define SOLID_GREEN 1
#define SOLID_RED 2

unsigned long prev_led_change_millis = 0; // When the last LED change occurred.
int red_leader = -1;                      // Position of the lowest red LED.
int led_goal = SOLID_GREEN;               // Just a goal, not necessarily the current state.

// Full Strip of NeoPixels, to be broken up
// Adafruit_NeoPixel strip(LED_STRAND_SIZE, LED_PIN, NEO_GRB + NEO_KHZ800);
int8_t pins[8] = {PIN_SERIAL1_RX, PIN_SERIAL1_TX, 9, 6, 13, 12, 11, 10};
Adafruit_NeoPXL8 strip(LED_STRAND_SIZE, pins, NEO_RGBW);

JrkG2I2C jrk0(11);                       // Pololu motor controller
JrkG2I2C jrk1(12);                       // Pololu motor controller
JrkG2I2C jrk2(13);                       // Pololu motor controller
JrkG2I2C jrk3(14);                       // Pololu motor controller
JrkG2I2C jrk4(15);                       // Pololu motor controller
unsigned long dispense_begin_millis = 0; // When the motor began dispensing Paint
bool dispensing = false;

bool palette_clear = false;
bool not_dispensed = true;

// ███████╗████████╗██████╗ ██╗   ██╗ ██████╗████████╗██╗   ██╗██████╗ ███████╗███████╗
// ██╔════╝╚══██╔══╝██╔══██╗██║   ██║██╔════╝╚══██╔══╝██║   ██║██╔══██╗██╔════╝██╔════╝
// ███████╗   ██║   ██████╔╝██║   ██║██║        ██║   ██║   ██║██████╔╝█████╗  ███████╗
// ╚════██║   ██║   ██╔══██╗██║   ██║██║        ██║   ██║   ██║██╔══██╗██╔══╝  ╚════██║
// ███████║   ██║   ██║  ██║╚██████╔╝╚██████╗   ██║   ╚██████╔╝██║  ██║███████╗███████║
// ╚══════╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝  ╚═════╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝╚══════╝╚══════╝

typedef struct
{
    int pos;
    uint16_t leds[LED_STRAND_SIZE];
    JrkG2I2C jrk;
    int sensor_a; // position in 16 sensor sample arrays
    int sensor_b; // position in 16 sensor sample arrays
    int a_thresh; // threshold for pallete detection
    int b_thresh;
} nozzle;

nozzle n0{0, {0, 1, 2, 3, 4, 5, 6, 7}, jrk0, 6, 7, 130, 130}; // Rightmost, from rear view
nozzle n1{1, {8, 9, 10, 11, 12, 13, 14, 15}, jrk1, 8, 9, 130, 130};
nozzle n2{2, {16, 17, 18, 19, 20, 21, 22, 23}, jrk2, 10, 11, 130, 130};
nozzle n3{3, {24, 25, 26, 27, 28, 29, 30, 31}, jrk3, 12, 13, 130, 130};
nozzle n4{4, {32, 33, 34, 35, 36, 37, 38, 39}, jrk4, 14, 15, 130, 130}; // Leftmost, from rear view

nozzle nozzles[5] = {n0, n1, n2, n3, n4};
nozzle *engaged_nozzle = NULL; // Point to something, so we don't get weird.

//  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗ ████████╗██╗   ██╗██████╗ ███████╗███████╗
// ██╔══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔═══██╗╚══██╔══╝╚██╗ ██╔╝██╔══██╗██╔════╝██╔════╝
// ██████╔╝██████╔╝██║   ██║   ██║   ██║   ██║   ██║    ╚████╔╝ ██████╔╝█████╗  ███████╗
// ██╔═══╝ ██╔══██╗██║   ██║   ██║   ██║   ██║   ██║     ╚██╔╝  ██╔═══╝ ██╔══╝  ╚════██║
// ██║     ██║  ██║╚██████╔╝   ██║   ╚██████╔╝   ██║      ██║   ██║     ███████╗███████║
// ╚═╝     ╚═╝  ╚═╝ ╚═════╝    ╚═╝    ╚═════╝    ╚═╝      ╚═╝   ╚═╝     ╚══════╝╚══════╝

void sort(int arr[], int size);
void readSensors();
bool detect(int sensor_pin, int threshold);
bool detectNozzle(nozzle *noz);
void beginDispense(JrkG2I2C jrk);
void endDispense(JrkG2I2C jrk);
void ledGreen();
void ledRed();

// ███████╗███████╗████████╗██╗   ██╗██████╗
// ██╔════╝██╔════╝╚══██╔══╝██║   ██║██╔══██╗
// ███████╗█████╗     ██║   ██║   ██║██████╔╝
// ╚════██║██╔══╝     ██║   ██║   ██║██╔═══╝
// ███████║███████╗   ██║   ╚██████╔╝██║
// ╚══════╝╚══════╝   ╚═╝    ╚═════╝ ╚═╝

void setup()
{

    Serial.begin(9600);
    // while (!Serial)
    //     ;
    delay(1000);
    Serial.println("Multi-Nozzle Paint Dispenser");
    Watchdog.enable(4000);
    Serial.println("Watchdog enabled.");
    Wire.begin(); // Set up I2C.

    // Hardware SPI (specify CS, use any available digital)
    adc_a.begin(5);
    adc_b.begin(4);
    if (strip.begin()) // Start the NeoPixel strip
    {
        Serial.println("LED strip memory allocated an initialized successfully.");
    }
    else
    {
        Serial.println("LED strip FAILED to allocate memory and initialize pins.");
    }
    delay(100);
    for (int i=0; i<(NUM_LED_STRANDS * LED_STRAND_SIZE); i++) // All pixels
    {
        strip.setPixelColor(i, 0, PIXEL_BRIGHTNESS, 0); // Set Red
    }
    strip.show();
}

// ██╗      ██████╗  ██████╗ ██████╗
// ██║     ██╔═══██╗██╔═══██╗██╔══██╗
// ██║     ██║   ██║██║   ██║██████╔╝
// ██║     ██║   ██║██║   ██║██╔═══╝
// ███████╗╚██████╔╝╚██████╔╝██║
// ╚══════╝ ╚═════╝  ╚═════╝ ╚═╝

void loop()
{
    // End dispensing if DISPENSE_DURATION has elapsed:
    if (((millis() - dispense_begin_millis) > DISPENSE_DURATION) && dispensing)
    {
        Serial.println("Dispense Duration elapsed, stopping pump");
        //delay(2000); // Hold it so we can see in the serial monitor
        endDispense(engaged_nozzle->jrk);
        // TODO return LED strip to idle???
        // engaged_nozzle = NULL;
    }

    // Manage LED chain:
    if ((led_goal == SOLID_GREEN) && red_leader > -1)
    { // If our goal is green, but we aren't fully green yet
        if ((millis() - prev_led_change_millis) > UNFILL_DELAY)
        { // if we have waited long enough since the last led change
            //strip.setPixelColor(red_leader, 0, PIXEL_BRIGHTNESS, 0); // Set Green
            strip.setPixelColor(engaged_nozzle->leds[red_leader], 0, PIXEL_BRIGHTNESS, 0); // Set Green
            strip.show();
            red_leader--; // Decrement after led change
            prev_led_change_millis = millis();
        }
    }
    else if ((led_goal == SOLID_RED) && red_leader < LED_STRAND_SIZE)
    { // if our goal is red, but we aren't fully red yet
        if ((millis() - prev_led_change_millis) > FILL_DELAY)
        {                 // if we have waited long enough since the last led change
            red_leader++; // Increment before led change
            //strip.setPixelColor(red_leader, PIXEL_BRIGHTNESS, 0, 0); // Set Red
            strip.setPixelColor(engaged_nozzle->leds[red_leader], PIXEL_BRIGHTNESS, 0, 0); // Set Red
            strip.show();
            prev_led_change_millis = millis();
        }
    } // else we have achieved our led goal

    // Read sensors, start dispense if necessary, end if palette withdrawn early:

    readSensors();
    if (engaged_nozzle == NULL) // if not engaged, check for new detects
    {
        if (red_leader < 0)
        { // been empty long enough to allow new dispense event
            not_dispensed = true;
            //ledGreen();
        }
        palette_clear = true;
        //Serial.println("Palette gone from not engaged");

        for (int nozzle = 0; nozzle < 5; nozzle++)
        {
            if (detectNozzle(&nozzles[nozzle]))
            {
                engaged_nozzle = &nozzles[nozzle];
                Serial.print("============Detected and Engaged============== ");
                Serial.println(engaged_nozzle->pos);
                break; // Don't check the other nozzles
            }
        }
    }
    else // if we are engaged with a nozzle
    {
        if (detectNozzle(engaged_nozzle)) // if that nozzle still detects
        {                                 // palette now present
            if (palette_clear)
            { // Palette was not previously present
                palette_clear = false;
                Serial.println("Palette appeared");
                led_goal = SOLID_RED;
            }
            else
            { // Palette still there from previous dispense
                // if (not_dispensed)
                // {
                //     Serial.println("present predispense");
                // }
                // else
                // {
                //     Serial.println("post-dispense");
                // }
                if ((red_leader == LED_STRAND_SIZE) && not_dispensed)
                {
                    not_dispensed = false;
                    //ledRed();
                    //dispense();
                    Serial.println("++++++++++++++++++++++++ Dispensing ++++++++++++++++++++++++");
                    //delay(2000);
                    beginDispense(engaged_nozzle->jrk);
                }
            }
        }
        else                // if engaged no longer detects
        {                   // palette absent
            if (dispensing) // Palette was withdrawn (or sensor error) while dispensing
            {
                Serial.println("Palette disappeard prematurely");
#ifdef KEEP_DISPENSING_IF_PREMATURELY_WITHDRAWN
                endDispense(engaged_nozzle->jrk);
#endif
            }
            // Serial.print("Red Leader: ");
            // Serial.println(red_leader);
            if (red_leader < 0)
            { // been empty long enough to allow new dispense event
                not_dispensed = true;
                //ledGreen();
                engaged_nozzle = NULL; // Moved this up from below to try to fix LEDs not returning to red after disengage
            }
            //engaged_nozzle = NULL;
            palette_clear = true;
            Serial.println("Palette gone");
            led_goal = SOLID_GREEN;
        }
    }
    delay(5);

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
        // Serial.print(all_samples[current_sample][sample]);
        // Serial.print("\t");
        sample++;
    }
    for (int chan = 0; chan < 8; chan++)
    {
        all_samples[current_sample][sample] = adc_b.readADC(chan);
        // Serial.print(all_samples[current_sample][sample]);
        // Serial.print("\t");
        sample++;
    }
    if (current_sample < NUMBER_OF_SAMPLES)
    {
        current_sample++;
    }
    else
    {
        current_sample = 0;
    }

    // Serial.print("[");
    // Serial.print(count);
    // Serial.println("]");
    count++;
}

// ██████╗ ███████╗████████╗███████╗ ██████╗████████╗
// ██╔══██╗██╔════╝╚══██╔══╝██╔════╝██╔════╝╚══██╔══╝
// ██║  ██║█████╗     ██║   █████╗  ██║        ██║
// ██║  ██║██╔══╝     ██║   ██╔══╝  ██║        ██║
// ██████╔╝███████╗   ██║   ███████╗╚██████╗   ██║
// ╚═════╝ ╚══════╝   ╚═╝   ╚══════╝ ╚═════╝   ╚═╝

bool detect(int sample_pos, int threshold)
{ // Read a sensor, return true if it's over the threshold
    int samples[NUMBER_OF_SAMPLES];

    for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
    {
        samples[i] = all_samples[i][sample_pos];
    }

    sort(samples, NUMBER_OF_SAMPLES);                            // put samples into order
    int medianReading = samples[((int)(NUMBER_OF_SAMPLES / 2))]; // choose middle-ish sample

    if (medianReading > threshold)
    {
        return true;
    }
    return false;
}

bool detectNozzle(nozzle *noz)
{
    if (detect(noz->sensor_a, noz->a_thresh) && detect(noz->sensor_b, noz->b_thresh))
    {
        return true;
    }
    return false;
}

// ██████╗ ██╗███████╗██████╗ ███████╗███╗   ██╗███████╗███████╗
// ██╔══██╗██║██╔════╝██╔══██╗██╔════╝████╗  ██║██╔════╝██╔════╝
// ██║  ██║██║███████╗██████╔╝█████╗  ██╔██╗ ██║███████╗█████╗
// ██║  ██║██║╚════██║██╔═══╝ ██╔══╝  ██║╚██╗██║╚════██║██╔══╝
// ██████╔╝██║███████║██║     ███████╗██║ ╚████║███████║███████╗
// ╚═════╝ ╚═╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═══╝╚══════╝╚══════╝

void beginDispense(JrkG2I2C jrk)
{ // Start pump motor
#ifdef DEBUG
    Serial.println("Beginning Dispense");
#endif
    // To be sure only one runs at a time, stop all first
    jrk0.setTarget(STOP_SPEED);
    jrk1.setTarget(STOP_SPEED);
    jrk2.setTarget(STOP_SPEED);
    jrk3.setTarget(STOP_SPEED);
    jrk4.setTarget(STOP_SPEED);
    dispensing = true;
    dispense_begin_millis = millis();
    Serial.print("Speed before set: ");
    Serial.println(jrk.getTarget());
    jrk.setTarget(DISPENSE_SPEED);
    Serial.print("Speed after set: ");
    Serial.println(jrk.getTarget());
}

void endDispense(JrkG2I2C jrk)
{ // Stop pump motor
#ifdef DEBUG
    Serial.println("Ending Dispense");
#endif
    dispensing = false;
    jrk.setTarget(STOP_SPEED);
    delay(10);
    Serial.println("Stopping all in End Dispense");
    for (int i = 0; i < 5; i++)
    {
        nozzles[i].jrk.setTarget(STOP_SPEED);
        delay(10);
    }
}

// ██╗     ███████╗██████╗
// ██║     ██╔════╝██╔══██╗
// ██║     █████╗  ██║  ██║
// ██║     ██╔══╝  ██║  ██║
// ███████╗███████╗██████╔╝
// ╚══════╝╚══════╝╚═════╝

void ledGreen()
{ // Light strip full green, no delays
    for (int i = LED_STRAND_SIZE; i >= 0; i--)
    {
        strip.setPixelColor(i, 0, PIXEL_BRIGHTNESS, 0);
        strip.show();
    }
}

// NOT COMPILING:
// void nozzleLedGreen(uint16_t *leds[])
// { // Light strip full green, no delays
//     for (int i = LED_STRAND_SIZE; i >= 0; i--)
//     {
//         strip.setPixelColor(leds[i], 0, PIXEL_BRIGHTNESS, 0);
//         strip.show();
//     }
// }

void ledRed()
{ // Light strip full red, no delays
    for (int i = 0; i < LED_STRAND_SIZE; i++)
    {
        strip.setPixelColor(i, PIXEL_BRIGHTNESS, 0, 0);
        strip.show();
    }
}

// NOT COMPILING
// void nozzleLedRed(uint16_t *leds[])
// { // Light strip full red, no delays
//     for (int i = 0; i < LED_STRAND_SIZE; i++)
//     {
//         strip.setPixelColor(leds[i], PIXEL_BRIGHTNESS, 0, 0);
//         strip.show();
//     }
// }

// ███████╗███╗   ██╗██████╗
// ██╔════╝████╗  ██║██╔══██╗
// █████╗  ██╔██╗ ██║██║  ██║
// ██╔══╝  ██║╚██╗██║██║  ██║
// ███████╗██║ ╚████║██████╔╝
// ╚══════╝╚═╝  ╚═══╝╚═════╝

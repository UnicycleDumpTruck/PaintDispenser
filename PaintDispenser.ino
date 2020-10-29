//██████╗  █████╗ ██╗███╗   ██╗████████╗    ██████╗ ██╗███████╗██████╗ ███████╗███╗   ██╗███████╗███████╗██████╗
//██╔══██╗██╔══██╗██║████╗  ██║╚══██╔══╝    ██╔══██╗██║██╔════╝██╔══██╗██╔════╝████╗  ██║██╔════╝██╔════╝██╔══██╗
//██████╔╝███████║██║██╔██╗ ██║   ██║       ██║  ██║██║███████╗██████╔╝█████╗  ██╔██╗ ██║███████╗█████╗  ██████╔╝
//██╔═══╝ ██╔══██║██║██║╚██╗██║   ██║       ██║  ██║██║╚════██║██╔═══╝ ██╔══╝  ██║╚██╗██║╚════██║██╔══╝  ██╔══██╗
//██║     ██║  ██║██║██║ ╚████║   ██║       ██████╔╝██║███████║██║     ███████╗██║ ╚████║███████║███████╗██║  ██║
//╚═╝     ╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝   ╚═╝       ╚═════╝ ╚═╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═══╝╚══════╝╚══════╝╚═╝  ╚═╝

// Paint dispenser controlled by Feather M0 Express, has two Sharp IR sensors.
// https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/using-with-arduino-ide

// Pololu #3146 Jrk G2 18v19 Motor Controller connected via i2c, also using pins 5 and 6 for RST and ERR, not sure which.
// https://www.pololu.com/product/3146

// NeoPixel LED strip connected to pin 9 through 470Ω resistor

#include <JrkG2.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG true // Comment out this line to disable Serial.print statements

// Comment out following line to stop dispense if palette withdrawn
//#define KEEP_DISPENSING_IF_PREMATURELY_WITHDRAWN true

#define NUMBER_OF_SAMPLES 10 // Loop is delayed by at 2 * NUMBER_OF_SAMPLES millis
#define LEFT_SENSOR_PIN A1
#define RIGHT_SENSOR_PIN A0
#define LEFT_SENSOR_THRESHOLD 190
#define RIGHT_SENSOR_THRESHOLD 220
#define DISPENSE_DURATION 2000
#define DISPENSE_SPEED 1448
#define STOP_SPEED 2048

#define LED_PIN 9
#define LED_COUNT 13        // 13 leds on prototype housing
#define PIXEL_BRIGHTNESS 10 // kept dim when powered by Feather's onboard 3.3v regulator
#define FILL_DELAY 0        // Minimum delay between leds before dispense. Sensor sampling also delays.
#define UNFILL_DELAY 0      // Minimum delay between leds after dispense. Sensor sampling also delays.
#define SOLID_GREEN 1
#define SOLID_RED 2

unsigned long prev_led_change_millis = 0; // When the last LED change occurred.
int red_leader = -1;                      // Position of the lowest red LED.
int led_goal = SOLID_GREEN;               // Just a goal, not necessarily the current state.

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

JrkG2I2C jrk;                            // Pololu motor controller
unsigned long dispense_begin_millis = 0; // When the motor began dispensing Paint
bool dispensing = false;

bool palette_clear = false;
bool not_dispensed = true;

// ██████╗ ██████╗  ██████╗ ████████╗ ██████╗ ████████╗██╗   ██╗██████╗ ███████╗███████╗
// ██╔══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔═══██╗╚══██╔══╝╚██╗ ██╔╝██╔══██╗██╔════╝██╔════╝
// ██████╔╝██████╔╝██║   ██║   ██║   ██║   ██║   ██║    ╚████╔╝ ██████╔╝█████╗  ███████╗
// ██╔═══╝ ██╔══██╗██║   ██║   ██║   ██║   ██║   ██║     ╚██╔╝  ██╔═══╝ ██╔══╝  ╚════██║
// ██║     ██║  ██║╚██████╔╝   ██║   ╚██████╔╝   ██║      ██║   ██║     ███████╗███████║
// ╚═╝     ╚═╝  ╚═╝ ╚═════╝    ╚═╝    ╚═════╝    ╚═╝      ╚═╝   ╚═╝     ╚══════╝╚══════╝

void sort(int arr[], int size);
bool detect(int sensor_pin, int threshold);
void dispense();
void beginDispense();
void endDispense();
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
  Wire.begin(); // Set up I2C.
#ifdef DEBUG
  Serial.begin(115200); // Commence Serial communication
#endif
  strip.begin(); // Start the NeoPixel strip
  ledGreen();
}

// ███╗   ███╗ █████╗ ██╗███╗   ██╗    ██╗      ██████╗  ██████╗ ██████╗
// ████╗ ████║██╔══██╗██║████╗  ██║    ██║     ██╔═══██╗██╔═══██╗██╔══██╗
// ██╔████╔██║███████║██║██╔██╗ ██║    ██║     ██║   ██║██║   ██║██████╔╝
// ██║╚██╔╝██║██╔══██║██║██║╚██╗██║    ██║     ██║   ██║██║   ██║██╔═══╝
// ██║ ╚═╝ ██║██║  ██║██║██║ ╚████║    ███████╗╚██████╔╝╚██████╔╝██║
// ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝    ╚══════╝ ╚═════╝  ╚═════╝ ╚═╝
void loop()
{

  // End dispensing if DISPENSE_DURATION has elapsed:
  if (((millis() - dispense_begin_millis) > DISPENSE_DURATION) && dispensing)
  {
#ifdef DEBUG
    Serial.println("Duration elapsed");
#endif
    endDispense();
  }

  // Manage LED chain:
  if ((led_goal == SOLID_GREEN) && red_leader > -1)
  { // If our goal is green, but we aren't fully green yet
    if ((millis() - prev_led_change_millis) > UNFILL_DELAY)
    { // if we have waited long enough since the last led change
      strip.setPixelColor(red_leader, 0, PIXEL_BRIGHTNESS, 0);
      strip.show();
      red_leader--; // Decrement after led change
      prev_led_change_millis = millis();
    }
  }
  else if ((led_goal == SOLID_RED) && red_leader < LED_COUNT)
  { // if our goal is red, but we aren't fully red yet
    if ((millis() - prev_led_change_millis) > FILL_DELAY)
    {               // if we have waited long enough since the last led change
      red_leader++; // Increment before led change
      strip.setPixelColor(red_leader, PIXEL_BRIGHTNESS, 0, 0);
      strip.show();
      prev_led_change_millis = millis();
    }
  } // else we have achieved our led goal

  // Read sensors, start dispense if necessary, end if palette withdrawn early:
  bool leftDetect = detect(LEFT_SENSOR_PIN, LEFT_SENSOR_THRESHOLD);
  bool rightDetect = detect(RIGHT_SENSOR_PIN, RIGHT_SENSOR_THRESHOLD);
  if (leftDetect && rightDetect)
  { // palette now present
    if (palette_clear)
    { // Palette was not previously present
      palette_clear = false;
#ifdef DEBUG
      //Serial.println("Palette appeared");
#endif
      led_goal = SOLID_RED;
    }
    else
    { // Palette still there from previous dispense
#ifdef DEBUG
      //Serial.println("hanging around");
#endif
      if ((red_leader == LED_COUNT) && not_dispensed)
      {
        not_dispensed = false;
        //ledRed();
        //dispense();
        beginDispense();
      }
    }
  }
  else
  {                 // palette absent
    if (dispensing) // Palette was withdrawn (or sensor error) while dispensing
    {
#ifdef DEBUG
      Serial.println("Palette disappeard prematurely");
#endif
#ifdef KEEP_DISPENSING_IF_PREMATURELY_WITHDRAWN
      endDispense();
#endif
    }

    if (red_leader < 0)
    { // been empty long enough to allow new dispense event
      not_dispensed = true;
      //ledGreen();
    }

    palette_clear = true;
#ifdef DEBUG
    //Serial.println("Palette gone");
#endif
    led_goal = SOLID_GREEN;
  }

} // END OF MAIN LOOP

// ███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
// ██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
// █████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
// ██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
// ██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
// ╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝

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

void beginDispense()
{ // Start pump motor
#ifdef DEBUG
  Serial.println("Beginning Dispense");
#endif
  dispensing = true;
  dispense_begin_millis = millis();
  jrk.setTarget(DISPENSE_SPEED);
}

void endDispense()
{ // Stop pump motor
#ifdef DEBUG
  Serial.println("Ending Dispense");
#endif
  dispensing = false;
  jrk.setTarget(STOP_SPEED);
}

void ledGreen()
{ // Light strip full green, no delays
  for (int i = LED_COUNT; i >= 0; i--)
  {
    strip.setPixelColor(i, 0, PIXEL_BRIGHTNESS, 0);
    strip.show();
  }
}

void ledRed()
{ // Light strip full red, no delays
  for (int i = 0; i < LED_COUNT; i++)
  {
    strip.setPixelColor(i, PIXEL_BRIGHTNESS, 0, 0);
    strip.show();
  }
}

// ███████╗███╗   ██╗██████╗
// ██╔════╝████╗  ██║██╔══██╗
// █████╗  ██╔██╗ ██║██║  ██║
// ██╔══╝  ██║╚██╗██║██║  ██║
// ███████╗██║ ╚████║██████╔╝
// ╚══════╝╚═╝  ╚═══╝╚═════╝
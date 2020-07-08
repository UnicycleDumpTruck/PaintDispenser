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

#define NUMBER_OF_SAMPLES 20
#define LEFT_SENSOR_PIN A1
#define RIGHT_SENSOR_PIN A0
#define LEFT_SENSOR_THRESHOLD 200
#define RIGHT_SENSOR_THRESHOLD 245
#define DISPENSE_DURATION 2000
#define DISPENSE_SPEED 1448
#define STOP_SPEED 2048
#define POST_DISPENSE_DELAY 0 // Not used, as lights must 'unfill', functioning as delay

#define LED_PIN 9
#define LED_COUNT 13        // 13 leds on prototype housing
#define PIXEL_BRIGHTNESS 10 // kept dim when powered by Feather's onboard 3.3v regulator
#define FILL_DELAY 40       // delay between leds before dispense
#define UNFILL_DELAY 40     // delay between leds after dispense
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
  Wire.begin();         // Set up I2C.
  Serial.begin(115200); // Commence Serial communication
  strip.begin();        // Start the NeoPixel strip
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
    Serial.println("Duration elapsed");
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
  if (detect(LEFT_SENSOR_PIN, LEFT_SENSOR_THRESHOLD) && detect(RIGHT_SENSOR_PIN, RIGHT_SENSOR_THRESHOLD))
  { // palette now present
    if (palette_clear)
    { // Palette was not previously present
      palette_clear = false;
      //Serial.println("Palette appeared");
      led_goal = SOLID_RED;
    }
    else
    { // Palette still there from previous dispense
      //Serial.println("hanging around");
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
  { // palette absent
    if (dispensing)
    {
      Serial.println("Palette disappeard prematurely");
      endDispense();
    }

    if (red_leader < 0)
    { // been empty long enough to allow new dispense event
      not_dispensed = true;
      //ledGreen();
    }

    palette_clear = true;
    //Serial.println("Palette gone");
    led_goal = SOLID_GREEN;
  }

  delay(5); // we don't need it running too fast

} // END OF MAIN LOOP

// ███████╗██╗   ██╗███╗   ██╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗
// ██╔════╝██║   ██║████╗  ██║██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝
// █████╗  ██║   ██║██╔██╗ ██║██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗
// ██╔══╝  ██║   ██║██║╚██╗██║██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║
// ██║     ╚██████╔╝██║ ╚████║╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║
// ╚═╝      ╚═════╝ ╚═╝  ╚═══╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝

// Sort an array
void sort(float a[], int size)
{
  for (int i = 0; i < (size - 1); i++)
  {
    bool flag = true;
    for (int o = 0; o < (size - (i + 1)); o++)
    {
      if (a[o] > a[o + 1])
      {
        int t = a[o];
        a[o] = a[o + 1];
        a[o + 1] = t;
        flag = false;
      }
    }
    if (flag)
      break;
  }
}

bool detect(int sensor_pin, int threshold)
{ // Read a sensor, return true if it's over the threshold
  float samples[NUMBER_OF_SAMPLES];

  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    samples[i] = analogRead(sensor_pin);
  }

  sort(samples, NUMBER_OF_SAMPLES);
  float medianReading = samples[((int)(NUMBER_OF_SAMPLES / 2))];

  Serial.println(medianReading);
  if (medianReading > threshold)
  {
    return true;
  }
  return false;
}

void beginDispense()
{
  Serial.println("Beginning Dispense");
  dispensing = true;
  dispense_begin_millis = millis();
  jrk.setTarget(DISPENSE_SPEED);
}

void endDispense()
{
  Serial.println("Ending Dispense");
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
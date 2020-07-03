// Paint dispenser controlled by Feather M0 Express, has two Sharp IR sensors.
// https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/using-with-arduino-ide

// Uses Pololu #3146 Jrk G2 18v19 Motor Controller.


#include <JrkG2.h>
#include <Adafruit_NeoPixel.h>

#define LEFT_SENSOR_PIN A0
#define RIGHT_SENSOR_PIN A1
#define LEFT_SENSOR_THRESHOLD 200
#define RIGHT_SENSOR_THRESHOLD 200
#define DISPENSE_DURATION 1000
#define DISPENSE_SPEED 1800
#define STOP_SPEED 2048
#define POST_DISPENSE_DELAY 0

#define LED_PIN 9
#define LED_COUNT 13
#define PIXEL_BRIGHTNESS 10 // kept dim when powered by Feather 3.3v regulator
#define FILL_DELAY 40
#define UNFILL_DELAY 200
#define SOLID_GREEN 1
#define SOLID_RED   2

long prev_led_change_millis = 0; // When the last LED change occurred.
int red_leader = -1; // Position of the lowest red LED.
int led_goal = SOLID_GREEN; // Just a goal, not necessarily the current state.


Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

JrkG2I2C jrk; // Pololu motor controller

bool palette_clear = false;
bool not_dispensed = true;

bool detect(int sensor_pin, int threshold) {
  float measuredSensor = analogRead(sensor_pin);
  Serial.println(measuredSensor);
  if (measuredSensor > threshold) {
    return true;
  }
  return false;
}

void dispense() {
  Serial.println("Dispensing");
  jrk.setTarget(DISPENSE_SPEED);
  delay(DISPENSE_DURATION);
  jrk.setTarget(STOP_SPEED);
  delay(POST_DISPENSE_DELAY);
}

void ledGreen() {
  for (int i=LED_COUNT; i>=0; i--) {
    strip.setPixelColor(i, 0, PIXEL_BRIGHTNESS, 0);
    strip.show();
    //delay(FILL_DELAY);
  }
}

void ledRed() {
  for (int i=0; i<LED_COUNT; i++) {
    strip.setPixelColor(i, PIXEL_BRIGHTNESS, 0, 0);
    strip.show();
    //delay(FILL_DELAY);
  }
}

void setup()
{
  // Set up I2C.
  Wire.begin();
  Serial.begin(115200);
  strip.begin();
  ledGreen();
}

void loop()
{
  
  if ((led_goal == SOLID_GREEN) && red_leader > -1) {
    if ((millis() - prev_led_change_millis) > UNFILL_DELAY) {
      strip.setPixelColor(red_leader, 0, PIXEL_BRIGHTNESS, 0);
      strip.show();
      red_leader--;
      prev_led_change_millis = millis();
    }
  } else if ((led_goal == SOLID_RED) && red_leader < LED_COUNT) {
    if ((millis() - prev_led_change_millis) > FILL_DELAY) {
      red_leader++;
      strip.setPixelColor(red_leader, PIXEL_BRIGHTNESS, 0, 0);
      strip.show();
      prev_led_change_millis = millis();
    }
  }
  

  if (detect(LEFT_SENSOR_PIN, LEFT_SENSOR_THRESHOLD) && detect(RIGHT_SENSOR_PIN, RIGHT_SENSOR_THRESHOLD)) { // palette now present
    if (palette_clear) { // Palette was not previously present
      palette_clear = false;
      Serial.println("Palette appeared");
      led_goal = SOLID_RED;
    } else { // Palette still there from previous dispense
      Serial.println("hanging around");
      if ((red_leader == LED_COUNT) && not_dispensed) {
        not_dispensed = false;
        //ledRed();
        dispense();
      }
    } 
  } else { // palette absent
    if (red_leader < 0) { // been empty long enough to allow new dispense event
      not_dispensed = true;
      //ledGreen();
    }
    palette_clear = true;
    Serial.println("Palette gone");
    led_goal = SOLID_GREEN;
  }
  delay(5);
}
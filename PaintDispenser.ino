// Paint dispenser controlled by Feather M0 Express, has two Sharp IR sensors.
// Uses Pololu #3146 Jrk G2 18v19 Motor Controller.
// https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/using-with-arduino-ide


#include <JrkG2.h>
#include <Adafruit_NeoPixel.h>

#define LEFT_SENSOR_PIN A0
#define RIGHT_SENSOR_PIN A1
#define LEFT_SENSOR_THRESHOLD 200
#define RIGHT_SENSOR_THRESHOLD 200
#define DISPENSE_DURATION 1000
#define DISPENSE_SPEED 1800
#define STOP_SPEED 2048
#define POST_DISPENSE_DELAY 500

#define LED_PIN 9
#define LED_COUNT 13
#define PIXEL_BRIGHTNESS 10
#define FILL_DELAY 40


Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

JrkG2I2C jrk;

bool palette_clear = true;

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
  for (int i=0; i<LED_COUNT; i++) {
    strip.setPixelColor(i, 0, PIXEL_BRIGHTNESS, 0);
    strip.show();
    delay(FILL_DELAY);
  }
}

void ledRed() {
  for (int i=0; i<LED_COUNT; i++) {
    strip.setPixelColor(i, PIXEL_BRIGHTNESS, 0, 0);
    strip.show();
    delay(FILL_DELAY);
  }
}

void setup()
{
  // Set up I2C.
  Wire.begin();
  Serial.begin(115200);
  strip.begin();
  strip.show();

  strip.setPixelColor(0, PIXEL_BRIGHTNESS, 0, 0);
  strip.show();
}

void loop()
{
  if (detect(LEFT_SENSOR_PIN, LEFT_SENSOR_THRESHOLD) && detect(RIGHT_SENSOR_PIN, RIGHT_SENSOR_THRESHOLD)) {
    if (palette_clear) {
      palette_clear = false;
      Serial.println("Palette appeared");
      ledRed();
      dispense();
    } else { // Palette still there from previous dispense
      Serial.println("hanging around");
    } 
  } else { // no palette present
    palette_clear = true;
    Serial.println("Palette gone");
    ledGreen();
  }
  delay(100);
}
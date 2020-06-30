// Paint dispenser controlled by Feather M0 Express, has two Sharp IR sensors.
// Uses Pololu #3146 Jrk G2 18v19 Motor Controller.


#include <JrkG2.h>

JrkG2I2C jrk;

void setup()
{
  // Set up I2C.
  Wire.begin();
  Serial.begin(115200);
}

void loop()
{
  delay(1000);
  jrk.setTarget(2048);
  delay(1000);
  jrk.setTarget(1800);
  Serial.println("loop");
}
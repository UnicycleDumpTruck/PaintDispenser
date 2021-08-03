/***************************************************
Simple example of reading the MCP3008 analog input channels and printing
them all out.

Author: Carter Nelson
License: Public Domain
****************************************************/

#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc_a;
Adafruit_MCP3008 adc_b;

int count = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  delay(1000);
  Serial.println("MCP3008 simple test.");

  // Hardware SPI (specify CS, use any available digital)
  // Can use defaults if available, ex: UNO (SS=10) or Huzzah (SS=15)
  adc_a.begin(5);
  adc_b.begin(6);
  // Feather 32u4 (SS=17) or M0 (SS=16), defaults SS not broken out, must specify
  //adc.begin(10);  

  // Software SPI (specify all, use any available digital)
  // (sck, mosi, miso, cs);
  //adc.begin(13, 11, 12, 10);
}

void loop() {
  for (int chan=0; chan<8; chan++) {
    Serial.print(adc_a.readADC(chan)); Serial.print("\t");
    //delay(1);
  }
  for (int chan=0; chan<8; chan++) {
    Serial.print(adc_b.readADC(chan)); Serial.print("\t");
    //delay(1);
  }

  Serial.print("["); Serial.print(count); Serial.println("]");
  count++;
  
  delay(500);
}

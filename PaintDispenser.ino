// Paint dispenser controlled by Feather M0 Express, has two Sharp IR sensors.
// https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/using-with-arduino-ide

// Pololu #3146 Jrk G2 18v19 Motor Controller connected via i2c, also using pins 5 and 6 for RST and ERR, not sure which.

// NeoPixel LED strip connected to pin 9


#include <Adafruit_SleepyDog.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"

char ssid[] = SECRET_SSID;		// your network SSID (name)
char pass[] = SECRET_PASS;    	// your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;	// the WiFi radio's status

long prevWifiCheckMillis = 0;
long wifiInterval = 300000; // Check the wifi connection every 5 minutes

#define SPIWIFI       SPI  // The SPI port
#define SPIWIFI_SS    13   // Chip select pin
#define ESP32_RESETN  12   // Reset pin
#define SPIWIFI_ACK   11   // a.k.a BUSY or READY pin
#define ESP32_GPIO0   -1

IPAddress ip(10, 10, 212, 110);		// will change when moved to new VLAN
//IPAddress ip(192, 168, 0, 101);		// will change when moved to new VLAN
unsigned int localPort = 8888;      // local port to listen on

byte gateway[] = {10, 10, 212, 5}; // was .5 or .15
byte subnet[] = {255, 255, 255, 0};
byte dnsServer[] = {10, 10, 100, 20}; // byte dnsServer[] = {10, 10, 100, 20};
IPAddress splunkIp(10, 10, 100, 150);
unsigned int splunkPort = 4514;
String pingHost = "10.10.212.129";
int pingResult;
const char cEquals[] = "c=";
const char tEquals[] = " t=";
const char sEquals[] = " s=";
const char gEquals[] = " g=";
const char bEquals[] = " b=";
const char fEquals[] = " f=";


WiFiUDP Udp;

void selectWiFi() {
  //delay(100);
//  digitalWrite(SPIWIFI_SS, LOW);
  delay(100);
}

void setupWiFi()
{
	//Configure pins for Adafruit ATWINC1500 Feather

	// Set up the pins!
	WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);

	// check for the WiFi module:
	while (WiFi.status() == WL_NO_MODULE) {
		Serial.println("Communication with WiFi module failed!");
		// don't continue
		delay(1000);
	}
	String fv = WiFi.firmwareVersion();
	Serial.println(fv);
	if (fv < "1.0.0") {
		Serial.println("Please upgrade the firmware");
		while (1) delay(10);
	}
	Serial.println("Firmware OK");

	// print your MAC address:
	byte mac[6];
	WiFi.macAddress(mac);
	Serial.print("MAC: ");
	printMacAddress(mac);


	// attempt to connect to WiFi network:
	while ( status != WL_CONNECTED) {
		Serial.print(F("Attempting to connect to WPA SSID: "));
		Serial.println(ssid);
		
		WiFi.config(ip, dnsServer, gateway, subnet); 
		// Connect to WPA/WPA2 network:
		status = WiFi.begin(ssid, pass);

		// wait 10 seconds for connection:
		for (int i=0; i>10; i++) {
			delay(1000);
			Watchdog.reset();
		}
	}

	// you're connected now, so print out the data:
	Serial.print(F("You're connected to the network"));
	printCurrentNet();
	printWiFiData();

	Udp.begin(localPort);
	Udp.beginPacket(splunkIp, splunkPort);
	Udp.write("c=0 reconnect=1");
	Udp.endPacket();


}

#include <JrkG2.h>

#include <Adafruit_NeoPixel.h>

#define LEFT_SENSOR_PIN A0
#define RIGHT_SENSOR_PIN A1
#define LEFT_SENSOR_THRESHOLD 200
#define RIGHT_SENSOR_THRESHOLD 200
#define DISPENSE_DURATION 1000
#define DISPENSE_SPEED 1800
#define STOP_SPEED 2048
#define POST_DISPENSE_DELAY 0 // Not used, as lights must 'unfill', functioning as delay

#define LED_PIN 9
#define LED_COUNT 13 // 13 leds on prototype housing
#define PIXEL_BRIGHTNESS 10 // kept dim when powered by Feather's onboard 3.3v regulator
#define FILL_DELAY 40 // delay between leds before dispense
#define UNFILL_DELAY 40 // delay between leds after dispense
#define SOLID_GREEN 1
#define SOLID_RED   2

long prev_led_change_millis = 0; // When the last LED change occurred.
int red_leader = -1; // Position of the lowest red LED.
int led_goal = SOLID_GREEN; // Just a goal, not necessarily the current state.


Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

JrkG2I2C jrk; // Pololu motor controller

bool palette_clear = false;
bool not_dispensed = true;

bool detect(int sensor_pin, int threshold) { // Read a sensor, return true if it's over the threshold
  float measuredSensor = analogRead(sensor_pin);
  Serial.println(measuredSensor);
  if (measuredSensor > threshold) {
    return true;
  }
  return false;
}

void dispense() { // Run the motor at DISPENSE_SPEED for DISPENSE_DURATION
  Serial.println("Dispensing");
  jrk.setTarget(DISPENSE_SPEED);
  delay(DISPENSE_DURATION);
  jrk.setTarget(STOP_SPEED);
  delay(POST_DISPENSE_DELAY);
}

void ledGreen() { // Light strip full green, no delays
  for (int i=LED_COUNT; i>=0; i--) {
    strip.setPixelColor(i, 0, PIXEL_BRIGHTNESS, 0);
    strip.show();
  }
}

void ledRed() { // Light strip full red, no delays
  for (int i=0; i<LED_COUNT; i++) {
    strip.setPixelColor(i, PIXEL_BRIGHTNESS, 0, 0);
    strip.show();
  }
}

void sendSplunkEvent(int cubeID, int transmission_number)
{
	char buf[16];
	selectWiFi();
	Udp.beginPacket(splunkIp, splunkPort);
	Udp.write(cEquals);
	Udp.write(itoa(cubeID, buf, 10));
	Udp.write(tEquals);
	Udp.write(itoa(transmission_number, buf, 10));		
	if (Udp.endPacket() == 1) {
		Serial.print(F(" Packet sent"));
	} else {
		Serial.print(F(" Error sending UDP"));
	}		
	Serial.println(F(""));
}


void printWiFiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);

}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void setup()
{
	Wire.begin(); // Set up I2C.
	Serial.begin(115200); // Commence Serial communication
	strip.begin(); // Start the NeoPixel strip
	ledGreen();

	while (!Serial);

	pinMode(ESP32_RESETN,OUTPUT);
	digitalWrite(ESP32_RESETN,LOW);
	delay(500);
	digitalWrite(ESP32_RESETN,HIGH);

	setupWiFi();
	
	Serial.print(F("Pinging "));
	Serial.print(pingHost);
	Serial.print(F(": "));

	pingResult = WiFi.ping(pingHost);

	if (pingResult >= 0) {
		Serial.print(F("SUCCESS! RTT = "));
		Serial.print(pingResult);
		Serial.println(F(" ms"));
	} else {
		Serial.print(F("FAILED! Error code: "));
		Serial.println(pingResult);
	}

	Serial.println(F("Sending reboot=1"));
	Udp.beginPacket(splunkIp, splunkPort);
	Udp.write("c=0 reboot=1");
	Udp.endPacket();

	// First a normal example of using the watchdog timer.
	// Enable the watchdog by calling Watchdog.enable() as below.
	// This will turn on the watchdog timer with a ~4 second timeout
	// before reseting the Arduino. The estimated actual milliseconds
	// before reset (in milliseconds) is returned.
	// Make sure to reset the watchdog before the countdown expires or
	// the Arduino will reset!
	int countdownMS = Watchdog.enable(4000);
	Serial.print(F("Enabled the watchdog with max countdown of "));
	Serial.print(countdownMS, DEC);
	Serial.println(F(" milliseconds!"));
	Serial.println();

} // End of setup //

void loop()
{
  
  if ((led_goal == SOLID_GREEN) && red_leader > -1) { // If our goal is green, but we aren't fully green yet
    if ((millis() - prev_led_change_millis) > UNFILL_DELAY) { // if we have waited long enough since the last led change
      strip.setPixelColor(red_leader, 0, PIXEL_BRIGHTNESS, 0);
      strip.show();
      red_leader--; // Decrement after led change
      prev_led_change_millis = millis();
    }
  } else if ((led_goal == SOLID_RED) && red_leader < LED_COUNT) { // if our goal is red, but we aren't fully red yet
    if ((millis() - prev_led_change_millis) > FILL_DELAY) { // if we have waited long enough since the last led change
      red_leader++; // Increment before led change
      strip.setPixelColor(red_leader, PIXEL_BRIGHTNESS, 0, 0);
      strip.show();
      prev_led_change_millis = millis();
    }
  } // else we have achieved our led goal
  

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
  delay(5); // we don't need it running too fast
}
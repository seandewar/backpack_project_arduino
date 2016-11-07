/*
 Project Arduino Backpack Project
 
 Code for the entry for the Thales Project Arduino competition.
 Team members: Joel Mahon, Sam Hills and Sean Dewar, University of Leicester.

 created 03 Nov 2016
 by Sean Dewar
*/

#include <Servo.h>
#include <SoftwareSerial.h>

// NOTE: Custom Adafruit_Fingerprint lib that fixes corrupted data being read
// from fingerprint scanner and can use AltSoftSerial instead of SoftwareSerial.
#include <Adafruit_Fingerprint.h>

#include <Adafruit_GPS.h>


// Comment/un-comment the below line to disable/enable debug
// messages to be printed through Serial.
// NOTE: DO NOT USE IF PINS 0 OR 1 ARE CONNECTED TO SOMETHING!
#define BP_DEBUG

// Macro for compiling code with or without debug messages to
// hardware serial (pins 0 & 1).
#ifdef BP_DEBUG
  #define BDBG_PRINT(...) (Serial.print(__VA_ARGS__))
  #define BDBG_PRINTLN(...) (Serial.println(__VA_ARGS__))
#else
  #define BDBG_PRINT(...) (void)0
  #define BDBG_PRINTLN(...) (void)0
#endif


// The green (IN) and white (OUT) wires of the fingerprint scanner.
const int fpSerialInPin = 4;
const int fpSerialOutPin = 5;

// The RX and TX wires of the GPS.
const int gpsSerialRXPin = 2;
const int gpsSerialTXPin = 3;

// Pin for the servo that points towards where to go using GPS.
const int locatorServoPin = 10;
// Max angle that we will allow the servo to rotate in degrees.
// Half of this angle means that the servo should be facing straight forwards.
const int locatorServoMaxAngle = 120;


bool userAuthenticated = false;

SoftwareSerial fpScannerSerial(fpSerialInPin, fpSerialOutPin);
Adafruit_Fingerprint fpScanner(&fpScannerSerial);

SoftwareSerial gpsSerial(gpsSerialRXPin, gpsSerialTXPin);
Adafruit_GPS gps(&gpsSerial);

Servo locatorServo;


bool authFingerprint() {
  // Make sure that we have found the fp scanner.
  if (!fpScanner.verifyPassword()) return false;
  
  // Wait for a valid fingerprint.
  while (true) {
    delay(250);
    
    // Scan the fingerprint.
    uint8_t result = fpScanner.getImage();
    if (result != FINGERPRINT_OK) continue;
    
    result = fpScanner.image2Tz();
    if (result != FINGERPRINT_OK) continue;
    
    // Try to match the fingerprint.
    result = fpScanner.fingerFastSearch();
    if (result == FINGERPRINT_OK) return true;
    else if (result == FINGERPRINT_NOTFOUND) return false;
  }
}

// Interrupt called once per ms. Stores new GPS data one byte at a time.
SIGNAL(TIMER0_COMPA_vect) {
  if (gpsSerial.isListening())
    gps.read();
}

void setupGPSInterrupt() {
  // Set up an interrupt on Timer0, which is used for millis().
  // We'll interrupt somewhere in the middle and call our COMPA
  // ISR above to read more data from the GPS.
  OCR0A = 0xaf;
  TIMSK0 |= _BV(OCIE0A);
}

void setup() {
  // Setup pins and data rates for serials.
  Serial.begin(9600);
  BDBG_PRINTLN("Setup..");

  locatorServo.attach(locatorServoPin);

  fpScannerSerial.listen();
  fpScanner.begin(57600);
  
  // Auth using fingerprint scanner.
  BDBG_PRINTLN("Waiting for fingerprint..");
  while (!(userAuthenticated = authFingerprint())) {
    BDBG_PRINTLN("!! Failed to match fingerprint!");
    // TODO: Flash status LED to indicate failure.
  }
  
  // Successfully auth'd using fingerprint!
  BDBG_PRINT("Fingerprint matched! ID "); BDBG_PRINT(fpScanner.fingerID);
  BDBG_PRINT(", Confidence "); BDBG_PRINTLN(fpScanner.confidence);
  
  // Stop listening to the fingerprint scanner and listen to the GPS instead.
  gpsSerial.listen();
  gps.begin(9600);
  // Setup GPS to only use RMC as we only need 2D position for the locator.
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set 1 Hz update rate for GPS.
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // Allow us to read data from the GPS using interrupts.
  setupGPSInterrupt();
  
  // Finished setup, print auth status.
  BDBG_PRINT("Setup done; user auth'd: "); BDBG_PRINTLN(userAuthenticated);
  delay(1000);
}


void handleGPSLocator() {
  // Check if we have finished receiving a new GPS sentence.
  // If we have, try to parse it and checksum it. If this fails, just
  // wait for another sentence. Don't bother if we don't have a fix yet.
  if (!gps.newNMEAreceived() || !gps.parse(gps.lastNMEA())) return;
  if (!gps.fix) return;
  
  BDBG_PRINT("New GPS fix; quality: "); BDBG_PRINT(gps.fixquality);
  BDBG_PRINT(", satellites: "); BDBG_PRINT(gps.satellites);
  BDBG_PRINT(", angle: "); BDBG_PRINTLN(gps.angle, 4);
  BDBG_PRINT("  Location;");
  BDBG_PRINT(" lat: "); BDBG_PRINT(gps.latitude, 4); BDBG_PRINT(gps.lat);
  BDBG_PRINT(", lon: "); BDBG_PRINT(gps.longitude, 4); BDBG_PRINTLN(gps.lon);
  BDBG_PRINT("  In Degrees;");
  BDBG_PRINT(" lat: "); BDBG_PRINT(gps.latitudeDegrees, 4);
  BDBG_PRINT(", lon: "); BDBG_PRINTLN(gps.longitudeDegrees, 4);

  const int angle = locatorServoMaxAngle;
  // TODO
  const int servoAngle = locatorServoMaxAngle - max(0, min(120, (angle + 90))); 
  locatorServo.write(
}

void loop() {
  // Only allow the system to be operatable when the user has auth'd.
  if (userAuthenticated) {
    handleGPSLocator();
  }

  delay(1);
}

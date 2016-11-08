/*
 Project Arduino Tech Backpack Project
 
 Code for the tech backpack entry for the Thales Project Arduino competition.
 Team members: Joel Mahon, Sam Hills and Sean Dewar, University of Leicester.

 created 03 Nov 2016
 by Sean Dewar with great contributions by Joel Mahon.
*/


#include <Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_Fingerprint.h>
#include <Adafruit_GPS.h>


// Comment/un-comment the below line to disable/enable debug
// messages to be printed through Serial.
// NOTE: DO NOT USE IF PINS 0 OR 1 ARE CONNECTED TO SOMETHING!
//#define BP_DEBUG

// Macro for compiling code with or without debug messages to
// hardware serial (pins 0 & 1).
#ifdef BP_DEBUG
  #define BDBG_BEGIN(baud) (Serial.begin(baud))
  #define BDBG_PRINT(...) (Serial.print(__VA_ARGS__))
  #define BDBG_PRINTLN(...) (Serial.println(__VA_ARGS__))
#else
  #define BDBG_BEGIN(baud) (void)0
  #define BDBG_PRINT(...) (void)0
  #define BDBG_PRINTLN(...) (void)0
#endif


// Different locator modes.
enum LocatorMode {
  LOCATOR_FIX_QUALITY,
  LOCATOR_BEARING,
  LOCATOR_DISTANCE,
  LOCATOR_ALTITUDE,
  LOCATOR_SECONDS,
  LOCATOR_MINUTES,
  LOCATOR_HOURS,
  LOCATOR_NUM_MODES // Keep as last in enum.
};


// Mean radius of the Earth to the nearest metre.
const long earthRadius = 6371e3;

// The green (IN) and white (OUT) wires of the fingerprint scanner.
const int fpSerialInPin = 4;
const int fpSerialOutPin = 5;

// The TX and RX wires of the GPS.
//const int gpsSerialTXPin = 2;
//const int gpsSerialRXPin = 3;

// Pin for the servo that points towards where to go using GPS and
// the mode toggle button for the locator.
const int locatorServoPin = 10;
const int locatorModeButtonPin = 9;
// Max angle that we will allow the servo to rotate in degrees.
// Half of this angle means that the servo should be facing straight forwards.
const int locatorServoMaxAngle = 100;
const int locModeButtonChangeDestAfterMs = 1500;

// Pins for the visibility light and the button to toggle it.
const int visibilityLightPin = 7;
const int visibilityLightButtonPin = 8;
const int visLightButtonAlwaysOnAfterMs = 1500;

// Pin for the morse code buzzer, light and receiver.
const int morseBuzzerPin = 11;
const int morseLightPin = 12;
const int morseToggleButtonPin = 6;
const int morseReceiverInPin = 13;
// Frequency of tone to play from buzzer in Hz.
const int morseBuzzerFrequency = 2349;
// If pulse reading from the receiver is in this range, do not beep for morse.
const int morseReceiverNoBeepMax = 1900;


// True if user has successfully auth'd into the system using fingerprint.
bool userAuthenticated = false;

// Last state of the vis light and light toggle button.
int visLightButtonStatePrev = LOW;
bool visLightAlwaysOn = false;
unsigned long visLightButtonStayOnModeTime = 0;

// Last state of the morse code toggle button and state.
bool morseUseBuzzerNotLight = false;
int morseToggleButtonStatePrev = LOW;

// Destination co-ords. NOTE: Default to Morrisons close to Nixon Court.
double destinationLatDegrees = 52.6201813;
double destinationLonDegrees = -1.1337551;

LocatorMode locatorMode = LOCATOR_FIX_QUALITY;
int locatorModeButtonStatePrev = LOW;
unsigned long locModeButtonChangeDestModeTime = 0;

// Whether or not there's been a new update from our GPS.
// NOTE: Make sure we trigger an update by default so that the locator does
// something even if we don't yet have a GPS fix.
bool newGPSUpdate = true;

SoftwareSerial fpScannerSerial(fpSerialInPin, fpSerialOutPin);
Adafruit_Fingerprint fpScanner(&fpScannerSerial);

//SoftwareSerial gpsSerial(gpsSerialRXPin, gpsSerialTXPin);
//Adafruit_GPS gps(&gpsSerial);
Adafruit_GPS gps(&Serial);

Servo locatorServo;


bool authFingerprint() {
  return true;
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


void setup() {
  // Setup pins and data rates for serials.
  BDBG_BEGIN(9600);
  BDBG_PRINTLN("Setup..");

  locatorServo.attach(locatorServoPin);
  locatorServo.write(locatorServoMaxAngle / 2); // Init to middle pos.

  fpScannerSerial.listen();
  fpScanner.begin(57600);

  pinMode(locatorModeButtonPin, INPUT);
  pinMode(visibilityLightPin, OUTPUT);
  pinMode(visibilityLightButtonPin, INPUT);
  pinMode(morseBuzzerPin, OUTPUT);
  pinMode(morseLightPin, OUTPUT);
  pinMode(morseToggleButtonPin, INPUT);
  pinMode(morseReceiverInPin, INPUT);
  
  // Auth using fingerprint scanner.
  BDBG_PRINTLN("Waiting for fingerprint..");
  while (!(userAuthenticated = authFingerprint())) {
    BDBG_PRINTLN("!! Failed to match fingerprint! Try again.");

    // Flash vis light and buzz morse to indicate failure.
    analogWrite(morseBuzzerPin, 100);
    digitalWrite(visibilityLightPin, HIGH);
    delay(2000);
    analogWrite(morseBuzzerPin, 0);
    digitalWrite(visibilityLightPin, LOW);
  }
  
  // Successfully auth'd using fingerprint!
  BDBG_PRINT("Fingerprint matched! ID "); BDBG_PRINT(fpScanner.fingerID);
  BDBG_PRINT(", Confidence "); BDBG_PRINTLN(fpScanner.confidence);
  
  // Stop listening to the fingerprint scanner and listen to the GPS instead.
  gps.begin(9600);
  // Setup GPS to receive RMC and GGA sentences.
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set 1 Hz update rate for GPS.
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // Use interrupts for GPS reading.
  setupGPSInterrupts();
  
  // Finished setup, print auth status.
  BDBG_PRINT("Setup done; user auth'd: "); BDBG_PRINTLN(userAuthenticated);
  delay(1000);
}


SIGNAL(TIMER0_COMPA_vect) {
    // Read a byte from the GPS serial.
    gps.read();
}

void setupGPSInterrupts() {
  // Run our CompareA routine around once every millisecond using Timer0.
  OCR0A = 0xaf;
  TIMSK0 |= _BV(OCIE0A);
}


inline double degreesToRadians(const double deg) {
  return deg * M_PI / 180;
}

inline double radiansToDegrees(const double rad) {
  return rad * 180 / M_PI;
}

double calcBearingLatLon(const double lat1, const double lon1,
                         const double lat2, const double lon2) {
  const double y = sin(lon2 - lon1) * cos(lat2);
  const double x = cos(lat1) * sin(lat2) -
                   sin(lat1) * cos(lat2) * cos(lon2 - lon1);
                   
  return atan2(y, x);
}

double calcDistanceLatLon(const double lat1, const double lon1,
                          const double lat2, const double lon2) {
  const double deltaLat = lat2 - lat1;
  const double deltaLon = lon2 - lon1;

  const double a = sin(deltaLat * 0.5) * sin(deltaLat * 0.5) +
                   cos(lat1) * cos(lat2) *
                   sin(deltaLon * 0.5) * sin(deltaLon * 0.5);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  
  return earthRadius * c;
}


void handleGPSUpdate() {
  // Check if we have finished receiving a new GPS sentence.
  // If we have, try to parse it and checksum it. If this fails, just
  // wait for another sentence. Don't bother if we don't have a fix yet.
  if (!gps.newNMEAreceived() || !gps.parse(gps.lastNMEA())) return;
  if (!gps.fix) {
    BDBG_PRINTLN("Waiting for GPS fix..");
  } else {
    BDBG_PRINT("New GPS fix; quality "); BDBG_PRINT(gps.fixquality);
    BDBG_PRINT(", satellites "); BDBG_PRINTLN(gps.satellites);
    BDBG_PRINT("  Location;");
    BDBG_PRINT(" lat "); BDBG_PRINT(gps.latitude, 4); BDBG_PRINT(gps.lat);
    BDBG_PRINT(", lon "); BDBG_PRINT(gps.longitude, 4); BDBG_PRINTLN(gps.lon);
    BDBG_PRINT("  In Degrees;");
    BDBG_PRINT(" lat "); BDBG_PRINT(gps.latitudeDegrees, 4);
    BDBG_PRINT(", lon "); BDBG_PRINTLN(gps.longitudeDegrees, 4);
  }
  
  newGPSUpdate = true;
}


void handleGPSLocator() {
  bool updateLocator = newGPSUpdate;

  // Handle locator mode change / destination change.
  const int locatorModeButtonState = digitalRead(locatorModeButtonPin);
  if (locatorModeButtonState == HIGH &&
      locatorModeButtonStatePrev == LOW) {
      locModeButtonChangeDestModeTime = millis() +
                                        locModeButtonChangeDestAfterMs;
  } else if (locatorModeButtonState == LOW &&
             locatorModeButtonStatePrev == HIGH) {
    if (millis() >= locModeButtonChangeDestModeTime) {
      if (!gps.fix) {
        BDBG_PRINTLN("Can't set new destination - no GPS fix!");
      } else {
        // Set destination to current location and trigger update.
        destinationLatDegrees = gps.latitudeDegrees;
        destinationLonDegrees = gps.longitudeDegrees;
        
        BDBG_PRINT("Set new destination; (lat, lon) = (");
        BDBG_PRINT(destinationLatDegrees, 4);
        BDBG_PRINT(", "); BDBG_PRINT(destinationLonDegrees, 4);
        BDBG_PRINTLN(")");
      }
    } else {
      // Change to next mode and trigger update.
      locatorMode = (LocatorMode)((int)(locatorMode + 1));
      if ((int)locatorMode >= LOCATOR_NUM_MODES)
        locatorMode = (LocatorMode)0;
        
      BDBG_PRINT("New locator mode: "); BDBG_PRINTLN(locatorMode);
    }

    updateLocator = true;
  }

  locatorModeButtonStatePrev = locatorModeButtonState;

  // No need to update locator if no new GPS data / mode hasn't changed.
  if (!updateLocator) return;
  
  int locatorAngle;
  
  switch (locatorMode) {
    case LOCATOR_FIX_QUALITY: {
      locatorAngle = (int)round(locatorServoMaxAngle * gps.fixquality / 100.0);
      break;  
    }

    case LOCATOR_BEARING: {
      const double angle = radiansToDegrees(calcBearingLatLon(
                        degreesToRadians(gps.latitudeDegrees),
                        degreesToRadians(gps.longitudeDegrees),
                        //degreesToRadians(52.6165069),
                        //degreesToRadians(-1.1297154),
                        degreesToRadians(destinationLatDegrees),
                        degreesToRadians(destinationLonDegrees)));
      locatorAngle = (int)round((angle + 180.0) / 3.0);               
      break;
    }
    
    case LOCATOR_DISTANCE: {
      const double dist = calcDistanceLatLon(
                         degreesToRadians(gps.latitudeDegrees),
                         degreesToRadians(gps.longitudeDegrees),
                         degreesToRadians(destinationLatDegrees),
                         degreesToRadians(destinationLonDegrees));
      //const double distLog = log(dist / 1000.0) / log(2);
      //const double distLogScale = min(4, distLog / 4.0);
      //locatorAngle = (int)round(distLogScale * locatorServoMaxAngle);
      locatorAngle = (int)round(locatorServoMaxAngle * dist / 4000.0);
      break;
    }

    case LOCATOR_ALTITUDE: {
      locatorAngle = (int)round(locatorServoMaxAngle *
                       (gps.altitude + 1000.0) / 10000.0);
      break;
    }

    case LOCATOR_SECONDS: {
      locatorAngle = (int)round(locatorServoMaxAngle * gps.seconds / 59.0);
      break;
    }

    case LOCATOR_MINUTES: {
      locatorAngle = (int)round(locatorServoMaxAngle * gps.minute / 59.0);
      break;
    }

    case LOCATOR_HOURS: {
      locatorAngle = (int)round(locatorServoMaxAngle * gps.hour / 23.0);
      break;
    }
  }
  
  locatorServo.write(max(0, min(locatorServoMaxAngle, locatorAngle)));
}


void handleVisibilityLight() {
  const int visLightButtonState = digitalRead(visibilityLightButtonPin);

  if (visLightButtonState == HIGH && visLightButtonStatePrev == LOW) {
    visLightButtonStayOnModeTime = millis() + visLightButtonAlwaysOnAfterMs;
    visLightAlwaysOn = false;
  } else if (visLightButtonState == LOW && visLightButtonStatePrev == HIGH &&
             millis() >= visLightButtonStayOnModeTime) {
    visLightAlwaysOn = true;
  }

  digitalWrite(visibilityLightPin, (visLightAlwaysOn || visLightButtonState == HIGH) ? HIGH : LOW);
  visLightButtonStatePrev = visLightButtonState;
}


void handleMorseCode() {
  const int morseToggleButtonState = digitalRead(morseToggleButtonPin);

  if (morseToggleButtonState != morseToggleButtonStatePrev &&
      morseToggleButtonState == HIGH) {
    // Morse mode toggled - use light/buzzer instead.
    morseUseBuzzerNotLight = !morseUseBuzzerNotLight;

    // Make sure light/buzzer from previous state isn't activated now.
    if (morseUseBuzzerNotLight) {
      digitalWrite(morseLightPin, LOW);
    } else {
      analogWrite(morseBuzzerPin, 0);
    }
  }

  const int morseReceiverReading = pulseIn(morseReceiverInPin, HIGH, 100000);
  const bool receivingMorse = morseReceiverReading > morseReceiverNoBeepMax;
  
  // If we're receiving morse, use light/buzzer depending on our mode.
  if (morseUseBuzzerNotLight) {
    if (receivingMorse) {
      analogWrite(morseBuzzerPin, 100);
    } else {
      analogWrite(morseBuzzerPin, 0);
    }
  } else if (!morseUseBuzzerNotLight) {
    digitalWrite(morseLightPin, receivingMorse ? HIGH : LOW);
  }

  morseToggleButtonStatePrev = morseToggleButtonState;
}


void loop() {
  // Only allow the system to be operatable when the user has auth'd.
  if (userAuthenticated) {
    handleGPSUpdate();
    handleGPSLocator();
    handleVisibilityLight();
    handleMorseCode();
  }

  delay(1);
}

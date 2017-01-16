#define debugMode  // Comment out this line to eliminate serial prints helpful for debugging
//#define verboseSerial  // Comment out this line to eliminate extra, non-essential serial prints
//#define liveLaunch  // Activates buzzer and any other relevant functions for real launch situation

#include "IntersemaBaro.h"
#include <SoftwareSerial.h> 
#include <TinyGPS.h>
#include <Wire.h>

TinyGPS gps;
SoftwareSerial gpsSerial(8, 7);

Intersema::BaroPressure_MS5607B baro(true);

#define acclAddr (0x1d)
#define compAddr (0x1e)
#define gyroAddr (0x69)

#define dataPin A0
#define readyPin A1
#define landPin A2
#define debugPin 2
#define statusLED 3
#define tiltPin 4

const int sensDelay = 0;  // Delay after each sensor read (milliseconds)
const int gpsDelay = 10;  // Delay between GPS updates (seconds)
const int altTau = 355;

const int launchTriggerVal = 500;  // Altitude (cm) over which launch state triggered after 10 positive reads
const int landTriggerVal = 500;  // Altitude (cm) under which land state triggered after 10 positive reads

int16_t acclx;
int16_t accly;
int16_t acclz;
int16_t compx;
int16_t compy;
int16_t compz;
int16_t gyrox;
int16_t gyroy;
int16_t gyroz;

long altCalibVal;
long altCmInitial;
long altCm;
long altCmLast;
long altCmMax;

float altFeetMax;
float gpsLat;
float gpsLon;
float gpsLatInitial;
float gpsLonInitial;
//float landDistance;

int loopCount = 0;
int altLaunchCount = 0;
int altLandCount = 0;
int landDistance = 0;

boolean liveReading = false;
boolean loopSkip = false;
boolean driftComp = true;
boolean launchDetect = false;
boolean tiltDetect = false;
boolean recoveryDeploy = false;
boolean landDetect = false;

unsigned long landTime;

void setup() {
  pinMode(dataPin, INPUT);
  pinMode(readyPin, OUTPUT);
  pinMode(tiltPin, INPUT);
  pinMode(landPin, OUTPUT);
  pinMode(statusLED, OUTPUT);

  digitalWrite(readyPin, LOW);
  digitalWrite(landPin, LOW);

  Serial.begin(38400);
  Wire.begin();
  baro.init();

  zeroAltitude();  // Can probably delete this...calibration below!
  readAlt();

  while(altCm >= 152) {
#ifdef verboseSerial
    Serial.println(F("Re-zeroing."));
    Serial.println();
#endif
    zeroAltitude();
    delay(5000);
    readAlt();
  }
  altCmInitial = altCm;
  configureSensors();
  checkCalib();

  for(int z = 0; z < 5; z++) {
    delay(50);
    digitalWrite(statusLED, HIGH);
    delay(50);
    digitalWrite(statusLED, LOW);
  }
  gpsSerial.begin(9600);
  if(!gpsSerial.available()) {
    while(!gpsSerial.available()) {
      delay(1);
    }
  }
  getGPSData();
  gpsSerial.end();
  gpsLatInitial = gpsLat;
  gpsLonInitial = gpsLon;
  digitalWrite(readyPin, HIGH);
  sendExtraData();
  Serial.flush();

  for(int stat = 0; stat < 2; stat++) {
    delay(500);
    digitalWrite(statusLED, HIGH);
    delay(500);
    digitalWrite(statusLED, LOW);
  }
}

void loop() {
  if(digitalRead(dataPin) == LOW) {
    while(digitalRead(dataPin) == LOW) {
      delay(1);
    }
  }
  digitalWrite(statusLED, HIGH);
  delay(500);
  digitalWrite(statusLED, LOW);
#ifdef verboseSerial
  Serial.println(F("Stabilizing sensor reads."));
#endif
  for(int boot = 0; boot < 30; boot++) {
    readAlt();
    readAccl();
    readComp();
    readGyro();
  }
  zeroAltitude();
  digitalWrite(statusLED, HIGH);
#ifdef verboseSerial
  Serial.println(F("Sensors stabilized. Proceeding with data transmission."));
  Serial.println();
  Serial.flush();
#endif
  liveReading = true;
  while(landDetect == false) {
    readAlt();
    readAccl();
    readComp();
    readGyro();
    if(tiltDetect == false && digitalRead(tiltPin) == 1) {
      recoveryDeploy = true;
    }
    sendData();
    detectState();
    if(driftComp == true) driftCompensation();
  }
  Serial.flush();
  digitalWrite(landPin, HIGH);

#ifdef verboseSerial
  Serial.println(F("Landing detected. Switching to transmit mode."));
  Serial.println();
  Serial.flush();
#endif
  while(landDetect == true) {
    unsigned long landElapTime = millis() - landTime;
    if(landElapTime > (gpsDelay * 1000)) {
      gpsSerial.begin(4800);
      if(!gpsSerial.available()) {
        while(!gpsSerial.available()) {
          delay(1);
        }
      }
      getGPSData();
      gpsSerial.end();
      digitalWrite(readyPin, LOW);
      sendExtraData();
      Serial.flush();
      delay(1000);
#ifdef verboseSerial
      Serial.println(F("Program complete."));
#endif
      while(landDetect == true) {
        digitalWrite(statusLED, LOW);
        delay(500);
        digitalWrite(statusLED, HIGH);
        delay(50);
      }
    }
    delay(1);
  }
  delay(1);
}

void configureSensors() {
  // Accelerometer
  writeByte(acclAddr, 0x2c, 0x08);
  writeByte(acclAddr, 0x2d, 0x08);
  writeByte(acclAddr, 0x2e, 0x0);
  // Compass
  writeByte(compAddr, 0x0, 0x10);
  writeByte(compAddr, 0x1, 0x20);
  writeByte(compAddr, 0x2, 0x0);
}

void zeroAltitude() {
  long altTot = 0;
#ifdef verboseSerial
  Serial.print(F("Zeroing altimeter. Please wait..."));
#endif
  for(int y = 0; y < 10; y++) {
    long altPrep = baro.getHeightCentiMeters();
    delay(100);
  }
  for(int k = 0; k < 10; k++) {
    long altCalibRead = baro.getHeightCentiMeters();
    altTot += altCalibRead;
    delay(100);
  }
  altCalibVal = altTot / 10;
  delay(1000);
#ifdef verboseSerial
  Serial.println(F("altimeter ready."));
  Serial.println();
#endif
}

void checkCalib() {
#ifdef verboseSerial
  Serial.print(F("Confirming altimeter calibration..."));
#endif
  delay(5000);
  for(int x = 0; x < 10; x++) {
    readAlt();
    if(abs(altCm - altCmInitial) > 61) {
      zeroAltitude();
    }
  }
#ifdef verboseSerial
  Serial.println(F("altimeter correctly calibrated."));
  Serial.println();
#endif
}

void driftCompensation() {
  if(loopCount < altTau + 1 && loopSkip == false) {
    altCalibVal--;
    loopCount++;
  }
  if(loopCount == altTau) {
#ifdef debugMode
    Serial.println(F("Drift compensation complete."));
    Serial.flush();
#endif
    driftComp = false;
  }
  loopSkip = !loopSkip;
}

int readByte(uint8_t addr, uint8_t reg, uint8_t *data) {
  // Do an i2c write to set the register that will be read
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();

  // Read a byte from the device
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available())
  {
    *data = Wire.read();
  }
  else
  {
    // Read nothing back
    return -1;
  }
  return 0;
}

void writeByte(uint8_t addr, uint8_t reg, byte data) {
  // Begin the write sequence
  Wire.beginTransmission(addr);
  // First byte is to set the register pointer
  Wire.write(reg);
  // Write the data byte
  Wire.write(data);
  // End the write sequence; bytes are actually transmitted now
  Wire.endTransmission();
}

void readAlt() {
  altCm = baro.getHeightCentiMeters() - altCalibVal;
  if(liveReading == true) {
    if(altCm > altCmMax) {
      altCmMax = altCm;
    }
  }
  delay(sensDelay);
}

void readAccl() {
  uint8_t devId;
  uint8_t x_msb; // X-axis most significant byte
  uint8_t x_lsb; // X-axis least significant byte
  uint8_t y_msb; // Y-axis most significant byte
  uint8_t y_lsb; // Y-axis least significant byte
  uint8_t z_msb; // Z-axis most significant byte
  uint8_t z_lsb; // Z-axis least significant byte
  uint16_t x;
  uint16_t y;
  uint16_t z;

  // Read the device ID to verify as correct sensor
  if (readByte(acclAddr, 0x0, &devId) != 0) {
    Serial.println(F("Cannot read device ID from sensor"));
  }
  else if (devId != 0xe5) {
    Serial.print(F("Wrong/invalid device ID "));
    Serial.print(devId);
    Serial.println(F(" expected 0xe5)"));
  }
  else {
    // Read the output
    if ((readByte(acclAddr, 0x33, &x_msb) == 0) &&
      (readByte(acclAddr, 0x32, &x_lsb) == 0) &&
      (readByte(acclAddr, 0x35, &y_msb) == 0) &&
      (readByte(acclAddr, 0x34, &y_lsb) == 0) &&
      (readByte(acclAddr, 0x37, &z_msb) == 0) &&
      (readByte(acclAddr, 0x36, &z_lsb) == 0)) {
      x = (x_msb << 8) | x_lsb;
      y = (y_msb << 8) | y_lsb;
      z = (z_msb << 8) | z_lsb;

      // Perform 2's complement
      acclx = ~(x - 1);
      accly = ~(y - 1);
      acclz = ~(z - 1);
    }
  }
  delay(sensDelay);
}

void readComp() {
  uint8_t x_msb; // X-axis most significant byte
  uint8_t x_lsb; // X-axis least significant byte
  uint8_t y_msb; // Y-axis most significant byte
  uint8_t y_lsb; // Y-axis least significant byte
  uint8_t z_msb; // Z-axis most significant byte
  uint8_t z_lsb; // Z-axis least significant byte
  int x;
  int y;
  int z;
  // Get the value from the sensor
  if ((readByte(compAddr, 0x3, &x_msb) == 0) &&
    (readByte(compAddr, 0x4, &x_lsb) == 0) &&
    (readByte(compAddr, 0x5, &y_msb) == 0) &&
    (readByte(compAddr, 0x6, &y_lsb) == 0) &&
    (readByte(compAddr, 0x7, &z_msb) == 0) &&
    (readByte(compAddr, 0x8, &z_lsb) == 0)) {
    compx = x_msb << 8 | x_lsb;
    compy = y_msb << 8 | y_lsb;
    compz = z_msb << 8 | z_lsb;
  }
  delay(sensDelay);
}

void readGyro() {
  uint8_t x_msb; // X-axis most significant byte
  uint8_t x_lsb; // X-axis least significant byte
  uint8_t y_msb; // Y-axis most significant byte
  uint8_t y_lsb; // Y-axis least significant byte
  uint8_t z_msb; // Z-axis most significant byte
  uint8_t z_lsb; // Z-axis least significant byte
  uint16_t x;
  uint16_t y;
  uint16_t z;
  // Get the value from the sensor
  if ((readByte(gyroAddr, 0x1d, &x_msb) == 0) &&
    (readByte(gyroAddr, 0x1e, &x_lsb) == 0) &&
    (readByte(gyroAddr, 0x1f, &y_msb) == 0) &&
    (readByte(gyroAddr, 0x20, &y_lsb) == 0) &&
    (readByte(gyroAddr, 0x21, &z_msb) == 0) &&
    (readByte(gyroAddr, 0x22, &z_lsb) == 0)) {
    x = (x_msb << 8) | x_lsb;
    y = (y_msb << 8) | y_lsb;
    z = (z_msb << 8) | z_lsb;
    // Perform 2's complement
    gyrox = ~(x - 1);
    gyroy = ~(y - 1);
    gyroz = ~(z - 1);
  }
  delay(sensDelay);
}

void sendData() {
  // Altimeter
  Serial.print(altCm);
  Serial.print("/");
  // Accelerometer
  Serial.print(acclx);
  Serial.print(",");
  Serial.print(accly);
  Serial.print(",");
  Serial.print(acclz);
  Serial.print("/");
  // Compass
  Serial.print(compx);
  Serial.print(",");
  Serial.print(compy);
  Serial.print(",");
  Serial.print(compz);
  Serial.print("/");
  // Gyroscope
  Serial.print(gyrox);
  Serial.print(",");
  Serial.print(gyroy);
  Serial.print(",");
  Serial.println(gyroz);
  if(tiltDetect == false && recoveryDeploy == true) {
    Serial.println(F("Tilt detected."));
    tiltDetect = true;
    Serial.flush();
  }
}

void detectState() {
  long altChange = altCm - altCmInitial;
#ifdef debugMode
  if(digitalRead(debugPin) == 0) {
    landDetect = true;
#endif
#ifdef verboseSerial
    Serial.println(F("Debug landing triggered."));
    Serial.println();
    delay(500);
#endif
#ifdef debugMode
    while(digitalRead(debugPin) == 0) {
      delay(100);
    }
  }
#endif
  if(launchDetect == false && altChange > launchTriggerVal && altLaunchCount < 10) {
    altLaunchCount++;
  }
  else if(launchDetect == false && altChange > launchTriggerVal) {
    launchDetect == true;
  }
  if(launchDetect == true && landDetect == false && altChange < landTriggerVal && altLandCount < 10) {
    altLandCount++;
  }
  else if(launchDetect == true && landDetect == false && altChange < landTriggerVal) {
    landTime = millis();
    landDetect = true;
  }
}

void getGPSData() {
  float flat;
  float flon;
  boolean newdata = false;
  for(unsigned long start = millis(); millis() - start < 5000;) {
    while(gpsSerial.available()) {
      if(feedGPS()) {
        newdata = true;
      }
    }
  }
  if(newdata) {
    gps.f_get_position(&flat, &flon);
    gpsLat = flat;
    gpsLon = flon;
    if(landDetect == true) {
      float fdist = gps.distance_between(gpsLat, gpsLon, gpsLatInitial, gpsLonInitial);
      landDistance = fdist * 3.28;
    }
  }
#ifdef verboseSerial
  Serial.println();
  Serial.println(F("GPS coordinates sent."));
  Serial.println();
#endif
}

// Feed data as it becomes available
boolean feedGPS() {
  while(gpsSerial.available()) {
    if(gps.encode(gpsSerial.read()))
      return true;
  }
  return false;
}

void sendExtraData() {
  Serial.print(F("s"));
  Serial.print(gpsLat, 8);
  Serial.print(F("t"));
  Serial.print(gpsLon, 8);
  Serial.print(F("n"));
  if(landDetect == true) {
    Serial.print(landDistance);
    Serial.print(F("d"));
    altFeetMax = (float)altCmMax / 30.48;
    Serial.print(altFeetMax);
    Serial.print(F("a"));
  }
  Serial.println(F("e"));
}

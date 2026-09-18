/*
 * Automatic Distance Alert System
 * ------------------------------------------------------------
 * Hardware : Arduino Uno + HC-SR04 ultrasonic sensor + LED + piezo buzzer
 *
 * Features
 *   - Measures distance to the nearest object in front of the sensor
 *   - Turns an LED + buzzer ON inside a configurable threshold, OFF outside it
 *   - Threshold is configurable at runtime over the Serial Monitor and is
 *     remembered across resets (stored in EEPROM)
 *   - Median-of-3 filtering rejects the stray readings ultrasonic sensors give
 *   - Hysteresis stops the alert from flickering when an object sits exactly
 *     on the threshold
 *   - Buzzer beeps faster as the object gets closer (proximity feedback)
 *   - Fully non-blocking: no delay() in loop(), everything is millis()-timed
 *
 * Serial commands (9600 baud, Newline ending)
 *   T25   or   25    -> set threshold to 25 cm
 *   ?                -> print current status
 *
 * Wiring
 *   HC-SR04 VCC -> 5V      HC-SR04 GND  -> GND
 *   HC-SR04 TRIG-> D9      HC-SR04 ECHO -> D10
 *   Buzzer +    -> D11     Buzzer -     -> GND
 *   LED anode   -> D12 via 220R resistor, LED cathode -> GND
 */

#include <EEPROM.h>

/* ---------------- Pin map ---------------- */
const uint8_t PIN_TRIG   = 9;
const uint8_t PIN_ECHO   = 10;
const uint8_t PIN_BUZZER = 11;
const uint8_t PIN_LED    = 12;

/* ---------------- Tunable constants ---------------- */
const uint16_t MEASURE_INTERVAL_MS = 60;     // how often we ping
const uint16_t PRINT_INTERVAL_MS   = 250;    // how often we print to serial
const uint16_t ECHO_TIMEOUT_US     = 25000;  // ~4.3 m round trip
const float    HYSTERESIS_CM       = 2.0;    // anti-flicker band
const float    MIN_VALID_CM        = 2.0;    // HC-SR04 spec minimum
const float    MAX_VALID_CM        = 400.0;  // HC-SR04 spec maximum
const uint16_t BUZZER_FREQ_HZ      = 2000;
const uint16_t BEEP_MS             = 40;
const uint16_t BEEP_FAST_MS        = 70;     // beep gap at point-blank range
const uint16_t BEEP_SLOW_MS        = 450;    // beep gap at the threshold
const int      DEFAULT_THRESHOLD   = 25;     // cm
const int      EEPROM_ADDR         = 0;

/* ---------------- State ---------------- */
int   thresholdCm = DEFAULT_THRESHOLD;
bool  alertOn     = false;
float distanceCm  = -1.0;                    // -1 means "out of range"

unsigned long tLastMeasure = 0;
unsigned long tLastPrint   = 0;
unsigned long tLastBeep    = 0;

char    cmdBuf[16];
uint8_t cmdLen = 0;

/* ---------------- Setup ---------------- */
void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);

  Serial.begin(9600);

  int saved;
  EEPROM.get(EEPROM_ADDR, saved);
  if (saved >= (int)MIN_VALID_CM && saved <= (int)MAX_VALID_CM) {
    thresholdCm = saved;                     // restore last used threshold
  }

  Serial.println(F("=== Automatic Distance Alert System ==="));
  Serial.println(F("Commands:  T<cm> set threshold   |   ?  status"));
  Serial.print(F("Active threshold: "));
  Serial.print(thresholdCm);
  Serial.println(F(" cm"));
  Serial.println();
}

/* ---------------- Sensor ---------------- */

// One ultrasonic ping. Returns distance in cm, or -1.0 if invalid/out of range.
float pingOnce() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long echoUs = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (echoUs == 0) return -1.0;              // timed out, nothing in range

  // Speed of sound ~343 m/s = 0.0343 cm/us; halve it for the return trip.
  float cm = (echoUs * 0.0343) / 2.0;
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return -1.0;
  return cm;
}

// Median of three pings: one bad sample can no longer move the output.
float readDistance() {
  const float INVALID = MAX_VALID_CM + 1.0;  // sorts to the top
  float s[3];
  for (uint8_t i = 0; i < 3; i++) {
    float v = pingOnce();
    s[i] = (v < 0) ? INVALID : v;
    delayMicroseconds(3000);                 // let the echo die down
  }
  // tiny sorting network for 3 elements
  if (s[0] > s[1]) { float t = s[0]; s[0] = s[1]; s[1] = t; }
  if (s[1] > s[2]) { float t = s[1]; s[1] = s[2]; s[2] = t; }
  if (s[0] > s[1]) { float t = s[0]; s[0] = s[1]; s[1] = t; }

  return (s[1] >= INVALID) ? -1.0 : s[1];
}

/* ---------------- Alert logic ---------------- */

// Hysteresis: trip at the threshold, release only once the object has moved
// HYSTERESIS_CM beyond it. Prevents chattering at the boundary.
void updateAlert() {
  if (distanceCm < 0) {                      // nothing detected
    alertOn = false;
  } else if (!alertOn && distanceCm <= thresholdCm) {
    alertOn = true;
  } else if (alertOn && distanceCm > thresholdCm + HYSTERESIS_CM) {
    alertOn = false;
  }

  digitalWrite(PIN_LED, alertOn ? HIGH : LOW);
}

void updateBuzzer(unsigned long now) {
  if (!alertOn) {
    noTone(PIN_BUZZER);
    return;
  }
  // Closer object -> shorter gap between beeps.
  long gap = map((long)distanceCm,
                 (long)MIN_VALID_CM, (long)thresholdCm,
                 BEEP_FAST_MS, BEEP_SLOW_MS);
  gap = constrain(gap, BEEP_FAST_MS, BEEP_SLOW_MS);

  if (now - tLastBeep >= (unsigned long)gap) {
    tLastBeep = now;
    tone(PIN_BUZZER, BUZZER_FREQ_HZ, BEEP_MS);   // non-blocking
  }
}

/* ---------------- Serial ---------------- */
void printStatus() {
  Serial.print(F("Distance: "));
  if (distanceCm < 0) Serial.print(F("---- "));
  else { Serial.print(distanceCm, 1); Serial.print(F(" cm")); }

  Serial.print(F("  |  Threshold: "));
  Serial.print(thresholdCm);
  Serial.print(F(" cm  |  Status: "));
  Serial.println(alertOn ? F("ALERT - OBJECT TOO CLOSE") : F("clear"));
}

void setThreshold(int cm) {
  if (cm < (int)MIN_VALID_CM || cm > (int)MAX_VALID_CM) {
    Serial.print(F("Rejected: threshold must be 2-400 cm. Got "));
    Serial.println(cm);
    return;
  }
  thresholdCm = cm;
  EEPROM.put(EEPROM_ADDR, thresholdCm);      // .put only writes changed bytes
  Serial.print(F(">> Threshold set to "));
  Serial.print(thresholdCm);
  Serial.println(F(" cm (saved)"));
}

void handleCommand(char *cmd) {
  while (*cmd == ' ') cmd++;
  if (*cmd == '?')                 { printStatus(); return; }
  if (*cmd == 'T' || *cmd == 't')  { setThreshold(atoi(cmd + 1)); return; }
  if (isdigit((unsigned char)*cmd)){ setThreshold(atoi(cmd));     return; }
  Serial.println(F("Unknown command. Use T<cm> or ?"));
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen) { cmdBuf[cmdLen] = '\0'; handleCommand(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

/* ---------------- Main loop (non-blocking) ---------------- */
void loop() {
  unsigned long now = millis();

  readSerial();

  if (now - tLastMeasure >= MEASURE_INTERVAL_MS) {
    tLastMeasure = now;
    distanceCm = readDistance();
    updateAlert();
  }

  updateBuzzer(now);

  if (now - tLastPrint >= PRINT_INTERVAL_MS) {
    tLastPrint = now;
    printStatus();
  }
}

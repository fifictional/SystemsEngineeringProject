#include <Wire.h>

const int PEND_ENC_A_PIN = 2;
const int PEND_ENC_B_PIN = 3;

const float PEND_ENC_PPR = 2400.0f;
const float PEND_RAD_PER_TICK = (2.0f * PI) / PEND_ENC_PPR;

const unsigned long SAMPLE_PERIOD_US = 5000;   // 200 Hz
const unsigned long RECORD_TIME_MS   = 10000;  // 10 seconds

volatile long pendTicks = 0;
long pendOffset = 0;

// ---------------- Encoder ISR ----------------
void PEND_ISR() {
  int A = digitalRead(PEND_ENC_A_PIN);
  int B = digitalRead(PEND_ENC_B_PIN);
  if (A != B) pendTicks++;
  else        pendTicks--;
}

// ---------------- Serial Wait Helper ----------------
void waitForKey() {
  while (!Serial.available()) {}
  delay(50);  // allow buffer to fill
  while (Serial.available()) Serial.read();  // clear buffer
}

// ---------------- Setup ----------------
void setup() {

  Serial.begin(115200);

  pinMode(PEND_ENC_A_PIN, INPUT_PULLUP);
  pinMode(PEND_ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PEND_ENC_A_PIN), PEND_ISR, CHANGE);

  Serial.println("Let pendulum hang STRAIGHT DOWN.");
  Serial.println("Press any key to zero...");
  waitForKey();

  noInterrupts();
  pendOffset = pendTicks;
  interrupts();

  Serial.println("Zeroed.");
  Serial.println("Pull pendulum to ~15 deg and HOLD.");
  Serial.println("Press any key to start (1 second delay before recording)...");
  waitForKey();

  Serial.println("Starting in 1 second...");
  delay(1000);

  Serial.println("time_ms,theta_deg");
}

// ---------------- Loop ----------------
void loop() {

  static bool recordingStarted = false;
  static unsigned long startTimeUs = 0;
  static unsigned long lastSampleUs = 0;

  if (!recordingStarted) {
    startTimeUs = micros();
    lastSampleUs = startTimeUs;
    recordingStarted = true;
  }

  unsigned long nowUs = micros();

  // Stop after fixed time
  if ((nowUs - startTimeUs) >= RECORD_TIME_MS * 1000UL) {
    Serial.println("Recording finished.");
    while (1);  // stop forever
  }

  // Precise 200Hz sampling
  if ((nowUs - lastSampleUs) >= SAMPLE_PERIOD_US) {

    lastSampleUs += SAMPLE_PERIOD_US;  // prevent drift

    noInterrupts();
    long pt = pendTicks - pendOffset;
    interrupts();

    float theta_deg = (float)pt * PEND_RAD_PER_TICK * 180.0f / PI;

    unsigned long time_ms = (nowUs - startTimeUs) / 1000;

    Serial.print(time_ms);
    Serial.print(",");
    Serial.println(theta_deg, 6);
  }
}

#include <Wire.h>
#include <Motoron.h>

// =====================
// Motoron Setup
// =====================
MotoronI2C mc1(0x10);

// =====================
// Encoder Setup
// =====================
volatile long encoderCount = 0;

const int pinA = 3;
const int pinB = 5;

// =====================
// PID Parameters
// =====================
float Kp = 200.0;
float Ki = 0.0;
float Kd = 0.8;

// =====================
// PID Variables
// =====================
float targetAngle = 0;

float integral = 0;
float lastError = 0;

float lastAngle = 0;
float angleVelocity = 0;

// derivative filter strength
const float velocityAlpha = 0.3;

// integral protection
const float integralLimit = 500;

// =====================
// Timing
// =====================
unsigned long lastControl = 0;
const int controlPeriod = 800;   // 1250 Hz control

// =====================
// Encoder Interrupt
// =====================
void handleEncoder()
{
  bool A = digitalRead(pinA);
  bool B = digitalRead(pinB);

  if (A == B)
    encoderCount++;
  else
    encoderCount--;
}

// =====================
// Setup
// =====================
void setup()
{
  Wire1.begin();
  Serial.begin(115200);

  mc1.setBus(&Wire);
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.disableCommandTimeout();
  mc1.clearResetFlag();

  mc1.setMaxAcceleration(1, 800);
  mc1.setMaxDeceleration(1, 800);
  mc1.setMaxAcceleration(2, 800);
  mc1.setMaxDeceleration(2, 800);

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinA), handleEncoder, CHANGE);

  Serial.println("================================");
  Serial.println("Immediate Upright Calibration");
  Serial.println("Place pendulum upright before power on.");
  Serial.println("================================");

  targetAngle = encoderCount;

  Serial.print("Balance Point: ");
  Serial.println(targetAngle);

  lastControl = micros();
}

// =====================
// Main Control Loop
// =====================
void loop()
{
  if (micros() - lastControl < controlPeriod) return;

  unsigned long now = micros();
  float dt = (now - lastControl) / 1000000.0;
  lastControl = now;

  long currentCount = encoderCount;
  float angle = (float)currentCount;

  // =====================
  // Angle velocity estimate
  // =====================
  float rawVelocity = (angle - lastAngle) / dt;

  angleVelocity =
      velocityAlpha * angleVelocity +
      (1 - velocityAlpha) * rawVelocity;

  lastAngle = angle;

  // =====================
  // PID
  // =====================
  float error = targetAngle - angle;

  integral += error * dt;

  if (integral > integralLimit) integral = integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;

  float output =
      -(Kp * error +
        Ki * integral +
        Kd * angleVelocity);

  lastError = error;

  output = constrain(output, -800, 800);

  mc1.setSpeed(1, output);
  mc1.setSpeed(2, output);

  // Debug printing (slow)
  static int printCounter = 0;

  if (printCounter++ > 60)
  {
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" Vel: ");
    Serial.print(angleVelocity);
    Serial.print(" Output: ");
    Serial.println(output);

    printCounter = 0;
  }
}

#include <Wire.h>
#include <Motoron.h>

/*********** Motoron Controller ***********/
MotoronI2C mc1(0x10);

/*********** Timing ***********/
const float dt = 0.01f;   // 100 Hz control loop
unsigned long lastTime = 0;

/*********** Encoder Pins ***********/
// Pendulum encoder
const int PEND_A = 3;
const int PEND_B = 5;

// Cart encoder
const int CART_A = 2;
const int CART_B = 4;

/*********** Encoder States ***********/
volatile long pendCount = 0;
volatile long cartCount = 0;

volatile int pendLastA = 0;
volatile int cartLastA = 0;

/*********** State Variables ***********/
float theta = 0.0f, theta_dot = 0.0f;
float x = 0.0f, x_dot = 0.0f;

float theta_prev = 0.0f;
float x_prev = 0.0f;

/*********** Scaling Constants ***********/
// Pendulum: encoder_counts_per_revolution / (2 * PI)
// Cart: encoder_counts_per_revolution / (wheel_circumference_in_meters)
const float PEND_COUNTS_PER_RAD = 310.7f;  // TODO: CALIBRATE
const float CART_COUNTS_PER_M   = 1070.0f; // TODO: CALIBRATE

/*********** Filter Parameters ***********/
const float ALPHA = 0.4f;  // Low-pass filter coefficient (0-1)

/*********** Safety Limits ***********/
const float THETA_LIMIT = 0.9f;  // radians (~28 degrees)
const float X_LIMIT = 0.40f;     // meters (adjust to your track length)
const int MAX_MOTOR_SPEED = 800;

/*********** Motor Scaling ***********/
const float U_TO_SPEED = 150.0f;   // try 80..300
const int DEAD = 60;               // try 0..150 (set 0 to disable)

/*********** System State ***********/
bool systemActive = false;

/*********** PID Controller (based on your Python class) ***********/
class PID_controller {
public:
  float Kp, Ki, Kd, N;
  float integral;
  float prev_error;
  float prev_filtered_deriv;

  // Anti-windup clamp for integral state
  float integMin, integMax;

  PID_controller(float kp=50.0f, float ki=0.0f, float kd=1.0f, float n=10.0f,
                 float iMin=-10.0f, float iMax=10.0f)
  : Kp(kp), Ki(ki), Kd(kd), N(n),
    integral(0.0f), prev_error(0.0f), prev_filtered_deriv(0.0f),
    integMin(iMin), integMax(iMax) {}

  float step(float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    // Proportional
    float P = Kp * error;

    // Integral with clamp (anti-windup)
    integral += error * dt;
    if (integral > integMax) integral = integMax;
    if (integral < integMin) integral = integMin;
    float I = Ki * integral;

    // Derivative with low-pass filter (matches your Python form)
    float raw_derivative = (error - prev_error) / dt;
    float filtered_derivative = (N * raw_derivative + prev_filtered_deriv) / (1.0f + N);
    float D = Kd * filtered_derivative;

    // Update state
    prev_error = error;
    prev_filtered_deriv = filtered_derivative;

    return P + I + D;
  }

  void reset() {
    integral = 0.0f;
    prev_error = 0.0f;
    prev_filtered_deriv = 0.0f;
  }
};

/*********** PID Instance (matches your demo) ***********/
// Kp=55, Ki=0.001, Kd=1.5, N=10
PID_controller pid(55.0f, 0.001f, 1.5f, 10.0f, -10.0f, 10.0f);

/*********** Encoder ISRs ***********/
void pendISR() {
  int A = digitalRead(PEND_A);
  int B = digitalRead(PEND_B);

  if (A != pendLastA) {
    if (B != A) pendCount++;
    else        pendCount--;
    pendLastA = A;
  }
}

void cartISR() {
  int A = digitalRead(CART_A);
  int B = digitalRead(CART_B);

  if (A != cartLastA) {
    if (B != A) cartCount++;
    else        cartCount--;
    cartLastA = A;
  }
}

/*********** Motor Control ***********/
void drive(float u) {
  // Scale PID command into Motoron speed units
  int speed = (int)(-u * U_TO_SPEED);

  // Optional deadband compensation
  if (DEAD > 0) {
    if (speed > 0 && speed < DEAD) speed = DEAD;
    if (speed < 0 && speed > -DEAD) speed = -DEAD;
  }

  // Constrain to safe motor speed range
  speed = constrain(speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

  // Drive both motors at same speed
  mc1.setSpeed(1, speed);
  mc1.setSpeed(2, speed);
}

/*********** Safety Check ***********/
bool checkSafety() {
  if (abs(theta) > THETA_LIMIT) {
    Serial.println("ERROR: Pendulum angle limit exceeded");
    return false;
  }

  if (abs(x) > X_LIMIT) {
    Serial.println("ERROR: Cart position limit exceeded");
    return false;
  }

  return true;
}

/*********** Emergency Stop ***********/
void emergencyStop() {
  drive(0.0f);
  systemActive = false;
  pid.reset();
  Serial.println("EMERGENCY STOP ACTIVATED");
}

/*********** Setup ***********/
void setup() {
  Wire.begin();
  Serial.begin(115200);

  Serial.println("=== Inverted Pendulum PID Controller (no Kalman yet) ===");
  Serial.println("Initializing...");

  // Initialize Motoron controller
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();

  // Configure encoder pins
  pinMode(PEND_A, INPUT_PULLUP);
  pinMode(PEND_B, INPUT_PULLUP);
  pinMode(CART_A, INPUT_PULLUP);
  pinMode(CART_B, INPUT_PULLUP);

  // Read initial encoder states
  pendLastA = digitalRead(PEND_A);
  cartLastA = digitalRead(CART_A);

  // Attach interrupts (A channel only, as in your original)
  attachInterrupt(digitalPinToInterrupt(PEND_A), pendISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CART_A), cartISR, CHANGE);

  Serial.println("Encoders configured");

  // Wait for user to position pendulum upright
  Serial.println("\n*** MOVE PENDULUM TO UPRIGHT POSITION ***");
  Serial.println("Starting in 5 seconds...");
  delay(5000);

  // Reset encoder counts at upright position
  noInterrupts();
  pendCount = 0;
  cartCount = 0;
  interrupts();

  // Initialize state variables
  theta = 0.0f;
  x = 0.0f;
  theta_dot = 0.0f;
  x_dot = 0.0f;
  theta_prev = 0.0f;
  x_prev = 0.0f;

  pid.reset();

  systemActive = true;
  lastTime = millis();

  Serial.println("System ACTIVE");
  Serial.println("x,x_dot,theta,theta_dot,u");
}

/*********** Main Control Loop ***********/
void loop() {
  unsigned long now = millis();

  // Execute control loop at fixed frequency (~100 Hz)
  if (now - lastTime >= 10) {  // 10 ms
    lastTime += 10;

    // Read encoder counts atomically
    noInterrupts();
    long pCnt = pendCount;
    long cCnt = cartCount;
    interrupts();

    // Convert counts to physical units
    theta = (float)pCnt / PEND_COUNTS_PER_RAD; // rad
    x     = (float)cCnt / CART_COUNTS_PER_M;   // m

    // Velocity estimates with low-pass filter
    float theta_dot_raw = (theta - theta_prev) / dt;
    float x_dot_raw     = (x - x_prev) / dt;

    theta_dot = ALPHA * theta_dot_raw + (1.0f - ALPHA) * theta_dot;
    x_dot     = ALPHA * x_dot_raw     + (1.0f - ALPHA) * x_dot;

    theta_prev = theta;
    x_prev = x;

    // Safety check
    if (!checkSafety()) {
      emergencyStop();
      return;
    }

    // PID control (matches your Python demo structure)
    const float theta_ref = 0.0f;

    // In your Python: u = -pid.step(theta_ref, theta_hat, dt)
    float u = -pid.step(theta_ref, theta, dt);

    // Apply control signal
    if (systemActive) {
      drive(u);
    } else {
      drive(0.0f);
    }

    // Telemetry (CSV)
    Serial.print(x, 4);          Serial.print(",");
    Serial.print(x_dot, 4);      Serial.print(",");
    Serial.print(theta, 4);      Serial.print(",");
    Serial.print(theta_dot, 4);  Serial.print(",");
    Serial.println(u, 3);
  }

  // Serial commands: 's' stop, 'r' resume
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      emergencyStop();
    } else if (cmd == 'r' || cmd == 'R') {
      pid.reset();
      systemActive = true;
      Serial.println("System RESUMED");
    }
  }
}

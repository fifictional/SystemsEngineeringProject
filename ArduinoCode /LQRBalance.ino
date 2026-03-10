#include <Wire.h>
#include <Motoron.h>

/*********** Motoron Controller ***********/
MotoronI2C mc1(0x10);

/*********** LQR Gain Matrix ***********/
// Computed from Python LQR controller
// State vector: [x, x_dot, theta, theta_dot]
const float K[4] = {
2.23606798, // x gain
3.81322011, // x_dot gain
40.89839879, // theta gain
6.36709747 // theta_dot gain
};

/*********** Timing ***********/
const float dt = 0.01; // 100 Hz control loop
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
float theta = 0, theta_dot = 0;
float x = 0, x_dot = 0;

float theta_prev = 0;
float x_prev = 0;

/*********** Scaling Constants ***********/
// IMPORTANT: Calibrate these values based on your hardware!
// Pendulum: encoder_counts_per_revolution / (2 * PI)
// Cart: encoder_counts_per_revolution / (wheel_circumference_in_meters)
const float PEND_COUNTS_PER_RAD = 310.7; // TODO: CALIBRATE
const float CART_COUNTS_PER_M = 1070.0; // TODO: CALIBRATE

/*********** Safety Limits ***********/
const float THETA_LIMIT = 0.9; // radians (~28 degrees)
const float X_LIMIT = 3; // meters
const int MAX_MOTOR_SPEED = 800;

/*********** Filter Parameters ***********/
const float ALPHA = 0.4; // Low-pass filter coefficient (0-1)

/*********** Motor Scaling (NEW) ***********/
// Converts LQR output "u" into Motoron speed units.
// Start here, then tune until it moves smoothly without slamming.
const float U_TO_SPEED = 240.0; // try 80..300
const int DEAD = 90; // try 0..150 (set 0 to disable)

/*********** System State ***********/
bool systemActive = false;

/*********** Encoder ISRs ***********/
void pendISR() {
int A = digitalRead(PEND_A);
int B = digitalRead(PEND_B);

if (A != pendLastA) {
if (B != A) pendCount++;
else pendCount--;
pendLastA = A;
}
}

void cartISR() {
int A = digitalRead(CART_A);
int B = digitalRead(CART_B);

if (A != cartLastA) {
if (B != A) cartCount++;
else cartCount--;
cartLastA = A;
}
}

/*********** Motor Control ***********/
void drive(float u) {
// Scale LQR command into Motoron speed units
int speed = (int)(u * U_TO_SPEED);

// Optional deadband compensation so small commands still move the cart
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
drive(0);
systemActive = false;
Serial.println("EMERGENCY STOP ACTIVATED");
}

/*********** Setup ***********/
void setup() {
Wire.begin();
Serial.begin(115200);

Serial.println("=== Inverted Pendulum LQR Controller ===");
Serial.println("Initializing...");

// Initialize Motoron controller
mc1.reinitialize();
mc1.disableCrc();
mc1.clearResetFlag();

Serial.println("Motor controller initialized");

// Configure encoder pins
pinMode(PEND_A, INPUT_PULLUP);
pinMode(PEND_B, INPUT_PULLUP);
pinMode(CART_A, INPUT_PULLUP);
pinMode(CART_B, INPUT_PULLUP);

// Read initial encoder states
pendLastA = digitalRead(PEND_A);
cartLastA = digitalRead(CART_A);

// Attach interrupt handlers
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
theta = 0;
x = 0;
theta_dot = 0;
x_dot = 0;
theta_prev = 0;
x_prev = 0;

systemActive = true;
lastTime = millis();

Serial.println("System ACTIVE");
Serial.println("x,x_dot,theta,theta_dot,u");
}

/*********** Main Control Loop ***********/
void loop() {
unsigned long now = millis();

// Execute control loop at fixed frequency
if (now - lastTime >= dt * 1000) {
lastTime = now;

// Read encoder counts atomically
noInterrupts();
long pCnt = pendCount;
long cCnt = cartCount;
interrupts();

// Convert counts to physical units
theta = pCnt / PEND_COUNTS_PER_RAD;
x = cCnt / CART_COUNTS_PER_M;

// Compute velocities with low-pass filter to reduce noise
float theta_dot_raw = (theta - theta_prev) / dt;
float x_dot_raw = (x - x_prev) / dt;

theta_dot = ALPHA * theta_dot_raw + (1 - ALPHA) * theta_dot;
x_dot = ALPHA * x_dot_raw + (1 - ALPHA) * x_dot;

// Store previous values for next iteration
theta_prev = theta;
x_prev = x;

// Safety check
if (!checkSafety()) {
emergencyStop();
return;
}

// Compute LQR control law: u = -K * state
float u = - (K[0] * x
+ K[1] * x_dot
+ K[2] * theta
+ K[3] * theta_dot);

// Apply control signal
if (systemActive) {
drive(u);
}

// Output telemetry (CSV format)
Serial.print(x, 4); Serial.print(",");
Serial.print(x_dot, 4); Serial.print(",");
Serial.print(theta, 4); Serial.print(",");
Serial.print(theta_dot, 4); Serial.print(",");
Serial.println(u, 2);
}

// Check for emergency stop command via serial
if (Serial.available() > 0) {
char cmd = Serial.read();
if (cmd == 's' || cmd == 'S') {
emergencyStop();
} else if (cmd == 'r' || cmd == 'R') {
systemActive = true;
Serial.println("System RESUMED");
}
}
}

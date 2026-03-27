#include <Wire.h>
#include <Motoron.h>

/*********** Motoron Controller ***********/
MotoronI2C mc1(0x10);

/*********** LQR Gain Matrix ***********/
const float K[4] = {
-8.23606798,
-5.81322011,
58.89839879,
12.36709747
};

/*********** Target Position ***********/
float x_target = 2; // Default to 0, can be changed for sprint
const float T_move = 15.0; // Slower movement for small angles
float Ti = 0.0;

/*********** Timing ***********/
const float dt = 0.002;
unsigned long lastTime = 0;

/*********** Encoder Pins ***********/
const int PEND_A = 3;
const int PEND_B = 5;

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

/*********** Scaling ***********/
const float PEND_COUNTS_PER_RAD = 310.7;
const float CART_COUNTS_PER_M = 2667.0;

/*********** Safety ***********/
const float THETA_LIMIT = 0.9;
const float X_LIMIT = 5;
const int MAX_MOTOR_SPEED = 800;

/*********** Motor Scaling ***********/
const float U_TO_SPEED = 400.0;

/*********** DEAD ZONE ***********/
const int DEADZONE_FWD = 50; // Pushing Right
const int DEADZONE_REV = 50; // Pushing Left

// ... inside your drive(float u) function:
void drive(float u) {
int speed = (int)(u * U_TO_SPEED);
if (speed > 0) {
speed += DEADZONE_FWD;
} else if (speed < 0) {
speed -= DEADZONE_REV;
}
speed = constrain(speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
mc1.setSpeed(2, speed);
mc1.setSpeed(3, speed);
}

/*********** System State ***********/
bool systemActive = false;

/*********** Trajectory Helpers ***********/
float smoothstep(float s) {
return 3.0*s*s - 2.0*s*s*s;
}

float smoothstep_dot(float s, float T) {
return (6.0 * s * (1.0 - s)) / T;
}

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

/*********** Stop ***********/
void stopMotors() {
mc1.setSpeed(2, 0);
mc1.setSpeed(3, 0);
}

/*********** Sprint Initialization ***********/
void startSprint(float distance) {
if (systemActive && abs(theta) < 0.1 && abs(x_dot) < 0.01) {
x_target = distance;
Ti = 0.0; // Reset trajectory timer
Serial.print("Starting sprint to ");
Serial.print(distance);
Serial.println(" meters");
}
}

/*********** Safety ***********/
bool checkSafety() {
if (abs(theta) > THETA_LIMIT) return false;
if (abs(x) > X_LIMIT) return false;
return true;
}

/*********** Setup ***********/
void setup() {
Serial.begin(115200);
Wire.begin();

mc1.setBus(&Wire);
mc1.reinitialize();
mc1.disableCrc();
mc1.disableCommandTimeout();
mc1.clearResetFlag();

mc1.setPwmMode(2, 1);
mc1.setPwmMode(3, 1);

pinMode(PEND_A, INPUT_PULLUP);
pinMode(PEND_B, INPUT_PULLUP);
pinMode(CART_A, INPUT_PULLUP);
pinMode(CART_B, INPUT_PULLUP);

pendLastA = digitalRead(PEND_A);
cartLastA = digitalRead(CART_A);

attachInterrupt(digitalPinToInterrupt(PEND_A), pendISR, CHANGE);
attachInterrupt(digitalPinToInterrupt(CART_A), cartISR, CHANGE);

Serial.println("Place pendulum upright...");
delay(3000);

noInterrupts();
pendCount = 0;
cartCount = 0;
interrupts();

theta = 0; x = 0;
theta_dot = 0; x_dot = 0;
theta_prev = 0; x_prev = 0;
Ti = 0.0;

systemActive = true;
lastTime = micros();
}

/*********** Loop ***********/
void loop() {

if (!systemActive) {
stopMotors();
return;
}

if (micros() - lastTime < (unsigned long)(dt * 1000000)) return;
lastTime = micros();

noInterrupts();
long pCnt = pendCount;
long cCnt = cartCount;
interrupts();

theta = pCnt / PEND_COUNTS_PER_RAD;
x = cCnt / CART_COUNTS_PER_M;

theta_dot = (theta - theta_prev) / dt;
x_dot = (x - x_prev) / dt;

theta_prev = theta;
x_prev = x;

if (!checkSafety()) {
systemActive = false;
stopMotors();
return;
}

// Check for serial command to start sprint
if (Serial.available()) {
float dist = Serial.parseFloat();
if (dist > 0 && dist <= 3.0) {
startSprint(dist);
}
}

Ti += dt;

float x_ref, xdot_ref;

if (Ti < T_move) {
float s = Ti / T_move;
x_ref = x_target * smoothstep(s);
xdot_ref = x_target * smoothstep_dot(s, T_move);
} else {
x_ref = x_target;
xdot_ref = 0.0;
}

float u =
K[0] * (x - x_ref) +
K[1] * (x_dot - xdot_ref) +
K[2] * theta +
K[3] * theta_dot;

drive(u);

static int pc = 0;
if (pc++ > 80) {
Serial.print(theta); Serial.print(" ");
Serial.print(theta_dot); Serial.print(" ");
Serial.println(u);
pc = 0;
}
}

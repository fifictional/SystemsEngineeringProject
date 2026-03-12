#include <Motoron.h>

// make a motoron object for each motor shield w/ the default adresses
MotoronI2C mc1(0x10); // lower motor shield: M1 - left front; M2 - right front; M3 - middle left

void setup() {
  Wire1.begin();  // initialise I2C
  Serial.begin(9600);

  // initialize motoron controller
  mc1.reinitialize();    // reset the controller
  mc1.disableCrc();      // disable CRC check
  mc1.clearResetFlag();  // clear reset flag
}

// move forward: all motors rotate forward
void moveForward(int speed) {
  mc1.setSpeed(1, speed);  // front left
  mc1.setSpeed(2, speed);  // front right
}

// move backward: all motors rotate in reverse
void moveBackward(int speed) {
  mc1.setSpeed(1, -speed);
  mc1.setSpeed(2, -speed);
}

// turn left: front wheels rotate in opposite directions, rear wheel stops
void turnLeft(int speed) {
  mc1.setSpeed(1, -speed);  // front left wheel reverse
  mc1.setSpeed(2, speed);   // front right wheel forward
}

// turn right: front wheels rotate in opposite directions, rear wheel stops
void turnRight(int speed) {
  mc1.setSpeed(1, speed);   // front left wheel forward
  mc1.setSpeed(2, -speed);  // front right wheel reverse
}

// stop all wheels
void stopMotors() {
  mc1.setSpeed(1, 0);
  mc1.setSpeed(2, 0);
}

void loop() {

  // test forward
  moveForward(700);
  delay(1000);

  // stopMotors();
  // delay(2000);

  // // test backward
  // moveBackward(700);
  // delay(1000);

  // stopMotors();
  // delay(2000);

  // // test turning left
  // turnLeft(700);
  // delay(1000);

  // stopMotors();
  // delay(2000);

  // // test turning right
  // turnRight(700);
  // delay(1000);

  // stopMotors();
  // delay(2000);
}


#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc1(0x10);

/*********** Encoder Pins ***********/
const int ENC_A = 2;
const int ENC_B = 4;

/*********** Encoder State ***********/
volatile long count = 0;
volatile int lastA = 0;

/*********** ISR ***********/
void encoderISR()
{
  int A = digitalRead(ENC_A);
  int B = digitalRead(ENC_B);

  if (A != lastA)
  {
    if (B != A) count++;
    else count--;

    lastA = A;
  }
}

void setup()
{
  Serial.begin(115200);

  Wire.begin();
  mc1.setBus(&Wire);

  mc1.reinitialize();
  mc1.disableCrc();
  mc1.disableCommandTimeout();
  mc1.clearResetFlag();

  mc1.setPwmMode(1,1);  // important

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  lastA = digitalRead(ENC_A);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  Serial.println("Motor encoder test");
}

void loop()
{
  // run motor forward slowly
  mc1.setSpeed(1, 400);
  mc1.setSpeed(2, 400);

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint > 200)
  {
    lastPrint = millis();

    noInterrupts();
    long c = count;
    interrupts();

    Serial.print("Count: ");
    Serial.println(c);
  }
}

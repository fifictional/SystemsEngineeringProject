const int pinA = 3;    // Encoder A
const int pinB = 5;    // Encoder B

volatile long encoderPos = 0;
volatile int lastA = 0;

void onAchange() {
  int A = digitalRead(pinA);
  int B = digitalRead(pinB);

  if (A != lastA) {
    if (B != A) {
      encoderPos++;
    } else {
      encoderPos--;
    }
    lastA = A;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  lastA = digitalRead(pinA);

  attachInterrupt(digitalPinToInterrupt(pinA), onAchange, CHANGE);

  Serial.println("Encoder test start");
}

void loop() {
  static long lastPrint = 0;

  if (millis() - lastPrint > 200) {
    noInterrupts();
    long pos = encoderPos;
    interrupts();

    Serial.print("Encoder count: ");
    Serial.println(pos);

    lastPrint = millis();
  }
}

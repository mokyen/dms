const int ENCODER_A_PIN = 2;
const int ENCODER_B_PIN = 3;

volatile long encoderCount = 0;

void encoderA_ISR() {
  // Called on any change of A
  bool a = digitalRead(ENCODER_A_PIN);
  bool b = digitalRead(ENCODER_B_PIN);
  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void encoderB_ISR() {
  // Called on any change of B
  bool a = digitalRead(ENCODER_A_PIN);
  bool b = digitalRead(ENCODER_B_PIN);
  if (a != b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderB_ISR, CHANGE);

  Serial.println(F("Encoder test ready."));
}

void loop() {
  int a = digitalRead(ENCODER_A_PIN);
  int b = digitalRead(ENCODER_B_PIN);

  Serial.print(F("A: ")); Serial.print(a);
  Serial.print(F("  B: ")); Serial.print(b);
  Serial.print(F("  Count: ")); Serial.println(encoderCount);

  delay(200);
}
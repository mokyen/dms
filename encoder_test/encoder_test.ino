const int ENCODER_A_PIN = 4;
const int ENCODER_B_PIN = 3;

long encoderCount = 0;
uint8_t prevAB = 0;

const int8_t enc_table[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  prevAB = (digitalRead(ENCODER_A_PIN) << 1) | digitalRead(ENCODER_B_PIN);
  Serial.println(F("Polling encoder test ready."));
}

void loop() {
  uint8_t a = digitalRead(ENCODER_A_PIN);
  uint8_t b = digitalRead(ENCODER_B_PIN);
  uint8_t ab = (a << 1) | b;

  uint8_t idx = (prevAB << 2) | ab;
  encoderCount += enc_table[idx];
  prevAB = ab;

  Serial.print(F("A: ")); Serial.print(a);
  Serial.print(F("  B: ")); Serial.print(b);
  Serial.print(F("  Count: ")); Serial.println(encoderCount);

  delay(200);
}

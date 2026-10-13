const int PUL1 = 18, DIR1 = 19, ENA1 = 21;
const int PUL2 = 25, DIR2 = 26, ENA2 = 27;

const int STEP_DELAY_US = 200;      // half-period of each pulse (smaller = faster)
const unsigned long RUN_MS = 5000;  // 5 seconds

void runBoth(bool clockwise) {
  digitalWrite(DIR1, clockwise ? HIGH : LOW);
  digitalWrite(DIR2, clockwise ? HIGH : LOW);
  delay(1);  // let the DIR signal settle

  unsigned long start = millis();
  while (millis() - start < RUN_MS) {
    digitalWrite(PUL1, HIGH);
    digitalWrite(PUL2, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(PUL1, LOW);
    digitalWrite(PUL2, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void setup() {
  int pins[] = {PUL1, DIR1, ENA1, PUL2, DIR2, ENA2};
  for (int p : pins) pinMode(p, OUTPUT);
  digitalWrite(ENA1, LOW);  // enabled (common anode)
  digitalWrite(ENA2, LOW);
}

void loop() {
  runBoth(true);   // clockwise 5 s
  runBoth(false);  // anticlockwise 5 s
}
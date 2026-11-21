#include "HX711.h"

// ---- Pins ----
// Index order everywhere: 0 = RT, 1 = LT, 2 = RB, 3 = LB
const uint8_t HX_DT[4]  = {16, 18, 25, 32};
const uint8_t HX_SCK[4] = {17, 19, 26, 33};
const uint8_t BUZZER = 27, LED_RED = 4, LED_GREEN = 5;

// ---- Calibration (per channel, MUST be measured) ----
float scaleF[4] = {420.0f, 420.0f, 420.0f, 420.0f};   // counts per gram, placeholder

// ---- Geometry (mm): origin at left-bottom, +x right, +y top ----
const float l = 300.0;
const float b = 200.0;

// ---- Thresholds (placeholders until real hardware) ----
const float tolX = 0.10;
const float tolY = 0.10;
const float minTotal = 500.0;
const float emptyWeight = 5000.0;
const float emptyRatio = 0.35;
const float cellMax = 5000.0;

// ---- Result of one evaluation ----
struct Result {
  float F_net;
  float cogX, cogY;
  float dx, dy;
  bool tooLight;
  const char* dirV;
  const char* dirH;
  float payloadLimit;
  bool overload;
  bool cellOver[4];               // RT, LT, RB, LB
  bool anyCellOver;
};

// ---- Logic layer (unchanged) ----
Result evaluate(float F_RT, float F_LT, float F_RB, float F_LB) {
  Result r;

  r.F_net = F_RT + F_LT + F_RB + F_LB;

  float centreX = l / 2.0;
  float centreY = b / 2.0;

  if (r.F_net > 0.0) {
    r.cogX = (F_RT * l + F_RB * l) / r.F_net;
    r.cogY = (F_RT * b + F_LT * b) / r.F_net;
  } else {
    r.cogX = centreX;
    r.cogY = centreY;
  }

  r.dx = r.cogX - centreX;
  r.dy = r.cogY - centreY;
  float limitX = tolX * (l / 2.0);
  float limitY = tolY * (b / 2.0);

  r.tooLight = (r.F_net < minTotal);
  r.dirV = "";
  r.dirH = "";
  if (!r.tooLight) {
    if (r.dy > limitY)       r.dirV = "TOP";
    else if (r.dy < -limitY) r.dirV = "BOTTOM";
    if (r.dx > limitX)       r.dirH = "RIGHT";
    else if (r.dx < -limitX) r.dirH = "LEFT";
  }

  float gvw = emptyWeight / emptyRatio;
  r.payloadLimit = gvw - emptyWeight;
  if (r.payloadLimit > 4.0 * cellMax) r.payloadLimit = 4.0 * cellMax;
  r.overload = (r.F_net > r.payloadLimit);

  r.cellOver[0] = (F_RT > cellMax);
  r.cellOver[1] = (F_LT > cellMax);
  r.cellOver[2] = (F_RB > cellMax);
  r.cellOver[3] = (F_LB > cellMax);
  r.anyCellOver = r.cellOver[0] || r.cellOver[1] || r.cellOver[2] || r.cellOver[3];

  return r;
}

// ---- Input layer ----
HX711 hx[4];
long  offsetRaw[4];
float rawEma[4];
unsigned long lastReadMs[4];
bool cellFault[4] = {false, false, false, false};
const unsigned long SENSOR_TIMEOUT_MS = 1000;

// Returns false if any HX711 doesn't answer (read() would hang otherwise)
bool initCells() {
  const char* names[4] = {"RT", "LT", "RB", "LB"};
  bool ok = true;
  for (int i = 0; i < 4; i++) {
    hx[i].begin(HX_DT[i], HX_SCK[i]);
    if (!hx[i].wait_ready_timeout(1000)) {
      Serial.printf("ERROR: HX711 %s not responding (DT=%d SCK=%d)\n",
                    names[i], HX_DT[i], HX_SCK[i]);
      ok = false;
    }
  }
  if (!ok) return false;

  Serial.println("Keep the platform EMPTY and still...");
  delay(2000);
  for (int i = 0; i < 4; i++) {
    offsetRaw[i] = hx[i].read_average(10);
    rawEma[i] = (float)offsetRaw[i];
    lastReadMs[i] = millis();
  }
  return true;
}

// Non-blocking: only reads a cell if it has a fresh sample
bool getForces(float f[4]) {
  unsigned long now = millis();
  bool allOk = true;

  for (int i = 0; i < 4; i++) {
    if (hx[i].is_ready()) {
      long raw = hx[i].read();
      rawEma[i] += 0.3f * ((float)raw - rawEma[i]);
      lastReadMs[i] = now;
      cellFault[i] = false;
    } else if (now - lastReadMs[i] > SENSOR_TIMEOUT_MS) {
      cellFault[i] = true;
      allOk = false;
    }
    f[i] = (rawEma[i] - (float)offsetRaw[i]) / scaleF[i];
  }
  return allOk;
}

// ---- Output layer ----
void setOutputs(const Result& r) {
  unsigned long now = millis();
  bool alarm = r.overload || r.anyCellOver;
  bool warn  = (r.dirV[0] != '\0' || r.dirH[0] != '\0');

  if (alarm) {                         // red solid + buzzer pulsing
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(BUZZER, (now / 150) % 2);
  } else if (warn) {                   // red blinking
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, (now / 500) % 2);
    digitalWrite(BUZZER, LOW);
  } else {                             // green (balanced, or too light to judge)
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
    digitalWrite(BUZZER, LOW);
  }
}

void printResult(const Result& r) {
  Serial.print("F_net (g): ");
  Serial.println(r.F_net);
  Serial.print("CoG (mm): ");
  Serial.print(r.cogX);
  Serial.print(", ");
  Serial.println(r.cogY);
  Serial.print("Offset dx, dy (mm): ");
  Serial.print(r.dx);
  Serial.print(", ");
  Serial.println(r.dy);

  if (r.tooLight) {
    Serial.println("Too light to judge");
  } else if (r.dirV[0] == '\0' && r.dirH[0] == '\0') {
    Serial.println("Balanced");
  } else {
    Serial.print("Shifted: ");
    Serial.print(r.dirV);
    if (r.dirV[0] != '\0' && r.dirH[0] != '\0') Serial.print("-");
    Serial.println(r.dirH);
  }

  Serial.print("Payload limit (g): ");
  Serial.println(r.payloadLimit);
  if (r.overload) Serial.println("OVERLOAD");

  if (r.anyCellOver) {
    Serial.print("CELL OVERLOAD:");
    if (r.cellOver[0]) Serial.print(" RT");
    if (r.cellOver[1]) Serial.print(" LT");
    if (r.cellOver[2]) Serial.print(" RB");
    if (r.cellOver[3]) Serial.print(" LB");
    Serial.println();
  }
  Serial.println("----------------");
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // Startup test: both LEDs + buzzer on for 300 ms
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(BUZZER, HIGH);
  delay(300);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(BUZZER, LOW);

  if (!initCells()) {
    while (true) {                     // blink red forever if a sensor is missing
      digitalWrite(LED_RED, HIGH); delay(200);
      digitalWrite(LED_RED, LOW);  delay(200);
    }
  }
  Serial.println("Running.");
}

void loop() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  float f[4];                          // RT, LT, RB, LB in grams
  bool sensorsOk = getForces(f);

  if (!sensorsOk) {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, (now / 150) % 2);   // fast red blink = sensor fault
    digitalWrite(BUZZER, (now / 150) % 2);
    if (now - lastPrint >= 500) {
      lastPrint = now;
      const char* names[4] = {"RT", "LT", "RB", "LB"};
      Serial.print("SENSOR FAULT:");
      for (int i = 0; i < 4; i++) {
        if (cellFault[i]) Serial.printf(" %s", names[i]);
      }
      Serial.println();
    }
    return;
  }

  Result r = evaluate(f[0], f[1], f[2], f[3]);
  setOutputs(r);

  if (now - lastPrint >= 500) {
    lastPrint = now;
    printResult(r);
  }
}
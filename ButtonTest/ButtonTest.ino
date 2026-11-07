#include <dummy.h>

// Button + joystick test for ESP32. Open Serial Monitor at 115200.
// Press every button once; the sketch tracks which are confirmed.
// Send 's' in the Serial Monitor to see the current checklist.

const int JOY_X = 34, JOY_Y = 35, JOY_SW = 16;
const int BUZZER_PIN = 2;

const int buttonPins[] = {13, 14, 25, 26, 27, 32, 33, 17, 18, 19, 23, 4, 36, 15, 5};
const char* buttonNames[] = {"A", "B", "X", "Y", "L", "ZL", "R", "ZR",
                             "UP", "DOWN", "LEFT", "RIGHT", "HOME", "START", "BACK"};
const int N = sizeof(buttonPins) / sizeof(buttonPins[0]);

bool raw[N], stable[N], tested[N];
unsigned long changed[N];
bool swRaw = false, swStable = false, swTested = false;
unsigned long swChanged = 0;
bool joyPrint = false;  

int xMin = 4095, xMax = 0, yMin = 4095, yMax = 0;
unsigned long lastJoyPrint = 0;

void printChecklist() {
  int done = swTested ? 1 : 0;
  for (int i = 0; i < N; i++) if (tested[i]) done++;
  Serial.printf("\n--- %d/%d tested ---\n", done, N + 1);
  Serial.print("Still untested: ");
  bool any = false;
  for (int i = 0; i < N; i++) if (!tested[i]) { Serial.print(buttonNames[i]); Serial.print(" "); any = true; }
  if (!swTested) { Serial.print("JOYSTICK_SW "); any = true; }
  if (!any) Serial.print("none, all buttons work!");
  Serial.println("\n");
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== BUTTON TEST ===");

  for (int i = 0; i < N; i++) {
    pinMode(buttonPins[i], buttonPins[i] >= 34 ? INPUT : INPUT_PULLUP);  // GPIO 34-39 have no internal pull-up
    raw[i] = stable[i] = (digitalRead(buttonPins[i]) == LOW);
    changed[i] = millis();
    tested[i] = false;
    if (stable[i]) Serial.printf("WARNING: %s (GPIO %d) reads pressed at boot. Short to GND or stuck?\n",
                                 buttonNames[i], buttonPins[i]);
  }
  pinMode(JOY_SW, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  printChecklist();
}

void loop() {
  unsigned long now = millis();

  for (int i = 0; i < N; i++) {
    bool p = (digitalRead(buttonPins[i]) == LOW);
    if (p != raw[i]) { raw[i] = p; changed[i] = now; }
    if (raw[i] != stable[i] && now - changed[i] >= 15) {
      stable[i] = raw[i];
      Serial.printf("%-6s (GPIO %2d) %s\n", buttonNames[i], buttonPins[i],
                    stable[i] ? "PRESSED" : "released");
      if (stable[i]) {
        digitalWrite(BUZZER_PIN, HIGH); delay(15); digitalWrite(BUZZER_PIN, LOW);
        if (!tested[i]) { tested[i] = true; printChecklist(); }
      }
    }
  }

  bool sp = (digitalRead(JOY_SW) == LOW);
  if (sp != swRaw) { swRaw = sp; swChanged = now; }
  if (swRaw != swStable && now - swChanged >= 15) {
    swStable = swRaw;
    Serial.println(swStable ? "JOYSTICK_SW PRESSED" : "JOYSTICK_SW released");
    if (swStable && !swTested) { swTested = true; printChecklist(); }
  }

  // Joystick: print every 300 ms
  int x = analogRead(JOY_X), y = analogRead(JOY_Y);
  xMin = min(xMin, x); xMax = max(xMax, x);
  yMin = min(yMin, y); yMax = max(yMax, y);
    if (joyPrint && now - lastJoyPrint >= 300) {
    lastJoyPrint = now;
    Serial.printf("Joystick X=%4d (min %4d max %4d)  Y=%4d (min %4d max %4d)\n",
                  x, xMin, xMax, y, yMin, yMax);
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') printChecklist();
    if (c == 'j') joyPrint = !joyPrint;
  }
}
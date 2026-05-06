#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// NeoPixel
#define NEO_PIN    D6
#define NEO_COUNT  7
Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

// RF - tre moduler
const int PIN_X = A0;
const int PIN_Y = A1;
const int PIN_Z = A2;

const float SLOPE     = 0.0228f;
const float INTERCEPT = 0.4f;
const float EMA_ALPHA = 0.2f;

// EMA og last per akse
float emaX = -55.0f, emaY = -55.0f, emaZ = -55.0f;
float lastX = -55.0f, lastY = -55.0f, lastZ = -55.0f;

float baselineX = 0.0f, baselineY = 0.0f, baselineZ = 0.0f;

// Eksponeringsgrænser i dBm (til farveskala og bjælke)
// Justér disse efter dit miljø
const float EXP_LOW  = -60.0f; // Grøn — lav eksponering
const float EXP_HIGH = -20.0f; // Rød  — høj eksponering

// Knapper
const int BTN_SCREEN = PC13;
int screen = 0;

// ---- Læs én akse ----
float readAxis(int pin, float &ema, float &last) {
  int numSamples = (ema < -45.0f) ? 128 : 64;
  long sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  float raw   = sum / (float)numSamples;
  float volts = raw * 3.3f / 4095.0f;
  float dBm   = (INTERCEPT - volts) / SLOPE;
  if (dBm < -70.0f) dBm = -70.0f;

  ema = EMA_ALPHA * dBm + (1.0f - EMA_ALPHA) * ema;

  if (abs(ema - last) > 20.0f) {
    return last;
  }

  last = ema;
  return ema;
}

// ---- Kombiner tre akser til samlet eksponering ----
float combineAxes(float dX, float dY, float dZ) {
  float linX = pow(10.0f, dX / 10.0f);
  float linY = pow(10.0f, dY / 10.0f);
  float linZ = pow(10.0f, dZ / 10.0f);
  return 10.0f * log10(linX + linY + linZ);
}

// ---- Sæt baseline ----
void setBaseline() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Saetter baseline");
  lcd.setCursor(0, 1);
  lcd.print("Vent venligst...");

  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 10; i++) {
    sumX += readAxis(PIN_X, emaX, lastX);
    sumY += readAxis(PIN_Y, emaY, lastY);
    sumZ += readAxis(PIN_Z, emaZ, lastZ);
    delay(100);
  }
  baselineX = sumX / 10.0f;
  baselineY = sumY / 10.0f;
  baselineZ = sumZ / 10.0f;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Baseline sat!");
  lcd.setCursor(0, 1);
  lcd.print("Klar til maling");
  delay(1500);
  lcd.clear();
}

// ---- NeoPixel farveskala ----
void updateNeoPixel(float exposure) {
  float t = (exposure - EXP_LOW) / (EXP_HIGH - EXP_LOW);
  t = max(0.0f, min(1.0f, t));

  uint8_t r = 0, g = 0, b = 0;
  if (t < 0.5f) {
    r = (uint8_t)(t * 2.0f * 255);
    g = 255;
  } else {
    r = 255;
    g = (uint8_t)((1.0f - (t - 0.5f) * 2.0f) * 255);
  }

  for (int i = 0; i < NEO_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

// ---- LCD bjælke (12 tegn bred) ----
void printBar(float exposure) {
  float t = (exposure - EXP_LOW) / (EXP_HIGH - EXP_LOW);
  t = max(0.0f, min(1.0f, t));
  int filled = (int)(t * 12.0f);

  lcd.print("[");
  for (int i = 0; i < 12; i++) {
    lcd.print(i < filled ? "=" : "-");
  }
  lcd.print("]");
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(BTN_SCREEN, INPUT_PULLUP);

  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("RF Eksponeringm.");

  // strip.begin();
  // strip.setBrightness(80);

  delay(1000);
  setBaseline();
}

// ---- Loop ----
void loop() {
  static bool lastScreenState = HIGH;

  // Skift skærm
  bool screenState = digitalRead(BTN_SCREEN);
  if (screenState == LOW && lastScreenState == HIGH) {
    screen = (screen + 1) % 2;
    lcd.clear();
  }
  lastScreenState = screenState;

  // Læs alle tre akser
  float dX = readAxis(PIN_X, emaX, lastX);
  float dY = readAxis(PIN_Y, emaY, lastY);
  float dZ = readAxis(PIN_Z, emaZ, lastZ);

  // Samlet eksponering
  float exposure = combineAxes(dX, dY, dZ);

  // Opdatér NeoPixel
  // updateNeoPixel(exposure); // Genkommenter når strip er sat på

  if (screen == 0) {
    // Skærm 0: samlet eksponering + bjælke
    lcd.setCursor(0, 0);
    lcd.print("Exp:");
    lcd.print(exposure, 1);
    lcd.print(" dBm   ");
    lcd.setCursor(0, 1);
    printBar(exposure);

  } else {
    // Skærm 1: de tre akser enkeltvis
    lcd.setCursor(0, 0);
    lcd.print("X:");
    lcd.print(dX, 1);
    lcd.print(" Y:");
    lcd.print(dY, 1);
    lcd.print("  ");
    lcd.setCursor(0, 1);
    lcd.print("Z:");
    lcd.print(dZ, 1);
    lcd.print("            ");
  }

  // Serial output
  Serial.print("X: ");   Serial.print(dX, 1);
  Serial.print(" Y: ");  Serial.print(dY, 1);
  Serial.print(" Z: ");  Serial.print(dZ, 1);
  Serial.print(" | Exp: "); Serial.print(exposure, 1);
  Serial.println(" dBm");

  delay(200);
}
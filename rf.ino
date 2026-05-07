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

// RF - three modules
const int PIN_X = A0;
const int PIN_Y = A1;
const int PIN_Z = A2;

const float SLOPE     = 0.0228f;
const float INTERCEPT = 0.4f;
const float EMA_ALPHA = 0.2f;

// EMA and last per axis
float emaX = -55.0f, emaY = -55.0f, emaZ = -55.0f;
float lastX = -55.0f, lastY = -55.0f, lastZ = -55.0f;

float baselineX = 0.0f, baselineY = 0.0f, baselineZ = 0.0f;

// Exposure limits in dBm
const float EXP_LOW    = -50.0f; // Green — low exposure
const float EXP_HIGH   = -30.0f; // Red  — high exposure
const float EXP_THRESH = -55.0f; // Under this threshold, NeoPixel turns off

// Buttons
const int BTN_SCREEN = PC13;
int screen = 0;

// Forward declarations
void updateNeoPixel(float exposure);
void printBar(float exposure);
void setBaseline();
float readAxis(int pin, float &ema, float &last);
float combineAxes(float dX, float dY, float dZ);

// ---- Read one axis ----
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

// ---- Combine the three axes to one exposure ----
float combineAxes(float dX, float dY, float dZ) {
  float linX = pow(10.0f, dX / 10.0f);
  float linY = pow(10.0f, dY / 10.0f);
  float linZ = pow(10.0f, dZ / 10.0f);
  return 10.0f * log10(linX + linY + linZ);
}

// ---- Set the baseline ----
void setBaseline() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Defines baseline");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");

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
  lcd.print("Baseline set!");
  lcd.setCursor(0, 1);
  lcd.print("Ready to read");
  delay(1500);
  lcd.clear();
}

// ---- NeoPixel VU-meter ----
void updateNeoPixel(float exposure) {
  float t = (exposure - EXP_LOW) / (EXP_HIGH - EXP_LOW);
  t = max(0.0f, min(1.0f, t));

  int lit = (int)(t * NEO_COUNT);

  for (int i = 0; i < NEO_COUNT; i++) {
    if (i >= lit) {
      strip.setPixelColor(i, strip.Color(0, 0, 0));
    } else {
      float pos = (float)i / (NEO_COUNT - 1);
      uint8_t r = 0, g = 0, b = 0;
      if (pos < 0.5f) {
        r = (uint8_t)(pos * 2.0f * 255);
        g = 255;
      } else {
        r = 255;
        g = (uint8_t)((1.0f - (pos - 0.5f) * 2.0f) * 255);
      }
      strip.setPixelColor(i, strip.Color(r, g, b));
    }
  }
  strip.show();
}

// ---- LCD bar (12 wide) ----
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
  lcd.print("RF Exposure");

  strip.begin();
  strip.setBrightness(30);
  strip.show();

  delay(1000);
  setBaseline();
}

// ---- Loop ----
void loop() {
  static bool lastScreenState = HIGH;

  // Change screen
  bool screenState = digitalRead(BTN_SCREEN);
  if (screenState == LOW && lastScreenState == HIGH) {
    screen = (screen + 1) % 2;
    lcd.clear();
  }
  lastScreenState = screenState;

  // Read all three axes
  float dX = readAxis(PIN_X, emaX, lastX);
  float dY = readAxis(PIN_Y, emaY, lastY);
  float dZ = readAxis(PIN_Z, emaZ, lastZ);

  // Combined exposure
  float exposure = combineAxes(dX, dY, dZ);

  // Update NeoPixel
  updateNeoPixel(exposure);

  if (screen == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Exp:");
    lcd.print(exposure, 1);
    lcd.print(" dBm   ");
    lcd.setCursor(0, 1);
    printBar(exposure);

  } else {
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

  Serial.print("X: ");   Serial.print(dX, 1);
  Serial.print(" Y: ");  Serial.print(dY, 1);
  Serial.print(" Z: ");  Serial.print(dZ, 1);
  Serial.print(" | Exp: "); Serial.print(exposure, 1);
  Serial.println(" dBm");

  delay(200);
}
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// LCD - I2C address 0x27, 16 columns and 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

// NeoPixel - pin D6 and 7 leds, color order GRB at 800kHz
#define NEO_PIN    D6
#define NEO_COUNT  7
Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

// RF - Analog input for the three sensor modules
const int PIN_X = A0;
const int PIN_Y = A1;
const int PIN_Z = A2;

// Number of active RF axes - change from between 1-3 depending on how many active
const int ACTIVE_AXES = 3;

// Hardware offset calibration - compensates for variation between modules/antennas
// Adjustment of these to align all axes to a similar baseline
const float OFFSET_X =  2.0f;         // X reads ~2 dBm too low
const float OFFSET_Y = -2.0f;         // Y reads ~2 dBm too high
const float OFFSET_Z =  0.0f;         // Z is the reference

// Calculate the voltage from the RF-module to dBm
// dBm = (INTERCEPT - volts) / SLOPE
const float SLOPE     = 0.0228f;
const float INTERCEPT = 0.4f;

// Exponential Moving Average - fast filtering out the noise
// Low alpha means new calculations only weights 20% and the history is 80% - smoothing out short spikes
const float EMA_ALPHA = 0.2f;

// Slow rolling EMA around 100 samples
// (alpha = 1/100 = 0.01) - the filter responds slowly to changes giving long-term average for spotting trends
const float SLOW_EMA_ALPHA =  0.01f;
float slowEma =               -55.0f; // initialized to typical room baselinge
bool slowEmaInitialized =     false;  // tracks whether the first sample has been set
int slowEmaSampleCount =      0;      // Counting the samples from 0-100 to show the "warmup" process

// Buzzer
const int BUZZER_PIN    = D9;
const float BUZZ_THRESH = -40.0f;     // dBm level for when the buzzer starts
unsigned long lastBuzz  = 0;          // Timestamp of last buzz - for controlling of the beep intervals

// Fast EMA values per axis - updated every loop iteration to smooth out noise
float emaX = -55.0f, emaY = -55.0f, emaZ = -55.0f;

// Previous EMA values per axis - detecting and rejectting sudden jumps (>20 dBm change)
float lastX = -55.0f, lastY = -55.0f, lastZ = -55.0f;

// Saved the radiation from the background at startup - not used atm
float baselineX = 0.0f, baselineY = 0.0f, baselineZ = 0.0f;

// Exposure limits in dBm
const float EXP_LOW    = -50.0f;       // The bottom of the scale (all LEDs off)
const float EXP_HIGH   = -30.0f;       // The top of the scale (all LEDs on)
const float EXP_THRESH = -55.0f;       // Below this, the exposure is considered insignificant

// Buttons
const int BTN_SCREEN   = PC13;         // cycling through the screens - the blue button integrated on the board
const int SWITCH_PIN   = D5;           // Powering the device on/off
int screen             = 0;            // Currently displayed screen
bool powered           = true;         // Is the device active atm?

// Detection of a button press
bool lastScreenState = HIGH;

// Forward declarations - necessary because C++ reads from top to bottom
// some functions are called before they are defined
void updateNeoPixel(float exposure);
void updateBuzzer(float exposure);
void printBar(float exposure);
void setBaseline();
float readAxis(int pin, float &ema, float &last, float offset);
float combineAxes(float dX, float dY, float dZ);

// Read one axis, averages multiple analog samles and converting to dBM
// applying the fast EMA, rejection of sudden jumps and adds hardware offset
float readAxis(int pin, float &ema, float &last, float offset) {
  // Use more samples when the signal is weak - more noise at low level
  int numSamples = (ema < -45.0f) ? 128 : 64;
  long sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100); // Small delay between samples to avoid reading the same ADC (Analog to Converter) convertion twince
  }

  float raw             = sum / (float)numSamples;      // Average raw ADC value (0-4095 at 12-bit)
  float volts           = raw * 3.3f / 4095.0f;         // Convert to voltage (0-3.3v)
  float dBm             = (INTERCEPT - volts) / SLOPE;  // Convert voltage to dBm using calibration constants
  if (dBm < -70.0f) dBm = -70.0f;                       // clamp to floor - below -70 dBm is considered no signal

  // Fast EMA filter
  ema = EMA_ALPHA * dBm + (1.0f - EMA_ALPHA) * ema;

  //Rejection if the new EMA differs by more than 20 dBm from the last accepted value, return pevious
  if (abs(ema - last) > 20.0f) {
    return last + offset;
  }

  last = ema;
  return ema + offset;    // hardware offret bf returning
}

// Combine readings from active axes into a signel value in dBm
// Converts each axis from dBm to linear power, sum and then converting back to dBm
float combineAxes(float dX, float dY, float dZ) {
  float linX = pow(10.0f, dX / 10.0f);
  float linY = pow(10.0f, dY / 10.0f);
  float linZ = pow(10.0f, dZ / 10.0f);

  if (ACTIVE_AXES == 1) return dX;
  if (ACTIVE_AXES == 2) return 10.0f * log10(linX + linY);
  return 10.0f * log10(linX + linY + linZ);
}

// Update slow rolling EMA
// The first call seeds the EMA with actual value instead of starting from -55 - this would result in a long warmup
void updateSlowEma(float exposure) {
  if (!slowEmaInitialized) {
    slowEma = exposure;
    slowEmaInitialized = true;
  } else {
    slowEma = SLOW_EMA_ALPHA * exposure + (1.0f - SLOW_EMA_ALPHA) * slowEma;
  }
  if (slowEmaSampleCount < 100) slowEmaSampleCount++;
}

// Power off
// Resetting the slow EMA
void powerOff() {
  powered = false;
  slowEmaInitialized = false;
  slowEmaSampleCount = 0;

  strip.clear();
  strip.show();

  lcd.clear();
  lcd.noBacklight();

  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
}

// Measurement of the bg RF level across all active axes and saves it as a baseline
// Takes 10 samples with 100ms spacing - around 1 second total
// The device turns off immediately, if the switch is flipped during calibrations
void setBaseline() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Defines baseline");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");

  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 10; i++) {
    if (digitalRead(SWITCH_PIN) == HIGH) {
      powerOff();
      return;
    }
    sumX += readAxis(PIN_X, emaX, lastX, OFFSET_X);
if (ACTIVE_AXES >= 2) sumY += readAxis(PIN_Y, emaY, lastY, OFFSET_Y);
if (ACTIVE_AXES >= 3) sumZ += readAxis(PIN_Z, emaZ, lastZ, OFFSET_Z);
    delay(100);
  }

  // Store the average of 10 samples as the baseline for each axis
  baselineX = sumX / 10.0f;
  baselineY = sumY / 10.0f;
  baselineZ = sumZ / 10.0f;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Baseline set!");
  lcd.setCursor(0, 1);
  lcd.print("Ready to read");

  // Show confirmation messae for 1.5 seconds, checking switch in the meantime
  for (int i = 0; i < 15; i++) {
    if (digitalRead(SWITCH_PIN) == HIGH) {
      powerOff();
      return;
    }
    delay(100);
  }
  lcd.clear();
}

// ---- NeoPixel ----
// Low exposure = green LEDs lit from the right
// high exposure = More of the strip is lighting up
void updateNeoPixel(float exposure) {
  // Normalize the exposure to 0.0-1.0 within the defined range
  float t = (exposure - EXP_LOW) / (EXP_HIGH - EXP_LOW);
  t = max(0.0f, min(1.0f, t));

  // Number of LEDs
  int lit = (int)(t * NEO_COUNT);

  for (int i = 0; i < NEO_COUNT; i++) {
    if (i < (NEO_COUNT - lit)) {
      strip.setPixelColor(i, strip.Color(0, 0, 0));
    } else {
      // Color gradient: green (low) -> yellow -> red (high)
      float pos = (float)(NEO_COUNT - 1 - i) / (NEO_COUNT - 1);
      uint8_t r = 0, g = 0, b = 0;
      if (pos < 0.5f) {
        r = (uint8_t)(pos * 2.0f * 255);                    // Ramp red up
        g = 255;                                            // Keep green full
      } else {
        r = 255;                                            // Keed red full
        g = (uint8_t)((1.0f - (pos - 0.5f) * 2.0f) * 255);  // Ramp green down
      }
      strip.setPixelColor(i, strip.Color(r, g, b));
    }
  }
  strip.show();
}

// ----- Buzzer -----
// Below the threshold, the buzzer is silent, and above it beeps faster as the exposure increases
// The maximum exposure is 100ms (10beeps/sec), at threshold it is 800ms
void updateBuzzer(float exposure) {
  if (exposure < BUZZ_THRESH) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  // Normalize exposure between threshold and max
  float t = (exposure - BUZZ_THRESH) / (EXP_HIGH - BUZZ_THRESH);
  t = max(0.0f, min(1.0f, t));

  unsigned long interval = (unsigned long)(800 - t * 700);

  if (millis() - lastBuzz > interval) {
    lastBuzz = millis();
    tone(BUZZER_PIN, 1000, 50);   // 1kHz tone for 50ms
  }
}

// ----- LCD bar (12 wide) -----
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

// Setup
// Initializes serial, pins, I2C, LCD, NeoPixel
// If the switch is already ON at boot, the baseline calibration runs immediately
// If switch is off, the device starts in powered-down state
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);       // Use 12-bit ADC resolution (0-4095)

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BTN_SCREEN, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // I2C pins for LCD (STM32 alternate pin mapping)
  Wire.setSDA(D14);
  Wire.setSCL(D15);
  Wire.begin();

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("RF Exposure");

  strip.begin();
  strip.setBrightness(30);        // Moderate brightness to avoid drawing to much current
  strip.show();

  delay(1000);                    // Brief startup splash

  if (digitalRead(SWITCH_PIN) == LOW) {
    setBaseline();
  } else {
    powered = false;
    lcd.noBacklight();
    lcd.clear();
  }
}

// Loop
// Runs continuously while device is on
// Handles power toggle, screen switching, reading sensors and updating all outputs
void loop() {
  bool switchState = digitalRead(SWITCH_PIN);

  // Power on: switch flipped to ON while device was off - reset state and recalibrate
  if (switchState == LOW && !powered) {
    powered = true;
    screen = 0;
    emaX = -55.0f; emaY = -55.0f; emaZ = -55.0f;
    lastX = -55.0f; lastY = -55.0f; lastZ = -55.0f;
    slowEma = -55.0f;
    slowEmaInitialized = false;
    slowEmaSampleCount = 0;
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("RF Exposure");
    delay(1000);
    setBaseline();
  }

  // Power off: switch flipped to OFF while device was on
  if (switchState == HIGH && powered) {
    powerOff();
  }

  // Do nothing while powered off
  if (!powered) {
    delay(100);
    return;
  }

  // Change screen
  bool screenState = digitalRead(BTN_SCREEN);
  if (screenState == LOW && lastScreenState == HIGH) {
    screen = (screen + 1) % 3; // cycle through the three screens
    lcd.clear();
  }
  lastScreenState = screenState;

  // Read active axes with the offset applied
  float dX = readAxis(PIN_X, emaX, lastX, OFFSET_X);
  float dY = (ACTIVE_AXES >= 2) ? readAxis(PIN_Y, emaY, lastY, OFFSET_Y) : -70.0f;
  float dZ = (ACTIVE_AXES >= 3) ? readAxis(PIN_Z, emaZ, lastZ, OFFSET_Z) : -70.0f;

  // Combined exposure into a single value
  float exposure = combineAxes(dX, dY, dZ);

  // Update slow rolling EMA
  updateSlowEma(exposure);

  // Update NeoPixel and buzzer
  updateNeoPixel(exposure);
  updateBuzzer(exposure);

  // Screen 0: Live combined exposure with the bar
  if (screen == 0) {
      lcd.setCursor(0, 0);
      lcd.print(exposure, 1);
      lcd.print(" dBm   ");
      lcd.setCursor(0, 1);
      printBar(exposure);

// Screen 1: Slow rolling EMA (100 samples) with warmup
  } else if (screen == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Avg(100):");
    if (slowEmaSampleCount < 100) {
      lcd.print(slowEmaSampleCount);
      lcd.print("/100  ");
    } else {
      lcd.print("ready ");
    }
    lcd.setCursor(0, 1);
    lcd.print(slowEma, 1);
    lcd.print(" dBm   ");

  // Screen 2: Individual axis readings
  } else {
    lcd.setCursor(0, 0);
    lcd.print("X:"); lcd.print(dX, 1);
    lcd.print(" Y:"); lcd.print(dY, 1);
    lcd.print("  ");
    lcd.setCursor(0, 1);
    lcd.print("Z:"); lcd.print(dZ, 1);
    lcd.print(" dBm  ");
} 
  // Serial output for loggind and debugging via USB
  Serial.print("X: ");      Serial.print(dX, 2);       Serial.print(" dBm");
  Serial.print(" Y: ");     Serial.print(dY, 2);       Serial.print(" dBm");
  Serial.print(" Z: ");     Serial.print(dZ, 2);       Serial.print(" dBm");
  Serial.print(" | Exp: "); Serial.print(exposure, 2); Serial.print(" dBm");
  Serial.print(" | Avg: "); Serial.print(slowEma, 2);  Serial.println(" dBm");
  
  delay(200); // 4-5 updates per second
}
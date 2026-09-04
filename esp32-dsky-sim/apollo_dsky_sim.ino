/*
  Apollo 11 DSKY / Lunar Descent Simulation
  ------------------------------------------
  Hardware: ESP32, MPU6050 (I2C), 0.96" SSD1306 OLED (I2C, 128x64), 2 buttons

  This is not a port of the real AGC assembly (github.com/chrislgarry/apollo-11) -
  that code runs on a completely different 1960s architecture. Instead, this
  reproduces the DSKY interface and the shape of the descent programs (P63/P64/
  P66/P68), using the MPU6050 as a stand-in for the LM's attitude reference.

  Attitude is computed with a complementary filter: gyro integration for
  smooth short-term response, blended with the accelerometer's gravity
  vector to correct long-term drift.

  Wiring:
    ESP32 3V3  -> OLED VCC, MPU6050 VCC
    ESP32 GND  -> OLED GND,  MPU6050 GND
    ESP32 GPIO21 (SDA) -> OLED SDA, MPU6050 SDA
    ESP32 GPIO22 (SCL) -> OLED SCL, MPU6050 SCL
    ESP32 GPIO4  -> Button 1 (PRO / Proceed) -> other leg to GND
    ESP32 GPIO5  -> Button 2 (MODE / Reset)  -> other leg to GND
    (Both buttons use internal pull-ups, so no external resistor is needed.)

  Libraries needed (Arduino Library Manager):
    Adafruit SSD1306
    Adafruit GFX Library
    Adafruit MPU6050
    Adafruit Unified Sensor
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define BTN_PRO 4
#define BTN_MODE 5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

// ---- Mission programs (modeled on the real Apollo 11 descent programs) ----
enum Program {
  P00 = 0,   // Standby
  P63 = 63,  // Braking Phase
  P64 = 64,  // Approach Phase
  P66 = 66,  // Rate of Descent (manual)
  P68 = 68   // Landing confirmed
};

Program currentProgram = P00;
int verb = 37;   // V37 = change program, shown at idle
int noun = 0;

// Simulated flight data
float altitude = 15000.0;      // meters (PDI on the real mission started around 15 km)
float velocity = -45.0;        // m/s, negative = descending
float pitchDeg = 0.0;
float rollDeg = 0.0;
const float moonGravity = 1.62; // m/s^2

bool alarmActive = false;
bool alarmTriggeredThisPhase = false;

unsigned long lastPhysicsUpdate = 0;
unsigned long lastButtonPro = 0;
unsigned long lastButtonMode = 0;
unsigned long lastAttitudeUpdate = 0;

const float COMP_FILTER_ALPHA = 0.98; // weight given to the gyro; rest goes to accel

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(BTN_PRO, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 not found");
    while (1) delay(10);
  }

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  randomSeed(analogRead(0));

  showBootScreen();
  delay(1500);
}

void loop() {
  handleButtons();
  readAttitude();
  updatePhysics();
  maybeTriggerAlarm();
  drawDSKY();
  delay(50);
}

// ---------------- Buttons ----------------
void handleButtons() {
  if (digitalRead(BTN_PRO) == LOW && millis() - lastButtonPro > 300) {
    lastButtonPro = millis();
    onProceed();
  }
  if (digitalRead(BTN_MODE) == LOW && millis() - lastButtonMode > 300) {
    lastButtonMode = millis();
    onModeReset();
  }
}

void onProceed() {
  if (alarmActive) {
    // Acknowledge the alarm and continue, same as the real mission did with 1202/1201
    alarmActive = false;
    return;
  }

  switch (currentProgram) {
    case P00:
      currentProgram = P63;
      verb = 16; noun = 63; // V16N63: altitude, altitude rate, forward velocity
      alarmTriggeredThisPhase = false;
      break;
    case P63:
      currentProgram = P64;
      alarmTriggeredThisPhase = false;
      break;
    case P64:
      currentProgram = P66;
      verb = 6; noun = 63;
      break;
    case P66:
      // Stays in P66 until altitude reaches zero
      break;
    case P68:
      resetMission();
      break;
  }
}

void onModeReset() {
  resetMission();
}

void resetMission() {
  currentProgram = P00;
  verb = 37; noun = 0;
  altitude = 15000.0;
  velocity = -45.0;
  alarmActive = false;
  alarmTriggeredThisPhase = false;
}

// ---------------- Sensors ----------------
void readAttitude() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  float dt = (now - lastAttitudeUpdate) / 1000.0;
  lastAttitudeUpdate = now;
  if (dt <= 0 || dt > 1) dt = 0.02; // guard first run / long stalls

  // Accelerometer-only estimate (gravity vector). Drift-free but noisy,
  // and wrong under linear acceleration (not just tilt).
  float accelPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float accelRoll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;

  // Gyro-only estimate: integrate angular rate (rad/s -> deg/s -> deg).
  float gyroPitchRate = g.gyro.y * 180.0 / PI;
  float gyroRollRate  = g.gyro.x * 180.0 / PI;

  // Complementary filter: trust the gyro short-term, pull toward the
  // accelerometer's gravity reference over time to cancel drift.
  pitchDeg = COMP_FILTER_ALPHA * (pitchDeg + gyroPitchRate * dt) + (1.0 - COMP_FILTER_ALPHA) * accelPitch;
  rollDeg  = COMP_FILTER_ALPHA * (rollDeg  + gyroRollRate  * dt) + (1.0 - COMP_FILTER_ALPHA) * accelRoll;
}

// ---------------- Physics ----------------
void updatePhysics() {
  if (currentProgram != P63 && currentProgram != P64 && currentProgram != P66) return;

  unsigned long now = millis();
  float dt = (now - lastPhysicsUpdate) / 1000.0;
  if (dt <= 0 || dt > 1) { lastPhysicsUpdate = now; return; }
  lastPhysicsUpdate = now;

  // Tilting forward (positive pitch) increases simulated braking thrust,
  // similar to pointing the descent engine to slow the vehicle.
  float throttle = (pitchDeg + 30.0) / 60.0; // maps roughly -30..+30 deg to 0..1
  throttle = constrain(throttle, 0.0, 1.0);
  float thrustAccel = throttle * 2.2; // tuned so full throttle can overcome moon gravity

  velocity += (moonGravity - thrustAccel) * dt;
  altitude += velocity * dt;

  if (altitude <= 0) {
    altitude = 0;
    currentProgram = P68;
  }
}

// ---------------- Alarms (1202/1201 homage) ----------------
void maybeTriggerAlarm() {
  if (currentProgram != P63 && currentProgram != P64) return;
  if (alarmTriggeredThisPhase || alarmActive) return;

  // Small random chance per loop tick - roughly once per phase, just a nod to the real mission
  if (random(0, 3000) == 0) {
    alarmActive = true;
    alarmTriggeredThisPhase = true;
  }
}

// ---------------- Display ----------------
void showBootScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("APOLLO GUIDANCE");
  display.println("COMPUTER SIM");
  display.println();
  display.println("PRESS PRO TO START");
  display.display();
}

void drawDSKY() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("PROG "); display.println((int)currentProgram);

  display.setCursor(0, 10);
  display.print("VERB "); display.print(verb);
  display.print(" NOUN "); display.println(noun);

  display.drawLine(0, 20, 128, 20, SSD1306_WHITE);

  display.setCursor(0, 24);
  display.print("ALT  "); display.print(altitude, 0); display.println(" M");

  display.setCursor(0, 34);
  display.print("RATE "); display.print(velocity, 1); display.println(" M/S");

  display.setCursor(0, 44);
  display.print("PTCH "); display.print(pitchDeg, 0);
  display.print(" ROL "); display.println(rollDeg, 0);

  if (alarmActive) {
    display.fillRect(0, 54, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(2, 55);
    display.print("PROG ALARM 1202");
    display.setTextColor(SSD1306_WHITE);
  } else if (currentProgram == P68) {
    display.setCursor(0, 54);
    display.println("*** LANDED ***");
  } else {
    display.setCursor(0, 54);
    display.println("PRO=next  MODE=reset");
  }

  display.display();
}

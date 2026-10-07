//This version has the AS5600 but does not use it
//Fixes issue with move complete being sent back to gatway
//This version still only uses TMC2209 driver
//This version brings the TMC2209 version up to date with the Skaarhoj implementation
//This version published to github
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <FastAccelStepper.h>
#include <SPI.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
//#include "driver/uart.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
Preferences prefs;

// --- Device identity ---            This must be kept up to date and added to the manifest whenever an update is issued
#define DEVICE_TYPE "slider"  // or "slider", "ptz", "dolly", "jib"
#define HW_REVISION "revA"    //Board revision
#define FW_VERSION "3.00.01"  //Firmware revisioning system for OTA (main, secondary, Patch)


// ---------------------------------------------------------
// PINS
// ---------------------------------------------------------
#define STEPPER_STEP_PIN 27
#define STEPPER_DIR_PIN 26
#define MOTORS_EN_PIN 13
#define DIAG1_PIN 36

#define STEPPER_HOME_PIN 15  // limit switch (active LOW)
#define STALLGUARD_PIN 34    // DIAG (reserved)
#define I2C_MUX_ADDR 0x70    // TCA9548A default address

// UART2 link to WT32 gateway (JSON + OTA)
#define UART2_RX_PIN 16
#define UART2_TX_PIN 17

// SPI pins for AS5047P
#define ENC_MOSI 23
#define ENC_MISO 19
#define ENC_SCK 18
#define ENC_CS 32

//OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===============================
// Mechanical Setup (editable)
// ===============================
float steps_per_rev = 200.0f;  // 200 or 400
float microsteps = 64.0f;      // fixed for now
float mm_per_rev = 34.9f;      // pulley/belt dependent
float steps_per_mm = 321.3f;   // derived

// ===============================
// Motion Settings (editable)
// ===============================
float maxSpeed_mm_s = 25.0f;
float accel_mm_s2 = 600.0f;

float bst_max_vel_mm_s = 200.0f;
float bst_accel_mm_s2 = 200.0f;

bool invertDirection = false;

// ===============================
// Travel Limits (editable)
// ===============================
float LOGICAL_HOME_MM = 10.0f;
float SOFT_MIN_MM = 10.0f;
float SOFT_MAX_MM = 345.0f;

// ===============================
// Safety constants (NOT editable)
// ===============================
const float HOME_EMERGENCY_START = 10.0f;
const float HOME_EMERGENCY_END = 15.0f;

const float HOME_SLOW_START = 12.0f;
const float HOME_SLOW_END = 32.0f;

const float FAR_SLOW_START = 325.0f;
const float FAR_SLOW_END = 345.0f;
const float FAR_EMERGENCY = 340.0f;



// ---------------------------------------------------------
// CONSTANTS & GLOBALS
// ---------------------------------------------------------


// Fixed acceleration for absolute moves and shots
const float DEFAULT_ACCEL_MM_S2 = 4000.0f;

volatile bool inOtaUpdate = false;
volatile bool otaInProgress = false;
int ipScrollX = 35;              // current X position  --- OLED IP scroll state ---
uint16_t ipTextWidth = 0;        // measured width of the IP text
unsigned long lastIpScroll = 0;  // timestamp for non-blocking scroll
bool ipShouldScroll = false;     // whether scrolling is needed
String lastIpShown = "";         // detect IP changes
bool oledInitialized = false;
bool oledReady = false;
String gateway_ip = "0.0.0.0";  //So the IP can be viewed on the OLED


volatile bool forceLoopExit = false;


FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;
bool inPositionMove = false;
float abs_target_mm = 0.0f;

bool homed = false;
bool moving = false;
bool fault = false;
bool stall = false;
bool limitHit = false;

long logical_steps = 0;
float logical_mm = 0.0f;

// encoder tracking
uint16_t last_raw = 0;
long rev_count = 0;
float encoder_offset_mm = 0.0f;
float encoder_mm = 0.0f;
String uart2_rx;

float ramp_value = 0.0f;
float target_mm = 0.0f;

float current_speed_mm_s = 0.0f;
unsigned long lastUpdate = 0;

float joystick_speed = 0.0f;

// Timing
unsigned long moveStartTime = 0;
bool timingActive = false;

// ---------------------------------------------------------
// BST SHOT STATE
// ---------------------------------------------------------
bool inShot = false;
unsigned long shotEndTime = 0;
float shotSpeed = 0.0f;

// ---------------------------------------------------------
// HOMING STATE
// ---------------------------------------------------------
enum HomeState {
  HOME_IDLE,
  HOME_MOVING,
  HOME_DONE,
  HOME_FAULT
};

HomeState homeState = HOME_IDLE;
unsigned long homeStartTime = 0;

// ---------------------------------------------------------
// AS5600 Encoder read function
// ---------------------------------------------------------

uint16_t readAS5600Raw() {
  selectMuxChannel(6);  // AS5600 on channel 6

  Wire.beginTransmission(0x36);  // AS5600 address
  Wire.write(0x0C);              // RAW ANGLE register MSB
  Wire.endTransmission();

  Wire.requestFrom(0x36, 2);  // read MSB + LSB

  uint16_t msb = Wire.read();
  uint16_t lsb = Wire.read();

  return (msb << 8) | lsb;  // 12-bit angle (0–4095)
}


// ============================================================
//         Load The Editable variables from memory
// ============================================================
void loadConfigFromNVS() {

  prefs.begin("slider", true);  // read-only

  steps_per_rev = prefs.getFloat("steps_rev", 200.0f);
  microsteps = 64.0f;  // fixed
  mm_per_rev = prefs.getFloat("mm_rev", 34.9f);

  maxSpeed_mm_s = prefs.getFloat("max_speed", 25.0f);
  accel_mm_s2 = prefs.getFloat("accel", 600.0f);

  bst_max_vel_mm_s = prefs.getFloat("bst_max_vel", 200.0f);
  bst_accel_mm_s2 = prefs.getFloat("bst_accel", 200.0f);

  invertDirection = prefs.getBool("invert_dir", false);

  LOGICAL_HOME_MM = prefs.getFloat("home_offset", 10.0f);
  SOFT_MIN_MM = LOGICAL_HOME_MM;
  SOFT_MAX_MM = prefs.getFloat("soft_max", 345.0f);

  prefs.end();

  // Derived
  steps_per_mm = (steps_per_rev * microsteps) / mm_per_rev;
}

// =========================================================

//       MUX Assignments
// =========================================================
void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(I2C_MUX_ADDR);
  Wire.write(1 << channel);  // channel 0–7
  Wire.endTransmission();
}
// =================================================================================
//       Save the editable variables to memory when gateway sends 'set_config'
// =================================================================================
void saveConfigToNVS() {
  prefs.begin("slider", false);  // write mode

  prefs.putFloat("steps_rev", steps_per_rev);
  prefs.putFloat("mm_rev", mm_per_rev);

  prefs.putFloat("max_speed", maxSpeed_mm_s);
  prefs.putFloat("accel", accel_mm_s2);

  prefs.putFloat("bst_max_vel", bst_max_vel_mm_s);
  prefs.putFloat("bst_accel", bst_accel_mm_s2);

  prefs.putBool("invert_dir", invertDirection);

  prefs.putFloat("home_offset", LOGICAL_HOME_MM);
  prefs.putFloat("soft_max", SOFT_MAX_MM);

  prefs.end();
}

// ======================================================
//                     Update OLED
// ======================================================
void updateDisplayIP() {
  selectMuxChannel(5);


  if (!oledInitialized) return;

  const char* text = gateway_ip.c_str();

  // Detect IP change
  if (gateway_ip != lastIpShown) {
    lastIpShown = gateway_ip;

    // Measure text width
    int16_t x1, y1;
    uint16_t w, h;
    display.setTextSize(1);
    display.setFont();
    display.getTextBounds(text, 0, 20, &x1, &y1, &w, &h);

    ipTextWidth = w;
    ipScrollX = 35;  // reset
    ipShouldScroll = (w > (128 - 63));
  }

  // If it fits, draw static
  if (!ipShouldScroll) {
    display.clearDisplay();
    display.setCursor(35, 20);
    display.print(text);
    display.display();
    return;
  }

  // Non-blocking scroll
  if (millis() - lastIpScroll >= 20) {
    lastIpScroll = millis();

    display.clearDisplay();
    display.setCursor(ipScrollX, 20);
    display.print(text);
    display.display();

    ipScrollX--;

    // Loop when fully off-screen
    if (ipScrollX < -(int)ipTextWidth) {
      ipScrollX = 128;
    }
  }
}


void updatePositionFromEncoder() {
  uint16_t raw = readAS5600Raw();
  int16_t diff = (int16_t)raw - (int16_t)last_raw;

  // 12-bit wrap detection
  if (diff > 2048) rev_count--;
  if (diff < -2048) rev_count++;

  last_raw = raw;
  // 12-bit angle
  float angle = (float)raw / 4096.0f;
  float revs = rev_count + angle;
  float mm_raw = revs * mm_per_rev;
  mm_raw -= encoder_offset_mm;

  const float ENCODER_MM_SCALE = 1.1414f;
  const float ENCODER_MM_OFFSET = -44.3f;

  float mm_corrected = mm_raw * ENCODER_MM_SCALE + ENCODER_MM_OFFSET;

  encoder_mm = mm_corrected;
}

// ---------------------------------------------------------
// LIMIT SWITCH (DEBOUNCED)
// ---------------------------------------------------------
bool readLimitSwitch() {
  const int samples = 5;
  int count = 0;

  for (int i = 0; i < samples; i++) {
    if (digitalRead(STEPPER_HOME_PIN) == LOW) count++;
    delay(1);
  }

  return (count >= 3);
}

// ---------------------------------------------------------
// STEPPER SETUP & MOTION LIMITS
// ---------------------------------------------------------
void setupStepper() {
  engine.init();
  stepper = engine.stepperConnectToPin(STEPPER_STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(STEPPER_DIR_PIN);
    stepper->setEnablePin(MOTORS_EN_PIN, HIGH);
    stepper->setAutoEnable(true);
    stepper->setSpeedInHz(0);
    stepper->setAcceleration(1000);
  }
}

void applyMotionLimits() {
  if (!stepper) return;
  float accel_steps_s2 = accel_mm_s2 * steps_per_mm;
  stepper->setAcceleration(accel_steps_s2);
}

void stopMotion() {
  if (!stepper) return;

  stepper->forceStop();
  stepper->stopMove();
  stepper->setSpeedInHz(0);

  moving = false;

  // ******Tell gateway move is complete****** Add This to column code also
  StaticJsonDocument<128> evt;
  evt["event"] = "move_complete";
  evt["axis"] = "track";  // or your axis name

  String out;
  serializeJson(evt, out);
  Serial2.println(out);

  current_speed_mm_s = 0.0f;
}

// ---------------------------------------------------------
// CUBIC EASING HELPER
// ---------------------------------------------------------
float cubicEase(float t) {
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return t * t * (3.0f - 2.0f * t);
}

// ---------------------------------------------------------
// ABSOLUTE MOVE (continuous-speed based)
// ---------------------------------------------------------
void moveTo_mm(float target_mm, float speed_mm_s) {
  if (!stepper) return;
  if (!homed) {
    Serial.println("ABS move rejected: not homed");
    return;
  }

  if (target_mm < SOFT_MIN_MM) target_mm = SOFT_MIN_MM;
  if (target_mm > SOFT_MAX_MM) target_mm = SOFT_MAX_MM;

  abs_target_mm = target_mm;
  inPositionMove = true;

  float pos = logical_mm;
  float dir = (target_mm > pos) ? 1.0f : -1.0f;

  if (pos >= HOME_EMERGENCY_START && pos <= HOME_EMERGENCY_END) {
    if (dir < 0) {
      Serial.println("ABS MOVE BLOCKED: home emergency zone");
      inPositionMove = false;
      return;
    }
  }

  if (pos >= FAR_EMERGENCY) {
    if (dir > 0) {
      Serial.println("ABS MOVE BLOCKED: far-end emergency zone");
      inPositionMove = false;
      return;
    }
  }

  const float MAX_SPEED_MM_S = 100.0f;
  float norm = speed_mm_s / MAX_SPEED_MM_S;
  if (norm > 1.0f) norm = 1.0f;
  if (norm < -1.0f) norm = -1.0f;

  norm *= dir;

  // avoid zero command → would instantly satisfy completion test
  if (fabs(norm) < 0.1f) norm = 0.1f * dir;

  moveStartTime = millis();
  timingActive = true;
  joystick_speed = norm;


  Serial.printf("ABS MOVE (continuous): target_mm=%.2f  speed=%.2f  norm=%.2f\n",
                target_mm, speed_mm_s, norm);
}

// ---------------------------------------------------------
// RELATIVE MOVE
// ---------------------------------------------------------
void moveRel_mm(float delta_mm, float speed_mm_s) {
  if (!stepper) return;
  if (!homed) {
    Serial.println("Relative move rejected: not homed");
    return;
  }

  float target_mm = logical_mm + delta_mm;

  if (target_mm < SOFT_MIN_MM) target_mm = SOFT_MIN_MM;
  if (target_mm > SOFT_MAX_MM) target_mm = SOFT_MAX_MM;

  float speed = (speed_mm_s > 0) ? speed_mm_s : maxSpeed_mm_s;
  float maxSpeed_steps_s = speed * steps_per_mm;
  stepper->setSpeedInHz(maxSpeed_steps_s);

  long target_steps = (long)roundf(target_mm * steps_per_mm);
  stepper->moveTo(target_steps);
  moving = true;
  moveStartTime = millis();
  timingActive = true;
}

// ---------------------------------------------------------
// JOYSTICK CONTINUOUS VELOCITY ENGINE
// ---------------------------------------------------------
void setContinuousSpeed(float norm_speed) {
  if (!stepper) return;
  if (!homed) return;

  const float DEADBAND = 0.05f;
  const float MAX_SPEED_MM_S = 100.0f;

  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1e6f;
  if (dt <= 0) dt = 0.001f;
  lastUpdate = now;

  if (norm_speed < -1.0f) norm_speed = -1.0f;
  if (norm_speed > 1.0f) norm_speed = 1.0f;

  if (fabs(norm_speed) < DEADBAND) {
    norm_speed = 0.0f;
  }

  float target_speed_mm_s = norm_speed * MAX_SPEED_MM_S;

  float max_delta = accel_mm_s2 * dt;
  float delta = target_speed_mm_s - current_speed_mm_s;

  if (delta > max_delta) delta = max_delta;
  if (delta < -max_delta) delta = -max_delta;

  current_speed_mm_s += delta;

  if (fabs(current_speed_mm_s) < 0.01f) {
    current_speed_mm_s = 0.0f;
    stepper->forceStop();
    moving = false;
    return;
  }

  float pos = logical_mm;
  float safe_speed = current_speed_mm_s;

  if (pos >= HOME_EMERGENCY_START && pos <= HOME_EMERGENCY_END) {
    if (safe_speed < 0) {
      stepper->forceStop();
      moving = false;
      return;
    }
  }

  if (pos >= FAR_EMERGENCY) {
    if (safe_speed > 0) {
      stepper->forceStop();
      moving = false;
      return;
    }
  }

  float scale = 1.0f;

  if (pos >= HOME_SLOW_START && pos <= HOME_SLOW_END) {
    float t = (pos - HOME_SLOW_START) / (HOME_SLOW_END - HOME_SLOW_START);
    float s = cubicEase(t);
    if (s < scale) scale = s;
  }

  if (pos >= FAR_SLOW_START && pos <= FAR_SLOW_END) {
    float t = (FAR_SLOW_END - pos) / (FAR_SLOW_END - FAR_SLOW_START);
    float s = cubicEase(t);
    if (s < scale) scale = s;
  }

  safe_speed *= scale;

  float steps_per_s = safe_speed * steps_per_mm;

  stepper->setSpeedInHz(fabs(steps_per_s));

  if (steps_per_s > 0) {
    stepper->runForward();
  } else {
    stepper->runBackward();
  }

  moving = true;
}

// ---------------------------------------------------------
// HOMING
// ---------------------------------------------------------
void startHoming() {
  if (!stepper) return;

  Serial.println("=== HOMING START ===");

  stall = false;
  fault = false;
  limitHit = false;
  homed = false;

  if (readLimitSwitch()) {
    Serial.println("Carriage on limit at startup — retracting 20mm");

    float retract_mm = +20.0f;
    long retract_steps = (long)roundf(retract_mm * steps_per_mm);

    stepper->setSpeedInHz(8.0f * steps_per_mm);
    stepper->setAcceleration(2000);
    stepper->move(retract_steps);

    while (stepper->isRunning()) {
      updatePositionFromEncoder();
    }

    delay(100);

    if (readLimitSwitch()) {
      Serial.println("ERROR: Limit switch stuck active after retract");
      fault = true;
      homeState = HOME_FAULT;
      return;
    }
  }

  Serial.println("Fast homing pass...");

  float home_speed_mm_s = 15.0f;
  float steps_s = home_speed_mm_s * steps_per_mm;

  stepper->setSpeedInHz(steps_s);
  stepper->setAcceleration(steps_s * 2);

  stepper->move(-3000000);

  homeStartTime = millis();
  homeState = HOME_MOVING;
}

void updateHoming() {
  if (homeState != HOME_MOVING) return;



  if (readLimitSwitch()) {
    Serial.println("Limit switch hit (fast pass)");
    stepper->forceStop();
    limitHit = true;

    delay(50);

    Serial.println("Slow homing pass...");

    long retract_steps = (long)roundf(+3.0f * steps_per_mm);
    stepper->setSpeedInHz(5.0f * steps_per_mm);
    stepper->setAcceleration(2000);
    stepper->move(retract_steps);
    while (stepper->isRunning())
      ;
    delay(50);

    long slow_steps = (long)roundf(-10.0f * steps_per_mm);
    stepper->setSpeedInHz(2.0f * steps_per_mm);
    stepper->setAcceleration(1000);
    stepper->move(slow_steps);

    while (!readLimitSwitch()) {

      if (!stepper->isRunning()) break;
    }

    stepper->forceStop();
    delay(50);

    last_raw = readAS5600Raw();
    rev_count = 0;

    float angle = (float)last_raw / 16384.0f;
    float revs = angle;

    encoder_offset_mm = -revs * mm_per_rev;

    logical_mm = LOGICAL_HOME_MM;
    logical_steps = (long)roundf(LOGICAL_HOME_MM * steps_per_mm);
    stepper->setCurrentPosition(logical_steps);

    encoder_offset_mm -= LOGICAL_HOME_MM;

    long offset_steps = (long)roundf(LOGICAL_HOME_MM * steps_per_mm);

    stepper->setSpeedInHz(10.0f * steps_per_mm);
    stepper->setAcceleration(1000);
    stepper->move(offset_steps);

    while (stepper->isRunning()) {
    }

    homed = true;
    saveConfigToNVS();

    //Send ready message to gateway
    StaticJsonDocument<64> r;
    r["cmd"] = "ready";
    String out;
    Serial.println("=== SENDING READY  ===");

    serializeJson(r, out);
    Serial2.println(out);

    Serial.println("=== HOMING COMPLETE  ===");

    finishHoming();

    updateDisplayIP();
    homeState = HOME_DONE;
    return;
  }

  if (millis() - homeStartTime > 25000) {
    Serial.println("ERROR: Homing timeout");
    stepper->forceStop();
    fault = true;
    homeState = HOME_FAULT;
  }
}

void finishHoming() {
  // Mark homing complete
  homed = true;

  // Stop the motor
  stepper->forceStop();

  // Reset stepper position to PHYSICAL HOME (switch location)
  stepper->setCurrentPosition(0);

  // Apply your logical home offset (same as original firmware)
  stepper->setCurrentPosition(LOGICAL_HOME_MM * steps_per_mm);

  // Send READY to gateway
  StaticJsonDocument<64> readyDoc;
  readyDoc["cmd"] = "ready";

  String out;
  serializeJson(readyDoc, out);
  Serial2.println(out);

  Serial.println("Sent READY to gateway");
}



// ---------------------------------------------------------
// JSON COMMANDS
// ---------------------------------------------------------
void handleJsonCommand(const String& s) {
Serial.printf("UART2 RX: %s\n", s.c_str());

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, s)) return;

  const char* cmd = doc["cmd"] | "";
  Serial.printf("[CMD] %s\n", cmd);
  if (strcmp(cmd, "moveAbs") == 0) {
    Serial.println("===== MOVEABS RECEIVED =====");
    float pos_mm = doc["pos_mm"] | 0.0f;
    float sp = doc["speed"] | 50.0f;
    Serial.printf("Target=%.2f speed=%.2f time=%lu\n", pos_mm, sp, millis());
    moveTo_mm(pos_mm, sp);
    return;
  }
  // BST SHOT COMMAND (timed velocity move)
  if (strcmp(cmd, "shot") == 0) {
    float sp = doc["speed"] | 0.0f;
    float dur_ms = doc["duration_ms"] | 0;

    if (sp < -1.0f) sp = -1.0f;
    if (sp > 1.0f) sp = 1.0f;

    shotSpeed = sp;
    shotEndTime = millis() + dur_ms;
    inShot = true;

    inPositionMove = false;
    moving = false;

    Serial.printf("SHOT START: speed=%.3f  duration=%lu ms\n", sp, dur_ms);
    return;
  }

  // Gateway hello device ident
  if (strcmp(cmd, "hello") == 0) {
    if (inOtaUpdate) return;  // don't talk on Serial2 during OTA
    StaticJsonDocument<256> docOut;
    docOut["device"] = DEVICE_TYPE;
    docOut["hw"] = HW_REVISION;
    docOut["fw"] = FW_VERSION;


    JsonObject axes = docOut.createNestedObject("axes");

    JsonObject pan = axes.createNestedObject("pan");
    pan["type"] = "velocity";
    pan["range_mm"] = SOFT_MAX_MM;

    JsonObject track = axes.createNestedObject("track");
    track["type"] = "position";
    track["range_mm"] = SOFT_MAX_MM;

    docOut["version"] = "1.0";

    String out;
    serializeJson(docOut, out);
    Serial2.println(out);
    Serial.println("Sending hello reply to WT32...");
    return;
  }

  // BST RAMP / MOTION SETTINGS
  if (strcmp(cmd, "setMotion") == 0) {
    if (doc.containsKey("accel_mm_s2")) {
      bst_accel_mm_s2 = doc["accel_mm_s2"].as<float>();
      Serial.printf("BST ACCEL RECEIVED: %.2f\n", bst_accel_mm_s2);
    }

    if (doc.containsKey("max_vel_mm_s")) {
      bst_max_vel_mm_s = doc["max_vel_mm_s"].as<float>();
      Serial.printf("BST MAX VELOCITY RECEIVED: %.2f\n", bst_max_vel_mm_s);
    }

    return;
  }

  // Joystick speed
  if (strcmp(cmd, "setSpeed") == 0) {
    // ignore joystick while a shot or abs move is running
    if (inShot || inPositionMove) {
      Serial.println("setSpeed ignored: motion program active");
      return;
    }

    float sp = doc["speed"] | 0.0f;
    if (sp < -1.0f) sp = -1.0f;
    if (sp > 1.0f) sp = 1.0f;
    joystick_speed = sp;
    return;
  }


  // Homing
  if (strcmp(cmd, "home") == 0) {
    startHoming();
    return;
  }

  // Stop
  if (strcmp(cmd, "stop") == 0) {
    stopMotion();
    return;
  }

  // Absolute move
  if (strcmp(cmd, "moveAbs") == 0) {
    float pos_mm = doc["pos_mm"] | 0.0f;
    float sp = doc["speed"] | 50.0f;
    moveTo_mm(pos_mm, sp);
    return;
  }

  // Relative move
  if (strcmp(cmd, "moveRel") == 0) {
    float d_mm = doc["delta_mm"] | 0.0f;
    float sp = doc["speed"] | 0.0f;
    moveRel_mm(d_mm, sp);
    return;
  }

  // Fault clear
  if (strcmp(cmd, "clearFault") == 0) {
    fault = false;
    return;
  }

  // Rehome
  if (strcmp(cmd, "rehome") == 0) {
    if (moving) stopMotion();
    fault = false;
    homed = false;
    startHoming();
    return;
  }
  if (strcmp(cmd, "set_ip") == 0) {
    if (doc.containsKey("ip")) {
      JsonObject ipObj = doc["ip"];
      gateway_ip = ipObj["gateway"].as<String>();

      prefs.putString("last_ip", gateway_ip);  // save it
      Serial.print("Gateway IP updated: ");
      Serial.println(gateway_ip);

      if (oledInitialized) {
        updateDisplayIP();
      }
    }
    return;
  }

  //Send current slider configs to gateway
  if (strcmp(cmd, "get_config") == 0) {
    StaticJsonDocument<256> doc;
    Serial.println("DRIVER: Sending CONFIG");

    doc["cmd"] = "config";
    doc["steps_rev"] = steps_per_rev;
    doc["microsteps"] = microsteps;
    doc["mm_rev"] = mm_per_rev;
    doc["steps_mm"] = steps_per_mm;

    doc["max_speed"] = maxSpeed_mm_s;
    doc["accel"] = accel_mm_s2;
    doc["bst_max_vel"] = bst_max_vel_mm_s;
    doc["bst_accel"] = bst_accel_mm_s2;
    doc["invert_dir"] = invertDirection;

    doc["home_offset"] = LOGICAL_HOME_MM;
    doc["soft_max"] = SOFT_MAX_MM;

    String out;
    serializeJson(doc, out);
    Serial2.println(out);
  }

  //Apply new config values from gateway store to memory,  Save and reboot
  if (strcmp(cmd, "set_config") == 0) {
    JsonObject cfg = doc["cfg"];

    steps_per_rev = cfg["steps_rev"];
    mm_per_rev = cfg["mm_rev"];

    maxSpeed_mm_s = cfg["max_speed"];
    accel_mm_s2 = cfg["accel"];

    bst_max_vel_mm_s = cfg["bst_max_vel"];
    bst_accel_mm_s2 = cfg["bst_accel"];
    invertDirection = cfg["invert_dir"];

    LOGICAL_HOME_MM = cfg["home_offset"];
    SOFT_MIN_MM = LOGICAL_HOME_MM;
    SOFT_MAX_MM = cfg["soft_max"];

    steps_per_mm = (steps_per_rev * microsteps) / mm_per_rev;

    saveConfigToNVS();  // ← MUST happen AFTER assignments

    delay(200);  // ← MUST exist
    esp_restart();
  }
}



// ---------------------------------------------------------
// UART2 READER (JSON over Serial2)
// ---------------------------------------------------------
void readUart2() {
  //if (inOtaUpdate) return;  // 🔒 don't consume OTA stream

  while (Serial2.available()) {
    //Serial.println("Serial2 Available");
    char c = Serial2.read();
    if (c == '\n') {
      String line = uart2_rx;
      uart2_rx = "";
      line.trim();

      if (line.length() > 0) {
        Serial.print("UART2 RX: ");
        Serial.println(line);
        handleJsonCommand(line);
      }

    } else {
      uart2_rx += c;
      if (uart2_rx.length() > 512) uart2_rx = "";
    }
  }
}


// ---------------------------------------------------------
// STATUS JSON
// ---------------------------------------------------------
void sendStatus() {
  StaticJsonDocument<256> doc;
  doc["status"] = fault ? "error" : "ok";
  doc["homed"] = homed;
  doc["moving"] = moving;
  doc["pos_mm"] = logical_mm;
  doc["fault"] = fault;
  doc["stall"] = stall;
  doc["limit"] = limitHit;
  doc["move_complete"] = inPositionMove == false && moving == false;
  const char* home_state_str = "idle";
  switch (homeState) {
    case HOME_IDLE: home_state_str = "idle"; break;
    case HOME_MOVING: home_state_str = "moving"; break;
    case HOME_DONE: home_state_str = "done"; break;
    case HOME_FAULT: home_state_str = "fault"; break;
  }
  doc["home_state"] = home_state_str;

  String out;
  serializeJson(doc, out);
  Serial2.println(out);
}

// ---------------------------------------------------------
// LOCAL SERIAL (USB debug)
// ---------------------------------------------------------
void handleLocalSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "u") {
    moveRel_mm(+10.0f, maxSpeed_mm_s);
  } else if (cmd == "d") {
    moveRel_mm(-174.0f, maxSpeed_mm_s);
  } else if (cmd == "s") {
    stopMotion();
  } else if (cmd == "h") {
    startHoming();
  } else if (cmd.startsWith("g")) {
    float pos = cmd.substring(1).toFloat();
    moveTo_mm(pos, maxSpeed_mm_s);
  } else if (cmd.startsWith("v")) {
    float v = cmd.substring(1).toFloat();
    if (v < -1.0f) v = -1.0f;
    if (v > 1.0f) v = 1.0f;
    joystick_speed = v;
  } else if (cmd == "p") {
    Serial.printf("Stepper pos: %.3f mm (%ld steps)\n", logical_mm, logical_steps);
    Serial.printf("Encoder pos: %.3f mm\n", encoder_mm);
  } else if (cmd == "r") {
    uint16_t raw = readAS5600Raw();
    Serial.printf("Raw angle: %u\n", raw);
  } else if (cmd == "f") {
    fault = false;
  } else if (cmd == "H") {
    fault = false;
    homed = false;
    startHoming();
  }
}

// ---------------------------------------------------------
// OTA RECEIVER (over Serial2)
// ---------------------------------------------------------
static const size_t MAX_CHUNK_SIZE = 2048;
uint8_t chunkBuffer[MAX_CHUNK_SIZE];

uint32_t crc32_le(uint32_t crc, const uint8_t* buf, size_t len) {
  crc = ~crc;
  while (len--) {
    crc ^= *buf++;
    for (int i = 0; i < 8; i++) {
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
  }
  return ~crc;
}

bool readBytesExact(Stream& s, uint8_t* buf, size_t len, uint32_t timeout = 20000) {
  uint32_t start = millis();
  size_t received = 0;

  while (received < len && (millis() - start) < timeout) {
    int n = s.available();
    if (n > 0) {
      int toRead = (n > (int)(len - received)) ? (len - received) : n;
      int r = s.readBytes(buf + received, toRead);
      received += r;
    } else {
      delay(1);
    }
  }

  return received == len;
}



void sendACK() {
  Serial2.write(0x06);
}
void sendNACK() {
  Serial2.write(0x15);
}

// NOTE: sync bytes (0x55 0xAA) are now consumed by the OTA sync detector.
// This function now ONLY reads fwSize, fwCRC, chunkSize.
bool receiveMetadata(size_t& fwSize, uint32_t& fwCRC, uint16_t& chunkSize) {

  Serial.println("[ESP32] Waiting for metadata...");

  if (!readBytesExact(Serial2, (uint8_t*)&fwSize, 4)) {
    Serial.println("[ESP32] ERROR: Failed to read fwSize");
    return false;
  }
  if (!readBytesExact(Serial2, (uint8_t*)&fwCRC, 4)) {
    Serial.println("[ESP32] ERROR: Failed to read fwCRC");
    return false;
  }
  if (!readBytesExact(Serial2, (uint8_t*)&chunkSize, 2)) {
    Serial.println("[ESP32] ERROR: Failed to read chunkSize");
    return false;
  }

  Serial.printf("[ESP32] Metadata received: size=%u CRC=%08X chunk=%u\n",
                fwSize, fwCRC, chunkSize);

  if (chunkSize == 0 || chunkSize > MAX_CHUNK_SIZE) {
    Serial.printf("[ESP32] ERROR: Invalid chunk size %u (max %u)\n",
                  chunkSize, (unsigned)MAX_CHUNK_SIZE);
    return false;
  }

  sendACK();

  // 🔥 Flush anything that arrived early (partial first chunk, stray bytes)
  while (Serial2.available()) Serial2.read();
  delay(2);

  return true;
}




void startFirmwareUpdate() {
  stepper->forceStop();
  stepper->disableOutputs();
  otaInProgress = true;  // 🔥 BLOCK checkOtaSync()


  // 🔥 PAUSE MOTION + ISRs
  // (adapt these to your actual handles / pins)
  // Example:
  //engine.disableAllSteppers()



  // detachInterrupt(encoderPinA);
  // detachInterrupt(encoderPinB);
  // vTaskSuspend(motionTaskHandle);
  // vTaskSuspend(encoderTaskHandle);
  // vTaskSuspend(tmcTaskHandle);

  Serial.println("[ESP32] Starting firmware update...");

  size_t fwSize;
  uint32_t fwCRC;
  uint16_t chunkSize;

  // --- RECEIVE METADATA ---
  if (!receiveMetadata(fwSize, fwCRC, chunkSize)) {
    Serial.println("[ESP32] ERROR: Failed to receive metadata");
    otaInProgress = false;
    inOtaUpdate = false;
    return;
  }

  const esp_partition_t* otaPartition = esp_ota_get_next_update_partition(NULL);
  if (!otaPartition) {
    Serial.println("[ESP32] ERROR: No OTA partition found!");
    otaInProgress = false;
    inOtaUpdate = false;
    return;
  }

  Serial.printf("[ESP32] OTA target: label='%s' addr=0x%08X size=%u bytes\n",
                otaPartition->label, otaPartition->address, otaPartition->size);

  if (fwSize > otaPartition->size) {
    Serial.printf("[ESP32] ERROR: FW size %u exceeds partition size %u\n",
                  fwSize, otaPartition->size);
    otaInProgress = false;
    inOtaUpdate = false;
    return;
  }

  esp_ota_handle_t otaHandle;
  esp_err_t err = esp_ota_begin(otaPartition, OTA_SIZE_UNKNOWN, &otaHandle);
  if (err != ESP_OK) {
    Serial.printf("[ESP32] ERROR: esp_ota_begin failed: 0x%X\n", err);
    otaInProgress = false;
    inOtaUpdate = false;
    return;
  }

  Serial.println("[ESP32] OTA begin OK");



  size_t received = 0;
  uint32_t runningCRC = 0;
  bool firstChunk = true;
  bool otaError = false;

  // --- MAIN OTA LOOP ---
  while (received < fwSize) {

    // --- READ CHUNK LENGTH ---
    uint16_t len;
    if (!readBytesExact(Serial2, (uint8_t*)&len, 2)) {
      Serial.println("[ESP32] ERROR: Failed to read chunk length");
      sendNACK();
      otaError = true;
      break;
    }

    // --- FIRST CHUNK VALIDATION ---
    if (firstChunk) {
      if (len == 0 || len > 2048) {
        Serial.printf("[ESP32] ERROR: Bogus first len=%u, aborting OTA\n", len);
        sendNACK();
        otaError = true;
        break;  // 🔥 DO NOT continue OTA protocol
      }
      firstChunk = false;
    }

    Serial.printf("[ESP32] Chunk header len=%u\n", len);

    if (len == 0 || len > chunkSize || len > MAX_CHUNK_SIZE) {
      Serial.printf("[ESP32] ERROR: Invalid chunk len %u\n", len);
      sendNACK();
      otaError = true;
      break;
    }

    // --- READ CHUNK DATA ---
    if (!readBytesExact(Serial2, chunkBuffer, len)) {
      Serial.println("[ESP32] ERROR: Failed to read chunk data");
      sendNACK();
      otaError = true;
      break;
    }
    // 🔥 ADD THIS — only dump first chunk
    static bool dumped = false;
    if (!dumped) {
      Serial.println("[ESP32] First chunk dump:");
      for (int i = 0; i < len; i++) {
        if (i % 16 == 0) Serial.println();
        Serial.printf("%02X ", chunkBuffer[i]);
      }
      Serial.println();
      dumped = true;
    }
    // --- READ CHUNK CRC ---
    uint32_t chunkCRC;
    if (!readBytesExact(Serial2, (uint8_t*)&chunkCRC, 4)) {
      Serial.println("[ESP32] ERROR: Failed to read chunk CRC");
      sendNACK();
      otaError = true;
      break;
    }

    // --- VERIFY CRC ---
    uint32_t calcCRC = crc32_le(0, chunkBuffer, len);
    if (calcCRC != chunkCRC) {
      Serial.printf("[ESP32] CRC FAIL: expected %08X got %08X\n", chunkCRC, calcCRC);
      sendNACK();
      otaError = true;
      break;
    }

    // --- WRITE TO FLASH ---
    if (esp_ota_write(otaHandle, chunkBuffer, len) != ESP_OK) {
      Serial.println("[ESP32] ERROR: esp_ota_write failed");
      sendNACK();
      otaError = true;
      break;
    }

    received += len;
    runningCRC = crc32_le(runningCRC, chunkBuffer, len);

    Serial.printf("[ESP32] Chunk OK (%u/%u)\n", received, fwSize);
    sendACK();
  }

  // --- CLEAN ABORT PATH ---
  if (otaError) {
    Serial.println("[ESP32] OTA aborted, cleaning UART...");
    while (Serial2.available()) Serial2.read();  // 🔥 flush JSON/hello

    otaInProgress = false;
    inOtaUpdate = false;
    return;
  }

  // --- READ END MARKER ---
  uint8_t endMarker[2];
  if (!readBytesExact(Serial2, endMarker, 2)) {
    Serial.println("[ESP32] ERROR: Failed to read END marker");
    otaInProgress = false;
    inOtaUpdate = false;
    return;
  }

  if (endMarker[0] != 0xEE || endMarker[1] != 0xEE) {
    Serial.printf("[ESP32] ERROR: Bad END marker: %02X %02X\n",
                  endMarker[0], endMarker[1]);
    otaInProgress = false;
    inOtaUpdate = false;
    return;
  }

  Serial.println("[ESP32] END marker OK");
  sendACK();

  // --- FINALIZE OTA ---
  if (esp_ota_end(otaHandle) != ESP_OK) {
    Serial.println("[ESP32] ERROR: esp_ota_end failed");
    otaInProgress = false;
    inOtaUpdate = false;
    return;
  }

  if (esp_ota_set_boot_partition(otaPartition) != ESP_OK) {
    Serial.println("[ESP32] ERROR: Failed to set boot partition");
    otaInProgress = false;
    inOtaUpdate = false;
    return;
  }

  Serial.println("[ESP32] OTA update complete. Rebooting...");
  delay(200);

  otaInProgress = false;
  inOtaUpdate = false;
  esp_restart();
}



// ---------------------------------------------------------
// OTA SYNC DETECTOR
// ---------------------------------------------------------
void checkOtaSync() {
  if (otaInProgress) return;

  if (Serial2.available() < 2) return;

  uint8_t b1 = Serial2.peek();
  if (b1 != 0x55) return;

  Serial2.read();  // consume b1

  if (Serial2.available() == 0) {
    uart2_rx += char(b1);
    return;
  }

  uint8_t b2 = Serial2.peek();
  if (b2 != 0xAA) {
    Serial2.read();  // consume b2
    uart2_rx += char(b1);
    uart2_rx += char(b2);
    return;
  }

  // REAL OTA SYNC DETECTED
  while (Serial2.available()) {
    Serial2.read();  // flush everything, including b2
  }

  Serial.println("[ESP32] OTA SYNC DETECTED");

  otaInProgress = true;
  inOtaUpdate = true;
  startFirmwareUpdate();
  otaInProgress = false;
  inOtaUpdate = false;
}






// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
  pinMode(MOTORS_EN_PIN, OUTPUT);
  pinMode(STEPPER_HOME_PIN, INPUT_PULLUP);

  Serial.begin(115200);


  pinMode(ENC_CS, OUTPUT);
  digitalWrite(ENC_CS, HIGH);

  SPI.begin(ENC_SCK, ENC_MISO, ENC_MOSI);
  SPI.setFrequency(1000000);
  SPI.setDataMode(SPI_MODE1);

  setupStepper();
  applyMotionLimits();

  Serial2.begin(115200, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

  //Load slider config from memory
  Serial.println("=== DRIVER BOOT ===");

  loadConfigFromNVS();
  Serial.print("Loaded home_offset from NVS = ");
  Serial.println(LOGICAL_HOME_MM);

  Serial.println("ESP32 Slider Node (1/64, standalone TMC2209 + OTA )");
  Serial.print("Firmware Version:  ");
  Serial.print(DEVICE_TYPE);
  Serial.print("_");
  Serial.print(HW_REVISION);
  Serial.print("_");
  Serial.println(FW_VERSION);

  delay(300);

  //===============Setup OLED==============
  Serial.println("Starting OLED init...");

  Wire.begin(21, 22);
  Wire.endTransmission(true);  // release bus
  selectMuxChannel(5);         // OLED on channel 5
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
  } else {
    //Serial.println("OLED init OK");
    display.clearDisplay();
    //display.setFont(&FreeMonoOblique9pt7b);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(35, 20);
    display.println("BST_Boot..");
    display.display();

    oledInitialized = true;
  }
  prefs.begin("driver", false);
  gateway_ip = prefs.getString("last_ip", "NO IP");
  prefs.end();
  //pinMode(TMC2209_UART_PIN, INPUT_PULLUP);  // disable UART line
  //===============Start Homming==============
  Serial.print("[Slider Firmware Version] = ");
Serial.println(FW_VERSION);
  
  startHoming();
}

// ---------------------------------------------------------
// LOOP
// ---------------------------------------------------------
void loop() {
  if (forceLoopExit) {
    forceLoopExit = false;
    return;
  }
  // Serial2.println("{\"cmd\":\"test\",\"x\":123}");
  //   delay(1000);
  static unsigned long lastStatus = 0;

  // CORE UPDATES
  updateHoming();
  handleLocalSerial();

  // OTA PRE-EMPTION (robust sync detector BEFORE JSON parsing)
  checkOtaSync();
  // If we're in OTA, bail out and let the OTA code own Serial2
  if (otaInProgress) {
    return;
  }
  // APPLY INVERT DIRECTION HERE
  float effective_joystick_speed = joystick_speed;
  if (invertDirection) {
    effective_joystick_speed = -effective_joystick_speed;
  }
  // JSON + encoder + position
  readUart2();
  updatePositionFromEncoder();

  if (stepper) {
    long steps = stepper->getCurrentPosition();
    float mm = steps / steps_per_mm;
    logical_steps = steps;
    logical_mm = mm;
  }



  // LIMIT PROTECTION
  if (homed) {
    if (inPositionMove) {
      if (abs_target_mm < SOFT_MIN_MM || abs_target_mm > SOFT_MAX_MM) {
        stopMotion();
        inPositionMove = false;
      }
    } else {
      if (logical_mm <= SOFT_MIN_MM && effective_joystick_speed < 0) {
        stopMotion();
        if (stepper) stepper->setSpeedInHz(0);
      }
      if (logical_mm >= SOFT_MAX_MM && effective_joystick_speed > 0) {
        stopMotion();
        if (stepper) stepper->setSpeedInHz(0);
      }
    }
  }


  // ---------------------------------------------------------
  // ABSOLUTE MOVE COMPLETION (robust version)
  // ---------------------------------------------------------
  // ABSOLUTE MOVE COMPLETION (robust)
  if (inPositionMove) {
    // Don't evaluate completion in the first 50 ms after arming
    if (millis() - moveStartTime < 50) {
      // let setContinuousSpeed() ramp up first
      // and give logical_mm a chance to change
      return;
    }

    float pos = logical_mm;
    float err = abs_target_mm - pos;

    const float POS_TOL_MM = 0.5f;
    bool close_enough = fabs(err) < POS_TOL_MM;






    // Only treat it as "wrong direction" if we're actually moving
    const float SPEED_EPS = 0.5f;  // mm/s
    bool wrong_dir =
      (fabs(current_speed_mm_s) > SPEED_EPS) && ((err > 0 && current_speed_mm_s < 0) || (err < 0 && current_speed_mm_s > 0));


    if (close_enough || wrong_dir) {
      joystick_speed = 0.0f;
      current_speed_mm_s = 0.0f;
      stepper->forceStop();
      inPositionMove = false;
      moving = false;

      unsigned long duration = millis() - moveStartTime;

      // ⭐ ALWAYS send move_complete event to gateway
      StaticJsonDocument<128> evt;
      evt["event"] = "move_complete";
      evt["axis"] = "track";
      String out;
      serializeJson(evt, out);
      Serial2.println(out);

      // Debug print
      Serial.printf("MOVE COMPLETE: pos=%.2f target=%.2f err=%.2f dur=%.3f s\n",
                    pos, abs_target_mm, err, duration / 1000.0f);
    }
  }



  // SHOT MODE (timed velocity move)
  if (inShot) {
    if (millis() >= shotEndTime) {
      inShot = false;
      shotSpeed = 0.0f;
      joystick_speed = 0.0f;
      stepper->forceStop();
      moving = false;
      Serial.println("SHOT COMPLETE");
    } else {
      joystick_speed = shotSpeed;
    }
  }

  // APPLY ACCELERATION CORRECTLY
  if (!inShot && !inPositionMove) {
    // Joystick mode → use BST ramp acceleration
    accel_mm_s2 = bst_accel_mm_s2;
  } else {
    // Shots + ABS moves → use fixed acceleration
    accel_mm_s2 = DEFAULT_ACCEL_MM_S2;
  }

  applyMotionLimits();

  // DRIVE MOTOR (NO MAX VELOCITY SCALING)
  setContinuousSpeed(effective_joystick_speed);

  // STATUS UPDATE
  if (!inOtaUpdate && millis() - lastStatus > 200) {
    lastStatus = millis();
    sendStatus();
  }
  if (otaInProgress) return;
  updateDisplayIP();
}

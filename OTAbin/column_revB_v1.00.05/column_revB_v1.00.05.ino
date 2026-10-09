// ======================================================
//  ESP32 DC Lift Node — Quadrature Encoder + Encoder Homing
//  + UART2 OTA Receiver (from WT32)
//  Adapted for Pololu H2 36v11 (dual-PWM, SLP control)
// ======================================================
//This version Fixes the encoder issue

#include "driver/gpio.h"
#include "driver/ledc.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
Preferences prefs;

// ====== Device identity ========   This must be kept up to date and added to the manifest whenever an update is issued
#define DEVICE_TYPE "column"  // or "slider", "ptz", "dolly", "jib"
#define HW_REVISION "revB"    //Board Hardware revision
#define FW_VERSION "1.00.05"  //Firmware revisioning system for OTA (main, secondary, Patch)



// ======================================================
//                     PIN DEFINITIONS
// ======================================================

// Pololu 36v11 motor driver (H2)
#define MOTOR_PWMA_PIN 26   // controls OUTA
#define MOTOR_PWMB_PIN 27   // controls OUTB
#define MOTOR_SLP_PIN 13    // sleep (active HIGH, we control it)
#define MOTOR_CS_PIN 34     // current sense (optional)
#define MOTOR_FAULT_PIN 25  // fault (active LOW)

// Quadrature hall sensors
#define HALL_A_PIN 32
#define HALL_B_PIN 33

// Manual switches (active LOW)
#define MANUAL_UP_PIN 15
#define MANUAL_DOWN_PIN 23

// WT32 Bootloader pins (unused here, but kept)
#define WT32_BOOT_PIN 21
#define WT32_EN_PIN 4

// UART2 (OTA from WT32)
#define ESP32_RX_PIN 16
#define ESP32_TX_PIN 17

//OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// ======================================================
//                     PWM CONFIG (ESP‑IDF LEDC)
// ======================================================
#define PWM_FREQ 20000
#define PWM_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE

#define PWM_CHANNEL_A LEDC_CHANNEL_1
#define PWM_CHANNEL_B LEDC_CHANNEL_2



// ======================================================
// QUADRATURE ENCODER Globals
// ======================================================

volatile long hall_count = 0;
volatile uint32_t lastHallEdge_us = 0;

// Previous quadrature state
volatile uint8_t lastAB = 0;

// Transition table
//
// State encoding:
//   00 = 0
//   01 = 1
//   10 = 2
//   11 = 3
//
// Index = (oldState << 2) | newState
//
const int8_t quadTable[16] = {
  0, +1, -1, 0,
  -1, 0, 0, +1,
  +1, 0, 0, -1,
  0, -1, +1, 0
};




// ======================================================
//                 CONSTANTS & GLOBALS
// ======================================================
// --- OLED IP scroll state ---
int ipScrollX = 35;              // current X position
uint16_t ipTextWidth = 0;        // measured width of the IP text
unsigned long lastIpScroll = 0;  // timestamp for non-blocking scroll
bool ipShouldScroll = false;     // whether scrolling is needed
String lastIpShown = "";         // detect IP changes






bool oledInitialized = false;
bool oledReady = false;


String gateway_ip = "0.0.0.0";  //So the IP can be viewed on the OLED


volatile bool otaInProgress = false;
volatile bool inOtaUpdate = false;


float requested_mm_per_s = 10.0f;  // default safe speed

float integrated_mm = 0.0f;
unsigned long lastPosUpdate_us = 0;

// From calibration: 120851 counts / 44 mm
constexpr float COUNTS_PER_MM = 14.0f;  // This is the scale factor between the hall sensor and the real world mm
constexpr float MM_PER_COUNT = 1.0f / COUNTS_PER_MM;

//volatile long hall_count = 0;

const float SOFT_MIN_MM = 0.0f;
const float SOFT_MAX_MM = 975.0f;

float joystick_speed = 0.0f;
float current_speed_norm = 0.0f;

const float MAX_SPEED_NORM = 1.0f;
const float ACCEL_NORM_PER_S = 1.0f;
const int MAX_PWM = 400;  // Motor speed limiter (0..1023 with 10-bit PWM)

unsigned long lastMotionUpdate_us = 0;

bool homed = false;
bool homingActive = false;
unsigned long homingStart = 0;

bool moving = false;
bool fault = false;
bool stall = false;
int lastDirection = 0;  // -1 = down, +1 = up, 0 = stopped

float logical_mm = 0.0f;

unsigned long lastHallEdge_ms = 0;
const unsigned long HALL_TIMEOUT_MS = 3000;
const unsigned long HOMING_TIMEOUT_MS = 90000;

bool inPositionMove = false;
float abs_target_mm = 0.0f;

String uart2_rx;
unsigned long lastStatus_ms = 0;

// Homing back‑off
long lastHomeCount = 0;
unsigned long lastHomeCheck = 0;
// const int BACKOFF_COUNTS = 1500;  // adjust if needed
const int BACKOFF_COUNTS = (int)(25.0f * COUNTS_PER_MM + 0.5f);  // FIX: ~25 mm backoff



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

// ======================================================
//                     Update OLED
// ======================================================
void updateDisplayIP() {
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





// ======================================================
//                     Check OTA Sync
// ======================================================
void checkOtaSync() {
  if (otaInProgress) return;

  // Need at least 2 bytes to check for sync
  if (Serial2.available() < 2) return;

  // Peek first byte only
  uint8_t b1 = Serial2.peek();
  if (b1 != 0x55) return;  // Not OTA, do nothing

  // Read first byte (safe now)
  Serial2.read();

  // Now check second byte (must be available)
  if (Serial2.available() == 0) {
    // Not enough data yet — put the byte back into JSON buffer
    uart2_rx += char(b1);
    return;
  }

  uint8_t b2 = Serial2.peek();
  if (b2 != 0xAA) {
    // Not OTA sync — both bytes belong to JSON
    Serial2.read();  // consume b2
    uart2_rx += char(b1);
    uart2_rx += char(b2);
    return;
  }

  // --- REAL OTA SYNC DETECTED ---
  // Flush ALL leftover bytes so OTA starts clean
  while (Serial2.available()) {
    Serial2.read();
  }

  otaInProgress = true;
  startFirmwareUpdate();
  otaInProgress = false;
}









// ======================================================
//                     QUADRATURE ISR
// ======================================================

// void IRAM_ATTR hallISR_A() {
//   int a = digitalRead(HALL_A_PIN);
//   int b = digitalRead(HALL_B_PIN);
//   if (a == b) hall_count++;
//   else hall_count--;
//   lastHallEdge_ms = millis();
// }

// void IRAM_ATTR hallISR_B() {
//   int a = digitalRead(HALL_A_PIN);
//   int b = digitalRead(HALL_B_PIN);
//   if (a != b) hall_count++;
//   else hall_count--;
//   lastHallEdge_ms = millis();
// }

void IRAM_ATTR hallISR() {
  uint8_t a = digitalRead(HALL_A_PIN);
  uint8_t b = digitalRead(HALL_B_PIN);

  uint8_t newAB = (a << 1) | b;
  uint8_t index = (lastAB << 2) | newAB;

  hall_count += quadTable[index];

  lastAB = newAB;
  lastHallEdge_ms = millis();
}




// ======================================================
//                     MOTOR CONTROL (36v11 dual-PWM)
// ======================================================

void stopMotor() {
  ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, 0);
  ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);

  ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, 0);
  ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);

  moving = false;
  current_speed_norm = 0.0f;
}

void driveMotor(float norm_speed) {
  norm_speed = constrain(norm_speed, -1.0f, 1.0f);

  if (fabs(norm_speed) < 0.01f) {
    stopMotor();
    return;
  }

  int pwm = (int)(fabs(norm_speed) * MAX_PWM);
  pwm = constrain(pwm, 0, MAX_PWM);

  if (norm_speed > 0) {
    // UP: OUTA = PWM, OUTB = 0
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, pwm);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);

    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, 0);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);
  } else {
    // DOWN: OUTA = 0, OUTB = PWM
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, 0);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);

    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, pwm);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);
  }

  moving = true;
}

// ======================================================
//                 POSITION UPDATE
// ======================================================

void updateLogicalPosition() {
  logical_mm = hall_count * MM_PER_COUNT;

  // FIX: clamp to valid range so BST never sees nonsense
  if (logical_mm < SOFT_MIN_MM) logical_mm = SOFT_MIN_MM;
  if (logical_mm > SOFT_MAX_MM) logical_mm = SOFT_MAX_MM;
}

// ======================================================
//                     Debug Move Function
// ======================================================

void debugMove300mm() {
  Serial.println("=== DEBUG MOVE: UP 300mm (TIMED) ===");
  delay(2000);

  long startHall = hall_count;
  long targetHall = startHall + (long)(300.0f * COUNTS_PER_MM);

  Serial.printf("Pre‑move hall: %ld\n", startHall);

  unsigned long tStart = millis();

  driveMotor(+1.0f);  // run at full power for speed test

  while (hall_count < targetHall) {
    delay(5);
  }

  stopMotor();
  delay(50);

  unsigned long tEnd = millis();
  long endHall = hall_count;

  float mmMoved = (endHall - startHall) * MM_PER_COUNT;
  float elapsed_s = (tEnd - tStart) / 1000.0f;
  float mm_per_s = mmMoved / elapsed_s;

  Serial.println("=== DEBUG RESULT ===");
  Serial.printf("Start hall: %ld\n", startHall);
  Serial.printf("End hall:   %ld\n", endHall);
  Serial.printf("Delta hall: %ld\n", endHall - startHall);
  Serial.printf("Measured mm moved: %.3f\n", mmMoved);
  Serial.printf("Elapsed time: %.3f s\n", elapsed_s);
  Serial.printf("Speed: %.3f mm/s\n", mm_per_s);
  Serial.println("=======================");
}




void debugMove200mm() {
  Serial.println("=== DEBUG MOVE: UP 200mm ===");

  long startHall = hall_count;
  long targetHall = startHall + (long)(200.0f * COUNTS_PER_MM);

  driveMotor(+0.5f);

  while (hall_count < targetHall) {
    // no logical_mm here
    delay(5);
  }

  stopMotor();
  delay(50);

  long endHall = hall_count;

  Serial.println("=== DEBUG RESULT ===");
  Serial.printf("Start hall: %ld\n", startHall);
  Serial.printf("End hall:   %ld\n", endHall);
  Serial.printf("Delta hall: %ld\n", endHall - startHall);

  float mmMoved = (endHall - startHall) * MM_PER_COUNT;
  Serial.printf("Measured mm moved: %.3f\n", mmMoved);

  Serial.println("=======================");
}
// ======================================================
//                 ENCODER‑BASED HOMING (with back‑off)
// ======================================================

void startHoming() {
  Serial.println("=== HOMING START ===");

  fault = false;
  stall = false;

  homed = false;
  homingActive = true;
  homingStart = millis();

  hall_count = 0;
  logical_mm = 0;

  lastHomeCount = hall_count;
  lastHomeCheck = millis();
  lastHallEdge_ms = millis();

  driveMotor(-1.0f);  // DOWN
}

enum HomingState {
  HOMING_DRIVE_DOWN,
  HOMING_BACKOFF
};

HomingState homingState = HOMING_DRIVE_DOWN;
int quietSamples = 0;
unsigned long backoffStart = 0;
const unsigned long BACKOFF_TIMEOUT_MS = 2000;

void updateHoming() {
  if (!homingActive) return;

  unsigned long now = millis();

  if (now - homingStart > HOMING_TIMEOUT_MS) {
    Serial.println("HOMING TIMEOUT");
    stopMotor();
    homingActive = false;
    return;
  }

  switch (homingState) {
    case HOMING_DRIVE_DOWN:
      {
        Serial.printf("hall=%ld\n", hall_count);
        driveMotor(-1.0f);

        if (now - lastHomeCheck >= 100) {
          long delta = hall_count - lastHomeCount;
          lastHomeCount = hall_count;
          lastHomeCheck = now;

          if (abs(delta) < 3) quietSamples++;
          else quietSamples = 0;

          if (quietSamples >= 5) {
            Serial.println("=== HOMING BOTTOM DETECTED ===");
            quietSamples = 0;

            stopMotor();
            hall_count = 0;
            logical_mm = 0;

            Serial.println("Backing off...");

            homingState = HOMING_BACKOFF;
            backoffStart = now;
          }
        }
      }
      break;

    case HOMING_BACKOFF:
      {
        driveMotor(+0.5f);  // UP

        if (hall_count >= BACKOFF_COUNTS || now - backoffStart > BACKOFF_TIMEOUT_MS) {

          stopMotor();

          hall_count = 0;
          integrated_mm = 0;
          logical_mm = 0;

          homingActive = false;
          homed = true;
          fault = false;
          stall = false;

          Serial.println("=== HOMING COMPLETE (zeroed after backoff) ===");
          updateDisplayIP();
          // FIX: remove debug move here; it was shifting the "home" position
          //debugMove200mm();  // DEBUG ONLY AFTER HOMING
        }
      }
      break;
  }
}

// ======================================================
//                     STALL DETECTION
// ======================================================

void checkStall() {
  // Stall detection disabled for testing
}

// ======================================================
//                     ABSOLUTE MOVE
// ======================================================

void moveTo_mm(float target_mm) {
  if (!homed) return;
  if (fault) return;

  target_mm = constrain(target_mm, SOFT_MIN_MM, SOFT_MAX_MM);
  abs_target_mm = target_mm;
  inPositionMove = true;
  joystick_speed = 0.0f;
}

// ======================================================
//         CONTINUOUS SPEED ENGINE
// ======================================================

void setContinuousSpeed(float commanded_norm) {
  if (fault) {
    stopMotor();
    return;
  }

  if (homingActive) return;

  unsigned long now_us = micros();
  if (now_us - lastMotionUpdate_us < 5000) return;
  float dt = (now_us - lastMotionUpdate_us) / 1e6f;
  lastMotionUpdate_us = now_us;

  commanded_norm = constrain(commanded_norm, -1.0f, 1.0f);
  if (fabs(commanded_norm) < 0.05f) commanded_norm = 0.0f;

  float max_delta = ACCEL_NORM_PER_S * dt;
  float delta = commanded_norm - current_speed_norm;
  delta = constrain(delta, -max_delta, max_delta);
  current_speed_norm += delta;

  if (fabs(current_speed_norm) < 0.02f) {
    stopMotor();
    return;
  }

  float pos = logical_mm;
  float safe_speed = current_speed_norm;

  if (pos <= SOFT_MIN_MM && safe_speed > 0) {
    stopMotor();
    return;
  }
  if (pos >= SOFT_MAX_MM && safe_speed < 0) {
    stopMotor();
    return;
  }

  driveMotor(safe_speed);
}

// ======================================================
//                 JSON COMMANDS (UART2)
// ======================================================

void handleJsonCommand(const String& s) {
  Serial.printf("UART2 RX: %s\n", s.c_str());
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, s)) return;

  const char* cmd = doc["cmd"] | "";
  Serial.printf("[CMD] %s\n", cmd);
  Serial.printf("[JSON] %s\n", s.c_str());
  if (strcmp(cmd, "hello") == 0) {
    StaticJsonDocument<256> doc;

    doc["device"] = DEVICE_TYPE;
    doc["hw"] = HW_REVISION;
    doc["fw"] = FW_VERSION;

    JsonObject axes = doc.createNestedObject("axes");

    JsonObject lift = axes.createNestedObject("lift");
    lift["type"] = "position";
    lift["range_mm"] = SOFT_MAX_MM;

    JsonObject velocity = axes.createNestedObject("velocity");
    velocity["type"] = "velocity";
    velocity["range_mm"] = SOFT_MAX_MM;

    String out;
    serializeJson(doc, out);
    Serial2.println(out);
    return;
  }

  if (strcmp(cmd, "home") == 0) {
    startHoming();
    return;
  }
  if (strcmp(cmd, "stop") == 0) {
    stopMotor();
    inPositionMove = false;
    joystick_speed = 0;
    return;
  }
  if (strcmp(cmd, "setSpeed") == 0) {
    joystick_speed = doc["speed"] | 0.0f;
    inPositionMove = false;
    return;
  }
  if (strcmp(cmd, "moveAbs") == 0) {
    float pos = doc["pos_mm"] | 0.0f;
    requested_mm_per_s = doc["speed"] | 10.0f;  // <-- NEW
    moveTo_mm(pos);
    return;
  }
  if (strcmp(cmd, "clearFault") == 0) {
    fault = false;
    stall = false;
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
}



// ======================================================
//                     UART2 READER
// ======================================================

String uart2_json_rx;

#define UART2_BUF_SIZE 512
char uart2Buf[UART2_BUF_SIZE];
size_t uart2Head = 0;

void readUart2Json() {
  if (inOtaUpdate) return;

  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n') {
      uart2Buf[uart2Head] = 0;  // null terminate
      String line = String(uart2Buf);
      uart2Head = 0;

      line.trim();
      if (line.length() > 0) {
        Serial.print("UART2 JSON: ");
        Serial.println(line);
        handleJsonCommand(line);
      }
    } else {
      if (uart2Head < UART2_BUF_SIZE - 1) {
        uart2Buf[uart2Head++] = c;
      } else {
        uart2Head = 0;  // overflow protection
      }
    }
  }
}



// ======================================================
//                     STATUS JSON
// ======================================================

void sendStatus() {
  if (inOtaUpdate) return;  // 🔒 don't spam during OTA
  StaticJsonDocument<256> doc;

  doc["status"] = fault ? "error" : "ok";
  doc["homed"] = homed;
  doc["moving"] = moving;

  doc["pos_mm"] = logical_mm;
  doc["limit"] = false;
  doc["move_complete"] = inPositionMove == false && moving == false;
  if (homingActive) {
    doc["home_state"] = "moving";
  } else if (homed) {
    doc["home_state"] = "done";
  } else {
    doc["home_state"] = "idle";
  }

  String out;
  serializeJson(doc, out);
  Serial2.println(out);
}

// ======================================================
//              OTA RECEIVER (UART2 from WT32)
// ======================================================

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




void sendACK() {
  Serial2.write(0x06);
}
void sendNACK() {
  Serial2.write(0x15);
}

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

  // 🔥 Flush anything that arrived early
  while (Serial2.available()) Serial2.read();
  delay(2);

  return true;
}


void startFirmwareUpdate() {
  Serial.println("[ESP32] Starting firmware update...");

  // 🔒 Block motion and JSON
  stopMotor();
  digitalWrite(MOTOR_SLP_PIN, LOW);
  delay(50);

  otaInProgress = true;
  inOtaUpdate = true;

  size_t fwSize;
  uint32_t fwCRC;
  uint16_t chunkSize;

  if (!receiveMetadata(fwSize, fwCRC, chunkSize)) {
    Serial.println("[ESP32] ERROR: Failed to receive metadata");
    otaInProgress = false;
    inOtaUpdate = false;
    return;
  }

  const esp_partition_t* otaPartition = esp_ota_get_next_update_partition(NULL);



  if (!otaPartition) {
    Serial.println("[ESP32] ERROR: No OTA partition found!");
    return;
  }

  Serial.printf("[ESP32] OTA target: label='%s' addr=0x%08X size=%u bytes\n",
                otaPartition->label, otaPartition->address, otaPartition->size);

  if (fwSize > otaPartition->size) {
    Serial.printf("[ESP32] ERROR: FW size %u exceeds partition size %u\n",
                  fwSize, otaPartition->size);
    return;
  }

  esp_ota_handle_t otaHandle;
  esp_err_t err = esp_ota_begin(otaPartition, OTA_SIZE_UNKNOWN, &otaHandle);
  if (err != ESP_OK) {
    Serial.printf("[ESP32] ERROR: esp_ota_begin failed: 0x%X\n", err);
    return;
  }

  Serial.println("[ESP32] OTA begin OK");

  size_t received = 0;
  uint32_t runningCRC = 0;
  bool firstChunk = true;
  bool otaError = false;

  while (received < fwSize) {

    uint8_t hdr[2];

    if (!readBytesExact(Serial2, hdr, 2)) {
      Serial.println("[ESP32] ERROR: Failed to read chunk length");
      sendNACK();
      otaError = true;
      break;
    }

    uint16_t len = hdr[0] | (hdr[1] << 8);

    if (firstChunk) {
      if (len == 0 || len > 2048) {
        Serial.printf(
          "[ESP32] ERROR: Bogus first len=%u raw=%02X %02X\n",
          len, hdr[0], hdr[1]);
        sendNACK();
        otaError = true;
        break;
      }
      firstChunk = false;
    }

    if (len == 0 || len > chunkSize || len > MAX_CHUNK_SIZE) {

      Serial.printf(
        "[ESP32] ERROR: Invalid chunk len %u raw=%02X %02X (chunkSize=%u max=%u)\n",
        len,
        hdr[0],
        hdr[1],
        chunkSize,
        (unsigned)MAX_CHUNK_SIZE);

      sendNACK();
      otaError = true;
      break;
    }

    if (!readBytesExact(Serial2, chunkBuffer, len)) {
      Serial.println("[ESP32] ERROR: Failed to read chunk data");
      sendNACK();
      otaError = true;
      break;
    }

    uint32_t chunkCRC;
    if (!readBytesExact(Serial2, (uint8_t*)&chunkCRC, 4)) {
      Serial.println("[ESP32] ERROR: Failed to read chunk CRC");
      sendNACK();
      otaError = true;
      break;
    }

    uint32_t calcCRC = crc32_le(0, chunkBuffer, len);
    if (calcCRC != chunkCRC) {
      Serial.printf("[ESP32] CRC FAIL: expected %08X got %08X\n", chunkCRC, calcCRC);
      sendNACK();
      otaError = true;
      break;
    }

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

  if (otaError) {
    Serial.println("[ESP32] OTA aborted, cleaning UART...");
    while (Serial2.available()) Serial2.read();
    otaInProgress = false;
    inOtaUpdate = false;
    digitalWrite(MOTOR_SLP_PIN, HIGH);  // re‑enable driver if you want
    return;
  }


  uint8_t endMarker[2];
  if (!readBytesExact(Serial2, endMarker, 2)) {
    Serial.println("[ESP32] ERROR: Failed to read END marker");
    return;
  }

  if (endMarker[0] != 0xEE || endMarker[1] != 0xEE) {
    Serial.printf("[ESP32] ERROR: Bad END marker: %02X %02X\n",
                  endMarker[0], endMarker[1]);
    return;
  }

  Serial.println("[ESP32] END marker OK");
  sendACK();

  if (esp_ota_end(otaHandle) != ESP_OK) {
    Serial.println("[ESP32] ERROR: esp_ota_end failed");
    return;
  }

  if (esp_ota_set_boot_partition(otaPartition) != ESP_OK) {
    Serial.println("[ESP32] ERROR: Failed to set boot partition");
    return;
  }

  Serial.println("[ESP32] OTA update complete. Rebooting...");
  delay(200);
  otaInProgress = false;
  inOtaUpdate = false;
  esp_restart();
}

// ======================================================
//                         SETUP
// ======================================================

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, ESP32_RX_PIN, ESP32_TX_PIN);
  delay(3000);
  Serial.println("Driver UART online");
  Serial.println("[ESP32] OTA Receiver ready on UART2");

  Serial.print("Firmware Version:  ");
  Serial.print(DEVICE_TYPE);
  Serial.print("_");
  Serial.print(HW_REVISION);
  Serial.print("_");
  Serial.println(FW_VERSION);


  // Motor driver control pins
  pinMode(MOTOR_SLP_PIN, OUTPUT);
  digitalWrite(MOTOR_SLP_PIN, LOW);  // keep driver disabled during boot

  pinMode(MOTOR_CS_PIN, INPUT);
  pinMode(MOTOR_FAULT_PIN, INPUT);

  // LEDC PWM for PWMA and PWMB
  ledc_timer_config_t ledc_timer = {
    .speed_mode = PWM_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num = PWM_TIMER,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channelA = {
    .gpio_num = MOTOR_PWMA_PIN,
    .speed_mode = PWM_MODE,
    .channel = PWM_CHANNEL_A,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = PWM_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channelA);

  ledc_channel_config_t ledc_channelB = {
    .gpio_num = MOTOR_PWMB_PIN,
    .speed_mode = PWM_MODE,
    .channel = PWM_CHANNEL_B,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = PWM_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channelB);

  // Encoder
  pinMode(HALL_A_PIN, INPUT_PULLUP);
  pinMode(HALL_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), hallISR, CHANGE);

  pinMode(MANUAL_UP_PIN, INPUT_PULLUP);
  pinMode(MANUAL_DOWN_PIN, INPUT_PULLUP);

  lastHallEdge_ms = millis();
  lastMotionUpdate_us = micros();


  // Now that everything is configured, enable the motor driver
  digitalWrite(MOTOR_SLP_PIN, HIGH);

  delay(300);
  Serial.println("Starting OLED init...");

  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
  } else {
    //Serial.println("OLED init OK");
    display.clearDisplay();
    //display.setFont(&FreeMonoOblique9pt7b);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(35, 20);
    display.println("Homing..");
    display.display();

    oledInitialized = true;
  }
  prefs.begin("driver", false);
  gateway_ip = prefs.getString("last_ip", "NO IP");


  startHoming();
}

// ======================================================
//                          LOOP
// ======================================================

void loop() {
  // 0. OTA pre‑emption
  checkOtaSync();

  // --- 1. HOMING (exclusive) ---
  if (homingActive) {
    updateHoming();
    return;
  }

  // --- 2. UART2 JSON + position update ---
  readUart2Json();
  updateLogicalPosition();

  if (millis() - lastStatus_ms > 200) {
    lastStatus_ms = millis();
    sendStatus();
  }

  // --- 3. Manual buttons ---
  bool upPressed = (digitalRead(MANUAL_UP_PIN) == LOW);
  bool downPressed = (digitalRead(MANUAL_DOWN_PIN) == LOW);

  if (upPressed && !downPressed) {
    driveMotor(+0.98f);

    // Print encoder while moving UP
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 50) {  // print every 50ms
      lastPrint = millis();
      Serial.printf("Manual UP: hall=%ld  mm=%.2f\n",
                    hall_count,
                    hall_count * MM_PER_COUNT);
    }
    return;
  }

  if (downPressed && !upPressed) {
    driveMotor(-0.98f);

    // Print encoder while moving DOWN
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 50) {
      lastPrint = millis();
      Serial.printf("Manual DOWN: hall=%ld  mm=%.2f\n",
                    hall_count,
                    hall_count * MM_PER_COUNT);
    }
    return;
  }


  // --- 4. ABSOLUTE POSITION MOVE ---
  if (inPositionMove && homed && !fault) {
    float pos = logical_mm;
    float target = abs_target_mm;
    float error = target - pos;

    if (fabs(error) < 0.5f) {
      stopMotor();
      inPositionMove = false;
      return;
    }

    // Convert mm/s → motor power (simple linear mapping)
    // Your lift maxes out around ~16 mm/s, so clamp there.
    float mm_s = requested_mm_per_s;

    // Clamp to safe physical limits
    mm_s = constrain(mm_s, 2.0f, 16.0f);

    // Convert mm/s → motor power (tune this scale if needed)
    float motorPower = mm_s / 11.3f;  // 16 mm/s ≈ 0.53 power

    motorPower = constrain(motorPower, 0.20f, 1.00f);

    if (error > 0) driveMotor(+motorPower);
    else driveMotor(-motorPower);

    return;
  }

  // --- 5. Joystick control ---
  if (fabs(joystick_speed) > 0.05f) {
    float s = joystick_speed;

    if (s > 0) {
      driveMotor(+fabs(s));
    } else {
      driveMotor(-fabs(s));
    }

    return;
  }

  // --- 6. Idle ---
  stopMotor();
  if (otaInProgress) return;
  updateDisplayIP();
}

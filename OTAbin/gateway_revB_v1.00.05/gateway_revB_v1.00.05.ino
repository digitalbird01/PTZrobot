//Universal Gatway with 25 shot presets added
//Added motion Preset recorder
/*Added Microstepping 
This only changes the scale of the joystic inputs not microstepping. Move speeds are still controlled by "duration"
and are not effected by this.
This version published to Github
Added Factory default DNS DHCP IP adressing 
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ETH.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include "manifest_manager.h"
#include <Preferences.h>

// *****CURRENT GATWAY FIRMWARE******* (must be updated for each revision)
#define GATEWAY_DEVICE_TYPE "gateway"
#define GATEWAY_HW_REVISION "revB"
#define GATEWAY_FW_VERSION "1.00.05"


// ================== CONFIG ==================

Preferences netPrefs;

IPAddress cfgIP;
IPAddress cfgGW;
IPAddress cfgMASK;
IPAddress cfgDNS;


// Firmware server (Change this to the location of the update server)

//const char* FW_SERVER = "http://192.168.137.1:8000/"; //DigitalBird Home server

String ESP32_FW_URL;
String WT32_FW_URL;


#define BST_PORT 7606
#define FRAME_END 0x1E

// HTTP update port
#define HTTP_PORT 8080




// Ethernet (WT32-ETH01 / LAN8720)
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_ADDR 1
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_POWER_PIN 16
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN

//Factory reset button
#define RESET_PIN -1
uint32_t resetPressedTime = 0;
bool resetButtonActive = false;

IPAddress recoveryIP(192, 168, 10, 250);
IPAddress recoveryGW(192, 168, 10, 1);
IPAddress recoveryMask(255, 255, 255, 0);
IPAddress recoveryDNS(192, 168, 10, 1);

IPAddress localIP(192, 168, 10, 82);
IPAddress gatewayIP(192, 168, 10, 1);
IPAddress subnetIP(255, 255, 255, 0);
IPAddress dnsIP(192, 168, 10, 1);


// UART2 to driver
HardwareSerial LinkSerial(2);
const int UART2_RX_PIN = 14;
const int UART2_TX_PIN = 15;
const uint32_t UART2_BAUD = 115200;

// ================== GLOBALS ==================
String gatewayHwRevision = "revB";  // or "revA" depending on your hardware

volatile bool driverMoveComplete = false;
volatile bool driverQueryInProgress = false;

String driverHwRevision = "unknown";
String driverFwVersion = "0.0.0";

unsigned long ipConfigSendAt = 0;
bool ipConfigPending = false;

String dynamicManifestURL;  //dynamic URL globals manifest server
String dynamicFWServerURL;  //dynamic URL globals firmware server

TaskHandle_t httpTaskHandle = NULL;

String otaServerIP = "";


String controllerIP = "192.168.10.99";  // default or blank


bool gotDriverHandshake = false;
unsigned long lastHelloSend = 0;

volatile bool updateInProgress = false;

WiFiServer bstServer(BST_PORT);
WiFiClient bstClient;

WiFiServer httpServer(HTTP_PORT);

String bstRxBuffer;
String uart2RxBuffer;

bool ethGotIP = false;


// Position received from driver ESP32
float sliderPosMM = 0.0f;

// ================== DEVICE / AXIS MODEL ==================

struct AxisInfo {
  String name;
  String type;  // "velocity" or "position"
  float range;  // mm or deg
};

String deviceName = "unknown";

AxisInfo velocityAxis;
bool velocityAxisValid = false;

AxisInfo positionAxis;
bool positionAxisValid = false;

// ---- NEW: multi-axis + presets ----

static const int MAX_AXES = 5;
static const int MAX_PRESETS = 24;

int axisCount = 0;
String axisNames[MAX_AXES];  // names from driver handshake
float axisState[MAX_AXES];   // current pos_mm per axis
float axisTarget[MAX_AXES];  // last commanded target per axis
Preferences presetPrefs;
// ================== POSITION ACCESSOR ==================

float getSliderPositionMM() {
  return sliderPosMM;
}

// -----------------------------------------------------------------------------
// Recording / Playback Module
// -----------------------------------------------------------------------------

// State flags
static bool g_recordMode = false;
static bool g_playbackMode = false;
static bool g_loopMode = false;

// Recorded sequence
static const int MAX_RECORDING = 64;
static int g_recordedShots[MAX_RECORDING];
static int g_recordedCount = 0;
static int g_playbackIndex = 0;

// Duration source + preset recall
extern uint32_t getCurrentDuration();
extern void recallPreset(int shot, uint32_t dur);

// -----------------------------------------------------------------------------
// Movement wait helpers
// -----------------------------------------------------------------------------

bool isAxisMoving() {
  for (int i = 0; i < axisCount; i++) {
    float current = axisState[i];
    float target = axisTarget[i];

    if (fabs(current - target) > 1.0f) {
      return true;
    }
  }
  return false;
}

bool waitForMovementToFinish(uint32_t timeoutMs) {

  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    if (driverMoveComplete) {
      Serial.println("[PLAYBACK] Movement finished");
      return true;
    }
    vTaskDelay(10);
  }

  Serial.println("[PLAYBACK] Movement timeout!");
  return false;
}



// -----------------------------------------------------------------------------
// Recording control
// -----------------------------------------------------------------------------

void gatewaySetRecordMode(bool state) {
  if (state == g_recordMode) return;

  g_recordMode = state;
  g_playbackMode = false;  // safety: stop playback when entering record

  if (g_recordMode) {
    g_recordedCount = 0;
    g_playbackIndex = 0;
    Serial.println("[RECORD] START");
  } else {
    Serial.printf("[RECORD] STOP, shots=%d\n", g_recordedCount);
  }
}

// Call this whenever a preset is fired while record mode is ON
void gatewayRecordShot(int shotNumber) {

  Serial.println("Entering gateway record shot");

  Serial.printf(
    "[RECORD] gatewayRecordShot(%d) recordMode=%d count=%d\n",
    shotNumber,
    g_recordMode,
    g_recordedCount);





  if (!g_recordMode) return;
  if (g_recordedCount >= MAX_RECORDING) {
    Serial.println("[RECORD] Buffer full");
    return;
  }

  g_recordedShots[g_recordedCount] = shotNumber;
  g_recordedCount++;

  Serial.printf("[RECORD] Shot %d stored at index %d\n",
                shotNumber, g_recordedCount - 1);
}

// -----------------------------------------------------------------------------
// Playback control
// -----------------------------------------------------------------------------

void gatewaySetPlaybackMode(bool state) {
  if (state == g_playbackMode) return;

  g_playbackMode = state;
  g_playbackIndex = 0;

  if (g_playbackMode) {
    Serial.printf("[PLAYBACK] START, shots=%d\n", g_recordedCount);
  } else {
    Serial.println("[PLAYBACK] STOP");
  }
}

void gatewaySetLoopMode(bool state) {
  g_loopMode = state;
  Serial.printf("[LOOP] %s\n", g_loopMode ? "ON" : "OFF");
}

// -----------------------------------------------------------------------------
// Main playback engine
// -----------------------------------------------------------------------------

void gatewayHandlePlayback() {
  if (!g_playbackMode) return;
  if (g_recordedCount == 0) {
    Serial.println("[PLAYBACK] No shots recorded");
    g_playbackMode = false;
    return;
  }

  // Forward pass
  for (int i = 0; i < g_recordedCount; i++) {
    driverMoveComplete = false;  // ⭐ reset BEFORE sending the move
    int shot = g_recordedShots[i];
    uint32_t dur = getCurrentDuration();  // or per-shot if you add it

    Serial.printf("[PLAYBACK] Shot %d (index %d)\n", shot, i);
    recallPreset(shot, dur);

    waitForMovementToFinish(10000);
    vTaskDelay(50);
  }

  // Optional loop: reverse sequence
  if (g_loopMode) {
    for (int i = g_recordedCount - 1; i >= 0; i--) {

      int shot = g_recordedShots[i];
      uint32_t dur = getCurrentDuration();

      Serial.printf("[PLAYBACK] Loop shot %d (index %d)\n", shot, i);
      recallPreset(shot, dur);

      waitForMovementToFinish(10000);
      vTaskDelay(50);
    }
  }

  // One-shot playback done
  if (!g_loopMode) {
    g_playbackMode = false;
    Serial.println("[PLAYBACK] COMPLETE");
  }
}

// -----------------------------------------------------------------------------
// Preset recall + duration
// -----------------------------------------------------------------------------

void recallPreset(int shot, uint32_t duration) {
  float target[MAX_AXES];
  uint32_t storedDuration = 1;

  if (!presetLoad(shot - 1, axisCount, target, &storedDuration)) {
    Serial.printf("[PLAYBACK] recallPreset: no stored data for shot=%d\n", shot);
    return;
  }

  uint32_t moveDuration = (duration > 0) ? duration : storedDuration;
  if (g_recordMode) {
    gatewayRecordShot(shot);
  }
  moveAxesToPreset(target, axisCount, moveDuration);

  // Update axisTarget[] so movement-wait works
  for (int i = 0; i < axisCount; i++) {
    axisTarget[i] = target[i];
  }
}

uint32_t lastDuration = 1;

uint32_t getCurrentDuration() {
  return lastDuration;
}


// ================== Update Request handeler ==================

void handleUpdateRequest(WiFiClient& client, const String& method, const String& query) {

  // -----------------------------------------
  // 1. Extract SERVER IP from incoming request
  // -----------------------------------------
  IPAddress remote = client.remoteIP();
  otaServerIP = remote.toString();  // <-- GLOBAL STRING
  Serial.printf("[UPDATE] Server IP detected: %s\n", otaServerIP.c_str());

  // -----------------------------------------
  // 2. Build manifest URL using SERVER IP
  // -----------------------------------------
  String manifestUrl = "http://" + otaServerIP + ":8000/manifest.json";
  Serial.printf("[UPDATE] Manifest URL: %s\n", manifestUrl.c_str());



  // -----------------------------------------
  // 4. Respond to server
  // -----------------------------------------
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println();
  client.println("OTA triggered");
}



// ================== Gatway Hello ==================
void sendGatewayHello() {
  if (updateInProgress) return;
  StaticJsonDocument<256> doc;
  doc["device"] = GATEWAY_DEVICE_TYPE;
  doc["hw"] = GATEWAY_HW_REVISION;
  doc["fw"] = GATEWAY_FW_VERSION;

  String out;
  serializeJson(doc, out);

  Serial.print("[GATEWAY] Hello: ");
  Serial.println(out);
}

//===============Send IP info to driver for OLED
void sendIpConfigToDriver() {
  if (updateInProgress) return;

  StaticJsonDocument<192> doc;
  doc["cmd"] = "set_ip";

  JsonObject ip = doc.createNestedObject("ip");
  ip["gateway"] = ETH.localIP().toString();
  ip["gw"] = gatewayIP.toString();
  ip["mask"] = subnetIP.toString();
  ip["dns"] = dnsIP.toString();

  String out;
  serializeJson(doc, out);

  LinkSerial.println(out);
  Serial.println("[GATEWAY] Sent IP config to driver");
}



//==================================================================
//--------- NEW Network Config
//==================================================================

struct NetworkConfig {
  bool valid;      // Config exists
  bool useStatic;  // true = static, false = DHCP

  IPAddress ip;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns;
};

NetworkConfig netCfg;







//==================================================================
//--------- OLD IP config page
//==================================================================

// bool loadNetworkConfig() {
//   netPrefs.begin("netcfg", true);  // read-only
//   String ip = netPrefs.getString("ip", "");
//   String gw = netPrefs.getString("gw", "");
//   String mask = netPrefs.getString("mask", "");
//   String dns = netPrefs.getString("dns", "");
//   String ctrl = netPrefs.getString("controller", "");
//   controllerIP = ctrl;

//   netPrefs.end();

//   if (ip == "" || gw == "" || mask == "" || dns == "") {
//     return false;  // no saved config
//   }

//   cfgIP.fromString(ip);
//   cfgGW.fromString(gw);
//   cfgMASK.fromString(mask);
//   cfgDNS.fromString(dns);
//   return true;
// }

//==================================================================
//--------- NEW IP config page
//==================================================================

bool loadNetworkConfig() {
  Preferences prefs;

  // New format
  if (prefs.begin("network", true)) {
    bool valid = prefs.getBool("valid", false);

    if (valid) {
      netCfg.valid = true;
      netCfg.useStatic = prefs.getBool("static", true);

      netCfg.ip.fromString(prefs.getString("ip", ""));
      netCfg.gateway.fromString(prefs.getString("gw", ""));
      netCfg.subnet.fromString(prefs.getString("mask", ""));
      netCfg.dns.fromString(prefs.getString("dns", ""));

      prefs.end();

      Serial.println("[NET] Loaded NEW config");
      return true;
    }

    prefs.end();
  }

  // Legacy format
  if (prefs.begin("netcfg", true)) {
    String ip = prefs.getString("ip", "");
    String gw = prefs.getString("gw", "");
    String mask = prefs.getString("mask", "");
    String dns = prefs.getString("dns", "");

    prefs.end();

    if (ip.length()) {
      netCfg.valid = true;
      netCfg.useStatic = true;

      netCfg.ip.fromString(ip);
      netCfg.gateway.fromString(gw);
      netCfg.subnet.fromString(mask);
      netCfg.dns.fromString(dns);

      Serial.println("[NET] Loaded LEGACY config");
      return true;
    }
  }

  return false;
}

//==================================================================
//--------- OLD Save IP Confog
//==================================================================

// void saveNetworkConfig(const String& ip, const String& gw, const String& mask, const String& dns) {
//   Serial.println("[NVS] begin()");
//   netPrefs.begin("netcfg", false);

//   Serial.println("[NVS] writing ip");
//   netPrefs.putString("ip", ip);

//   Serial.println("[NVS] writing gw");
//   netPrefs.putString("gw", gw);

//   Serial.println("[NVS] writing mask");
//   netPrefs.putString("mask", mask);

//   Serial.println("[NVS] writing dns");
//   netPrefs.putString("dns", dns);

//   Serial.println("[NVS] writing controller");
//   netPrefs.putString("controller", controllerIP);

//   Serial.println("[NVS] end()");
//   netPrefs.end();

//   Serial.println("[NVS] write complete");
// }

//==================================================================
//--------- NEW Save NetworkConfig
//==================================================================

bool saveNetworkConfig(
  bool useStatic,
  IPAddress ip,
  IPAddress gateway,
  IPAddress subnet,
  IPAddress dns) {
  Preferences prefs;

  if (!prefs.begin("network", false))
    return false;

  prefs.putBool("valid", true);
  prefs.putBool("static", useStatic);

  prefs.putString("ip", ip.toString());
  prefs.putString("gw", gateway.toString());
  prefs.putString("mask", subnet.toString());
  prefs.putString("dns", dns.toString());

  prefs.end();

  return true;
}

//==================================================================
//--------- NEW Clear Configuration (Return to DHCP)
//==================================================================
void clearNetworkConfig() {
  Preferences prefs;

  if (prefs.begin("network", false)) {
    prefs.clear();
    prefs.end();
  }

  Serial.println("[NET] Configuration cleared");
}



//==================================================================
//--------- OLD Network page
//==================================================================
// void handleNetworkPage(WiFiClient& client) {
//   String html =
//     "<html><body>"
//     "<h2>Gateway Network Configuration</h2>"
//     "<form method='POST' action='/network'>"
//     "IP: <input name='ip' value='"
//     + ETH.localIP().toString() + "'><br>"
//                                  "Gateway: <input name='gw' value='"
//     + gatewayIP.toString() + "'><br>"
//                              "Subnet: <input name='mask' value='"
//     + subnetIP.toString() + "'><br>"
//                             "DNS: <input name='dns' value='"
//     + dnsIP.toString() + "'><br><br>"
//                          "Controller IP: <input name='controller' value='"
//     + controllerIP + "'><br><br>"
//                      "<input type='submit' value='Save & Reboot'>"
//                      "</form>"
//                      "</body></html>";

//   client.println("HTTP/1.1 200 OK");
//   client.println("Content-Type: text/html");
//   client.printf("Content-Length: %d\r\n", html.length());
//   client.println("Connection: close");
//   client.println();
//   client.print(html);
// }

//==================================================================
//--------- NEW Network page
//==================================================================
void handleNetworkPage(WiFiClient& client) {
  String html =
    "<html><body>"
    "<h2>Gateway Network Configuration</h2>"
    //"<form method='POST' action='/network"
    "<form method='POST' action='/network'>"


    "IP: <input name='ip' value='"

    + ETH.localIP().toString() + "'><br>"

                                 "Gateway: <input name='gw' value='"
    + gatewayIP.toString() + "'><br>"

                             "Subnet: <input name='mask' value='"
    + subnetIP.toString() + "'><br>"

                            "DNS: <input name='dns' value='"
    + dnsIP.toString() + "'><br><br>"

                         "Controller IP: <input name='controller' value='"
    + controllerIP + "'><br><br>"

                     "<input type='submit' value='Save & Reboot'>"

                     "</form>"

                     "<hr>"

                     "<h3>Network Recovery</h3>"

                     "<p>Clear saved network settings and reboot into Recovery Mode.</p>"

                     "<form method='POST' action='/factoryreset'>"
                    

                     "<input type='submit' value='Clear Static IP & Enter Recovery Mode'>"

                     "</form>"

                     "</body></html>";

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.printf("Content-Length: %d\r\n", html.length());
  client.println("Connection: close");
  client.println();
  client.print(html);
}

String getField(const String& body, const String& key) {
  int s = body.indexOf(key + "=");

  if (s < 0)
    return "";

  s += key.length() + 1;

  int e = body.indexOf('&', s);

  if (e < 0)
    e = body.length();

  return body.substring(s, e);
}
//==================================================================
//--------- NEW handleNetworkPost
//==================================================================

void handleNetworkPost(WiFiClient& client, const String& body) {
  Serial.println("[NETPOST] Entered handleNetworkPost()");
  Serial.printf("[NETPOST] Raw body: %s\n", body.c_str());

  String ip = getField(body, "ip");
  String gw = getField(body, "gw");
  String mask = getField(body, "mask");
  String dns = getField(body, "dns");
  String ctrl = getField(body, "controller");

  controllerIP = ctrl;

  Serial.printf(
    //"[NETPOST] ip=%s gw=%s mask=%s dns=%s ctrl=%s\n",
    ip.c_str(),
    gw.c_str(),
    mask.c_str(),
    dns.c_str(),
    ctrl.c_str());

  // Convert Strings to IPAddress
  IPAddress ipAddr;
  IPAddress gwAddr;
  IPAddress maskAddr;
  IPAddress dnsAddr;

  //Serial.printf("[NETPOST] ip='%s'\n", ip.c_str());
  //Serial.printf("[NETPOST] gw='%s'\n", gw.c_str());
  //Serial.printf("[NETPOST] mask='%s'\n", mask.c_str());
  //Serial.printf("[NETPOST] dns='%s'\n", dns.c_str());
  if (!ipAddr.fromString(ip) || !gwAddr.fromString(gw) || !maskAddr.fromString(mask) || !dnsAddr.fromString(dns)) {
    Serial.println("[NETPOST] Invalid IP address supplied");

    String msg =
      "<html><body><h3>Invalid IP configuration</h3></body></html>";

    client.println("HTTP/1.1 400 Bad Request");
    client.println("Content-Type: text/html");
    client.printf("Content-Length: %d\r\n", msg.length());
    client.println("Connection: close");
    client.println();
    client.print(msg);

    client.stop();
    return;
  }

  Serial.println("[NETPOST] Saving network config...");

  saveNetworkConfig(
    true,  // useStatic
    ipAddr,
    gwAddr,
    maskAddr,
    dnsAddr);

  String msg =
    "<html><body><h3>Saved. Rebooting...</h3></body></html>";

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.printf("Content-Length: %d\r\n", msg.length());
  client.println("Connection: close");
  client.println();
  client.print(msg);

  Serial.println("[NETPOST] HTML sent");

  client.stop();

  Serial.println("[NETPOST] Client stopped, about to reboot...");

  delay(250);

  ESP.restart();
}


//==================================================================
//--------- OLD handleNetworkPost
//==================================================================

// void handleNetworkPost(WiFiClient& client, const String& body) {
//   Serial.println("[NETPOST] Entered handleNetworkPost()");
//   Serial.printf("[NETPOST] Raw body: %s\n", body.c_str());
//   auto getField = [&](const String& key) {
//     int s = body.indexOf(key + "=");
//     if (s < 0) return String("");
//     s += key.length() + 1;
//     int e = body.indexOf('&', s);
//     if (e < 0) e = body.length();
//     return body.substring(s, e);
//   };

//   String ip = getField("ip");
//   String gw = getField("gw");
//   String mask = getField("mask");
//   String dns = getField("dns");
//   String ctrl = getField("controller");

//   controllerIP = ctrl;
//   Serial.printf("[NETPOST] ip=%s gw=%s mask=%s dns=%s ctrl=%s\n",
//                 ip.c_str(), gw.c_str(), mask.c_str(), dns.c_str(), ctrl.c_str());
//   // Save everything in one atomic call

//   saveNetworkConfig(ip, gw, mask, dns);
//   Serial.println("[NETPOST] Saving network config...");
//   String msg = "<html><body><h3>Saved. Rebooting...</h3></body></html>";
//   client.println("HTTP/1.1 200 OK");
//   client.println("Content-Type: text/html");
//   client.printf("Content-Length: %d\r\n", msg.length());
//   client.println("Connection: close");
//   client.println();
//   client.print(msg);

//   Serial.println("[NETPOST] HTML sent");
//   // close the client before reboot
//   client.stop();
//   Serial.println("[NETPOST] Client stopped, about to reboot...");

//   // short pause
//   delay(50);

//   // hardware reset
//   ESP.restart();
//   //esp_deep_sleep(1000);  // 1ms deep sleep → reboot optional method

//   Serial.println("[NETPOST] If you see this, deep sleep did NOT trigger!");
// }


//==================================================================
//--------- PRESET STORAGE (NVS)
//==================================================================

void presetInit() {
  presetPrefs.begin("presets", false);
  presetPrefs.end();
}

void presetStore(int shotIndex, int axisCount, const float* axisState, uint32_t duration) {
  if (shotIndex < 0 || shotIndex >= MAX_PRESETS) return;

  presetPrefs.begin("presets", false);

  char key[16];

  // axis positions
  for (int i = 0; i < axisCount; i++) {
    sprintf(key, "p%d_a%d", shotIndex + 1, i);  // shots are 1-based
    presetPrefs.putFloat(key, axisState[i]);
  }

  // duration
  sprintf(key, "p%d_dur", shotIndex + 1);
  presetPrefs.putUInt(key, duration);

  presetPrefs.end();

  Serial.printf("[PRESET] Stored shot=%d axes=%d duration=%u\n",
                shotIndex + 1, axisCount, duration);
}

bool presetLoad(int shotIndex, int axisCount, float* targetOut, uint32_t* durationOut) {
  if (shotIndex < 0 || shotIndex >= MAX_PRESETS) return false;

  presetPrefs.begin("presets", true);

  char key[16];
  bool anyAxis = false;

  for (int i = 0; i < axisCount; i++) {
    sprintf(key, "p%d_a%d", shotIndex + 1, i);
    if (presetPrefs.isKey(key)) {
      targetOut[i] = presetPrefs.getFloat(key, 0.0f);
      anyAxis = true;
    } else {
      targetOut[i] = axisState[i];  // fallback: current pos
    }
  }

  sprintf(key, "p%d_dur", shotIndex + 1);
  *durationOut = presetPrefs.getUInt(key, 1);  // default 1s

  presetPrefs.end();

  Serial.printf("[PRESET] Loaded shot=%d axes=%d duration=%u (anyAxis=%d)\n",
                shotIndex + 1, axisCount, *durationOut, anyAxis);

  return anyAxis;
}

//==================================================================
//--------- Slider config page
//==================================================================
String waitForDriverJson(const char* expectedCmd, uint32_t timeout = 2000) {
  if (updateInProgress) {
    // Driver is in OTA mode — no JSON will be sent
    return "";
  }

  uint32_t start = millis();
  String buf = "";

  while (millis() - start < timeout) {
    while (LinkSerial.available()) {
      char c = LinkSerial.read();

      // Debug: print every character received
      Serial.print(c);

      if (c == '\n') {
        Serial.print("[GATEWAY UART RX LINE] ");
        Serial.println(buf);

        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, buf);

        if (!err) {
          const char* cmd = doc["cmd"];
          if (cmd && strcmp(cmd, expectedCmd) == 0) {
            return buf;
          }
        }

        buf = "";
      } else {
        buf += c;
      }
    }
  }

  return "";
}









void handleSliderConfigPage(WiFiClient& client) {
  if (updateInProgress) {
    client.println("HTTP/1.1 503 Service Unavailable");
    client.println("Content-Type: text/plain");
    client.println();
    client.println("OTA update in progress. Config page unavailable.");
    return;
  }

  // 🔒 Enter critical section: block uart2ReceiveLoop()
  driverQueryInProgress = true;

  // 1. Wait for READY from driver
  // Serial.println("[SLIDER CONFIG] Waiting for READY...");
  // String ready = waitForDriverJson("ready", 8000);

  // Skip READY test above
  // String ready = "";

  //   if (ready.length() == 0) {
  //     Serial.println("[SLIDER CONFIG] No READY received — driver may still be homing");
  //     // Continue anyway — user may want to view defaults
  //   }

  // 2. Request CONFIG
  Serial.println("[SLIDER CONFIG] Requesting CONFIG...");

  while (LinkSerial.available()) {
    LinkSerial.read();
  }
  vTaskDelay(1);

  Serial.println("[SLIDER CONFIG] Requesting CONFIG...");

  LinkSerial.println("{\"cmd\":\"get_config\"}");




  // 3. Wait for CONFIG reply
  String reply = waitForDriverJson("config", 3000);
  Serial.print("[SLIDER CONFIG] CONFIG reply = ");
  Serial.println(reply);

  // 🔓 Leave critical section BEFORE generating HTML
  driverQueryInProgress = false;

  if (reply.length() == 0) {
    Serial.println("[SLIDER CONFIG] No CONFIG received — using defaults");
    reply = "{}";
  }

  // Parse JSON
  StaticJsonDocument<256> doc;
  deserializeJson(doc, reply);

  Serial.printf("PARSED home_offset=%.2f soft_max=%.2f\n", doc["home_offset"] | -1.0f, doc["soft_max"] | -1.0f);

  float steps_rev = doc["steps_rev"] | 200.0f;
  float mm_rev = doc["mm_rev"] | 34.9f;
  float steps_mm = doc["steps_mm"] | 321.3f;
  float max_speed = doc["max_speed"] | 25.0f;
  float accel = doc["accel"] | 600.0f;
  float bst_max_vel = doc["bst_max_vel"] | 200.0f;
  float bst_accel = doc["bst_accel"] | 200.0f;
  bool invert_dir = doc["invert_dir"] | false;
  float home_offset = doc["home_offset"] | 10.0f;
  float soft_max = doc["soft_max"] | 345.0f;

  // Build HTML
  String html =
    "<html><body>"
    "<h2>Slider Configuration</h2>"
    "<form method='POST' action='/slider_config'>"

    "<h3>Mechanical Setup</h3>"
    "<style>.lbl{display:inline-block;width:180px;}</style>"
    "<span class='lbl'>Steps per rev:</span><input name='steps_rev' value='"
    + String(steps_rev) + "'><br>"
                          "<span class='lbl'>Microsteps:</span><input value='64' disabled><br>"
                          "<span class='lbl'>mm per rev:</span><input name='mm_rev' value='"
    + String(mm_rev) + "'><br>"
                       "<span class='lbl'>Steps per mm:</span><input value='"
    + String(steps_mm) + "' disabled><br>"

                         "</table>"

                         "<h3>Motion Settings</h3>"
                         "<table>"
                         "<tr><td>Max speed (mm/s):</td><td><input name='max_speed' value='"
    + String(max_speed) + "'></td></tr>"
                          "<tr><td>Acceleration (mm/s²):</td><td><input name='accel' value='"
    + String(accel) + "'></td></tr>"
                      "<tr><td>Joystick max vel (mm/s):</td><td><input name='bst_max_vel' value='"
    + String(bst_max_vel) + "'></td></tr>"
                            "<tr><td>Joystick accel (mm/s²):</td><td><input name='bst_accel' value='"
    + String(bst_accel) + "'></td></tr>"
                          "<tr><td>Invert direction:</td><td><input type='checkbox' name='invert_dir' "
    + (invert_dir ? "checked" : "") + "></td></tr>"
                                      "</table>"

                                      "<h3>Travel Limits</h3>"
                                      "<table>"
                                      "<tr><td>Logical home offset (mm):</td><td><input name='home_offset' value='"
    + String(home_offset) + "'></td></tr>"
                            "<tr><td>Soft max mm (max travel):</td><td><input name='soft_max' value='"
    + String(soft_max) + "'></td></tr>"
                         "</table>"

                         "<br><input type='submit' value='Save & Reboot'>"
                         "</form>"
                         "</body></html>";

  // Send response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.printf("Content-Length: %d\r\n", html.length());
  client.println("Connection: close");
  client.println();
  client.print(html);
}



void handleSliderConfigPost(WiFiClient& client, const String& body) {
  if (updateInProgress) {
    client.println("HTTP/1.1 503 Service Unavailable");
    client.println("Content-Type: text/plain");
    client.println();
    client.println("OTA update in progress. Config page unavailable.");
    return;
  }

  auto getField = [&](const String& key) {
    int s = body.indexOf(key + "=");
    if (s < 0) return String("");
    s += key.length() + 1;
    int e = body.indexOf('&', s);
    if (e < 0) e = body.length();
    return body.substring(s, e);
  };

  StaticJsonDocument<256> cfg;

  cfg["steps_rev"] = getField("steps_rev").toFloat();
  cfg["mm_rev"] = getField("mm_rev").toFloat();
  cfg["max_speed"] = getField("max_speed").toFloat();
  cfg["accel"] = getField("accel").toFloat();
  cfg["bst_max_vel"] = getField("bst_max_vel").toFloat();
  cfg["bst_accel"] = getField("bst_accel").toFloat();
  cfg["invert_dir"] = body.indexOf("invert_dir=") >= 0;
  cfg["home_offset"] = getField("home_offset").toFloat();
  cfg["soft_max"] = getField("soft_max").toFloat();

  StaticJsonDocument<512> out;
  out["cmd"] = "set_config";
  out["cfg"] = cfg;

  String send;
  serializeJson(out, send);

  Serial.println("=== SENDING CONFIG TO DRIVER ===");
  Serial.println(send);
  Serial.println("===============================");


  LinkSerial.println(send);

  String msg = "<html><body><h3>Saved. Slider rebooting...</h3></body></html>";
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.printf("Content-Length: %d\r\n", msg.length());
  client.println("Connection: close");
  client.println();
  client.print(msg);
}


// ================== SEND SYSTEMSTATUS TO BST ==================

unsigned long lastStatusSend = 0;
void sendTrackStatus() {
  if (updateInProgress) return;
  if (!bstClient.connected()) return;

  unsigned long now = millis();
  if (now - lastStatusSend < 100) return;
  lastStatusSend = now;

  float pos_mm = getSliderPositionMM();
  float maxTravel = positionAxisValid ? positionAxis.range : 345.0f;

  int pos_bts = map(pos_mm, 0.0f, maxTravel, 0, 10000);
  int pos_percent = map(pos_mm, 0.0f, maxTravel, 0, 100);

  StaticJsonDocument<256> doc;

  doc["command"] = "update";
  doc["path"] = "head/systemstatus";

  JsonObject track = doc["data"].createNestedObject("track");
  track["position"] = pos_bts;
  track["positionPercent"] = pos_percent;
  track["minPosition"] = 0;
  track["maxPosition"] = 10000;

  String out;
  serializeJson(doc, out);
  bstClient.print(out);
  bstClient.write(FRAME_END);
}

// ================== ETH EVENT ==================

void ETH_Event(WiFiEvent_t event) {
  Serial.printf("[ETH] Event=%d\n", event);

  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[ETH] Started");
      ETH.setHostname("bst-gateway");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[ETH] PHY Connected");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      ethGotIP = true;
      Serial.print("[ETH] Got IP: ");
      Serial.println(ETH.localIP());
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
      ethGotIP = false;
      Serial.println("[ETH] Disconnected");
      break;

    default:
      break;
  }
}

// ================== DRIVER HANDSHAKE PARSING ==================

void parseDriverHandshake(JsonDocument& doc) {

  if (!doc.containsKey("device")) return;
  gotDriverHandshake = true;
  deviceName = doc["device"].as<String>();

  // 🔥 NEW: read hardware revision + firmware version
  if (doc.containsKey("hw"))
    driverHwRevision = doc["hw"].as<String>();

  if (doc.containsKey("fw"))
    driverFwVersion = doc["fw"].as<String>();

  Serial.printf("[GATEWAY] Device: %s  HW=%s  FW=%s\n",
                deviceName.c_str(),
                driverHwRevision.c_str(),
                driverFwVersion.c_str());

  velocityAxisValid = false;
  positionAxisValid = false;
  axisCount = 0;  // NEW: reset axis list

  if (doc.containsKey("axes")) {
    JsonObject ax = doc["axes"];

    for (JsonPair kv : ax) {
      String axisName = kv.key().c_str();
      JsonObject a = kv.value().as<JsonObject>();

      String type = a["type"] | "";
      float range = a.containsKey("range_mm") ? a["range_mm"].as<float>() : 0;

      Serial.printf("[GATEWAY] Axis %s → type=%s, range=%.1f\n", axisName.c_str(), type.c_str(), range);

      // Track first position axis as "primary" NEW
      if (type == "position") {
        if (!positionAxisValid || axisName == "track" || axisName == "lift") {
          positionAxis.name = axisName;
          positionAxis.type = type;
          positionAxis.range = range;
          positionAxisValid = true;
        }


        if (type == "velocity") {
          if (!velocityAxisValid || axisName == "track" || axisName == "tilt") {
            velocityAxis.name = axisName;
            velocityAxis.type = type;
            velocityAxis.range = range;
            velocityAxisValid = true;
          }
        }

        // NEW: collect all position axes for presets
        if (type == "position" && axisCount < MAX_AXES) {
          axisNames[axisCount] = axisName;
          axisState[axisCount] = 0.0f;  // initial
          axisCount++;
        }
      }
    }
  }

  if (positionAxisValid) {
    Serial.printf("[GATEWAY] Position axis: %s (range=%.1f)\n",
                  positionAxis.name.c_str(), positionAxis.range);
  }
  if (velocityAxisValid) {
    Serial.printf("[GATEWAY] Velocity axis: %s\n",
                  velocityAxis.name.c_str());
  }
  // NEW: send IP config after handshake
  // Instead of sending immediately:
  ipConfigPending = true;
  ipConfigSendAt = millis() + 3000;  // send 3 seconds later
}

// ================== UART2 PARSER ==================

void handleDriverStatus(const String& line) {
  StaticJsonDocument<512> doc;

  if (deserializeJson(doc, line)) return;

  // event-based completion
  if (doc["event"] == "move_complete") {
    driverMoveComplete = true;
    Serial.println("[GATEWAY] move_complete (event)");
    return;
  }

  // status-based completion
  if (doc.containsKey("move_complete") && doc["move_complete"] == true) {
    driverMoveComplete = true;
    //Serial.println("[GATEWAY] move_complete (status)");
  }

  if (doc.containsKey("device")) {
    parseDriverHandshake(doc);
    return;
  }

  if (doc.containsKey("axes")) {
    JsonObject ax = doc["axes"];
    for (int i = 0; i < axisCount; i++) {
      const String& name = axisNames[i];
      if (ax.containsKey(name)) {
        JsonObject a = ax[name];
        if (a.containsKey("pos_mm")) {
          axisState[i] = a["pos_mm"].as<float>();
        }
      }
    }
  }

  if (doc.containsKey("pos_mm")) {
    sliderPosMM = doc["pos_mm"].as<float>();
    if (positionAxisValid && axisCount > 0) {
      axisState[0] = sliderPosMM;
    }
  }
}


void uart2ReceiveLoop() {
  if (updateInProgress) return;

  // ⭐ REMOVE THIS — it blocks all status packets during playback
  if (driverQueryInProgress) return;

  while (LinkSerial.available()) {
    char c = LinkSerial.read();

    if (c == '\n') {
      String line = uart2RxBuffer;  // <-- line exists ONLY here
      uart2RxBuffer = "";
      line.trim();

      if (line.length() > 0) {
        // ⭐ handleDriverStatus() MUST run even during driverQueryInProgress
        handleDriverStatus(line);
      }

    } else {
      uart2RxBuffer += c;
      if (uart2RxBuffer.length() > 512) uart2RxBuffer = "";
    }
  }
}


// ================== DRIVER COMMAND HELPERS ==================

void sendSpeedToDriver(const String& axisName, float sp) {
  if (updateInProgress) return;

  StaticJsonDocument<96> doc;
  doc["cmd"] = "setSpeed";
  doc["axis"] = axisName;
  doc["speed"] = sp;
  String out;
  serializeJson(doc, out);
  if (updateInProgress) return;

  LinkSerial.println(out);
}

void sendMoveAbsToDriver(const String& axisName, float targetMM, float speed) {
  if (updateInProgress) return;
  StaticJsonDocument<128> doc;
  doc["cmd"] = "moveAbs";
  doc["axis"] = axisName;
  doc["pos_mm"] = targetMM;
  doc["speed"] = speed;
  String out;
  serializeJson(doc, out);
  if (updateInProgress) return;

  LinkSerial.println(out);
}

void sendScaleModeToDriver(const String& axisName, int mode) {
  if (updateInProgress) return;
  StaticJsonDocument<96> doc;
  doc["cmd"] = "setScaleMode";
  doc["axis"] = axisName;
  doc["mode"] = mode;
  String out;
  serializeJson(doc, out);
  if (updateInProgress) return;
  LinkSerial.println(out);
  Serial.printf(
    "[SCALE MODE] axis=%s mode=%d\n",
    axisName.c_str(),
    mode);
}


void moveAxesToPreset(const float* target, int axisCount, uint32_t duration) {
  if (duration == 0) duration = 1;

  for (int i = 0; i < axisCount; i++) {
    float current = axisState[i];
    float distance = fabsf(target[i] - current);
    float speed = distance / (float)duration;  // mm/s or deg/s

    // Optional: clamp speed per axis if needed
    // speed = constrain(speed, minSpeed, maxSpeed);

    sendMoveAbsToDriver(axisNames[i], target[i], speed);

    Serial.printf("[PRESET MOVE] axis=%s current=%.2f target=%.2f dur=%u speed=%.2f\n",
                  axisNames[i].c_str(), current, target[i], duration, speed);
  }
}

// -----------------------------------------------------------------------------
// Optional preset recall callback (required because sendJsonToDevice calls it)
// -----------------------------------------------------------------------------
void gatewayOnPresetRecall(int shot) {
  Serial.printf("[PRESET] Recall %d\n", shot);
}









// ================== JSON → UART CONTROL ==================
void sendJsonToDevice(const JsonObject& data) {
  if (updateInProgress) return;

  // COLUMN JOYSTICK — use TILT directly
  if (deviceName == "column" && data.containsKey("column") && data["column"].containsKey("currentSpeed")) {

    float sp = data["column"]["currentSpeed"].as<float>();
    sendSpeedToDriver("column", sp);
    return;
  }

  // SLIDER JOYSTICK — use PAN + jsRamp acceleration
  if (deviceName == "slider" && data.containsKey("track")) {

    JsonObject pan = data["track"];

    // Slider acceleration curve
    if (pan.containsKey("jsRamp")) {
      int jsRamp = pan["jsRamp"].as<int>();

      float t = (jsRamp + 10.0f) / 20.0f;
      t = constrain(t, 0.0f, 1.0f);

      float s = t * t * (3.0f - 2.0f * t);

      const float accelMin = 20.0f;
      const float accelMax = 200.0f;
      float accel = accelMin + (accelMax - accelMin) * s;

      StaticJsonDocument<96> doc;
      doc["cmd"] = "setMotion";
      doc["accel_mm_s2"] = accel;

      String out;
      serializeJson(doc, out);
      if (updateInProgress) return;

      LinkSerial.println(out);
    }

    // Slider speed
    if (pan.containsKey("currentSpeed")) {
      float sp = pan["currentSpeed"].as<float>();
      sendSpeedToDriver("track", sp);
      return;
    }
  }

  // UNIVERSAL POSITION AXIS (SHOT RECALL)
  if (positionAxisValid && data.containsKey("setPosition")) {
    JsonObject sp = data["setPosition"];

    if (sp.containsKey("axis")) {
      JsonObject axis = sp["axis"];
      Serial.print("[SHOT] Axis keys: ");
      serializeJson(axis, Serial);
      Serial.println();

      if (axis.containsKey("track")) {
        int trackVal = axis["track"].as<int>();
        trackVal = constrain(trackVal, 0, 10000);

        float targetMM = map(trackVal, 0, 10000, 0.0f, positionAxis.range);

        float duration = sp["duration"] | 0.0f;
        float speed = 0.0f;

        if (duration > 0.0f) {
          float currentMM = sliderPosMM;
          float distance = fabsf(targetMM - currentMM);
          speed = distance / duration;
        }

        sendMoveAbsToDriver(positionAxis.name, targetMM, speed);

        Serial.printf("[SHOT] track=%d → axis=%s target=%.2f mm, speed=%.2f mm/s\n",
                      trackVal, positionAxis.name.c_str(), targetMM, speed);
        return;
      }
    }
  }

  // ================== PRESET STORE NEW ==================
  if (data.containsKey("storeShot")) {
    JsonObject ss = data["storeShot"];
    int shot = ss["shot"] | 0;

    if (shot >= 1 && shot <= MAX_PRESETS && axisCount > 0) {
      // For now, use a fixed duration or last known duration.
      // You can later track lastMoveDuration if needed.
      uint32_t duration = 1;  // default 1s
      for (int i = 0; i < axisCount; i++) {
        Serial.printf(
          "[STORE] axis=%s state=%.2f\n",
          axisNames[i].c_str(),
          axisState[i]);
      }
      presetStore(shot - 1, axisCount, axisState, duration);
    } else {
      Serial.printf("[PRESET] storeShot invalid: shot=%d axisCount=%d\n", shot, axisCount);
    }
    return;
  }


  // ================== PRESET RECALL (Unified) ==================
  if (data.containsKey("recallShot")) {

    JsonObject rs = data["recallShot"];
    int shot = rs["shot"] | 0;
    uint32_t durationOverride = rs["duration"] | 0;

    if (shot >= 1 && shot <= MAX_PRESETS && axisCount > 0) {

      // Use override if provided, otherwise use lastDuration
      uint32_t dur = (durationOverride > 0) ? durationOverride : getCurrentDuration();

      // ⭐ Unified preset recall path
      recallPreset(shot, dur);  // <-- This triggers recording automatically

      return;
    }

    Serial.printf("[PRESET] recallShot invalid: shot=%d axisCount=%d\n", shot, axisCount);
    return;
  }




  // STOP
  if (data.containsKey("allstop") && data["allstop"].as<bool>()) {
    StaticJsonDocument<64> doc;
    doc["cmd"] = "stop";
    String out;
    serializeJson(doc, out);
    if (updateInProgress) return;

    LinkSerial.println(out);
    return;
  }

  // HOME
  if (data.containsKey("home") && data["home"].as<bool>()) {
    StaticJsonDocument<96> doc;
    doc["cmd"] = "home";
    doc["mode"] = "switch";
    String out;
    serializeJson(doc, out);
    if (updateInProgress) return;
    if (updateInProgress) return;

    LinkSerial.println(out);
    return;
  }
}

// ================== ESP32 FLASHING (UART OTA) ==================
uint32_t crc32_le(uint32_t crc, const uint8_t* buf, size_t len) {
  crc = ~crc;
  while (len--) {
    crc ^= *buf++;
    for (int i = 0; i < 8; i++)
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return ~crc;
}

bool waitForAck(HardwareSerial& uart, uint32_t timeoutMs = 8000) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (uart.available()) {
      int b = uart.read();
      if (b == 0x06) return true;  // ACK
      // ignore everything else
    }
  }
  return false;
}

void flushUartRx(HardwareSerial& uart) {
  while (uart.available()) uart.read();
}

bool sendMetadata(HardwareSerial& uart, uint32_t fwSize, uint32_t fwCRC, uint16_t chunkSize) {
  uint8_t header[2] = { 0x55, 0xAA };

  // --- OTA SYNC ---
  uart.write(header, 2);
  uart.flush();

  delay(50);  // allow ESP32 to enter OTA mode

  // --- METADATA ---
  uart.write((uint8_t*)&fwSize, 4);
  uart.write((uint8_t*)&fwCRC, 4);
  uart.write((uint8_t*)&chunkSize, 2);
  uart.flush();

  return waitForAck(uart);
}

bool sendChunk(HardwareSerial& uart, const uint8_t* data, uint16_t len) {
  uart.write((uint8_t*)&len, 2);
  uart.write(data, len);

  uint32_t crc = crc32_le(0, data, len);
  uart.write((uint8_t*)&crc, 4);

  uart.flush();
  return waitForAck(uart);
}

int readHttpExact(WiFiClient* stream, uint8_t* buf, int len) {
  int received = 0;
  uint32_t start = millis();

  while (received < len && millis() - start < 5000) {  // 5s timeout
    int avail = stream->available();
    if (avail > 0) {
      int toRead = min(avail, len - received);
      int n = stream->read(buf + received, toRead);
      if (n > 0) {
        received += n;
      }
    } else {
      delay(1);
    }
  }

  return received;
}


bool sendEnd(HardwareSerial& uart) {
  uint8_t endMarker[2] = { 0xEE, 0xEE };
  uart.write(endMarker, 2);
  uart.flush();
  return waitForAck(uart);
}

bool flashESP32FromURL(const char* url) {

  updateInProgress = true;  // 🔥 BLOCK uart2ReceiveLoop()

  // FULL UART FLUSH BEFORE ANYTHING
  flushUartRx(LinkSerial);
  delay(2);

  Serial.printf("[UPD] Downloading ESP32 firmware (pass 1, CRC): %s\n", url);

  HTTPClient http;
  WiFiClient client;

  if (!http.begin(client, url)) {
    Serial.println("[UPD] http.begin() FAILED");
    updateInProgress = false;
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != 200) {
    Serial.printf("[UPD] HTTP GET failed: %d\n", httpCode);
    http.end();
    updateInProgress = false;
    return false;
  }

  int fwSize = http.getSize();
  Serial.printf("[UPD] ESP32 FW size: %d bytes\n", fwSize);
  if (fwSize <= 0) {
    Serial.println("[UPD] Invalid firmware size");
    http.end();
    updateInProgress = false;
    return false;
  }

  // PASS 1: compute CRC
  WiFiClient* stream = http.getStreamPtr();
  uint8_t buf[1024];
  int remaining = fwSize;
  uint32_t fwCRC = 0;

  while (remaining > 0) {
    int toRead = remaining > sizeof(buf) ? sizeof(buf) : remaining;
    int r = readHttpExact(stream, buf, toRead);
    if (r != toRead) {
      Serial.println("[UPD] Read error during CRC pass");
      http.end();
      updateInProgress = false;
      return false;
    }
    fwCRC = crc32_le(fwCRC, buf, r);
    remaining -= r;
  }

  http.end();
  Serial.printf("[UPD] ESP32 FW CRC: %08X\n", fwCRC);


  // PASS 2: stream to ESP32
  Serial.printf("[UPD] Downloading ESP32 firmware (pass 2, stream): %s\n", url);

  if (!http.begin(client, url)) {
    Serial.println("[UPD] http.begin() FAILED (pass 2)");
    updateInProgress = false;
    return false;
  }

  httpCode = http.GET();
  if (httpCode != 200) {
    Serial.printf("[UPD] HTTP GET failed (pass 2): %d\n", httpCode);
    http.end();
    updateInProgress = false;
    return false;
  }

  int fwSize2 = http.getSize();
  if (fwSize2 != fwSize) {
    Serial.printf("[UPD] Size mismatch between pass1 (%d) and pass2 (%d)\n", fwSize, fwSize2);
    http.end();
    updateInProgress = false;
    return false;
  }

  stream = http.getStreamPtr();
  const uint16_t CHUNK_SIZE = 64;

  // 🔥 CRITICAL: flush BEFORE metadata
  flushUartRx(LinkSerial);
  delay(2);

  if (!sendMetadata(LinkSerial, fwSize, fwCRC, CHUNK_SIZE)) {
    Serial.println("[UPD] ESP32 metadata ACK failed");
    http.end();
    updateInProgress = false;
    return false;
  }

  // FIRST CHUNK
  int firstRead = readHttpExact(stream, buf, CHUNK_SIZE);
  Serial.printf("[UPD] First chunk readHttpExact got %d bytes\n", firstRead);

  if (firstRead != CHUNK_SIZE) {
    Serial.printf("[UPD] First chunk incomplete: got %d bytes\n", firstRead);
    http.end();
    updateInProgress = false;
    return false;
  }
  // 🔥 ADD THIS
  Serial.println("[WT32] First chunk dump:");
  for (int i = 0; i < CHUNK_SIZE; i++) {
    if (i % 16 == 0) Serial.println();
    Serial.printf("%02X ", buf[i]);
  }
  Serial.println();
  uint32_t testCRC = crc32_le(0, buf, CHUNK_SIZE);
  Serial.printf("[WT32] calcCRC=%08X\n", testCRC);

  if (!sendChunk(LinkSerial, buf, CHUNK_SIZE)) {
    Serial.println("[UPD] ESP32 first chunk failed");
    http.end();
    updateInProgress = false;
    return false;
  }

  int sent = CHUNK_SIZE;

  // Remaining chunks
  while (sent < fwSize) {
    int toRead = (fwSize - sent) > CHUNK_SIZE ? CHUNK_SIZE : (fwSize - sent);
    int r = readHttpExact(stream, buf, toRead);

    if (r != toRead) {
      Serial.printf("[UPD] Incomplete chunk: got %d of %d\n", r, toRead);
      http.end();
      updateInProgress = false;
      return false;
    }

    if (!sendChunk(LinkSerial, buf, (uint16_t)r)) {
      Serial.println("[UPD] ESP32 chunk failed, retrying...");
      delay(50);
      continue;
    }

    sent += r;
    Serial.printf("[UPD] Sent %d/%d bytes to ESP32\n", sent, fwSize);
  }

  http.end();

  if (!sendEnd(LinkSerial)) {
    Serial.println("[UPD] ESP32 end ACK failed");
    updateInProgress = false;
    return false;
  }

  Serial.println("[UPD] ESP32 update OK");

  updateInProgress = false;  // 🔥 RE‑ENABLE uart2ReceiveLoop()
  return true;
}




// ================== WT32 STREAMING OTA ==================

bool flashWT32Streaming(const char* url) {
  Serial.printf("[UPD] Streaming WT32 OTA from: %s\n", url);

  HTTPClient http;
  WiFiClient client;

  if (!http.begin(client, url)) {
    Serial.println("[UPD] http.begin() FAILED");
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != 200) {
    Serial.printf("[UPD] HTTP GET failed: %d\n", httpCode);
    http.end();
    return false;
  }

  int fwSize = http.getSize();
  Serial.printf("[UPD] WT32 FW size: %d bytes\n", fwSize);

  const esp_partition_t* otaPartition = esp_ota_get_next_update_partition(NULL);
  if (!otaPartition) {
    Serial.println("[UPD] No OTA partition!");
    http.end();
    return false;
  }

  Serial.printf("[UPD] OTA partition: addr=0x%08X size=%u\n",
                otaPartition->address, otaPartition->size);

  esp_ota_handle_t otaHandle = 0;
  if (esp_ota_begin(otaPartition, fwSize, &otaHandle) != ESP_OK) {
    Serial.println("[UPD] esp_ota_begin FAILED");
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  uint8_t buff[1024];
  int written = 0;

  while (written < fwSize) {
    int toRead = min(1024, fwSize - written);
    int r = stream->readBytes(buff, toRead);
    if (r <= 0) break;

    if (esp_ota_write(otaHandle, buff, r) != ESP_OK) {
      Serial.println("[UPD] esp_ota_write FAILED");
      esp_ota_end(otaHandle);
      http.end();
      return false;
    }

    written += r;
    delay(1);
  }

  http.end();

  if (written != fwSize) {
    Serial.println("[UPD] Size mismatch");
    esp_ota_end(otaHandle);
    return false;
  }

  if (esp_ota_end(otaHandle) != ESP_OK) {
    Serial.println("[UPD] esp_ota_end FAILED");
    return false;
  }

  if (esp_ota_set_boot_partition(otaPartition) != ESP_OK) {
    Serial.println("[UPD] set_boot_partition FAILED");
    return false;
  }

  Serial.println("[UPD] WT32 update OK — rebooting...");
  delay(200);
  //ESP.restart();
  esp_deep_sleep(1000);  // 1ms deep sleep → full hardware reset
  return true;           // not reached
}

// ================== UNIFIED UPDATE ENGINE ==================

bool startUpdateTarget(const String& target) {
  if (!ethGotIP) {
    Serial.println("[UPD] Cannot update: no IP");
    return false;
  }
  if (updateInProgress) {
    Serial.println("[UPD] Update already in progress");
    return false;
  }

  updateInProgress = true;
  Serial.printf("[UPD] Starting update for target='%s'\n", target.c_str());

  // 🔥 ALWAYS refresh manifest first
  if (!manifestDownload(dynamicManifestURL.c_str())) {
    Serial.println("[UPD] Manifest download FAILED");
    updateInProgress = false;
    return false;
  }

  if (!manifestLoad()) {
    Serial.println("[UPD] Manifest load FAILED");
    updateInProgress = false;
    return false;
  }


  // // 🔥 Determine correct firmware file path (NOT full URL)
  // String filePath = manifestGetFirmwareURL(
  //   deviceName,  // "slider"
  //   "revA",      // hardware revision
  //   "3.00.00"    // current FW version (device will report this soon)
  // );

  // if (filePath.length() == 0) {
  //   Serial.println("[UPD] No firmware entry for this device");
  //   updateInProgress = false;
  //   return false;
  // }

  // 🔥 Prepend SERVER IP + port to build full URL
  // String fwURL = "http://" + otaServerIP + ":8000" + filePath;
  // ESP32_FW_URL = fwURL;

  Serial.printf("[UPD] Starting update for target='%s'\n", target.c_str());

  bool ok = true;

  if (target == "esp32" || target == "all") {

    // 🔥 Manifest-driven firmware selection (file path only)
    String filePath = manifestGetFirmwareURL(
      deviceName,        // e.g. "slider"
      driverHwRevision,  // e.g. "revA"
      driverFwVersion    // e.g. "3.00.00"
    );

    if (filePath.length() == 0) {
      Serial.println("[UPD] No matching firmware in manifest");
      updateInProgress = false;
      return false;
    }

    // 🔥 Build full URL using SERVER IP
    String fwURL = "http://" + otaServerIP + ":8000" + filePath;
    Serial.printf("[UPD] Selected firmware: %s\n", fwURL.c_str());

    ok = flashESP32FromURL(fwURL.c_str());

    if (!ok) {
      Serial.println("[UPD] ESP32 update FAILED");
      updateInProgress = false;
      return false;
    }
  }

  if (target == "wt32" || target == "all") {

    String filePath = manifestGetFirmwareURL(
      "gateway",
      gatewayHwRevision,  // <-- FIXED
      GATEWAY_FW_VERSION);

    if (filePath.length() == 0) {
      Serial.println("[UPD] No gateway firmware entry");
      updateInProgress = false;
      return false;
    }

    String gwURL = "http://" + otaServerIP + ":8000" + filePath;
    WT32_FW_URL = gwURL;

    ok = flashWT32Streaming(WT32_FW_URL.c_str());

    if (!ok) {
      Serial.println("[UPD] WT32 update FAILED");
      updateInProgress = false;
      return false;
    }
  }


  updateInProgress = false;
  return ok;
}

// ================== BST SERVER LOOP ==================

void sendBstJsonReply(const char* status, const char* message) {
  if (!bstClient.connected()) return;

  StaticJsonDocument<256> doc;
  doc["status"] = status;
  doc["message"] = message;

  String out;
  serializeJson(doc, out);
  bstClient.print(out);
  bstClient.write(FRAME_END);
}

void handleBstFrame(const String& frame) {
  Serial.print("[BST RX] ");
  Serial.println(frame);

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, frame);
  if (err) {
    Serial.print("[BST] JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }

  const char* command = doc["command"] | "";

  if (strcmp(command, "update") == 0) {
    JsonObject data = doc["data"];
    if (!data.isNull()) sendJsonToDevice(data);
    return;
  }

  if (strcmp(command, "hello") == 0) {
    StaticJsonDocument<512> reply;

    reply["command"] = "hello";
    reply["device"] = "camACE";

    JsonObject axis = reply.createNestedObject("axis");
    JsonObject track = axis.createNestedObject("track");
    track["type"] = "position";

    JsonObject range = track.createNestedObject("range");
    range["min"] = -174.0f;
    range["max"] = 173.0f;

    String out;
    serializeJson(reply, out);
    bstClient.print(out);
    bstClient.write(FRAME_END);

    Serial.println("[BST] Sent hello reply");
    return;
  }

  if (strcmp(command, "updateFirmware") == 0) {
    const char* target = doc["target"] | "all";
    String t = target;

    Serial.printf("[BST] Firmware update requested: target=%s\n", t.c_str());

    if (t != "esp32" && t != "wt32" && t != "all") {
      sendBstJsonReply("error", "Invalid target");
      return;
    }

    if (!ethGotIP) {
      sendBstJsonReply("error", "No IP");
      return;
    }

    if (updateInProgress) {
      sendBstJsonReply("error", "Update already in progress");
      Serial.println("[UPD] Update already in progress");
      return;
    }

    sendBstJsonReply("ok", "Update started");

    // Run update (blocking)
    startUpdateTarget(t);
    return;
  }

  // ⭐⭐⭐ INSERT PATCH HERE — DRIVER STATUS PACKETS ⭐⭐⭐
  if (doc.containsKey("pos_mm")) {

    float pos = doc["pos_mm"].as<float>();

    // Update slider position
    sliderPosMM = pos;

    // Update axisState for the track axis (usually index 0)
    axisState[0] = pos;

    // (Optional) You may log moving state, but do NOT store it
    // if (doc.containsKey("moving")) {
    //     Serial.printf("[BST] moving=%d\n", doc["moving"].as<bool>());
    // }

    return;
  }
  // ⭐⭐⭐ END OF PATCH ⭐⭐⭐

  Serial.print("[BST] Unhandled command: ");
  Serial.println(command);
}


#define BST_BUF_SIZE 1024
char bstBuf[BST_BUF_SIZE];
size_t bstHead = 0;

void bstServerLoop() {
  if (!bstClient || !bstClient.connected()) {
    WiFiClient newClient = bstServer.available();
    if (newClient) {
      bstClient.stop();
      bstClient = newClient;
      bstHead = 0;
      Serial.println("[BST] Client connected");
    }
  }

  if (!bstClient || !bstClient.connected()) return;

  while (bstClient.available()) {
    char c = bstClient.read();

    if (c == FRAME_END) {
      // complete frame
      bstBuf[bstHead] = 0;  // null terminate
      handleBstFrame(String(bstBuf));
      bstHead = 0;
    } else {
      if (bstHead < BST_BUF_SIZE - 1) {
        bstBuf[bstHead++] = c;
      } else {
        // overflow, reset
        bstHead = 0;
      }
    }
  }
}

// ================== HTTP UPDATE SERVER (PORT 8080) ==================

void sendHttpJson(WiFiClient& client, int code, const char* status, const char* message) {
  StaticJsonDocument<256> doc;
  doc["status"] = status;
  doc["message"] = message;

  String body;
  serializeJson(doc, body);

  client.printf("HTTP/1.1 %d\r\n", code);
  client.println("Content-Type: application/json");
  client.printf("Content-Length: %d\r\n", body.length());
  client.println("Connection: close");
  client.println();
  client.print(body);
}

String getQueryParam(const String& query, const String& key) {
  int start = query.indexOf(key + "=");
  if (start < 0) return "";
  start += key.length() + 1;
  int end = query.indexOf('&', start);
  if (end < 0) end = query.length();
  return query.substring(start, end);
}

void handleVersionRequest(WiFiClient& client) {
  StaticJsonDocument<192> doc;

  doc["gatewayFw"] = GATEWAY_FW_VERSION;
  doc["driverFw"] = driverFwVersion;
  doc["driverHw"] = driverHwRevision;
  doc["device"] = deviceName;  // ← correct variable

  String out;
  serializeJson(doc, out);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();
  client.print(out);
}

bool performOtaUpdate(const String& target) {
  updateInProgress = true;

  // Build manifest URL using server IP
  String manifestUrl = "http://" + otaServerIP + ":8000/manifest.json";
  Serial.printf("[UPD] Manifest URL: %s\n", manifestUrl.c_str());

  // Download manifest
  if (!manifestDownload(manifestUrl.c_str())) {
    Serial.println("[UPD] Manifest download FAILED");
    updateInProgress = false;
    return false;
  }

  // Load manifest
  if (!manifestLoad()) {
    Serial.println("[UPD] Manifest load FAILED");
    updateInProgress = false;
    return false;
  }


  // //🔥 Determine correct firmware file path (NOT full URL)
  // String filePath = manifestGetFirmwareURL(deviceName,"revA","3.00.00");

  // if (filePath.length() == 0) {
  //   Serial.println("[UPD] No firmware entry for this device");
  //   updateInProgress = false;
  //   return false;
  // }

  // //🔥 Prepend SERVER IP + port
  // String fwURL = "http://" + otaServerIP + ":8000" + filePath;
  // ESP32_FW_URL = fwURL;

  Serial.printf("[UPD] Starting update for target='%s'\n", target.c_str());
  Serial.printf("[UPD] Current device=%s hw=%s fw=%s\n",
                deviceName.c_str(),
                driverHwRevision.c_str(),
                driverFwVersion.c_str());
  bool ok = true;

  if (target == "esp32" || target == "all") {

    String filePath = manifestGetFirmwareURL(
      deviceName,
      driverHwRevision,
      driverFwVersion);

    if (filePath.length() == 0) {
      Serial.println("[UPD] No matching firmware in manifest");
      updateInProgress = false;
      return false;
    }

    String fwURL = "http://" + otaServerIP + ":8000" + filePath;
    Serial.printf("[UPD] Selected firmware: %s\n", fwURL.c_str());

    ok = flashESP32FromURL(fwURL.c_str());

    if (!ok) {
      Serial.println("[UPD] ESP32 update FAILED");
      updateInProgress = false;
      return false;
    }
  }

  if (target == "wt32" || target == "all") {

    String filePath = manifestGetFirmwareURL("gateway", "revA", GATEWAY_FW_VERSION);

    if (filePath.length() == 0) {
      Serial.println("[UPD] No gateway firmware entry");
      updateInProgress = false;
      return false;
    }

    String gwURL = "http://" + otaServerIP + ":8000" + filePath;
    WT32_FW_URL = gwURL;

    ok = flashWT32Streaming(WT32_FW_URL.c_str());

    if (!ok) {
      Serial.println("[UPD] WT32 update FAILED");
      updateInProgress = false;
      return false;
    }
  }

  updateInProgress = false;
  return ok;
}



void httpServerLoop() {
  WiFiClient client = httpServer.available();
  if (!client) return;

  // Read request line
  String reqLine = client.readStringUntil('\n');
  reqLine.trim();
  if (reqLine.length() == 0) {
    client.stop();
    return;
  }

  // -----------------------------------------
  // Parse method + full path
  // -----------------------------------------
  int sp1 = reqLine.indexOf(' ');
  int sp2 = reqLine.indexOf(' ', sp1 + 1);

  if (sp1 < 0 || sp2 < 0) {
    sendHttpJson(client, 400, "error", "Bad request");
    client.stop();
    return;
  }

  String method = reqLine.substring(0, sp1);          // GET or POST
  String fullPath = reqLine.substring(sp1 + 1, sp2);  // /network?x=y

  // Strip query params
  String cleanPath = fullPath;
  int q = cleanPath.indexOf('?');
  if (q >= 0) cleanPath = cleanPath.substring(0, q);

  // Extract query string
  String query = "";
  if (q >= 0) query = fullPath.substring(q + 1);

  // Debug
  Serial.printf("[HTTP] method=%s path=%s\n", method.c_str(), cleanPath.c_str());

  // Drain headers
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r" || line.length() == 0) break;
  }

  // -----------------------------------------
  // /network GET → show page
  // /network POST → save + reboot
  // -----------------------------------------
  if (cleanPath == "/network" || cleanPath == "/network/") {

    if (method == "GET") {
      handleNetworkPage(client);
      client.stop();
      return;
    }

    if (method == "POST") {
      String body = "";
      uint32_t timeout = millis() + 2000;

      while (millis() < timeout) {
        while (client.available()) {
          body += char(client.read());
        }
        if (!client.connected()) break;
        vTaskDelay(1);
      }

      handleNetworkPost(client, body);
      client.stop();
      return;
    }

    sendHttpJson(client, 405, "error", "Method not allowed");
    client.stop();
    return;
  }

  // -----------------------------------------
  // /factoryreset POST
  // -----------------------------------------
  if (cleanPath == "/factoryreset" || cleanPath == "/factoryreset/") {

    if (method == "POST") {
      handleFactoryReset(client);
      client.stop();
      return;
    }

    sendHttpJson(client, 405, "error", "Method not allowed");
    client.stop();
    return;
  }





  // -----------------------------------------
  // /slider_config GET → show slider config page
  // /slider_config POST → save + reboot
  // -----------------------------------------
  if (cleanPath == "/slider_config" || cleanPath == "/slider_config/") {

    // Block config page during OTA
    if (updateInProgress) {
      client.println("HTTP/1.1 503 Service Unavailable");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("OTA update in progress. Config page unavailable.");
      client.stop();
      return;
    }

    if (method == "GET") {
      handleSliderConfigPage(client);
      client.stop();
      return;
    }

    if (method == "POST") {

      // Read full POST body reliably
      String body = "";
      uint32_t timeout = millis() + 2000;

      while (millis() < timeout) {
        while (client.available()) {
          body += char(client.read());
        }
        if (!client.connected()) break;
        vTaskDelay(1);  // allow TCP to deliver remaining segments
      }

      handleSliderConfigPost(client, body);

      // Allow TCP stack to flush the reboot message
      delay(200);

      client.stop();
      return;
    }



    sendHttpJson(client, 405, "error", "Method not allowed");
    client.stop();
    return;
  }
  // -----------------------------------------
  // /update
  //
  if (cleanPath == "/update") {

    // Extract SERVER IP
    IPAddress remote = client.remoteIP();
    otaServerIP = remote.toString();
    Serial.printf("[UPDATE] Server IP detected: %s\n", otaServerIP.c_str());

    // Build manifest URL
    dynamicManifestURL = "http://" + otaServerIP + ":8000/manifest.json";
    Serial.printf("[UPDATE] Manifest URL set to: %s\n", dynamicManifestURL.c_str());

    // Extract target
    String target = "all";
    if (query.startsWith("target=")) {
      target = query.substring(strlen("target="));
    }

    // Call the REAL OTA engine
    bool ok = startUpdateTarget(target);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println();
    client.println(ok ? "OTA OK" : "OTA FAILED");

    client.stop();
    return;
  }




  // -----------------------------------------
  // /version
  // -----------------------------------------
  if (cleanPath == "/version") {
    handleVersionRequest(client);
    client.stop();
    return;
  }

  // -----------------------------------------
  // /update
  // -----------------------------------------
  if (cleanPath == "/update") {

    // Extract SERVER IP
    IPAddress remote = client.remoteIP();
    otaServerIP = remote.toString();
    Serial.printf("[UPDATE] Server IP detected: %s\n", otaServerIP.c_str());

    // Extract target from query
    String target = "all";
    if (query.startsWith("target=")) {
      target = query.substring(strlen("target="));
    }

    bool ok = startUpdateTarget(target);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println();
    client.println(ok ? "OTA OK" : "OTA FAILED");

    client.stop();
    return;
  }

  // -----------------------------------------
  // / or /. → Cowboy Trigger JSON commands
  // -----------------------------------------
  if (cleanPath == "." || cleanPath == "/." || cleanPath == "/") {

    if (method == "POST" || method == "PUT") {
      String body = "";
      uint32_t timeout = millis() + 2000;

      while (millis() < timeout) {
        while (client.available()) {
          body += char(client.read());
        }
        if (!client.connected()) break;
        vTaskDelay(1);
      }

      Serial.println("=== RAW BODY RECEIVED ===");
      Serial.println(body);
      Serial.println("==========================");

      // ⭐ Cowboy command routing (tagged bodies)
      if (body.startsWith("PreRec:")) {
        bool state = body.endsWith("1");
        gatewaySetRecordMode(state);
        Serial.printf("[RECORD] %s\n", state ? "ON" : "OFF");
      } else if (body.startsWith("PrePlay:")) {
        bool state = body.endsWith("1");
        gatewaySetPlaybackMode(state);
        Serial.printf("[PLAYBACK] %s\n", state ? "ON" : "OFF");
      } else if (body.startsWith("PreLoop:")) {
        bool state = body.endsWith("1");
        gatewaySetLoopMode(state);
        Serial.printf("[LOOP] %s\n", state ? "ON" : "OFF");
      } else if (body.startsWith("ScaleMode:")) {

        int mode = body.substring(10).toInt();
        Serial.printf(
          "[COWBOY] ScaleMode=%d\n", mode);

        sendScaleModeToDriver(positionAxis.name, mode);

      } else {
        Serial.print("[COWBOY] Unknown body: ");
        Serial.println(body);
      }

      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("OK");
      client.stop();
      return;
    }

    sendHttpJson(client, 405, "error", "Method not allowed");
    client.stop();
    return;
  }


  // -----------------------------------------
  // Unknown path
  // -----------------------------------------
  sendHttpJson(client, 404, "error", "Not found");
  client.stop();
}



void httpTask(void* param) {
  while (true) {
    httpServerLoop();  // your existing blocking HTTP handler
    vTaskDelay(1);     // yield to other tasks
  }
}

void uart2Task(void* param) {
  while (true) {
    uart2ReceiveLoop();  // your existing function
    vTaskDelay(1);       // yield
  }
}

// -----------------------------------------
// FACTORY RESET FROM WEB UI
// -----------------------------------------
void handleFactoryReset(WiFiClient& client) {
  Serial.println("[FACTORY] Network reset requested");

  Preferences prefs;

  // New format
  if (prefs.begin("network", false)) {
    prefs.clear();
    prefs.end();
  }

  // Legacy format
  if (prefs.begin("netcfg", false)) {
    prefs.clear();
    prefs.end();
  }

  String msg =
    "<html><body>"
    "<h2>Network Reset Complete</h2>"
    "<p>Saved network settings have been erased.</p>"
    "<p>The gateway is now rebooting.</p>"
    "<p>After restart the device will attempt to obtain an IP address via DHCP.</p>"
    "<p>If no DHCP server can be reached the system will use default IP 192.168.10.250.</p>"
    "</body></html>";

  Serial.println("[FACTORY] Network reset requested");
  Serial.println("[FACTORY] Clearing network configuration");
  Serial.println("[FACTORY] Rebooting into DHCP mode");

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.printf("Content-Length: %d\r\n", msg.length());
  client.println("Connection: close");
  client.println();
  client.print(msg);

  client.stop();

  delay(500);

  ESP.restart();
}

// -----------------------------------------
// FACTORY RESET FROM BUTTON
// -----------------------------------------

void factoryResetNetwork() {
  Serial.println("[FACTORY] Clearing network configuration");

  Preferences prefs;

  // New network configuration
  if (prefs.begin("network", false)) {
    prefs.clear();
    prefs.end();
  }

  // Legacy network configuration
  if (prefs.begin("netcfg", false)) {
    prefs.clear();
    prefs.end();
  }

  Serial.println("[FACTORY] Network configuration removed");
  Serial.println("[FACTORY] Rebooting into DHCP mode");

  delay(500);

  ESP.restart();
}

// ================== SETUP ==================

void setup() {
  Serial.begin(115200);
  Serial.printf("Boot reason: %d\n", esp_reset_reason());
  delay(500);


  LinkSerial.begin(UART2_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  WiFi.onEvent(ETH_Event);

  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, HIGH);
  delay(200);


  // ================== OLD ERH.begin ==================

  // ETH.begin(
  //   ETH_TYPE,
  //   ETH_ADDR,
  //   ETH_MDC_PIN,
  //   ETH_MDIO_PIN,
  //   ETH_POWER_PIN,
  //   ETH_CLK_MODE);



  // if (loadNetworkConfig()) {
  //   Serial.println("[NET] Using saved static IP");
  //   ETH.config(cfgIP, cfgGW, cfgMASK, cfgDNS);
  // } else {
  //   Serial.println("[NET] Using default static IP");
  //   ETH.config(localIP, gatewayIP, subnetIP, dnsIP);
  // }

  //    Serial.println("[NVS] Clearing netcfg namespace...");
  // Preferences prefs;
  // prefs.begin("netcfg", false);
  // prefs.clear();     // wipe only this namespace
  // prefs.end();
  // Serial.println("[NVS] netcfg cleared");

  // ================== NEW ETH BEGIN ==================
  //ETH.begin();
  ETH.begin(
    ETH_TYPE,
    ETH_ADDR,
    ETH_MDC_PIN,
    ETH_MDIO_PIN,
    ETH_POWER_PIN,
    ETH_CLK_MODE);

  delay(1000);

  Serial.printf("[ETH] After begin IP=%s\n", ETH.localIP().toString().c_str());

  if (!ethGotIP && ETH.localIP().toString() != "0.0.0.0") {
    ethGotIP = true;
    Serial.printf("[ETH] Late IP acquired: %s\n", ETH.localIP().toString().c_str());
  }
  if (loadNetworkConfig()) {

    if (netCfg.useStatic) {

      Serial.println("[NET] Using saved static IP");

      bool ok = ETH.config(
        netCfg.ip,
        netCfg.gateway,
        netCfg.subnet,
        netCfg.dns);
      Serial.printf("[New NET] IP : %s\n", netCfg.ip.toString().c_str());
      Serial.printf("[NEW NET] GW : %s\n", netCfg.gateway.toString().c_str());
      Serial.printf("[NEW NET] MASK : %s\n", netCfg.subnet.toString().c_str());
      Serial.printf("[NEW NET] DNS : %s\n", netCfg.dns.toString().c_str());



      Serial.printf("[NET] ETH.config returned: %s\n",
                    ok ? "SUCCESS" : "FAIL");

    } else {

      Serial.println("[NET] DHCP / Recovery mode");
      Serial.println("[NET] Attempting DHCP...");

      uint32_t dhcpStart = millis();

      while (ETH.localIP() == IPAddress(0, 0, 0, 0) && millis() - dhcpStart < 10000) {
        delay(500);
        Serial.print("*");
      }

      Serial.println();

      if (ETH.localIP() != IPAddress(0, 0, 0, 0)) {

        Serial.printf(
          "[NET] DHCP acquired: %s\n",
          ETH.localIP().toString().c_str());

      } else {

        Serial.println(
          "[NET] DHCP timeout. Applying recovery address.");

        bool ok = ETH.config(
          recoveryIP,
          recoveryGW,
          recoveryMask,
          recoveryDNS);

        Serial.printf(
          "[NET] Recovery IP %s (%s)\n",
          recoveryIP.toString().c_str(),
          ok ? "SUCCESS" : "FAIL");
      }
    }

  } else {

    Serial.println("[NET] Recovery mode");
    Serial.println("[NET] Attempting DHCP...");

    uint32_t dhcpStart = millis();

    while (ETH.localIP() == IPAddress(0, 0, 0, 0) && millis() - dhcpStart < 30000) {
      delay(500);
      Serial.print("*");
    }

    Serial.println();

    if (ETH.localIP() != IPAddress(0, 0, 0, 0)) {

      Serial.printf(
        "[NET] DHCP acquired: %s\n",
        ETH.localIP().toString().c_str());

    } else {
      Serial.println(
        "[NET] DHCP timeout. Applying recovery address.");
      bool ok = ETH.config(
        recoveryIP,
        recoveryGW,
        recoveryMask,
        recoveryDNS);
      Serial.printf(
        "[NET] Recovery IP %s (%s)\n",
        recoveryIP.toString().c_str(),
        ok ? "SUCCESS" : "FAIL");
    }
  }


  // ================== NEW IP Acquisition Monitoring ==================
  if (!ethGotIP && ETH.localIP() != IPAddress(0, 0, 0, 0)) {
    ethGotIP = true;

    Serial.println("================================");
    Serial.printf("[ETH] IP : %s\n",
                  ETH.localIP().toString().c_str());
    Serial.printf("[ETH] MASK : %s\n",
                  ETH.subnetMask().toString().c_str());
    Serial.printf("[ETH] GW : %s\n",
                  ETH.gatewayIP().toString().c_str());
    Serial.printf("[ETH] DNS : %s\n",
                  ETH.dnsIP().toString().c_str());

    if (netCfg.valid && netCfg.useStatic)
      Serial.println("[ETH] MODE : STATIC");
    else
      Serial.println("[ETH] MODE : DHCP");

    Serial.println("================================");
  }





  xTaskCreatePinnedToCore(
    uart2Task,
    "UART2_RX",
    4096,
    NULL,
    1,  // priority
    NULL,
    1  // core
  );

  sendGatewayHello();

  Serial.println("[ETH] Waiting for link...");
  while (!ETH.linkUp()) {
    delay(250);
    Serial.print(".");
  }

  Serial.println("[ETH] Waiting for DHCP...");

  uint32_t dhcpStart = millis();

  while (ETH.localIP() == IPAddress(0, 0, 0, 0) && millis() - dhcpStart < 30000) {
    delay(500);
    Serial.print("*");
  }

  Serial.println();

  Serial.printf("[ETH] Final IP: %s\n",
                ETH.localIP().toString().c_str());








  Serial.println();

  Serial.print("[ETH] Link up. IP: ");
  Serial.println(ETH.localIP());

  manifestInit();  // mount SPIFFS only
  presetInit();    // NEW: init preset namespace

  // Send handshake request to driver
  {
    gotDriverHandshake = false;
    lastHelloSend = millis();
  }

  bstServer.begin();
  Serial.print("[BST] TCP server listening on port ");
  Serial.println(BST_PORT);

  httpServer.begin();
  Serial.print("[HTTP] Update server listening on port ");
  Serial.println(HTTP_PORT);

  xTaskCreatePinnedToCore(
    httpTask,
    "HTTP Task",
    8192,
    NULL,
    1,
    &httpTaskHandle,
    0);
  Serial.print("[Gatway Firmware Version] = ");
  Serial.println(GATEWAY_FW_VERSION);
}

// ================== LOOP ==================

void loop() {
  bstServerLoop();

  // Repeating hello handshake until driver responds
  if (!gotDriverHandshake) {
    if (millis() - lastHelloSend > 500) {
      StaticJsonDocument<64> hello;
      hello["cmd"] = "hello";
      String out;
      serializeJson(hello, out);
      if (updateInProgress) return;

      LinkSerial.println(out);
      Serial.println("[GATEWAY] Re-sent hello to driver");
      lastHelloSend = millis();
    }
  }
  // Delayed IP config send
  if (ipConfigPending && millis() > ipConfigSendAt) {
    if (!updateInProgress) {
      sendIpConfigToDriver();
      ipConfigPending = false;
    }
  }


  // Playback engine
  if (!updateInProgress) {
    gatewayHandlePlayback();
  }

  //uart2ReceiveLoop();
  sendTrackStatus();

  //Factory Reset button
  // if (digitalRead(RESET_PIN) == LOW) {
  //   if (!resetButtonActive) {
  //     resetButtonActive = true;
  //     resetPressedTime = millis();

  //     Serial.println("[FACTORY] Reset button pressed");
  //   }

  //   if (millis() - resetPressedTime > 5000) {
  //     Serial.println("[FACTORY] Factory reset requested");

  //     factoryResetNetwork();
  //   }
  // } else {
  //   resetButtonActive = false;
  // }
  // static uint32_t lastPrint = 0;

  // if (millis() - lastPrint > 5000) {
  //   lastPrint = millis();

  //   Serial.printf(
  //     "[DHCP] Current IP: %s\n",
  //     ETH.localIP().toString().c_str());
  // }
}

#include "manifest_manager.h"
#include <SPIFFS.h>
#include <HTTPClient.h>
#include <WiFi.h>

static const char* MANIFEST_PATH = "/manifest.json";
static DynamicJsonDocument manifestDoc(4096);

bool manifestInit() {
  if (!SPIFFS.begin(true)) {
    Serial.println("[MANIFEST] SPIFFS mount failed");
    return false;
  }
  return true;
}

bool manifestDownload(const char* url) {
  Serial.printf("[MANIFEST] Downloading manifest: %s\n", url);

  HTTPClient http;
  WiFiClient client;

  if (!http.begin(client, url)) {
    Serial.println("[MANIFEST] http.begin FAILED");
    return false;
  }

  int code = http.GET();
  if (code != 200) {
    Serial.printf("[MANIFEST] HTTP GET failed: %d\n", code);
    http.end();
    return false;
  }

  int size = http.getSize();
  if (size <= 0) {
    Serial.println("[MANIFEST] Invalid manifest size");
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  File f = SPIFFS.open(MANIFEST_PATH, "w");
  if (!f) {
    Serial.println("[MANIFEST] Failed to open manifest.json for writing");
    http.end();
    return false;
  }

  uint8_t buf[512];
  int remaining = size;

  while (remaining > 0) {
    int toRead = remaining > sizeof(buf) ? sizeof(buf) : remaining;
    int r = stream->readBytes(buf, toRead);
    if (r <= 0) {
      Serial.println("[MANIFEST] Read error");
      f.close();
      http.end();
      return false;
    }
    f.write(buf, r);
    remaining -= r;
  }

  f.close();
  http.end();

  Serial.println("[MANIFEST] Download complete");
  return true;
}

bool manifestLoad() {
  File f = SPIFFS.open(MANIFEST_PATH, "r");
  serializeJsonPretty(manifestDoc, Serial);
  Serial.println();

  if (!f) {
    Serial.println("[MANIFEST] No manifest.json found");
    return false;
  }

  DeserializationError err = deserializeJson(manifestDoc, f);
  f.close();

  if (err) {
    Serial.printf("[MANIFEST] JSON parse error: %s\n", err.c_str());
    return false;
  }

  Serial.println("[MANIFEST] Loaded OK");
  return true;
}


// ======================================================
//   NEW: Proper semantic version comparator
// ======================================================
static int compareVersions(const String& a, const String& b) {
  int a1 = 0, a2 = 0, a3 = 0;
  int b1 = 0, b2 = 0, b3 = 0;

  sscanf(a.c_str(), "%d.%d.%d", &a1, &a2, &a3);
  sscanf(b.c_str(), "%d.%d.%d", &b1, &b2, &b3);

  if (a1 != b1) return (a1 > b1) ? 1 : -1;
  if (a2 != b2) return (a2 > b2) ? 1 : -1;
  if (a3 != b3) return (a3 > b3) ? 1 : -1;

  return 0;
}


// ======================================================
//   UPDATED: Correctly pick newest firmware version
// ======================================================
String manifestGetFirmwareURL(const String& deviceType,
                              const String& hwRevision,
                              const String& currentVersion) {
  if (!manifestDoc.containsKey("devices")) {
    Serial.println("[MANIFEST] No 'devices' array");
    return "";
  }

  JsonArray arr = manifestDoc["devices"].as<JsonArray>();

  String bestVersion = "";
  String bestURL = "";

  for (JsonObject obj : arr) {
    String dt = obj["deviceType"] | "";
    String rev = obj["hwRevision"] | "";
    String ver = obj["fwVersion"] | "";
    String url = obj["url"] | "";

    Serial.printf(
      "[MANIFEST] Entry: dt='%s' rev='%s' ver='%s'\n",
      dt.c_str(),
      rev.c_str(),
      ver.c_str());

    Serial.printf(
      "[MANIFEST] Target: dt='%s' rev='%s' cur='%s'\n",
      deviceType.c_str(),
      hwRevision.c_str(),
      currentVersion.c_str());

    if (dt != deviceType) {
      Serial.println("[MANIFEST] Device type mismatch");
      continue;
    }

    if (rev != hwRevision) {
      Serial.println("[MANIFEST] HW revision mismatch");
      continue;
    }

    int cmp = compareVersions(ver, currentVersion);

    Serial.printf(
      "[MANIFEST] Version compare: %s vs %s = %d\n",
      ver.c_str(),
      currentVersion.c_str(),
      cmp);

    if (cmp <= 0) {
      Serial.println("[MANIFEST] Version not newer");
      continue;
    }

    Serial.println("[MANIFEST] Candidate accepted");

    if (bestVersion == "" || compareVersions(ver, bestVersion) > 0) {
      bestVersion = ver;
      bestURL = url;
    }
  }

  if (bestURL == "") {
    Serial.println("[MANIFEST] No matching newer firmware");
    return "";
  }

  Serial.printf("[MANIFEST] Selected %s %s → %s\n",
                deviceType.c_str(), bestVersion.c_str(), bestURL.c_str());

  return bestURL;
}

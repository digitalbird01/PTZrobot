#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

struct ManifestEntry {
    String deviceType;
    String hwRevision;
    String fwVersion;
    String url;
};

bool manifestInit();
bool manifestDownload(const char* url);
bool manifestLoad();
String manifestGetFirmwareURL(const String& deviceType,
                              const String& hwRevision,
                              const String& currentVersion);

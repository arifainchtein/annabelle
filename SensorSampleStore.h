#ifndef ANNABELLE_SENSOR_SAMPLE_STORE_H
#define ANNABELLE_SENSOR_SAMPLE_STORE_H
// Soil / rain samples and Gloria status packets (2026-11-26) - see Annabelle.ino's processLora().
//
// SensorSampleStore keeps the last SAMPLE_STORE_SIZE SensorSampleRecords from every device (Gloria's fixed
// probes, Paula2's walked samples with GPS, bench tests), for the web page (/SensorSamples) and for the Pi:
// each new sample is sent once to the Pi as a "SensorSampleDeserializer#..." line with AsyncData. Every line
// carries the sample's source - device name, serial, kind, role, position - so the Pi can track each one.
//
// GloriaDataQueue holds GloriaData packets until the Pi takes them ("GloriaDataDeserializer#..." lines).
//
// In a header of its own for the same reason as HeardEntry.h (the IDE's auto-prototypes).
#include <Arduino.h>
#include <ArduinoJson.h>
#include <GloriaData.h>
#include <SensorSampleRecord.h>
#include <ModbusSensorBus.h>
#include "DeviceSerial.h"

#define SAMPLE_STORE_SIZE 48
#define GLORIA_PI_PENDING 8

static inline void printCleanName(Print &out, const char *s, size_t n) {
  for (size_t i = 0; i < n && s[i]; i++) {
    char c = s[i];
    if (c >= 32 && c < 127 && c != '#') out.print(c);
  }
}

static inline void printFloatOrNan(Print &out, float v, int dec) {
  if (isnan(v)) out.print("nan");
  else out.print(v, dec);
}

// The driver of a sample (same library as the senders), or nullptr for a model Annabelle does not know
static inline const ModbusSensorDriver *sampleDriver(const SensorSampleRecord &r) {
  char id[sizeof(r.driver) + 1];
  memcpy(id, r.driver, sizeof(r.driver));
  id[sizeof(r.driver)] = 0;
  for (uint8_t k = 0; k < MODBUS_SENSOR_DRIVER_COUNT; k++) {
    if (strcmp(id, MODBUS_SENSOR_DRIVERS[k].id) == 0) return &MODBUS_SENSOR_DRIVERS[k];
  }
  return nullptr;
}

static const char *sampleKindText(uint8_t k) {
  return k == SAMPLE_SPOT ? "spot" : k == SAMPLE_TEST ? "test" : "monitor";
}

struct StoredSample {
  SensorSampleRecord r;
  uint32_t heardEpoch;  // Annabelle RTC
  bool toPi;            // not yet sent to the Pi
};

class SensorSampleStore {
public:
  void begin() {
    slots = (StoredSample *)calloc(SAMPLE_STORE_SIZE, sizeof(StoredSample));
  }

  // false when it is a copy already stored (a relayed resend): same serial, sampleId and time
  bool add(const SensorSampleRecord &r, uint32_t epoch) {
    if (!slots) return false;
    bool added = false;
    portENTER_CRITICAL(&mux);
    bool dup = false;
    for (int i = 0; i < count && !dup; i++) {
      const SensorSampleRecord &o = slots[(head - 1 - i + SAMPLE_STORE_SIZE) % SAMPLE_STORE_SIZE].r;
      dup = o.sampleId == r.sampleId && o.secondsTime == r.secondsTime && memcmp(o.serialnumberarray, r.serialnumberarray, 8) == 0;
    }
    if (!dup) {
      StoredSample &s = slots[head];
      s.r = r;
      s.heardEpoch = epoch;
      s.toPi = true;
      head = (head + 1) % SAMPLE_STORE_SIZE;
      if (count < SAMPLE_STORE_SIZE) count++;
      added = true;
    }
    portEXIT_CRITICAL(&mux);
    return added;
  }

  int pendingCount() {
    int n = 0;
    portENTER_CRITICAL(&mux);
    for (int i = 0; i < count; i++) n += slots[i].toPi ? 1 : 0;
    portEXIT_CRITICAL(&mux);
    return n;
  }

  // SensorSampleDeserializer#1#devicename#serial#secondsTime#sampleId#kind#role#driver#address#latitude#
  //   longitude#altitude#hdop#rssi#snr#heardEpoch#valueCount#id:value#id:value#...#
  // Oldest first. A value without a known driver is sent as "v<n>:value".
  int pushPendingToSerial(Print &out) {
    int sent = 0;
    if (!slots) return 0;
    for (int i = count - 1; i >= 0; i--) {
      int at = (head - 1 - i + SAMPLE_STORE_SIZE) % SAMPLE_STORE_SIZE;
      StoredSample s;
      portENTER_CRITICAL(&mux);
      bool want = slots[at].toPi;
      if (want) { s = slots[at]; slots[at].toPi = false; }
      portEXIT_CRITICAL(&mux);
      if (!want) continue;
      const SensorSampleRecord &r = s.r;
      const ModbusSensorDriver *drv = sampleDriver(r);
      out.print("SensorSampleDeserializer#1#");
      printCleanName(out, r.devicename, sizeof(r.devicename));
      out.print('#'); out.print(DeviceSerial::toString(r.serialnumberarray));
      out.print('#'); out.print(r.secondsTime);
      out.print('#'); out.print(r.sampleId);
      out.print('#'); out.print(sampleKindText(r.kind));
      out.print('#'); out.print(modbusSensorRoleId(r.role));
      out.print('#'); printCleanName(out, r.driver, sizeof(r.driver));
      out.print('#'); out.print(r.sensorAddress);
      out.print('#'); out.print(r.latitude, 7);
      out.print('#'); out.print(r.longitude, 7);
      out.print('#'); out.print(r.altitude, 1);
      out.print('#'); out.print(r.hdop, 2);
      out.print('#'); out.print(r.rssi, 0);
      out.print('#'); out.print(r.snr, 1);
      out.print('#'); out.print(s.heardEpoch);
      uint8_t n = r.valueCount > SENSOR_SAMPLE_VALUES ? SENSOR_SAMPLE_VALUES : r.valueCount;
      out.print('#'); out.print(n);
      for (uint8_t k = 0; k < n; k++) {
        out.print('#');
        if (drv && k < drv->valueCount) out.print(drv->values[k].id);
        else { out.print('v'); out.print(k); }
        out.print(':');
        printFloatOrNan(out, r.values[k], drv && k < drv->valueCount ? drv->values[k].decimals + 1 : 3);
      }
      out.println('#');
      sent++;
    }
    return sent;
  }

  // Newest first, with each value's id, label and units from the driver table
  void toJson(JsonDocument &json) {
    JsonArray list = json["samples"].to<JsonArray>();
    if (!slots) return;
    for (int i = 0; i < count; i++) {
      StoredSample s;
      portENTER_CRITICAL(&mux);
      s = slots[(head - 1 - i + SAMPLE_STORE_SIZE) % SAMPLE_STORE_SIZE];
      portEXIT_CRITICAL(&mux);
      const SensorSampleRecord &r = s.r;
      const ModbusSensorDriver *drv = sampleDriver(r);
      JsonObject o = list.add<JsonObject>();
      char name[17];
      size_t k = 0;
      for (; k < sizeof(r.devicename) && r.devicename[k]; k++) name[k] = r.devicename[k];
      name[k] = 0;
      o["device"] = name;
      o["serial"] = DeviceSerial::toString(r.serialnumberarray);
      o["time"] = r.secondsTime;
      o["heard"] = s.heardEpoch;
      o["id"] = r.sampleId;
      o["kind"] = sampleKindText(r.kind);
      o["role"] = modbusSensorRoleId(r.role);
      o["sensor"] = drv ? drv->name : "Unknown model";
      o["addr"] = r.sensorAddress;
      o["lat"] = r.latitude;
      o["lon"] = r.longitude;
      o["alt"] = r.altitude;
      o["hdop"] = r.hdop;
      o["rssi"] = r.rssi;
      JsonArray vals = o["values"].to<JsonArray>();
      uint8_t n = r.valueCount > SENSOR_SAMPLE_VALUES ? SENSOR_SAMPLE_VALUES : r.valueCount;
      for (uint8_t v = 0; v < n; v++) {
        JsonObject vo = vals.add<JsonObject>();
        if (drv && v < drv->valueCount) {
          vo["id"] = drv->values[v].id;
          vo["label"] = drv->values[v].label;
          vo["units"] = drv->values[v].units;
          vo["dec"] = drv->values[v].decimals;
        } else {
          vo["id"] = String("v") + v;
        }
        if (!isnan(r.values[v])) vo["v"] = r.values[v];
      }
    }
  }

private:
  StoredSample *slots = nullptr;
  int head = 0, count = 0;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};

// GloriaData packets waiting for the Pi (newest kept when full). On the heap: Annabelle's static RAM is full.
class GloriaDataQueue {
public:
  void begin() { q = (GloriaData *)calloc(GLORIA_PI_PENDING, sizeof(GloriaData)); }
  void add(const GloriaData &d) {
    if (!q) return;
    portENTER_CRITICAL(&mux);
    if (n == GLORIA_PI_PENDING) {  // drop the oldest
      for (int i = 1; i < n; i++) q[i - 1] = q[i];
      n--;
      dropped++;
    }
    q[n++] = d;
    portEXIT_CRITICAL(&mux);
  }
  int pendingCount() { return n; }
  uint32_t droppedCount() { return dropped; }

  // GloriaDataDeserializer#1#devicename#shortname#serial#secondsTime#batteryV#batterymA#solarV#solarmA#
  //   runtimeH#overnightmAh#temperature#batteryTemperature#rtcBatV#chemistry#rssi#snr#operatingStatus#sleep#
  //   wifiStatus#opMode#relay#pumpOn#auto#pumpFault#emergencyOpen#pumpReason#flowLpm#flowTotalL#sensors#
  //   sensorsMissing#latitude#longitude#altitude#
  int pushPendingToSerial(Print &out) {
    int sent = 0;
    while (q) {
      GloriaData d;
      portENTER_CRITICAL(&mux);
      bool have = n > 0;
      if (have) {
        d = q[0];
        for (int i = 1; i < n; i++) q[i - 1] = q[i];
        n--;
      }
      portEXIT_CRITICAL(&mux);
      if (!have) break;
      out.print("GloriaDataDeserializer#1#");
      printCleanName(out, d.devicename, sizeof(d.devicename));
      out.print('#'); printCleanName(out, d.deviceshortname, sizeof(d.deviceshortname));
      out.print('#'); out.print(DeviceSerial::toString(d.serialnumberarray));
      out.print('#'); out.print(d.secondsTime);
      out.print('#'); out.print(d.batteryVoltage, 2);
      out.print('#'); out.print(d.batteryCurrentMa, 1);
      out.print('#'); out.print(d.solarVoltage, 2);
      out.print('#'); out.print(d.solarCurrentMa, 1);
      out.print('#'); out.print(d.estimatedRuntime, 1);
      out.print('#'); out.print(d.overnightMah, 1);
      out.print('#'); out.print(d.temperature, 1);
      out.print('#'); out.print(d.batteryTemperature, 1);
      out.print('#'); out.print(d.rtcBatVolt, 2);
      out.print('#'); out.print(d.batteryChemistry);
      out.print('#'); out.print(d.rssi, 0);
      out.print('#'); out.print(d.snr, 1);
      out.print('#'); out.print((int)d.operatingStatus);
      out.print('#'); out.print((long)d.sleepTime);
      out.print('#'); out.print((int)d.wifiStatus);
      out.print('#'); out.print((int)d.opMode);
      out.print('#'); out.print(d.relayStatus);
      out.print('#'); out.print(d.pumpOn);
      out.print('#'); out.print(d.autoMode);
      out.print('#'); out.print(d.pumpFault);
      out.print('#'); out.print(d.emergencyOpen);
      out.print('#'); out.print(d.pumpReason);
      out.print('#'); out.print(d.flowRateLpm, 2);
      out.print('#'); out.print(d.flowTotalLitres, 1);
      out.print('#'); out.print(d.sensorCount);
      out.print('#'); out.print(d.sensorsMissing);
      out.print('#'); out.print(d.latitude, 7);
      out.print('#'); out.print(d.longitude, 7);
      out.print('#'); out.print(d.altitude, 1);
      out.println('#');
      sent++;
    }
    return sent;
  }

private:
  GloriaData *q = nullptr;
  int n = 0;
  uint32_t dropped = 0;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};
#endif

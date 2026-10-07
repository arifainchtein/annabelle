#include <LittleFS.h>
#include "Arduino.h"
#include <LoRa.h>
#include <FastLED.h>
#include <SPI.h>
#include <ErrorManager.h>
#include <ErrorDefinitions.h>
#include <GloriaTankFlowPumpData.h>
#include <LangleyData.h>
#include <ChinampaData.h>
#include <CommaRecord.h>
#include <DigitalStablesData.h>
#include <SeedlingMonitoringData.h>
#include <SolarPowerData.h>
#include <AnnabelleData.h>
#include <AnnabelleWifiManager.h>
#include <DataManager.h>
#include <PCF8563TimeManager.h>
#include <Esp32SecretManager.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include <DigitalStablesDataSerializer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WeatherForecastManager.h>
#include <VitalSignsRecord.h>
#include <VitalSignsStore.h>
#include <DeviceIdentityStore.h>
#include <esp_system.h>
#include <esp_core_dump.h>

SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// Display settings for 2.4" SSD1309 — same OLED hardware as Paula
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define SLEEP_SWITCH_26 26
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int SWITCH_PIN_LEFT = 32;
const int SWITCH_PIN_RIGHT = 33;

#define LED_PIN 19
#define NUM_LEDS 6

WeatherForecastManager* weatherForecastManager;

DataManager dataManager(Serial, LittleFS);
ErrorManager errorManager;
LoRaError cadResult;
float avgRssi = 0;
uint8_t loraActivitySeconds = 0;  // counts down each second; keeps leds[3,5] lit after a LoRa rx
bool weatherDownloading = false;  // true while HTTP download is in progress
bool weatherPending = false;      // set when trigger fires; runs when serial is free
unsigned long secondsSinceLastAsyncData = 60;  // starts in the yellow band until the Pi's first AsyncData; counts up each second; reset when the Pi pulls data via AsyncData

boolean debug = false;

String LIFE_CYCLE_EVENT_START_SYNCHRONOUS_CYCLE = "Start Synchronous Cycle";
String LIFE_CYCLE_EVENT_END_SYNCHRONOUS_CYCLE = "End Synchronous Cycle";
String LIFE_CYCLE_EVENT_START_ASYNCHRONOUS_CYCLE = "Start Asynchronous Cycle";
String LIFE_CYCLE_EVENT_END_ASYNCHRONOUS_CYCLE = "End Asynchronous Cycle";
String LIFE_CYCLE_EVENT_END_AWAKE = "End Awake";
String LIFE_CYCLE_EVENT_START_AWAKE = "Start Awake";
String COMMAND_REBOOT = "$Reboot";
String COMMAND_SHUTDOWN = "$Shutdown";
String USER_COMMAND = "UserCommand";

// PI_CONTROL is part of the board design (a physical button wired to the Pi for
// reboot/shutdown signalling) even though current production hardware doesn't
// populate it yet. Kept defined so the firmware already supports it once it is.
#define PI_CONTROL 39
#define PI_CONTROL_SHORT_PRESS_TIME 1000  // milliseconds
#define PI_CONTROL_LONG_PRESS_TIME 3000   // milliseconds
int piControlLastState = LOW;
int piControlCurrentState;
unsigned long piControlPressedTime = 0;
unsigned long piControlReleasedTime = 0;
String userCommandState = "Ok";
#define PI_STATE_SYNC 0
#define PI_STATE_ASYNC 1
#define PI_STATE_UNDEFINED 2
uint8_t piCurrentState;

bool gloriaTankFlowPumpNewData = false;
bool digitalStablesDataNewData = false;
bool seedlingMonitoringDataNewData = false;
bool chinampaDataNewData = false;
bool commaDataNewData = false;
bool langleyDataNewData = false;

OneWire oneWire(27);
DallasTemperature tempSensor(&oneWire);
#define SCK 14
#define MOSI 13
#define MISO 12
#define LoRa_SS 15
#define LORA_RESET 16
#define LORA_DI0 17

#define RTC_BATT_VOLT 36
#define OP_MODE 34
#define RTC_CLK_OUT 4

// LoRa parameters, registers and constants
#define REG_OP_MODE            0x01
#define REG_IRQ_FLAGS          0x12
#define REG_RSSI_VALUE         0x1B
#define MODE_CAD               0x87
#define IRQ_CAD_DONE_MASK      0x04
#define IRQ_CAD_DETECTED_MASK  0x02

#define CAD_TIMEOUT      5000    // CAD timeout in milliseconds
#define MAX_RETRIES      5       // Maximum transmission retries
#define MIN_BACKOFF      500     // Minimum backoff time in milliseconds
#define MAX_BACKOFF      1500    // Maximum backoff time in milliseconds
#define RSSI_THRESHOLD   -60     // RSSI threshold in dBm

bool timeIsSet = false;
DigitalStablesDataSerializer digitalStablesDataSerializer;

PanchoConfigData panchoConfigData;
AnnabelleData annabelleData;

//
// Every LoRa struct is identified by its size alone, so all of them must differ - checked at
// compile time instead of discovered in the field. Covers what Annabelle receives plus what it
// sends (devices dispatch on size too). Member function, not a free one, so the Arduino
// prototype generator leaves it alone.
//
struct LoRaPacketSizeCheck {
  static constexpr bool allDistinct(const size_t* sizes, size_t n, size_t i, size_t j) {
    return i >= n ? true
         : j >= n ? allDistinct(sizes, n, i + 1, i + 2)
         : (sizes[i] != sizes[j] && allDistinct(sizes, n, i, j + 1));
  }
};
constexpr size_t loraPacketSizes[] = {
  sizeof(LangleyData), sizeof(GloriaTankFlowPumpData), sizeof(DigitalStablesData), sizeof(ChinampaData),
  sizeof(SeedlingMonitorData), sizeof(CommaRecord), sizeof(VitalSignsRecord), sizeof(WeatherForecastUpdate),
  sizeof(GraveyardShiftUpdate), sizeof(RequestCommand), sizeof(DeviceIdentityRecord)
};
static_assert(LoRaPacketSizeCheck::allDistinct(loraPacketSizes, sizeof(loraPacketSizes) / sizeof(loraPacketSizes[0]), 0, 1),
              "two LoRa packet structs have the same size - receivers dispatch on size, so they would be confused");

// Latest VitalSignsRecord per device, relayed to the Pi with AsyncData - see VitalSignsStore.h
VitalSignsStore vitalSignsStore;
// Latest DeviceIdentityRecord per device (product definition + running build), relayed the same way
DeviceIdentityStore deviceIdentityStore;

String serialNumber;
uint8_t delayTime = 10;

CRGB leds[NUM_LEDS];

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

String secondsSinceLastDataSampling;
PCF8563TimeManager timeManager(Serial);
GeneralFunctions generalFunctions;
Esp32SecretManager secretManager(timeManager);
String secret;

AnnabelleWifiManager wifiManager(Serial, LittleFS, dataManager, timeManager, secretManager, annabelleData);

byte msgCount = 0;  // count of outgoing messages

String currentIpAddress = "No IP";
bool inPulse = true;
String ipAddress = "";

bool internetAvailable;
bool loraActive = false;
bool inSerial = false;

RTCInfoRecord currentTimerRecord;
volatile bool clockTicked = false;
volatile bool loraReceived = false;
volatile int loraPacketSize = 0;
// Set by onLoraDio0() (DIO0 rose: RX done, or TX done during a send). loop() then reads the radio
// with LoRa.parsePacket(). The radio is NOT touched from the interrupt: arduino-LoRa's own
// LoRa.onReceive() handler does SPI transactions inside the ISR, which take the SPI bus mutex -
// when that collided with SPI use in loop() the board panicked with
// "assert failed: xQueueSemaphoreTake queue.c:1718" (core dump 2026-10-05, every few minutes).
volatile bool loraDio0Fired = false;
int lastReceivedPacketSize = 0;

RTCInfoRecord lastReceptionRTCInfoRecord;

GloriaTankFlowPumpData gloriaTankFlowPumpData;
LangleyData langleyData;
DigitalStablesData digitalStablesData;
ChinampaData chinampaData;
SeedlingMonitorData seedlingMonitorData;
CommaRecord commaRecord;
String timezone;

// RTC <- internet time. The PCF8563 has no daylight-saving logic and the only code that set it
// from the internet (SetTimeFromInternet) ran on a manual serial command, so on 2026-10-04 (DST
// start) Annabelle stayed on standard time. configTzTime() in setup() starts SNTP, which keeps
// syncing in the background; syncRtcFromNtp() copies the local time (TZ rule in `timezone`, DST
// included) into the RTC whenever they differ by more than RTC_SYNC_TOLERANCE_SEC. Tried every
// minute until the first successful check after boot, then hourly at minute 7.
#define RTC_SYNC_TOLERANCE_SEC 5
bool rtcCheckedSinceBoot = false;

CRGBPalette16 currentPalette;
TBlendType currentBlending;

// Distinct LED-show palettes so a LoRa reception looks different from a Pi AsyncData pull
CRGBPalette16 loraReceivePalette(CRGB::Red, CRGB::HotPink, CRGB::LightPink, CRGB::White);
CRGBPalette16 asyncDataPalette(CRGB::Yellow, CRGB::Orange, CRGB::Yellow, CRGB::Orange);

bool switchPositionLeft = true;  // left position = show received data, right position = show weather forecast
bool weatherScreenShown = false;  // the weather page is on the OLED, so loop() may refresh its clock line (cleared by centerText())

//
// Reset diagnostics - the PCB has no external watchdog, so record why the ESP32
// last reset (esp_reset_reason) plus how long it had been up, to tell apart
// power-loss/EN-pin resets (both POWERON), brownouts and firmware crashes.
//
#define RESET_LOG_FILE "/resetlog.txt"
#define RESET_LOG_MAX_LINES 20
#define RESET_DIAG_MAGIC 0xA5B3C7D1
// RTC_NOINIT survives software resets, panics, watchdog resets and usually brownouts,
// but not a power-on or an EN-pin reset - the magic tells us whether it's valid.
RTC_NOINIT_ATTR uint32_t resetDiagMagic;
RTC_NOINIT_ATTR uint32_t resetDiagBootCount;
RTC_NOINIT_ATTR uint32_t resetDiagUptimeSeconds;
RTC_NOINIT_ATTR uint32_t resetDiagMinFreeHeap;  // ESP.getMinFreeHeap() as of the last second before the reset
String resetReason = "";
uint32_t previousUptimeSeconds = 0;
uint32_t previousMinFreeHeap = 0;  // lowest free heap reached before the reset (0 = unknown) - a falling value points to a leak
bool previousUptimeKnown = false;
//
// Crash capture - the ESP32 core writes a core dump to the "coredump" flash partition on every
// panic. At boot recordCoreDump() turns it into one line, keeps it in LAST_CRASH_FILE (newest
// crash only, survives power loss) and appends it to RESET_LOG_FILE; the dump stays in flash for
// esp-coredump and a fingerprint stops it being reported twice. The line rides on every AsyncData terminator as "|Crash=..." until the next crash:
//   <rtc date time>;<panic reason>;<task>;<pc>;<backtrace pcs, space separated>;<elf sha256 prefix>;
//   cause=<exccause> vaddr=<excvaddr> a0=<return address> epc1=... (EPC registers present in the dump)
// Decode the addresses with xtensa-esp32-elf-addr2line -pfiaC -e Annabelle.ino.elf <pc> <bt...>
// against the .elf of the build that crashed (the sha prefix identifies it) - keep a copy of the
// .elf of every build that gets flashed (Projects/Annabelle/claude/builds/).
//
#define LAST_CRASH_FILE "/lastcrash.txt"
String lastCrash = "";

//
// Lora Functions
//

void LoRa_rxMode() {
  LoRa.disableInvertIQ();  // normal mode
  LoRa.receive();          // set receive mode
}

void LoRa_txMode() {
  LoRa.idle();              // set standby mode
  LoRa.disableInvertIQ();   // normal mode
}

LoRaError performCAD() {
  if (!loraActive) {
    errorManager.setLoRaError(LORA_INIT_FAILED);
    return LORA_INIT_FAILED;
  }
  const int SAMPLES = 3;
  const int CHECK_DELAY = 2;
  float rssiSum = 0;
  LoRa.idle();
  LoRa.receive();
  for (int i = 0; i < SAMPLES; i++) {
    rssiSum += LoRa.rssi();
    delay(CHECK_DELAY);
  }
  avgRssi = rssiSum / SAMPLES;
  if (avgRssi > RSSI_THRESHOLD) {
    LoRa.idle();
    errorManager.setLoRaError(LORA_CHANNEL_BUSY);
    return LORA_CHANNEL_BUSY;
  }
  rssiSum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    rssiSum += LoRa.rssi();
    delay(CHECK_DELAY);
  }
  avgRssi = rssiSum / SAMPLES;
  LoRa.idle();
  if (debug) Serial.print("checkcad, avgRssi=");
  if (debug) Serial.println(avgRssi);
  if (avgRssi > RSSI_THRESHOLD) {
    return LORA_CHANNEL_BUSY;
  }
  errorManager.clearLoRaError(LORA_CHANNEL_BUSY);
  return LORA_OK;
}

template <typename T>
uint8_t calculateChecksum(const T& data) {
  uint8_t checksum = 0;
  const uint8_t* dataPtr = (const uint8_t*)&data;

  size_t checksumOffset = 0;
  if constexpr (std::is_same<T, DigitalStablesData>::value) {
    checksumOffset = offsetof(DigitalStablesData, checksum);
  } else if constexpr (std::is_same<T, CommaRecord>::value) {
    checksumOffset = offsetof(CommaRecord, checksum);
  } else if constexpr (std::is_same<T, LangleyData>::value) {
    checksumOffset = offsetof(LangleyData, checksum);
  } else if constexpr (std::is_same<T, VitalSignsRecord>::value) {
    checksumOffset = offsetof(VitalSignsRecord, checksum);
  } else if constexpr (std::is_same<T, DeviceIdentityRecord>::value) {
    checksumOffset = offsetof(DeviceIdentityRecord, checksum);
  }

  for (size_t i = 0; i < checksumOffset; i++) {
    checksum ^= dataPtr[i];
  }
  for (size_t i = checksumOffset + sizeof(uint8_t); i < sizeof(T); i++) {
    checksum ^= dataPtr[i];
  }
  return checksum;
}

template <typename T>
bool validateChecksum(const T& receivedData) {
  T tempData = receivedData;
  uint8_t receivedChecksum = 0;

  if constexpr (std::is_same<T, DigitalStablesData>::value) {
    receivedChecksum = tempData.checksum;
    tempData.checksum = 0;
  } else if constexpr (std::is_same<T, CommaRecord>::value) {
    receivedChecksum = tempData.checksum;
    tempData.checksum = 0;
  } else if constexpr (std::is_same<T, LangleyData>::value) {
    receivedChecksum = tempData.checksum;
    tempData.checksum = 0;
  } else if constexpr (std::is_same<T, VitalSignsRecord>::value) {
    receivedChecksum = tempData.checksum;
    tempData.checksum = 0;
  } else if constexpr (std::is_same<T, DeviceIdentityRecord>::value) {
    receivedChecksum = tempData.checksum;
    tempData.checksum = 0;
  }

  uint8_t calculatedChecksum = calculateChecksum(tempData);
  return (calculatedChecksum == receivedChecksum);
}

// Field time sync: a WeatherForecastUpdate carries Annabelle's time stamped for the end of its
// transmit. 169 bytes at SF9/125 kHz is ~0.86 s on air, stamped just before beginPacket().
#define WEATHER_AIRTIME_SEC 1

// WEATHER_FLAG_DST when the TZ rule says daylight saving is in effect now (0 if NTP never set
// the system clock). Re-asserts TZ first - see syncRtcFromNtp().
uint8_t annabelleDstFlag() {
  setenv("TZ", timezone.c_str(), 1);
  tzset();
  struct tm now;
  if (!getLocalTime(&now, 0)) return 0;
  return now.tm_isdst > 0 ? WEATHER_FLAG_DST : 0;
}

template <typename T>
int sendMessage(const T& inputData) {
  T dataToSend = inputData;
  if (debug) Serial.print("sending lora, size=");
  if (debug) Serial.println(sizeof(T));
  long code = secretManager.generateCode();
  if constexpr (std::is_same<T, DigitalStablesData>::value) {
    dataToSend.totpcode = code;
    dataToSend.checksum = 0;
    dataToSend.checksum = calculateChecksum(dataToSend);
  }
  LoRa_txMode();
  uint8_t result = 99;
  int retries = 0;
  boolean keepGoing = true;
  long startsendingtime = millis();
  while (keepGoing) {
    cadResult = performCAD();
    if (cadResult == LORA_OK) {
      if constexpr (std::is_same<T, WeatherForecastUpdate>::value) {
        // Stamped here, after any CAD back-off, and the code calculated at that same time so
        // receivers can check it without trusting their own clocks.
        dataToSend.annabelleTime = (uint32_t)(timeManager.getTimeForCodeGeneration() + WEATHER_AIRTIME_SEC);
        dataToSend.flags = annabelleDstFlag();
        dataToSend.totpcode = secretManager.generateCodeAt(dataToSend.annabelleTime);
      }
      LoRa.beginPacket();
      LoRa.write((uint8_t *)&dataToSend, sizeof(T));
      if (!LoRa.endPacket()) {
        result = LORA_TX_FAILED;
      } else {
        result = LORA_OK;
      }
      if (debug) Serial.print("took ");
      if (debug) Serial.println(millis() - startsendingtime);
      keepGoing = false;
    } else if (cadResult == LORA_CHANNEL_BUSY) {
      int backoff = random(MIN_BACKOFF * (1 << retries), MAX_BACKOFF * (1 << retries));
      if (debug) Serial.print("Channel busy, retry ");
      if (debug) Serial.print(retries + 1);
      if (debug) Serial.print(" of ");
      if (debug) Serial.println(MAX_RETRIES);
      delay(backoff);
      retries++;
      keepGoing = retries < MAX_RETRIES;
    } else {
      result = cadResult;
      keepGoing = false;
    }
  }
  if (result == 99) result = LORA_MAX_RETRIES_REACHED;
  if (debug) Serial.print("Lora returns ");
  if (debug) Serial.println(result);
  LoRa_rxMode();
  msgCount++;
  return result;
}

void processLora(int packetSize) {
  if (debug) Serial.println("Received lora size=" + String(packetSize));
  if (packetSize == 0) return;  // if there's no packet, return

  if (packetSize == sizeof(LangleyData)) {
    memset(&langleyData, 0, sizeof(LangleyData));
    LoRa.readBytes((uint8_t*)&langleyData, sizeof(LangleyData));
    // Validate on the as-received bytes before touching anything else - rssi/snr live inside
    // the checksummed range of the struct, and Langley_West's checksum was computed over what
    // it actually transmitted, which can't include the receiver's own RSSI/SNR. Overwriting
    // them first (as this used to do) made every checksum fail unconditionally, silently
    // dropping every Langley packet - see conversation 2026-07-21.
    bool checksumOk = validateChecksum(langleyData);
    langleyData.rssi = LoRa.packetRssi();
    langleyData.snr = LoRa.packetSnr();
    if (debug) {
      Serial.print("received langleyData from ");
      Serial.print(langleyData.devicename);
      Serial.print(" cksum=");
      Serial.println(checksumOk ? "ok" : "BAD");
    }
    // Corrupt LoRa packets (weak-signal bit errors) can mangle the device name - e.g.
    // "Langley_West" garbled to something else - which would otherwise silently create/update
    // the wrong Telepathon DeneChain. Dropping on a bad checksum keeps this dynamic (new
    // devices like a future "Langley_East" still register automatically, no allow-list needed)
    // while rejecting whole packets whose bytes don't match what was actually transmitted -
    // see conversation 2026-07-20.
    if (checksumOk) {
      dataManager.storeLangleyData(langleyData);
      langleyDataNewData = true;
    }

  } else if (packetSize == sizeof(GloriaTankFlowPumpData)) {
    memset(&gloriaTankFlowPumpData, 0, sizeof(GloriaTankFlowPumpData));
    LoRa.readBytes((uint8_t*)&gloriaTankFlowPumpData, sizeof(GloriaTankFlowPumpData));
    gloriaTankFlowPumpData.rssi = LoRa.packetRssi();
    gloriaTankFlowPumpData.snr = LoRa.packetSnr();
    dataManager.storeGloria(gloriaTankFlowPumpData);
    gloriaTankFlowPumpNewData = true;
    if (debug) Serial.print("received GloriaTankFlowPumpData from ");
    if (debug) Serial.println(gloriaTankFlowPumpData.devicename);

  } else if (packetSize == sizeof(DigitalStablesData)) {
    memset(&digitalStablesData, 0, sizeof(DigitalStablesData));
    LoRa.readBytes((uint8_t*)&digitalStablesData, sizeof(DigitalStablesData));
    digitalStablesData.rssi = LoRa.packetRssi();
    digitalStablesData.snr = LoRa.packetSnr();
    dataManager.storeDigitalStablesData(digitalStablesData);
    if (debug) dataManager.printDigitalStablesData(digitalStablesData);
    digitalStablesDataNewData = true;
    if (debug) Serial.print("received digitalStablesData from ");
    if (debug) Serial.print(digitalStablesData.devicename);
    if (debug) Serial.print(" ");
    if (debug) Serial.print(TimeUtils::epochToString(digitalStablesData.secondsTime));
    if (debug) Serial.print(" voltage=");
    if (debug) Serial.print(digitalStablesData.batteryVoltage);
    if (debug) Serial.print(" current=");
    if (debug) Serial.print(digitalStablesData.batteryCurrent);
    if (debug) Serial.print(" operatingStatus=");
    if (debug) Serial.println(digitalStablesData.operatingStatus);

  } else if (packetSize == sizeof(ChinampaData)) {
    memset(&chinampaData, 0, sizeof(ChinampaData));
    LoRa.readBytes((uint8_t*)&chinampaData, sizeof(ChinampaData));
    chinampaData.rssi = LoRa.packetRssi();
    chinampaData.snr = LoRa.packetSnr();
    dataManager.storeChinampaData(chinampaData);
    chinampaDataNewData = true;
    if (debug) Serial.print("received chinampaData from ");
    if (debug) Serial.println(chinampaData.devicename);

  } else if (packetSize == sizeof(SeedlingMonitorData)) {
    memset(&seedlingMonitorData, 0, sizeof(SeedlingMonitorData));
    LoRa.readBytes((uint8_t*)&seedlingMonitorData, sizeof(SeedlingMonitorData));
    seedlingMonitorData.rssi = LoRa.packetRssi();
    seedlingMonitorData.snr = LoRa.packetSnr();
    dataManager.storeSeedlingMonitorData(seedlingMonitorData);
    seedlingMonitoringDataNewData = true;
    if (debug) Serial.print("received SeedlingMonitorData from ");
    if (debug) Serial.println(seedlingMonitorData.devicename);

  } else if (packetSize == sizeof(CommaRecord)) {
    memset(&commaRecord, 0, sizeof(CommaRecord));
    LoRa.readBytes((uint8_t*)&commaRecord, sizeof(CommaRecord));
    bool checksumOk = validateChecksum(commaRecord);
    if (debug) {
      Serial.print("CommaRecord from ");
      Serial.print(commaRecord.devicename);
      Serial.print(" idx=");
      Serial.print(commaRecord.index);
      Serial.print("/");
      Serial.print(commaRecord.total);
      Serial.print(" v=");
      Serial.print(commaRecord.voltage);
      Serial.print(" cksum=");
      Serial.println(checksumOk ? "ok" : "BAD");
    }
    if (checksumOk) {
      dataManager.storeCommaRecord(commaRecord);
      commaDataNewData = true;
    }

  } else if (packetSize == sizeof(VitalSignsRecord)) {
    // Sent by each device right after its data pulse - only the latest per device is kept
    // (running totals), see VitalSignsStore.h. rssi/snr are Annabelle's own measurement of
    // this packet, kept outside the checksummed record.
    VitalSignsRecord vitalSignsRecord;
    LoRa.readBytes((uint8_t*)&vitalSignsRecord, sizeof(VitalSignsRecord));
    bool checksumOk = validateChecksum(vitalSignsRecord);
    if (debug) {
      Serial.print("VitalSigns seq=");
      Serial.print(vitalSignsRecord.seq);
      Serial.print(" resets=");
      Serial.print(vitalSignsRecord.resetCount);
      Serial.print(" cksum=");
      Serial.println(checksumOk ? "ok" : "BAD");
    }
    if (checksumOk) {
      vitalSignsStore.store(vitalSignsRecord, LoRa.packetRssi(), LoRa.packetSnr());
    }

  } else if (packetSize == sizeof(DeviceIdentityRecord)) {
    // Product definition + running build, sent rarely (after a real reset, after
    // SetProductDefinition, daily) - only the latest per device is kept, see DeviceIdentityStore.h.
    DeviceIdentityRecord deviceIdentityRecord;
    LoRa.readBytes((uint8_t*)&deviceIdentityRecord, sizeof(DeviceIdentityRecord));
    bool checksumOk = validateChecksum(deviceIdentityRecord);
    if (debug) {
      Serial.print("DeviceIdentity build=");
      Serial.print(deviceIdentityRecord.firmwareBuild);
      Serial.print(" labelBuild=");
      Serial.print(deviceIdentityRecord.labelBuild);
      Serial.print(" cksum=");
      Serial.println(checksumOk ? "ok" : "BAD");
    }
    if (checksumOk) {
      deviceIdentityStore.store(deviceIdentityRecord, LoRa.packetRssi(), LoRa.packetSnr());
    }
  }
}

//
// end of lora functions
//
// interrupt functions
//

void IRAM_ATTR clockTick() {
  portENTER_CRITICAL_ISR(&mux);
  clockTicked = true;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR onLoraDio0() {
  loraDio0Fired = true;
}

//
// end of interrupt functions
//

// FastLED.show() must never run on two cores at once: the RMT driver returns an error for a
// refresh started while the previous one is still sending, and FastLED's ESP_ERROR_CHECK turns
// that into abort(). That was the 2026-10-05 19:46 PANIC (loop() -> updateStatusLeds() on core 1
// while ledShowTask was animating on core 0). Every show goes through showLeds().
SemaphoreHandle_t ledMutex = NULL;  // created in setup() right after FastLED.addLeds
volatile bool runLedShow = false;   // set by loop(), cleared by ledShowTask when the animation ends

void showLeds() {
  if (ledMutex) xSemaphoreTake(ledMutex, portMAX_DELAY);
  FastLED.show();
  if (ledMutex) xSemaphoreGive(ledMutex);
}

// Updates the 3 status LEDs (leds[0..2]) based on current system state.
// leds[3,5] are LoRa/weather activity LEDs; leds[4] is the Pi-AsyncData heartbeat.
void updateStatusLeds() {
  if (runLedShow) return;  // ledShowTask owns the strip during an animation
  // LED 0 — WiFi: green=internet ok, yellow=IP but ping failed, red=no connection
  if (wifiManager.getInternetAvailable()) {
    leds[0] = CRGB(0, 255, 0);
  } else {
    String ip = wifiManager.getIpAddress();
    bool hasIp = (ip.length() > 0 && ip != "0.0.0.0");
    leds[0] = hasIp ? CRGB(255, 200, 0) : CRGB(255, 0, 0);
  }
  // LED 1 — LoRa
  leds[1] = loraActive ? CRGB(0, 255, 0) : CRGB(255, 0, 0);
  // LED 2 — weather data: green=data available, red=no data yet
  leds[2] = weatherForecastManager->hasValidForecasts() ? CRGB(0, 255, 0) : CRGB(255, 0, 0);
  // LEDs 3,5 — activity: blue=downloading, green=recent LoRa rx (stays 3s), off otherwise
  if (weatherDownloading) {
    leds[3] = leds[5] = CRGB(0, 150, 255);
  } else if (loraActivitySeconds > 0) {
    leds[3] = leds[5] = CRGB(0, 255, 0);
    loraActivitySeconds--;
  } else {
    leds[3] = leds[5] = CRGB(0, 0, 0);
  }
  // LED 4 — time since the Pi last pulled data via AsyncData: green <1min, yellow <2min, red otherwise
  if (secondsSinceLastAsyncData < 150) {
    leds[4] = CRGB(0, 255, 0);
  } else if (secondsSinceLastAsyncData < 240) {
    leds[4] = CRGB(255, 255, 0);
  } else {
    leds[4] = CRGB(255, 0, 0);
  }
  showLeds();
}

TaskHandle_t ledShowTask = NULL;
int ledShowDuration = 250;

// Brightness follows a sine wave across the strip (sin8), so each LED's intensity
// is a snapshot of a traveling wave rather than a flat color fill. As colorIndex
// advances over time, the bright peak of that wave moves from leds[0] to leds[5].
void FillLEDsFromPaletteColors(uint8_t colorIndex) {
  const uint8_t wavelength = 255 / NUM_LEDS;
  for (int i = 0; i < NUM_LEDS; ++i) {
    uint8_t brightness = sin8(colorIndex - (i * wavelength));
    leds[i] = ColorFromPalette(currentPalette, colorIndex, brightness, currentBlending);
  }
}

void performLedShow(int millisseconds) {
  long startmillis = millis();
  long currentMillis = startmillis;
  static uint8_t startIndex = 0;
  while (currentMillis < startmillis + millisseconds) {
    startIndex += 4;  // slow, steady left-to-right wave motion
    FillLEDsFromPaletteColors(startIndex);
    showLeds();
    delay(30);  // pace the animation so it's a smooth wave, not a flicker
    currentMillis = millis();
  }
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  showLeds();
}

void ledShowTaskFunction(void* parameter) {
  while (true) {
    if (runLedShow) {
      performLedShow(ledShowDuration);
      runLedShow = false;
    }
    vTaskDelay(1);
  }
}

void centerText(String text, int y) {
  weatherScreenShown = false;  // every other page draws its header with centerText()
  display.setTextSize(1);
  display.setTextColor(WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - w) / 2;
  display.setCursor(x, y);
  display.print(text);
}

bool isSentByArraySet(uint8_t* arr, size_t size) {
  for (size_t i = 0; i < size; i++) {
    if (arr[i] != 0) return true;
  }
  return false;
}

void formatRain(char* out, int size, double val) {
  if (val < 1.0) {
    int frac = min((int)round(val * 100), 99);
    snprintf(out, size, ".%02d", frac);
  } else {
    snprintf(out, size, "%3d", (int)round(val));
  }
}

// Page header: device name, a space and the RTC time as hh:mm (the name is cut to 15 characters
// so the line fits in 21 characters = 126 px).
String titleWithTime(String name) {
  char t[6];
  snprintf(t, sizeof(t), "%02d:%02d", currentTimerRecord.hour, currentTimerRecord.minute);
  if (name.length() > 15) name = name.substring(0, 15);
  return name + " " + t;
}

// Weather page top line: Annabelle's RTC date and time. Redrawn every second by loop() while the
// page is shown, so the OLED always shows the clock Annabelle is using.
void drawWeatherTitle() {
  char title[22];
  snprintf(title, sizeof(title), "WF %d/%d/%02d %02d:%02d:%02d",
           currentTimerRecord.date,
           currentTimerRecord.month,
           currentTimerRecord.year % 100,
           currentTimerRecord.hour,
           currentTimerRecord.minute,
           currentTimerRecord.second);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(title);
}

void showWeatherForecast() {
  WeatherForecast* forecasts = weatherForecastManager->getForecasts();
  if (forecasts == nullptr) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  drawWeatherTitle();

  display.setCursor(0, 9);
  char buf[22];

  snprintf(buf, sizeof(buf), "C %2dh %3d %2dh %3d",
           forecasts[0].hour, forecasts[0].cloudiness,
           forecasts[1].hour, forecasts[1].cloudiness);
  display.println(buf);
  snprintf(buf, sizeof(buf), "  %2dh %3d %2dh %3d",
           forecasts[2].hour, forecasts[2].cloudiness,
           forecasts[3].hour, forecasts[3].cloudiness);
  display.println(buf);

  snprintf(buf, sizeof(buf), "T %2dh %3d %2dh %3d",
           forecasts[0].hour, (int)forecasts[0].temperature,
           forecasts[1].hour, (int)forecasts[1].temperature);
  display.println(buf);
  snprintf(buf, sizeof(buf), "  %2dh %3d %2dh %3d",
           forecasts[2].hour, (int)forecasts[2].temperature,
           forecasts[3].hour, (int)forecasts[3].temperature);
  display.println(buf);

  char r0[5], r1[5], r2[5], r3[5];
  formatRain(r0, sizeof(r0), forecasts[0].rain);
  formatRain(r1, sizeof(r1), forecasts[1].rain);
  formatRain(r2, sizeof(r2), forecasts[2].rain);
  formatRain(r3, sizeof(r3), forecasts[3].rain);
  snprintf(buf, sizeof(buf), "R %2dh %s %2dh %s",
           forecasts[0].hour, r0, forecasts[1].hour, r1);
  display.println(buf);
  snprintf(buf, sizeof(buf), "  %2dh %s %2dh %s",
           forecasts[2].hour, r2, forecasts[3].hour, r3);
  display.println(buf);

  display.display();
  weatherScreenShown = true;
}

void showChinampaPage1() {
  display.clearDisplay();
  centerText(titleWithTime(chinampaData.devicename), 0);

  display.setTextSize(1);
  display.setCursor(0, 10);
  if (chinampaData.alertstatus) {
    if (chinampaData.alertcode > 0 && chinampaData.alertcode < 10) {
      display.print("Alrt:");
      if (chinampaData.alertcode == 1) {
        display.println("Fish Data Stale");
      } else if (chinampaData.alertcode == 2) {
        display.println("Sump Stale");
      } else if (chinampaData.alertcode == 3) {
        display.println("Fish & Sump Stale");
      } else if (chinampaData.alertcode == 4) {
        display.println("Fish flow<2");
      } else if (chinampaData.alertcode == 5) {
        display.println("Sump too low");
      } else if (chinampaData.alertcode == 6) {
        display.println("Add H20");
      } else if (chinampaData.alertcode == 7) {
        display.println("Fish Sensor Bad");
      } else if (chinampaData.alertcode == 8) {
        display.println("Sump Sensor Bad");
      } else if (chinampaData.alertcode == 9) {
        display.println("F & S Sensors Bad");
      } else {
        display.println(chinampaData.alertcode);
      }
    }
  }

  display.print("Pump: ");
  if (chinampaData.pumprelaystatus) {
    display.print("ON");
  } else {
    display.print("OFF");
  }

  display.print(" FS:");
  if (chinampaData.fishtankoutflowsolenoidrelaystatus) {
    display.println("OPEN");
  } else {
    display.println("CLOSED");
  }

  display.print("F Last:");
  if (chinampaData.secondsSinceLastFishTankData > 0 && chinampaData.secondsSinceLastFishTankData < 1000) {
    display.print(chinampaData.secondsSinceLastFishTankData);
  } else {
    display.print("Bad");
  }
  display.print(" Flow:");
  display.println(chinampaData.fishtankoutflowflowRate);

  display.print("S Last:");
  if (chinampaData.secondsSinceLastSumpTroughData > 0 && chinampaData.secondsSinceLastSumpTroughData < 1000) {
    display.print(chinampaData.secondsSinceLastSumpTroughData);
  } else {
    display.print("Bad");
  }

  display.print(" H:");
  display.print((int)chinampaData.sumpTroughMeasuredHeight);

  if (chinampaData.sumpTroughMeasuredHeight >= (chinampaData.sumpTroughHeight - chinampaData.minimumSumpTroughLevel)) {
    display.println(" Red");
  } else if (chinampaData.sumpTroughMeasuredHeight < (chinampaData.sumpTroughHeight - chinampaData.minimumSumpTroughLevel) && chinampaData.sumpTroughMeasuredHeight >= (chinampaData.sumpTroughHeight - chinampaData.maximumSumpTroughLevel)) {
    display.println(" Green");
  } else if (chinampaData.sumpTroughMeasuredHeight < (chinampaData.sumpTroughHeight - chinampaData.maximumSumpTroughLevel)) {
    display.println("  Blue");
  }

  display.print("uT:");
  display.print(chinampaData.microtemperature);
  display.print(" OT:");
  display.print(chinampaData.outdoortemperature);
  display.print(" RTC:");
  display.print(chinampaData.rtcBatVolt);
  display.println("V");
  float rssi = chinampaData.rssi;
  float snr = chinampaData.snr;
  display.print("rssi:");
  display.print((int)rssi);
  display.print(" snr:");
  display.println(snr);
  display.display();
}

void showChinampaPage2() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  centerText(titleWithTime(chinampaData.devicename), 0);
  display.setCursor(0, 10);

  if (chinampaData.sensorstatus[0]) {
    display.println("uTemp high");
  } else {
    display.println("uTemp ok");
  }
  if (chinampaData.sensorstatus[1]) {
    display.println("Fish Tank Change>10%");
  } else {
    display.println("Fish Tank Ok");
  }
  if (chinampaData.sensorstatus[2]) {
    display.println("Sump Height Change>10%");
  } else {
    display.println("Sump Height Ok");
  }
  display.display();
}

void showDigitalStablesDetail() {
  display.clearDisplay();
  centerText(titleWithTime(digitalStablesData.devicename), 0);
  display.setTextSize(1);
  display.setCursor(0, 10);

  if (isSentByArraySet(digitalStablesData.sentbyarray, sizeof(digitalStablesData.sentbyarray))) {
    display.print("Sent by: ");
    char buffer[9];
    memcpy(buffer, digitalStablesData.sentbyarray, 8);
    buffer[8] = '\0';
    display.println(buffer);
  }

  display.print(digitalStablesData.batteryVoltage);
  display.print("V ");
  display.print(digitalStablesData.batteryCurrent);
  display.print("mA ");
  display.println(digitalStablesData.estimatedRuntime);

  display.print("rssi:");
  display.print((int)digitalStablesData.rssi);
  display.print(" snr:");
  display.println(digitalStablesData.snr);
  display.print("H: ");
  display.print(digitalStablesData.measuredHeight);

  if (digitalStablesData.measuredHeight >= (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelminimumcm)) {
    display.println("  Red");
  } else if (digitalStablesData.measuredHeight < (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelminimumcm) && digitalStablesData.measuredHeight >= (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelmaximumcm)) {
    display.println("  Green");
  } else if (digitalStablesData.measuredHeight < (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelmaximumcm)) {
    display.println("  Blue");
  }

  display.print("Mi:");
  display.print((int)digitalStablesData.troughlevelminimumcm);
  display.print(" Ma:");
  display.print((int)digitalStablesData.troughlevelmaximumcm);
  display.print(" TH: ");
  display.println((int)digitalStablesData.maximumScepticHeight);

  display.print("u Temp:");
  display.print(digitalStablesData.temperature);
  display.print(" Out T:");
  display.print(digitalStablesData.outdoortemperature);
  display.display();
}

void showCommaRecordPage() {
  display.clearDisplay();
  centerText(titleWithTime(commaRecord.devicename), 0);
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("V:");
  display.print(commaRecord.voltage, 3);
  display.print("V  ");
  display.print(commaRecord.index);
  display.print("/");
  display.println(commaRecord.total);
  display.print("rssi:");
  display.print((int)LoRa.packetRssi());
  display.print(" snr:");
  display.println(LoRa.packetSnr());
  display.display();
}

void showLangleyPage() {
  display.clearDisplay();
  centerText(titleWithTime(langleyData.devicename), 0);
  display.setTextSize(1);
  display.setCursor(0, 12);

  // E: energizer — live fence pulse voltage (kV), pulse count since last send, energizer battery voltage
  display.print("E ");
  display.print(langleyData.fenceVoltage, 1);
  display.print("kV ");
  display.print(langleyData.pulseCount);
  display.print(" ");
  display.print(langleyData.energizerBatteryVoltage, 2);
  display.println("V");

  // S: solar input bus
  display.print("S ");
  display.print(langleyData.solarVoltage, 2);
  display.print("V ");
  display.print(langleyData.solarCurrentMa, 0);
  display.println("mA");

  // B: system battery bus
  display.print("B ");
  display.print(langleyData.batteryVoltage, 2);
  display.print("V ");
  display.print(langleyData.batteryCurrentMa, 0);
  display.println("mA");

  display.print("rssi:");
  display.print((int)langleyData.rssi);
  display.print(" snr:");
  display.println(langleyData.snr);

  display.display();
}

void showGenericReceivedData(String label, const char* deviceName, float rssi, float snr) {
  display.clearDisplay();
  centerText(titleWithTime(label), 0);
  display.setTextSize(1);
  display.setCursor(0, 12);
  display.println(deviceName);
  display.print("rssi: ");
  display.print((int)rssi);
  display.print(" snr: ");
  display.println(snr);
  display.display();
}

void showUnidentifiedPage() {
  display.clearDisplay();
  centerText(titleWithTime("Unidentified"), 0);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("loraPacketSize: ");
  display.println(lastReceivedPacketSize);
  display.display();
}

// Dispatches to the right OLED page based on the most recently received LoRa packet.
// Called both right after a fresh reception, and when the switch is moved back to
// the "show received data" position so the last known data redraws immediately.
void showLastReceivedData() {
  if (lastReceivedPacketSize == sizeof(ChinampaData)) {
    showChinampaPage1();
  } else if (lastReceivedPacketSize == sizeof(DigitalStablesData)) {
    showDigitalStablesDetail();
  } else if (lastReceivedPacketSize == sizeof(CommaRecord)) {
    showCommaRecordPage();
  } else if (lastReceivedPacketSize == sizeof(GloriaTankFlowPumpData)) {
    showGenericReceivedData("Gloria", gloriaTankFlowPumpData.devicename, gloriaTankFlowPumpData.rssi, gloriaTankFlowPumpData.snr);
  } else if (lastReceivedPacketSize == sizeof(SeedlingMonitorData)) {
    showGenericReceivedData("Seedling", seedlingMonitorData.devicename, seedlingMonitorData.rssi, seedlingMonitorData.snr);
  } else if (lastReceivedPacketSize == sizeof(LangleyData)) {
    showLangleyPage();
  } else if (lastReceivedPacketSize == sizeof(WeatherForecastUpdate)) {
    showWeatherForecast();
  } else if (lastReceivedPacketSize > 0) {
    showUnidentifiedPage();
  } else {
    display.clearDisplay();
    centerText("Please Wait", 0);
    display.display();
  }
}

// Not called from loop() — current hardware doesn't populate PI_CONTROL, but the
// button is part of the design, so the handling logic stays in place for when it is.
void checkPiControlButton() {
  piControlCurrentState = digitalRead(PI_CONTROL);
  if (piControlLastState == HIGH && piControlCurrentState == LOW) {  // button is pressed
    piControlPressedTime = millis();
  } else if (piControlLastState == LOW && piControlCurrentState == HIGH) {  // button is released
    piControlReleasedTime = millis();
    long pressDuration = piControlReleasedTime - piControlPressedTime;
    if (pressDuration < PI_CONTROL_SHORT_PRESS_TIME && piCurrentState == PI_STATE_ASYNC) {
      userCommandState = COMMAND_REBOOT;
      display.clearDisplay();
      centerText("Please Wait", 0);
      display.display();
    }
    if (pressDuration >= PI_CONTROL_LONG_PRESS_TIME && piCurrentState == PI_STATE_ASYNC) {
      userCommandState = COMMAND_SHUTDOWN;
      display.clearDisplay();
      centerText("Pi Shutdown", 0);
      display.display();
    }
  }
  piControlLastState = piControlCurrentState;
}

const char* resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "POWERON";    // power loss OR EN pin pulled low - the ESP32 can't tell them apart
    case ESP_RST_EXT: return "EXT_PIN";        // not reported by the original ESP32 (EN resets show as POWERON)
    case ESP_RST_SW: return "SW";              // ESP.restart()
    case ESP_RST_PANIC: return "PANIC";        // crash / Guru Meditation
    case ESP_RST_INT_WDT: return "INT_WDT";    // interrupt watchdog (ISR too long / interrupts blocked)
    case ESP_RST_TASK_WDT: return "TASK_WDT";  // task watchdog
    case ESP_RST_WDT: return "WDT";            // other watchdog
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";  // supply voltage dipped
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

// Called once in setup(), after LittleFS and the RTC are up. Appends one line to
// RESET_LOG_FILE (keeping the last RESET_LOG_MAX_LINES): RTC time, reason, boot count,
// and the uptime reached before this reset (or "?" when RTC memory didn't survive).
void recordResetReason(RTCInfoRecord& now) {
  resetReason = resetReasonToString(esp_reset_reason());

  if (resetDiagMagic == RESET_DIAG_MAGIC) {
    previousUptimeSeconds = resetDiagUptimeSeconds;
    previousMinFreeHeap = resetDiagMinFreeHeap;
    previousUptimeKnown = true;
    resetDiagBootCount++;
  } else {
    resetDiagMagic = RESET_DIAG_MAGIC;
    resetDiagBootCount = 1;
  }
  resetDiagUptimeSeconds = 0;
  resetDiagMinFreeHeap = ESP.getMinFreeHeap();

  char line[128];
  snprintf(line, sizeof(line), "%04d-%02d-%02d %02d:%02d:%02d reason=%s boot=%lu prevUptime=%s prevMinHeap=%s",
           now.year, now.month, now.date, now.hour, now.minute, now.second,
           resetReason.c_str(), (unsigned long)resetDiagBootCount,
           previousUptimeKnown ? String(previousUptimeSeconds).c_str() : "?",
           previousUptimeKnown ? String(previousMinFreeHeap).c_str() : "?");

  // Keep only the newest lines so the log can't grow without bound
  String kept = "";
  int lineCount = 0;
  File in = LittleFS.open(RESET_LOG_FILE, "r");
  if (in) {
    String all = in.readString();
    in.close();
    int start = all.length();
    while (start > 0 && lineCount < RESET_LOG_MAX_LINES - 1) {
      int prev = all.lastIndexOf('\n', start - 2);
      start = prev + 1;
      lineCount++;
      if (prev < 0) break;
    }
    kept = all.substring(start);
  }
  File out = LittleFS.open(RESET_LOG_FILE, "w");
  if (out) {
    out.print(kept);
    out.println(line);
    out.close();
  }
  if (debug) Serial.println(line);
  recordCoreDump(now);
}

// Called from recordResetReason(). Never lets '|' or '#' through - they delimit the AsyncData line.
// The dump itself is left in flash (the next panic overwrites it) so the full dump can still be read
// out and opened with esp-coredump; LAST_CRASH_FILE holds the summary line plus a fingerprint of the
// dump, so the same dump is reported once, not again on every boot.
void recordCoreDump(RTCInfoRecord& now) {
  String knownFingerprint = "";
  File f = LittleFS.open(LAST_CRASH_FILE, "r");
  if (f) {
    lastCrash = f.readStringUntil('\n');
    knownFingerprint = f.readStringUntil('\n');
    f.close();
    lastCrash.trim();
    knownFingerprint.trim();
  }
  if (esp_core_dump_image_check() != ESP_OK) return;  // no (valid) dump in flash

  esp_core_dump_summary_t* summary = (esp_core_dump_summary_t*)malloc(sizeof(esp_core_dump_summary_t));
  if (summary == nullptr) return;
  if (esp_core_dump_get_summary(summary) != ESP_OK) {
    free(summary);
    return;
  }
  // Fingerprint: dump size + crash pc + task + backtrace - identical on every boot for the same
  // dump, different for a new crash.
  size_t dumpAddr = 0, dumpSize = 0;
  esp_core_dump_image_get(&dumpAddr, &dumpSize);
  uint32_t fp = (uint32_t)dumpSize ^ summary->exc_pc ^ summary->exc_tcb;
  for (uint32_t i = 0; i < summary->exc_bt_info.depth && i < 16; i++) fp = fp * 31 + summary->exc_bt_info.bt[i];
  char fingerprint[12];
  snprintf(fingerprint, sizeof(fingerprint), "%08lx", (unsigned long)fp);
  if (knownFingerprint == fingerprint) {  // already reported
    free(summary);
    return;
  }

  char reason[200] = "";
  if (esp_core_dump_get_panic_reason(reason, sizeof(reason)) != ESP_OK) strcpy(reason, "?");
  char buf[64];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d;", now.year, now.month, now.date, now.hour, now.minute, now.second);
  String line = String(buf) + reason + ";" + String(summary->exc_task) + ";";
  snprintf(buf, sizeof(buf), "0x%08lx;", (unsigned long)summary->exc_pc);
  line += buf;
  for (uint32_t i = 0; i < summary->exc_bt_info.depth && i < 16; i++) {
    snprintf(buf, sizeof(buf), "%s0x%08lx", i > 0 ? " " : "", (unsigned long)summary->exc_bt_info.bt[i]);
    line += buf;
  }
  if (summary->exc_bt_info.corrupted) line += " (corrupted)";
  line += ";";
  for (int i = 0; i < 8 && summary->app_elf_sha256[i]; i++) line += (char)summary->app_elf_sha256[i];
  // Registers: exception cause and address, a0 = return address of the crashing function (windowed
  // ABI: top 2 bits are the call size, so restore the 0x4 code-segment prefix), and every EPC
  // register the dump holds - when the panic started in an interrupt, an EPC holds the address of
  // the code it interrupted.
  const esp_core_dump_summary_extra_info_t& ex = summary->ex_info;
  snprintf(buf, sizeof(buf), ";cause=%lu vaddr=0x%08lx a0=0x%08lx", (unsigned long)ex.exc_cause,
           (unsigned long)ex.exc_vaddr, (unsigned long)((ex.exc_a[0] & 0x3FFFFFFFUL) | 0x40000000UL));
  line += buf;
  for (int i = 0; i < EPCx_REGISTER_COUNT; i++) {
    if (!(ex.epcx_reg_bits & (1 << i))) continue;
    snprintf(buf, sizeof(buf), " epc%d=0x%08lx", i + 1, (unsigned long)ex.epcx[i]);
    line += buf;
  }
  free(summary);

  line.replace("|", "/");
  line.replace("#", "/");
  line.replace("\n", " ");
  line.replace("\r", " ");
  lastCrash = line;

  File out = LittleFS.open(LAST_CRASH_FILE, "w");
  if (out) {
    out.println(lastCrash);
    out.println(fingerprint);
    out.close();
  }
  File log = LittleFS.open(RESET_LOG_FILE, "a");
  if (log) {
    log.print("crash ");
    log.println(lastCrash);
    log.close();
  }
  if (debug) Serial.println("Core dump: " + lastCrash);
}

void printResetInfo() {
  Serial.print("ResetReason=");
  Serial.println(resetReason);
  Serial.print("BootCount=");
  Serial.println(resetDiagBootCount);
  Serial.print("PrevUptimeSeconds=");
  if (previousUptimeKnown) Serial.println(previousUptimeSeconds);
  else Serial.println("?");
  Serial.print("UptimeSeconds=");
  Serial.println(millis() / 1000);
  Serial.print("PrevMinFreeHeap=");
  if (previousUptimeKnown) Serial.println(previousMinFreeHeap);
  else Serial.println("?");
  Serial.print("FreeHeap=");
  Serial.println(ESP.getFreeHeap());
  Serial.print("MinFreeHeap=");
  Serial.println(ESP.getMinFreeHeap());
  Serial.print("MaxAllocHeap=");
  Serial.println(ESP.getMaxAllocHeap());
  Serial.print("LastCrash=");
  Serial.println(lastCrash.length() ? lastCrash : "none");
  File in = LittleFS.open(RESET_LOG_FILE, "r");
  if (in) {
    while (in.available()) Serial.write(in.read());
    in.close();
  }
}

void setStationMode(String ipAddress) {
  if (debug) Serial.println("setting Station mode, address " + ipAddress);
  leds[0] = CRGB(0, 0, 255);
  showLeds();
  display.clearDisplay();
  centerText("Station Mode", 0);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(ipAddress);
  display.setCursor(0, 40);
  display.println("Rst: " + resetReason);
  display.display();
  delay(2000);
}

void setApMode() {
  leds[0] = CRGB(0, 0, 255);
  showLeds();
  if (debug) Serial.println("setting AP mode");
  String apAddress = wifiManager.getApAddress();
  if (debug) Serial.println("AP address " + apAddress);
  display.clearDisplay();
  centerText("AP Mode", 0);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(apAddress);
  display.setCursor(0, 40);
  display.println("Rst: " + resetReason);
  display.display();
  delay(1000);
  leds[0] = CRGB(0, 255, 0);
  leds[1] = loraActive ? CRGB(0, 0, 255) : CRGB(255, 0, 0);
  showLeds();
}

// Writes the PCF8563 time registers (0x02-0x08, BCD) directly. PCF8563TimeManager::setTime(String)
// prints the date to Serial without a newline - that is the Pi's command line here - and
// setTime(RTCInfoRecord) has no return statement, so neither is used.
void writeRtc(const struct tm &t) {
  auto bcd = [](int v) -> uint8_t { return (uint8_t)(((v / 10) << 4) | (v % 10)); };
  Wire.beginTransmission(0x51);  // PCF8563
  Wire.write(0x02);
  Wire.write(bcd(t.tm_sec));
  Wire.write(bcd(t.tm_min));
  Wire.write(bcd(t.tm_hour));
  Wire.write(bcd(t.tm_mday));
  Wire.write(bcd(t.tm_wday));
  Wire.write(bcd(t.tm_mon + 1));
  Wire.write(bcd(t.tm_year - 100));
  Wire.endTransmission();
}

// See RTC_SYNC_TOLERANCE_SEC. Does nothing until SNTP has synced (getLocalTime() is false before).
void syncRtcFromNtp() {
  // WifiManager::connectSTA() starts a task that calls configTime(36000, 3600, ...) on every
  // connect, which replaces TZ with a fixed offset (no Australian DST rule) - possibly after the
  // configTzTime() in setup(). Re-assert the real rule before reading local time.
  setenv("TZ", timezone.c_str(), 1);
  tzset();
  struct tm ntp;
  if (!getLocalTime(&ntp, 0)) return;  // 0 ms: never wait in loop()
  rtcCheckedSinceBoot = true;
  time_t ntpEpoch = time(nullptr);

  struct tm rtc = {};
  rtc.tm_year = currentTimerRecord.year - 1900;
  rtc.tm_mon = currentTimerRecord.month - 1;
  rtc.tm_mday = currentTimerRecord.date;
  rtc.tm_hour = currentTimerRecord.hour;
  rtc.tm_min = currentTimerRecord.minute;
  rtc.tm_sec = currentTimerRecord.second;
  rtc.tm_isdst = -1;  // let the TZ rule decide
  long diff = (long)(ntpEpoch - mktime(&rtc));
  if (labs(diff) <= RTC_SYNC_TOLERANCE_SEC) return;

  char from[24];
  snprintf(from, sizeof(from), "%04d-%02d-%02d %02d:%02d:%02d", currentTimerRecord.year, currentTimerRecord.month,
           currentTimerRecord.date, currentTimerRecord.hour, currentTimerRecord.minute, currentTimerRecord.second);
  writeRtc(ntp);
  currentTimerRecord = timeManager.now();
  char to[24];
  snprintf(to, sizeof(to), "%04d-%02d-%02d %02d:%02d:%02d", currentTimerRecord.year, currentTimerRecord.month,
           currentTimerRecord.date, currentTimerRecord.hour, currentTimerRecord.minute, currentTimerRecord.second);
  String line = String("clock ") + from + " -> " + to + " (" + diff + " s, NTP" + (ntp.tm_isdst > 0 ? " AEDT)" : " AEST)");
  if (debug) Serial.println(line);
  File log = LittleFS.open(RESET_LOG_FILE, "a");
  if (log) {
    log.println(line);
    log.close();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(SLEEP_SWITCH_26, OUTPUT);
  digitalWrite(SLEEP_SWITCH_26, HIGH);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    if (debug) Serial.println(F("SSD1309 allocation failed"));
    for (;;)
      ;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Please Wait"));
  display.display();

  pinMode(SWITCH_PIN_LEFT, INPUT_PULLUP);
  pinMode(SWITCH_PIN_RIGHT, INPUT_PULLUP);

  if (!LittleFS.begin(true)) {
    if (debug) Serial.println("LittleFS Mount Failed, formatting...");
    LittleFS.format();
    if (!LittleFS.begin(false)) {
      if (debug) Serial.println("LittleFS Mount Failed even after formatting");
      return;
    } else {
      if (debug) Serial.println("LittleFS Mount Succces after formating");
    }
  } else {
    if (debug) Serial.println("LittleFS Mount Succces");
  }

  if (debug) Serial.println("DigitalStablesData size=" + String(sizeof(DigitalStablesData)));
  if (debug) Serial.println("ChinampaData size=" + String(sizeof(ChinampaData)));
  if (debug) Serial.println("SeedlingMonitorData size=" + String(sizeof(SeedlingMonitorData)));
  if (debug) Serial.println("CommaRecord size=" + String(sizeof(CommaRecord)));

  dataManager.start();
  double latitude = -37.13305556;
  double longitude = 144.47472222;
  double altitude = 410.0;
  secret = "J5KFCNCPIRCTGT2UJUZFSMQK";
  String apiKey = "103df7bb3e4010e033d494f031b483e0";

  timezone = "AEST-10AEDT,M10.1.0,M4.1.0/3";
  setenv("TZ", timezone.c_str(), 1);
  tzset();

  pinMode(RTC_CLK_OUT, INPUT_PULLUP);
  tempSensor.begin();
  uint8_t address[8];
  if (tempSensor.getAddress(address, 0)) {
    for (uint8_t i = 0; i < 8; i++) {
      serialNumber += String(address[i], HEX);
    }
    memcpy(annabelleData.serialnumberarray, address, sizeof(address));
  } else {
    if (debug) Serial.println(F("Error fetching the temp sensor address"));
  }

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  ledMutex = xSemaphoreCreateMutex();
  FastLED.setBrightness(40);
  timeManager.start();
  timeManager.PCF8563osc1Hz();
  digitalWrite(RTC_CLK_OUT, HIGH);
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);
  currentTimerRecord = timeManager.now();
  recordResetReason(currentTimerRecord);

  weatherForecastManager = new WeatherForecastManager(Serial, latitude, longitude, apiKey.c_str());
  weatherForecastManager->initialize(currentTimerRecord);
  weatherForecastManager->loadForecasts(Serial);

  pinMode(RTC_BATT_VOLT, INPUT);
  pinMode(OP_MODE, INPUT_PULLUP);

  //
  // lora code
  //
  SPI.begin(SCK, MISO, MOSI);
  pinMode(LoRa_SS, OUTPUT);
  pinMode(LORA_RESET, OUTPUT);
  pinMode(LORA_DI0, INPUT);
  digitalWrite(LoRa_SS, HIGH);
  delay(100);
  LoRa.setPins(LoRa_SS, LORA_RESET, LORA_DI0);

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 255, 0);
  }
  showLeds();

  display.clearDisplay();
  centerText("LoRa", 0);
  display.display();

  if (!LoRa.begin(433E6)) {
    while (1)
      ;
    leds[1] = CRGB(255, 0, 0);
    display.clearDisplay();
    centerText("LORA Failed", 0);
    display.display();
  } else {
    loraActive = true;
    LoRa.enableCrc();
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    display.clearDisplay();
    centerText("LORA Active", 0);
    display.display();
  }

  String grp = secretManager.getGroupIdentifier();
  char gprid[grp.length()];
  grp.toCharArray(gprid, grp.length());
  strcpy(annabelleData.groupidentifier, gprid);

  String identifier = "AnnabelleV";
  char ty[identifier.length() + 1];
  identifier.toCharArray(ty, identifier.length() + 1);
  strcpy(annabelleData.deviceTypeId, ty);

  panchoConfigData.fieldId = secretManager.getFieldId();

  delay(1000);

  wifiManager.start();
  bool stationmode = wifiManager.getStationMode();
  wifiManager.setSerialNumber(serialNumber);
  wifiManager.setLora(loraActive);
  String ssid = wifiManager.getSSID();

  if (stationmode) {
    ipAddress = wifiManager.getIpAddress();
    if (ipAddress == "" || ipAddress == "0.0.0.0") {
      setApMode();
    } else {
      setStationMode(ipAddress);
    }
  } else {
    setApMode();
  }

  configTzTime(timezone.c_str(), "pool.ntp.org", "time.google.com");  // SNTP; works once WiFi has internet

  internetAvailable = wifiManager.getInternetAvailable();
  if (debug) Serial.print(F("internet avail="));
  if (debug) Serial.println(internetAvailable);

  if (loraActive) {
    // Not LoRa.onReceive() - see loraDio0Fired.
    attachInterrupt(digitalPinToInterrupt(LORA_DI0), onLoraDio0, RISING);
    LoRa.receive();
  }

  pinMode(OP_MODE, INPUT_PULLUP);

  pinMode(PI_CONTROL, INPUT);
  piCurrentState = PI_STATE_UNDEFINED;

  xTaskCreatePinnedToCore(
    ledShowTaskFunction,
    "ledShowTask",
    5000,
    NULL,
    1,
    &ledShowTask,
    0);

  updateStatusLeds();  // show initial status at boot
}

void loop() {
  int hour, minute, second;

  if (clockTicked) {
    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);

    currentTimerRecord = timeManager.now();
    timeIsSet = true;
    wifiManager.setCurrentTimerRecord(currentTimerRecord);
    secondsSinceLastAsyncData++;
    resetDiagUptimeSeconds = millis() / 1000;  // RTC memory, read back after the next reset
    resetDiagMinFreeHeap = ESP.getMinFreeHeap();
    // Keeps DataManager's flash-wear rate estimate current without threading
    // a time parameter through the whole enqueue/overflow call chain.
    dataManager.setCurrentEpoch(TimeUtils::getEpochTime(currentTimerRecord.year, currentTimerRecord.month, currentTimerRecord.date, currentTimerRecord.hour, currentTimerRecord.minute, currentTimerRecord.second));

    hour = currentTimerRecord.hour;
    minute = currentTimerRecord.minute;
    second = currentTimerRecord.second;

    if (second == 30 && (!rtcCheckedSinceBoot || minute == 7)) {
      syncRtcFromNtp();
    }
    if (weatherScreenShown && !switchPositionLeft) {  // live clock on the weather page
      display.fillRect(0, 0, SCREEN_WIDTH, 8, SSD1306_BLACK);
      drawWeatherTitle();
      display.display();
    }

    // Set pending flag at trigger time; actual work deferred until serial is free
    if (minute % 5 == 0 && second == 0) {
      weatherPending = true;
    }

    if (weatherPending && Serial.available() == 0 && !inSerial) {
      weatherPending = false;
      if (debug) Serial.println("About to get and send weatherforecasts");
      bool wifiAvail = wifiManager.getInternetAvailable();
      if (!wifiAvail) {
        // getInternetAvailable() only reflects the last ping; without an active
        // recheck here, one failed ping (e.g. at boot, before DHCP/DNS settled)
        // latches false forever and this 5-minute trigger never downloads again.
        // Mirrors Daffodil.ino's recheck-on-reconnect handling.
        wifiManager.checkInternetConnectionAvailable();
        wifiAvail = wifiManager.getInternetAvailable();
      }
      if (wifiAvail) {
        weatherDownloading = true;
        bool success = weatherForecastManager->downloadWeatherData();
        weatherDownloading = false;
        updateStatusLeds();
        if (debug) Serial.println(success ? "Weather forecast downloaded" : "Weather forecast download failed");
      } else {
        if (debug) Serial.println("No internet, skipping weather forecast download");
      }
      WeatherForecast* forecasts = weatherForecastManager->getForecasts();
      if (forecasts != nullptr) {
        WeatherForecastUpdate weatherForecastUpdate;
        for (int i = 0; i < 4; i++) {
          weatherForecastUpdate.forecasts[i] = forecasts[i];
        }
        weatherForecastUpdate.totpcode = secretManager.generateCode();
        if (debug) Serial.print("sending weather via LoRa, size=");
        if (debug) Serial.println(sizeof(WeatherForecastUpdate));
        sendMessage(weatherForecastUpdate);  // ends in RX mode; loop polls for packets
        if (!switchPositionLeft) showWeatherForecast();
      } else {
        if (debug) Serial.println("No forecast data available to send");
      }
    }

    int left = digitalRead(SWITCH_PIN_LEFT);
    int right = digitalRead(SWITCH_PIN_RIGHT);
    bool previousSwitchPositionLeft = switchPositionLeft;
    if (left == LOW && right == HIGH) {
      switchPositionLeft = true;
    } else if (left == HIGH && right == LOW) {
      switchPositionLeft = false;
    }
    if (switchPositionLeft != previousSwitchPositionLeft) {
      if (switchPositionLeft) {
        showLastReceivedData();
      } else {
        showWeatherForecast();
      }
    }

    updateStatusLeds();  // refresh status LEDs every second
  }

  if (loraDio0Fired) {
    loraDio0Fired = false;
    // Reads and clears the IRQ flags and points the FIFO at the packet; leaves the radio in standby
    // after a packet, or switches it to single RX when there was none (CRC error, TX done).
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
      loraPacketSize = packetSize;
      loraReceived = true;
    } else {
      LoRa_rxMode();  // back to continuous receive
    }
  }

  if (loraReceived) {
    if (debug) Serial.printf("lora recive loraPacketSize: %d \n", loraPacketSize);
    if (debug) Serial.println("");
    processLora(loraPacketSize);
    LoRa_rxMode();  // parsePacket() left the radio in standby
    loraReceived = false;
    // A VitalSigns (and sometimes a DeviceIdentity) packet follows a data pulse - keep showing
    // that device's data page instead of switching to "Unidentified".
    if (loraPacketSize != sizeof(VitalSignsRecord) && loraPacketSize != sizeof(DeviceIdentityRecord)) lastReceivedPacketSize = loraPacketSize;
    loraActivitySeconds = 3;

    currentPalette = loraReceivePalette;
    currentBlending = LINEARBLEND;
    ledShowDuration = 2000;
    runLedShow = true;

    if (switchPositionLeft) {
      showLastReceivedData();
    }
  }
  loraPacketSize = 0;

  if (Serial.available() != 0) {
    inSerial = true;
    String command = Serial.readString();

    if (command.startsWith("getWF")) {
      if (debug) Serial.println("About to get and send weatherforecasts");
      bool wifiAvail = wifiManager.getInternetAvailable();
      if (!wifiAvail) {
        for (int i = 0; i < 3 && !wifiAvail; i++) {
          wifiManager.checkInternetConnectionAvailable();
          wifiAvail = wifiManager.getInternetAvailable();
        }
      }
      if (wifiAvail) {
        bool success = weatherForecastManager->downloadWeatherData();
        if (debug) Serial.println(success ? "Weather forecast downloaded" : "Weather forecast download failed");
      } else {
        if (debug) Serial.println("No internet after retries, skipping weather forecast download");
      }
      WeatherForecast* forecasts = weatherForecastManager->getForecasts();
      if (forecasts != nullptr) {
        WeatherForecastUpdate weatherForecastUpdate;
        for (int i = 0; i < 4; i++) {
          weatherForecastUpdate.forecasts[i] = forecasts[i];
        }
        weatherForecastUpdate.totpcode = secretManager.generateCode();
        sendMessage(weatherForecastUpdate);
        if (!switchPositionLeft) showWeatherForecast();
        if (debug) Serial.println("Weather send complete");
      } else {
        if (debug) Serial.println("No forecast data available to send");
      }
    } else if (command == USER_COMMAND) {
      Serial.println(F("Ok-"));
    } else if (command == LIFE_CYCLE_EVENT_START_AWAKE) {
      Serial.print(F("Ok-"));
      Serial.println(LIFE_CYCLE_EVENT_START_AWAKE);
      currentPalette = RainbowStripeColors_p;
      currentBlending = NOBLEND;
    } else if (command == LIFE_CYCLE_EVENT_END_AWAKE) {
      Serial.print(F("Ok-"));
      Serial.println(LIFE_CYCLE_EVENT_END_AWAKE);
    } else if (command == LIFE_CYCLE_EVENT_START_SYNCHRONOUS_CYCLE) {
      Serial.print(F("Ok-"));
      Serial.println(LIFE_CYCLE_EVENT_START_SYNCHRONOUS_CYCLE);
      piCurrentState = PI_STATE_SYNC;
      userCommandState = "Ok";
    } else if (command == LIFE_CYCLE_EVENT_END_SYNCHRONOUS_CYCLE) {
      Serial.print(F("Ok-"));
      Serial.println(LIFE_CYCLE_EVENT_END_SYNCHRONOUS_CYCLE);
    } else if (command == LIFE_CYCLE_EVENT_START_ASYNCHRONOUS_CYCLE) {
      Serial.print(F("Ok-"));
      Serial.println(LIFE_CYCLE_EVENT_START_ASYNCHRONOUS_CYCLE);
      piCurrentState = PI_STATE_ASYNC;
    } else if (command == LIFE_CYCLE_EVENT_END_ASYNCHRONOUS_CYCLE) {
      Serial.print(F("Ok-"));
      Serial.println(LIFE_CYCLE_EVENT_END_ASYNCHRONOUS_CYCLE);
    } else if (command.startsWith("Ping")) {
      Serial.println(F("Ok-Ping"));
      Serial.flush();
    } else if (command.startsWith("debug")) {
      int debugv = generalFunctions.getValue(command, '#', 1).toInt();
      debug = debugv > 0;
      Serial.println("Ok-debug");
      Serial.flush();
    } else if (command.startsWith("printCurrentChinampaData")) {
      dataManager.printChinampaData(chinampaData);
      Serial.println("Ok-printCurrentDSDData");
      Serial.flush();
    } else if (command.startsWith("ScanNetworks")) {
      wifiManager.scanNetworks();
    } else if (command.startsWith("SetGroupId")) {
      String grpId = generalFunctions.getValue(command, '#', 1);
      secretManager.setGroupIdentifier(grpId);
    } else if (command.startsWith("SetDeviceSensorConfig")) {
      // SetDeviceSensorConfig#Sceptic#SCEP#NoSensor#Temperature#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#40#50#
      String devicename = generalFunctions.getValue(command, '#', 1);
      String deviceshortname = generalFunctions.getValue(command, '#', 2);
      String sensor1name = generalFunctions.getValue(command, '#', 3);
      String sensor2name = generalFunctions.getValue(command, '#', 4);
      String tz = generalFunctions.getValue(command, '#', 5);
      double latitude = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 6));
      double longitude = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 7));
      double altitude = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 8));
      digitalStablesData.minimumEfficiencyForLed = generalFunctions.getValue(command, '#', 9).toInt();
      digitalStablesData.minimumEfficiencyForWifi = generalFunctions.getValue(command, '#', 10).toInt();

      devicename.toCharArray(digitalStablesData.devicename, devicename.length() + 1);
      deviceshortname.toCharArray(digitalStablesData.deviceshortname, deviceshortname.length() + 1);
      sensor1name.toCharArray(digitalStablesData.sensor1name, sensor1name.length() + 1);
      sensor2name.toCharArray(digitalStablesData.sensor2name, sensor2name.length() + 1);

      secretManager.saveDeviceSensorConfig(devicename, deviceshortname, sensor1name, sensor2name, tz, latitude, longitude, altitude, digitalStablesData.minimumEfficiencyForLed, digitalStablesData.minimumEfficiencyForWifi);
      Serial.println(F("Ok-SetDeviceSensorConfig"));
    } else if (command.startsWith("GetGroupId")) {
      Serial.println("Ok-GetGroupId");
    } else if (command.startsWith("GetWifiStatus")) {
      uint8_t status = wifiManager.getWifiStatus();
      Serial.println("Ok-GetWifiStatus");
    } else if (command.startsWith("GetRememberedValueData")) {
      Serial.println("Ok-GetRememberedValueData");
    } else if (command.startsWith("ConfigWifiSTA")) {
      //ConfigWifiSTA#ssid#password#hostname
      String ssid = generalFunctions.getValue(command, '#', 1);
      String password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      bool staok = wifiManager.configWifiSTA(ssid, password, hostname);
      leds[0] = staok ? CRGB(0, 0, 255) : CRGB(255, 0, 0);
      showLeds();
      Serial.println("Ok-ConfigWifiSTA");
    } else if (command.startsWith("ConfigWifiAP")) {
      //ConfigWifiAP#soft_ap_ssid#soft_ap_password#hostname
      String soft_ap_ssid = generalFunctions.getValue(command, '#', 1);
      String soft_ap_password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      bool stat = wifiManager.configWifiAP(soft_ap_ssid, soft_ap_password, hostname);
      leds[0] = stat ? CRGB(0, 255, 0) : CRGB(255, 0, 0);
      showLeds();
      Serial.println("Ok-ConfigWifiAP");
    } else if (command.startsWith("GetOperationMode")) {
      uint8_t switchState = digitalRead(OP_MODE);
      Serial.println(switchState == LOW ? F("PGM") : F("RUN"));
    } else if (command.startsWith("SetTimeFromInternet")) {
      wifiManager.setTimeFromInternet();
    } else if (command.startsWith("SetTime")) {
      //SetTime#8#5#24#4#18#22#25
      timeManager.setTime(command);
      Serial.println("Ok-SetTime");
    } else if (command.startsWith("SetFieldId")) {
      // fieldId= GeneralFunctions::getValue(command, '#', 1).toInt();
    } else if (command.startsWith("GetTime")) {
      timeManager.printTimeToSerial(currentTimerRecord);
      Serial.flush();
    } else if (command.startsWith("GetCommandCode")) {
      long code = secretManager.generateCode();
      //
      // patch a bug in the totp library
      // if the first digit is a zero, it
      // returns a 5 digit number
      if (code < 100000) {
        Serial.print("0");
        Serial.println(code);
      } else {
        Serial.println(code);
      }
      Serial.flush();
    } else if (command.startsWith("GetSerialNumber")) {
      Serial.println(serialNumber);
      Serial.println("Ok-GetSerialNumber");
      Serial.flush();
    } else if (command.startsWith("VerifyUserCode")) {
      String codeInString = generalFunctions.getValue(command, '#', 1);
      long userCode = codeInString.toInt();
      boolean validCode = true;  //secretManager.checkCode( userCode);
      Serial.println(validCode ? "Ok-Valid Code" : "Failure-Invalid Code");
      Serial.flush();
    } else if (command.startsWith("GetSecret")) {
      String secretCode = secretManager.readSecret();
      Serial.println(secretCode);
      Serial.println("Ok-GetSecret");
      Serial.flush();
    } else if (command.startsWith("SetSecret")) {
      //SetSecret#IZQWS3TDNB2GK2LO#6#30
      String secret = generalFunctions.getValue(command, '#', 1);
      int numberDigits = generalFunctions.getValue(command, '#', 2).toInt();
      int periodSeconds = generalFunctions.getValue(command, '#', 3).toInt();
      secretManager.saveSecret(secret, numberDigits, periodSeconds);
      Serial.println("Ok-SetSecret");
      Serial.flush();
    } else if (command == "Flush") {
      while (Serial.read() >= 0)
        ;
      Serial.println("Ok-Flush");
      Serial.flush();
    } else if (command.startsWith("PulseStart")) {
      inPulse = true;
      Serial.println("Ok-PulseStart");
      Serial.flush();
    } else if (command.startsWith("PulseFinished")) {
      inPulse = false;
      Serial.println("Ok-PulseFinished");
      Serial.flush();
    } else if (command.startsWith("IPAddr")) {
      currentIpAddress = generalFunctions.getValue(command, '#', 1);
      Serial.println("Ok-IPAddr");
      Serial.flush();
    } else if (command.startsWith("SSID")) {
      String currentSSID = generalFunctions.getValue(command, '#', 1);
      wifiManager.setCurrentSSID(currentSSID.c_str());
      Serial.println("Ok-currentSSID");
      Serial.flush();
    } else if (command.startsWith("GetIpAddress")) {
      Serial.println(wifiManager.getIpAddress());
      Serial.println("Ok-GetIpAddress");
      Serial.flush();
    } else if (command.startsWith("ResetInfo")) {
      printResetInfo();
      Serial.println("Ok-ResetInfo");
      Serial.flush();
    } else if (command.startsWith("RestartWifi")) {
      wifiManager.restartWifi();
      Serial.println("Ok-restartWifi");
      Serial.flush();
    } else if (command.startsWith("GetSensorData")) {
      DigitalStablesDataSerializer digitalStablesDataSerializer;
      digitalStablesDataSerializer.pushToSerial(Serial, digitalStablesData);
      Serial.flush();
    } else if (command.startsWith("AsyncDataCount")) {
      // Must be checked before "AsyncData" below (startsWith would match both).
      // Non-destructive peek so the Pi knows how many lines to read before it
      // actually asks for the data with AsyncData.
      Serial.println(dataManager.getPendingQueueItemCount() + vitalSignsStore.pendingCount() + deviceIdentityStore.pendingCount());
      //
      // Per-type available/dropped/flash-health breakdown, added 2026-08-04
      // for capacity planning as device counts grow (many Daffodils, several
      // Langleys, etc). "available" is RAM-queue + flash-overflow combined;
      // "dropped" is cumulative-since-boot records permanently lost (both
      // tiers full); "flashHealthDays" is -1 when not applicable/not enough
      // data yet.
      //
      Serial.print("QueueStatus#");
      Serial.print("DS=");
      Serial.print(dataManager.getDSDQueueCount() + dataManager.getDSDOverflowCount());
      Serial.print(",");
      Serial.print(dataManager.getDSDDroppedCount());
      Serial.print(",");
      Serial.print(dataManager.getDSDFlashHealthDaysRemaining());
      Serial.print("|Gloria=");
      Serial.print(dataManager.getGloriaQueueCount() + dataManager.getGloriaOverflowCount());
      Serial.print(","); Serial.print(dataManager.getGloriaDroppedCount());
      Serial.print(","); Serial.print(dataManager.getGloriaFlashHealthDaysRemaining());
      Serial.print("|Seedling=");
      Serial.print(dataManager.getSeedlingQueueCount() + dataManager.getSeedlingOverflowCount());
      Serial.print(","); Serial.print(dataManager.getSeedlingDroppedCount());
      Serial.print(","); Serial.print(dataManager.getSeedlingFlashHealthDaysRemaining());
      Serial.print("|Chinampa=");
      Serial.print(dataManager.getChinampaQueueCount() + dataManager.getChinampaOverflowCount());
      Serial.print(","); Serial.print(dataManager.getChinampaDroppedCount());
      Serial.print(","); Serial.print(dataManager.getChinampaFlashHealthDaysRemaining());
      Serial.print("|Comma=");
      Serial.print(dataManager.getCommaQueueCount() + dataManager.getCommaOverflowCount());
      Serial.print(","); Serial.print(dataManager.getCommaDroppedCount());
      Serial.print(","); Serial.print(dataManager.getCommaFlashHealthDaysRemaining());
      Serial.print("|Langley=");
      Serial.print(dataManager.getLangleyQueueCount() + dataManager.getLangleyOverflowCount());
      Serial.print(","); Serial.print(dataManager.getLangleyDroppedCount());
      Serial.print(","); Serial.print(dataManager.getLangleyFlashHealthDaysRemaining());
      // Vital signs: RAM-only, latest per device, never dropped/flash - hence 0 and -1
      Serial.print("|Vital="); Serial.print(vitalSignsStore.pendingCount());
      Serial.print(",0,-1");
      Serial.print("|Identity="); Serial.print(deviceIdentityStore.pendingCount());
      Serial.println(",0,-1");
      Serial.println("Ok-AsyncDataCount");
      Serial.flush();
    } else if (command.startsWith("AsyncData")) {
      secondsSinceLastAsyncData = 0;
      currentPalette = asyncDataPalette;
      currentBlending = LINEARBLEND;
      ledShowDuration = 2000;
      runLedShow = true;
      // Peeked before each process*Queue() drains it, so we can report how
      // many records this call actually relayed -- see conversation
      // 2026-08-04 (the "how many available vs downloaded" diagnostic).
      int gloriaDownloaded = 0, dsDownloaded = 0, chinampaDownloaded = 0;
      int seedlingDownloaded = 0, commaDownloaded = 0, langleyDownloaded = 0;
      if (gloriaTankFlowPumpNewData) {
        gloriaDownloaded = dataManager.getGloriaQueueCount();
        dataManager.processGloriaQueue();
        gloriaTankFlowPumpNewData = false;
      }
      if (digitalStablesDataNewData) {
        dsDownloaded = dataManager.getDSDQueueCount();
        dataManager.processDigitalStablesDataQueue();
        dataManager.clearAllDSDData();
        digitalStablesDataNewData = false;
      }
      if (chinampaDataNewData) {
        chinampaDownloaded = dataManager.getChinampaQueueCount();
        dataManager.processChinampaDataQueue();
        dataManager.clearAllChinampaData();
        chinampaDataNewData = false;
      }
      if (seedlingMonitoringDataNewData) {
        seedlingDownloaded = dataManager.getSeedlingQueueCount();
        dataManager.processSeedlingMonitorDataQueue();
        seedlingMonitoringDataNewData = false;
      }
      if (commaDataNewData) {
        commaDownloaded = dataManager.getCommaQueueCount();
        dataManager.processCommaRecordQueue();
        dataManager.clearAllCommaRecords();
        commaDataNewData = false;
      }
      // Overflow tiers are independent of the *NewData flags above (which
      // only track "did a fresh LoRa packet arrive this cycle") -- overflow
      // can hold backlog even when nothing new came in this cycle, so always
      // check every type's overflow, not just when new data triggered the
      // RAM path for that type.
      int dsOverflowDownloaded = dataManager.getDSDOverflowCount();
      dataManager.processDSDOverflow();
      dsDownloaded += dsOverflowDownloaded;

      int gloriaOverflowDownloaded = dataManager.getGloriaOverflowCount();
      dataManager.processGloriaOverflow();
      gloriaDownloaded += gloriaOverflowDownloaded;

      int seedlingOverflowDownloaded = dataManager.getSeedlingOverflowCount();
      dataManager.processSeedlingOverflow();
      seedlingDownloaded += seedlingOverflowDownloaded;

      int chinampaOverflowDownloaded = dataManager.getChinampaOverflowCount();
      dataManager.processChinampaOverflow();
      chinampaDownloaded += chinampaOverflowDownloaded;

      int commaOverflowDownloaded = dataManager.getCommaOverflowCount();
      dataManager.processCommaOverflow();
      commaDownloaded += commaOverflowDownloaded;
      if (langleyDataNewData) {
        langleyDownloaded = dataManager.getLangleyQueueCount();
        dataManager.processLangleyQueue();
        langleyDataNewData = false;
      }
      int langleyOverflowDownloaded = dataManager.getLangleyOverflowCount();
      dataManager.processLangleyOverflow();
      langleyDownloaded += langleyOverflowDownloaded;
      // Counted in AsyncDataCount's total above, so AnnabelleReader's line bound includes them
      int vitalDownloaded = vitalSignsStore.pushPendingToSerial(Serial);
      int identityDownloaded = deviceIdentityStore.pushPendingToSerial(Serial);

      Serial.print("Ok-AsyncData#");
      Serial.print("DS="); Serial.print(dsDownloaded);
      Serial.print("|Gloria="); Serial.print(gloriaDownloaded);
      Serial.print("|Seedling="); Serial.print(seedlingDownloaded);
      Serial.print("|Chinampa="); Serial.print(chinampaDownloaded);
      Serial.print("|Comma="); Serial.print(commaDownloaded);
      Serial.print("|Langley="); Serial.print(langleyDownloaded);
      Serial.print("|Vital="); Serial.print(vitalDownloaded);
      Serial.print("|Identity="); Serial.print(identityDownloaded);
      //
      // Reset diagnostics ride on the AsyncData terminator so the Hypothalamus
      // records them every Async Cycle (generic to any microcontroller):
      // ResetInfo=<reason>,<bootCount>,<prevUptimeSeconds or -1>,<uptimeSeconds>,
      //           <freeHeap>,<minFreeHeap>,<maxAllocHeap>,<prevMinFreeHeap or -1>,
      //           <RTC local time YYYYMMDDhhmmss>
      // (heap in bytes; minFreeHeap falling day after day = leak/fragmentation)
      //
      Serial.print("|ResetInfo="); Serial.print(resetReason);
      Serial.print(","); Serial.print(resetDiagBootCount);
      Serial.print(","); Serial.print(previousUptimeKnown ? (long)previousUptimeSeconds : -1L);
      Serial.print(","); Serial.print(millis() / 1000);
      Serial.print(","); Serial.print(ESP.getFreeHeap());
      Serial.print(","); Serial.print(ESP.getMinFreeHeap());
      Serial.print(","); Serial.print(ESP.getMaxAllocHeap());
      Serial.print(","); Serial.print(previousUptimeKnown ? (long)previousMinFreeHeap : -1L);
      char rtcNow[16];  // the clock Annabelle runs on - the Hypothalamus compares it with its own
      snprintf(rtcNow, sizeof(rtcNow), "%04d%02d%02d%02d%02d%02d", currentTimerRecord.year, currentTimerRecord.month,
               currentTimerRecord.date, currentTimerRecord.hour, currentTimerRecord.minute, currentTimerRecord.second);
      Serial.print(","); Serial.print(rtcNow);
      // Last panic (from the core dump, see recordCoreDump) - only once there has been one.
      if (lastCrash.length()) {
        Serial.print("|Crash="); Serial.print(lastCrash);
      }
      Serial.println();
      Serial.flush();
    } else if (command.startsWith("GetLifeCycleData")) {
      Serial.println("Ok-GetLifeCycleData");
      Serial.flush();
    } else if (command.startsWith("GetWPSSensorData")) {
      Serial.println("Ok-GetWPSSensorData");
      Serial.flush();
    } else {
      Serial.println("Failure-Command Not Found-" + command);
      Serial.flush();
    }
    LoRa_rxMode();
    inSerial = false;
  }
}

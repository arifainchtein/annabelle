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
int lastReceivedPacketSize = 0;

RTCInfoRecord lastReceptionRTCInfoRecord;

GloriaTankFlowPumpData gloriaTankFlowPumpData;
LangleyData langleyData;
DigitalStablesData digitalStablesData;
ChinampaData chinampaData;
SeedlingMonitorData seedlingMonitorData;
CommaRecord commaRecord;
String timezone;

CRGBPalette16 currentPalette;
TBlendType currentBlending;

// Distinct LED-show palettes so a LoRa reception looks different from a Pi AsyncData pull
CRGBPalette16 loraReceivePalette(CRGB::Red, CRGB::HotPink, CRGB::LightPink, CRGB::White);
CRGBPalette16 asyncDataPalette(CRGB::Yellow, CRGB::Orange, CRGB::Yellow, CRGB::Orange);

bool switchPositionLeft = true;  // left position = show received data, right position = show weather forecast

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
  }

  uint8_t calculatedChecksum = calculateChecksum(tempData);
  return (calculatedChecksum == receivedChecksum);
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
    langleyData.rssi = LoRa.packetRssi();
    langleyData.snr = LoRa.packetSnr();
    dataManager.storeLangleyData(langleyData);
    langleyDataNewData = true;
    if (debug) Serial.print("received langleyData from ");
    if (debug) Serial.println(langleyData.devicename);

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
    dataManager.printDigitalStablesData(digitalStablesData);
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

void onReceive(int packetSize) {
  loraReceived = true;
  loraPacketSize = packetSize;
}

//
// end of interrupt functions
//

// Updates the 3 status LEDs (leds[0..2]) based on current system state.
// leds[3,5] are LoRa/weather activity LEDs; leds[4] is the Pi-AsyncData heartbeat.
void updateStatusLeds() {
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
  FastLED.show();
}

TaskHandle_t ledShowTask = NULL;
bool runLedShow = false;
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
    FastLED.show();
    delay(30);  // pace the animation so it's a smooth wave, not a flicker
    currentMillis = millis();
  }
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
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

void showWeatherForecast() {
  WeatherForecast* forecasts = weatherForecastManager->getForecasts();
  if (forecasts == nullptr) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  char title[22];
  snprintf(title, sizeof(title), "WF %d/%d/%02d %02d:%02d:%02d",
           currentTimerRecord.date,
           currentTimerRecord.month,
           currentTimerRecord.year % 100,
           currentTimerRecord.hour,
           currentTimerRecord.minute,
           currentTimerRecord.second);
  display.setCursor(0, 0);
  display.print(title);

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
}

void showChinampaPage1() {
  display.clearDisplay();
  centerText(chinampaData.devicename, 0);

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
  centerText(chinampaData.devicename, 0);
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
  centerText(digitalStablesData.devicename, 0);
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
  centerText(commaRecord.devicename, 0);
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
  centerText(langleyData.devicename, 0);
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
  centerText(label, 0);
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
  centerText("Unidentified", 0);
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

void setStationMode(String ipAddress) {
  if (debug) Serial.println("setting Station mode, address " + ipAddress);
  leds[0] = CRGB(0, 0, 255);
  FastLED.show();
  display.clearDisplay();
  centerText("Station Mode", 0);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(ipAddress);
  display.display();
  delay(2000);
}

void setApMode() {
  leds[0] = CRGB(0, 0, 255);
  FastLED.show();
  if (debug) Serial.println("setting AP mode");
  String apAddress = wifiManager.getApAddress();
  if (debug) Serial.println("AP address " + apAddress);
  display.clearDisplay();
  centerText("AP Mode", 0);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(apAddress);
  display.display();
  delay(1000);
  leds[0] = CRGB(0, 255, 0);
  leds[1] = loraActive ? CRGB(0, 0, 255) : CRGB(255, 0, 0);
  FastLED.show();
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
  FastLED.setBrightness(40);
  timeManager.start();
  timeManager.PCF8563osc1Hz();
  digitalWrite(RTC_CLK_OUT, HIGH);
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);
  currentTimerRecord = timeManager.now();

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
  FastLED.show();

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

  internetAvailable = wifiManager.getInternetAvailable();
  if (debug) Serial.print(F("internet avail="));
  if (debug) Serial.println(internetAvailable);

  if (loraActive) {
    LoRa.onReceive(onReceive);
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

    hour = currentTimerRecord.hour;
    minute = currentTimerRecord.minute;
    second = currentTimerRecord.second;

    // Set pending flag at trigger time; actual work deferred until serial is free
    if (minute % 5 == 0 && second == 0) {
      weatherPending = true;
    }

    if (weatherPending && Serial.available() == 0 && !inSerial) {
      weatherPending = false;
      if (debug) Serial.println("About to get and send weatherforecasts");
      bool wifiAvail = wifiManager.getInternetAvailable();
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

  if (loraReceived) {
    if (debug) Serial.printf("lora recive loraPacketSize: %d \n", loraPacketSize);
    if (debug) Serial.println("");
    processLora(loraPacketSize);
    loraReceived = false;
    lastReceivedPacketSize = loraPacketSize;
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
      FastLED.show();
      Serial.println("Ok-ConfigWifiSTA");
    } else if (command.startsWith("ConfigWifiAP")) {
      //ConfigWifiAP#soft_ap_ssid#soft_ap_password#hostname
      String soft_ap_ssid = generalFunctions.getValue(command, '#', 1);
      String soft_ap_password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      bool stat = wifiManager.configWifiAP(soft_ap_ssid, soft_ap_password, hostname);
      leds[0] = stat ? CRGB(0, 255, 0) : CRGB(255, 0, 0);
      FastLED.show();
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
    } else if (command.startsWith("RestartWifi")) {
      wifiManager.restartWifi();
      Serial.println("Ok-restartWifi");
      Serial.flush();
    } else if (command.startsWith("GetSensorData")) {
      DigitalStablesDataSerializer digitalStablesDataSerializer;
      digitalStablesDataSerializer.pushToSerial(Serial, digitalStablesData);
      Serial.flush();
    } else if (command.startsWith("AsyncData")) {
      secondsSinceLastAsyncData = 0;
      currentPalette = asyncDataPalette;
      currentBlending = LINEARBLEND;
      ledShowDuration = 2000;
      runLedShow = true;
      if (gloriaTankFlowPumpNewData) {
        dataManager.processGloriaQueue();
        gloriaTankFlowPumpNewData = false;
      }
      if (digitalStablesDataNewData) {
        dataManager.processDigitalStablesDataQueue();
        dataManager.clearAllDSDData();
        digitalStablesDataNewData = false;
      }
      if (chinampaDataNewData) {
        dataManager.processChinampaDataQueue();
        dataManager.clearAllChinampaData();
        chinampaDataNewData = false;
      }
      if (seedlingMonitoringDataNewData) {
        dataManager.processSeedlingMonitorDataQueue();
        seedlingMonitoringDataNewData = false;
      }
      if (commaDataNewData) {
        dataManager.processCommaRecordQueue();
        dataManager.clearAllCommaRecords();
        commaDataNewData = false;
      }
      if (langleyDataNewData) {
        dataManager.processLangleyQueue();
        langleyDataNewData = false;
      }
      Serial.println("Ok-AsyncData");
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

#include <LittleFS.h>
#include <DeviceSerial.h>  // the serial number is the efuse MAC (2026-10-30)
#include "esp_heap_caps.h"
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
#include <PowerProfileRecord.h>
#include <PowerProfileStore.h>
#include <DeviceCommandRecord.h>
#include <DeviceCommandQueue.h>
#include "HeardEntry.h"
#include <Preferences.h>
#include <esp_system.h>
#include <esp_core_dump.h>

SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// Shown on the OLED at start-up, answered to the Pi's GetFirmware and in GetWebData (like Cleotilde).
// Add 1 to firmwareVersion with every flashed change; the build time tells two builds of one version apart.
const int firmwareVersion = 10;
#define FIRMWARE_BUILD_TIME __DATE__ " " __TIME__

// Display settings for 2.4" SSD1309 — same OLED hardware as Paula
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define SLEEP_SWITCH_26 26
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// TPL5010 hardware watchdog on the WallyI2CDataVisualizer shield (2026-10-22). The shield replaced
// the one with the slide switch: Annabelle went dark several times after the Pi lost power and
// nothing reset her. If DONE isn't pulsed within the TPL5010's period, its RSTN pulls EN low (JP3
// closed) - a real hardware reset. watchdogTask (core 0) pulses DONE every second, but only while
// loop() is alive, so a hung loop() still gets Annabelle reset. GPIO33 (WAKE) is left unused.
#define WDT_DONE_PIN 25
#define LED_POWER_PIN 23           // TPS2051B EN on the shield: HIGH powers the WS2812B LEDs (active high,
                                   // despite the ~EN on the schematic symbol - Daffodil's LED_CONTROL too)
#define WDT_SETUP_LIMIT_MS 180000UL  // setup() (WiFi scan and connect) may take this long
#define WDT_LOOP_STALL_MS 90000UL    // loop() silent this long counts as hung (weather download < 30 s)
volatile uint32_t loopAliveMs = 0;  // millis() at the top of the last loop() - 0 until setup() ends
TaskHandle_t watchdogTaskHandle = NULL;

#define LED_PIN 19
#define NUM_LEDS 6
TaskHandle_t ledShowTask = NULL;
int ledShowDuration = 250;
// FastLED.show() must never run on two cores at once: the RMT driver returns an error for a
// refresh started while the previous one is still sending, and FastLED's ESP_ERROR_CHECK turns
// that into abort(). That was the 2026-10-05 19:46 PANIC (loop() -> updateStatusLeds() on core 1
// while ledShowTask was animating on core 0). Every show goes through showLeds().
SemaphoreHandle_t ledMutex = NULL;  // created in setup() right after FastLED.addLeds
volatile bool runLedShow = false;   // set by loop(), cleared by ledShowTask when the animation ends


WeatherForecastManager* weatherForecastManager;

DataManager dataManager(Serial, LittleFS);
ErrorManager errorManager;
LoRaError cadResult;
float avgRssi = 0;
bool weatherPending = false;      // set when trigger fires; runs when serial is free
const char *weatherTrigger = "timer";  // who set weatherPending: "timer" or "web" (Send Now)

// Weather on request (2026-10-13). The 5-minute broadcast reaches units that are awake; a sleeping
// Daffodil is awake ~2 s per wake and almost never hears one, so its forecast goes stale. When its
// pre-sleep data packet (sleepTime > 0) says the forecast is stale (opMode bit 1 clear), Annabelle
// sends the cached forecast straight back: after that unit's VitalSigns, the last packet before it
// listens (Daffodil.ino listenForWeatherReply), or WEATHER_REPLY_FALLBACK_MS after the data packet
// if the VitalSigns doesn't arrive. No download - the cached forecast is at most 5 min old.
#define WEATHER_REPLY_AFTER_VS_MS 100        // give the unit time to switch its radio to receive
#define WEATHER_REPLY_FALLBACK_MS 2200       // data packet + 1 s gap + VitalSigns airtime + margin
#define WEATHER_REPLY_MIN_GAP_MS 240000UL    // per unit: a unit that keeps rejecting it costs at most this often
#define WEATHER_REPLY_UNITS 8
bool weatherReplyPending = false;
uint8_t weatherReplySerial[8];
unsigned long weatherReplyDueMs = 0;
struct WeatherReplyLog { uint8_t serial[8]; unsigned long ms; };
WeatherReplyLog weatherReplyLog[WEATHER_REPLY_UNITS] = {};
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
  sizeof(GraveyardShiftUpdate), sizeof(RequestCommand), sizeof(DeviceIdentityRecord),
  sizeof(VitalSignsRecordV2), sizeof(VitalSignsRecordV3), sizeof(PowerProfileRecord), sizeof(DeviceCommandRecord)
};
static_assert(LoRaPacketSizeCheck::allDistinct(loraPacketSizes, sizeof(loraPacketSizes) / sizeof(loraPacketSizes[0]), 0, 1),
              "two LoRa packet structs have the same size - receivers dispatch on size, so they would be confused");

// Latest VitalSignsRecord per device, relayed to the Pi with AsyncData - see VitalSignsStore.h
VitalSignsStore vitalSignsStore;
// Latest DeviceIdentityRecord per device (product definition + running build), relayed the same way
DeviceIdentityStore deviceIdentityStore;
// Latest PowerProfileRecord per device (battery/sensor parameters for the Cerebellum's SOC), same way
PowerProfileStore powerProfileStore;

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
volatile uint32_t loraDio0AtMs = 0;  // millis() when DIO0 last rose
volatile bool ownTxDone = false;     // sendMessage() finished: the next empty parsePacket() is its TX-done
int lastReceivedPacketSize = 0;

// Radio listening statistics since boot (2026-10-15), served under "radio" on /Telepathons. TopTank's
// packets were arriving only ~55% of the time at a strong signal (-76 dBm, SNR 11) with data and
// VitalSigns lost at random - these measure how long Annabelle itself is not listening:
//   busy    parsePacket() -> back in receive, in loraRxTask (since 2026-10-16 just the copy into rxSlots;
//           before, all of processLora()). parsePacket() leaves the radio in standby, so a packet
//           arriving while processLora() runs (flash writes included) is never heard at all.
//   latency DIO0 -> parsePacket(). The radio is still receiving, but a second packet in that time
//           replaces the first (DIO0 stays high, so no second interrupt - it can't be counted).
//   tx      sendMessage(): CAD, back-offs and the transmit. txClobbered = a received packet still
//           waiting to be read when a send started (the send overwrites it).
//   noPacket DIO0 without a valid packet (CRC/header error) - our own TX-done is not counted.
// Written by loraRxTask and loop() (same core); read by the async_tcp task without a lock (32-bit reads, a torn
// snapshot only mixes two moments).
struct RadioStats {
  uint32_t rxPackets, noPacket;
  uint32_t busyMsTotal, busyMsMax, busyOver1s;
  uint32_t latencyMsTotal, latencyMsMax, latencyOver1s;
  uint32_t txCount, txMsTotal, txMsMax, txClobbered;
  uint32_t dio0Stuck;  // loraRxTask found DIO0 high with no interrupt pending and recovered
  uint32_t rxDropped;  // packet read but the queue for loop() was full
  // Where late reads come from (2026-10-16 evening: late reads and the average delay crept up
  // after ~7 h): how long loraRxTask waited for loraMutex (held by sends), and the longest loop().
  uint32_t lockWaitMsMax, lockWaitOver1s;
  uint32_t loopMsMax, loopOver1s;
  uint32_t wifiReconnects, wifiDownSecMax;  // WiFi watchdog (2026-10-17)
};
RadioStats radioStats = {};
// Reset diagnostics are defined further down (near recordResetReason()); fillTelepathonsJson()
// reports them.
extern String resetReason;
extern bool previousUptimeKnown;
extern uint8_t previousLoopStage;
extern uint32_t resetDiagLowMemRestarts;
const char *loopStageName(uint8_t s);
// Failed memory requests (2026-10-31: v6 aborted in AsyncTCP tcp_accept when new AsyncClient
// failed, although the heap never fell below ~34 KB free - fragmentation or a corrupted heap?).
// heap_caps calls allocFailed() on every failed malloc/new, before the caller handles it or
// aborts; the last one is kept in RTC memory so the boot after a panic can report it.
struct AllocFailRecord {
  uint32_t count;          // failed requests this boot
  uint32_t size;           // bytes the last one asked for
  uint32_t caps;           // MALLOC_CAP_* flags it asked for
  uint32_t freeBytes;      // free 8-bit heap right after it failed
  uint32_t largestBlock;   // largest free 8-bit block right after it failed
  uint32_t uptimeSeconds;  // when it failed
  char task[16];           // task that asked
};
extern AllocFailRecord allocFail;
extern AllocFailRecord previousAllocFail;
// Declared here so the Arduino builder doesn't put its own prototypes above the struct
void allocFailed(size_t size, uint32_t caps, const char *functionName);
void formatAllocFail(const AllocFailRecord &r, char *buf, size_t n);
// Fragmentation tracker data - see fragBegin()/fragEnd() further down
#define FRAG_STAGES 13
struct FragStat {
  int32_t blockChange;  // bytes, summed over all runs of the step (negative = it shrank the block)
  uint32_t runs;
};
FragStat fragStats[FRAG_STAGES] = {};
uint32_t fragBootBlock = 0;

// Radio and memory history: one snapshot every 10 minutes, the last 72 (12 h), served on
// /RadioHistory for overnight analysis. Kept in RTC slow memory (RTC_DATA_ATTR), not DRAM, which is
// full; cleared by a power-on reset. Heap values are in units of 16 bytes to fit uint16_t.
#define RADIO_HISTORY_SLOTS 72
#define RADIO_HISTORY_MS (10UL * 60UL * 1000UL)
struct RadioHistoryRow {
  uint32_t epoch;
  uint16_t heapFree16, heapBlock16;  // free heap and largest free block, /16
  uint16_t rx, noPacket, late;       // in these 10 minutes
  uint16_t latencyAvgMs, loopMaxMs, lockWaitMaxMs;
};
// RTC_NOINIT (2026-10-17): survives crashes, watchdog and WiFi restarts - only a power-on clears it
// (the magic is checked on the first radioHistoryTick()).
#define RADIO_HISTORY_MAGIC 0x52484953
RTC_NOINIT_ATTR RadioHistoryRow radioHistory[RADIO_HISTORY_SLOTS];
RTC_NOINIT_ATTR uint8_t radioHistoryHead, radioHistoryCount;
RTC_NOINIT_ATTR uint32_t radioHistoryMagic;
uint32_t histLoopMaxMs = 0, histLockWaitMaxMs = 0;  // since the last snapshot

// The last few noPacket events with the signal they came in at (2026-10-15): 28 of 88 packets
// were arriving corrupted - the strength says whose they are (TopTank about -76 dBm, FISHTANK
// about -107). Packet RSSI/SNR registers are set at RX done whatever the CRC says.
#define CORRUPT_PACKETS_KEPT 8
struct CorruptPacket {
  uint32_t epoch;
  int16_t rssi;
  int8_t snrQ;  // SNR x 4
};
CorruptPacket corruptPackets[CORRUPT_PACKETS_KEPT] = {};
uint8_t corruptPacketsHead = 0;  // next slot to write

// Received packets, read out of the radio by loraRxTask the moment DIO0 rises (2026-10-16).
// Before, loop() read the radio itself, and loop() is sometimes 1-4 s late (Pi serial commands,
// weather download, web requests): the radio keeps receiving, so a second packet in that time
// replaced the first - TopTank delivered ~50% at -72 dBm. loraRxTask (priority 3, core 1, above
// loop()'s 1) reads within milliseconds whatever loop() is doing and queues the packet here;
// loop() runs processLora() on it later, from rxPacketRead()/rxPacketRssi()/rxPacketSnr().
// loraMutex: one user of the radio at a time - loraRxTask, or loop()'s sendMessage()/LoRa_rxMode().
#define RX_SLOTS 4
struct RxPacket {
  uint8_t len;
  int16_t rssi;
  float snr;
  uint8_t data[255];
};
RxPacket rxSlots[RX_SLOTS];
volatile uint8_t rxHead = 0;  // written by loraRxTask
volatile uint8_t rxTail = 0;  // written by loop()
RxPacket *rxCurrent = nullptr;  // the packet processLora() is reading
uint16_t rxCurrentPos = 0;
int lastRxRssi = 0;      // last packet processed - for the OLED
float lastRxSnr = 0;
SemaphoreHandle_t loraMutex = nullptr;
TaskHandle_t loraRxTaskHandle = nullptr;

void loraLock() {
  if (loraMutex) xSemaphoreTake(loraMutex, portMAX_DELAY);
}
void loraUnlock() {
  if (loraMutex) xSemaphoreGive(loraMutex);
}

// processLora()'s replacement for LoRa.readBytes(): copies from the queued packet, zero-fills
// past its end.
size_t rxPacketRead(uint8_t *dst, size_t len) {
  size_t n = 0;
  while (n < len && rxCurrent && rxCurrentPos < rxCurrent->len) dst[n++] = rxCurrent->data[rxCurrentPos++];
  while (n < len) dst[n++] = 0;
  return len;
}
int rxPacketRssi() {
  return rxCurrent ? rxCurrent->rssi : 0;
}
float rxPacketSnr() {
  return rxCurrent ? rxCurrent->snr : 0;
}

// Last-heard table: one row per telepathon, updated by processLora() on every packet and served
// as JSON for the web page (data/telepathons.js): /Telepathons has every row with its newest card,
// /TelepathonHistory the cards kept for one row. A row is real once one of its packets checks out
// - checksum, or for the types without one (Chinampa, Gloria, Seedling) a readable name and a
// device clock within two days of real time. Everything else is junk: packets from a device whose
// struct only shares its size with ours (FISHTANK, 2026-10-11), or weak-signal bit errors. A real
// row keeps only the packets that checked out; the page piles the junk rows at the bottom. Rows are
// matched by name, then by serial (VitalSigns/DeviceIdentity carry only a serial). RAM only - it
// starts empty after a reset. heardMux guards it: processLora() runs in loop(), the web requests in
// the async_tcp task.
HeardEntry heard[MAX_HEARD];
int heardCount = 0;
portMUX_TYPE heardMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE webMux = portMUX_INITIALIZER_UNLOCKED;  // weatherStatus and the settings queue below

// Commands for one device (2026-10-14) - so far a Daffodil's WiFi on/off from the web page. Held
// and resent until the device's data shows the change (DeviceCommandQueue.h). Guarded by heardMux
// like the rows it is shown on; transmitted from loop() only.
DeviceCommandQueue deviceCommandQueue;

// Outcome of the last WeatherForecastUpdate broadcast (5-minute timer, the Pi's getWF, or the
// page's Send Now) - the weather line under the header of data/index.html. Written by
// sendWeatherForecast() in loop(), read by the /Telepathons request, both under webMux.
enum WeatherDownload : uint8_t { WEATHER_DL_OK = 1, WEATHER_DL_FAILED, WEATHER_DL_NO_INTERNET };
#define WEATHER_NO_FORECAST -1  // WeatherSendStatus.lora: there was nothing to send
struct WeatherSendStatus {
  uint32_t epoch;     // Annabelle RTC, 0 = nothing sent since the reset
  uint8_t download;   // WeatherDownload
  int8_t lora;        // LoRaError from sendMessage(), or WEATHER_NO_FORECAST
  uint8_t retries;    // CAD back-offs before it went out, or before giving up
  char trigger[6];    // "timer", "Pi", "web"
  WeatherForecast forecasts[4];
};
WeatherSendStatus weatherStatus = {};

static bool serialIsSet(const uint8_t *s) {
  for (int i = 0; i < 8; i++) if (s[i]) return true;
  return false;
}

// The printable-ASCII start of a fixed-size name field, cut at the first byte that isn't - bit
// errors behind a real name then land on the same row instead of making a new one.
static void cleanName(const char *name, size_t nameLen, char out[17]) {
  size_t n = 0;
  if (name) {
    while (n < nameLen && n < 16 && name[n] >= 0x20 && name[n] <= 0x7E) {
      out[n] = name[n];
      n++;
    }
  }
  out[n] = '\0';
}

// A name somebody gave: at least 2 characters, all printable up to the terminator.
static bool nameLooksReal(const char *name, size_t nameLen) {
  size_t n = 0;
  while (n < nameLen && name[n] != '\0') {
    if (name[n] < 0x20 || name[n] > 0x7E) return false;
    n++;
  }
  return n >= 2;
}

// Real Unix time from SNTP, 0 until it has synced. Device clocks (secondsTime) are Unix time;
// Annabelle's own RTC epoch (heardNow) is local time counted as UTC.
static uint32_t realNow() {
  time_t t = time(nullptr);
  return t > 1700000000 ? (uint32_t)t : 0;
}

// How far the device clock is ahead of real time, NAN when either clock is unknown.
static float clockOffset(long deviceTime) {
  uint32_t now = realNow();
  if (!now || deviceTime <= 0) return NAN;
  return (float)(deviceTime - (long)now);
}

// For packet types without a checksum.
static bool plausible(const char *name, size_t nameLen, long deviceTime) {
  if (!nameLooksReal(name, nameLen)) return false;
  float off = clockOffset(deviceTime);
  return isnan(off) || fabsf(off) < 2 * 86400;
}

static uint32_t heardNow() {
  return (uint32_t)timeManager.getCurrentTimeInSeconds(currentTimerRecord);
}

// Finds (or creates) the row for a device. Call inside heardMux. name is cleaned, "" for
// serial-only records. A serial-only record also matches a row with no name yet, and a named
// packet adopts the unnamed row a VitalSigns made before its first data packet arrived. When the
// table is full, the junk row heard from longest ago is reused, or failing that the oldest row.
static HeardEntry *heardRow(const char *name, const uint8_t *serial) {
  bool hasSerial = serial && serialIsSet(serial);
  if (name[0]) {
    for (int i = 0; i < heardCount; i++) {
      if (strcmp(heard[i].name, name) == 0) return &heard[i];
    }
  }
  for (int i = 0; i < heardCount; i++) {
    if (name[0] && heard[i].name[0]) continue;
    if (hasSerial ? memcmp(heard[i].serial, serial, 8) == 0 : (!name[0] && !serialIsSet(heard[i].serial))) return &heard[i];
  }
  int idx = heardCount;
  if (heardCount < MAX_HEARD) {
    heardCount++;
  } else {
    idx = -1;
    for (int i = 0; i < MAX_HEARD; i++) {
      if (heard[i].okCount == 0 && (idx < 0 || heard[i].lastEpoch < heard[idx].lastEpoch)) idx = i;
    }
    if (idx < 0) {
      idx = 0;
      for (int i = 1; i < MAX_HEARD; i++) if (heard[i].lastEpoch < heard[idx].lastEpoch) idx = i;
    }
  }
  HeardEntry *e = &heard[idx];
  uint16_t gen = e->gen + 1;
  memset(e, 0, sizeof(HeardEntry));
  e->gen = gen;
  if (hasSerial) memcpy(e->serial, serial, 8);
  return e;
}

// Weather/time-sync bookkeeping for a Daffodil or Langley data packet that checked out: its opMode
// byte and clock offset. A jump of 10 s or more towards zero between two packets is taken as
// the device setting its RTC from a WeatherForecastUpdate (Daffodil acceptAnnabelleTime corrects
// anything over 5 s) - the device doesn't report the correction itself. Call inside heardMux.
static void noteSync(HeardEntry *e, uint32_t epoch, int opMode, float off) {
  bool fresh = opMode & 0x02;
  if (fresh) {
    if (!e->weatherFresh) e->weatherFreshSince = epoch;
    e->weatherLastFresh = epoch;
  }
  e->weatherFresh = fresh;
  e->weatherSeen = true;
  if (isnan(off)) return;
  // A timestamp more than a day off is a bad date in that one packet (TopTank sent dates in 2030-32
  // with the right time of day, 2026-10-13), not the clock: skip it, so the next good packet isn't
  // reported as a "correction" of a clock that was never wrong.
  if (fabsf(off) > 86400) return;
  if (e->offsetKnown && fabsf(off) <= fabsf(e->lastOffset) - 10) {
    e->clockCorrectedAt = epoch;
    e->clockCorrectedBy = e->lastOffset;
  }
  e->lastOffset = off;
  e->offsetKnown = true;
}

// Records one packet on its device's row. kind null keeps the row's (serial-only records).
static void noteCard(const char *rawName, size_t nameLen, const uint8_t *serial, const char *kind, HeardCardType type,
                     bool ok, int rssi, float snr, const float *v, int nv) {
  char name[17];
  cleanName(rawName, nameLen, name);
  HeardCard card;
  card.epoch = heardNow();
  card.type = type;
  card.ok = ok;
  card.rssi = rssi;
  card.snr = snr;
  for (int i = 0; i < CARD_VALUES; i++) card.v[i] = i < nv ? v[i] : NAN;

  portENTER_CRITICAL(&heardMux);
  HeardEntry *e = heardRow(name, serial);
  if (name[0] && !e->name[0]) strcpy(e->name, name);
  if (ok && serial && serialIsSet(serial)) memcpy(e->serial, serial, 8);
  if (kind && (!e->kind[0] || type != CARD_VITAL)) strncpy(e->kind, kind, sizeof(e->kind) - 1);
  e->lastEpoch = card.epoch;
  e->rssi = rssi;
  e->snr = snr;
  if (ok) {
    if (e->okCount++ == 0) e->histLen = e->histHead = 0;  // just became real: drop the junk cards
  } else {
    e->badCount++;
  }
  if (ok || e->okCount == 0) {
    e->history[e->histHead] = card;
    e->histHead = (e->histHead + 1) % HEARD_HISTORY;
    if (e->histLen < HEARD_HISTORY) e->histLen++;
  }
  portEXIT_CRITICAL(&heardMux);
}

// Weather, clock and WiFi of a Daffodil or Langley data packet that checked out (page's Radio tab).
static void noteSyncAndWifi(const char *rawName, size_t nameLen, const uint8_t *serial, int opMode, float offset, uint8_t wifiStatus) {
  char name[17];
  cleanName(rawName, nameLen, name);
  portENTER_CRITICAL(&heardMux);
  HeardEntry *e = heardRow(name, serial);
  noteSync(e, heardNow(), opMode, offset);
  e->wifiKnown = true;
  e->wifiStatus = wifiStatus;
  portEXIT_CRITICAL(&heardMux);
}

// The readings a Daffodil's function uses, in the order telepathons.js FUNCTIONS labels them, after
// its board temperature and its outdoor temperature/humidity (the I2C sensor, any function - NAN when
// the Daffodil reports -99, no sensor). NAN = unused slot. Same split as the Daffodil's own page (index.js refreshScreen).
static void daffodilSensors(const DigitalStablesData &d, float out[DAFFODIL_SENSORS]) {
  for (int i = 0; i < DAFFODIL_SENSORS; i++) out[i] = NAN;
  out[0] = (int8_t)d.temperature;  // DS18B20 on the board, whole degrees (negative wraps in the uint8_t)
  if (d.outdoortemperature != -99) { out[1] = d.outdoortemperature; out[2] = d.outdoorhumidity; }
  float *f = out + 3;
  switch (d.currentFunctionValue) {
    case FUN_1_FLOW: f[0] = d.flowRate; f[1] = d.totalMilliLitres; break;
    case FUN_2_FLOW: f[0] = d.flowRate; f[1] = d.totalMilliLitres; f[2] = d.flowRate2; f[3] = d.totalMilliLitres2; break;
    case FUN_1_FLOW_1_TANK: f[0] = d.flowRate; f[1] = d.totalMilliLitres; f[2] = d.tank1PressurePsi; break;
    case FUN_1_TANK: f[0] = d.tank1PressurePsi; break;
    case FUN_2_TANK: f[0] = d.tank1PressurePsi; f[1] = d.tank2PressurePsi; break;
    case DAFFODIL_SCEPTIC_TANK: f[0] = d.measuredHeight; f[1] = d.maximumScepticHeight; break;
    case DAFFODIL_WATER_TROUGH: f[0] = d.measuredHeight; f[1] = d.troughlevelminimumcm; f[2] = d.troughlevelmaximumcm; break;
    // DAFFODIL_TEMP_SOILMOISTURE: its outdoor temperature/humidity are already in out[1], out[2]
    case VOLTAGE_MONITOR: f[0] = d.flowRate; break;  // the voltage on sensor 1 (Daffodil.ino reads it into flowRate)
    case DAFFODIL_WATER_TROUGH_TANK1: f[0] = d.measuredHeight; f[1] = d.tank1PressurePsi; break;
    case DAFFODIL_2_WATER_TROUGH: f[0] = d.measuredHeight; f[1] = d.measuredHeight2; break;
    case DAFFODIL_WATER_TROUGH_WATER_TEMP: f[0] = d.measuredHeight; f[1] = d.measuredHeight2; break;  // measuredHeight2 is the water temperature
  }
}

void noteDigitalStablesPacket(const DigitalStablesData &d, bool ok) {
  float v[] = { d.batteryVoltage, d.batteryCurrent, d.panelVoltage, d.panelCurrent, d.v50Voltage,
                (float)d.operatingStatus, (float)d.sleepTime, (float)d.opMode, clockOffset(d.secondsTime) };
  static_assert(sizeof(v) / sizeof(v[0]) <= CARD_VALUES, "more values than a HeardCard holds");
  noteCard(d.devicename, sizeof(d.devicename), d.serialnumberarray, "Daffodil", CARD_DAFFODIL, ok, (int)d.rssi, d.snr, v, 9);
  if (ok) {
    noteSyncAndWifi(d.devicename, sizeof(d.devicename), d.serialnumberarray, d.opMode, v[8], d.wifiStatus);
    float sensors[DAFFODIL_SENSORS];
    daffodilSensors(d, sensors);
    char name[17];
    cleanName(d.devicename, sizeof(d.devicename), name);
    portENTER_CRITICAL(&heardMux);
    HeardEntry *e = heardRow(name, d.serialnumberarray);
    e->function = d.currentFunctionValue;
    memcpy(e->sensors, sensors, sizeof(sensors));
    portEXIT_CRITICAL(&heardMux);
  }
}

void noteChinampaPacket(const ChinampaData &d, bool ok) {
  float v[] = { d.fishTankMeasuredHeight, d.sumpTroughMeasuredHeight, d.rtcBatVolt, (float)d.microtemperature, d.lux,
                (float)d.alertcode, clockOffset(d.secondsTime) };
  static_assert(sizeof(v) / sizeof(v[0]) <= CARD_VALUES, "more values than a HeardCard holds");
  noteCard(d.devicename, sizeof(d.devicename), d.serialnumberarray, "Chinampa", CARD_CHINAMPA, ok, (int)d.rssi, d.snr, v, 7);
}

void noteLangleyPacket(const LangleyData &d, bool ok) {
  float v[] = { d.batteryVoltage, d.batteryCurrentMa, d.solarVoltage, d.solarCurrentMa, d.fenceVoltageAvg,
                d.energizerBatteryVoltage, d.temperature, clockOffset(d.secondsTime), d.sleepTime };
  static_assert(sizeof(v) / sizeof(v[0]) <= CARD_VALUES, "more values than a HeardCard holds");
  noteCard(d.devicename, sizeof(d.devicename), d.serialnumberarray, "Langley", CARD_LANGLEY, ok, (int)d.rssi, d.snr, v, 9);
  // sleepTime/wifiStatus/opMode since 2026-10-23 - older Langleys send 1800/3.75/3.25 there until reflashed
  if (ok) noteSyncAndWifi(d.devicename, sizeof(d.devicename), d.serialnumberarray, (int)d.opMode, v[7], (uint8_t)d.wifiStatus);
}

void noteGloriaPacket(const GloriaTankFlowPumpData &d, bool ok) {
  float v[] = { d.solarvoltage, d.capacitorvoltage, d.totalMilliLitres, d.totalMilliLitres2, (float)d.temperature, clockOffset(d.secondsTime) };
  static_assert(sizeof(v) / sizeof(v[0]) <= CARD_VALUES, "more values than a HeardCard holds");
  noteCard(d.devicename, sizeof(d.devicename), d.serialnumberarray, "Gloria", CARD_GLORIA, ok, (int)d.rssi, d.snr, v, 6);
}

void noteSeedlingPacket(const SeedlingMonitorData &d, bool ok) {
  float v[] = { d.temperature, clockOffset(d.secondsTime) };
  static_assert(sizeof(v) / sizeof(v[0]) <= CARD_VALUES, "more values than a HeardCard holds");
  noteCard(d.devicename, sizeof(d.devicename), d.serialnumberarray, "Seedling", CARD_SEEDLING, ok, (int)d.rssi, d.snr, v, 2);
}

void noteVitalSignsPacket(const VitalSignsRecord &r, bool ok, int rssi, float snr) {
  float v[] = { (float)r.resetCount, (float)r.seq, r.wakeVoltage_mV / 1000.0f, r.minVoltageSinceReport_mV / 1000.0f,
                (float)r.txDurationMs, (float)r.loraTxFailCount, (float)r.lastWakeDriftSec, (float)r.lastAwakeSec,
                (float)r.wakeCount };
  static_assert(sizeof(v) / sizeof(v[0]) <= CARD_VALUES, "more values than a HeardCard holds");
  noteCard(nullptr, 0, r.serialnumberarray, "Vital", CARD_VITAL, ok, rssi, snr, v, 9);
}

void noteIdentityPacket(const DeviceIdentityRecord &r, bool ok, int rssi, float snr) {
  noteCard(nullptr, 0, r.serialnumberarray, nullptr, CARD_IDENTITY, ok, rssi, snr, nullptr, 0);
}

void noteCommaPacket(const CommaRecord &r, bool ok, int rssi, float snr) {
  float v[] = { r.voltage, (float)r.index, (float)r.total };
  static_assert(sizeof(v) / sizeof(v[0]) <= CARD_VALUES, "more values than a HeardCard holds");
  noteCard(r.devicename, sizeof(r.devicename), r.serialnumber, nullptr, CARD_COMMA, ok, rssi, snr, v, 3);
}

void notePowerProfilePacket(const PowerProfileRecord &r, bool ok, int rssi, float snr) {
  float v[] = { (float)r.capacity_mAh };
  static_assert(sizeof(v) / sizeof(v[0]) <= CARD_VALUES, "more values than a HeardCard holds");
  noteCard(nullptr, 0, r.serialnumberarray, nullptr, CARD_POWER, ok, rssi, snr, v, 1);
}

static void cardJson(JsonObject o, const HeardCard &c) {
  o["t"] = c.epoch;
  o["ty"] = c.type;
  o["ok"] = c.ok;
  o["rssi"] = c.rssi;
  o["snr"] = c.snr;
  JsonArray v = o["v"].to<JsonArray>();
  int n = CARD_VALUES;
  while (n > 0 && isnan(c.v[n - 1])) n--;
  for (int i = 0; i < n; i++) v.add(c.v[i]);  // NAN serializes as null
}

static void weatherJson(JsonObject w) {
  WeatherSendStatus s;
  portENTER_CRITICAL(&webMux);
  s = weatherStatus;
  portEXIT_CRITICAL(&webMux);
  w["pending"] = weatherPending;
  if (!s.epoch) return;
  w["t"] = s.epoch;
  w["download"] = s.download;
  w["lora"] = s.lora;
  w["retries"] = s.retries;
  w["maxRetries"] = MAX_RETRIES;
  w["trigger"] = s.trigger;
  if (s.lora == WEATHER_NO_FORECAST) return;
  JsonArray fa = w["forecasts"].to<JsonArray>();
  for (int i = 0; i < 4; i++) {
    const WeatherForecast &f = s.forecasts[i];
    JsonObject o = fa.add<JsonObject>();
    o["t"] = f.secondsTime;  // Unix time
    o["hour"] = f.hour;
    o["temp"] = f.temperature;
    o["cloud"] = f.cloudiness;
    o["humidity"] = f.humidity;
    o["pressure"] = f.pressure;
    o["rain"] = f.rain;
  }
}

// One row without its history - what /Telepathons sends.
struct HeardSummary {
  char name[17];
  uint8_t serial[8];
  char kind[12];
  uint16_t gen;
  uint32_t lastEpoch, okCount, badCount;
  int16_t rssi;
  float snr;
  uint8_t histLen;
  HeardCard front;
  bool weatherSeen, weatherFresh, offsetKnown;
  uint32_t weatherFreshSince, weatherLastFresh, clockCorrectedAt;
  float lastOffset, clockCorrectedBy;
  bool wifiKnown;
  uint8_t wifiStatus;
  // the row's last WiFi request - only what the page shows (a whole Slot per row overflowed DRAM)
  uint8_t cmdState, cmdOn, cmdSends;
  uint32_t cmdAgeSec;
};

// Static: these run only in the async_tcp task, one request at a time, and are too big for its stack.
static HeardSummary heardSummaries[MAX_HEARD];
static HeardCard historyCopy[HEARD_HISTORY];

// Newest data packet (Daffodil..Seedling) and newest vital signs of a row, for the page's
// Sensors/Power/Radio tabs - the front card alone is usually the vital signs a device sends 1 s
// after its data, which hid the data. One row at a time under the lock, so only two cards of static
// memory instead of two per row in heardSummaries.
static HeardCard latestData, latestVital;
static int8_t latestFunction;
static float latestSensors[DAFFODIL_SENSORS];
static void copyLatestCards(int row, uint16_t gen, bool &hasData, bool &hasVital) {
  hasData = hasVital = false;
  latestFunction = 0;
  portENTER_CRITICAL(&heardMux);
  const HeardEntry &e = heard[row];
  if (e.gen == gen) {
    latestFunction = e.function;
    memcpy(latestSensors, e.sensors, sizeof(latestSensors));
    for (int i = 0; i < e.histLen && !(hasData && hasVital); i++) {
      const HeardCard &c = e.history[(e.histHead + HEARD_HISTORY - 1 - i) % HEARD_HISTORY];
      if (!hasData && c.type >= CARD_DAFFODIL && c.type <= CARD_SEEDLING) { latestData = c; hasData = true; }
      if (!hasVital && c.type == CARD_VITAL) { latestVital = c; hasVital = true; }
    }
  }
  portEXIT_CRITICAL(&heardMux);
}

static void serialHex(const uint8_t *s, char out[17]) {
  for (int b = 0; b < 8; b++) snprintf(out + b * 2, 3, "%02X", s[b]);
}

// /Telepathons JSON. Copies the rows under the lock, builds the JSON outside it.
void fillTelepathonsJson(JsonDocument &json) {
  int n;
  portENTER_CRITICAL(&heardMux);
  n = heardCount;
  for (int i = 0; i < n; i++) {
    const HeardEntry &e = heard[i];
    HeardSummary &s = heardSummaries[i];
    memcpy(s.name, e.name, sizeof(s.name));
    memcpy(s.serial, e.serial, sizeof(s.serial));
    memcpy(s.kind, e.kind, sizeof(s.kind));
    s.gen = e.gen;
    s.lastEpoch = e.lastEpoch;
    s.okCount = e.okCount;
    s.badCount = e.badCount;
    s.rssi = e.rssi;
    s.snr = e.snr;
    s.histLen = e.histLen;
    if (e.histLen) s.front = e.history[(e.histHead + HEARD_HISTORY - 1) % HEARD_HISTORY];
    s.weatherSeen = e.weatherSeen;
    s.weatherFresh = e.weatherFresh;
    s.weatherFreshSince = e.weatherFreshSince;
    s.weatherLastFresh = e.weatherLastFresh;
    s.offsetKnown = e.offsetKnown;
    s.lastOffset = e.lastOffset;
    s.clockCorrectedAt = e.clockCorrectedAt;
    s.clockCorrectedBy = e.clockCorrectedBy;
    s.wifiKnown = e.wifiKnown;
    s.wifiStatus = e.wifiStatus;
    DeviceCommandQueue::Slot c = deviceCommandQueue.status(e.serial);
    s.cmdState = c.state;
    s.cmdOn = c.command == DEVICE_COMMAND_WIFI_ON;
    s.cmdSends = c.sends > 255 ? 255 : c.sends;
    s.cmdAgeSec = (millis() - c.createdMs) / 1000;
  }
  portEXIT_CRITICAL(&heardMux);

  json["now"] = heardNow();
  json["uptimeSec"] = millis() / 1000;
  weatherJson(json["weather"].to<JsonObject>());
  // Radio listening statistics since boot - see RadioStats. deafPct: share of the uptime the radio
  // was not receiving (processing a packet, or sending).
  JsonObject radio = json["radio"].to<JsonObject>();
  radio["rxPackets"] = radioStats.rxPackets;
  radio["noPacket"] = radioStats.noPacket;
  radio["busyMsTotal"] = radioStats.busyMsTotal;
  radio["busyMsMax"] = radioStats.busyMsMax;
  radio["busyOver1s"] = radioStats.busyOver1s;
  radio["latencyMsTotal"] = radioStats.latencyMsTotal;
  radio["latencyMsMax"] = radioStats.latencyMsMax;
  radio["latencyOver1s"] = radioStats.latencyOver1s;
  radio["txCount"] = radioStats.txCount;
  radio["txMsTotal"] = radioStats.txMsTotal;
  radio["txMsMax"] = radioStats.txMsMax;
  radio["txClobbered"] = radioStats.txClobbered;
  radio["dio0Stuck"] = radioStats.dio0Stuck;
  radio["rxDropped"] = radioStats.rxDropped;
  radio["lockWaitMsMax"] = radioStats.lockWaitMsMax;
  radio["lockWaitOver1s"] = radioStats.lockWaitOver1s;
  radio["loopMsMax"] = radioStats.loopMsMax;
  radio["loopOver1s"] = radioStats.loopOver1s;
  radio["wifiReconnects"] = radioStats.wifiReconnects;
  radio["wifiDownSecMax"] = radioStats.wifiDownSecMax;
  radio["resetReason"] = resetReason;
  radio["resetStage"] = previousUptimeKnown ? loopStageName(previousLoopStage) : "";
  radio["lowMemRestarts"] = resetDiagLowMemRestarts;
  // Fragmentation by step: [name, total change of the largest free block in bytes, runs]
  radio["fragBootBlock"] = fragBootBlock;
  JsonArray frag = radio["frag"].to<JsonArray>();
  for (int i = 1; i < FRAG_STAGES; i++) {
    if (!fragStats[i].runs) continue;
    JsonArray f = frag.add<JsonArray>();
    f.add(loopStageName(i));
    f.add(fragStats[i].blockChange);
    f.add(fragStats[i].runs);
  }
  // Memory (2026-10-16): the /Telepathons reply gets cut off when no free block is big enough.
  radio["heapFree"] = ESP.getFreeHeap();
  radio["heapMin"] = ESP.getMinFreeHeap();
  // heapFree/heapMin count all internal RAM, including ~30 KB of IRAM that malloc/new can't use
  // (32-bit access only). heapFree8/heapMin8 are the 8-bit heap that ordinary allocations come from.
  radio["heapFree8"] = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_8BIT);
  radio["heapMin8"] = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  radio["heapBlock"] = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  // Failed memory requests (allocFailed()): [count, last size, caps, free, largest block, task, uptime s]
  // for this boot and for the boot before the last reset
  const AllocFailRecord *allocRecords[2] = { &allocFail, &previousAllocFail };
  const char *allocKeys[2] = { "allocFail", "allocFailPrev" };
  for (int i = 0; i < 2; i++) {
    AllocFailRecord r = *allocRecords[i];
    r.task[sizeof(r.task) - 1] = 0;
    JsonArray a = radio[allocKeys[i]].to<JsonArray>();
    a.add(r.count);
    a.add(r.size);
    a.add(r.caps);
    a.add(r.freeBytes);
    a.add(r.largestBlock);
    a.add((const char *)r.task);
    a.add(r.uptimeSeconds);
  }
  // Fixed-memory web replies (AnnabelleWifiManager, 2026-10-17)
  radio["replyPeak"] = (uint32_t)wifiManager.getReplyPeakBytes();
  radio["arenaPeak"] = (uint32_t)wifiManager.getArenaPeakBytes();
  radio["replyBusy"] = wifiManager.getReplyBusyRejects();
  radio["replyTooBig"] = wifiManager.getReplyTooBig();
  // Web gate (AnnabelleWifiManager, 2026-10-22): requests turned away so the page can't run memory out
  radio["webBusy"] = wifiManager.getWebBusy();
  radio["webLowMemory"] = wifiManager.getWebLowMemory();
  uint32_t upMs = millis();
  radio["deafPct"] = upMs ? round(1000.0 * (radioStats.busyMsTotal + radioStats.txMsTotal) / upMs) / 10.0 : 0;
  // Last corrupted packets, newest first: [epoch, rssi dBm, snr dB]
  JsonArray corrupt = radio["corrupt"].to<JsonArray>();
  for (int i = 1; i <= CORRUPT_PACKETS_KEPT; i++) {
    const CorruptPacket &c = corruptPackets[(corruptPacketsHead + CORRUPT_PACKETS_KEPT - i) % CORRUPT_PACKETS_KEPT];
    if (!c.epoch) break;
    JsonArray e = corrupt.add<JsonArray>();
    e.add(c.epoch);
    e.add(c.rssi);
    e.add(c.snrQ / 4.0);
  }
  JsonArray rows = json["telepathons"].to<JsonArray>();
  for (int i = 0; i < n; i++) {
    HeardSummary &s = heardSummaries[i];
    JsonObject r = rows.add<JsonObject>();
    char sn[17];
    serialHex(s.serial, sn);
    r["row"] = i;
    r["gen"] = s.gen;
    r["name"] = s.name[0] ? s.name : sn;
    r["serial"] = sn;
    r["kind"] = s.kind;
    r["real"] = s.okCount > 0;
    r["okCount"] = s.okCount;
    r["badCount"] = s.badCount;
    r["lastEpoch"] = s.lastEpoch;
    r["rssi"] = s.rssi;
    r["snr"] = s.snr;
    r["cards"] = s.histLen;
    // Junk rows (no packet ever checked out) send no card - their names and counts are enough for
    // the pile, and leaving 15+ junk cards out keeps this reply small enough to always fit in memory
    // (2026-10-16: it was being cut off after several hours). Their cards stay on /TelepathonHistory.
    if (s.histLen && s.okCount > 0) {
      cardJson(r["front"].to<JsonObject>(), s.front);
      // The newest data and vital signs cards when the front isn't already that card.
      bool hasData, hasVital;
      copyLatestCards(i, s.gen, hasData, hasVital);
      if (hasData && latestData.type != s.front.type) cardJson(r["data"].to<JsonObject>(), latestData);
      if (hasVital && s.front.type != CARD_VITAL) cardJson(r["vital"].to<JsonObject>(), latestVital);
      // Daffodil function and its sensor readings (daffodilSensors), NAN -> null
      if (latestFunction) {
        r["fn"] = latestFunction;
        JsonArray sv = r["sens"].to<JsonArray>();
        int ns = DAFFODIL_SENSORS;
        while (ns > 0 && isnan(latestSensors[ns - 1])) ns--;
        for (int k = 0; k < ns; k++) sv.add(latestSensors[k]);
      }
    }
    if (s.weatherSeen) {
      JsonObject sy = r["sync"].to<JsonObject>();
      sy["fresh"] = s.weatherFresh;
      sy["freshSince"] = s.weatherFreshSince;
      sy["lastFresh"] = s.weatherLastFresh;
      if (s.offsetKnown) sy["offset"] = s.lastOffset;
      if (s.clockCorrectedAt) {
        sy["correctedAt"] = s.clockCorrectedAt;
        sy["correctedBy"] = s.clockCorrectedBy;
      }
    }
    // WiFi (Daffodils): last reported status (0 off, 1 own hotspot, 2 router, 3 router + internet)
    // and the last on/off request from this page - state 1 waiting, 2 done, 3 gave up.
    if (s.wifiKnown) {
      JsonObject w = r["wifi"].to<JsonObject>();
      w["status"] = s.wifiStatus;
      if (s.cmdState != DEVICE_COMMAND_STATE_NONE) {
        JsonObject c = w["cmd"].to<JsonObject>();
        c["on"] = (bool)s.cmdOn;
        c["state"] = s.cmdState;
        c["sends"] = s.cmdSends;
        c["ageSec"] = s.cmdAgeSec;
      }
    }
  }
}

// ---- Annabelle's own status for the Pi (2026-11-01, firmware 10) ----
// One extra line in the AsyncData reply, so the Pi treats Annabelle like the devices she hears: a
// telepathon "Annabelle" with her serial number, stored and sent on to devices.digitalstables.com
// (AnnabelleStatusDeserializer in the Teleonome framework). Only when something is new since the last
// one (a packet arrived, the weather forecast went out or failed, the first after a reset), and at
// least every ANNABELLE_STATUS_HEARTBEAT_MS so a quiet night still shows she is alive.
// Wire format, '#' separated:
//   AnnabelleStatusDeserializer#1#name#serialnumber#epoch#uptimeSec#firmwareVersion#
//   weatherEpoch#weatherDownload#weatherLora#weatherRetries#weatherTrigger#weatherPending#
//   rxPackets#noPacket#rxDropped#txCount#deafPct#wifiReconnects#wifiDownSecMax#
//   resetReason#lowMemRestarts#heapFree8#heapMin8#heapBlock#loopMsMax#allocFailCount
#define ANNABELLE_STATUS_HEARTBEAT_MS 600000UL
// In RTC memory, not DRAM: v9 left 0 bytes of static DRAM (v10 with these in DRAM overflowed by 8).
// RTC_NOINIT keeps its value across a reset, so setup() clears it: every boot starts with "not sent".
struct AnnabelleStatusState {
  uint32_t sent, rx, weather, ms;
};
RTC_NOINIT_ATTR AnnabelleStatusState annabelleStatus;

static bool annabelleStatusDue() {
  if (serialNumber.length() == 0) return false;
  if (!annabelleStatus.sent) return true;
  if (radioStats.rxPackets != annabelleStatus.rx) return true;
  uint32_t weatherEpoch;
  portENTER_CRITICAL(&webMux);
  weatherEpoch = weatherStatus.epoch;
  portEXIT_CRITICAL(&webMux);
  if (weatherEpoch != annabelleStatus.weather) return true;
  return millis() - annabelleStatus.ms >= ANNABELLE_STATUS_HEARTBEAT_MS;
}

// Returns 1 when a line was sent, 0 otherwise (counted in AsyncData's reply like the other stores).
static int pushAnnabelleStatusToSerial(Stream &out) {
  if (!annabelleStatusDue()) return 0;
  WeatherSendStatus w;
  portENTER_CRITICAL(&webMux);
  w = weatherStatus;
  portEXIT_CRITICAL(&webMux);
  w.trigger[sizeof(w.trigger) - 1] = 0;
  uint32_t upMs = millis();
  float deafPct = upMs ? round(1000.0 * (radioStats.busyMsTotal + radioStats.txMsTotal) / upMs) / 10.0 : 0;
  out.print("AnnabelleStatusDeserializer#1#Annabelle#");
  out.print(serialNumber); out.print('#');
  out.print(heardNow()); out.print('#');
  out.print(upMs / 1000); out.print('#');
  out.print(firmwareVersion); out.print('#');
  out.print(w.epoch); out.print('#');
  out.print(w.download); out.print('#');
  out.print(w.lora); out.print('#');
  out.print(w.retries); out.print('#');
  out.print(w.epoch ? w.trigger : ""); out.print('#');
  out.print(weatherPending ? 1 : 0); out.print('#');
  out.print(radioStats.rxPackets); out.print('#');
  out.print(radioStats.noPacket); out.print('#');
  out.print(radioStats.rxDropped); out.print('#');
  out.print(radioStats.txCount); out.print('#');
  out.print(deafPct, 1); out.print('#');
  out.print(radioStats.wifiReconnects); out.print('#');
  out.print(radioStats.wifiDownSecMax); out.print('#');
  out.print(resetReason); out.print('#');
  out.print(resetDiagLowMemRestarts); out.print('#');
  out.print((uint32_t)heap_caps_get_free_size(MALLOC_CAP_8BIT)); out.print('#');
  out.print((uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT)); out.print('#');
  out.print((uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)); out.print('#');
  out.print(radioStats.loopMsMax); out.print('#');
  out.println(allocFail.count);
  annabelleStatus.sent = 1;
  annabelleStatus.rx = radioStats.rxPackets;
  annabelleStatus.weather = w.epoch;
  annabelleStatus.ms = upMs;
  return 1;
}

// /TelepathonHistory JSON: the row's cards, newest first. Empty when the row has since been
// reused for another device (gen changed).
void fillTelepathonHistoryJson(JsonDocument &json, int row, int gen) {
  int n = 0;
  portENTER_CRITICAL(&heardMux);
  if (row >= 0 && row < heardCount && (gen < 0 || heard[row].gen == gen)) {
    const HeardEntry &e = heard[row];
    n = e.histLen;
    for (int i = 0; i < n; i++) historyCopy[i] = e.history[(e.histHead + HEARD_HISTORY - 1 - i) % HEARD_HISTORY];
  }
  portEXIT_CRITICAL(&heardMux);

  json["row"] = row;
  json["gen"] = gen;
  JsonArray cards = json["cards"].to<JsonArray>();
  for (int i = 0; i < n; i++) cardJson(cards.add<JsonObject>(), historyCopy[i]);
}

// Settings page (data/settings.js -> AnnabelleWifiManager::setSettingsAction). The request arrives
// in the async_tcp task; the RTC (I2C, shared with the OLED) and preferences are only touched from
// loop(), so it is queued here and the page reads webResult back through GetWebData.
enum WebAction : uint8_t { WEB_NONE, WEB_SET_TIME, WEB_SET_TIME_INTERNET, WEB_SET_GPS, WEB_SEND_WEATHER, WEB_DEVICE_WIFI };
volatile uint8_t webAction = WEB_NONE;
char webArgA[32], webArgB[32];
char webResult[96] = "";
float boardTemperature = NAN;  // Annabelle's DS18B20, read once a minute in loop()
#define CONFIG_PREFS "annabelleCfg"

String queueWebAction(const String &form, const String &a, const String &b) {
  uint8_t action = form == "ManualSetTime" ? WEB_SET_TIME
                 : form == "SetTimeViaInternet" ? WEB_SET_TIME_INTERNET
                 : form == "SetGPS" ? WEB_SET_GPS
                 : form == "SendWeatherNow" ? WEB_SEND_WEATHER
                 : form == "DeviceWifi" ? WEB_DEVICE_WIFI
                 : WEB_NONE;
  if (action == WEB_NONE) return "";
  portENTER_CRITICAL(&webMux);
  bool busy = webAction != WEB_NONE;
  if (!busy) {
    strlcpy(webArgA, a.c_str(), sizeof(webArgA));
    strlcpy(webArgB, b.c_str(), sizeof(webArgB));
    webAction = action;
  }
  portEXIT_CRITICAL(&webMux);
  return busy ? "Busy with the previous request - try again" : "Queued";
}

static void setWebResult(const String &text) {
  portENTER_CRITICAL(&webMux);
  strlcpy(webResult, text.c_str(), sizeof(webResult));
  portEXIT_CRITICAL(&webMux);
}

void addWebDataExtras(DynamicJsonDocument &json) {
  char result[sizeof(webResult)];
  portENTER_CRITICAL(&webMux);
  memcpy(result, webResult, sizeof(result));
  portEXIT_CRITICAL(&webMux);
  json["secondsTime"] = heardNow();  // local time counted as UTC - read it back with the UTC getters
  json["firmware"] = "Version " + String(firmwareVersion) + " (" FIRMWARE_BUILD_TIME ")";
  json["ntpSynced"] = realNow() != 0;
  json["uptimeSec"] = millis() / 1000;
  json["boardTemperature"] = boardTemperature;
  json["settingsResult"] = result;
}

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
// Same gradient as loraReceivePalette in blue: Annabelle sending a WeatherForecastUpdate
CRGBPalette16 weatherSendPalette(CRGB::Blue, CRGB::DodgerBlue, CRGB::LightSkyBlue, CRGB::White);

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
// Where loop() was when the board reset (2026-10-17: an unexplained WDT reset at 06:54 after a
// night of router trouble). Set with LOOP_STAGE() at each big step; read back after the reset.
RTC_NOINIT_ATTR uint8_t resetDiagLoopStage;
// Planned low-memory restarts since the last power-on (lowMemoryWatchdogTick(), 2026-10-29)
RTC_NOINIT_ATTR uint32_t resetDiagLowMemRestarts;
// Last failed memory request (struct and comment near the top, with the other Telepathons externs)
RTC_NOINIT_ATTR AllocFailRecord allocFail;
AllocFailRecord previousAllocFail = {};  // allocFail as the last boot left it (count 0 = none, or not known)
enum LoopStage : uint8_t {
  STAGE_TOP = 1, STAGE_CLOCK, STAGE_WEB_ACTION, STAGE_WEATHER_DOWNLOAD, STAGE_WEATHER_TX, STAGE_DISPLAY,
  STAGE_LORA, STAGE_COMMAND_TX, STAGE_WEATHER_REPLY, STAGE_SERIAL, STAGE_WIFI_RESTART, STAGE_ASYNCDATA,
  STAGE_LOWMEM_RESTART
};
const char *loopStageName(uint8_t s) {
  static const char *names[] = { "unknown", "loop top", "1 s clock work", "web settings action", "weather download",
                                 "weather broadcast", "display", "LoRa packet", "device command send", "weather reply",
                                 "Pi serial command", "restart: WiFi lost", "Pi data download (AsyncData)",
                                 "restart: low memory" };
  return s < sizeof(names) / sizeof(names[0]) ? names[s] : "unknown";
}
#define LOOP_STAGE(s) (resetDiagLoopStage = (s))
// Registered in setup() with heap_caps_register_failed_alloc_callback(). Runs in whichever task
// asked, after the heap lock is released - it must not allocate.
void allocFailed(size_t size, uint32_t caps, const char *functionName) {
  allocFail.count++;
  allocFail.size = size;
  allocFail.caps = caps;
  allocFail.freeBytes = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  allocFail.largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  allocFail.uptimeSeconds = millis() / 1000;
  strlcpy(allocFail.task, pcTaskGetName(NULL), sizeof(allocFail.task));
}
// " allocFails=<n> allocSize=<bytes> allocCaps=0x.. allocFree=<bytes> allocBlock=<bytes> allocTask=<task>
// allocAt=<uptime s>" - space separated so decode_crash.sh skips it; "" when there was no failure.
void formatAllocFail(const AllocFailRecord &r, char *buf, size_t n) {
  if (!r.count) { buf[0] = 0; return; }
  char task[sizeof(r.task)];
  strlcpy(task, r.task, sizeof(task));
  for (char *c = task; *c; c++) if (*c == ' ' || *c == '|' || *c == '#' || *c == ';') *c = '_';
  snprintf(buf, n, " allocFails=%lu allocSize=%lu allocCaps=0x%lx allocFree=%lu allocBlock=%lu allocTask=%s allocAt=%lu",
           (unsigned long)r.count, (unsigned long)r.size, (unsigned long)r.caps, (unsigned long)r.freeBytes,
           (unsigned long)r.largestBlock, task, (unsigned long)r.uptimeSeconds);
}
// Which loop() step fragments the heap (2026-10-17: the largest free block shrinks from ~51 KB at
// boot while total free memory stays ~86 KB). fragBegin()/fragEnd() around each big step add the
// change in the largest free block to that step's total; served as "frag" on /Telepathons. What
// the steps don't explain happened elsewhere (web requests, WiFi, the LoRa task).
uint32_t fragStartBlock = 0;
static inline void fragBegin(uint8_t stage) {
  LOOP_STAGE(stage);
  fragStartBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
}
static inline void fragEnd(uint8_t stage) {
  if (stage >= FRAG_STAGES) return;
  fragStats[stage].blockChange += (int32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) - (int32_t)fragStartBlock;
  fragStats[stage].runs++;
}
uint8_t previousLoopStage = 0;  // resetDiagLoopStage as it was before this boot
bool wifiStationWanted = false;  // station mode configured - set in setup(), used by wifiWatchdogTick()
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
  } else if constexpr (std::is_same<T, VitalSignsRecordV2>::value) {
    checksumOffset = offsetof(VitalSignsRecordV2, checksum);
  } else if constexpr (std::is_same<T, VitalSignsRecordV3>::value) {
    checksumOffset = offsetof(VitalSignsRecordV3, checksum);
  } else if constexpr (std::is_same<T, PowerProfileRecord>::value) {
    checksumOffset = offsetof(PowerProfileRecord, checksum);
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
  } else if constexpr (std::is_same<T, VitalSignsRecordV2>::value) {
    receivedChecksum = tempData.checksum;
    tempData.checksum = 0;
  } else if constexpr (std::is_same<T, VitalSignsRecordV3>::value) {
    receivedChecksum = tempData.checksum;
    tempData.checksum = 0;
  } else if constexpr (std::is_same<T, PowerProfileRecord>::value) {
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

int lastSendRetries = 0;  // CAD back-offs in the last sendMessage(), for weatherStatus

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
  loraLock();  // loraRxTask finishes any packet it is reading first
  if (loraDio0Fired) radioStats.txClobbered++;  // a received packet not yet read - the send replaces it
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
      if constexpr (std::is_same<T, DeviceCommandRecord>::value) {
        // Same rule as the forecast: the code is for the time stamped here, after any CAD back-off.
        dataToSend.senderTime = (uint32_t)timeManager.getTimeForCodeGeneration();
        dataToSend.totpcode = secretManager.generateCodeAt(dataToSend.senderTime);
        dataToSend.sender = DEVICE_COMMAND_SENDER_ANNABELLE;
        dataToSend.checksum = deviceCommandChecksum(dataToSend);
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
      // Listen while backing off (2026-10-17). The radio used to sit in standby for the whole
      // 0.5-1.5 s (doubling) back-off, so a packet arriving then was lost without trace - TopTank's
      // VitalSigns, sent ~1-2.5 s after its data packet, kept landing in the back-off of the weather
      // reply Annabelle queues for it (TX Fail Count 0, 7 of 47 reports lost). Back in receive mode
      // with the lock released, loraRxTask takes anything that arrives; then try again.
      LoRa_rxMode();
      loraUnlock();
      delay(backoff);
      loraLock();
      LoRa_txMode();
      retries++;
      keepGoing = retries < MAX_RETRIES;
    } else {
      result = cadResult;
      keepGoing = false;
    }
  }
  if (result == 99) result = LORA_MAX_RETRIES_REACHED;
  lastSendRetries = retries;
  if (debug) Serial.print("Lora returns ");
  if (debug) Serial.println(result);
  // Our own TX-done raised DIO0. Leave loraDio0Fired set: loraRxTask's parsePacket() then clears ALL the
  // IRQ flags - endPacket() clears only TX_DONE, and an RX_DONE left over from a packet that arrived
  // just before this send keeps DIO0 high, so it never rises again and Annabelle hears nothing more
  // (2026-10-15 22:39: clearing the flag here left the radio deaf all night). ownTxDone keeps that
  // parsePacket() out of radioStats.noPacket.
  if (result == LORA_OK || result == LORA_TX_FAILED) ownTxDone = true;  // a CAD give-up sent nothing
  LoRa_rxMode();
  loraUnlock();
  uint32_t txMs = millis() - startsendingtime;
  radioStats.txCount++;
  radioStats.txMsTotal += txMs;
  if (txMs > radioStats.txMsMax) radioStats.txMsMax = txMs;
  msgCount++;
  return result;
}

// Downloads the forecast when there is internet and broadcasts its first 4 entries - also
// Annabelle's field time sync (see WEATHER_AIRTIME_SEC). loop() only: LoRa/SPI must never be
// touched from the async_tcp task (see loraDio0Fired). internetRechecks: extra pings before giving
// up on the download - getInternetAvailable() only reflects the last ping, and one failed ping
// (e.g. at boot, before DHCP/DNS settled) would otherwise latch false and stop every download.
// Mirrors Daffodil.ino's recheck-on-reconnect handling. The outcome goes to weatherStatus.
// Sends the 4 forecast entries as a WeatherForecastUpdate, with the blue LED show. loop() only.
int broadcastForecast(WeatherForecast *forecasts) {
  WeatherForecastUpdate weatherForecastUpdate;
  for (int i = 0; i < 4; i++) weatherForecastUpdate.forecasts[i] = forecasts[i];
  weatherForecastUpdate.totpcode = secretManager.generateCode();
  if (debug) Serial.print("sending weather via LoRa, size=");
  if (debug) Serial.println(sizeof(WeatherForecastUpdate));
  currentPalette = weatherSendPalette;
  currentBlending = LINEARBLEND;
  ledShowDuration = 2000;
  runLedShow = true;
  return sendMessage(weatherForecastUpdate);  // ends in RX mode; loop polls for packets
}

// The weather-on-request reply (see weatherReplyPending): cached forecast, no download, and
// weatherStatus (the page's "last broadcast" line) left alone.
void sendWeatherReply() {
  WeatherForecast *forecasts = weatherForecastManager->getForecasts();
  if (forecasts == nullptr || !weatherForecastManager->hasValidForecasts()) return;
  int result = broadcastForecast(forecasts);
  if (debug) Serial.printf("weather reply sent, lora=%d\n", result);
}

// A Daffodil's or Langley's pre-sleep data packet with a stale forecast: queue a reply, unless this
// unit got one in the last WEATHER_REPLY_MIN_GAP_MS.
void queueWeatherReply(const uint8_t *serial, const char *name, long sleepTime, int opMode) {
  if (sleepTime <= 0 || (opMode & 0x02) || !weatherForecastManager->hasValidForecasts()) return;
  unsigned long now = millis();
  int slot = -1, oldest = 0;
  for (int i = 0; i < WEATHER_REPLY_UNITS; i++) {
    if (memcmp(weatherReplyLog[i].serial, serial, 8) == 0) { slot = i; break; }
    if (weatherReplyLog[i].ms < weatherReplyLog[oldest].ms) oldest = i;
  }
  if (slot >= 0 && weatherReplyLog[slot].ms != 0 && now - weatherReplyLog[slot].ms < WEATHER_REPLY_MIN_GAP_MS) return;
  if (slot < 0) slot = oldest;
  memcpy(weatherReplyLog[slot].serial, serial, 8);
  weatherReplyLog[slot].ms = now;
  memcpy(weatherReplySerial, serial, 8);
  weatherReplyDueMs = now + WEATHER_REPLY_FALLBACK_MS;
  weatherReplyPending = true;
  if (debug) Serial.printf("weather reply queued for %.12s (stale forecast, sleeping %ld s)\n", name, sleepTime);
}

void sendWeatherForecast(int internetRechecks, const char *trigger) {
  if (debug) Serial.println("About to get and send weatherforecasts");
  // WiFi down (router out, 2026-10-17): no pings or downloads - each one waits for its timeouts
  // inside loop(), which stalled loop() for minutes at a time overnight.
  bool wifiUp = WiFi.status() == WL_CONNECTED;
  bool wifiAvail = wifiUp && wifiManager.getInternetAvailable();
  for (int i = 0; wifiUp && i < internetRechecks && !wifiAvail; i++) {
    wifiManager.checkInternetConnectionAvailable();
    wifiAvail = wifiManager.getInternetAvailable();
  }
  WeatherSendStatus s = {};
  strlcpy(s.trigger, trigger, sizeof(s.trigger));
  if (wifiAvail) {
    LOOP_STAGE(STAGE_WEATHER_DOWNLOAD);
    bool success = weatherForecastManager->downloadWeatherData();
    LOOP_STAGE(STAGE_WEATHER_TX);
    updateStatusLeds();
    s.download = success ? WEATHER_DL_OK : WEATHER_DL_FAILED;
    if (debug) Serial.println(success ? "Weather forecast downloaded" : "Weather forecast download failed");
  } else {
    s.download = WEATHER_DL_NO_INTERNET;
    if (debug) Serial.println("No internet, skipping weather forecast download");
  }
  WeatherForecast* forecasts = weatherForecastManager->getForecasts();
  if (forecasts != nullptr) {
    for (int i = 0; i < 4; i++) s.forecasts[i] = forecasts[i];
    s.lora = broadcastForecast(forecasts);
    s.retries = lastSendRetries;
  } else {
    s.lora = WEATHER_NO_FORECAST;
    if (debug) Serial.println("No forecast data available to send");
  }
  s.epoch = heardNow();
  portENTER_CRITICAL(&webMux);
  weatherStatus = s;
  portEXIT_CRITICAL(&webMux);
}

// A VitalSigns record that arrived (v4, or v3 upgraded): relay table, web page, and - it is the
// device's last packet before it listens - the timing of a weather reply or device command.
void handleVitalSigns(const VitalSignsRecord &record, bool checksumOk) {
  if (debug) {
    Serial.print("VitalSigns v");
    Serial.print(record.version);
    Serial.print(" seq=");
    Serial.print(record.seq);
    Serial.print(" resets=");
    Serial.print(record.resetCount);
    Serial.print(" cksum=");
    Serial.println(checksumOk ? "ok" : "BAD");
  }
  if (checksumOk) {
    vitalSignsStore.store(record, rxPacketRssi(), rxPacketSnr());
  }
  noteVitalSignsPacket(record, checksumOk, rxPacketRssi(), rxPacketSnr());
  if (checksumOk && weatherReplyPending && memcmp(record.serialnumberarray, weatherReplySerial, 8) == 0) {
    weatherReplyDueMs = millis() + WEATHER_REPLY_AFTER_VS_MS;  // its last packet - it is listening now
  }
  if (checksumOk) {
    portENTER_CRITICAL(&heardMux);
    deviceCommandQueue.noteVitalSigns(record.serialnumberarray, millis());
    portEXIT_CRITICAL(&heardMux);
  }
}

void processLora(int packetSize) {
  if (debug) Serial.println("Received lora size=" + String(packetSize));
  if (packetSize == 0) return;  // if there's no packet, return

  if (packetSize == sizeof(LangleyData)) {
    memset(&langleyData, 0, sizeof(LangleyData));
    rxPacketRead((uint8_t*)&langleyData, sizeof(LangleyData));
    // Validate on the as-received bytes before touching anything else - rssi/snr live inside
    // the checksummed range of the struct, and Langley_West's checksum was computed over what
    // it actually transmitted, which can't include the receiver's own RSSI/SNR. Overwriting
    // them first (as this used to do) made every checksum fail unconditionally, silently
    // dropping every Langley packet - see conversation 2026-07-21.
    bool checksumOk = validateChecksum(langleyData);
    langleyData.rssi = rxPacketRssi();
    langleyData.snr = rxPacketSnr();
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
      // Weather reply and WiFi commands, as for a Daffodil (Langley.ino since 2026-10-23)
      queueWeatherReply(langleyData.serialnumberarray, langleyData.devicename,
                        (long)langleyData.sleepTime, (int)langleyData.opMode);
      portENTER_CRITICAL(&heardMux);
      deviceCommandQueue.noteDataPacket(langleyData.serialnumberarray, (int)langleyData.wifiStatus != 0,
                                        langleyData.sleepTime > 0, millis());
      portEXIT_CRITICAL(&heardMux);
    }
    noteLangleyPacket(langleyData, checksumOk);

  } else if (packetSize == sizeof(GloriaTankFlowPumpData)) {
    memset(&gloriaTankFlowPumpData, 0, sizeof(GloriaTankFlowPumpData));
    rxPacketRead((uint8_t*)&gloriaTankFlowPumpData, sizeof(GloriaTankFlowPumpData));
    gloriaTankFlowPumpData.rssi = rxPacketRssi();
    gloriaTankFlowPumpData.snr = rxPacketSnr();
    // No checksum in this struct: junk = name not printable or clock > 2 days off (plausible()).
    bool ok = plausible(gloriaTankFlowPumpData.devicename, sizeof(gloriaTankFlowPumpData.devicename), gloriaTankFlowPumpData.secondsTime);
    if (ok) {
      dataManager.storeGloria(gloriaTankFlowPumpData);
      gloriaTankFlowPumpNewData = true;
    }
    noteGloriaPacket(gloriaTankFlowPumpData, ok);
    if (debug) Serial.print("received GloriaTankFlowPumpData from ");
    if (debug) Serial.println(gloriaTankFlowPumpData.devicename);

  } else if (packetSize == sizeof(DigitalStablesData)) {
    memset(&digitalStablesData, 0, sizeof(DigitalStablesData));
    rxPacketRead((uint8_t*)&digitalStablesData, sizeof(DigitalStablesData));
    // Checked on the bytes as received - rssi/snr are inside the checksummed range (see Langley
    // above). Junk (bad checksum) is dropped here and never queued for the Pi (2026-10-13) - only
    // counted on the web page. FISHTANK and SumpTrough pass; FISH (0 ok / 24 bad) does not.
    bool checksumOk = validateChecksum(digitalStablesData);
    digitalStablesData.rssi = rxPacketRssi();
    digitalStablesData.snr = rxPacketSnr();
    if (checksumOk) {
      dataManager.storeDigitalStablesData(digitalStablesData);
      digitalStablesDataNewData = true;
      queueWeatherReply(digitalStablesData.serialnumberarray, digitalStablesData.devicename,
                        digitalStablesData.sleepTime, digitalStablesData.opMode);
      portENTER_CRITICAL(&heardMux);
      deviceCommandQueue.noteDataPacket(digitalStablesData.serialnumberarray, digitalStablesData.wifiStatus != 0,
                                        digitalStablesData.sleepTime > 0, millis());
      portEXIT_CRITICAL(&heardMux);
    }
    noteDigitalStablesPacket(digitalStablesData, checksumOk);
    if (debug && !checksumOk) Serial.println("digitalStablesData checksum BAD - dropped");
    if (debug) dataManager.printDigitalStablesData(digitalStablesData);
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
    rxPacketRead((uint8_t*)&chinampaData, sizeof(ChinampaData));
    chinampaData.rssi = rxPacketRssi();
    chinampaData.snr = rxPacketSnr();
    // No checksum in this struct, and Chinampa must never be lost: junk = name not printable only.
    // The clock is NOT checked (unlike Gloria/Seedling) - a drifted or reset Chinampa RTC must not
    // drop its data. Since the 2026-10-13 restart: Chinampa 100 ok / 2 bad; 3 of 4 junk sources
    // of this size had unreadable names.
    bool ok = nameLooksReal(chinampaData.devicename, sizeof(chinampaData.devicename));
    if (ok) {
      dataManager.storeChinampaData(chinampaData);
      chinampaDataNewData = true;
    }
    noteChinampaPacket(chinampaData, ok);
    if (debug) Serial.print("received chinampaData from ");
    if (debug) Serial.println(chinampaData.devicename);

  } else if (packetSize == sizeof(SeedlingMonitorData)) {
    memset(&seedlingMonitorData, 0, sizeof(SeedlingMonitorData));
    rxPacketRead((uint8_t*)&seedlingMonitorData, sizeof(SeedlingMonitorData));
    seedlingMonitorData.rssi = rxPacketRssi();
    seedlingMonitorData.snr = rxPacketSnr();
    // No checksum in this struct: junk = name not printable or clock > 2 days off (plausible()).
    bool ok = plausible(seedlingMonitorData.devicename, sizeof(seedlingMonitorData.devicename), seedlingMonitorData.secondsTime);
    if (ok) {
      dataManager.storeSeedlingMonitorData(seedlingMonitorData);
      seedlingMonitoringDataNewData = true;
    }
    noteSeedlingPacket(seedlingMonitorData, ok);
    if (debug) Serial.print("received SeedlingMonitorData from ");
    if (debug) Serial.println(seedlingMonitorData.devicename);

  } else if (packetSize == sizeof(CommaRecord)) {
    memset(&commaRecord, 0, sizeof(CommaRecord));
    rxPacketRead((uint8_t*)&commaRecord, sizeof(CommaRecord));
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
    noteCommaPacket(commaRecord, checksumOk, rxPacketRssi(), rxPacketSnr());

  } else if (packetSize == sizeof(VitalSignsRecord)) {
    // Sent by each device right after its data pulse - only the latest per device is kept
    // (running totals), see VitalSignsStore.h. rssi/snr are Annabelle's own measurement of
    // this packet, kept outside the checksummed record.
    VitalSignsRecord vitalSignsRecord;
    rxPacketRead((uint8_t*)&vitalSignsRecord, sizeof(VitalSignsRecord));
    handleVitalSigns(vitalSignsRecord, validateChecksum(vitalSignsRecord));

  } else if (packetSize == sizeof(VitalSignsRecordV3)) {
    // Version 3 vital signs (89 bytes, no WiFi totals) from a device not yet reflashed with v4 -
    // upgraded (WiFi totals zero) and handled like any other record, weather reply timing included.
    VitalSignsRecordV3 v3;
    rxPacketRead((uint8_t*)&v3, sizeof(VitalSignsRecordV3));
    handleVitalSigns(vitalSignsFromV3(v3), validateChecksum(v3));

  } else if (packetSize == sizeof(DeviceIdentityRecord)) {
    // Product definition + running build, sent rarely (after a real reset, after
    // SetProductDefinition, daily) - only the latest per device is kept, see DeviceIdentityStore.h.
    DeviceIdentityRecord deviceIdentityRecord;
    rxPacketRead((uint8_t*)&deviceIdentityRecord, sizeof(DeviceIdentityRecord));
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
      deviceIdentityStore.store(deviceIdentityRecord, rxPacketRssi(), rxPacketSnr());
    }
    noteIdentityPacket(deviceIdentityRecord, checksumOk, rxPacketRssi(), rxPacketSnr());

  } else if (packetSize == sizeof(VitalSignsRecordV2)) {
    // Version 2 vital signs (69 bytes, no battery ledger) from a device not yet reflashed with v3 -
    // upgraded to the v3 layout (ledger zero, no temperature) and relayed like any other record.
    VitalSignsRecordV2 v2;
    rxPacketRead((uint8_t*)&v2, sizeof(VitalSignsRecordV2));
    bool checksumOk = validateChecksum(v2);
    if (debug) {
      Serial.print("VitalSigns v2 cksum=");
      Serial.println(checksumOk ? "ok" : "BAD");
    }
    if (checksumOk) {
      VitalSignsRecord upgraded = vitalSignsFromV2(v2);
      vitalSignsStore.store(upgraded, rxPacketRssi(), rxPacketSnr());
    }
    noteVitalSignsPacket(vitalSignsFromV2(v2), checksumOk, rxPacketRssi(), rxPacketSnr());

  } else if (packetSize == sizeof(PowerProfileRecord)) {
    // Battery/sensor parameters, sent after a real reset and daily - latest per device, see PowerProfileStore.h.
    PowerProfileRecord powerProfileRecord;
    rxPacketRead((uint8_t*)&powerProfileRecord, sizeof(PowerProfileRecord));
    bool checksumOk = validateChecksum(powerProfileRecord);
    if (debug) {
      Serial.print("PowerProfile capacity=");
      Serial.print(powerProfileRecord.capacity_mAh);
      Serial.print(" cksum=");
      Serial.println(checksumOk ? "ok" : "BAD");
    }
    if (checksumOk) {
      powerProfileStore.store(powerProfileRecord, rxPacketRssi(), rxPacketSnr());
    }
    notePowerProfilePacket(powerProfileRecord, checksumOk, rxPacketRssi(), rxPacketSnr());
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
  loraDio0AtMs = millis();
  loraDio0Fired = true;
  BaseType_t woken = pdFALSE;
  if (loraRxTaskHandle) vTaskNotifyGiveFromISR(loraRxTaskHandle, &woken);
  if (woken) portYIELD_FROM_ISR();
}

// Reads every packet out of the radio as soon as DIO0 rises (see rxSlots). Also the safety net
// for a stuck DIO0 (high with no interrupt pending - an IRQ flag left set, so it never rises
// again): checked every 20 ms.
void loraRxTask(void *arg) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
    if (!loraDio0Fired && digitalRead(LORA_DI0) == LOW) continue;
    uint32_t lockStartMs = millis();
    loraLock();  // waits while loop() is sending
    uint32_t lockWaitMs = millis() - lockStartMs;
    if (lockWaitMs > radioStats.lockWaitMsMax) radioStats.lockWaitMsMax = lockWaitMs;
    if (lockWaitMs > histLockWaitMaxMs) histLockWaitMaxMs = lockWaitMs;
    if (lockWaitMs > 1000) radioStats.lockWaitOver1s++;
    bool fired = loraDio0Fired;
    if (!fired && digitalRead(LORA_DI0) == LOW) {  // a send cleared it meanwhile
      loraUnlock();
      continue;
    }
    if (!fired) {
      radioStats.dio0Stuck++;
      loraDio0AtMs = millis();
    }
    loraDio0Fired = false;
    bool afterOwnTx = ownTxDone;
    ownTxDone = false;
    uint32_t parseMs = millis();
    if (!afterOwnTx) {
      uint32_t latencyMs = parseMs - loraDio0AtMs;
      radioStats.latencyMsTotal += latencyMs;
      if (latencyMs > radioStats.latencyMsMax) radioStats.latencyMsMax = latencyMs;
      if (latencyMs > 1000) radioStats.latencyOver1s++;
    }
    // Reads and clears the IRQ flags and points the FIFO at the packet; leaves the radio in standby
    // after a packet, or switches it to single RX when there was none (CRC error, TX done).
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
      radioStats.rxPackets++;
      uint8_t next = (rxHead + 1) % RX_SLOTS;
      if (next == rxTail) {
        radioStats.rxDropped++;  // loop() hasn't caught up - the packet is lost
      } else {
        RxPacket &p = rxSlots[rxHead];
        p.len = (uint8_t)min(packetSize, (int)sizeof(p.data));
        LoRa.readBytes(p.data, p.len);
        p.rssi = (int16_t)LoRa.packetRssi();
        p.snr = LoRa.packetSnr();
        rxHead = next;
      }
      LoRa_rxMode();  // straight back to listening
      uint32_t busyMs = millis() - parseMs;
      radioStats.busyMsTotal += busyMs;
      if (busyMs > radioStats.busyMsMax) radioStats.busyMsMax = busyMs;
      if (busyMs > 1000) radioStats.busyOver1s++;
    } else if (afterOwnTx) {
      LoRa_rxMode();  // our own TX-done - back to continuous receive
    } else {
      radioStats.noPacket++;
      CorruptPacket &c = corruptPackets[corruptPacketsHead];
      c.epoch = heardNow();
      c.rssi = (int16_t)LoRa.packetRssi();
      c.snrQ = (int8_t)constrain(lroundf(LoRa.packetSnr() * 4), -128, 127);
      corruptPacketsHead = (corruptPacketsHead + 1) % CORRUPT_PACKETS_KEPT;
      LoRa_rxMode();  // back to continuous receive
    }
    loraUnlock();
  }
}

//
// end of interrupt functions
//


void showLeds() {
  if (ledMutex) xSemaphoreTake(ledMutex, portMAX_DELAY);
  FastLED.show();
  if (ledMutex) xSemaphoreGive(ledMutex);
}

// Updates the 3 status LEDs (leds[0..2]) based on current system state.
// leds[3,5] are LoRa/weather activity LEDs; leds[4] is the Pi-AsyncData heartbeat.
// Status on the first 3 LEDs only (2026-10-22): on the WallyI2CDataVisualizer shield in use, the
// chain is broken after LED 3 (hardware), so LEDs 3-5 stay dark. The LED show still drives all 6.
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
  // LED 2 — Hypothalamus: time since the Pi last pulled data via AsyncData - green <2.5 min,
  // yellow <4 min, red otherwise
  if (secondsSinceLastAsyncData < 150) {
    leds[2] = CRGB(0, 255, 0);
  } else if (secondsSinceLastAsyncData < 240) {
    leds[2] = CRGB(255, 255, 0);
  } else {
    leds[2] = CRGB(255, 0, 0);
  }
  for (int i = 3; i < NUM_LEDS; i++) leds[i] = CRGB(0, 0, 0);
  showLeds();
}



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

  // opMode bit0 = USB programming jumper fitted (Daffodil usbJumperFitted()): the CP2104 USB chip
  // is powered and drains the battery in deep sleep, and the TPL5010 watchdog is disconnected.
  // Shown inverted in place of the "Sent by" line so the page still fits the OLED.
  if (digitalStablesData.opMode & 0x01) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.println(" !! USB JUMPER ON !! ");
    display.setTextColor(SSD1306_WHITE);
  } else if (isSentByArraySet(digitalStablesData.sentbyarray, sizeof(digitalStablesData.sentbyarray))) {
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
  display.print(lastRxRssi);
  display.print(" snr:");
  display.println(lastRxSnr);
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
    previousLoopStage = resetDiagLoopStage;
    previousUptimeKnown = true;
    resetDiagBootCount++;
    previousAllocFail = allocFail;
    previousAllocFail.task[sizeof(previousAllocFail.task) - 1] = 0;
  } else {
    resetDiagMagic = RESET_DIAG_MAGIC;
    resetDiagBootCount = 1;
    resetDiagLowMemRestarts = 0;
  }
  resetDiagUptimeSeconds = 0;
  resetDiagMinFreeHeap = ESP.getMinFreeHeap();
  memset(&allocFail, 0, sizeof(allocFail));
  heap_caps_register_failed_alloc_callback(allocFailed);

  char allocText[160];
  formatAllocFail(previousAllocFail, allocText, sizeof(allocText));
  char line[320];
  snprintf(line, sizeof(line), "%04d-%02d-%02d %02d:%02d:%02d reason=%s boot=%lu prevUptime=%s prevMinHeap=%s stage=%s%s",
           now.year, now.month, now.date, now.hour, now.minute, now.second,
           resetReason.c_str(), (unsigned long)resetDiagBootCount,
           previousUptimeKnown ? String(previousUptimeSeconds).c_str() : "?",
           previousUptimeKnown ? String(previousMinFreeHeap).c_str() : "?",
           previousUptimeKnown ? loopStageName(previousLoopStage) : "?", allocText);
  resetDiagLoopStage = 0;

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
  // The last failed memory request before the panic (allocFailed()), if there was one
  char allocText[160];
  formatAllocFail(previousAllocFail, allocText, sizeof(allocText));
  line += allocText;

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


void logClockChange(const RTCInfoRecord &before, const char *how) {
  char line[96];
  snprintf(line, sizeof(line), "clock %04d-%02d-%02d %02d:%02d:%02d -> %04d-%02d-%02d %02d:%02d:%02d (%s)",
           before.year, before.month, before.date, before.hour, before.minute, before.second,
           currentTimerRecord.year, currentTimerRecord.month, currentTimerRecord.date,
           currentTimerRecord.hour, currentTimerRecord.minute, currentTimerRecord.second, how);
  if (debug) Serial.println(line);
  File log = LittleFS.open(RESET_LOG_FILE, "a");
  if (log) {
    log.println(line);
    log.close();
  }
}

// Runs a settings request queued by queueWebAction(). Called from loop()'s tick block.
void processWebAction() {
  uint8_t action;
  char a[sizeof(webArgA)], b[sizeof(webArgB)];
  portENTER_CRITICAL(&webMux);
  action = webAction;
  memcpy(a, webArgA, sizeof(a));
  memcpy(b, webArgB, sizeof(b));
  portEXIT_CRITICAL(&webMux);
  if (action == WEB_NONE) return;

  RTCInfoRecord before = currentTimerRecord;
  if (action == WEB_SET_TIME) {
    // "YYYY-MM-DDTHH:MM[:SS]" from the page's datetime-local input, Annabelle's local time.
    struct tm t = {};
    int sec = 0;
    int got = sscanf(a, "%d-%d-%dT%d:%d:%d", &t.tm_year, &t.tm_mon, &t.tm_mday, &t.tm_hour, &t.tm_min, &sec);
    if (got < 5 || t.tm_year < 2024 || t.tm_mon < 1 || t.tm_mon > 12 || t.tm_mday < 1 || t.tm_mday > 31) {
      setWebResult(String("Not a valid time: ") + a);
    } else {
      t.tm_sec = sec;
      t.tm_year -= 1900;
      t.tm_mon -= 1;
      t.tm_isdst = -1;
      setenv("TZ", timezone.c_str(), 1);  // see syncRtcFromNtp()
      tzset();
      mktime(&t);  // fills in the weekday
      writeRtc(t);
      currentTimerRecord = timeManager.now();
      logClockChange(before, "web, manual");
      setWebResult("Clock set by hand");
    }
  } else if (action == WEB_SET_TIME_INTERNET) {
    setenv("TZ", timezone.c_str(), 1);
    tzset();
    struct tm ntp;
    if (!getLocalTime(&ntp, 0)) {
      setWebResult("No internet time yet - SNTP has not synced");
    } else {
      writeRtc(ntp);
      currentTimerRecord = timeManager.now();
      logClockChange(before, ntp.tm_isdst > 0 ? "web, NTP AEDT" : "web, NTP AEST");
      setWebResult("Clock set from internet time");
    }
  } else if (action == WEB_SEND_WEATHER) {
    // Picked up by the weatherPending check further down this same tick, which does the download
    // and the LoRa send - never from the async_tcp task (see loraDio0Fired).
    weatherTrigger = "web";
    weatherPending = true;
  } else if (action == WEB_DEVICE_WIFI) {
    // a = the device's serial as 16 hex digits, b = "on" or "off" (telepathons.js WiFi button)
    uint8_t serial[8];
    bool valid = strlen(a) == 16;
    for (int i = 0; valid && i < 8; i++) {
      char hex[3] = { a[i * 2], a[i * 2 + 1], 0 };
      char *end;
      serial[i] = (uint8_t)strtoul(hex, &end, 16);
      valid = *end == 0;
    }
    bool on = strcmp(b, "on") == 0;
    if (!valid || (!on && strcmp(b, "off") != 0)) {
      setWebResult(String("Not a valid WiFi request: ") + a + " " + b);
    } else {
      portENTER_CRITICAL(&heardMux);
      deviceCommandQueue.request(serial, on ? DEVICE_COMMAND_WIFI_ON : DEVICE_COMMAND_WIFI_OFF, 0, millis());
      portEXIT_CRITICAL(&heardMux);
      setWebResult(String("WiFi ") + b + " requested for " + a);
    }
  } else if (action == WEB_SET_GPS) {
    double lat = atof(a), lon = atof(b);
    if (lat < -90 || lat > 90 || lon < -180 || lon > 180 || (lat == 0 && lon == 0)) {
      setWebResult(String("Not a valid position: ") + a + ", " + b);
    } else {
      Preferences prefs;
      prefs.begin(CONFIG_PREFS, false);
      prefs.putDouble("lat", lat);
      prefs.putDouble("lon", lon);
      prefs.end();
      annabelleData.latitude = lat;
      annabelleData.longitude = lon;
      setWebResult("GPS saved - the weather forecast uses it after the next restart");
    }
  }
  portENTER_CRITICAL(&webMux);
  webAction = WEB_NONE;
  portEXIT_CRITICAL(&webMux);
}

// Annabelle's DS18B20 (also its serial number) as the board temperature: a conversion is started at
// second 15 and read at second 17, so loop() never waits the ~750 ms a blocking read takes.
void tickBoardTemperature(uint8_t second) {
  if (second == 15) {
    tempSensor.setWaitForConversion(false);
    tempSensor.requestTemperatures();
  } else if (second == 17) {
    float t = tempSensor.getTempCByIndex(0);
    boardTemperature = (t <= -127) ? NAN : t;  // -127 = disconnected (the sketchbook DallasTemperature has no DEVICE_DISCONNECTED_C)
  }
}

void setup() {
  annabelleStatus = {};  // Annabelle's status line starts "not sent" on every boot (RTC_NOINIT, see above)
  pinMode(WDT_DONE_PIN, OUTPUT);
  digitalWrite(WDT_DONE_PIN, LOW);
  xTaskCreatePinnedToCore(watchdogTask, "tplWatchdog", 2048, NULL, 2, &watchdogTaskHandle, 0);
  pinMode(LED_POWER_PIN, OUTPUT);
  digitalWrite(LED_POWER_PIN, HIGH);
  Serial.begin(115200);
  // readString() returns after this much silence. The Pi writes each command in one go; the default
  // 1000 ms made every Pi command cost loop() a full second (~10 a minute) - 2026-10-17.
  Serial.setTimeout(100);
  Wire.begin();

  pinMode(SLEEP_SWITCH_26, OUTPUT);
  digitalWrite(SLEEP_SWITCH_26, HIGH);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    if (debug) Serial.println(F("SSD1309 allocation failed"));
    for (;;)
      ;
  }
  // Start-up screen like Cleotilde's: the name large, then the firmware version and build time,
  // held 2 s so they can be read before the boot screens replace them.
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  {
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds("Annabelle", 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, 10);
    display.print("Annabelle");
  }
  centerText("Version " + String(firmwareVersion), 38);
  centerText(FIRMWARE_BUILD_TIME, 50);
  display.display();
  delay(2000);

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
  // A position saved from the web page (Settings > GPS) replaces the built-in one.
  {
    Preferences prefs;
    if (prefs.begin(CONFIG_PREFS, true)) {
      latitude = prefs.getDouble("lat", latitude);
      longitude = prefs.getDouble("lon", longitude);
      prefs.end();
    }
  }
  annabelleData.latitude = latitude;
  annabelleData.longitude = longitude;
  secret = "J5KFCNCPIRCTGT2UJUZFSMQK";
  String apiKey = "103df7bb3e4010e033d494f031b483e0";

  timezone = "AEST-10AEDT,M10.1.0,M4.1.0/3";
  setenv("TZ", timezone.c_str(), 1);
  tzset();

  pinMode(RTC_CLK_OUT, INPUT_PULLUP);
  tempSensor.begin();
  uint8_t address[8];
  if (tempSensor.getAddress(address, 0)) {
    // 2026-10-30: the serial number is the efuse MAC, as the factory records it, not this sensor's ID
    DeviceSerial::fill(annabelleData.serialnumberarray);
    serialNumber = DeviceSerial::toString(annabelleData.serialnumberarray);
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

  // strlcpy (2026-10-22): the old copy through a grp.length()-sized buffer had no room for the end
  // marker, so an empty group identifier left stack garbage in the field and broke GetWebData's JSON.
  String grp = secretManager.getGroupIdentifier();
  strlcpy(annabelleData.groupidentifier, grp.c_str(), sizeof(annabelleData.groupidentifier));

  String identifier = "AnnabelleV";
  char ty[identifier.length() + 1];
  identifier.toCharArray(ty, identifier.length() + 1);
  strcpy(annabelleData.deviceTypeId, ty);

  panchoConfigData.fieldId = secretManager.getFieldId();

  delay(1000);

  wifiManager.setTelepathonsProvider(fillTelepathonsJson);
  wifiManager.setHistoryProvider(fillTelepathonHistoryJson);
  wifiManager.setRadioHistoryProvider(fillRadioHistoryJson);
  wifiManager.setWebDataExtras(addWebDataExtras);
  wifiManager.setSettingsAction(queueWebAction);
  wifiManager.pingVerbose = debug;  // its ping lines went to the Pi's serial line
  wifiManager.start();
  bool stationmode = wifiManager.getStationMode();
  wifiStationWanted = stationmode && wifiManager.getSSID() != "";
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
    // Not LoRa.onReceive() - see loraDio0Fired. Packets are read by loraRxTask - see rxSlots.
    LoRa.receive();  // before the task starts - from here on only with loraMutex
    loraMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(loraRxTask, "loraRx", 4096, nullptr, 3, &loraRxTaskHandle, 1);
    attachInterrupt(digitalPinToInterrupt(LORA_DI0), onLoraDio0, RISING);
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

// Pets the TPL5010 (see WDT_DONE_PIN). Started first thing in setup() so a slow boot isn't reset.
void watchdogTask(void *arg) {
  for (;;) {
    uint32_t aliveMs = loopAliveMs;  // read before millis() so the difference can't wrap
    uint32_t nowMs = millis();
    bool alive = aliveMs ? nowMs - aliveMs < WDT_LOOP_STALL_MS : nowMs < WDT_SETUP_LIMIT_MS;
    if (alive) {
      digitalWrite(WDT_DONE_PIN, HIGH);
      delayMicroseconds(100);
      digitalWrite(WDT_DONE_PIN, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// WiFi watchdog (2026-10-17). After a router outage Annabelle stayed off the house WiFi until it
// was reset by hand (memory down 44 KB, unreachable, LoRa still flowing to the Pi). Once a second:
// while a station connection is wanted but missing, WiFi.reconnect() every minute; after
// WIFI_RESTART_AFTER_SEC without WiFi - or that long in the AP fallback because the network wasn't
// there at boot - restart, which joins the network the normal way. The data queue for the Pi is
// in flash, so a restart loses only what is in RAM.
#define WIFI_RESTART_AFTER_SEC (15UL * 60UL)
void wifiWatchdogTick() {
  static uint32_t downSec = 0;
  if (!wifiStationWanted) return;
  if (WiFi.status() == WL_CONNECTED) {
    downSec = 0;
    return;
  }
  downSec++;
  if (downSec > radioStats.wifiDownSecMax) radioStats.wifiDownSecMax = downSec;
  if (downSec % 60 == 30 && (WiFi.getMode() & WIFI_MODE_STA)) {
    radioStats.wifiReconnects++;
    WiFi.reconnect();  // returns at once; the connection happens in the background
  }
  if (downSec >= WIFI_RESTART_AFTER_SEC) {
    LOOP_STAGE(STAGE_WIFI_RESTART);  // reported as the reset stage after the restart
    if (debug) Serial.println("WiFi gone for 15 min - restarting");
    delay(100);
    ESP.restart();
  }
}

// Low-memory watchdog (2026-10-29). On 10-23 and twice on 10-28 Annabelle panicked out of memory
// (LittleFS.exists() in loop(), then the async_tcp accept). Once a second: when the largest free
// block has stayed under WEB_MIN_FREE_BLOCK (the web gate's limit) for LOWMEM_RESTART_AFTER_SEC,
// restart before it crashes. Reported as reason SW, stage "restart: low memory", and counted in
// resetDiagLowMemRestarts (lowMemRestarts on /Telepathons).
#define LOWMEM_RESTART_AFTER_SEC (10UL * 60UL)
void lowMemoryWatchdogTick() {
  static uint32_t lowSec = 0;
  if (heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) >= WEB_MIN_FREE_BLOCK) {
    lowSec = 0;
    return;
  }
  if (++lowSec >= LOWMEM_RESTART_AFTER_SEC) {
    LOOP_STAGE(STAGE_LOWMEM_RESTART);  // reported as the reset stage after the restart
    resetDiagLowMemRestarts++;
    if (debug) Serial.println("Low memory for 10 min - restarting");
    delay(100);
    ESP.restart();
  }
}

// One RadioHistoryRow every RADIO_HISTORY_MS: the change in radioStats since the last one.
void radioHistoryTick() {
  static uint32_t lastMs = 0;
  static RadioStats last = {};
  uint32_t now = millis();
  if (lastMs != 0 && now - lastMs < RADIO_HISTORY_MS) return;
  if (lastMs == 0) {  // first call: just take the baseline
    if (radioHistoryMagic != RADIO_HISTORY_MAGIC || radioHistoryHead >= RADIO_HISTORY_SLOTS || radioHistoryCount > RADIO_HISTORY_SLOTS) {
      radioHistoryHead = 0;
      radioHistoryCount = 0;
      radioHistoryMagic = RADIO_HISTORY_MAGIC;
    }
    lastMs = now;
    last = radioStats;
    return;
  }
  lastMs = now;
  RadioStats cur = radioStats;
  RadioHistoryRow &h = radioHistory[radioHistoryHead];
  h.epoch = heardNow();
  h.heapFree16 = (uint16_t)min((uint32_t)65535, ESP.getFreeHeap() / 16);
  h.heapBlock16 = (uint16_t)min((uint32_t)65535, (uint32_t)(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 16));
  uint32_t rx = cur.rxPackets - last.rxPackets, np = cur.noPacket - last.noPacket;
  h.rx = (uint16_t)min(rx, (uint32_t)65535);
  h.noPacket = (uint16_t)min(np, (uint32_t)65535);
  h.late = (uint16_t)min(cur.latencyOver1s - last.latencyOver1s, (uint32_t)65535);
  h.latencyAvgMs = (uint16_t)min(rx + np ? (cur.latencyMsTotal - last.latencyMsTotal) / (rx + np) : 0, (uint32_t)65535);
  h.loopMaxMs = (uint16_t)min(histLoopMaxMs, (uint32_t)65535);
  h.lockWaitMaxMs = (uint16_t)min(histLockWaitMaxMs, (uint32_t)65535);
  histLoopMaxMs = 0;
  histLockWaitMaxMs = 0;
  radioHistoryHead = (radioHistoryHead + 1) % RADIO_HISTORY_SLOTS;
  if (radioHistoryCount < RADIO_HISTORY_SLOTS) radioHistoryCount++;
  last = cur;
}

// /RadioHistory JSON, oldest first: rows of [epoch, heapFree, heapBlock, rx, corrupted, late,
// latencyAvgMs, loopMaxMs, lockWaitMaxMs] - heap values in bytes.
void fillRadioHistoryJson(JsonDocument &json) {
  json["periodSec"] = RADIO_HISTORY_MS / 1000;
  json["now"] = heardNow();
  JsonArray rows = json["rows"].to<JsonArray>();
  for (int i = radioHistoryCount; i >= 1; i--) {
    const RadioHistoryRow &h = radioHistory[(radioHistoryHead + RADIO_HISTORY_SLOTS - i) % RADIO_HISTORY_SLOTS];
    JsonArray r = rows.add<JsonArray>();
    r.add(h.epoch);
    r.add((uint32_t)h.heapFree16 * 16);
    r.add((uint32_t)h.heapBlock16 * 16);
    r.add(h.rx);
    r.add(h.noPacket);
    r.add(h.late);
    r.add(h.latencyAvgMs);
    r.add(h.loopMaxMs);
    r.add(h.lockWaitMaxMs);
  }
}

void loop() {
  int hour, minute, second;
  loopAliveMs = millis();
  {
    static uint32_t lastLoopMs = 0;
    uint32_t nowMs = millis();
    if (lastLoopMs != 0) {
      uint32_t loopMs = nowMs - lastLoopMs;
      if (loopMs > radioStats.loopMsMax) radioStats.loopMsMax = loopMs;
      if (loopMs > histLoopMaxMs) histLoopMaxMs = loopMs;
      if (loopMs > 1000) radioStats.loopOver1s++;
    }
    lastLoopMs = nowMs;
  }
  radioHistoryTick();
  LOOP_STAGE(STAGE_TOP);
  if (!fragBootBlock) fragBootBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

  if (clockTicked) {
    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);
    LOOP_STAGE(STAGE_CLOCK);
    wifiWatchdogTick();
    lowMemoryWatchdogTick();

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
    fragBegin(STAGE_WEB_ACTION);
    processWebAction();
    fragEnd(STAGE_WEB_ACTION);
    LOOP_STAGE(STAGE_CLOCK);
    tickBoardTemperature(second);
    if (weatherScreenShown) {  // live clock on the weather page
      display.fillRect(0, 0, SCREEN_WIDTH, 8, SSD1306_BLACK);
      drawWeatherTitle();
      display.display();
    }

    // Set pending flag at trigger time; actual work deferred until serial is free
    if (minute % 5 == 0 && second == 0) {
      weatherPending = true;
      weatherTrigger = "timer";
    }

    if (weatherPending && Serial.available() == 0 && !inSerial) {
      weatherPending = false;
      fragBegin(STAGE_WEATHER_DOWNLOAD);
      sendWeatherForecast(1, weatherTrigger);
      fragEnd(STAGE_WEATHER_DOWNLOAD);
    }

    updateStatusLeds();  // refresh status LEDs every second
  }

  if (!loraReceived && rxTail != rxHead) {
    rxCurrent = &rxSlots[rxTail];
    rxCurrentPos = 0;
    loraPacketSize = rxCurrent->len;
    loraReceived = true;
  }

  if (loraReceived) {
    if (debug) Serial.printf("lora recive loraPacketSize: %d \n", loraPacketSize);
    if (debug) Serial.println("");
    fragBegin(STAGE_LORA);
    processLora(loraPacketSize);
    fragEnd(STAGE_LORA);
    lastRxRssi = rxPacketRssi();
    lastRxSnr = rxPacketSnr();
    rxCurrent = nullptr;
    rxTail = (rxTail + 1) % RX_SLOTS;  // slot free for loraRxTask again
    loraReceived = false;
    // A VitalSigns (and sometimes a DeviceIdentity) packet follows a data pulse - keep showing
    // that device's data page instead of switching to "Unidentified".
    if (loraPacketSize != sizeof(VitalSignsRecord) && loraPacketSize != sizeof(DeviceIdentityRecord) &&
        loraPacketSize != sizeof(VitalSignsRecordV2) && loraPacketSize != sizeof(VitalSignsRecordV3) &&
        loraPacketSize != sizeof(PowerProfileRecord)) lastReceivedPacketSize = loraPacketSize;

    currentPalette = loraReceivePalette;
    currentBlending = LINEARBLEND;
    ledShowDuration = 2000;
    runLedShow = true;

    showLastReceivedData();
  }
  loraPacketSize = 0;

  // Device commands first: a sleeping Daffodil listens for both in the same window after its
  // VitalSigns, and stops listening once the forecast it was waiting for arrives.
  {
    DeviceCommandRecord command;
    portENTER_CRITICAL(&heardMux);
    bool haveCommand = deviceCommandQueue.takeDue(millis(), command);
    portEXIT_CRITICAL(&heardMux);
    if (haveCommand) {
      fragBegin(STAGE_COMMAND_TX);
      int result = sendMessage(command);
      fragEnd(STAGE_COMMAND_TX);
      if (debug) Serial.printf("device command %u sent, lora=%d\n", command.command, result);
    }
  }

  if (weatherReplyPending && (long)(millis() - weatherReplyDueMs) >= 0) {
    weatherReplyPending = false;
    fragBegin(STAGE_WEATHER_REPLY);
    sendWeatherReply();
    fragEnd(STAGE_WEATHER_REPLY);
  }

  if (Serial.available() != 0) {
    fragBegin(STAGE_SERIAL);
    inSerial = true;
    String command = Serial.readString();
    if (command.startsWith("AsyncData") && !command.startsWith("AsyncDataCount")) LOOP_STAGE(STAGE_ASYNCDATA);

    if (command.startsWith("getWF")) {
      sendWeatherForecast(3, "Pi");
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
    } else if (command.startsWith("GetFirmware")) {
      Serial.println("Version " + String(firmwareVersion) + " " FIRMWARE_BUILD_TIME);
      Serial.println(F("Ok-GetFirmware"));
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
      // + 1 when Annabelle's own status line will go too (pushAnnabelleStatusToSerial, 2026-11-01)
      Serial.println(dataManager.getPendingQueueItemCount() + vitalSignsStore.pendingCount() + deviceIdentityStore.pendingCount() + powerProfileStore.pendingCount()
                     + (annabelleStatusDue() ? 1 : 0));
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
      Serial.print(",0,-1");
      Serial.print("|Power="); Serial.print(powerProfileStore.pendingCount());
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
      int powerDownloaded = powerProfileStore.pushPendingToSerial(Serial);
      int statusDownloaded = pushAnnabelleStatusToSerial(Serial);

      Serial.print("Ok-AsyncData#");
      Serial.print("DS="); Serial.print(dsDownloaded);
      Serial.print("|Gloria="); Serial.print(gloriaDownloaded);
      Serial.print("|Seedling="); Serial.print(seedlingDownloaded);
      Serial.print("|Chinampa="); Serial.print(chinampaDownloaded);
      Serial.print("|Comma="); Serial.print(commaDownloaded);
      Serial.print("|Langley="); Serial.print(langleyDownloaded);
      Serial.print("|Vital="); Serial.print(vitalDownloaded);
      Serial.print("|Identity="); Serial.print(identityDownloaded);
      Serial.print("|Power="); Serial.print(powerDownloaded);
      Serial.print("|Annabelle="); Serial.print(statusDownloaded);
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
    loraLock();
    LoRa_rxMode();
    loraUnlock();
    inSerial = false;
    fragEnd(resetDiagLoopStage == STAGE_ASYNCDATA ? STAGE_ASYNCDATA : STAGE_SERIAL);
  }
}

#ifndef ANNABELLE_HEARD_ENTRY_H
#define ANNABELLE_HEARD_ENTRY_H
#include <Arduino.h>

// One row of Annabelle's last-heard table (see Annabelle.ino, /Telepathons). In its own header
// because the IDE inserts its auto-prototypes above the first function in the sketch - a struct
// defined in the .ino after that point is unknown to the prototypes that use it.
#define MAX_HEARD 24
#define HEARD_HISTORY 15   // cards kept per row, newest replaces oldest (16 until 2026-11-10: static RAM was full)
#define DAFFODIL_SENSORS 7   // board temperature, outdoor temperature and humidity + up to 4 function readings
#define CARD_VALUES 9   // Daffodil and VitalSigns cards send 9 - noteSync() reads v[8], the Daffodil clock offset

// Relays seen in packets' sentbyarray (2026-11-11) - any device may relay (Chinampa does, Paula when
// testing), so their names are learnt, not assumed. HeardCard.via 1..RELAY_NAMES_MAX picks one;
// RELAY_VIA_UNNAMED = a relay whose name didn't fit the table.
#define RELAY_NAMES_MAX 8
#define RELAY_VIA_UNNAMED (RELAY_NAMES_MAX + 1)

// Packet types on a card - telepathons.js has the matching labels (CARD_TYPES).
enum HeardCardType : uint8_t {
  CARD_DAFFODIL = 1, CARD_CHINAMPA, CARD_LANGLEY, CARD_GLORIA, CARD_SEEDLING,
  CARD_VITAL, CARD_IDENTITY, CARD_COMMA, CARD_POWER, CARD_LINK  // CARD_LINK: LinkHealthRecord (2026-11-10)
};

// One received packet: when, how it was heard, whether it checked out, and up to CARD_VALUES
// readings (meaning depends on type, NAN = not sent).
struct HeardCard {
  uint32_t epoch;   // Annabelle RTC
  uint8_t type;     // HeardCardType
  bool ok : 1;      // checksum matched, or (types without one) name and clock look sane
  // Who resent it (2026-11-11), from the packet's sentbyarray: 0 = empty sentby (sent by the device
  // itself), else 1 + its index in Annabelle.ino's relayNames. Packed into ok's byte - no RAM added
  // (the cards fill static RAM). Only DigitalStablesData and ChinampaData have a sentbyarray.
  uint8_t via : 7;
  int16_t rssi;
  float snr;
  float v[CARD_VALUES];
};

struct HeardEntry {
  char name[17];
  uint8_t serial[8];
  char kind[12];
  uint16_t gen;          // +1 each time the row is reused for another device - the page drops stale history
  uint32_t lastEpoch;    // Annabelle RTC, last packet of any type
  uint32_t okCount;      // packets that checked out
  uint32_t badCount;     // packets that did not
  int16_t rssi;
  float snr;
  // Daffodils only, from their data packets (Annabelle.ino noteSync): did Annabelle's
  // WeatherForecastUpdate reach them - opMode bit 1, set while the device holds a forecast less
  // than 31 min old - and did their clock get corrected from it (offset jumping towards zero).
  bool weatherSeen;      // at least one data packet since the row was made
  bool weatherFresh;     // latest data packet says fresh
  uint32_t weatherFreshSince;  // Annabelle RTC, first packet of the current fresh run
  uint32_t weatherLastFresh;   // last packet that said fresh
  bool offsetKnown;
  float lastOffset;            // device clock minus real time, s
  uint32_t clockCorrectedAt;   // Annabelle RTC, packet where the offset jumped towards zero
  float clockCorrectedBy;      // offset before that jump, s
  bool wifiKnown;              // Daffodils: at least one data packet reported its WiFi
  uint8_t wifiStatus;          // from the latest one: 0 off, 1 own hotspot, 2 router, 3 router + internet
  // Daffodils only: the latest data packet's function (config switch, FUN_1_FLOW...) and the sensor
  // readings that function uses (Annabelle.ino daffodilSensors) - board temperature, outdoor temperature
  // and humidity first. Kept
  // here, not on the cards, so the 16 cards per row stay 9 values each.
  int8_t function;
  float sensors[DAFFODIL_SENSORS];
  // Link health (2026-11-10): data packets heard from this device, and that count and the device's
  // own dataSent at its previous LinkHealthRecord - the delivery ratio is heard / sent between two.
  // 16-bit like the device's counters (unsigned differences survive wrap-around).
  uint16_t dataHeard;
  uint16_t linkDataHeardAt;
  uint16_t linkSentAt;
  bool linkKnown;
  uint8_t histHead;      // next slot to write in history
  uint8_t histLen;
  HeardCard history[HEARD_HISTORY];
};

#endif

#ifndef ANNABELLE_ROUTE_ENTRY_H
#define ANNABELLE_ROUTE_ENTRY_H
#include <Arduino.h>
#include <DigitalStablesData.h>
#include <ChinampaData.h>
#include <LangleyData.h>

// Routes (2026-11-11) - see Annabelle.ino's "Routes" section. In its own header for the same reason
// as HeardEntry.h: the IDE's auto-prototypes go above the first function, before any struct defined
// in the sketch.
#define ROUTE_WINDOW_MS 4000
#define ROUTE_PENDING 4
#define ROUTE_DONE 16  // the Pi asks every few seconds - 16 readings is plenty (heap, ~2 KB in all)
#define ROUTE_LIST 40
#define ROUTE_KIND_DSD 1
#define ROUTE_KIND_CHINAMPA 2
#define ROUTE_KIND_LANGLEY 3
#define ROUTE_BYTES_MAX (sizeof(ChinampaData) > sizeof(DigitalStablesData) ? (sizeof(ChinampaData) > sizeof(LangleyData) ? sizeof(ChinampaData) : sizeof(LangleyData)) : (sizeof(DigitalStablesData) > sizeof(LangleyData) ? sizeof(DigitalStablesData) : sizeof(LangleyData)))
struct RoutePending {
  uint8_t kind;  // ROUTE_KIND_*, 0 = free
  uint32_t firstMs;
  uint8_t serial[8];
  long secondsTime;
  char routes[ROUTE_LIST];
  uint8_t bytes[ROUTE_BYTES_MAX];  // the first copy, rssi/snr as Annabelle heard it
};
struct RouteDone {
  uint8_t serial[8];
  long secondsTime;
  char routes[ROUTE_LIST];
};
#endif

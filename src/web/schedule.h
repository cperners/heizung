#pragma once
#include <stdint.h>
// Strict HH:MM parsing; no decimal-hour interpretation.
inline bool parseScheduleTime(const char* text, uint16_t& minutes) {
  if (!text) return false;
  unsigned length = 0;
  while (length < 6 && text[length]) ++length;
  if (length != 5 || text[2] != ':') return false;
  for (unsigned i=0; i<5; ++i)
    if (i != 2 && (text[i] < '0' || text[i] > '9')) return false;
  const unsigned h = (text[0]-'0')*10 + text[1]-'0';
  const unsigned m = (text[3]-'0')*10 + text[4]-'0';
  if (h > 23 || m > 59) return false;
  minutes = h*60 + m;
  return true;
}

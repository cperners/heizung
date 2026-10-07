#include "ntp_diagnostics.h"
#include "../config.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <sys/time.h>
#include <math.h>
struct NtpProbeResult { double offsetMs; double delayMs; unsigned long measuredAt; int state; };
NtpProbeResult ntpProbeResults[3] = {};
portMUX_TYPE ntpProbeMux = portMUX_INITIALIZER_UNLOCKED;
const char* ntpProbeServers[3] = {NTP_SERVER1, NTP_SERVER2, NTP_SERVER3};
static double ntpProbeNow() {
  timeval tv; gettimeofday(&tv, nullptr);
  return double(tv.tv_sec) + double(tv.tv_usec) / 1000000.0;
}
static uint32_t ntpProbeWord(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}
static double ntpProbeStamp(const uint8_t* p) {
  return double(ntpProbeWord(p)) - 2208988800.0 + double(ntpProbeWord(p+4)) / 4294967296.0;
}
void ntpProbeTask(void*) {
  WiFiUDP udp;
  for (;;) {
    for (int i = 0; i < 3; ++i) {
      NtpProbeResult result = {};
      result.state = 1; // No usable reply.
      if (WiFi.status() == WL_CONNECTED && time(nullptr) > 1700000000) {
        IPAddress address;
        if (WiFi.hostByName(ntpProbeServers[i], address) && udp.begin(0)) {
          uint8_t request[48] = {}; request[0] = 0x23;
          double t1 = ntpProbeNow();
          uint32_t seconds = uint32_t(t1 + 2208988800.0);
          uint32_t fraction = uint32_t((t1 - floor(t1)) * 4294967296.0);
          for (int b=0; b<4; ++b) { request[40+b] = seconds >> (24-8*b); request[44+b] = fraction >> (24-8*b); }
          udp.beginPacket(address, 123); udp.write(request, sizeof(request));
          if (udp.endPacket()) {
            unsigned long start = millis();
            while (millis() - start < 2000UL) {
              int size = udp.parsePacket();
              if (size >= 48) {
                const double t4 = ntpProbeNow(); uint8_t response[48];
                int count = udp.read(response, sizeof(response));
                if (count == 48 && udp.remoteIP() == address && udp.remotePort() == 123 &&
                    (response[0] & 7) == 4 && (response[0] >> 6) != 3 && response[1] > 0 && response[1] < 16 &&
                    memcmp(response+24, request+40, 8) == 0) {
                  const double t2 = ntpProbeStamp(response+32), t3 = ntpProbeStamp(response+40);
                  const double elapsed = (millis()-start)/1000.0;
                  const double delay = (t4-t1)-(t3-t2);
                  if (t2 > 1700000000 && t3 >= t2 && fabs((t4-t1)-elapsed) < 0.1 && delay >= -0.001 && delay < 2.0) {
                    result.offsetMs = ((t2-t1)+(t3-t4))*500.0;
                    result.delayMs = fmax(0.0, delay*1000.0); result.state = 2; result.measuredAt = millis();
                    break;
                  }
                }
              }
              vTaskDelay(pdMS_TO_TICKS(10));
            }
          }
          udp.stop();
        }
      } else result.state = 0;
      portENTER_CRITICAL(&ntpProbeMux); ntpProbeResults[i] = result; portEXIT_CRITICAL(&ntpProbeMux);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}
String ntpProbeDisplay() {
  NtpProbeResult snapshot[3];
  portENTER_CRITICAL(&ntpProbeMux); memcpy(snapshot, ntpProbeResults, sizeof(snapshot)); portEXIT_CRITICAL(&ntpProbeMux);
  String text;
  for (int i=0; i<3; ++i) {
    text += ntpProbeServers[i]; text += ": ";
    if (snapshot[i].state == 2) {
      char line[120]; snprintf(line, sizeof(line), "%+.1f ms (Laufzeit %.1f ms, vor %lu s)", snapshot[i].offsetMs, snapshot[i].delayMs, (millis()-snapshot[i].measuredAt)/1000UL); text += line;
    } else text += snapshot[i].state == 1 ? "Keine gueltige Antwort" : "Warte auf WLAN und gueltige Zeit";
    text += "\n";
  }
  return text;
}


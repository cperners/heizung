#include "cpu_load.h"
#include <Arduino.h>
#include <esp_freertos_hooks.h>

namespace {
// Coarse tick sampling: a tick counts as busy if no idle hook ran since
// the previous tick. Partial-tick activity is not measured as exact CPU time.
portMUX_TYPE cpuLoadMux = portMUX_INITIALIZER_UNLOCKED;
bool cpuIdleSeen[2] = {};
uint32_t cpuSampleTicks[2] = {}, cpuBusyTicks[2] = {};
bool cpuLoadReady = false;
char cpuLoadText[80] = "Messung startet";
void IRAM_ATTR sampleCpuTick(unsigned core) {
  portENTER_CRITICAL_ISR(&cpuLoadMux);
  ++cpuSampleTicks[core];
  if (!cpuIdleSeen[core]) ++cpuBusyTicks[core];
  cpuIdleSeen[core] = false;
  portEXIT_CRITICAL_ISR(&cpuLoadMux);
}
void IRAM_ATTR cpuTick0() { sampleCpuTick(0); }
void IRAM_ATTR cpuTick1() { sampleCpuTick(1); }
bool cpuIdle0() {
  portENTER_CRITICAL(&cpuLoadMux); cpuIdleSeen[0] = true; portEXIT_CRITICAL(&cpuLoadMux);
  return true;
}
bool cpuIdle1() {
  portENTER_CRITICAL(&cpuLoadMux); cpuIdleSeen[1] = true; portEXIT_CRITICAL(&cpuLoadMux);
  return true;
}
}
void updateCpuLoad() {
  static unsigned long last = millis();
  const unsigned long now = millis();
  if (!cpuLoadReady || now-last < 5000UL) return;
  last = now;
  uint32_t total[2], busy[2];
  portENTER_CRITICAL(&cpuLoadMux);
  for (unsigned i=0; i<2; ++i) {
    total[i]=cpuSampleTicks[i]; busy[i]=cpuBusyTicks[i];
    cpuSampleTicks[i]=cpuBusyTicks[i]=0;
  }
  portEXIT_CRITICAL(&cpuLoadMux);
  if (!total[0] || !total[1]) { snprintf(cpuLoadText,sizeof(cpuLoadText),"Nicht verfuegbar"); return; }
  snprintf(cpuLoadText,sizeof(cpuLoadText),"Kern 0: ~%.0f %% / Kern 1: ~%.0f %%",
           100.0*busy[0]/total[0],100.0*busy[1]/total[1]);
}


void beginCpuLoad() {
  const bool idle0 = esp_register_freertos_idle_hook_for_cpu(cpuIdle0,0) == ESP_OK;
  const bool idle1 = esp_register_freertos_idle_hook_for_cpu(cpuIdle1,1) == ESP_OK;
  const bool tick0 = esp_register_freertos_tick_hook_for_cpu(cpuTick0,0) == ESP_OK;
  const bool tick1 = esp_register_freertos_tick_hook_for_cpu(cpuTick1,1) == ESP_OK;
  cpuLoadReady = idle0 && idle1 && tick0 && tick1;
  if (!cpuLoadReady) {
    esp_deregister_freertos_idle_hook(cpuIdle0); esp_deregister_freertos_idle_hook(cpuIdle1);
    esp_deregister_freertos_tick_hook(cpuTick0); esp_deregister_freertos_tick_hook(cpuTick1);
    snprintf(cpuLoadText,sizeof(cpuLoadText),"Nicht verfuegbar");
  }
}
const char* cpuLoadDisplay() { return cpuLoadText; }

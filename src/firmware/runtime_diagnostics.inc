// Sample in the main loop only; web callbacks read the existing status snapshot.
struct RuntimeDiagnostics {
  uint32_t freeHeap = 0;
  uint32_t minimumHeap = 0;
  uint32_t largestBlock = 0;
  uint64_t recentLoopMaxUs = 0;
  uint64_t maximumLoopUs = 0;
  uint32_t slowLoopCount = 0;
};
RuntimeDiagnostics runtimeDiagnostics;

void updateRuntimeDiagnostics() {
  static int64_t previousStart = 0;
  static int64_t lastSample = 0;
  static uint64_t windowMaxUs = 0;
  const int64_t now = esp_timer_get_time();
  if (previousStart != 0) {
    const uint64_t interval = static_cast<uint64_t>(now - previousStart);
    if (interval > windowMaxUs) windowMaxUs = interval;
    if (interval > runtimeDiagnostics.maximumLoopUs) runtimeDiagnostics.maximumLoopUs = interval;
    if (interval > 100000ULL && runtimeDiagnostics.slowLoopCount != UINT32_MAX)
      ++runtimeDiagnostics.slowLoopCount;
  }
  previousStart = now;
  if (lastSample != 0 && now - lastSample < 1000000LL) return;
  lastSample = now;
  runtimeDiagnostics.recentLoopMaxUs = windowMaxUs;
  windowMaxUs = 0;
  // All three figures use the same 8-bit-addressable heap capability.
  runtimeDiagnostics.freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  runtimeDiagnostics.minimumHeap = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  runtimeDiagnostics.largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
}

void addRuntimeDiagnostics(JsonDocument& doc) {
  JsonObject health = doc["diagnostics"].to<JsonObject>();
  health["heapFreeBytes"] = runtimeDiagnostics.freeHeap;
  health["heapMinBytes"] = runtimeDiagnostics.minimumHeap;
  health["heapLargestBlockBytes"] = runtimeDiagnostics.largestBlock;
  health["loopRecentMaxMs"] = runtimeDiagnostics.recentLoopMaxUs / 1000.0;
  health["loopMaxMs"] = runtimeDiagnostics.maximumLoopUs / 1000.0;
  health["loopOver100ms"] = runtimeDiagnostics.slowLoopCount;
}

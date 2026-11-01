// Only the main loop/setup writes outputs and owns the bounded event queue.
// MQTT callbacks touch ACK/connection markers under relayAckMux only.
struct RelayEvent {
  uint64_t uptimeMs;
  int64_t unixMs; // 0 until system clock is synchronized
  uint32_t sequence;
  uint8_t states, changed, kind;
};
static constexpr uint16_t RELAY_EVENT_CAPACITY = 128;
static RelayEvent relayEvents[RELAY_EVENT_CAPACITY];
static uint16_t relayEventHead=0, relayEventCount=0;
static uint32_t relaySequence=0, relayLost=0, relayBootId=0;
static uint8_t relayStates=0;
static bool relayEventsReady=false, relayResyncNeeded=false;
static portMUX_TYPE relayAckMux=portMUX_INITIALIZER_UNLOCKED;
static uint16_t relayTrackedPacket=0, relayAckedPacket=0;
static uint32_t relayConnectionEpoch=0;
static uint16_t relayInFlight=0;
static uint32_t relaySentEpoch=0, relaySeenEpoch=UINT32_MAX;
static uint64_t relayRetryAfter=0;
static const uint8_t relayPins[]={BrennerPin,HeizungPin,BoilerPin,MischerAuf_Pin,MischerZu_Pin};

static bool queueRelayEvent(uint8_t changed, uint8_t kind) {
  const uint32_t sequence=++relaySequence;
  if (relayEventCount == RELAY_EVENT_CAPACITY) {
    ++relayLost;
    relayResyncNeeded=true;
    return false;
  }
  struct timeval tv;
  gettimeofday(&tv,nullptr);
  RelayEvent& e=relayEvents[(relayEventHead+relayEventCount)%RELAY_EVENT_CAPACITY];
  e.uptimeMs=static_cast<uint64_t>(esp_timer_get_time())/1000;
  e.unixMs=tv.tv_sec >= 1700000000 ? static_cast<int64_t>(tv.tv_sec)*1000+tv.tv_usec/1000 : 0;
  e.sequence=sequence; e.states=relayStates; e.changed=changed; e.kind=kind;
  ++relayEventCount;
  return true;
}

void beginRelayEvents() {
  relayBootId=esp_random();
  for (uint8_t i=0;i<5;++i) if (digitalRead(relayPins[i])) relayStates|=1U<<i;
  relayEventsReady=true;
  queueRelayEvent(0,0); // startup snapshot, not a switching event
}

void writeRelayOutput(uint8_t pin, uint8_t value) {
  digitalWrite(pin,value); // never wait for network, ACK or storage here
  if (!relayEventsReady) return;
  for (uint8_t i=0;i<5;++i) {
    if (relayPins[i]!=pin) continue;
    const uint8_t bit=1U<<i;
    const uint8_t next=value ? relayStates|bit : relayStates&~bit;
    if (next!=relayStates) {
      relayStates=next;
      queueRelayEvent(bit,1);
    }
    return;
  }
}

void onRelayEventPublished(uint16_t packetId) {
  portENTER_CRITICAL(&relayAckMux);
  // Also remember a very early ACK arriving before publish() returns its ID.
  if (!relayTrackedPacket || packetId==relayTrackedPacket) relayAckedPacket=packetId;
  portEXIT_CRITICAL(&relayAckMux);
}

void relayEventsDisconnected() {
  portENTER_CRITICAL(&relayAckMux);
  ++relayConnectionEpoch;
  relayTrackedPacket=relayAckedPacket=0;
  portEXIT_CRITICAL(&relayAckMux);
}

void serviceRelayEvents() {
  const uint64_t now=static_cast<uint64_t>(esp_timer_get_time())/1000;
  uint32_t epoch;
  uint16_t acknowledged;
  portENTER_CRITICAL(&relayAckMux);
  epoch=relayConnectionEpoch; acknowledged=relayAckedPacket;
  portEXIT_CRITICAL(&relayAckMux);
  if (relayInFlight) {
    if (epoch!=relaySentEpoch) {
      relayInFlight=0; // keep queue head for replay after reconnect
    } else if (acknowledged==relayInFlight) {
      relayEventHead=(relayEventHead+1)%RELAY_EVENT_CAPACITY;
      --relayEventCount;
      relayInFlight=0;
      portENTER_CRITICAL(&relayAckMux);
      relayTrackedPacket=relayAckedPacket=0;
      portEXIT_CRITICAL(&relayAckMux);
    } else return; // one unacknowledged event maximum, no unbounded MQTT backlog
  }
  if (relayResyncNeeded && relayEventCount<RELAY_EVENT_CAPACITY) {
    relayResyncNeeded=false;
    queueRelayEvent(0,3); // explicit current snapshot after a queue overflow
  }
  if (mqttSettingsAreChanging() || !asyncMqttClient.connected()) return;
  if (relaySeenEpoch!=epoch) {
    // A full queue already contains all older records; its overflow recovery
    // will provide the current snapshot once there is room.
    if (relayEventCount<RELAY_EVENT_CAPACITY) queueRelayEvent(0,2);
    else relayResyncNeeded=true;
    relaySeenEpoch=epoch;
  }
  if (!relayEventCount || now<relayRetryAfter) return;
  const RelayEvent& e=relayEvents[relayEventHead];
  const char* kind=e.kind==0 ? "startup" : e.kind==1 ? "change" : e.kind==2 ? "connected" : "resync";
  char topic[128],payload[512];
  const int topicLength=snprintf(topic,sizeof(topic),"%sschaltereignis",MQTT_TEXT);
  const int length=snprintf(payload,sizeof(payload),
    "{\"bootId\":\"%08lx\",\"seq\":%lu,\"typ\":\"%s\",\"uptimeMs\":%llu,\"unixMs\":%lld,"
    "\"changedMask\":%u,\"BrennerRelais\":%u,\"HeizungsRelais\":%u,\"BoilerRelais\":%u,"
    "\"MischerAufRelais\":%u,\"MischerZuRelais\":%u,\"verlorenSeitStart\":%lu}",
    static_cast<unsigned long>(relayBootId),static_cast<unsigned long>(e.sequence),kind,
    static_cast<unsigned long long>(e.uptimeMs),static_cast<long long>(e.unixMs),e.changed,
    (e.states>>0)&1,(e.states>>1)&1,(e.states>>2)&1,(e.states>>3)&1,(e.states>>4)&1,
    static_cast<unsigned long>(relayLost));
  if (topicLength<0 || topicLength>=static_cast<int>(sizeof(topic)) || length<0 || length>=static_cast<int>(sizeof(payload))) return;
  portENTER_CRITICAL(&relayAckMux);
  relayTrackedPacket=relayAckedPacket=0;
  portEXIT_CRITICAL(&relayAckMux);
  relayInFlight=asyncMqttClient.publish(topic,1,false,payload,length);
  relaySentEpoch=epoch;
  portENTER_CRITICAL(&relayAckMux);
  relayTrackedPacket=relayInFlight;
  portEXIT_CRITICAL(&relayAckMux);
  if (!relayInFlight) relayRetryAfter=now+1000;
}

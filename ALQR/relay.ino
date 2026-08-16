#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1
#define MAX_HOPS 4

uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t myMac[6];

// ------------------------------------------------------------
// PACKET STRUCTURES
// ------------------------------------------------------------

#define PACKET_COMMAND   1
#define PACKET_TELEMETRY 2

struct __attribute__((packed)) HopData {
  uint8_t mac[6];
  int16_t rssi;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t type;
  uint8_t sourceMac[6];
  uint32_t seqID;
  uint8_t hopCount;
  HopData hops[MAX_HOPS];
};

struct __attribute__((packed)) CommandPacket {
  uint8_t type;
  uint32_t seqID;
  char command[16];
  uint8_t routeLength;
  uint8_t currentHop;
  uint8_t route[MAX_HOPS + 1][6]; 
};

// ------------------------------------------------------------
// LOOP PREVENTION
// ------------------------------------------------------------

#define HISTORY_SIZE 20
uint32_t seenTelemetry[HISTORY_SIZE] = {0};
uint64_t seenCommands[HISTORY_SIZE] = {0};
uint8_t tIndex = 0, cIndex = 0;

uint64_t commandKey(uint32_t seqID, uint8_t currentHop) {
  return ((uint64_t)seqID << 8) | currentHop;
}

bool isTelemetrySeen(uint32_t seqID) {
  for (int i = 0; i < HISTORY_SIZE; i++) {
    if (seenTelemetry[i] == seqID) return true;
  }
  return false;
}

void markTelemetrySeen(uint32_t seqID) {
  seenTelemetry[tIndex] = seqID;
  tIndex = (tIndex + 1) % HISTORY_SIZE;
}

bool isCommandHopSeen(uint32_t seqID, uint8_t currentHop) {
  uint64_t key = commandKey(seqID, currentHop);
  for (int i = 0; i < HISTORY_SIZE; i++) {
    if (seenCommands[i] == key) return true;
  }
  return false;
}

void markCommandHopSeen(uint32_t seqID, uint8_t currentHop) {
  seenCommands[cIndex] = commandKey(seqID, currentHop);
  cIndex = (cIndex + 1) % HISTORY_SIZE;
}

// ------------------------------------------------------------
// RECEIVE & FORWARD LOGIC
// ------------------------------------------------------------

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t type = data[0];

  if (type == PACKET_TELEMETRY && len == sizeof(TelemetryPacket)) {
    TelemetryPacket t;
    memcpy(&t, data, sizeof(t));

    // 1. Kill Broadcast Storms 
    if (isTelemetrySeen(t.seqID)) return;
    markTelemetrySeen(t.seqID);

    // 2. Kill Routing Loops 
    for (int i = 0; i < t.hopCount; i++) {
      if (memcmp(t.hops[i].mac, myMac, 6) == 0) return; 
    }

    // 3. Append Data & Forward
    if (t.hopCount < MAX_HOPS) {
      memcpy(t.hops[t.hopCount].mac, myMac, 6); 
      t.hops[t.hopCount].rssi = info->rx_ctrl->rssi; 
      t.hopCount++;

      esp_now_send(broadcastMac, (uint8_t *)&t, sizeof(t));
      
      Serial.print("[PIGGYBACK] Forwarding Telemetry. Total Hops: ");
      Serial.println(t.hopCount);
    }
  }

  if (type == PACKET_COMMAND && len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));

    // 1. Kill Broadcast Storms
    if (isCommandHopSeen(cmd.seqID, cmd.currentHop)) return;
    markCommandHopSeen(cmd.seqID, cmd.currentHop);

    // 2. Check if I am the next hop
    if (cmd.currentHop < cmd.routeLength) {
      if (memcmp(cmd.route[cmd.currentHop], myMac, 6) == 0) {
        
        Serial.print("[ROUTING] Relaying Command: ");
        Serial.println(cmd.command);

        cmd.currentHop++;
        esp_now_send(broadcastMac, (uint8_t *)&cmd, sizeof(cmd));
      }
    }
  }
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  delay(100);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  // Safely read MAC Address
  WiFi.macAddress(myMac);
  if (myMac[0] == 0 && myMac[1] == 0 && myMac[5] == 0) {
    Serial.println("WARNING: Hardware MAC failed. Using Synthetic MAC.");
    myMac[0] = 0xAA; myMac[1] = 0xBB; myMac[2] = 0xCC;
    myMac[3] = 0xDD; myMac[4] = 0xEE; myMac[5] = 0xFF;
  }

  Serial.println("\n========================================");
  Serial.println("  ECHOLINK - UNIVERSAL RELAY");
  Serial.println("  DYNAMIC MESH MODE");
  Serial.print("  MY MAC: ");
  for(int i=0; i<6; i++) { Serial.print(myMac[i], HEX); if(i<5) Serial.print(":"); }
  Serial.println("\n========================================");

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(onDataReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() { 
  delay(10); 
}
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1
#define MAX_HOPS 4

#define FORWARD_BUTTON   13
#define BACKWARD_BUTTON  14
#define LEFT_BUTTON      27
#define RIGHT_BUTTON     33

uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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
// ROUTE CACHING & ALQR
// ------------------------------------------------------------

struct ActiveRoute {
  uint8_t targetMac[6];
  uint8_t routeLength;
  uint8_t route[MAX_HOPS + 1][6];
  float bestScore;
  unsigned long lastUpdated;
  bool isValid;
};

ActiveRoute currentBestRoute = { .targetMac = {0}, .routeLength = 0, .route = {{0}}, .bestScore = -9999.0, .lastUpdated = 0, .isValid = false };

String macToStr(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// ------------------------------------------------------------
// COMMAND LOOP
// ------------------------------------------------------------

uint32_t commandSeq = 0;
String lastCommand = "";
unsigned long lastCmdTime = 0;

void sendCommand(const char *command) {
  if (!currentBestRoute.isValid) {
    Serial.println("[TX] Cannot send: No active route to car discovered yet.");
    return;
  }

  CommandPacket cmd = {};
  cmd.type = PACKET_COMMAND;
  cmd.seqID = ++commandSeq;
  strncpy(cmd.command, command, sizeof(cmd.command) - 1);
  
  cmd.routeLength = currentBestRoute.routeLength;
  cmd.currentHop = 0;
  memcpy(cmd.route, currentBestRoute.route, sizeof(currentBestRoute.route));

  esp_now_send(broadcastMac, (uint8_t *)&cmd, sizeof(cmd));

  Serial.print("[TX] Sent: ");
  Serial.print(command);
  Serial.print(" | Path: TX -> ");
  for (int i = 0; i < cmd.routeLength; i++) {
    Serial.print(macToStr(cmd.route[i]));
    if (i < cmd.routeLength - 1) {
      Serial.print(" -> ");
    }
  }
  Serial.println();
}

void readButtons() {
  bool f = digitalRead(FORWARD_BUTTON) == LOW;
  bool b = digitalRead(BACKWARD_BUTTON) == LOW;
  bool l = digitalRead(LEFT_BUTTON) == LOW;
  bool r = digitalRead(RIGHT_BUTTON) == LOW;

  String command;
  if (f && !b) command = "FORWARD";
  else if (b && !f) command = "BACKWARD";
  else if (l && !r) command = "LEFT";
  else if (r && !l) command = "RIGHT";
  else command = "STOP";

  if (millis() - lastCmdTime > 100) {
    if (command != "STOP" || command != lastCommand) {
      sendCommand(command.c_str());
      lastCommand = command;
      lastCmdTime = millis();
    }
  }
}

// ------------------------------------------------------------
// ALQR PROCESSING (FIXED MAC OFFSET BUG)
// ------------------------------------------------------------

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;
  
  if (data[0] == PACKET_TELEMETRY) {
    if (len != sizeof(TelemetryPacket)) return;
    TelemetryPacket t;
    memcpy(&t, data, sizeof(t));

    int bottleneckRSSI = info->rx_ctrl->rssi; 
    for (int i = 0; i < t.hopCount; i++) {
      if (t.hops[i].rssi < bottleneckRSSI) {
        bottleneckRSSI = t.hops[i].rssi;
      }
    }

    float rssiScore = map(bottleneckRSSI, -90, -40, 0, 100);
    rssiScore = constrain(rssiScore, 0, 100);

    int totalHops = t.hopCount + 1; 
    float hopScore = 100 - ((totalHops - 1) * 20); 

    float finalScore = (0.6 * rssiScore) + (0.4 * hopScore);

    // ==========================================
    // THE FIX: EXACT MAC RECONSTRUCTION
    // Build route ONLY using the MACs explicitly in the payload
    // ==========================================
    uint8_t tempRoute[MAX_HOPS + 1][6];
    int routeIndex = 0;
    
    // Reverse the nodes (e.g. Node 2, then Node 1)
    for (int i = t.hopCount - 1; i >= 0; i--) {
      memcpy(tempRoute[routeIndex++], t.hops[i].mac, 6);
    }
    // Finally add the Car
    memcpy(tempRoute[routeIndex], t.sourceMac, 6);

    bool isSameRoute = false;
    if (currentBestRoute.isValid && currentBestRoute.routeLength == totalHops) {
      isSameRoute = true;
      for (int i = 0; i < totalHops; i++) {
        if (memcmp(currentBestRoute.route[i], tempRoute[i], 6) != 0) {
          isSameRoute = false;
          break;
        }
      }
    }

    if (isSameRoute) {
      currentBestRoute.bestScore = finalScore;
      currentBestRoute.lastUpdated = millis();
    } 
    else {
      if (!currentBestRoute.isValid || finalScore > currentBestRoute.bestScore + 5.0 || (millis() - currentBestRoute.lastUpdated > 2000)) {
        
        currentBestRoute.bestScore = finalScore;
        currentBestRoute.lastUpdated = millis();
        currentBestRoute.isValid = true;
        currentBestRoute.routeLength = totalHops;
        memcpy(currentBestRoute.targetMac, t.sourceMac, 6);
        memcpy(currentBestRoute.route, tempRoute, sizeof(tempRoute));

        Serial.println("\n[ALQR] --- NEW ROUTE SELECTED ---");
        Serial.print("Hops: "); Serial.println(totalHops);
        Serial.print("Bottleneck RSSI: "); Serial.print(bottleneckRSSI); Serial.println(" dBm");
        Serial.print("ALQR Score: "); Serial.println(finalScore);
        Serial.println("---------------------------------");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(FORWARD_BUTTON, INPUT_PULLUP); 
  pinMode(BACKWARD_BUTTON, INPUT_PULLUP);
  pinMode(LEFT_BUTTON, INPUT_PULLUP); 
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println("\n========================================");
  Serial.println("  ECHOLINK - TRANSMITTER (ALQR BRAIN)");
  Serial.println("  DYNAMIC MESH MODE");
  Serial.println("========================================");

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(onDataReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  readButtons();
  delay(5);
}
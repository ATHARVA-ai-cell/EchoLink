//node 1
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1
#define NODE_TRANSMITTER 1
#define NODE_RELAY_A     2
#define NODE_RECEIVER    3

uint8_t TRANSMITTER_MAC[] = { 0x4C, 0xC3, 0x82, 0xC4, 0xE0, 0x10 };
uint8_t RECEIVER_MAC[]    = { 0x68, 0xFE, 0x71, 0x0D, 0x3B, 0x54 };

#define PACKET_COMMAND       1
#define PACKET_COMMAND_ACK   2
#define PACKET_PROBE         3
#define PACKET_PROBE_ACK     4
#define PACKET_TELEMETRY     5

struct Packet {
  uint8_t type;
  uint8_t source;
  uint8_t destination;
  uint32_t packetID;
  char command[16];
  uint8_t hopCount;
  uint8_t ttl;
  uint32_t txTimestamp;      // Clock Sync for E2E
  uint32_t relayTimestamp;   // Clock for Link 2
  int16_t rssiLink1;
  int16_t rssiLink2;
  uint32_t packetsSent;
  uint32_t packetsReceived;
  uint32_t packetsLost;
  uint32_t bytesReceived;
  float pdr;
  float packetLossPercent;
  float latency;
  float throughput;
  float jitter;
  uint32_t totalPacketsSent;
  uint32_t totalPacketsReceived;
  uint32_t totalPacketsLost;
  float endToEndLatency;
  float endToEndThroughput;
  float endToEndJitter;
  uint32_t telemetrySequence;
};

// --- Network Time Sync ---
int32_t txClockOffset = 0; // Dynamic clock offset between TX and Relay

// --- LINK 1 ---
uint32_t link1Sent = 0;
uint32_t link1Received = 0;
uint32_t link1Bytes = 0;
int link1RSSI = 0;
uint32_t link1ProbeSentTime = 0;
uint32_t link1LatencyTotal = 0;
uint32_t link1LatencySamples = 0;
uint32_t lastLink1Latency = 0;
uint32_t link1JitterTotal = 0;
uint32_t link1JitterSamples = 0;

// --- LINK 2 ---
uint32_t link2Sent = 0;
uint32_t link2Received = 0;
uint32_t link2Bytes = 0;
int link2RSSI = 0;
uint32_t link2LatencyTotal = 0;
uint32_t link2LatencySamples = 0;
uint32_t lastLink2Latency = 0;
uint32_t link2JitterTotal = 0;
uint32_t link2JitterSamples = 0;

// --- END-TO-END ---
uint32_t e2eSent = 0;
uint32_t e2eReceived = 0;
uint32_t e2eBytes = 0;
uint32_t e2eLatencyTotal = 0;
uint32_t e2eLatencySamples = 0;
uint32_t lastE2ELatency = 0;
uint32_t e2eJitterTotal = 0;
uint32_t e2eJitterSamples = 0;

uint32_t commandAckCount = 0;
unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL = 1000;
uint32_t telemetrySequence = 0;


float calculatePDR(uint32_t sent, uint32_t received) {
  if (sent == 0) return 0.0;
  return (((float)received / (float)sent) * 100.0);
}
float calculateLoss(uint32_t sent, uint32_t received) {
  return (100.0 - calculatePDR(sent, received));
}
float average(uint32_t total, uint32_t samples) {
  if (samples == 0) return 0.0;
  return ((float)total / (float)samples);
}
float calculateThroughput(uint32_t bytes, unsigned long elapsed) {
  if (elapsed == 0) return 0.0;
  float bits = (float)bytes * 8.0;
  float seconds = (float)elapsed / 1000.0;
  return (bits / seconds / 1000.0);
}

void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {}

bool addPeer(uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  return (esp_now_add_peer(&peerInfo) == ESP_OK);
}

void printTelemetry() {
  unsigned long elapsed = millis();
  if (elapsed == 0) elapsed = 1;

  float link1PDR = calculatePDR(link1Sent, link1Received);
  float link2PDR = calculatePDR(link2Sent, link2Received);
  float e2ePDR = calculatePDR(e2eSent, e2eReceived);

  Serial.println("\n================ ECHOLINK TELEMETRY ================");

  // LINK 1
  Serial.println("\nLINK 1: TX -> RELAY-A");
  Serial.println("--------------------------------");
  Serial.print("RSSI              : "); Serial.print(link1RSSI); Serial.println(" dBm");
  Serial.print("Packets Sent      : "); Serial.println(link1Sent);
  Serial.print("Packets Received  : "); Serial.println(link1Received);
  Serial.print("PDR               : "); Serial.print(link1PDR, 2); Serial.println(" %");
  Serial.print("Packet Loss       : "); Serial.print(calculateLoss(link1Sent, link1Received), 2); Serial.println(" %");
  Serial.print("Average Latency   : "); Serial.print(average(link1LatencyTotal, link1LatencySamples), 2); Serial.println(" ms");
  Serial.print("Throughput        : "); Serial.print(calculateThroughput(link1Bytes, elapsed), 2); Serial.println(" kbps");
  Serial.print("Jitter            : "); Serial.print(average(link1JitterTotal, link1JitterSamples), 2); Serial.println(" ms");
  Serial.println("Hop Count         : 1");

  // LINK 2
  Serial.println("\nLINK 2: RELAY-A -> RX");
  Serial.println("--------------------------------");
  Serial.print("RSSI              : "); Serial.print(link2RSSI); Serial.println(" dBm");
  Serial.print("Packets Sent      : "); Serial.println(link2Sent);
  Serial.print("Packets Received  : "); Serial.println(link2Received);
  Serial.print("PDR               : "); Serial.print(link2PDR, 2); Serial.println(" %");
  Serial.print("Packet Loss       : "); Serial.print(calculateLoss(link2Sent, link2Received), 2); Serial.println(" %");
  Serial.print("Average Latency   : "); Serial.print(average(link2LatencyTotal, link2LatencySamples), 2); Serial.println(" ms");
  Serial.print("Throughput        : "); Serial.print(calculateThroughput(link2Bytes, elapsed), 2); Serial.println(" kbps");
  Serial.print("Jitter            : "); Serial.print(average(link2JitterTotal, link2JitterSamples), 2); Serial.println(" ms");
  Serial.println("Hop Count         : 1");

  // END TO END
  Serial.println("\nEND-TO-END: TX -> RELAY-A -> RX");
  Serial.println("--------------------------------");
  Serial.print("Packets Sent      : "); Serial.println(e2eSent);
  Serial.print("Packets Received  : "); Serial.println(e2eReceived);
  Serial.print("PDR               : "); Serial.print(e2ePDR, 2); Serial.println(" %");
  Serial.print("Packet Loss       : "); Serial.print(calculateLoss(e2eSent, e2eReceived), 2); Serial.println(" %");
  Serial.print("Average Latency   : "); Serial.print(average(e2eLatencyTotal, e2eLatencySamples), 2); Serial.println(" ms");
  Serial.print("Throughput        : "); Serial.print(calculateThroughput(e2eBytes, elapsed), 2); Serial.println(" kbps");
  Serial.print("Jitter            : "); Serial.print(average(e2eJitterTotal, e2eJitterSamples), 2); Serial.println(" ms");
  Serial.println("Total Hop Count   : 2");
  Serial.println("\n=====================================================");
}

void sendTelemetry() {
  Packet telemetry = {};
  telemetry.type = PACKET_TELEMETRY;
  telemetry.source = NODE_RELAY_A;
  telemetry.destination = NODE_TRANSMITTER;
  telemetry.telemetrySequence = ++telemetrySequence;
  telemetry.rssiLink1 = link1RSSI;
  telemetry.rssiLink2 = link2RSSI;

  telemetry.packetsSent = link1Sent;
  telemetry.packetsReceived = link1Received;
  telemetry.packetsLost = link1Sent - min(link1Sent, link1Received);
  telemetry.pdr = calculatePDR(link1Sent, link1Received);
  telemetry.packetLossPercent = calculateLoss(link1Sent, link1Received);
  telemetry.latency = average(link1LatencyTotal, link1LatencySamples);
  telemetry.throughput = calculateThroughput(link1Bytes, millis());
  telemetry.jitter = average(link1JitterTotal, link1JitterSamples);

  telemetry.totalPacketsSent = e2eSent;
  telemetry.totalPacketsReceived = e2eReceived;
  telemetry.totalPacketsLost = e2eSent - min(e2eSent, e2eReceived);
  telemetry.endToEndLatency = average(e2eLatencyTotal, e2eLatencySamples);
  telemetry.endToEndThroughput = calculateThroughput(e2eBytes, millis());
  telemetry.endToEndJitter = average(e2eJitterTotal, e2eJitterSamples);

  esp_now_send(TRANSMITTER_MAC, (uint8_t *)&telemetry, sizeof(telemetry));
  telemetry.destination = NODE_RECEIVER;
  esp_now_send(RECEIVER_MAC, (uint8_t *)&telemetry, sizeof(telemetry));
}

void forwardCommand(Packet packet) {
  if (packet.ttl == 0) return;
  packet.hopCount++;
  packet.ttl--;
  link2Sent++;
  esp_now_send(RECEIVER_MAC, (uint8_t *)&packet, sizeof(packet));
}

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Packet)) {
    Serial.println("[RELAY] INVALID PACKET");
    return;
  }

  Packet packet;
  memcpy(&packet, data, sizeof(packet));
  int incomingRSSI = info->rx_ctrl->rssi;

  if (packet.type == PACKET_COMMAND && packet.source == NODE_TRANSMITTER) {
    link1Received++;
    link1RSSI = incomingRSSI;
    link1Bytes += sizeof(Packet);
    e2eSent++;
    forwardCommand(packet);
    return;
  }

  if (packet.type == PACKET_PROBE && packet.source == NODE_TRANSMITTER) {
    link1Received++;
    link1RSSI = incomingRSSI;
    link1Bytes += sizeof(Packet);
    e2eSent++;

    // --- Time Sync Magic ---
    // Calculate difference between Relay's clock and TX clock
    txClockOffset = packet.txTimestamp - millis(); 

    Packet ack = {};
    ack.type = PACKET_PROBE_ACK;
    ack.source = NODE_RELAY_A;
    ack.destination = NODE_TRANSMITTER;
    ack.packetID = packet.packetID;
    ack.txTimestamp = packet.txTimestamp;
    ack.hopCount = packet.hopCount;
    ack.ttl = 5;

    esp_now_send(TRANSMITTER_MAC, (uint8_t *)&ack, sizeof(ack));

    packet.hopCount++;
    packet.ttl--;
    link2Sent++;
    
    // Set timestamp exclusively for Link 2 math
    packet.relayTimestamp = millis(); 

    esp_now_send(RECEIVER_MAC, (uint8_t *)&packet, sizeof(packet));
    return;
  }

  if (packet.type == PACKET_PROBE_ACK && packet.source == NODE_RECEIVER) {
    link2Received++;
    link2RSSI = incomingRSSI;
    link2Bytes += sizeof(Packet);
    e2eReceived++;
    e2eBytes += sizeof(Packet);

    // --- Link 2 Calculations ---
    uint32_t link2Lat = millis() - packet.relayTimestamp;
    link2LatencyTotal += link2Lat;
    link2LatencySamples++;

    if (link2LatencySamples > 1) {
      uint32_t diff;
      if (link2Lat > lastLink2Latency) diff = link2Lat - lastLink2Latency;
      else diff = lastLink2Latency - link2Lat;
      link2JitterTotal += diff;
      link2JitterSamples++;
    }
    lastLink2Latency = link2Lat;

    // --- End To End Calculations (Common Clock) ---
    // Estimate current time on the TX node's clock
    uint32_t syncedTime = millis() + txClockOffset; 
    uint32_t e2eLat = syncedTime - packet.txTimestamp; 
    
    e2eLatencyTotal += e2eLat;
    e2eLatencySamples++;

    if (e2eLatencySamples > 1) {
      uint32_t difference;
      if (e2eLat > lastE2ELatency) difference = e2eLat - lastE2ELatency;
      else difference = lastE2ELatency - e2eLat;
      e2eJitterTotal += difference;
      e2eJitterSamples++;
    }
    lastE2ELatency = e2eLat;

    packet.destination = NODE_TRANSMITTER;
    esp_now_send(TRANSMITTER_MAC, (uint8_t *)&packet, sizeof(packet));
    return;
  }

  if (packet.type == PACKET_COMMAND_ACK && packet.source == NODE_RECEIVER) {
    link2Received++;
    link2RSSI = incomingRSSI;
    esp_now_send(TRANSMITTER_MAC, (uint8_t *)&packet, sizeof(packet));
    return;
  }

  if (packet.type == PACKET_TELEMETRY) {
    printTelemetry();
    return;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println("\n========================================");
  Serial.println("          ECHOLINK - RELAY-A");
  Serial.println("          ESP32 CORE 3.3.11");
  Serial.println("========================================");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceive);

  if (!addPeer(TRANSMITTER_MAC)) {
    Serial.println("TRANSMITTER PEER FAILED");
    return;
  }
  if (!addPeer(RECEIVER_MAC)) {
    Serial.println("RECEIVER PEER FAILED");
    return;
  }

  Serial.println("\nTRANSMITTER PEER : READY");
  Serial.println("RECEIVER PEER    : READY");
  Serial.println("ESP-NOW          : READY");
  Serial.println("TELEMETRY        : ACTIVE");
  Serial.println("========================================");
}

void loop() {
  if (millis() - lastTelemetry >= TELEMETRY_INTERVAL) {
    lastTelemetry = millis();
    printTelemetry();
    sendTelemetry();
  }
  delay(10);
}
//transmitter
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1
#define NODE_TRANSMITTER 1
#define NODE_RELAY_A     2
#define NODE_RECEIVER    3

#define FORWARD_BUTTON   13
#define BACKWARD_BUTTON  14
#define LEFT_BUTTON      27
#define RIGHT_BUTTON     33

uint8_t RELAY_A_MAC[] = { 0x8C, 0x94, 0xDF, 0x4D, 0x9C, 0x5C };

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
  uint32_t txTimestamp;      // Replaced old timestamp
  uint32_t relayTimestamp;   // Added for Link 2
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

uint32_t commandID = 0;
String lastCommand = "";
unsigned long lastCommandSendTime = 0;

uint32_t commandSent = 0;
uint32_t commandAckReceived = 0;
uint32_t commandLatencyTotal = 0;
uint32_t lastCommandLatency = 0;
uint32_t commandJitterTotal = 0;
uint32_t commandJitterSamples = 0;

uint32_t probeID = 0;
uint32_t probeSent = 0;
uint32_t probeAckReceived = 0;
uint32_t probeBytesReceived = 0;
uint32_t lastProbeRTT = 0;
uint32_t probeRTTTotal = 0;
uint32_t probeJitterTotal = 0;
uint32_t probeJitterSamples = 0;
unsigned long probeStartTime = 0;
const unsigned long PROBE_INTERVAL_MS = 200;

uint32_t telemetrySequence = 0;
float latestLink1RSSI = 0;
float latestLink2RSSI = 0;
unsigned long lastTelemetryPrint = 0;

const uint32_t PACKET_BYTES = sizeof(Packet);

void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {}

bool addRelayPeer() {
  if (esp_now_is_peer_exist(RELAY_A_MAC)) return true;
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RELAY_A_MAC, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  return (esp_now_add_peer(&peerInfo) == ESP_OK);
}

void printTelemetry(Packet &p) {
  Serial.println();
  Serial.println("================ ECHOLINK TELEMETRY ================");
  Serial.println("\nLINK 1: TX -> RELAY-A");
  Serial.println("--------------------------------");
  Serial.print("RSSI              : "); Serial.print(p.rssiLink1); Serial.println(" dBm");
  Serial.print("Packets Sent      : "); Serial.println(p.packetsSent);
  Serial.print("Packets Received  : "); Serial.println(p.packetsReceived);
  Serial.print("PDR               : "); Serial.print(p.pdr, 2); Serial.println(" %");
  Serial.print("Packet Loss       : "); Serial.print(p.packetLossPercent, 2); Serial.println(" %");
  Serial.print("Average Latency   : "); Serial.print(p.latency, 2); Serial.println(" ms");
  Serial.print("Throughput        : "); Serial.print(p.throughput, 2); Serial.println(" kbps");
  Serial.print("Jitter            : "); Serial.print(p.jitter, 2); Serial.println(" ms");
  Serial.println("Hop Count         : 1");

  Serial.println("\nLINK 2: RELAY-A -> RX");
  Serial.println("--------------------------------");
  Serial.print("RSSI              : "); Serial.print(p.rssiLink2); Serial.println(" dBm");
  Serial.print("Packets Sent      : "); Serial.println(p.packetsSent);
  Serial.print("Packets Received  : "); Serial.println(p.packetsReceived);
  Serial.print("PDR               : "); Serial.print(p.pdr, 2); Serial.println(" %");
  Serial.print("Packet Loss       : "); Serial.print(p.packetLossPercent, 2); Serial.println(" %");
  Serial.print("Average Latency   : "); Serial.print(p.latency, 2); Serial.println(" ms");
  Serial.print("Throughput        : "); Serial.print(p.throughput, 2); Serial.println(" kbps");
  Serial.print("Jitter            : "); Serial.print(p.jitter, 2); Serial.println(" ms");
  Serial.println("Hop Count         : 1");

  Serial.println("\nEND-TO-END: TX -> RELAY-A -> RX");
  Serial.println("--------------------------------");
  Serial.print("Packets Sent      : "); Serial.println(p.totalPacketsSent);
  Serial.print("Packets Received  : "); Serial.println(p.totalPacketsReceived);
  Serial.print("PDR               : ");
  float e2ePDR = 0;
  if (p.totalPacketsSent > 0) e2ePDR = ((float)p.totalPacketsReceived / (float)p.totalPacketsSent) * 100.0;
  Serial.print(e2ePDR, 2); Serial.println(" %");
  Serial.print("Packet Loss       : "); Serial.print(100.0 - e2ePDR, 2); Serial.println(" %");
  Serial.print("Average Latency   : "); Serial.print(p.endToEndLatency, 2); Serial.println(" ms");
  Serial.print("Throughput        : "); Serial.print(p.endToEndThroughput, 2); Serial.println(" kbps");
  Serial.print("Jitter            : "); Serial.print(p.endToEndJitter, 2); Serial.println(" ms");
  Serial.println("Total Hop Count   : 2");
  Serial.println("\n=====================================================");
}

void sendCommand(const char *command) {
  Packet packet = {};
  packet.type = PACKET_COMMAND;
  packet.source = NODE_TRANSMITTER;
  packet.destination = NODE_RECEIVER;
  packet.packetID = ++commandID;
  strncpy(packet.command, command, sizeof(packet.command) - 1);
  packet.hopCount = 0;
  packet.ttl = 5;
  packet.txTimestamp = millis(); // Track TX time

  lastCommandSendTime = millis();
  commandSent++;
  esp_now_send(RELAY_A_MAC, (uint8_t *)&packet, sizeof(packet));

  Serial.println("\n[TX] COMMAND SENT");
  Serial.print("Command : "); Serial.println(command);
  Serial.print("Packet ID : "); Serial.println(packet.packetID);
}

void sendProbe() {
  Packet packet = {};
  packet.type = PACKET_PROBE;
  packet.source = NODE_TRANSMITTER;
  packet.destination = NODE_RECEIVER;
  packet.packetID = ++probeID;
  packet.hopCount = 0;
  packet.ttl = 5;
  packet.txTimestamp = millis(); // Track TX time

  probeSent++;
  esp_now_send(RELAY_A_MAC, (uint8_t *)&packet, sizeof(packet));
}

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Packet)) return;

  Packet packet;
  memcpy(&packet, data, sizeof(packet));

  if (packet.type == PACKET_COMMAND_ACK) {
    commandAckReceived++;
    uint32_t latency = millis() - lastCommandSendTime;
    commandLatencyTotal += latency;

    if (commandAckReceived > 1) {
      uint32_t difference;
      if (latency > lastCommandLatency) difference = latency - lastCommandLatency;
      else difference = lastCommandLatency - latency;
      commandJitterTotal += difference;
      commandJitterSamples++;
    }
    lastCommandLatency = latency;
    Serial.print("[TX] END-TO-END COMMAND RTT: ");
    Serial.print(latency); Serial.println(" ms");
    return;
  }

  if (packet.type == PACKET_PROBE_ACK) {
    probeAckReceived++;
    uint32_t rtt = millis() - packet.txTimestamp; // Math using txTimestamp
    probeRTTTotal += rtt;

    if (probeAckReceived > 1) {
      uint32_t difference;
      if (rtt > lastProbeRTT) difference = rtt - lastProbeRTT;
      else difference = lastProbeRTT - rtt;
      probeJitterTotal += difference;
      probeJitterSamples++;
    }
    lastProbeRTT = rtt;
    return;
  }

  if (packet.type == PACKET_TELEMETRY) {
    printTelemetry(packet);
  }
}

void readButtons() {
  bool forward = digitalRead(FORWARD_BUTTON) == LOW;
  bool backward = digitalRead(BACKWARD_BUTTON) == LOW;
  bool left = digitalRead(LEFT_BUTTON) == LOW;
  bool right = digitalRead(RIGHT_BUTTON) == LOW;
  String command = "";

  if (forward && !backward) command = "FORWARD";
  else if (backward && !forward) command = "BACKWARD";
  else if (left && !right) command = "LEFT";
  else if (right && !left) command = "RIGHT";
  else command = "STOP";

  if (command != lastCommand) {
    sendCommand(command.c_str());
    lastCommand = command;
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
  Serial.println("       ECHOLINK - TRANSMITTER");
  Serial.println("       ESP32 CORE 3.3.11");
  Serial.println("========================================");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceive);

  if (!addRelayPeer()) {
    Serial.println("RELAY-A PEER FAILED");
    return;
  }

  Serial.println("RELAY-A PEER : READY");
  Serial.println("ESP-NOW      : READY");
  Serial.println("TELEMETRY    : ACTIVE");
  Serial.println("========================================");
}

void loop() {
  readButtons();
  if (millis() - probeStartTime >= PROBE_INTERVAL_MS) {
    probeStartTime = millis();
    sendProbe();
  }
  delay(20);
}
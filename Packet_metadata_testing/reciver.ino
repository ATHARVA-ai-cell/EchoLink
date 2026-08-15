//receiver
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1
#define NODE_TRANSMITTER 1
#define NODE_RELAY_A     2
#define NODE_RECEIVER    3

uint8_t RELAY_A_MAC[] = { 0x8C, 0x94, 0xDF, 0x4D, 0x9C, 0x5C };

const uint8_t STEER_IN1 = 26;
const uint8_t STEER_IN2 = 25;
const uint8_t DRIVE_IN1 = 22;
const uint8_t DRIVE_IN2 = 23;

const unsigned long COMMAND_TIMEOUT_MS = 700;
const unsigned long STEERING_PULSE_MS = 150;

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
  uint32_t txTimestamp;      // Preserved from TX
  uint32_t relayTimestamp;   // Preserved from Relay
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

enum DriveState { DRIVE_STOP, DRIVE_FORWARD, DRIVE_BACKWARD };
enum SteeringState { STEER_CENTER, STEER_LEFT, STEER_RIGHT };
DriveState driveState = DRIVE_STOP;
SteeringState steeringState = STEER_CENTER;

unsigned long lastDriveCommandTime = 0;
unsigned long steeringStopTime = 0;

uint32_t link2Received = 0;
uint32_t link2Bytes = 0;
int link2RSSI = 0;

uint32_t lastProbeID = 0;
bool firstProbe = true;
unsigned long lastTelemetryPrint = 0;

void stopDrive() {
  digitalWrite(DRIVE_IN1, LOW);
  digitalWrite(DRIVE_IN2, LOW);
  driveState = DRIVE_STOP;
}
void driveForward() {
  digitalWrite(DRIVE_IN1, HIGH);
  digitalWrite(DRIVE_IN2, LOW);
  driveState = DRIVE_FORWARD;
  lastDriveCommandTime = millis();
}
void driveBackward() {
  digitalWrite(DRIVE_IN1, LOW);
  digitalWrite(DRIVE_IN2, HIGH);
  driveState = DRIVE_BACKWARD;
  lastDriveCommandTime = millis();
}
void stopSteering() {
  digitalWrite(STEER_IN1, LOW);
  digitalWrite(STEER_IN2, LOW);
  steeringState = STEER_CENTER;
}
void steerLeft() {
  digitalWrite(STEER_IN1, HIGH);
  digitalWrite(STEER_IN2, LOW);
  steeringState = STEER_LEFT;
  steeringStopTime = millis() + STEERING_PULSE_MS;
}
void steerRight() {
  digitalWrite(STEER_IN1, LOW);
  digitalWrite(STEER_IN2, HIGH);
  steeringState = STEER_RIGHT;
  steeringStopTime = millis() + STEERING_PULSE_MS;
}
void emergencyStop() {
  stopDrive();
  stopSteering();
}

void processCommand(const char *command) {
  if (strcmp(command, "FORWARD") == 0) driveForward();
  else if (strcmp(command, "BACKWARD") == 0) driveBackward();
  else if (strcmp(command, "LEFT") == 0) steerLeft();
  else if (strcmp(command, "RIGHT") == 0) steerRight();
  else if (strcmp(command, "STOP") == 0) emergencyStop();
}

void sendCommandACK(Packet received) {
  Packet ack = {};
  ack.type = PACKET_COMMAND_ACK;
  ack.source = NODE_RECEIVER;
  ack.destination = NODE_TRANSMITTER;
  ack.packetID = received.packetID;
  strncpy(ack.command, received.command, sizeof(ack.command) - 1);
  ack.hopCount = received.hopCount;
  ack.ttl = 5;
  ack.txTimestamp = received.txTimestamp;       // Echo back to network
  ack.relayTimestamp = received.relayTimestamp; // Echo back to network

  esp_now_send(RELAY_A_MAC, (uint8_t *)&ack, sizeof(ack));
}

void sendProbeACK(Packet received) {
  Packet ack = {};
  ack.type = PACKET_PROBE_ACK;
  ack.source = NODE_RECEIVER;
  ack.destination = NODE_RELAY_A;
  ack.packetID = received.packetID;
  ack.hopCount = received.hopCount;
  ack.ttl = 5;
  ack.txTimestamp = received.txTimestamp;       // Echo back to network
  ack.relayTimestamp = received.relayTimestamp; // Echo back to network

  esp_now_send(RELAY_A_MAC, (uint8_t *)&ack, sizeof(ack));
}

void printTelemetry(Packet &p) {
  // Console printing logic preserved as requested
  Serial.println("\n================ ECHOLINK TELEMETRY ================");
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
  float pdr = 0;
  if (p.totalPacketsSent > 0) pdr = ((float)p.totalPacketsReceived / (float)p.totalPacketsSent) * 100.0;
  Serial.print("PDR               : "); Serial.print(pdr, 2); Serial.println(" %");
  Serial.print("Packet Loss       : "); Serial.print(100.0 - pdr, 2); Serial.println(" %");
  Serial.print("Average Latency   : "); Serial.print(p.endToEndLatency, 2); Serial.println(" ms");
  Serial.print("Throughput        : "); Serial.print(p.endToEndThroughput, 2); Serial.println(" kbps");
  Serial.print("Jitter            : "); Serial.print(p.endToEndJitter, 2); Serial.println(" ms");
  Serial.println("Total Hop Count   : 2");
  Serial.println("\n=====================================================");
}

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Packet)) {
    Serial.println("[RX] INVALID PACKET");
    return;
  }

  Packet packet;
  memcpy(&packet, data, sizeof(packet));
  int incomingRSSI = info->rx_ctrl->rssi;

  if (packet.type == PACKET_COMMAND) {
    link2Received++;
    link2RSSI = incomingRSSI;
    processCommand(packet.command);
    sendCommandACK(packet);

    Serial.println("\n================================");
    Serial.println("       RECEIVER / RC CAR");
    Serial.println("================================");
    Serial.print("Packet ID : "); Serial.println(packet.packetID);
    Serial.print("Command   : "); Serial.println(packet.command);
    Serial.print("RSSI      : "); Serial.print(incomingRSSI); Serial.println(" dBm");
    Serial.print("Hop Count : "); Serial.println(packet.hopCount);
    return;
  }

  if (packet.type == PACKET_PROBE) {
    link2Received++;
    link2Bytes += sizeof(Packet);
    link2RSSI = incomingRSSI;
    lastProbeID = packet.packetID;
    firstProbe = false;
    sendProbeACK(packet);
    return;
  }

  if (packet.type == PACKET_TELEMETRY) {
    printTelemetry(packet);
    return;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);
  pinMode(DRIVE_IN1, OUTPUT);
  pinMode(DRIVE_IN2, OUTPUT);
  emergencyStop();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println("\n================================");
  Serial.println("      ECHOLINK - RECEIVER");
  Serial.println("      ESP32 CORE 3.3.11");
  Serial.println("================================");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_recv_cb(onDataReceive);

  if (!esp_now_is_peer_exist(RELAY_A_MAC)) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, RELAY_A_MAC, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("RELAY-A PEER FAILED");
      return;
    }
  }

  Serial.println("\nRELAY-A PEER : READY");
  Serial.println("ESP-NOW      : READY");
  Serial.println("MOTOR SAFETY : ACTIVE");
  Serial.println("TELEMETRY    : ACTIVE");
  Serial.println("================================");
}

void loop() {
  if (steeringState != STEER_CENTER && millis() >= steeringStopTime) stopSteering();
  if (driveState != DRIVE_STOP && millis() - lastDriveCommandTime > COMMAND_TIMEOUT_MS) {
    stopDrive();
    Serial.println("[SAFETY] Drive timeout -> STOP");
  }
  delay(2);
}
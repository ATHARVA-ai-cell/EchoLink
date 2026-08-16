#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1
#define MAX_HOPS 4

// Motor Pins
const uint8_t STEER_IN1 = 26;
const uint8_t STEER_IN2 = 25;
const uint8_t DRIVE_IN1 = 22;
const uint8_t DRIVE_IN2 = 23;

const unsigned long COMMAND_TIMEOUT_MS = 700;
const unsigned long STEERING_PULSE_MS = 150;
const unsigned long TELEMETRY_INTERVAL_MS = 250; // Heartbeat interval

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
  uint8_t route[MAX_HOPS + 1][6]; // Pre-calculated path from TX
};

// ------------------------------------------------------------
// LOOP PREVENTION
// ------------------------------------------------------------

#define HISTORY_SIZE 20
uint64_t seenCommands[HISTORY_SIZE] = {0};
uint8_t cIndex = 0;

uint64_t commandKey(uint32_t seqID, uint8_t currentHop) {
  return ((uint64_t)seqID << 8) | currentHop;
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
// MOTOR STATE
// ------------------------------------------------------------

enum DriveState { DRIVE_STOP, DRIVE_FORWARD, DRIVE_BACKWARD };
enum SteeringState { STEER_CENTER, STEER_LEFT, STEER_RIGHT };

DriveState driveState = DRIVE_STOP;
SteeringState steeringState = STEER_CENTER;

unsigned long lastDriveCommandTime = 0;
unsigned long steeringStopTime = 0;
unsigned long lastTelemetryTime = 0;
uint32_t telemetrySeqID = 0;

// ------------------------------------------------------------
// MOTOR CONTROL
// ------------------------------------------------------------

void stopDrive() { digitalWrite(DRIVE_IN1, LOW); digitalWrite(DRIVE_IN2, LOW); driveState = DRIVE_STOP; }
void driveForward() { digitalWrite(DRIVE_IN1, HIGH); digitalWrite(DRIVE_IN2, LOW); driveState = DRIVE_FORWARD; lastDriveCommandTime = millis(); }
void driveBackward() { digitalWrite(DRIVE_IN1, LOW); digitalWrite(DRIVE_IN2, HIGH); driveState = DRIVE_BACKWARD; lastDriveCommandTime = millis(); }
void stopSteering() { digitalWrite(STEER_IN1, LOW); digitalWrite(STEER_IN2, LOW); steeringState = STEER_CENTER; }
void steerLeft() { digitalWrite(STEER_IN1, HIGH); digitalWrite(STEER_IN2, LOW); steeringState = STEER_LEFT; steeringStopTime = millis() + STEERING_PULSE_MS; }
void steerRight() { digitalWrite(STEER_IN1, LOW); digitalWrite(STEER_IN2, HIGH); steeringState = STEER_RIGHT; steeringStopTime = millis() + STEERING_PULSE_MS; }
void emergencyStop() { stopDrive(); stopSteering(); }

void processCommand(const char *command) {
  if (strcmp(command, "FORWARD") == 0) driveForward();
  else if (strcmp(command, "BACKWARD") == 0) driveBackward();
  else if (strcmp(command, "LEFT") == 0) steerLeft();
  else if (strcmp(command, "RIGHT") == 0) steerRight();
  else if (strcmp(command, "STOP") == 0) emergencyStop();
  
  Serial.print("[COMMAND EXECUTED] ");
  Serial.println(command);
}

// ------------------------------------------------------------
// TELEMETRY PIGGYBACK TRIGGER
// ------------------------------------------------------------

void broadcastTelemetry() {
  TelemetryPacket p = {};
  p.type = PACKET_TELEMETRY;
  memcpy(p.sourceMac, myMac, 6);
  p.seqID = ++telemetrySeqID;
  p.hopCount = 0; // Starts at 0 hops

  esp_now_send(broadcastMac, (uint8_t *)&p, sizeof(p));
}

// ------------------------------------------------------------
// RECEIVE
// ------------------------------------------------------------

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t type = data[0];

  if (type == PACKET_COMMAND && len == sizeof(CommandPacket)) {
    CommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));

    // 1. Loop Prevention 
    if (isCommandHopSeen(cmd.seqID, cmd.currentHop)) return;
    markCommandHopSeen(cmd.seqID, cmd.currentHop);

    // 2. Are we the intended target for this hop?
    if (cmd.currentHop < cmd.routeLength) {
      if (memcmp(cmd.route[cmd.currentHop], myMac, 6) == 0) {
        // The packet has arrived at the car!
        processCommand(cmd.command);
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

  pinMode(STEER_IN1, OUTPUT); pinMode(STEER_IN2, OUTPUT);
  pinMode(DRIVE_IN1, OUTPUT); pinMode(DRIVE_IN2, OUTPUT);
  emergencyStop();

  WiFi.mode(WIFI_STA);
  delay(100); // Give the WiFi chip time to wake up
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  // Try to read the hardware MAC safely
  WiFi.macAddress(myMac);

  // FOOLPROOF FIX: If the hardware MAC reads as 00:00:00:00:00:00, use a synthetic one!
  if (myMac[0] == 0 && myMac[1] == 0 && myMac[2] == 0 && myMac[3] == 0 && myMac[4] == 0 && myMac[5] == 0) {
    Serial.println("WARNING: Hardware MAC failed to read. Using Synthetic MAC.");
    myMac[0] = 0x11;
    myMac[1] = 0x22;
    myMac[2] = 0x33;
    myMac[3] = 0x44;
    myMac[4] = 0x55;
    myMac[5] = 0x66;
  }

  Serial.println("\n========================================");
  Serial.println("  ECHOLINK - RECEIVER (CAR)");
  Serial.println("  DYNAMIC MESH MODE");
  Serial.print("  MY MAC: ");
  for(int i = 0; i < 6; i++) { 
    Serial.print(myMac[i], HEX); 
    if(i < 5) Serial.print(":"); 
  }
  Serial.println("\n========================================");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_register_recv_cb(onDataReceive);

  // Register Universal Broadcast Peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------

void loop() {
  // Motor Safety Timeouts
  if (steeringState != STEER_CENTER && millis() >= steeringStopTime) stopSteering();
  if (driveState != DRIVE_STOP && millis() - lastDriveCommandTime > COMMAND_TIMEOUT_MS) stopDrive();

  // Network Heartbeat (Piggyback Origin)
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = millis();
    broadcastTelemetry();
  }
  
  delay(5);
}
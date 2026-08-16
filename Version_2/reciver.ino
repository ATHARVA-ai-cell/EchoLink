//reciver
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1

#define NODE_TRANSMITTER 1
#define NODE_RELAY_A     2
#define NODE_RECEIVER    3

uint8_t RELAY_A_MAC[] = {
  0x8C, 0x94, 0xDF, 0x4D, 0x9C, 0x5C
};

const uint8_t STEER_IN1 = 26;
const uint8_t STEER_IN2 = 25;

const uint8_t DRIVE_IN1 = 22;
const uint8_t DRIVE_IN2 = 23;

const unsigned long COMMAND_TIMEOUT_MS = 700;
const unsigned long STEERING_PULSE_MS = 150;

#define PACKET_COMMAND          1
#define PACKET_COMMAND_ACK      2
#define PACKET_DIRECT_PROBE     3
#define PACKET_DIRECT_ACK       4
#define PACKET_RELAY_PROBE      5
#define PACKET_RELAY_LINK1_ACK  6
#define PACKET_RELAY_FINAL_ACK  7
#define PACKET_WINDOW_START     8
#define PACKET_TELEMETRY        9

struct CommandPacket {
  uint8_t type;
  uint8_t source;
  uint8_t destination;

  uint32_t packetID;

  char command[16];

  uint8_t hopCount;
  uint8_t ttl;
};

struct AckPacket {
  uint8_t type;
  uint8_t source;
  uint8_t destination;

  uint32_t packetID;

  int16_t measuredRSSI;
};

struct Metrics {
  uint32_t packetsSent;
  uint32_t packetsReceived;
  uint32_t bytesReceived;

  int16_t rssi;

  float pdr;
  float packetLoss;
  float latency;
  float throughput;
  float jitter;

  uint8_t hopCount;
};

struct TelemetryPacket {
  uint8_t type;
  uint8_t source;
  uint8_t destination;

  uint32_t windowID;

  Metrics direct;
  Metrics link1;
  Metrics link2;
  Metrics relayE2E;
};

enum DriveState {
  DRIVE_STOP,
  DRIVE_FORWARD,
  DRIVE_BACKWARD
};

enum SteeringState {
  STEER_CENTER,
  STEER_LEFT,
  STEER_RIGHT
};

DriveState driveState = DRIVE_STOP;

SteeringState steeringState =
  STEER_CENTER;

unsigned long lastDriveCommandTime = 0;

unsigned long steeringStopTime = 0;

// ------------------------------------------------------------
// METRICS RECEIVED
// ------------------------------------------------------------

TelemetryPacket latestTelemetry = {};

bool telemetryAvailable = false;

// ------------------------------------------------------------
// MOTOR
// ------------------------------------------------------------

void stopDrive()
{
  digitalWrite(
    DRIVE_IN1,
    LOW
  );

  digitalWrite(
    DRIVE_IN2,
    LOW
  );

  driveState =
    DRIVE_STOP;
}

void driveForward()
{
  digitalWrite(
    DRIVE_IN1,
    HIGH
  );

  digitalWrite(
    DRIVE_IN2,
    LOW
  );

  driveState =
    DRIVE_FORWARD;

  lastDriveCommandTime =
    millis();
}

void driveBackward()
{
  digitalWrite(
    DRIVE_IN1,
    LOW
  );

  digitalWrite(
    DRIVE_IN2,
    HIGH
  );

  driveState =
    DRIVE_BACKWARD;

  lastDriveCommandTime =
    millis();
}

void stopSteering()
{
  digitalWrite(
    STEER_IN1,
    LOW
  );

  digitalWrite(
    STEER_IN2,
    LOW
  );

  steeringState =
    STEER_CENTER;
}

void steerLeft()
{
  digitalWrite(
    STEER_IN1,
    HIGH
  );

  digitalWrite(
    STEER_IN2,
    LOW
  );

  steeringState =
    STEER_LEFT;

  steeringStopTime =
    millis() +
    STEERING_PULSE_MS;
}

void steerRight()
{
  digitalWrite(
    STEER_IN1,
    LOW
  );

  digitalWrite(
    STEER_IN2,
    HIGH
  );

  steeringState =
    STEER_RIGHT;

  steeringStopTime =
    millis() +
    STEERING_PULSE_MS;
}

void emergencyStop()
{
  stopDrive();
  stopSteering();
}

void processCommand(
  const char *command
)
{
  if (
    strcmp(command, "FORWARD") == 0
  )
    driveForward();

  else if (
    strcmp(command, "BACKWARD") == 0
  )
    driveBackward();

  else if (
    strcmp(command, "LEFT") == 0
  )
    steerLeft();

  else if (
    strcmp(command, "RIGHT") == 0
  )
    steerRight();

  else if (
    strcmp(command, "STOP") == 0
  )
    emergencyStop();
}

// ------------------------------------------------------------
// COMMAND ACK
// ------------------------------------------------------------

void sendCommandACK(
  CommandPacket &received
)
{
  CommandPacket ack = {};

  ack.type =
    PACKET_COMMAND_ACK;

  ack.source =
    NODE_RECEIVER;

  ack.destination =
    NODE_RELAY_A;

  ack.packetID =
    received.packetID;

  strncpy(
    ack.command,
    received.command,
    sizeof(ack.command) - 1
  );

  ack.hopCount =
    received.hopCount;

  ack.ttl = 5;

  esp_now_send(
    RELAY_A_MAC,
    (uint8_t *)&ack,
    sizeof(ack)
  );
}

// ------------------------------------------------------------
// DIRECT ACK
// ------------------------------------------------------------

void sendDirectACK(
  CommandPacket &probe,
  int measuredRSSI
)
{
  AckPacket ack = {};

  ack.type =
    PACKET_DIRECT_ACK;

  ack.source =
    NODE_RECEIVER;

  ack.destination =
    NODE_TRANSMITTER;

  ack.packetID =
    probe.packetID;

  ack.measuredRSSI =
    measuredRSSI;

  uint8_t TX_MAC[] = {
    0x4C, 0xC3, 0x82, 0xC4, 0xE0, 0x10
  };

  if (!esp_now_is_peer_exist(TX_MAC))
  {
    esp_now_peer_info_t peerInfo = {};

    memcpy(
      peerInfo.peer_addr,
      TX_MAC,
      6
    );

    peerInfo.channel =
      WIFI_CHANNEL;

    peerInfo.encrypt =
      false;

    esp_now_add_peer(
      &peerInfo
    );
  }

  esp_now_send(
    TX_MAC,
    (uint8_t *)&ack,
    sizeof(ack)
  );
}

// ------------------------------------------------------------
// RELAY FINAL ACK
// ------------------------------------------------------------

void sendRelayFinalACK(
  CommandPacket &probe,
  int measuredRSSI
)
{
  AckPacket ack = {};

  ack.type =
    PACKET_RELAY_FINAL_ACK;

  ack.source =
    NODE_RECEIVER;

  ack.destination =
    NODE_RELAY_A;

  ack.packetID =
    probe.packetID;

  ack.measuredRSSI =
    measuredRSSI;

  esp_now_send(
    RELAY_A_MAC,
    (uint8_t *)&ack,
    sizeof(ack)
  );
}

// ------------------------------------------------------------
// TELEMETRY DISPLAY
// ------------------------------------------------------------

void printMetric(
  const char *title,
  Metrics &m
)
{
  Serial.println();
  Serial.println(title);
  Serial.println("--------------------------------");

  Serial.print("RSSI              : ");
  Serial.print(m.rssi);
  Serial.println(" dBm");

  Serial.print("Packets Sent      : ");
  Serial.println(m.packetsSent);

  Serial.print("Packets Received  : ");
  Serial.println(m.packetsReceived);

  Serial.print("PDR               : ");
  Serial.print(m.pdr, 2);
  Serial.println(" %");

  Serial.print("Packet Loss       : ");
  Serial.print(m.packetLoss, 2);
  Serial.println(" %");

  Serial.print("Average Latency   : ");
  Serial.print(m.latency, 2);
  Serial.println(" ms");

  Serial.print("Throughput        : ");
  Serial.print(m.throughput, 2);
  Serial.println(" kbps");

  Serial.print("Jitter            : ");
  Serial.print(m.jitter, 2);
  Serial.println(" ms");

  Serial.print("Hop Count         : ");
  Serial.println(m.hopCount);
}

void printTelemetry()
{
  Serial.println();
  Serial.println();
  Serial.println("============================================================");
  Serial.println("                 ECHOLINK TELEMETRY");
  Serial.println("                    RECEIVER / RC CAR");
  Serial.println("============================================================");

  Serial.print("Measurement Window : ");
  Serial.println(
    latestTelemetry.windowID
  );

  printMetric(
    "PATH A: DIRECT TX -> RX",
    latestTelemetry.direct
  );

  printMetric(
    "PATH B / LINK 1: TX -> RELAY-A",
    latestTelemetry.link1
  );

  printMetric(
    "PATH B / LINK 2: RELAY-A -> RX",
    latestTelemetry.link2
  );

  printMetric(
    "PATH B / END-TO-END: TX -> RELAY-A -> RX",
    latestTelemetry.relayE2E
  );

  Serial.println();
  Serial.println("Latency = RTT / 2");
  Serial.println("Throughput = received bytes / measurement window");

  Serial.println("============================================================");
}

// ------------------------------------------------------------
// RECEIVE
// ------------------------------------------------------------

void onDataReceive(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
)
{
  if (len < 1)
    return;

  uint8_t type =
    data[0];

  // ----------------------------------------------------------
  // COMMAND
  // ----------------------------------------------------------

  if (type == PACKET_COMMAND)
  {
    if (len != sizeof(CommandPacket))
      return;

    CommandPacket packet;

    memcpy(
      &packet,
      data,
      sizeof(packet)
    );

    processCommand(
      packet.command
    );

    Serial.println();
    Serial.println("================================");
    Serial.println("       RECEIVER / RC CAR");
    Serial.println("================================");

    Serial.print("Packet ID : ");
    Serial.println(
      packet.packetID
    );

    Serial.print("Command   : ");
    Serial.println(
      packet.command
    );

    Serial.print("RSSI      : ");
    Serial.print(
      info->rx_ctrl->rssi
    );
    Serial.println(" dBm");

    Serial.print("Hop Count : ");
    Serial.println(
      packet.hopCount
    );

    return;
  }

  // ----------------------------------------------------------
  // WINDOW START
  // ----------------------------------------------------------

  if (type == PACKET_WINDOW_START)
  {
    return;
  }

  // ----------------------------------------------------------
  // DIRECT PROBE
  // ----------------------------------------------------------

  if (type == PACKET_DIRECT_PROBE)
  {
    if (len != sizeof(CommandPacket))
      return;

    CommandPacket probe;

    memcpy(
      &probe,
      data,
      sizeof(probe)
    );

    int measuredRSSI =
      info->rx_ctrl->rssi;

    sendDirectACK(
      probe,
      measuredRSSI
    );

    return;
  }

  // ----------------------------------------------------------
  // RELAY PROBE
  // ----------------------------------------------------------

  if (type == PACKET_RELAY_PROBE)
  {
    if (len != sizeof(CommandPacket))
      return;

    CommandPacket probe;

    memcpy(
      &probe,
      data,
      sizeof(probe)
    );

    int measuredRSSI =
      info->rx_ctrl->rssi;

    sendRelayFinalACK(
      probe,
      measuredRSSI
    );

    return;
  }

  // ----------------------------------------------------------
  // TELEMETRY
  // ----------------------------------------------------------

  if (type == PACKET_TELEMETRY)
  {
    if (len != sizeof(TelemetryPacket))
      return;

    memcpy(
      &latestTelemetry,
      data,
      sizeof(latestTelemetry)
    );

    telemetryAvailable =
      true;

    printTelemetry();

    return;
  }
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------

void setup()
{
  Serial.begin(115200);

  delay(1000);

  pinMode(
    STEER_IN1,
    OUTPUT
  );

  pinMode(
    STEER_IN2,
    OUTPUT
  );

  pinMode(
    DRIVE_IN1,
    OUTPUT
  );

  pinMode(
    DRIVE_IN2,
    OUTPUT
  );

  emergencyStop();

  WiFi.mode(
    WIFI_STA
  );

  esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  Serial.println();
  Serial.println("========================================");
  Serial.println("          ECHOLINK - RECEIVER");
  Serial.println("          RC CAR NODE");
  Serial.println("          ESP32 CORE 3.3.11");
  Serial.println("========================================");

  Serial.print("MAC: ");
  Serial.println(
    WiFi.macAddress()
  );

  if (
    esp_now_init()
    != ESP_OK
  )
  {
    Serial.println(
      "ESP-NOW INIT FAILED"
    );

    return;
  }

  esp_now_register_recv_cb(
    onDataReceive
  );

  if (
    !esp_now_is_peer_exist(
      RELAY_A_MAC
    )
  )
  {
    esp_now_peer_info_t peerInfo = {};

    memcpy(
      peerInfo.peer_addr,
      RELAY_A_MAC,
      6
    );

    peerInfo.channel =
      WIFI_CHANNEL;

    peerInfo.encrypt =
      false;

    if (
      esp_now_add_peer(
        &peerInfo
      ) != ESP_OK
    )
    {
      Serial.println(
        "RELAY-A PEER FAILED"
      );

      return;
    }
  }

  Serial.println(
    "RELAY-A PEER : READY"
  );

  Serial.println(
    "ESP-NOW      : READY"
  );

  Serial.println(
    "MOTOR SAFETY : ACTIVE"
  );

  Serial.println(
    "TELEMETRY    : READY"
  );

  Serial.println("========================================");
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------

void loop()
{
  if (
    steeringState != STEER_CENTER &&
    millis() >= steeringStopTime
  )
  {
    stopSteering();
  }

  if (
    driveState != DRIVE_STOP &&
    millis() -
    lastDriveCommandTime >
    COMMAND_TIMEOUT_MS
  )
  {
    stopDrive();

    Serial.println(
      "[SAFETY] Drive timeout -> STOP"
    );
  }

  delay(2);
}
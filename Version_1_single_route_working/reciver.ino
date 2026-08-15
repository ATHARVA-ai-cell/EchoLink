#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>


// ============================================================
// CONFIGURATION
// ============================================================

#define WIFI_CHANNEL 1

#define NODE_TRANSMITTER 1
#define NODE_RELAY_A     2
#define NODE_RECEIVER    3


// ============================================================
// RELAY-A MAC
// ============================================================

uint8_t RELAY_A_MAC[] =
{
  0x8C, 0x94, 0xDF, 0x4D, 0x9C, 0x5C
};


// ============================================================
// MOTOR GPIO
// ============================================================

// Steering motor
const uint8_t STEER_IN1 = 26;
const uint8_t STEER_IN2 = 25;


// Drive motor
const uint8_t DRIVE_IN1 = 22;
const uint8_t DRIVE_IN2 = 23;


// ============================================================
// SAFETY
// ============================================================

const unsigned long COMMAND_TIMEOUT_MS =
  700;


// ============================================================
// STEERING
// ============================================================

const unsigned long STEERING_PULSE_MS =
  150;


// ============================================================
// PACKET TYPES
// ============================================================

#define PACKET_COMMAND   1
#define PACKET_ACK       2
#define PACKET_TELEMETRY 3


// ============================================================
// PACKET
// ============================================================

struct Packet
{
  uint8_t type;

  uint8_t source;

  uint8_t destination;

  uint32_t packetID;

  char command[16];

  uint8_t hopCount;

  uint8_t ttl;

  uint32_t timestamp;

  int rssiLink1;

  int rssiLink2;

  uint32_t packetsReceived;

  uint32_t packetsLost;
};


// ============================================================
// MOTOR STATES
// ============================================================

enum DriveState
{
  DRIVE_STOP,
  DRIVE_FORWARD,
  DRIVE_BACKWARD
};


enum SteeringState
{
  STEER_CENTER,
  STEER_LEFT,
  STEER_RIGHT
};


DriveState driveState =
  DRIVE_STOP;


SteeringState steeringState =
  STEER_CENTER;


// ============================================================
// TIMERS
// ============================================================

unsigned long lastDriveCommandTime =
  0;

unsigned long steeringStopTime =
  0;


// ============================================================
// NETWORK
// ============================================================

int rssiFromRelay = 0;

uint32_t packetsReceived = 0;

uint32_t lastPacketID = 0;


// ============================================================
// MOTOR FUNCTIONS
// ============================================================

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


// ============================================================
// STEERING
// ============================================================

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


// ============================================================
// SEND ACK
// ============================================================

void sendACK(
  Packet received
)
{
  Packet ack = {};

  ack.type =
    PACKET_ACK;

  ack.source =
    NODE_RECEIVER;

  ack.destination =
    NODE_TRANSMITTER;

  ack.packetID =
    received.packetID;


  strncpy(
    ack.command,
    received.command,
    sizeof(ack.command) - 1
  );


  ack.hopCount =
    received.hopCount;


  ack.ttl =
    5;


  ack.timestamp =
    received.timestamp;


  esp_now_send(
    RELAY_A_MAC,
    (uint8_t *)&ack,
    sizeof(ack)
  );
}


// ============================================================
// SEND TELEMETRY
// ============================================================

void sendTelemetry()
{
  Packet telemetry = {};

  telemetry.type =
    PACKET_TELEMETRY;

  telemetry.source =
    NODE_RECEIVER;

  telemetry.destination =
    NODE_RELAY_A;

  telemetry.packetID =
    lastPacketID;


  telemetry.rssiLink2 =
    rssiFromRelay;


  telemetry.packetsReceived =
    packetsReceived;


  telemetry.timestamp =
    millis();


  esp_now_send(
    RELAY_A_MAC,
    (uint8_t *)&telemetry,
    sizeof(telemetry)
  );
}


// ============================================================
// PROCESS COMMAND
// ============================================================

void processCommand(
  const char *command
)
{
  if (
    strcmp(
      command,
      "FORWARD"
    ) == 0
  )
  {
    driveForward();
  }


  else if (
    strcmp(
      command,
      "BACKWARD"
    ) == 0
  )
  {
    driveBackward();
  }


  else if (
    strcmp(
      command,
      "LEFT"
    ) == 0
  )
  {
    steerLeft();
  }


  else if (
    strcmp(
      command,
      "RIGHT"
    ) == 0
  )
  {
    steerRight();
  }


  else if (
    strcmp(
      command,
      "STOP"
    ) == 0
  )
  {
    emergencyStop();
  }
}


// ============================================================
// RECEIVE CALLBACK
// ============================================================

void onDataReceive(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
)
{
  if (
    len != sizeof(Packet)
  )
  {
    Serial.println(
      "[RECEIVER] Invalid packet"
    );

    return;
  }


  Packet packet;

  memcpy(
    &packet,
    data,
    sizeof(packet)
  );


  int incomingRSSI =
    info->rx_ctrl->rssi;


  // ==========================================================
  // COMMAND FROM RELAY-A
  // ==========================================================

  if (
    packet.source ==
    NODE_TRANSMITTER
    &&
    packet.type ==
    PACKET_COMMAND
  )
  {
    rssiFromRelay =
      incomingRSSI;


    packetsReceived++;

    lastPacketID =
      packet.packetID;


    Serial.println();

    Serial.println(
      "================================"
    );

    Serial.println(
      "       RECEIVER / RC CAR"
    );

    Serial.println(
      "================================"
    );


    Serial.print(
      "Packet ID : "
    );

    Serial.println(
      packet.packetID
    );


    Serial.print(
      "Command   : "
    );

    Serial.println(
      packet.command
    );


    Serial.print(
      "RSSI      : "
    );

    Serial.print(
      rssiFromRelay
    );

    Serial.println(
      " dBm"
    );


    Serial.print(
      "Hop Count : "
    );

    Serial.println(
      packet.hopCount
    );


    // Execute motor command

    processCommand(
      packet.command
    );


    // ACK

    sendACK(
      packet
    );


    // Telemetry

    sendTelemetry();
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);


  // ==========================================================
  // MOTOR GPIO
  // ==========================================================

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


  // ==========================================================
  // WIFI / ESP-NOW
  // ==========================================================

  WiFi.mode(
    WIFI_STA
  );


  esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );


  Serial.println();

  Serial.println(
    "================================"
  );

  Serial.println(
    "    ESP32 RECEIVER - RC CAR"
  );

  Serial.println(
    "================================"
  );


  Serial.print(
    "RECEIVER MAC: "
  );

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


  // ==========================================================
  // ADD RELAY-A
  // ==========================================================

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


  Serial.println();

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
    "================================"
  );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // Steering timeout
  // ----------------------------------------------------------

  if (
    steeringState != STEER_CENTER
    &&
    millis() >= steeringStopTime
  )
  {
    stopSteering();
  }


  // ----------------------------------------------------------
  // Drive safety timeout
  // ----------------------------------------------------------

  if (
    driveState != DRIVE_STOP
    &&
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
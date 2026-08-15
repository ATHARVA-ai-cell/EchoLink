#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ============================================================
// CONFIGURATION
// ============================================================

#define WIFI_CHANNEL 1

// ============================================================
// REMOTE CONTROL BUTTONS
// ============================================================

#define FORWARD_BUTTON   13
#define BACKWARD_BUTTON  14
#define LEFT_BUTTON      27
#define RIGHT_BUTTON     33


// ============================================================
// RELAY-A MAC
// ============================================================

uint8_t RELAY_A_MAC[] = {
  0x8C, 0x94, 0xDF, 0x4D, 0x9C, 0x5C
};


// ============================================================
// NODE IDs
// ============================================================

#define NODE_TRANSMITTER 1
#define NODE_RELAY_A     2
#define NODE_RECEIVER    3


// ============================================================
// PACKET TYPES
// ============================================================

#define PACKET_COMMAND   1
#define PACKET_ACK       2
#define PACKET_TELEMETRY 3


// ============================================================
// PACKET STRUCTURE
// MUST BE IDENTICAL TO RELAY AND RECEIVER
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
// VARIABLES
// ============================================================

uint32_t packetCounter = 0;

uint32_t packetsSent = 0;

uint32_t packetsAcked = 0;

int lastRSSILink1 = 0;

int lastRSSILink2 = 0;


// ============================================================
// CURRENT COMMAND
// ============================================================

String lastCommand = "STOP";


// ============================================================
// SEND CALLBACK
// Arduino ESP32 Core 3.x / 3.3.11
// ============================================================

void onDataSent(
  const esp_now_send_info_t *info,
  esp_now_send_status_t status
)
{
  Serial.print("[ESP-NOW] Send status: ");

  if (status == ESP_NOW_SEND_SUCCESS)
  {
    Serial.println("SUCCESS");
  }
  else
  {
    Serial.println("FAILED");
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
  if (len != sizeof(Packet))
  {
    Serial.println("[ERROR] Invalid packet size");
    return;
  }

  Packet packet;

  memcpy(
    &packet,
    data,
    sizeof(packet)
  );


  // ==========================================================
  // ACK
  // ==========================================================

  if (packet.type == PACKET_ACK)
  {
    packetsAcked++;

    Serial.println();
    Serial.println(
      "************ ACK RECEIVED ************"
    );

    Serial.print("Packet ID     : ");
    Serial.println(packet.packetID);

    Serial.print("Command       : ");
    Serial.println(packet.command);

    Serial.println("End-to-end    : SUCCESS");

    Serial.println(
      "***************************************"
    );
  }


  // ==========================================================
  // TELEMETRY
  // ==========================================================

  else if (packet.type == PACKET_TELEMETRY)
  {
    lastRSSILink1 = packet.rssiLink1;

    lastRSSILink2 = packet.rssiLink2;

    Serial.println();
    Serial.println();

    Serial.println(
      "========================================"
    );

    Serial.println(
      "        ESP-NOW NETWORK STATUS"
    );

    Serial.println(
      "========================================"
    );


    Serial.println();

    Serial.println("NODES");

    Serial.println(
      "----------------------------------------"
    );

    Serial.println(
      "TRANSMITTER : ONLINE"
    );

    Serial.println(
      "RELAY-A     : ONLINE"
    );

    Serial.println(
      "RECEIVER    : ONLINE"
    );


    Serial.println();

    Serial.println("ROUTE");

    Serial.println(
      "----------------------------------------"
    );

    Serial.println(
      "TRANSMITTER -> RELAY-A -> RECEIVER"
    );


    Serial.println();

    Serial.println("LINK 1");

    Serial.println(
      "----------------------------------------"
    );

    Serial.println(
      "TRANSMITTER -> RELAY-A"
    );

    Serial.print("RSSI          : ");

    Serial.print(
      packet.rssiLink1
    );

    Serial.println(" dBm");


    Serial.println();

    Serial.println("LINK 2");

    Serial.println(
      "----------------------------------------"
    );

    Serial.println(
      "RELAY-A -> RECEIVER"
    );

    Serial.print("RSSI          : ");

    Serial.print(
      packet.rssiLink2
    );

    Serial.println(" dBm");


    Serial.println();

    Serial.println("PACKET STATISTICS");

    Serial.println(
      "----------------------------------------"
    );

    Serial.print("Packets Sent      : ");

    Serial.println(
      packetsSent
    );

    Serial.print("Packets ACKed     : ");

    Serial.println(
      packetsAcked
    );

    Serial.print("Packets Received  : ");

    Serial.println(
      packet.packetsReceived
    );

    Serial.print("Packets Lost      : ");

    Serial.println(
      packet.packetsLost
    );


    uint32_t total =
      packet.packetsReceived +
      packet.packetsLost;


    if (total > 0)
    {
      float deliveryRatio =
        ((float)packet.packetsReceived /
        total) * 100.0;

      Serial.print(
        "Delivery Ratio    : "
      );

      Serial.print(
        deliveryRatio,
        2
      );

      Serial.println(" %");
    }


    Serial.println();

    Serial.println("LATENCY");

    Serial.println(
      "----------------------------------------"
    );


    uint32_t latency =
      millis() -
      packet.timestamp;


    Serial.print(
      "Approx. RTT       : "
    );

    Serial.print(
      latency
    );

    Serial.println(" ms");


    Serial.println();

    Serial.println(
      "NETWORK STATUS    : CONNECTED"
    );

    Serial.println(
      "========================================"
    );
  }
}


// ============================================================
// ADD RELAY-A
// ============================================================

void addRelayPeer()
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
      "[ERROR] RELAY-A peer failed"
    );
  }
  else
  {
    Serial.println(
      "[ESP-NOW] RELAY-A peer added"
    );
  }
}


// ============================================================
// SEND COMMAND
// ============================================================

void sendCommand(
  const char *command
)
{
  Packet packet = {};

  packet.type =
    PACKET_COMMAND;

  packet.source =
    NODE_TRANSMITTER;

  packet.destination =
    NODE_RECEIVER;

  packet.packetID =
    ++packetCounter;

  strncpy(
    packet.command,
    command,
    sizeof(packet.command) - 1
  );

  packet.hopCount =
    0;

  packet.ttl =
    5;

  packet.timestamp =
    millis();


  packetsSent++;


  esp_err_t result =
    esp_now_send(
      RELAY_A_MAC,
      (uint8_t *)&packet,
      sizeof(packet)
    );


  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    "          REMOTE COMMAND"
  );

  Serial.println(
    "========================================"
  );


  Serial.print(
    "Command       : "
  );

  Serial.println(
    command
  );


  Serial.print(
    "Packet ID     : "
  );

  Serial.println(
    packet.packetID
  );


  Serial.print(
    "Route         : "
  );

  Serial.println(
    "TRANSMITTER -> RELAY-A -> RECEIVER"
  );


  Serial.print(
    "Send result   : "
  );


  if (result == ESP_OK)
  {
    Serial.println(
      "OK"
    );
  }
  else
  {
    Serial.println(
      "ERROR"
    );
  }


  Serial.println(
    "========================================"
  );
}


// ============================================================
// READ BUTTONS
// ============================================================

void readButtons()
{
  bool forwardPressed =
    digitalRead(FORWARD_BUTTON) == LOW;

  bool backwardPressed =
    digitalRead(BACKWARD_BUTTON) == LOW;

  bool leftPressed =
    digitalRead(LEFT_BUTTON) == LOW;

  bool rightPressed =
    digitalRead(RIGHT_BUTTON) == LOW;


  String newCommand = "";


  // ----------------------------------------------------------
  // DRIVE
  // ----------------------------------------------------------

  if (forwardPressed && !backwardPressed)
  {
    newCommand = "FORWARD";
  }

  else if (backwardPressed && !forwardPressed)
  {
    newCommand = "BACKWARD";
  }


  // ----------------------------------------------------------
  // STEERING
  // ----------------------------------------------------------

  else if (leftPressed && !rightPressed)
  {
    newCommand = "LEFT";
  }

  else if (rightPressed && !leftPressed)
  {
    newCommand = "RIGHT";
  }


  // ----------------------------------------------------------
  // CONFLICT
  // ----------------------------------------------------------

  else if (
    (forwardPressed && backwardPressed) ||
    (leftPressed && rightPressed)
  )
  {
    newCommand = "STOP";
  }


  // ----------------------------------------------------------
  // NO BUTTON
  // ----------------------------------------------------------

  else
  {
    newCommand = "STOP";
  }


  // ----------------------------------------------------------
  // SEND ONLY WHEN COMMAND CHANGES
  // ----------------------------------------------------------

  if (newCommand != lastCommand)
  {
    sendCommand(
      newCommand.c_str()
    );

    lastCommand = newCommand;
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);


  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    " ESP32 RC CAR - PHYSICAL REMOTE"
  );

  Serial.println(
    "========================================"
  );


  // ==========================================================
  // BUTTON CONFIGURATION
  // ==========================================================

  pinMode(
    FORWARD_BUTTON,
    INPUT_PULLUP
  );

  pinMode(
    BACKWARD_BUTTON,
    INPUT_PULLUP
  );

  pinMode(
    LEFT_BUTTON,
    INPUT_PULLUP
  );

  pinMode(
    RIGHT_BUTTON,
    INPUT_PULLUP
  );


  Serial.println(
    "[REMOTE] Buttons initialized"
  );

  Serial.println(
    "Forward  -> GPIO 13"
  );

  Serial.println(
    "Backward -> GPIO 14"
  );

  Serial.println(
    "Left     -> GPIO 27"
  );

  Serial.println(
    "Right    -> GPIO 33"
  );


  // ==========================================================
  // WIFI
  // ==========================================================

  WiFi.mode(
    WIFI_STA
  );


  esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );


  Serial.print(
    "TRANSMITTER MAC: "
  );

  Serial.println(
    WiFi.macAddress()
  );


  // ==========================================================
  // ESP-NOW
  // ==========================================================

  if (
    esp_now_init()
    != ESP_OK
  )
  {
    Serial.println(
      "[ERROR] ESP-NOW INIT FAILED"
    );

    return;
  }


  esp_now_register_send_cb(
    onDataSent
  );

  esp_now_register_recv_cb(
    onDataReceive
  );


  addRelayPeer();


  Serial.println();

  Serial.println(
    "ESP-NOW STATUS: READY"
  );


  Serial.println();

  Serial.println(
    "PHYSICAL REMOTE READY"
  );

  Serial.println(
    "========================================"
  );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  readButtons();

  delay(50);
}
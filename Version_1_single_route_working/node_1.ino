#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ============================================================
// ESP-NOW CHANNEL
// ============================================================

#define WIFI_CHANNEL 1


// ============================================================
// NODE IDs
// ============================================================

#define NODE_TRANSMITTER 1
#define NODE_RELAY_A     2
#define NODE_RECEIVER    3


// ============================================================
// MAC ADDRESSES
// ============================================================

// TRANSMITTER
uint8_t TRANSMITTER_MAC[] =
{
  0x4C,
  0xC3,
  0x82,
  0xC4,
  0xE0,
  0x10
};


// RECEIVER / RC CAR
uint8_t RECEIVER_MAC[] =
{
  0x68,
  0xFE,
  0x71,
  0x0D,
  0x3B,
  0x54
};


// ============================================================
// PACKET TYPES
// ============================================================

#define PACKET_COMMAND    1
#define PACKET_ACK        2
#define PACKET_TELEMETRY  3


// ============================================================
// PACKET STRUCTURE
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
// NETWORK VARIABLES
// ============================================================

int rssiFromTransmitter = 0;

int rssiFromReceiver = 0;

uint32_t packetsReceived = 0;

uint32_t packetsForwarded = 0;

uint32_t packetsLost = 0;

uint32_t lastPacketID = 0;


// ============================================================
// SEND CALLBACK
// Compatible with ESP32 Arduino Core 3.x
// ============================================================

void onDataSent(
  const esp_now_send_info_t *info,
  esp_now_send_status_t status
)
{
  Serial.print(
    "[ESP-NOW] SEND STATUS: "
  );

  if (
    status == ESP_NOW_SEND_SUCCESS
  )
  {
    Serial.println("SUCCESS");
  }
  else
  {
    Serial.println("FAILED");
  }
}


// ============================================================
// ADD ESP-NOW PEER
// ============================================================

bool addPeer(
  uint8_t *mac
)
{
  if (
    esp_now_is_peer_exist(mac)
  )
  {
    return true;
  }


  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    mac,
    6
  );


  peerInfo.channel =
    WIFI_CHANNEL;


  peerInfo.encrypt =
    false;


  esp_err_t result =
    esp_now_add_peer(
      &peerInfo
    );


  if (
    result == ESP_OK
  )
  {
    return true;
  }


  Serial.print(
    "[ERROR] Peer add failed: "
  );

  Serial.println(
    result
  );

  return false;
}


// ============================================================
// PRINT RSSI
// ============================================================

void printNetworkInfo()
{
  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    "           RELAY-A STATUS"
  );

  Serial.println(
    "========================================"
  );


  Serial.println();

  Serial.println(
    "NETWORK"
  );

  Serial.println(
    "----------------------------------------"
  );

  Serial.println(
    "TRANSMITTER -> RELAY-A -> RECEIVER"
  );


  Serial.println();

  Serial.println(
    "LINK 1"
  );

  Serial.println(
    "----------------------------------------"
  );

  Serial.print(
    "TRANSMITTER -> RELAY-A RSSI : "
  );

  Serial.print(
    rssiFromTransmitter
  );

  Serial.println(
    " dBm"
  );


  Serial.println();

  Serial.println(
    "LINK 2"
  );

  Serial.println(
    "----------------------------------------"
  );

  Serial.print(
    "RECEIVER -> RELAY-A RSSI    : "
  );

  Serial.print(
    rssiFromReceiver
  );

  Serial.println(
    " dBm"
  );


  Serial.println();

  Serial.println(
    "PACKETS"
  );

  Serial.println(
    "----------------------------------------"
  );

  Serial.print(
    "Packets Received : "
  );

  Serial.println(
    packetsReceived
  );


  Serial.print(
    "Packets Forwarded: "
  );

  Serial.println(
    packetsForwarded
  );


  Serial.print(
    "Packets Lost     : "
  );

  Serial.println(
    packetsLost
  );


  Serial.println(
    "========================================"
  );
}


// ============================================================
// RECEIVE CALLBACK
//
// ESP32 Arduino Core 3.3.11 uses:
// const esp_now_recv_info_t *info
// ============================================================

void onDataReceive(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
)
{
  // ----------------------------------------------------------
  // CHECK PACKET SIZE
  // ----------------------------------------------------------

  if (
    len != sizeof(Packet)
  )
  {
    Serial.println(
      "[ERROR] Invalid packet size"
    );

    return;
  }


  // ----------------------------------------------------------
  // COPY PACKET
  // ----------------------------------------------------------

  Packet packet;

  memcpy(
    &packet,
    data,
    sizeof(Packet)
  );


  // ----------------------------------------------------------
  // GET RSSI
  // ----------------------------------------------------------

  int incomingRSSI =
    info->rx_ctrl->rssi;


  // ==========================================================
  // COMMAND FROM TRANSMITTER
  // ==========================================================

  if (
    packet.source ==
    NODE_TRANSMITTER
    &&
    packet.type ==
    PACKET_COMMAND
  )
  {
    // Store RSSI of
    // TRANSMITTER -> RELAY-A

    rssiFromTransmitter =
      incomingRSSI;


    packetsReceived++;

    lastPacketID =
      packet.packetID;


    Serial.println();

    Serial.println(
      "========================================"
    );

    Serial.println(
      "       COMMAND RECEIVED AT RELAY-A"
    );

    Serial.println(
      "========================================"
    );


    Serial.print(
      "Packet ID     : "
    );

    Serial.println(
      packet.packetID
    );


    Serial.print(
      "Command       : "
    );

    Serial.println(
      packet.command
    );


    Serial.print(
      "RSSI          : "
    );

    Serial.print(
      incomingRSSI
    );

    Serial.println(
      " dBm"
    );


    Serial.print(
      "Hop Count     : "
    );

    Serial.println(
      packet.hopCount
    );


    Serial.print(
      "TTL           : "
    );

    Serial.println(
      packet.ttl
    );


    // --------------------------------------------------------
    // FORWARD PACKET TO RECEIVER
    // --------------------------------------------------------

    if (
      packet.ttl > 0
    )
    {
      packet.hopCount++;

      packet.ttl--;


      esp_err_t result =
        esp_now_send(
          RECEIVER_MAC,
          (uint8_t *)&packet,
          sizeof(Packet)
        );


      if (
        result == ESP_OK
      )
      {
        packetsForwarded++;


        Serial.println();

        Serial.println(
          "[FORWARD] RELAY-A -> RECEIVER"
        );

        Serial.println(
          "[FORWARD] Queued successfully"
        );
      }
      else
      {
        packetsLost++;


        Serial.println();

        Serial.println(
          "[FORWARD] FAILED"
        );

        Serial.print(
          "Error code: "
        );

        Serial.println(
          result
        );
      }
    }
    else
    {
      packetsLost++;

      Serial.println(
        "[DROP] TTL expired"
      );
    }


    printNetworkInfo();
  }


  // ==========================================================
  // ACK FROM RECEIVER
  // ==========================================================

  else if (
    packet.source ==
    NODE_RECEIVER
    &&
    packet.type ==
    PACKET_ACK
  )
  {
    rssiFromReceiver =
      incomingRSSI;


    Serial.println();

    Serial.println(
      "========================================"
    );

    Serial.println(
      "          ACK RECEIVED AT RELAY"
    );

    Serial.println(
      "========================================"
    );


    Serial.print(
      "Packet ID     : "
    );

    Serial.println(
      packet.packetID
    );


    Serial.print(
      "Command       : "
    );

    Serial.println(
      packet.command
    );


    Serial.print(
      "RSSI          : "
    );

    Serial.print(
      incomingRSSI
    );

    Serial.println(
      " dBm"
    );


    // --------------------------------------------------------
    // FORWARD ACK TO TRANSMITTER
    // --------------------------------------------------------

    esp_err_t result =
      esp_now_send(
        TRANSMITTER_MAC,
        (uint8_t *)&packet,
        sizeof(Packet)
      );


    if (
      result == ESP_OK
    )
    {
      Serial.println();

      Serial.println(
        "[ACK] Forwarded -> TRANSMITTER"
      );
    }
    else
    {
      Serial.println();

      Serial.println(
        "[ACK] Forward FAILED"
      );
    }
  }


  // ==========================================================
  // TELEMETRY FROM RECEIVER
  // ==========================================================

  else if (
    packet.source ==
    NODE_RECEIVER
    &&
    packet.type ==
    PACKET_TELEMETRY
  )
  {
    rssiFromReceiver =
      incomingRSSI;


    Serial.println();

    Serial.println(
      "========================================"
    );

    Serial.println(
      "       TELEMETRY RECEIVED AT RELAY"
    );

    Serial.println(
      "========================================"
    );


    Serial.print(
      "Packet ID          : "
    );

    Serial.println(
      packet.packetID
    );


    Serial.print(
      "Receiver RSSI      : "
    );

    Serial.print(
      incomingRSSI
    );

    Serial.println(
      " dBm"
    );


    Serial.print(
      "Packets Received   : "
    );

    Serial.println(
      packet.packetsReceived
    );


    Serial.print(
      "Packets Lost       : "
    );

    Serial.println(
      packet.packetsLost
    );


    // --------------------------------------------------------
    // UPDATE LINK-2 RSSI
    // --------------------------------------------------------

    packet.rssiLink2 =
      rssiFromReceiver;


    // --------------------------------------------------------
    // UPDATE LINK-1 RSSI
    // --------------------------------------------------------

    packet.rssiLink1 =
      rssiFromTransmitter;


    // --------------------------------------------------------
    // FORWARD TELEMETRY
    // --------------------------------------------------------

    esp_err_t result =
      esp_now_send(
        TRANSMITTER_MAC,
        (uint8_t *)&packet,
        sizeof(Packet)
      );


    if (
      result == ESP_OK
    )
    {
      Serial.println();

      Serial.println(
        "[TELEMETRY] Forwarded -> TRANSMITTER"
      );
    }
    else
    {
      Serial.println();

      Serial.println(
        "[TELEMETRY] Forward FAILED"
      );
    }


    printNetworkInfo();
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(
    115200
  );


  delay(
    1000
  );


  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    "        ESP32 RELAY-A"
  );

  Serial.println(
    "      Arduino Core 3.3.11"
  );

  Serial.println(
    "========================================"
  );


  // ==========================================================
  // WIFI STATION MODE
  // ==========================================================

  WiFi.mode(
    WIFI_STA
  );


  delay(
    100
  );


  // ==========================================================
  // SET ESP-NOW CHANNEL
  // ==========================================================

  esp_err_t channelResult =
    esp_wifi_set_channel(
      WIFI_CHANNEL,
      WIFI_SECOND_CHAN_NONE
    );


  if (
    channelResult == ESP_OK
  )
  {
    Serial.print(
      "[WiFi] Channel: "
    );

    Serial.println(
      WIFI_CHANNEL
    );
  }
  else
  {
    Serial.println(
      "[ERROR] Failed to set channel"
    );
  }


  // ==========================================================
  // PRINT MAC
  // ==========================================================

  Serial.print(
    "RELAY-A MAC: "
  );

  Serial.println(
    WiFi.macAddress()
  );


  // ==========================================================
  // INITIALIZE ESP-NOW
  // ==========================================================

  if (
    esp_now_init()
    != ESP_OK
  )
  {
    Serial.println();

    Serial.println(
      "[ERROR] ESP-NOW INIT FAILED"
    );

    return;
  }


  Serial.println(
    "[ESP-NOW] Initialized"
  );


  // ==========================================================
  // REGISTER CALLBACKS
  // ==========================================================

  esp_now_register_send_cb(
    onDataSent
  );


  esp_now_register_recv_cb(
    onDataReceive
  );


  // ==========================================================
  // ADD TRANSMITTER PEER
  // ==========================================================

  Serial.println();

  Serial.println(
    "[ESP-NOW] Adding TRANSMITTER..."
  );


  if (
    addPeer(
      TRANSMITTER_MAC
    )
  )
  {
    Serial.println(
      "[ESP-NOW] TRANSMITTER PEER: READY"
    );
  }
  else
  {
    Serial.println(
      "[ESP-NOW] TRANSMITTER PEER: FAILED"
    );
  }


  // ==========================================================
  // ADD RECEIVER PEER
  // ==========================================================

  Serial.println();

  Serial.println(
    "[ESP-NOW] Adding RECEIVER..."
  );


  if (
    addPeer(
      RECEIVER_MAC
    )
  )
  {
    Serial.println(
      "[ESP-NOW] RECEIVER PEER: READY"
    );
  }
  else
  {
    Serial.println(
      "[ESP-NOW] RECEIVER PEER: FAILED"
    );
  }


  // ==========================================================
  // READY
  // ==========================================================

  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    "         RELAY-A READY"
  );

  Serial.println(
    "========================================"
  );

  Serial.println(
    "Role       : RELAY / MULTI-HOP NODE"
  );

  Serial.println(
    "Channel    : 1"
  );

  Serial.println(
    "Encryption : OFF"
  );

  Serial.println(
    "Path       : TRANSMITTER <-> RELAY-A <-> RECEIVER"
  );

  Serial.println(
    "Status     : READY"
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
  delay(
    10
  );
}
//node 1
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1

#define NODE_TRANSMITTER 1
#define NODE_RELAY_A     2
#define NODE_RECEIVER    3 

uint8_t TRANSMITTER_MAC[] = {
  0x4C, 0xC3, 0x82, 0xC4, 0xE0, 0x10
};

uint8_t RECEIVER_MAC[] = {
  0x68, 0xFE, 0x71, 0x0D, 0x3B, 0x54
};

#define PACKET_COMMAND          1
#define PACKET_COMMAND_ACK      2
#define PACKET_DIRECT_PROBE     3
#define PACKET_DIRECT_ACK       4
#define PACKET_RELAY_PROBE      5
#define PACKET_RELAY_LINK1_ACK  6
#define PACKET_RELAY_FINAL_ACK  7
#define PACKET_WINDOW_START     8
#define PACKET_TELEMETRY        9

const unsigned long MEASUREMENT_WINDOW_MS = 10000;

// ------------------------------------------------------------
// PACKETS
// ------------------------------------------------------------

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

// ------------------------------------------------------------
// LINK 1
// ------------------------------------------------------------

//uint32_t link1Sent = 0;
uint32_t link1Received = 0;
uint32_t link1Bytes = 0;

int link1RSSI = 0;

uint32_t link1LatencyTotal = 0;
uint32_t link1LatencySamples = 0;

uint32_t link1JitterTotal = 0;
uint32_t link1JitterSamples = 0;

uint32_t lastLink1Latency = 0;

// ------------------------------------------------------------
// LINK 2
// ------------------------------------------------------------

uint32_t link2Sent = 0;
uint32_t link2Received = 0;
uint32_t link2Bytes = 0;

int link2RSSI = 0;

uint32_t link2LatencyTotal = 0;
uint32_t link2LatencySamples = 0;

uint32_t link2JitterTotal = 0;
uint32_t link2JitterSamples = 0;

uint32_t lastLink2Latency = 0;

// ------------------------------------------------------------
// LINK 1 OUTSTANDING
// ------------------------------------------------------------

//uint32_t link1OutstandingID = 0;

//unsigned long link1Start = 0;

//bool link1Waiting = false;

// ------------------------------------------------------------
// LINK 2 OUTSTANDING
// ------------------------------------------------------------

uint32_t link2OutstandingID = 0;

unsigned long link2Start = 0;

bool link2Waiting = false;

// ------------------------------------------------------------
// WINDOW
// ------------------------------------------------------------

uint32_t windowID = 0;

unsigned long windowStart = 0;

bool telemetryReceivedFromTX = false;

TelemetryPacket latestTXTelemetry = {};

// ------------------------------------------------------------
// PEER
// ------------------------------------------------------------

bool addPeer(uint8_t *mac)
{
  if (esp_now_is_peer_exist(mac))
    return true;

  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    mac,
    6
  );

  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  return esp_now_add_peer(
    &peerInfo
  ) == ESP_OK;
}

// ------------------------------------------------------------
// CALCULATIONS
// ------------------------------------------------------------

float calculatePDR(
  uint32_t sent,
  uint32_t received
)
{
  if (sent == 0)
    return 0;

  return (
    (float)received /
    (float)sent *
    100.0
  );
}

float calculateLoss(
  uint32_t sent,
  uint32_t received
)
{
  if (sent == 0)
    return 0;

  return 100.0 -
         calculatePDR(
           sent,
           received
         );
}

float calculateAverage(
  uint32_t total,
  uint32_t samples
)
{
  if (samples == 0)
    return 0;

  return (
    (float)total /
    (float)samples
  );
}

float calculateThroughput(
  uint32_t bytes,
  unsigned long elapsed
)
{
  if (elapsed == 0)
    return 0;

  return (
    ((float)bytes * 8.0) /
    ((float)elapsed / 1000.0) /
    1000.0
  );
}

Metrics makeMetrics(
  uint32_t sent,
  uint32_t received,
  uint32_t bytes,
  int rssi,
  uint32_t latencyTotal,
  uint32_t latencySamples,
  uint32_t jitterTotal,
  uint32_t jitterSamples,
  uint8_t hops
)
{
  Metrics m = {};

  unsigned long elapsed =
    millis() - windowStart;

  if (elapsed == 0)
    elapsed = 1;

  m.packetsSent = sent;
  m.packetsReceived = received;
  m.bytesReceived = bytes;

  m.rssi = rssi;

  m.pdr =
    calculatePDR(
      sent,
      received
    );

  m.packetLoss =
    calculateLoss(
      sent,
      received
    );

  m.latency =
    calculateAverage(
      latencyTotal,
      latencySamples
    );

  m.throughput =
    calculateThroughput(
      bytes,
      elapsed
    );

  m.jitter =
    calculateAverage(
      jitterTotal,
      jitterSamples
    );

  m.hopCount = hops;

  return m;
}

// ------------------------------------------------------------
// RESET
// ------------------------------------------------------------

void resetMeasurements()
{
  link1Received = 0;
  link1Bytes = 0;

  link1RSSI = 0;

  link1LatencyTotal = 0;
  link1LatencySamples = 0;

  link1JitterTotal = 0;
  link1JitterSamples = 0;

  lastLink1Latency = 0;

  link2Sent = 0;  link2Received = 0;
  link2Bytes = 0;

  link2RSSI = 0;

  link2LatencyTotal = 0;
  link2LatencySamples = 0;

  link2JitterTotal = 0;
  link2JitterSamples = 0;

  lastLink2Latency = 0;

  link2Waiting = false;
}

// ------------------------------------------------------------
// PRINT
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

void printCombinedTelemetry()
{
  Serial.println();
  Serial.println();
  Serial.println("============================================================");
  Serial.println("                 ECHOLINK TELEMETRY");
  Serial.println("                    NODE 1 / RELAY-A");
  Serial.println("============================================================");

  Serial.print("Measurement Window : ");
  Serial.println(
    latestTXTelemetry.windowID
  );

  printMetric(
    "PATH A: DIRECT TX -> RX",
    latestTXTelemetry.direct
  );

  printMetric(
    "PATH B / LINK 1: TX -> RELAY-A",
    latestTXTelemetry.link1
  );

  printMetric(
    "PATH B / LINK 2: RELAY-A -> RX",
    latestTXTelemetry.link2
  );

  printMetric(
    "PATH B / END-TO-END: TX -> RELAY-A -> RX",
    latestTXTelemetry.relayE2E
  );

  Serial.println();
  Serial.println("Latency = RTT / 2");
  Serial.println("Throughput = received bytes / measurement window");

  Serial.println("============================================================");
}

// ------------------------------------------------------------
// SEND FINAL RELAY TELEMETRY
// ------------------------------------------------------------

void sendCombinedTelemetry()
{
  TelemetryPacket report =
    latestTXTelemetry;

  report.source =
    NODE_RELAY_A;

  report.destination =
    NODE_TRANSMITTER;

  // report.link1 and report.link2 are already correct here —
  // they were set in the PACKET_TELEMETRY handler just before
  // this function was called.

  // Send complete report to TX
  esp_now_send(
    TRANSMITTER_MAC,
    (uint8_t *)&report,
    sizeof(report)
  );

  // Send complete report to RX
  report.destination =
    NODE_RECEIVER;

  esp_now_send(
    RECEIVER_MAC,
    (uint8_t *)&report,
    sizeof(report)
  );
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

    CommandPacket command;

    memcpy(
      &command,
      data,
      sizeof(command)
    );

    command.hopCount++;

    esp_now_send(
      RECEIVER_MAC,
      (uint8_t *)&command,
      sizeof(command)
    );

    return;
  }

  // ----------------------------------------------------------
  // WINDOW START
  // ----------------------------------------------------------

  if (type == PACKET_WINDOW_START)
  {
    if (len != sizeof(CommandPacket))
      return;

    CommandPacket p;

    memcpy(
      &p,
      data,
      sizeof(p)
    );

    windowID =
      p.packetID;

    windowStart =
      millis();

    resetMeasurements();

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

    // ------------------------------
    // LINK 1 RECEIVED
    // ------------------------------

    link1Received++;

    link1Bytes +=
      sizeof(probe);

    link1RSSI =
      info->rx_ctrl->rssi;

    // Send Link-1 ACK
    AckPacket ack = {};

    ack.type =
      PACKET_RELAY_LINK1_ACK;

    ack.source =
      NODE_RELAY_A;

    ack.destination =
      NODE_TRANSMITTER;

    ack.packetID =
      probe.packetID;

    ack.measuredRSSI =
      link1RSSI;

    esp_now_send(
      TRANSMITTER_MAC,
      (uint8_t *)&ack,
      sizeof(ack)
    );

    // ------------------------------
    // LINK 2 FORWARD
    // ------------------------------

    link2Sent++;

    link2OutstandingID =
      probe.packetID;

    link2Start =
      millis();

    link2Waiting = true;

    probe.hopCount++;

    esp_now_send(
      RECEIVER_MAC,
      (uint8_t *)&probe,
      sizeof(probe)
    );

    return;
  }

  // ----------------------------------------------------------
  // LINK 2 ACK
  // ----------------------------------------------------------

  if (type == PACKET_RELAY_FINAL_ACK)
  {
    if (len != sizeof(AckPacket))
      return;

    AckPacket ack;

    memcpy(
      &ack,
      data,
      sizeof(ack)
    );

    if (
      !link2Waiting ||
      ack.packetID !=
      link2OutstandingID
    )
    {
      return;
    }

    uint32_t rtt =
      millis() - link2Start;

    uint32_t latency =
      rtt / 2;

    link2Received++;

    link2Bytes +=
      sizeof(CommandPacket);

    link2RSSI =
      ack.measuredRSSI;

    link2LatencyTotal +=
      latency;

    link2LatencySamples++;

    if (
      link2LatencySamples > 1
    )
    {
      uint32_t diff =
        abs(
          (int)latency -
          (int)lastLink2Latency
        );

      link2JitterTotal += diff;
      link2JitterSamples++;
    }

    lastLink2Latency =
      latency;

    link2Waiting = false;

    // Forward final ACK to TX
    ack.source =
      NODE_RELAY_A;

    ack.destination =
      NODE_TRANSMITTER;

    ack.type =
      PACKET_RELAY_FINAL_ACK;

    esp_now_send(
      TRANSMITTER_MAC,
      (uint8_t *)&ack,
      sizeof(ack)
    );

    return;
  }

  // ----------------------------------------------------------
  // TELEMETRY FROM TX
  // ----------------------------------------------------------

  if (type == PACKET_TELEMETRY)
  {
    if (len != sizeof(TelemetryPacket))
      return;

    TelemetryPacket packet;

    memcpy(
      &packet,
      data,
      sizeof(packet)
    );

    latestTXTelemetry =
      packet;

    telemetryReceivedFromTX =
      true;

    // Replace TX's link data with our
    // actual relay measurements
latestTXTelemetry.link1 =
  makeMetrics(
    packet.relayE2E.packetsSent,
    link1Received,
    link1Bytes,
    link1RSSI,
    0, 0, 0, 0,   // don't use relay's own (always-empty) latency totals
    1
  );

latestTXTelemetry.link1.latency = packet.link1.latency;  // use TX-measured latency
latestTXTelemetry.link1.jitter  = packet.link1.jitter;    // use TX-measured jitter
    latestTXTelemetry.link2 =
      makeMetrics(
        link2Sent,
        link2Received,
        link2Bytes,
        link2RSSI,
        link2LatencyTotal,
        link2LatencySamples,
        link2JitterTotal,
        link2JitterSamples,
        1
      );

    sendCombinedTelemetry();

    printCombinedTelemetry();

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

  WiFi.mode(WIFI_STA);

  esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  Serial.println();
  Serial.println("========================================");
  Serial.println("          ECHOLINK - RELAY-A");
  Serial.println("          NODE 1");
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
    !addPeer(TRANSMITTER_MAC)
  )
  {
    Serial.println(
      "TRANSMITTER PEER FAILED"
    );

    return;
  }

  if (
    !addPeer(RECEIVER_MAC)
  )
  {
    Serial.println(
      "RECEIVER PEER FAILED"
    );

    return;
  }

  Serial.println(
    "TRANSMITTER PEER : READY"
  );

  Serial.println(
    "RECEIVER PEER    : READY"
  );

  Serial.println(
    "ESP-NOW          : READY"
  );

  Serial.println(
    "TELEMETRY        : READY"
  );

  Serial.println("========================================");
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------

void loop()
{
  delay(10);
}
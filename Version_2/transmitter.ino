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

uint8_t RELAY_A_MAC[] = {
  0x8C, 0x94, 0xDF, 0x4D, 0x9C, 0x5C
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
const unsigned long PROBE_INTERVAL_MS = 500;

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
// DIRECT
// ------------------------------------------------------------

uint32_t directProbeID = 0;
uint32_t directSent = 0;
uint32_t directReceived = 0;
uint32_t directBytes = 0;

int directRSSI = 0;

uint32_t directLatencyTotal = 0;
uint32_t directLatencySamples = 0;
uint32_t directJitterTotal = 0;
uint32_t directJitterSamples = 0;
uint32_t lastDirectLatency = 0;

uint32_t directOutstandingID = 0;
unsigned long directStart = 0;
bool directWaiting = false;

// ------------------------------------------------------------
// RELAY E2E
// ------------------------------------------------------------

uint32_t relayProbeID = 0;
uint32_t relaySent = 0;
uint32_t relayReceived = 0;
uint32_t relayBytes = 0;

uint32_t relayLatencyTotal = 0;
uint32_t relayLatencySamples = 0;
uint32_t relayJitterTotal = 0;
uint32_t relayJitterSamples = 0;
uint32_t lastRelayLatency = 0;

uint32_t relayOutstandingID = 0;
unsigned long relayStart = 0;
bool relayWaiting = false;
uint32_t link1LatencyTotal = 0;
uint32_t link1LatencySamples = 0;
uint32_t link1JitterTotal = 0;
uint32_t link1JitterSamples = 0;
uint32_t lastLink1Latency = 0;
// ------------------------------------------------------------
// LINK DATA FROM RELAY
// ------------------------------------------------------------

Metrics latestLink1 = {};
Metrics latestLink2 = {};

uint32_t latestWindowID = 0;

// ------------------------------------------------------------
// WINDOW
// ------------------------------------------------------------

uint32_t windowID = 0;
unsigned long windowStart = 0;
unsigned long lastProbeTime = 0;
bool nextProbeDirect = true;

// ------------------------------------------------------------
// COMMAND
// ------------------------------------------------------------

uint32_t commandID = 0;
String lastCommand = "";

// ------------------------------------------------------------
// HELPERS
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

float pdr(
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

float loss(
  uint32_t sent,
  uint32_t received
)
{
  if (sent == 0)
    return 0;

  return 100.0 -
         pdr(sent, received);
}

float average(
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

float throughput(
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
    pdr(sent, received);

  m.packetLoss =
    loss(sent, received);

  m.latency =
    average(
      latencyTotal,
      latencySamples
    );

  m.throughput =
    throughput(
      bytes,
      elapsed
    );

  m.jitter =
    average(
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
  directSent = 0;
  directReceived = 0;
  directBytes = 0;
  directRSSI = 0;
  directLatencyTotal = 0;
  directLatencySamples = 0;
  directJitterTotal = 0;
  directJitterSamples = 0;
  lastDirectLatency = 0;
  directWaiting = false;

  relaySent = 0;
  relayReceived = 0;
  relayBytes = 0;
  relayLatencyTotal = 0;
  relayLatencySamples = 0;
  relayJitterTotal = 0;
  relayJitterSamples = 0;
  lastRelayLatency = 0;
  relayWaiting = false;
link1LatencyTotal = 0;
link1LatencySamples = 0;
link1JitterTotal = 0;
link1JitterSamples = 0;
lastLink1Latency = 0;
  latestLink1 = {};
  latestLink2 = {};
}

// ------------------------------------------------------------
// WINDOW START
// ------------------------------------------------------------

void startWindow()
{
  windowID++;

  resetMeasurements();

  windowStart = millis();

  CommandPacket p = {};

  p.type = PACKET_WINDOW_START;
  p.source = NODE_TRANSMITTER;
  p.destination = NODE_RELAY_A;
  p.packetID = windowID;

  esp_now_send(
    RELAY_A_MAC,
    (uint8_t *)&p,
    sizeof(p)
  );

  p.destination = NODE_RECEIVER;

  esp_now_send(
    RECEIVER_MAC,
    (uint8_t *)&p,
    sizeof(p)
  );

  Serial.println();
  Serial.println("================================================");
  Serial.print("MEASUREMENT WINDOW #");
  Serial.println(windowID);
  Serial.println("Duration : 10 seconds");
  Serial.println("================================================");
}

// ------------------------------------------------------------
// DIRECT PROBE
// ------------------------------------------------------------

void sendDirectProbe()
{
  if (directWaiting)
    return;

  CommandPacket p = {};

  p.type = PACKET_DIRECT_PROBE;
  p.source = NODE_TRANSMITTER;
  p.destination = NODE_RECEIVER;
  p.packetID = ++directProbeID;
  p.hopCount = 0;
  p.ttl = 5;

  directSent++;

  directOutstandingID =
    p.packetID;

  directStart = millis();

  directWaiting = true;

  esp_now_send(
    RECEIVER_MAC,
    (uint8_t *)&p,
    sizeof(p)
  );
}

// ------------------------------------------------------------
// RELAY PROBE
// ------------------------------------------------------------

void sendRelayProbe()
{
  if (relayWaiting)
    return;

  CommandPacket p = {};

  p.type = PACKET_RELAY_PROBE;
  p.source = NODE_TRANSMITTER;
  p.destination = NODE_RECEIVER;
  p.packetID = ++relayProbeID;
  p.hopCount = 0;
  p.ttl = 5;

  relaySent++;

  relayOutstandingID =
    p.packetID;

  relayStart = millis();

  relayWaiting = true;

  esp_now_send(
    RELAY_A_MAC,
    (uint8_t *)&p,
    sizeof(p)
  );
}

// ------------------------------------------------------------
// COMMAND
// ------------------------------------------------------------

void sendCommand(
  const char *command
)
{
  CommandPacket p = {};

  p.type = PACKET_COMMAND;
  p.source = NODE_TRANSMITTER;
  p.destination = NODE_RECEIVER;
  p.packetID = ++commandID;

  strncpy(
    p.command,
    command,
    sizeof(p.command) - 1
  );

  p.hopCount = 0;
  p.ttl = 5;

  esp_now_send(
    RELAY_A_MAC,
    (uint8_t *)&p,
    sizeof(p)
  );

  Serial.println();
  Serial.println("[TX] COMMAND SENT");

  Serial.print("Command   : ");
  Serial.println(command);

  Serial.print("Packet ID : ");
  Serial.println(p.packetID);
}

// ------------------------------------------------------------
// BUTTONS
// ------------------------------------------------------------

void readButtons()
{
  bool f =
    digitalRead(FORWARD_BUTTON) == LOW;

  bool b =
    digitalRead(BACKWARD_BUTTON) == LOW;

  bool l =
    digitalRead(LEFT_BUTTON) == LOW;

  bool r =
    digitalRead(RIGHT_BUTTON) == LOW;

  String command;

  if (f && !b)
    command = "FORWARD";
  else if (b && !f)
    command = "BACKWARD";
  else if (l && !r)
    command = "LEFT";
  else if (r && !l)
    command = "RIGHT";
  else
    command = "STOP";

  if (command != lastCommand)
  {
    sendCommand(
      command.c_str()
    );

    lastCommand = command;
  }
}

// ------------------------------------------------------------
// TELEMETRY PRINT
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
  unsigned long elapsed =
    millis() - windowStart;

  Metrics direct =
    makeMetrics(
      directSent,
      directReceived,
      directBytes,
      directRSSI,
      directLatencyTotal,
      directLatencySamples,
      directJitterTotal,
      directJitterSamples,
      1
    );

  Metrics relay =
    makeMetrics(
      relaySent,
      relayReceived,
      relayBytes,
      0,
      relayLatencyTotal,
      relayLatencySamples,
      relayJitterTotal,
      relayJitterSamples,
      2
    );

  Serial.println();
  Serial.println();
  Serial.println("============================================================");
  Serial.println("                 ECHOLINK TELEMETRY");
  Serial.println("============================================================");

  Serial.print("Measurement Window : ");
  Serial.println(windowID);

  Serial.print("Window Elapsed     : ");
  Serial.print(elapsed);
  Serial.println(" ms");

  printMetric(
    "PATH A: DIRECT TX -> RX",
    direct
  );

  printMetric(
    "PATH B / LINK 1: TX -> RELAY-A",
    latestLink1
  );

  printMetric(
    "PATH B / LINK 2: RELAY-A -> RX",
    latestLink2
  );

  printMetric(
    "PATH B / END-TO-END: TX -> RELAY-A -> RX",
    relay
  );

  Serial.println();
  Serial.println("Latency = RTT / 2");
  Serial.println("Throughput = received bytes / measurement window");

  Serial.println("============================================================");
}

// ------------------------------------------------------------
// SEND TX METRICS TO RELAY
// ------------------------------------------------------------

void sendTelemetry()
{
  TelemetryPacket p = {};

  p.type = PACKET_TELEMETRY;
  p.source = NODE_TRANSMITTER;
  p.destination = NODE_RELAY_A;

  p.windowID = windowID;

  p.direct =
    makeMetrics(
      directSent,
      directReceived,
      directBytes,
      directRSSI,
      directLatencyTotal,
      directLatencySamples,
      directJitterTotal,
      directJitterSamples,
      1
    );

  p.relayE2E =
    makeMetrics(
      relaySent,
      relayReceived,
      relayBytes,
      0,
      relayLatencyTotal,
      relayLatencySamples,
      relayJitterTotal,
      relayJitterSamples,
      2
    );

 p.link1 = makeMetrics(
  relaySent,
  0,                      // placeholder, relay overwrites this with real receive count
  0,
  0,
  link1LatencyTotal,
  link1LatencySamples,
  link1JitterTotal,
  link1JitterSamples,
  1
);
  p.link2 = latestLink2;

  esp_now_send(
    RELAY_A_MAC,
    (uint8_t *)&p,
    sizeof(p)
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

  uint8_t type = data[0];

  if (type == PACKET_DIRECT_ACK)
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
      directWaiting &&
      ack.packetID ==
      directOutstandingID
    )
    {
      uint32_t rtt =
        millis() - directStart;

      uint32_t latency =
        rtt / 2;

      directReceived++;

      directBytes +=
        sizeof(CommandPacket);

      directRSSI =
        ack.measuredRSSI;

      directLatencyTotal +=
        latency;

      directLatencySamples++;

      if (
        directLatencySamples > 1
      )
      {
        uint32_t diff =
          abs(
            (int)latency -
            (int)lastDirectLatency
          );

        directJitterTotal += diff;
        directJitterSamples++;
      }

      lastDirectLatency =
        latency;

      directWaiting = false;
    }

    return;
  }
if (type == PACKET_RELAY_LINK1_ACK)
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
    relayWaiting &&
    ack.packetID ==
    relayOutstandingID
  )
  {
    uint32_t rtt =
      millis() - relayStart;

    uint32_t latency =
      rtt / 2;

    link1LatencyTotal +=
      latency;

    link1LatencySamples++;

    if (
      link1LatencySamples > 1
    )
    {
      uint32_t diff =
        abs(
          (int)latency -
          (int)lastLink1Latency
        );

      link1JitterTotal += diff;
      link1JitterSamples++;
    }

    lastLink1Latency =
      latency;
    // don't clear relayWaiting — PACKET_RELAY_FINAL_ACK still needs it for E2E
  }

  return;
}
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
      relayWaiting &&
      ack.packetID ==
      relayOutstandingID
    )
    {
      uint32_t rtt =
        millis() - relayStart;

      uint32_t latency =
        rtt / 2;

      relayReceived++;

      relayBytes +=
        sizeof(CommandPacket);

      relayLatencyTotal +=
        latency;

      relayLatencySamples++;

      if (
        relayLatencySamples > 1
      )
      {
        uint32_t diff =
          abs(
            (int)latency -
            (int)lastRelayLatency
          );

        relayJitterTotal += diff;
        relayJitterSamples++;
      }

      lastRelayLatency =
        latency;

      relayWaiting = false;
    }

    return;
  }

  if (type == PACKET_TELEMETRY)
  {
    if (len != sizeof(TelemetryPacket))
      return;

    TelemetryPacket p;

    memcpy(
      &p,
      data,
      sizeof(p)
    );

    latestLink1 = p.link1;
    latestLink2 = p.link2;
    latestWindowID = p.windowID;

    printTelemetry();   // now fires once all 4 metrics are for the same window

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

  WiFi.mode(WIFI_STA);

  esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  Serial.println();
  Serial.println("========================================");
  Serial.println("       ECHOLINK - TRANSMITTER");
  Serial.println("       ESP32 CORE 3.3.11");
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

  if (!addPeer(RELAY_A_MAC))
  {
    Serial.println(
      "RELAY-A PEER FAILED"
    );
    return;
  }

  if (!addPeer(RECEIVER_MAC))
  {
    Serial.println(
      "RECEIVER PEER FAILED"
    );
    return;
  }

  Serial.println(
    "RELAY-A PEER : READY"
  );

  Serial.println(
    "RECEIVER PEER: READY"
  );

  Serial.println(
    "ESP-NOW      : READY"
  );

  Serial.println(
    "4 BUTTONS    : READY"
  );

  Serial.println(
    "TELEMETRY    : READY"
  );

  Serial.println("========================================");

  startWindow();

  lastProbeTime =
    millis();
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------

void loop()
{
  readButtons();

  if (
    millis() - lastProbeTime >=
    PROBE_INTERVAL_MS
  )
  {
    lastProbeTime =
      millis();

    if (nextProbeDirect)
      sendDirectProbe();
    else
      sendRelayProbe();

    nextProbeDirect =
      !nextProbeDirect;
  }

  if (
    millis() - windowStart >=
    MEASUREMENT_WINDOW_MS
  )
  {
    sendTelemetry();

    startWindow();
  }

  delay(5);
}
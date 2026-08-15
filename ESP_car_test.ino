/*
 * ============================================================
 * Adaptive ESP32 Mesh RC Car
 * Phase 1 - Local WebSocket Motor Control
 * ============================================================
 *
 * Hardware:
 *   ESP32 DevKit V1
 *   L298N
 *   Steering Motor -> OUT1 / OUT2
 *   Drive Motor    -> OUT3 / OUT4
 *
 * Connections:
 *
 *   ESP32 GPIO26 -> L298N IN1
 *   ESP32 GPIO25 -> L298N IN2
 *   ESP32 GPIO22 -> L298N IN3
 *   ESP32 GPIO23 -> L298N IN4
 *
 * Network:
 *   ESP32 creates its own Wi-Fi Access Point
 *   SSID: RC_CAR_TEST
 *   Password: rc-car-1234
 *
 *   Dashboard:
 *   http://192.168.4.1
 *
 *   WebSocket:
 *   ws://192.168.4.1:81
 *
 * Motor mapping:
 *
 *   STEERING:
 *      LEFT  -> IN1 HIGH, IN2 LOW
 *      RIGHT -> IN1 LOW,  IN2 HIGH
 *      STOP  -> IN1 LOW,  IN2 LOW
 *
 *   DRIVE:
 *      FORWARD  -> IN3 HIGH, IN4 LOW
 *      BACKWARD -> IN3 LOW,  IN4 HIGH
 *      STOP     -> IN3 LOW, IN4 LOW
 *
 * Safety:
 *   If no valid drive command is received within
 *   COMMAND_TIMEOUT_MS, the drive motor is stopped.
 *
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>


// ============================================================
// GPIO CONFIGURATION
// ============================================================

// Steering motor -> L298N OUT1 / OUT2
const uint8_t STEER_IN1 = 26;
const uint8_t STEER_IN2 = 25;

// Drive motor -> L298N OUT3 / OUT4
const uint8_t DRIVE_IN1 = 22;
const uint8_t DRIVE_IN2 = 23;


// ============================================================
// WIFI CONFIGURATION
// ============================================================

const char* AP_SSID = "RC_CAR_TEST";
const char* AP_PASSWORD = "rc-car-1234";


// ============================================================
// SERVERS
// ============================================================

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);


// ============================================================
// SAFETY CONFIGURATION
// ============================================================

// If no drive command arrives within this period,
// the drive motor is automatically stopped.

const unsigned long COMMAND_TIMEOUT_MS = 700;


// ============================================================
// STEERING CONFIGURATION
// ============================================================

// Steering is a mechanical steering motor rather than
// a continuously rotating drive motor.
//
// Start conservatively.
// Increase/decrease after testing.

const unsigned long STEERING_PULSE_MS = 150;


// ============================================================
// STATE
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

DriveState driveState = DRIVE_STOP;
SteeringState steeringState = STEER_CENTER;


// Last valid DRIVE command received
unsigned long lastDriveCommandTime = 0;

// Steering pulse control
unsigned long steeringStopTime = 0;


// ============================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================

void stopDrive()
{
    digitalWrite(DRIVE_IN1, LOW);
    digitalWrite(DRIVE_IN2, LOW);

    driveState = DRIVE_STOP;
}


void driveForward()
{
    digitalWrite(DRIVE_IN1, HIGH);
    digitalWrite(DRIVE_IN2, LOW);

    driveState = DRIVE_FORWARD;

    lastDriveCommandTime = millis();
}


void driveBackward()
{
    digitalWrite(DRIVE_IN1, LOW);
    digitalWrite(DRIVE_IN2, HIGH);

    driveState = DRIVE_BACKWARD;

    lastDriveCommandTime = millis();
}


// ============================================================
// STEERING CONTROL
// ============================================================

void stopSteering()
{
    digitalWrite(STEER_IN1, LOW);
    digitalWrite(STEER_IN2, LOW);

    steeringState = STEER_CENTER;
}


void steerLeft()
{
    digitalWrite(STEER_IN1, HIGH);
    digitalWrite(STEER_IN2, LOW);

    steeringState = STEER_LEFT;

    steeringStopTime = millis() + STEERING_PULSE_MS;
}


void steerRight()
{
    digitalWrite(STEER_IN1, LOW);
    digitalWrite(STEER_IN2, HIGH);

    steeringState = STEER_RIGHT;

    steeringStopTime = millis() + STEERING_PULSE_MS;
}


// ============================================================
// COMPLETE STOP
// ============================================================

void emergencyStop()
{
    stopDrive();
    stopSteering();

    Serial.println("[SAFETY] EMERGENCY STOP");
}


// ============================================================
// STATE TO STRING
// ============================================================

String getDriveState()
{
    switch (driveState)
    {
        case DRIVE_FORWARD:
            return "FORWARD";

        case DRIVE_BACKWARD:
            return "BACKWARD";

        default:
            return "STOP";
    }
}


String getSteeringState()
{
    switch (steeringState)
    {
        case STEER_LEFT:
            return "LEFT";

        case STEER_RIGHT:
            return "RIGHT";

        default:
            return "CENTER";
    }
}


// ============================================================
// SEND CURRENT STATE TO CLIENT
// ============================================================

void sendState(uint8_t clientNum)
{
    String message = "{";

    message += "\"drive\":\"";
    message += getDriveState();
    message += "\",";

    message += "\"steering\":\"";
    message += getSteeringState();
    message += "\"";

    message += "}";

    webSocket.sendTXT(clientNum, message);
}


// ============================================================
// PROCESS COMMAND
// ============================================================

void processCommand(String command, uint8_t clientNum)
{
    command.trim();
    command.toUpperCase();

    Serial.print("[COMMAND] ");
    Serial.println(command);


    // --------------------------------------------------------
    // DRIVE
    // --------------------------------------------------------

    if (command == "FORWARD")
    {
        driveForward();
    }

    else if (command == "BACKWARD")
    {
        driveBackward();
    }

    else if (command == "DRIVE_STOP")
    {
        stopDrive();
    }


    // --------------------------------------------------------
    // STEERING
    // --------------------------------------------------------

    else if (command == "LEFT")
    {
        steerLeft();
    }

    else if (command == "RIGHT")
    {
        steerRight();
    }

    else if (command == "CENTER")
    {
        stopSteering();
    }


    // --------------------------------------------------------
    // COMPLETE STOP
    // --------------------------------------------------------

    else if (command == "STOP")
    {
        emergencyStop();
    }


    // --------------------------------------------------------
    // UNKNOWN COMMAND
    // --------------------------------------------------------

    else
    {
        Serial.println("[WARNING] Unknown command");
        return;
    }


    // Send updated state back to browser
    sendState(clientNum);
}


// ============================================================
// WEBSOCKET EVENT HANDLER
// ============================================================

void webSocketEvent(
    uint8_t clientNum,
    WStype_t type,
    uint8_t* payload,
    size_t length
)
{
    switch (type)
    {
        // ----------------------------------------------------
        // CLIENT CONNECTED
        // ----------------------------------------------------

        case WStype_CONNECTED:
        {
            IPAddress ip = webSocket.remoteIP(clientNum);

            Serial.print("[WS] Client connected: ");
            Serial.println(ip);

            sendState(clientNum);

            break;
        }


        // ----------------------------------------------------
        // CLIENT DISCONNECTED
        // ----------------------------------------------------

        case WStype_DISCONNECTED:
        {
            Serial.print("[WS] Client disconnected: ");
            Serial.println(clientNum);

            // Very important:
            // If controller disconnects, stop the car.

            emergencyStop();

            break;
        }


        // ----------------------------------------------------
        // TEXT MESSAGE
        // ----------------------------------------------------

        case WStype_TEXT:
        {
            String command = "";

            for (size_t i = 0; i < length; i++)
            {
                command += (char)payload[i];
            }

            processCommand(command, clientNum);

            break;
        }


        default:
            break;
    }
}


// ============================================================
// WEB DASHBOARD
// ============================================================

const char MAIN_PAGE[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta name="viewport"
      content="width=device-width,
               initial-scale=1.0,
               maximum-scale=1.0,
               user-scalable=no">

<title>ESP32 RC Car</title>

<style>

* {
    box-sizing: border-box;
    -webkit-user-select: none;
    user-select: none;
}

body {

    margin: 0;

    font-family:
        Arial,
        Helvetica,
        sans-serif;

    background: #111;

    color: white;

    text-align: center;
}


.container {

    max-width: 500px;

    margin: auto;

    padding: 20px;
}


h1 {

    margin-top: 10px;

    margin-bottom: 5px;
}


.subtitle {

    color: #aaa;

    margin-bottom: 20px;
}


.status {

    padding: 10px;

    border-radius: 8px;

    background: #222;

    margin-bottom: 20px;
}


.connected {

    color: #00ff88;
}


.disconnected {

    color: #ff4444;
}


.state {

    display: flex;

    justify-content: space-around;

    background: #1b1b1b;

    padding: 12px;

    border-radius: 10px;

    margin-bottom: 25px;
}


.stateBox {

    width: 45%;
}


.stateTitle {

    color: #888;

    font-size: 13px;
}


.stateValue {

    font-size: 18px;

    font-weight: bold;

    margin-top: 5px;
}


.controls {

    display: grid;

    grid-template-columns:
        1fr
        1fr
        1fr;

    gap: 12px;

    max-width: 350px;

    margin: auto;
}


button {

    height: 80px;

    border: none;

    border-radius: 15px;

    font-size: 18px;

    font-weight: bold;

    background: #333;

    color: white;

    touch-action: manipulation;

}


button:active {

    background: #555;

    transform: scale(0.96);
}


.forward {

    grid-column: 2;

}


.left {

    grid-column: 1;

    grid-row: 2;
}


.stop {

    grid-column: 2;

    grid-row: 2;

    background: #8b0000;
}


.right {

    grid-column: 3;

    grid-row: 2;
}


.backward {

    grid-column: 2;

    grid-row: 3;
}


.center {

    grid-column: 1 / 4;

    margin-top: 15px;

    height: 55px;

    background: #444;
}


.info {

    margin-top: 25px;

    color: #777;

    font-size: 13px;
}

</style>

</head>


<body>


<div class="container">


<h1>ESP32 RC CAR</h1>

<div class="subtitle">
Phase 1 Local Control
</div>


<div id="connection"
     class="status disconnected">

DISCONNECTED

</div>


<div class="state">

<div class="stateBox">

<div class="stateTitle">
DRIVE
</div>

<div id="drive"
     class="stateValue">

STOP

</div>

</div>


<div class="stateBox">

<div class="stateTitle">
STEERING
</div>

<div id="steering"
     class="stateValue">

CENTER

</div>

</div>

</div>


<div class="controls">


<button class="forward"
        id="forward">

▲
<br>
FORWARD

</button>


<button class="left"
        id="left">

◀
<br>
LEFT

</button>


<button class="stop"
        id="stop">

■
<br>
STOP

</button>


<button class="right"
        id="right">

▶
<br>
RIGHT

</button>


<button class="backward"
        id="backward">

▼
<br>
BACKWARD

</button>


<button class="center"
        id="center">

CENTER STEERING

</button>


</div>


<div class="info">

ESP32 WebSocket Control

</div>


</div>


<script>


let socket;


// ============================================================
// CONNECT WEBSOCKET
// ============================================================

function connectWebSocket()
{

    socket = new WebSocket(
        "ws://" +
        window.location.hostname +
        ":81/"
    );


    socket.onopen = function()
    {

        document
            .getElementById("connection")
            .innerText = "CONNECTED";

        document
            .getElementById("connection")
            .className =
                "status connected";
    };


    socket.onclose = function()
    {

        document
            .getElementById("connection")
            .innerText = "DISCONNECTED";

        document
            .getElementById("connection")
            .className =
                "status disconnected";


        // Try reconnecting

        setTimeout(
            connectWebSocket,
            1000
        );
    };


    socket.onerror = function()
    {

        socket.close();

    };


    socket.onmessage = function(event)
    {

        try
        {

            let state =
                JSON.parse(event.data);


            if (state.drive)
            {

                document
                    .getElementById("drive")
                    .innerText =
                        state.drive;

            }


            if (state.steering)
            {

                document
                    .getElementById("steering")
                    .innerText =
                        state.steering;

            }

        }

        catch(error)
        {

            console.log(error);

        }

    };

}


// ============================================================
// SEND COMMAND
// ============================================================

function sendCommand(command)
{

    if (
        socket &&
        socket.readyState === WebSocket.OPEN
    )
    {

        socket.send(command);

    }

}


// ============================================================
// DRIVE BUTTONS
// ============================================================

// Forward

document
    .getElementById("forward")
    .addEventListener(
        "pointerdown",
        function(event)
        {

            event.preventDefault();

            sendCommand("FORWARD");

        }
    );


document
    .getElementById("forward")
    .addEventListener(
        "pointerup",
        function(event)
        {

            event.preventDefault();

            sendCommand("DRIVE_STOP");

        }
    );


document
    .getElementById("forward")
    .addEventListener(
        "pointercancel",
        function()
        {

            sendCommand("DRIVE_STOP");

        }
    );


// Backward

document
    .getElementById("backward")
    .addEventListener(
        "pointerdown",
        function(event)
        {

            event.preventDefault();

            sendCommand("BACKWARD");

        }
    );


document
    .getElementById("backward")
    .addEventListener(
        "pointerup",
        function(event)
        {

            event.preventDefault();

            sendCommand("DRIVE_STOP");

        }
    );


document
    .getElementById("backward")
    .addEventListener(
        "pointercancel",
        function()
        {

            sendCommand("DRIVE_STOP");

        }
    );


// ============================================================
// STEERING
// ============================================================

document
    .getElementById("left")
    .addEventListener(
        "pointerdown",
        function(event)
        {

            event.preventDefault();

            sendCommand("LEFT");

        }
    );


document
    .getElementById("right")
    .addEventListener(
        "pointerdown",
        function(event)
        {

            event.preventDefault();

            sendCommand("RIGHT");

        }
    );


document
    .getElementById("center")
    .addEventListener(
        "pointerdown",
        function(event)
        {

            event.preventDefault();

            sendCommand("CENTER");

        }
    );


// ============================================================
// STOP
// ============================================================

document
    .getElementById("stop")
    .addEventListener(
        "pointerdown",
        function(event)
        {

            event.preventDefault();

            sendCommand("STOP");

        }
    );


// ============================================================
// START
// ============================================================

connectWebSocket();


</script>


</body>

</html>

)rawliteral";


// ============================================================
// HTTP ROOT
// ============================================================

void handleRoot()
{
    server.send(
        200,
        "text/html",
        MAIN_PAGE
    );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{

    Serial.begin(115200);

    delay(500);


    Serial.println();
    Serial.println();
    Serial.println(
        "================================="
    );
    Serial.println(
        " ESP32 RC CAR - PHASE 1"
    );
    Serial.println(
        " WebSocket Motor Control"
    );
    Serial.println(
        "================================="
    );


    // --------------------------------------------------------
    // GPIO
    // --------------------------------------------------------

    pinMode(STEER_IN1, OUTPUT);
    pinMode(STEER_IN2, OUTPUT);

    pinMode(DRIVE_IN1, OUTPUT);
    pinMode(DRIVE_IN2, OUTPUT);


    // Always start stopped

    emergencyStop();


    // --------------------------------------------------------
    // WIFI ACCESS POINT
    // --------------------------------------------------------

    WiFi.mode(WIFI_AP);

    bool apStarted =
        WiFi.softAP(
            AP_SSID,
            AP_PASSWORD
        );


    if (apStarted)
    {

        Serial.println(
            "[WIFI] Access Point started"
        );

        Serial.print(
            "[WIFI] SSID: "
        );

        Serial.println(
            AP_SSID
        );

        Serial.print(
            "[WIFI] IP: "
        );

        Serial.println(
            WiFi.softAPIP()
        );

    }

    else
    {

        Serial.println(
            "[WIFI] Failed to start AP"
        );

    }


    // --------------------------------------------------------
    // HTTP SERVER
    // --------------------------------------------------------

    server.on(
        "/",
        handleRoot
    );

    server.begin();


    Serial.println(
        "[HTTP] Server started"
    );


    // --------------------------------------------------------
    // WEBSOCKET SERVER
    // --------------------------------------------------------

    webSocket.begin();

    webSocket.onEvent(
        webSocketEvent
    );


    Serial.println(
        "[WS] WebSocket server started on port 81"
    );


    Serial.println();
    Serial.println(
        "Open http://192.168.4.1"
    );
    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{

    server.handleClient();

    webSocket.loop();


    // --------------------------------------------------------
    // STEERING TIMER
    // --------------------------------------------------------

    if (
        steeringState != STEER_CENTER &&
        millis() >= steeringStopTime
    )
    {

        stopSteering();

    }


    // --------------------------------------------------------
    // DRIVE FAILSAFE
    // --------------------------------------------------------

    if (
        driveState != DRIVE_STOP &&
        millis() - lastDriveCommandTime >
            COMMAND_TIMEOUT_MS
    )
    {

        Serial.println(
            "[SAFETY] Drive command timeout"
        );

        stopDrive();

    }


    delay(2);
}
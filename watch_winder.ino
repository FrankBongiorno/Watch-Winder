#include <WiFi.h> // ESP32 Wi-Fi library
#include <AsyncTCP.h> // ESP32 async TCP library for web server
#include <ESPAsyncWebServer.h> // ESP32 async web server
#include <AsyncMqttClient.h> // Async MQTT library for ESP32
#include <Stepper.h> // Stepper motor library
#include <ArduinoOTA.h> // OTA updates (ESP32 version)
#include "arduino_secrets.h"

// Wi-Fi and MQTT settings from arduino_secrets.h
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;
IPAddress mqtt_server(192, 168, 1, 100); // Replace with the actual IP of threesticks.dyndns.org
const int mqtt_port = 1883;
const char* mqtt_user = mqtt_username;
const char* mqtt_pass = mqtt_password;
const char* mqtt_topic_status = "watch-winder/status";
const char* mqtt_topic_command = "watch-winder/command";

// Static IP configuration
IPAddress staticIP(192, 168, 8, 101);
IPAddress gateway(192, 168, 8, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2(8, 8, 4, 4);

// OTA settings
const char* ota_password = "watchwinder123"; // Password for OTA updates

// Stepper motor settings
const int STEPS_PER_REVOLUTION = 4096; // 28BYJ-48 with 1:64 gear ratio
const int RPM = 5; // Slow speed for watch winder
const int ROTATIONS_PER_CYCLE = 10; // 10 rotations per direction
const unsigned long PAUSE_DURATION_MS = 5 * 60 * 1000UL; // 5 minutes in milliseconds

// Define pins for ULN2003 driver (ESP32-compatible)
const int IN1 = 14; // GPIO14
const int IN2 = 12; // GPIO12
const int IN3 = 13; // GPIO13
const int IN4 = 15; // GPIO15

// Initialize the Stepper library
Stepper myStepper(STEPS_PER_REVOLUTION, IN1, IN3, IN2, IN4);

// Initialize MQTT and web server
AsyncMqttClient mqttClient;
AsyncWebServer server(80);

// Timer for MQTT reconnection
TimerHandle_t mqttReconnectTimer;

// State machine states
enum WinderState {
  STOPPED,
  ROTATING_CLOCKWISE,
  PAUSING_AFTER_CLOCKWISE,
  ROTATING_COUNTERCLOCKWISE,
  PAUSING_AFTER_COUNTERCLOCKWISE
};

// State variables
WinderState currentState = STOPPED;
bool winderRunning = false;
int currentRotation = 0;
unsigned long lastStateChangeTime = 0;

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT");
  mqttClient.subscribe(mqtt_topic_command, 0);
  mqttClient.publish(mqtt_topic_status, 0, false, "started");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT");
  if (WiFi.isConnected()) {
    xTimerChangePeriod(mqttReconnectTimer, 5000, 0); // Retry after 5 seconds
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String message = "";
  for (size_t i = 0; i < len; i++) {
    message += payload[i];
  }
  if (String(topic) == mqtt_topic_command) {
    if (message == "start") {
      winderRunning = true;
      mqttClient.publish(mqtt_topic_status, 0, false, "running");
    } else if (message == "stop") {
      winderRunning = false;
      currentState = STOPPED;
      currentRotation = 0;
      mqttClient.publish(mqtt_topic_status, 0, false, "stopped");
    }
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("Watch Winder Starting...");

  // Set the speed of the stepper motor
  myStepper.setSpeed(RPM);

  // Configure static IP
  WiFi.config(staticIP, gateway, subnet, dns1, dns2);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // Set up OTA
  ArduinoOTA.setHostname("watch-winder");
  ArduinoOTA.setPassword(ota_password);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready");

  // Set up MQTT
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCredentials(mqtt_user, mqtt_pass);
  connectToMqtt();

  // Set up web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><title>Watch Winder</title>";
    html += "<style>body { font-family: Arial, sans-serif; text-align: center; background-color: #f4f4f4; }";
    html += "button { padding: 10px 20px; margin: 10px; background-color: #4CAF50; color: white; border: none; border-radius: 5px; cursor: pointer; }";
    html += "button:hover { background-color: #45a049; }";
    html += "</style></head><body>";
    html += "<h1>Watch Winder Control</h1>";
    html += "<p>Status: <span id='status'>" + String(winderRunning ? "Running" : "Stopped") + "</span></p>";
    html += "<button onclick=\"fetch('/start')\">Start Winder</button>";
    html += "<button onclick=\"fetch('/stop')\">Stop Winder</button>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request){
    winderRunning = true;
    mqttClient.publish(mqtt_topic_status, 0, false, "running");
    request->send(200, "text/plain", "Winder started");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    winderRunning = false;
    currentState = STOPPED;
    currentRotation = 0;
    mqttClient.publish(mqtt_topic_status, 0, false, "stopped");
    request->send(200, "text/plain", "Winder stopped");
  });

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();

  if (winderRunning) {
    unsigned long currentTime = millis();

    switch (currentState) {
      case STOPPED:
        currentState = ROTATING_CLOCKWISE;
        currentRotation = 0;
        lastStateChangeTime = currentTime;
        Serial.println("Rotating Clockwise...");
        break;

      case ROTATING_CLOCKWISE:
        if (currentRotation < ROTATIONS_PER_CYCLE) {
          myStepper.step(STEPS_PER_REVOLUTION);
          currentRotation++;
        } else {
          currentState = PAUSING_AFTER_CLOCKWISE;
          lastStateChangeTime = currentTime;
          Serial.println("Pausing after clockwise...");
        }
        break;

      case PAUSING_AFTER_CLOCKWISE:
        if (currentTime - lastStateChangeTime >= PAUSE_DURATION_MS) {
          currentState = ROTATING_COUNTERCLOCKWISE;
          currentRotation = 0;
          lastStateChangeTime = currentTime;
          Serial.println("Rotating Counterclockwise...");
        }
        break;

      case ROTATING_COUNTERCLOCKWISE:
        if (currentRotation < ROTATIONS_PER_CYCLE) {
          myStepper.step(-STEPS_PER_REVOLUTION);
          currentRotation++;
        } else {
          currentState = PAUSING_AFTER_COUNTERCLOCKWISE;
          lastStateChangeTime = currentTime;
          Serial.println("Pausing after counterclockwise...");
        }
        break;

      case PAUSING_AFTER_COUNTERCLOCKWISE:
        if (currentTime - lastStateChangeTime >= PAUSE_DURATION_MS) {
          currentState = ROTATING_CLOCKWISE;
          currentRotation = 0;
          lastStateChangeTime = currentTime;
          Serial.println("Rotating Clockwise...");
        }
        break;
    }
  } else {
    currentState = STOPPED;
    currentRotation = 0;
  }
}
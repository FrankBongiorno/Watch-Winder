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
const char* mqtt_hostname = "threesticks.dyndns.org";
IPAddress mqtt_server;
const int mqtt_port = 1883;
const char* mqtt_user = mqtt_username;
const char* mqtt_pass = mqtt_password;
const char* mqtt_topic_status = "watch-winder/status";
const char* mqtt_topic_command = "watch-winder/command";
const char* mqtt_topic_rotation_pattern = "watch-winder/rotation_pattern";
const char* mqtt_topic_cw_duration = "watch-winder/cw_duration";
const char* mqtt_topic_ccw_duration = "watch-winder/ccw_duration";
const char* mqtt_topic_pause_duration = "watch-winder/pause_duration";
const char* mqtt_topic_min_rpm = "watch-winder/min_rpm";
const char* mqtt_topic_max_rpm = "watch-winder/max_rpm";
const char* mqtt_topic_vary_speed = "watch-winder/vary_speed";
const char* mqtt_topic_current_rpm = "watch-winder/current_rpm";
const char* mqtt_topic_runtime = "watch-winder/runtime";
const char* mqtt_topic_cycle_count = "watch-winder/cycle_count";
const char* mqtt_topic_next_cycle = "watch-winder/next_cycle";

// Static IP configuration
IPAddress staticIP(192, 168, 8, 101);
IPAddress gateway(192, 168, 8, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2(8, 8, 4, 4);

// OTA settings
const char* ota_password = "watchwinder123";

// Stepper motor settings
const int STEPS_PER_REVOLUTION = 4096; // 28BYJ-48 with 1:64 gear ratio
unsigned long MIN_RPM = 5; // Default min speed
unsigned long MAX_RPM = 10; // Default max speed
bool varySpeed = false; // Default to not varying speed
const unsigned long SPEED_SWITCH_INTERVAL_MS = 10 * 1000UL; // Switch speed every 10 seconds
const unsigned long DEBOUNCE_INTERVAL_MS = 1000UL; // Debounce interval for MQTT commands (1 second)
const unsigned long STEPS_PER_MINUTE_BASE = 5 * STEPS_PER_REVOLUTION; // Base steps per minute at 5 RPM
unsigned long CW_DURATION_MINUTES = 2; // Default 2 minutes clockwise
unsigned long CCW_DURATION_MINUTES = 2; // Default 2 minutes counterclockwise
unsigned long PAUSE_DURATION_MINUTES = 5; // Default 5 minutes pause
unsigned long CW_STEPS = CW_DURATION_MINUTES * STEPS_PER_MINUTE_BASE; // Steps for CW rotation at base RPM
unsigned long CCW_STEPS = CCW_DURATION_MINUTES * STEPS_PER_MINUTE_BASE; // Steps for CCW rotation at base RPM
unsigned long PAUSE_DURATION_MS = PAUSE_DURATION_MINUTES * 60 * 1000UL; // Pause duration in milliseconds

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

// Timer for MQTT reconnection, DNS re-resolution, and speed variation
TimerHandle_t mqttReconnectTimer;
TimerHandle_t dnsResolveTimer;
TimerHandle_t speedSwitchTimer;
const unsigned long DNS_RESOLVE_INTERVAL_MS = 5 * 60 * 1000UL; // Re-resolve every 5 minutes

// State machine states
enum WinderState {
  STOPPED,
  ROTATING_CW,
  ROTATING_CCW,
  PAUSING
};

// Rotation pattern options
enum RotationPattern {
  ALWAYS_CW,
  ALWAYS_CCW,
  ALTERNATE
};

// State variables
WinderState currentState = STOPPED;
RotationPattern rotationPattern = ALTERNATE; // Default to alternating
bool winderRunning = false;
bool isClockwise = true; // Start with clockwise for alternating pattern
bool useMinSpeed = true; // Start with min speed for variation
bool selfPublished = false; // Flag to track self-published messages
unsigned long currentSteps = 0;
unsigned long lastStateChangeTime = 0;
unsigned long lastSpeedSwitchTime = 0;
unsigned long startTime = 0; // Track when winder started
unsigned long runtime = 0; // Total runtime in seconds
unsigned long lastRuntimeUpdate = 0; // Last time runtime was published
unsigned long cycleCount = 0; // Number of completed cycles

// Debounce variables
unsigned long lastCommandTime = 0;
String lastCommandTopic = "";
String lastCommandValue = "";

// Function to resolve DNS and update MQTT server IP
bool resolveMqttServer() {
  Serial.print("Resolving MQTT broker hostname: ");
  Serial.println(mqtt_hostname);
  if (WiFi.hostByName(mqtt_hostname, mqtt_server)) {
    Serial.print("Resolved MQTT broker IP: ");
    Serial.println(mqtt_server);
    return true;
  } else {
    Serial.println("Failed to resolve MQTT broker hostname");
    return false;
  }
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  Serial.print("Broker IP: ");
  Serial.println(mqtt_server);
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT");
  mqttClient.subscribe(mqtt_topic_command, 0);
  mqttClient.subscribe(mqtt_topic_rotation_pattern, 0);
  mqttClient.subscribe(mqtt_topic_cw_duration, 0);
  mqttClient.subscribe(mqtt_topic_ccw_duration, 0);
  mqttClient.subscribe(mqtt_topic_pause_duration, 0);
  mqttClient.subscribe(mqtt_topic_min_rpm, 0);
  mqttClient.subscribe(mqtt_topic_max_rpm, 0);
  mqttClient.subscribe(mqtt_topic_vary_speed, 0);
  selfPublished = true; // Mark the following publishes as self-initiated
  mqttClient.publish(mqtt_topic_status, 0, false, "started");
  mqttClient.publish(mqtt_topic_rotation_pattern, 0, false, rotationPattern == ALWAYS_CW ? "cw" : rotationPattern == ALWAYS_CCW ? "ccw" : "alternate");
  mqttClient.publish(mqtt_topic_cw_duration, 0, false, String(CW_DURATION_MINUTES).c_str());
  mqttClient.publish(mqtt_topic_ccw_duration, 0, false, String(CCW_DURATION_MINUTES).c_str());
  mqttClient.publish(mqtt_topic_pause_duration, 0, false, String(PAUSE_DURATION_MINUTES).c_str());
  mqttClient.publish(mqtt_topic_min_rpm, 0, false, String(MIN_RPM).c_str());
  mqttClient.publish(mqtt_topic_max_rpm, 0, false, String(MAX_RPM).c_str());
  mqttClient.publish(mqtt_topic_vary_speed, 0, false, varySpeed ? "true" : "false");
  mqttClient.publish(mqtt_topic_current_rpm, 0, false, String(useMinSpeed ? MIN_RPM : MAX_RPM).c_str());
  mqttClient.publish(mqtt_topic_cycle_count, 0, false, String(cycleCount).c_str());
  selfPublished = false;
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.print("Disconnected from MQTT. Reason: ");
  Serial.println(static_cast<uint8_t>(reason));
  if (WiFi.isConnected()) {
    xTimerChangePeriod(mqttReconnectTimer, 5000, 0); // Retry after 5 seconds
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String message = "";
  for (size_t i = 0; i < len; i++) {
    message += payload[i];
  }

  // Ignore self-published messages
  if (selfPublished) {
    Serial.print("Ignoring self-published message on topic: ");
    Serial.println(topic);
    return;
  }

  // Debounce: Ignore rapid repeated commands
  unsigned long currentTime = millis();
  if (String(topic) == lastCommandTopic && message == lastCommandValue && (currentTime - lastCommandTime) < DEBOUNCE_INTERVAL_MS) {
    Serial.print("Debouncing repeated command on topic: ");
    Serial.print(topic);
    Serial.print(", value: ");
    Serial.println(message);
    return;
  }
  lastCommandTopic = topic;
  lastCommandValue = message;
  lastCommandTime = currentTime;

  Serial.print("Received MQTT message on topic: ");
  Serial.print(topic);
  Serial.print(", value: ");
  Serial.println(message);

  if (String(topic) == mqtt_topic_command) {
    if (message == "start") {
      winderRunning = true;
      startTime = millis();
      runtime = 0;
      isClockwise = (rotationPattern != ALWAYS_CCW);
      useMinSpeed = true;
      myStepper.setSpeed(useMinSpeed ? MIN_RPM : MAX_RPM);
      selfPublished = true;
      mqttClient.publish(mqtt_topic_status, 0, false, "running");
      mqttClient.publish(mqtt_topic_current_rpm, 0, false, String(useMinSpeed ? MIN_RPM : MAX_RPM).c_str());
      selfPublished = false;
      if (varySpeed) {
        xTimerStart(speedSwitchTimer, 0);
        Serial.println("Started speed switch timer");
      }
    } else if (message == "stop") {
      winderRunning = false;
      currentState = STOPPED;
      currentSteps = 0;
      xTimerStop(speedSwitchTimer, 0);
      Serial.println("Stopped speed switch timer");
      selfPublished = true;
      mqttClient.publish(mqtt_topic_status, 0, false, "stopped");
      selfPublished = false;
    }
  } else if (String(topic) == mqtt_topic_rotation_pattern) {
    if (message == "cw") {
      rotationPattern = ALWAYS_CW;
      isClockwise = true;
    } else if (message == "ccw") {
      rotationPattern = ALWAYS_CCW;
      isClockwise = false;
    } else if (message == "alternate") {
      rotationPattern = ALTERNATE;
      isClockwise = true; // Start with CW for alternating
    }
    selfPublished = true;
    mqttClient.publish(mqtt_topic_rotation_pattern, 0, false, rotationPattern == ALWAYS_CW ? "cw" : rotationPattern == ALWAYS_CCW ? "ccw" : "alternate");
    selfPublished = false;
  } else if (String(topic) == mqtt_topic_cw_duration) {
    unsigned long newDuration = message.toInt();
    if (newDuration > 0) {
      CW_DURATION_MINUTES = newDuration;
      CW_STEPS = CW_DURATION_MINUTES * (useMinSpeed ? MIN_RPM : MAX_RPM) * STEPS_PER_REVOLUTION / 5;
      selfPublished = true;
      mqttClient.publish(mqtt_topic_cw_duration, 0, false, String(CW_DURATION_MINUTES).c_str());
      selfPublished = false;
    }
  } else if (String(topic) == mqtt_topic_ccw_duration) {
    unsigned long newDuration = message.toInt();
    if (newDuration > 0) {
      CCW_DURATION_MINUTES = newDuration;
      CCW_STEPS = CCW_DURATION_MINUTES * (useMinSpeed ? MIN_RPM : MAX_RPM) * STEPS_PER_REVOLUTION / 5;
      selfPublished = true;
      mqttClient.publish(mqtt_topic_ccw_duration, 0, false, String(CCW_DURATION_MINUTES).c_str());
      selfPublished = false;
    }
  } else if (String(topic) == mqtt_topic_pause_duration) {
    unsigned long newDuration = message.toInt();
    if (newDuration > 0) {
      PAUSE_DURATION_MINUTES = newDuration;
      PAUSE_DURATION_MS = PAUSE_DURATION_MINUTES * 60 * 1000UL;
      selfPublished = true;
      mqttClient.publish(mqtt_topic_pause_duration, 0, false, String(PAUSE_DURATION_MINUTES).c_str());
      selfPublished = false;
    }
  } else if (String(topic) == mqtt_topic_min_rpm) {
    unsigned long newRPM = message.toInt();
    if (newRPM >= 1 && newRPM <= 15) {
      MIN_RPM = newRPM;
      if (useMinSpeed) {
        myStepper.setSpeed(MIN_RPM);
        selfPublished = true;
        mqttClient.publish(mqtt_topic_current_rpm, 0, false, String(MIN_RPM).c_str());
        selfPublished = false;
      }
      CW_STEPS = CW_DURATION_MINUTES * MIN_RPM * STEPS_PER_REVOLUTION / 5;
      CCW_STEPS = CCW_DURATION_MINUTES * MIN_RPM * STEPS_PER_REVOLUTION / 5;
      selfPublished = true;
      mqttClient.publish(mqtt_topic_min_rpm, 0, false, String(MIN_RPM).c_str());
      selfPublished = false;
    }
  } else if (String(topic) == mqtt_topic_max_rpm) {
    unsigned long newRPM = message.toInt();
    if (newRPM >= 1 && newRPM <= 15) {
      MAX_RPM = newRPM;
      if (!useMinSpeed) {
        myStepper.setSpeed(MAX_RPM);
        selfPublished = true;
        mqttClient.publish(mqtt_topic_current_rpm, 0, false, String(MAX_RPM).c_str());
        selfPublished = false;
      }
      CW_STEPS = CW_DURATION_MINUTES * MAX_RPM * STEPS_PER_REVOLUTION / 5;
      CCW_STEPS = CCW_DURATION_MINUTES * MAX_RPM * STEPS_PER_REVOLUTION / 5;
      selfPublished = true;
      mqttClient.publish(mqtt_topic_max_rpm, 0, false, String(MAX_RPM).c_str());
      selfPublished = false;
    }
  } else if (String(topic) == mqtt_topic_vary_speed) {
    varySpeed = (message == "true");
    if (winderRunning && varySpeed) {
      xTimerStart(speedSwitchTimer, 0);
      Serial.println("Started speed switch timer");
    } else {
      xTimerStop(speedSwitchTimer, 0);
      Serial.println("Stopped speed switch timer");
      useMinSpeed = true;
      myStepper.setSpeed(MIN_RPM);
      selfPublished = true;
      mqttClient.publish(mqtt_topic_current_rpm, 0, false, String(MIN_RPM).c_str());
      selfPublished = false;
    }
    selfPublished = true;
    mqttClient.publish(mqtt_topic_vary_speed, 0, false, varySpeed ? "true" : "false");
    selfPublished = false;
  }
}

void dnsResolveCallback(TimerHandle_t xTimer) {
  if (resolveMqttServer()) {
    if (!mqttClient.connected()) {
      connectToMqtt();
    }
  }
}

void speedSwitchCallback(TimerHandle_t xTimer) {
  if (winderRunning) {
    unsigned long currentTime = millis();
    Serial.print("Speed switch triggered at: ");
    Serial.print(currentTime);
    Serial.print(" ms, Interval: ");
    Serial.print(SPEED_SWITCH_INTERVAL_MS);
    Serial.println(" ms");
    useMinSpeed = !useMinSpeed;
    unsigned long currentRPM = useMinSpeed ? MIN_RPM : MAX_RPM;
    myStepper.setSpeed(currentRPM);
    selfPublished = true;
    mqttClient.publish(mqtt_topic_current_rpm, 0, false, String(currentRPM).c_str());
    selfPublished = false;
    // Adjust steps based on the new RPM
    CW_STEPS = CW_DURATION_MINUTES * currentRPM * STEPS_PER_REVOLUTION / 5;
    CCW_STEPS = CCW_DURATION_MINUTES * currentRPM * STEPS_PER_REVOLUTION / 5;
    lastSpeedSwitchTime = currentTime;
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("Watch Winder Starting...");

  // Set the initial speed of the stepper motor
  myStepper.setSpeed(MIN_RPM);

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
  dnsResolveTimer = xTimerCreate("dnsTimer", pdMS_TO_TICKS(DNS_RESOLVE_INTERVAL_MS), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(dnsResolveCallback));
  speedSwitchTimer = xTimerCreate("speedTimer", pdMS_TO_TICKS(SPEED_SWITCH_INTERVAL_MS), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(speedSwitchCallback));
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setKeepAlive(30);
  mqttClient.setClientId("WatchWinder");
  mqttClient.setCredentials(mqtt_user, mqtt_pass);

  // Resolve MQTT broker hostname and connect
  if (resolveMqttServer()) {
    connectToMqtt();
  }

  // Start the DNS re-resolution timer
  xTimerStart(dnsResolveTimer, 0);

  // Set up web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><title>Watch Winder</title>";
    html += "<style>body { font-family: Arial, sans-serif; text-align: center; background-color: #f4f4f4; }";
    html += "label, select, input { display: block; margin: 10px auto; }";
    html += "button { padding: 10px 20px; margin: 10px; background-color: #4CAF50; color: white; border: none; border-radius: 5px; cursor: pointer; }";
    html += "button:hover { background-color: #45a049; }";
    html += "</style></head><body>";
    html += "<h1>Watch Winder Control</h1>";
    html += "<p>Status: <span id='status'>" + String(winderRunning ? "Running" : "Stopped") + "</span></p>";
    html += "<p>Current Direction: <span id='currentDirection'>" + String(isClockwise ? "Clockwise" : "Counterclockwise") + "</span></p>";
    html += "<p>Current RPM: <span id='currentRPM'>" + String(useMinSpeed ? MIN_RPM : MAX_RPM) + "</span></p>";
    html += "<p>Rotation Pattern: <span id='rotationPattern'>" + String(rotationPattern == ALWAYS_CW ? "Always CW" : rotationPattern == ALWAYS_CCW ? "Always CCW" : "Alternate") + "</span></p>";
    html += "<p>Runtime: <span id='runtime'>" + String(runtime) + "</span> seconds</p>";
    html += "<p>Time Until Next Phase: <span id='nextCycle'>0</span> seconds</p>";
    html += "<p>Cycle Count: <span id='cycleCount'>" + String(cycleCount) + "</span></p>";
    html += "<button onclick=\"fetch('/start')\">Start Winder</button>";
    html += "<button onclick=\"fetch('/stop')\">Stop Winder</button>";
    html += "<label for='rotationPattern'>Rotation Pattern:</label>";
    html += "<select id='rotationPattern' onchange=\"fetch('/setRotationPattern?value=' + this.value)\">";
    html += "<option value='cw'" + String(rotationPattern == ALWAYS_CW ? " selected" : "") + ">Always CW</option>";
    html += "<option value='ccw'" + String(rotationPattern == ALWAYS_CCW ? " selected" : "") + ">Always CCW</option>";
    html += "<option value='alternate'" + String(rotationPattern == ALTERNATE ? " selected" : "") + ">Alternate CW/CCW</option>";
    html += "</select>";
    html += "<label for='cwDuration'>CW Duration (minutes):</label>";
    html += "<input type='number' id='cwDuration' value='" + String(CW_DURATION_MINUTES) + "' onchange=\"fetch('/setCWDuration?value=' + this.value)\">";
    html += "<label for='ccwDuration'>CCW Duration (minutes):</label>";
    html += "<input type='number' id='ccwDuration' value='" + String(CCW_DURATION_MINUTES) + "' onchange=\"fetch('/setCCWDuration?value=' + this.value)\">";
    html += "<label for='pauseDuration'>Pause Duration (minutes):</label>";
    html += "<input type='number' id='pauseDuration' value='" + String(PAUSE_DURATION_MINUTES) + "' onchange=\"fetch('/setPauseDuration?value=' + this.value)\">";
    html += "<label for='minRPM'>Min RPM (1-15):</label>";
    html += "<input type='number' id='minRPM' value='" + String(MIN_RPM) + "' min='1' max='15' onchange=\"fetch('/setMinRPM?value=' + this.value)\">";
    html += "<label for='maxRPM'>Max RPM (1-15):</label>";
    html += "<input type='number' id='maxRPM' value='" + String(MAX_RPM) + "' min='1' max='15' onchange=\"fetch('/setMaxRPM?value=' + this.value)\">";
    html += "<label for='varySpeed'>Vary Speed Between Min/Max:</label>";
    html += "<input type='checkbox' id='varySpeed' " + String(varySpeed ? "checked" : "") + " onchange=\"fetch('/setVarySpeed?value=' + (this.checked ? 'true' : 'false'))\">";
    html += "<script>";
    html += "setInterval(async () => {";
    html += "  const response = await fetch('/getStatus');";
    html += "  const data = await response.json();";
    html += "  document.getElementById('status').innerText = data.status;";
    html += "  document.getElementById('currentDirection').innerText = data.currentDirection;";
    html += "  document.getElementById('currentRPM').innerText = data.currentRPM;";
    html += "  document.getElementById('rotationPattern').value = data.rotationPattern;";
    html += "  document.getElementById('rotationPattern').innerText = data.rotationPattern === 'cw' ? 'Always CW' : data.rotationPattern === 'ccw' ? 'Always CCW' : 'Alternate CW/CCW';";
    html += "  document.getElementById('runtime').innerText = data.runtime;";
    html += "  document.getElementById('nextCycle').innerText = data.nextCycle;";
    html += "  document.getElementById('cycleCount').innerText = data.cycleCount;";
    html += "  document.getElementById('cwDuration').value = data.cwDuration;";
    html += "  document.getElementById('ccwDuration').value = data.ccwDuration;";
    html += "  document.getElementById('pauseDuration').value = data.pauseDuration;";
    html += "  document.getElementById('minRPM').value = data.minRPM;";
    html += "  document.getElementById('maxRPM').value = data.maxRPM;";
    html += "  document.getElementById('varySpeed').checked = data.varySpeed === 'true';";
    html += "}, 1000);";
    html += "</script>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request){
    winderRunning = true;
    startTime = millis();
    runtime = 0;
    isClockwise = (rotationPattern != ALWAYS_CCW);
    useMinSpeed = true;
    myStepper.setSpeed(useMinSpeed ? MIN_RPM : MAX_RPM);
    selfPublished = true;
    mqttClient.publish(mqtt_topic_status, 0, false, "running");
    mqttClient.publish(mqtt_topic_current_rpm, 0, false, String(useMinSpeed ? MIN_RPM : MAX_RPM).c_str());
    selfPublished = false;
    if (varySpeed) {
      xTimerStart(speedSwitchTimer, 0);
      Serial.println("Started speed switch timer");
    }
    request->send(200, "text/plain", "Winder started");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    winderRunning = false;
    currentState = STOPPED;
    currentSteps = 0;
    xTimerStop(speedSwitchTimer, 0);
    Serial.println("Stopped speed switch timer");
    selfPublished = true;
    mqttClient.publish(mqtt_topic_status, 0, false, "stopped");
    selfPublished = false;
    request->send(200, "text/plain", "Winder stopped");
  });

  server.on("/setRotationPattern", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      String pattern = request->getParam("value")->value();
      if (pattern == "cw") {
        rotationPattern = ALWAYS_CW;
        isClockwise = true;
      } else if (pattern == "ccw") {
        rotationPattern = ALWAYS_CCW;
        isClockwise = false;
      } else if (pattern == "alternate") {
        rotationPattern = ALTERNATE;
        isClockwise = true; // Start with CW for alternating
      }
      selfPublished = true;
      mqttClient.publish(mqtt_topic_rotation_pattern, 0, false, rotationPattern == ALWAYS_CW ? "cw" : rotationPattern == ALWAYS_CCW ? "ccw" : "alternate");
      selfPublished = false;
    }
    request->send(200, "text/plain", "Rotation pattern set");
  });

  server.on("/setCWDuration", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      unsigned long newDuration = request->getParam("value")->value().toInt();
      if (newDuration > 0) {
        CW_DURATION_MINUTES = newDuration;
        CW_STEPS = CW_DURATION_MINUTES * (useMinSpeed ? MIN_RPM : MAX_RPM) * STEPS_PER_REVOLUTION / 5;
        selfPublished = true;
        mqttClient.publish(mqtt_topic_cw_duration, 0, false, String(CW_DURATION_MINUTES).c_str());
        selfPublished = false;
      }
    }
    request->send(200, "text/plain", "CW duration set");
  });

  server.on("/setCCWDuration", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      unsigned long newDuration = request->getParam("value")->value().toInt();
      if (newDuration > 0) {
        CCW_DURATION_MINUTES = newDuration;
        CCW_STEPS = CCW_DURATION_MINUTES * (useMinSpeed ? MIN_RPM : MAX_RPM) * STEPS_PER_REVOLUTION / 5;
        selfPublished = true;
        mqttClient.publish(mqtt_topic_ccw_duration, 0, false, String(CCW_DURATION_MINUTES).c_str());
        selfPublished = false;
      }
    }
    request->send(200, "text/plain", "CCW duration set");
  });

  server.on("/setPauseDuration", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      unsigned long newDuration = request->getParam("value")->value().toInt();
      if (newDuration > 0) {
        PAUSE_DURATION_MINUTES = newDuration;
        PAUSE_DURATION_MS = PAUSE_DURATION_MINUTES * 60 * 1000UL;
        selfPublished = true;
        mqttClient.publish(mqtt_topic_pause_duration, 0, false, String(PAUSE_DURATION_MINUTES).c_str());
        selfPublished = false;
      }
    }
    request->send(200, "text/plain", "Pause duration set");
  });

  server.on("/setMinRPM", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      unsigned long newRPM = request->getParam("value")->value().toInt();
      if (newRPM >= 1 && newRPM <= 15) {
        MIN_RPM = newRPM;
        if (useMinSpeed) {
          myStepper.setSpeed(MIN_RPM);
          selfPublished = true;
          mqttClient.publish(mqtt_topic_current_rpm, 0, false, String(MIN_RPM).c_str());
          selfPublished = false;
        }
        CW_STEPS = CW_DURATION_MINUTES * MIN_RPM * STEPS_PER_REVOLUTION / 5;
        CCW_STEPS = CCW_DURATION_MINUTES * MIN_RPM * STEPS_PER_REVOLUTION / 5;
        selfPublished = true;
        mqttClient.publish(mqtt_topic_min_rpm, 0, false, String(MIN_RPM).c_str());
        selfPublished = false;
      }
    }
    request->send(200, "text/plain", "Min RPM set");
  });

  server.on("/setMaxRPM", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      unsigned long newRPM = request->getParam("value")->value().toInt();
      if (newRPM >= 1 && newRPM <= 15) {
        MAX_RPM = newRPM;
        if (!useMinSpeed) {
          myStepper.setSpeed(MAX_RPM);
          selfPublished = true;
          mqttClient.publish(mqtt_topic_current_rpm, 0, false, String(MAX_RPM).c_str());
          selfPublished = false;
        }
        CW_STEPS = CW_DURATION_MINUTES * MAX_RPM * STEPS_PER_REVOLUTION / 5;
        CCW_STEPS = CCW_DURATION_MINUTES * MAX_RPM * STEPS_PER_REVOLUTION / 5;
        selfPublished = true;
        mqttClient.publish(mqtt_topic_max_rpm, 0, false, String(MAX_RPM).c_str());
        selfPublished = false;
      }
    }
    request->send(200, "text/plain", "Max RPM set");
  });

  server.on("/setVarySpeed", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      varySpeed = (request->getParam("value")->value() == "true");
      if (winderRunning && varySpeed) {
        xTimerStart(speedSwitchTimer, 0);
        Serial.println("Started speed switch timer");
      } else {
        xTimerStop(speedSwitchTimer, 0);
        Serial.println("Stopped speed switch timer");
        useMinSpeed = true;
        myStepper.setSpeed(MIN_RPM);
        selfPublished = true;
        mqttClient.publish(mqtt_topic_current_rpm, 0, false, String(MIN_RPM).c_str());
        selfPublished = false;
      }
      selfPublished = true;
      mqttClient.publish(mqtt_topic_vary_speed, 0, false, varySpeed ? "true" : "false");
      selfPublished = false;
    }
    request->send(200, "text/plain", "Vary speed set");
  });

  server.on("/getStatus", HTTP_GET, [](AsyncWebServerRequest *request){
    unsigned long nextCycleTime = 0;
    if (winderRunning) {
      unsigned long elapsed = millis() - lastStateChangeTime;
      unsigned long currentRPM = useMinSpeed ? MIN_RPM : MAX_RPM;
      if (currentState == ROTATING_CW && elapsed < (CW_DURATION_MINUTES * 60 * 1000UL)) {
        nextCycleTime = ((CW_DURATION_MINUTES * 60 * 1000UL) - elapsed) / 1000;
      } else if (currentState == ROTATING_CCW && elapsed < (CCW_DURATION_MINUTES * 60 * 1000UL)) {
        nextCycleTime = ((CCW_DURATION_MINUTES * 60 * 1000UL) - elapsed) / 1000;
      } else if (currentState == PAUSING && elapsed < PAUSE_DURATION_MS) {
        nextCycleTime = (PAUSE_DURATION_MS - elapsed) / 1000;
      }
    }
    String json = "{";
    json += "\"status\":\"" + String(winderRunning ? "Running" : "Stopped") + "\",";
    json += "\"currentDirection\":\"" + String(isClockwise ? "Clockwise" : "Counterclockwise") + "\",";
    json += "\"currentRPM\":" + String(useMinSpeed ? MIN_RPM : MAX_RPM) + ",";
    json += "\"rotationPattern\":\"" + String(rotationPattern == ALWAYS_CW ? "cw" : rotationPattern == ALWAYS_CCW ? "ccw" : "alternate") + "\",";
    json += "\"runtime\":" + String(runtime) + ",";
    json += "\"nextCycle\":" + String(nextCycleTime) + ",";
    json += "\"cycleCount\":" + String(cycleCount) + ",";
    json += "\"cwDuration\":" + String(CW_DURATION_MINUTES) + ",";
    json += "\"ccwDuration\":" + String(CCW_DURATION_MINUTES) + ",";
    json += "\"pauseDuration\":" + String(PAUSE_DURATION_MINUTES) + ",";
    json += "\"minRPM\":" + String(MIN_RPM) + ",";
    json += "\"maxRPM\":" + String(MAX_RPM) + ",";
    json += "\"varySpeed\":\"" + String(varySpeed ? "true" : "false") + "\"";
    json += "}";
    request->send(200, "application/json", json);
  });

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();

  if (winderRunning) {
    unsigned long currentTime = millis();

    // Update runtime
    if (startTime > 0) {
      runtime = (currentTime - startTime) / 1000;
      if (currentTime - lastRuntimeUpdate >= 1000) {
        selfPublished = true;
        mqttClient.publish(mqtt_topic_runtime, 0, false, String(runtime).c_str());
        selfPublished = false;
        lastRuntimeUpdate = currentTime;
      }
    }

    switch (currentState) {
      case STOPPED:
        if (rotationPattern == ALWAYS_CCW) {
          currentState = ROTATING_CCW;
        } else {
          currentState = ROTATING_CW;
        }
        currentSteps = 0;
        lastStateChangeTime = currentTime;
        Serial.println(isClockwise ? "Rotating Clockwise..." : "Rotating Counterclockwise...");
        break;

      case ROTATING_CW:
        if (currentSteps < CW_STEPS) {
          myStepper.step(1); // Clockwise
          currentSteps++;
        } else {
          currentState = (rotationPattern == ALWAYS_CW || (rotationPattern == ALTERNATE && !isClockwise)) ? PAUSING : ROTATING_CCW;
          if (currentState == ROTATING_CCW) {
            isClockwise = false;
            currentSteps = 0;
            Serial.println("Rotating Counterclockwise...");
          } else {
            cycleCount++;
            selfPublished = true;
            mqttClient.publish(mqtt_topic_cycle_count, 0, false, String(cycleCount).c_str());
            selfPublished = false;
            Serial.println("Pausing...");
          }
          lastStateChangeTime = currentTime;
        }
        break;

      case ROTATING_CCW:
        if (currentSteps < CCW_STEPS) {
          myStepper.step(-1); // Counterclockwise
          currentSteps++;
        } else {
          currentState = (rotationPattern == ALWAYS_CCW || (rotationPattern == ALTERNATE && isClockwise)) ? PAUSING : ROTATING_CW;
          if (currentState == ROTATING_CW) {
            isClockwise = true;
            currentSteps = 0;
            Serial.println("Rotating Clockwise...");
          } else {
            cycleCount++;
            selfPublished = true;
            mqttClient.publish(mqtt_topic_cycle_count, 0, false, String(cycleCount).c_str());
            selfPublished = false;
            Serial.println("Pausing...");
          }
          lastStateChangeTime = currentTime;
        }
        break;

      case PAUSING:
        if (currentTime - lastStateChangeTime >= PAUSE_DURATION_MS) {
          if (rotationPattern == ALWAYS_CW) {
            currentState = ROTATING_CW;
            isClockwise = true;
          } else if (rotationPattern == ALWAYS_CCW) {
            currentState = ROTATING_CCW;
            isClockwise = false;
          } else {
            currentState = isClockwise ? ROTATING_CW : ROTATING_CCW;
          }
          currentSteps = 0;
          lastStateChangeTime = currentTime;
          Serial.println(isClockwise ? "Rotating Clockwise..." : "Rotating Counterclockwise...");
        }
        break;
    }
  } else {
    currentState = STOPPED;
    currentSteps = 0;
    startTime = 0;
    runtime = 0;
    xTimerStop(speedSwitchTimer, 0);
  }
}

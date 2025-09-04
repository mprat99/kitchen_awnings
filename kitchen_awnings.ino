// =======================
// Includes
// =======================
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <time.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <EEPROM.h>
#include <string.h>
#include <WiFiClient.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <INA226_WE.h>
#include "LittleFS.h"
#include "esp_log.h"


// =======================
// Pin Definitions (Updated)
// =======================

// Stepper 0
#define STEP0      32
#define DIR0       33   
#define ENABLE0    25
#define STALL0     34
#define UART_ADDR0 0    

// Stepper 1
#define STEP1      26
#define DIR1       27
#define ENABLE1    14
#define STALL1     35
#define UART_ADDR1 2

// ESP32 UART pins for TMC UART
#define UART_RX     16
#define UART_TX     17

// Boost converter (XL6019) enable (BJT base)
#define MOTOR_PWR_EN 18

// Solar charger control (kept HIGH, no logic)
#define PV_EN       23
#define BAT_EN      19

// Rain sensor
#define RAIN_PIN    39

// I2C pins
#define I2C_SDA     21
#define I2C_SCL     22

// =======================
// Hardware Configuration
// =======================
#define R_SENSE      0.11f // TMC2209 sense resistor
#define STALL_VALUE  40     // StallGuard threshold

HardwareSerial SerialTMC(2); // UART2 for both drivers

// Two TMC2209 over same UART, different addresses
TMC2209Stepper driver0(&SerialTMC, R_SENSE, UART_ADDR0);
TMC2209Stepper driver1(&SerialTMC, R_SENSE, UART_ADDR1);

// =======================
// FastAccelStepper
// =======================
FastAccelStepperEngine engine;
FastAccelStepper *stepper[2];

// Motion parameters
const uint32_t speed = 7000;      // in steps/s
const uint32_t acceleration = 2000;   // in steps/s²


// Enable pins (active LOW)
int enablePin[] = { ENABLE0, ENABLE1 };
// int stallPin[] = {STALL0, STALL1};

// =======================
// Motion Control Variables
// =======================
bool moveFlag[] = { false, false };
bool homed[] = { false, false };


int targetPosition[]     = { 0, 0 };
int currentPosition[]    = { 0, 0 };
int beforeRainPosition[] = { 0, 0 };

int downPosition[] = { 300000, -300000 };
int maxPosition[]  = { 50000, -50000 };

const char *positionReason[] = {"Power On", "Power On"};
String moveTimeStr[2]        = {"00:00h", "00:00h"};

// Per-motor speed zone flags
bool slowSpeedSet[] = { false, false };
bool fastSpeedSet[] = { false, false };
long startPosition[2];   // where motor started last move
bool upMoveActive[2];    // track if going up
// =======================
// Network Configuration
// =======================
const char *ssid     = "KITCHENWIFI";
const char *password = "RujMfdvc";

const char *apSSID     = "Kitchen Awnings";
const char *apPassword = "Croquets";

IPAddress localIp;

IPAddress staticIP(192, 168, 1, 155);  // ESP32 static IP
IPAddress gateway(192, 168, 1, 1);     // Gateway
IPAddress subnet(255, 255, 255, 0);    // Subnet mask
IPAddress primaryDNS(8, 8, 8, 8);      // Primary DNS
IPAddress secondaryDNS(8, 8, 4, 4);    // Secondary DNS

AsyncWebServer server(80);

const int ssidMaxSize     = 32;
const int passwordMaxSize = 64;

struct WiFiCredentials {
  char ssid[ssidMaxSize];
  char password[passwordMaxSize];
};
WiFiCredentials credentials;

// =======================
// Timing & Scheduling
// =======================
int morningHour   = 7;
int morningMinute = 30;
int nightHour     = 23;
int nightMinute   = 0;

int currentHour   = 0;
int currentMinute = 0;
int currentSecond = 0;

unsigned long currentMillis         = 0;
unsigned long checkTimeMillis       = 0;
unsigned long checkTimeInterval     = 60000;
unsigned long checkRainMillis       = 0;
unsigned long checkRainInterval     = 10000;
unsigned long apStartTime           = 0;
unsigned long highCPUFreqStart      = 0;
unsigned long apTimeout             = 10000;
unsigned long lastAPIrequestMillis  = 0;
unsigned long lastAPIrequestTimeout  = 5000;
unsigned long highCPUFreqTimeout    = 30000;


// =======================
// State Flags
// =======================
bool initialHome      = false;
bool timeSynced       = false;  
bool rainSwitch       = true;
bool scheduledAwnings = false;
bool moveUpMorning    = false;
bool moveDownNight    = false;
bool homeFlag         = false;
bool apModeEnabled    = true;
bool highCPU          = false;

// Use one shared rain flag per motor so logic is independent
int rainState = 0;
bool rainFlag[] = { false, false };

bool settingEnd[] = { false, false };

// Independent stall flags (per motor)
volatile bool stallFlag[] = { false, false };
volatile bool attached[] = {false, false};

// =======================
// INA226 measurement
// // =======================
INA226_WE inaBat(0x40); // Battery V/I
INA226_WE inaPV(0x41);  // PV current

// IMPORTANT: set to your real shunt values if different
float R_SHUNT_BAT = 0.013f; // ohms
float R_SHUNT_PV  = 0.02436f; // ohms

unsigned long inaTriggerMillis = 0;
unsigned long inaReadMillis    = 0;
bool inaMeasuring = false;
unsigned long inaPeriod   = 10000; 

float batteryVoltage = 0.0f;
float batteryCurrent = 0.0f;
float pvCurrent      = 0.0f;
float lowVoltageValue = 9.0f;
bool  lowVoltage     = false; // Battery under 9V
bool chargerInitialized = false;

bool serialDebug = false;

#define DEBUG_PRINT(x)        do { if (serialDebug) Serial.print(x); } while (0)
#define DEBUG_PRINTLN(x)      do { if (serialDebug) Serial.println(x); } while (0)
#define DEBUG_PRINTF(format, ...) \
  do { if (serialDebug) Serial.printf(format, __VA_ARGS__); } while (0)

// =======================
// Function Prototypes
// =======================
void checkTime();
void checkRain();
void IRAM_ATTR stallDetected0();
void IRAM_ATTR stallDetected1();

void initCharger();
void connectCharger();
void disconnectCharger();
void configureServer();
void connectToWiFi();
void printResetReason();
bool checkInternet();

String getMoveTimeString();

void home();
void moveUp(uint8_t i);
void moveDown(uint8_t i);
void stop(uint8_t i);

void motorPowerOn();
void motorPowerOff();

void inaSetup();
void inaTrigger();
void inaRead();

// HTTP handlers you referenced (kept minimal but functional)
void handleMove(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleSave(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleSetEnd(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleSaveWifi(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);

void setupDrivers() {
  driver0.begin();
  driver0.toff(4);
  driver0.blank_time(24);
  driver0.rms_current(1600);
  driver0.microsteps(16);
  driver0.en_spreadCycle(false);
  driver0.pwm_autoscale(true);
  driver0.TCOOLTHRS(1000);
  driver0.SGTHRS(STALL_VALUE);

  driver1.begin();
  driver1.toff(4);
  driver1.blank_time(24);
  driver1.rms_current(1600);
  driver1.microsteps(16);
  driver1.en_spreadCycle(false);
  driver1.pwm_autoscale(true);
  driver1.TCOOLTHRS(1000);
  driver1.SGTHRS(STALL_VALUE);


}

void setupSteppers() {
  pinMode(ENABLE0, OUTPUT);
  pinMode(ENABLE1, OUTPUT);
  digitalWrite(ENABLE0, HIGH); // disable initially
  digitalWrite(ENABLE1, HIGH);

  pinMode(STEP0, OUTPUT);
  pinMode(DIR0, OUTPUT);
  pinMode(STALL0, INPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(STALL1, INPUT);

  engine.init();
  
  stepper[0] = engine.stepperConnectToPin(STEP0);
  stepper[1] = engine.stepperConnectToPin(STEP1);
  
  stepper[0]->setDirectionPin(DIR0);
  stepper[0]->setEnablePin(ENABLE0, true);
  stepper[0]->setSpeedInHz(speed);
  stepper[0]->setAcceleration(2000);
  stepper[0]->setAutoEnable(true);
  
  stepper[1]->setDirectionPin(DIR1);
  stepper[1]->setEnablePin(ENABLE1, true);
  stepper[1]->setSpeedInHz(speed);
  stepper[1]->setAcceleration(2000);
  stepper[1]->setAutoEnable(true);
}

// =======================
// Setup
// =======================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(200);

  esp_log_level_set("i2c.master", ESP_LOG_NONE);

  if (Serial.availableForWrite()) {
    serialDebug = true;
    DEBUG_PRINTLN("Serial connected. Debug prints enabled.");
  }

  // Pins
  pinMode(MOTOR_PWR_EN, OUTPUT);
  digitalWrite(MOTOR_PWR_EN, HIGH); // motors power OFF initially


  pinMode(BAT_EN, OUTPUT);
  // digitalWrite(BAT_EN, HIGH);

  delay(300); // first connect battery, then PV
  
  pinMode(PV_EN, OUTPUT);
  // digitalWrite(PV_EN, HIGH);

  pinMode(RAIN_PIN, INPUT);

  motorPowerOn();


  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  inaSetup();

  // UART for TMC2209
  SerialTMC.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  setupDrivers();
  setupSteppers();
  delay(500);

  // Reset reason
  printResetReason();

  // EEPROM credentials
  EEPROM.begin(sizeof(WiFiCredentials));
  EEPROM.get(0, credentials);
  delay(100);

  // Stall interrupts per motor
  attachInterrupt(digitalPinToInterrupt(STALL0), stallDetected0, RISING);
  attachInterrupt(digitalPinToInterrupt(STALL1), stallDetected1, RISING);

  // Initial INA read
  inaTrigger();
  delay(9);
  inaRead();

  if (!lowVoltage) {
    initCharger();
    rainState = digitalRead(RAIN_PIN); 
    if (rainState) {
      home();
    }
    checkRain();
  }

  if(!LittleFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  connectToWiFi();
  delay(150);
  configureServer();
  delay(150);
  server.begin();

  // Disable Bluetooth
  btStop();

}

// =======================
// Web Server Routes
// =======================
void configureServer() {

  // ------------------------------
  // Serve main page
  // ------------------------------
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    inaPeriod = 2000;
    inaRead();
    request->send(LittleFS, "/index.html", "text/html");
  });

  // ------------------------------
  // Serve CSS
  // ------------------------------
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/style.css", "text/css");
  });

  // ------------------------------
  // Serve sensor/data JSON
  // ------------------------------
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    inaRead();
    lastAPIrequestMillis = currentMillis;
    // if (batteryCurrent < 40.0f) {
    //   R_SHUNT_BAT -= 0.001;
    //   inaBat.setResistorRange(R_SHUNT_BAT, 4.0f);
    // }

    float awning0Percent = 100.0f * ((float)stepper[0]->getCurrentPosition()) / ((float)downPosition[0]);
    float awning1Percent = 100.0f * ((float)stepper[1]->getCurrentPosition()) / ((float)downPosition[1]);
    int dist0 = stepper[0]->targetPos() - stepper[0]->getCurrentPosition();
    int dist1 = stepper[1]->targetPos() - stepper[1]->getCurrentPosition();
    String json = "{";
    json += "\"batteryVoltage\":" + String(batteryVoltage, 2) + ",";
    json += "\"batteryCurrent\":" + String(batteryCurrent, 2) + ",";
    json += "\"pvCurrent\":" + String(pvCurrent, 2) + ",";
    json += "\"pos0\":" + String(stepper[0]->getCurrentPosition()) + ",";
    json += "\"pos1\":" + String(stepper[1]->getCurrentPosition()) + ",";
    json += "\"dist0\":" + String(dist0) + ",";
    json += "\"dist1\":" + String(dist1) + ",";
    json += "\"awning0Percent\":" + String(awning0Percent, 1) + ",";
    json += "\"awning1Percent\":" + String(awning1Percent, 1) + ",";
    json += "\"homeFlag\":" + String(homeFlag) + ",";
    json += "\"raining\":" + String(rainState);
    // json += "\"R_SHUNT_BAT\":" + String(R_SHUNT_BAT, 5);
    json += "}";

    request->send(200, "application/json", json);
  });


  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {

    String json = "{";
    json += "\"currentHour\":" + String(currentHour) + ",";
    json += "\"currentMinute\":" + String(currentMinute) + ",";
    json += "\"currentSecond\":" + String(currentSecond) + ",";
    json += "\"moveUpMorning\":" + String(moveUpMorning) + ",";
    json += "\"moveDownNight\":" + String(moveDownNight) + ",";
    json += "\"automaticDayNight\":" + String(scheduledAwnings) + ",";
    json += "\"morningHour\":" + String(morningHour) + ",";
    json += "\"morningMinute\":" + String(morningMinute) + ",";
    json += "\"nightHour\":" + String(nightHour) + ",";
    json += "\"nightMinute\":" + String(nightMinute) + ",";
    json += "\"timeSynced\":" + String(timeSynced) + ",";
  json += "\"positionReason0\":\"" + String(positionReason[0]) + "\",";
  json += "\"positionReason1\":\"" + String(positionReason[1]) + "\"";
    json += "}";

    request->send(200, "application/json", json);
  });
  // ------------------------------
  // POST routes with body handlers
  // ------------------------------
  server.on("/move", HTTP_POST,
            [](AsyncWebServerRequest *request) {
              // request->send(200, "text/plain", "Data received");
            },
            NULL,  // No upload handler needed
            handleMove);

  server.on("/saveConfig", HTTP_POST,
            [](AsyncWebServerRequest *request) {
              // request->send(200, "text/plain", "Data Saved");
            },
            NULL,
            handleSave);

  server.on("/setEndPosition", HTTP_POST,
            [](AsyncWebServerRequest *request) {
            },
            NULL,
            handleSetEnd);

  server.on("/saveWifi", HTTP_POST,
            [](AsyncWebServerRequest *request) {
              // request->send(200, "text/plain", "Data Saved");
            },
            NULL,
            handleSaveWifi);

  // ------------------------------
  // Special POST route without body
  // ------------------------------
  server.on("/setHome", HTTP_POST, [](AsyncWebServerRequest *request) {
    positionReason[0] = "Home Button";
    positionReason[1] = "Home Button";
    home();
    request->send(200, "text/plain", "Homing");
  });

}

// =======================
// Interrupts
// =======================
void IRAM_ATTR stallDetected0() { stallFlag[0] = true; }
void IRAM_ATTR stallDetected1() { stallFlag[1] = true; }
// void IRAM_ATTR stallDetected() {
//   if (digitalRead(STALL0) == HIGH) stallFlag[0] = true;
//   if (digitalRead(STALL1) == HIGH) stallFlag[1] = true;
// }



// =======================
// Helpers
// =======================
void initCharger() {
  disconnectCharger();
  delay(1000);
  connectCharger();
  chargerInitialized = true;
}

void connectCharger() {
  digitalWrite(BAT_EN, LOW);
  delay(500);
  digitalWrite(PV_EN, LOW);
}

void disconnectCharger() {
  digitalWrite(PV_EN, HIGH);
  delay(500);
  digitalWrite(BAT_EN, HIGH);

}

String getMoveTimeString() {
  String hh = (currentHour < 10) ? "0" + String(currentHour) : String(currentHour);
  String mm = (currentMinute < 10) ? "0" + String(currentMinute) : String(currentMinute);
  return (hh + ":" + mm + "h");
}

void motorPowerOn()  { 
  digitalWrite(MOTOR_PWR_EN, LOW); 
  }

void motorPowerOff() {
  // only power off if both motors idle and disabled
  if (!moveFlag[0] && !moveFlag[1] &&
      digitalRead(ENABLE0) == HIGH && digitalRead(ENABLE1) == HIGH) {
    digitalWrite(MOTOR_PWR_EN, HIGH);
  }
}

// =======================
// Replace home() function
// =======================
void home() {
  initialHome = true;
  homeFlag = true;
  String t = getMoveTimeString();
  for (int i = 0; i < 2; i++) {
    homed[i] = false;
    moveFlag[i] = true;
    stepper[i]->setCurrentPosition(maxPosition[i]);
    stepper[i]->moveTo(0);
    moveTimeStr[i] = t;
  }
}

void handleMove(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  DynamicJsonDocument json(1024);
  DeserializationError err = deserializeJson(json, data, len);
  if (err) { 
    request->send(400, "text/plain", "Bad JSON"); 
    return; 
  }

  const char *buttonId = json["buttonId"];
  if (!buttonId) { 
    request->send(400, "text/plain", "Missing buttonId"); 
    return; 
  }

  int i = (buttonId[strlen(buttonId) - 1] == '0') ? 0 : 1;

  if (i < 0 || i >= 2) {
    request->send(400, "text/plain", "Invalid motor index");
    return;
  }

  positionReason[i] = "Remote button";
  moveTimeStr[i] = getMoveTimeString();

  if (strstr(buttonId, "up") != NULL) {
    moveUp(i);
    request->send(200, "text/plain", "Moving up");
  }
  else if (strstr(buttonId, "stop") != NULL) {
    stop(i);
    request->send(200, "text/plain", "Stopping");
  }
  else if (strstr(buttonId, "down") != NULL) {
    moveDown(i);
    request->send(200, "text/plain", "Moving down");
  }
  else {
    request->send(400, "text/plain", "Unknown command");
  }
}

void moveUp(uint8_t i) {
  targetPosition[i] = 0;
  move(i);
}

void stop(uint8_t i) {
  stepper[i]->forceStop();
  currentPosition[i] = stepper[i]->getCurrentPosition();
  targetPosition[i] = currentPosition[i];
}

void moveDown(uint8_t i) {
  targetPosition[i] = settingEnd[i] ? maxPosition[i] : downPosition[i];
  move(i);
}

void move(uint8_t i) {
  if (digitalRead(MOTOR_PWR_EN) == HIGH) {
    motorPowerOn();
    }
  stepper[i]->moveTo(targetPosition[i]); 
  moveFlag[i] = true;
}

void checkMotors() {
  for (int i = 0; i < 2; i++) {
    if (stallFlag[i]) {
      stepper[i]->forceStop();
      stallFlag[i] = false;
    }
    if (moveFlag[i] && !stepper[i]->isRunning()) {

      if (homeFlag) {
        stepper[i]->setCurrentPosition(0);
        homed[i] = true;
        if (homed[0] && homed[1]) {
          homeFlag = false;
        }
      }
      currentPosition[i] = stepper[i]->getCurrentPosition();
      targetPosition[i] = currentPosition[i];
      moveFlag[i] = false;
      motorPowerOff();
    }
  }
}

// =======================
// INA226 Setup & Non-blocking Trigger/Read
// =======================
void inaSetup() {
  // Battery INA
  if (!inaBat.init()) {
    DEBUG_PRINTLN("INA226 battery init failed!");
  }
  inaBat.setAverage(AVERAGE_4);
  inaBat.setConversionTime(CONV_TIME_1100); // ~8.2ms
  inaBat.setMeasureMode(TRIGGERED);
  inaBat.setResistorRange(R_SHUNT_BAT, 4.0f);

  // PV INA
  if (!inaPV.init()) {
    DEBUG_PRINTLN("INA226 PV init failed!");
  }
  inaPV.setAverage(AVERAGE_4);
  inaPV.setConversionTime(CONV_TIME_1100);
  inaPV.setMeasureMode(TRIGGERED);
  inaPV.setResistorRange(R_SHUNT_PV, 4.0f);
}

void inaTrigger() {
  inaBat.startSingleMeasurement();
  inaPV.startSingleMeasurement();
  inaTriggerMillis = currentMillis;
  inaReadMillis = currentMillis;
  inaMeasuring  = true;
}

void inaRead() {
  batteryVoltage = inaBat.getBusVoltage_V();
  batteryCurrent = inaBat.getCurrent_mA();
  pvCurrent      = inaPV.getCurrent_mA();
  lowVoltage     = (batteryVoltage < lowVoltageValue);
  inaMeasuring = false;
}


// =======================
// Time / WiFi / AP handling
// =======================
void printResetReason() {
  esp_reset_reason_t resetReason = esp_reset_reason();
  const char* reason;

  switch (resetReason) {
    case ESP_RST_UNKNOWN:    reason = "Reset reason: Unknown"; break;
    case ESP_RST_POWERON:    reason = "Reset reason: Power-on"; break;
    case ESP_RST_EXT:        reason = "Reset reason: External pin reset"; break;
    case ESP_RST_SW:         reason = "Reset reason: Software reset via esp_restart"; break;
    case ESP_RST_PANIC:      reason = "Reset reason: Panic (software crash)"; break;
    case ESP_RST_INT_WDT:    reason = "Reset reason: Interrupt watchdog"; break;
    case ESP_RST_TASK_WDT:   reason = "Reset reason: Task watchdog"; break;
    case ESP_RST_WDT:        reason = "Reset reason: Other watchdog"; break;
    case ESP_RST_DEEPSLEEP:  reason = "Reset reason: Deep sleep"; break;
    case ESP_RST_BROWNOUT:   reason = "Reset reason: Brownout"; break;
    case ESP_RST_SDIO:       reason = "Reset reason: SDIO"; break;
    default:                 reason = "Reset reason: Unknown"; break;
  }

  positionReason[0] = reason;
  positionReason[1] = reason;
  String t = getMoveTimeString();
  moveTimeStr[0] = t;
  moveTimeStr[1] = t;

  DEBUG_PRINTLN(moveTimeStr[0]);
  DEBUG_PRINTLN(reason);
}

void connectToWiFi() {
  // AP+STA at boot
  WiFi.mode(WIFI_AP_STA);
  delay(50);

  // Configure static IP for STA
  if (!WiFi.config(staticIP, gateway, subnet, primaryDNS, secondaryDNS)) {
    DEBUG_PRINTLN("Failed to configure Static IP");
  } else {
    DEBUG_PRINTLN("Static IP configured!");
  }

  // Connect to stored STA credentials
  WiFi.begin(credentials.ssid, credentials.password);

  // Basic connection attempts without LED logic
  for (uint8_t i = 0; i < 5; i++) {
    if (WiFi.status() != WL_CONNECTED) {
      DEBUG_PRINTLN("Connecting to WiFi...");
      delay(1000);
    } else {
      DEBUG_PRINTLN("Connected to WiFi");
      DEBUG_PRINTLN(WiFi.localIP());
      localIp = WiFi.localIP();
      // NTP
      const char* tz = "CET-1CEST,M3.5.0,M10.5.0/3";
      setenv("TZ", tz, 1);
      tzset();
      configTime(0, 0, "pool.ntp.org");
      break;
    }
  }

  // Start AP (auto-disable after 10s if no clients)
  WiFi.softAP(apSSID, apPassword);
  apStartTime = millis();
  DEBUG_PRINTLN("Access Point started");
  DEBUG_PRINT("AP IP: ");
  DEBUG_PRINTLN(WiFi.softAPIP());

}

bool checkInternet() {
  IPAddress testIP;
  return WiFi.hostByName("www.google.com", testIP);
}

void checkTime() {
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  currentHour = timeinfo->tm_hour;
  currentMinute = timeinfo->tm_min;
  currentSecond = timeinfo->tm_sec;

  DEBUG_PRINTF("Current time: %02d:%02d:%02d\n", currentHour, currentMinute, currentSecond);

  if (scheduledAwnings) {
    if (currentHour == morningHour && currentMinute == morningMinute && !moveUpMorning) {
      for (uint8_t i = 0; i < 2; i++) {
        if (currentPosition[i] != 0 && !rainFlag[i]) {
          moveUpMorning = true;
          moveUp(i);
          positionReason[i] = "Scheduled Morning";
          moveTimeStr[i] = getMoveTimeString();
        }
      }
    } else if (currentHour == nightHour && currentMinute == nightMinute && !moveDownNight) {
      for (uint8_t i = 0; i < 2; i++) {
        if (!rainFlag[i]) {
          moveDown(i);
          positionReason[i] = "Scheduled Night";
          moveTimeStr[i] = getMoveTimeString();
        }
      }
      moveUpMorning = false;
      moveDownNight  = true;
    } else if (moveUpMorning && (currentHour != morningHour || currentMinute != morningMinute)) {
      moveUpMorning = false;
    } else if (moveDownNight && (currentHour != nightHour || currentMinute != nightMinute)) {
      moveDownNight = false;
    }
  }

  checkTimeMillis = millis() + 60000UL;

}

bool isNTPReady() {
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  return (now > 1609459200UL) && (timeinfo->tm_hour != 1 || timeinfo->tm_min > 0);
}

void syncToNextMinute() {
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  
  unsigned long secondsLeft = 60 - timeinfo->tm_sec;
  checkTimeMillis = millis() + (secondsLeft * 1000UL);
}

// =======================
// Rain
// =======================

void checkRain() {
  checkRainMillis = currentMillis;
  rainState = digitalRead(RAIN_PIN);

  if (rainState == LOW) {
    for (uint8_t i = 0; i < 2; i++) {
      if (!rainFlag[i]) {
        rainFlag[i] = true;
        beforeRainPosition[i] = currentPosition[i];
        moveDown(i);
        positionReason[i] = "It's Raining";
        moveTimeStr[i] = getMoveTimeString();
      }
    }
  } else { // HIGH
    for (uint8_t i = 0; i < 2; i++) {
      if (rainFlag[i]) {
        rainFlag[i] = false;
        if (initialHome) {
          if (!moveDownNight) {
            int target = (moveUpMorning) ? 0 : beforeRainPosition[i];
            if (currentPosition[i] != target) {
              targetPosition[i] = target;
              moveUp(i);
              positionReason[i] = "Stopped Raining";
              moveTimeStr[i] = getMoveTimeString();
            }
            moveUpMorning = false;
          } else if (currentPosition[i] != beforeRainPosition[i]) {
            moveDown(i);
            positionReason[i] = "Stopped Raining";
            moveTimeStr[i] = getMoveTimeString();
          }
        } else {
          home();
          positionReason[0] = "Home After Rain";
          positionReason[1] = "Home After Rain";
          String t = getMoveTimeString();
          moveTimeStr[0] = t;
          moveTimeStr[1] = t;
        }
      }
    }
  }
}

// =======================
// HTTP minimal handlers
// =======================

void handleSave(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t, size_t) {
  // Placeholder in your original; keep minimal parse so it's functional
  DynamicJsonDocument json(1024);
  if (deserializeJson(json, data, len)) return;
  // Example optional fields:
  if (json.containsKey("morningHour"))   morningHour   = json["morningHour"];
  if (json.containsKey("morningMinute")) morningMinute = json["morningMinute"];
  if (json.containsKey("nightHour"))     nightHour     = json["nightHour"];
  if (json.containsKey("nightMinute"))   nightMinute   = json["nightMinute"];
  if (json.containsKey("scheduled"))     scheduledAwnings = json["scheduled"];
  request->send(200, "text/plain", "Data Saved");
}

void handleSetEnd(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t, size_t) {
  DynamicJsonDocument json(1024);
  if (deserializeJson(json, data, len)) return;
  // Accept { motor:0/1, down:-N, max:-M, setEnd:true/false }
  int m = json["motor"] | 0;
  if (m < 0 || m > 1) m = 0;
  if (json.containsKey("down")) downPosition[m] = json["down"];
  if (json.containsKey("max"))  maxPosition[m]  = json["max"];
  if (json.containsKey("setEnd")) settingEnd[m] = json["setEnd"];
  request->send(200, "text/plain", "Data Saved");
}

void handleSaveWifi(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t, size_t) {
  DynamicJsonDocument json(512);
  if (deserializeJson(json, data, len)) return;
  const char* ss = json["ssid"] | "";
  const char* pw = json["password"] | "";
  strncpy(credentials.ssid, ss, ssidMaxSize);
  credentials.ssid[ssidMaxSize-1] = 0;
  strncpy(credentials.password, pw, passwordMaxSize);
  credentials.password[passwordMaxSize-1] = 0;
  EEPROM.put(0, credentials);
  EEPROM.commit();
}


// =======================
// Main loop
// =======================
void loop() {
  currentMillis = millis();


  if (currentMillis - inaTriggerMillis >= inaPeriod) {
    inaTrigger();
  }

  if (inaMeasuring && (currentMillis - inaReadMillis >= 9)) { 
    inaRead();
  }

  if (currentMillis - lastAPIrequestMillis >= lastAPIrequestTimeout) {
    inaPeriod = 10000;
  }

  // // AP auto-off after 10s if no clients
  if (apModeEnabled && (currentMillis - apStartTime >= apTimeout)) {
    uint8_t connectedDevices = WiFi.softAPgetStationNum();
    if (connectedDevices == 0) {
      DEBUG_PRINTLN("No devices connected to AP, disabling AP mode");
      WiFi.softAPdisconnect(true);
      apModeEnabled = false;
    }
  }

  if (!timeSynced && isNTPReady()) {
    syncToNextMinute();  // Align to next HH:MM:00
    timeSynced = true;
  }
  

  if (!lowVoltage) {
    DEBUG_PRINTLN("NO LOW VOLTAGE");
    DEBUG_PRINTLN(currentMillis);
    DEBUG_PRINTLN(checkTimeMillis);
    if (currentMillis >= checkTimeMillis) {
          DEBUG_PRINTLN("currentMillis >= checkTimeMillis");
          
      if (!chargerInitialized && !moveFlag[0] && !moveFlag[1]) {
        initCharger();
      }
      if (!checkInternet()) {
        connectToWiFi();
        timeSynced = false;
      }
      if (timeSynced) {
        checkTime();
      }     
    }

    if (currentMillis - checkRainMillis >= checkRainInterval) {
      checkRain();
    }
    
    checkMotors();
  } 

}

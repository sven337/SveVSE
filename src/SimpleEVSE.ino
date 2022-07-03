/*
  Copyright (c) 2018 CurtRod

  Released to Public Domain

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/
#include <Arduino.h>
#include "features.h"

#include <ESP8266WiFi.h>              // Whole thing is about using Wi-Fi networks
#include <ESP8266mDNS.h>              // Zero-config Library (Bonjour, Avahi)
#include <ESPAsyncTCP.h>              // Async TCP Library is mandatory for Async Web Server
#include <FS.h>                       // SPIFFS Library for storing web files to serve to web browsers
#include <WiFiUdp.h>                  // Library for manipulating UDP packets which is used by NTP Client to get Timestamps
#include <SoftwareSerial.h>           // Using GPIOs for Serial Modbus communication


#include <TimeLib.h>                  // Library for converting epochtime to a date
#include <SPI.h>                      // SPI protocol
#include <ArduinoJson.h>              // JSON Library for Encoding and Parsing Json object to send browser
#include <ESPAsyncWebServer.h>        // Async Web Server with built-in WebSocket Plug-in
#include <SPIFFSEditor.h>             // This creates a web page on server which can be used to edit text based files

#include <ModbusMaster.h>
#include <ModbusIP_ESP8266.h>

#include "mqtt_params.h"
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <string>
#include <deque>
#include "proto.h"
#include "ntp.h"
#include "syslog.h"
#include "websrc.h"
#include "config.h"
#include "templates.h"

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVER_PORT, MQTT_USER, MQTT_PASSWORD);

Adafruit_MQTT_Subscribe mqtt_PAPP = Adafruit_MQTT_Subscribe(&mqtt, "edf/PAPP");
Adafruit_MQTT_Subscribe mqtt_ADPS = Adafruit_MQTT_Subscribe(&mqtt, "edf/ADPS");

#define PAPP_SUBSCRIBED 6000
#define PAPP_MARGIN 1000
uint16_t PAPP_now = 0;
bool PAPP_updated = false;
uint32_t ADPS_stop_until = 0;

uint8_t sw_min = 3; //Firmware Minor Version
uint8_t sw_rev = 0; //Firmware Revision
String sw_add = "";

#ifdef ESP8266
uint8_t sw_maj = 2; //Firmware Major Version
String swVersion = String(sw_maj) + "." + String(sw_min) + "." + String(sw_rev) + sw_add;
#endif

//////////////////////////////////////////////////////////////////////////////////////////
///////       Variables For Whole Scope
//////////////////////////////////////////////////////////////////////////////////////////
//EVSE Variables
bool noEVSE = false;
uint16_t reg2005DefaultValues = 0;
bool highResolution = false;
uint32_t startChargingTimestamp = 0;
uint32_t stopChargingTimestamp = 0;
bool manualStop = false;
uint16_t currentToSet = 6;
uint16_t currentBeforeRemoteTimeout = 0;
uint8_t evseStatus = 0;
bool evseActive = false;
bool vehicleCharging = false;
bool timerActive = false;
int buttonState = HIGH;
int prevButtonState = HIGH;
AsyncWebParameter* awp;
AsyncWebParameter* awp2;
const char * initLog = "{\"type\":\"latestlog\",\"list\":[]}";
bool sliderStatus = true;
uint8_t evseErrorCount = 0;
bool update1000 = true;
bool update2000 = true;
unsigned long updateInterval = 2000;
bool updateHelper1000 = false;
uint32_t mbErrCount = 0;
uint32_t mbReadCount = 0;


//objects and instances
SoftwareSerial SecondSer(D1, D2); //SoftwareSerial object (RX, TX)


ModbusMaster evseNode;
AsyncWebServer server(80);    // Create AsyncWebServer instance on port "80"
AsyncWebSocket ws("/ws");     // Create WebSocket instance on URL "/ws"
NtpClient ntp;
EvseWiFiConfig config = EvseWiFiConfig();
//DynamicJsonDocument syslogJson(24576);
std::deque<String> syslogDeque;

unsigned long lastModbusAction = 0;
unsigned long buttonTimer = 0;

//Loop
unsigned long millisUpdateEvse = 0;
unsigned long millisRemoteHeartbeat = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long previousLoopMillis = 0;
unsigned long previousLedAction = 0;
unsigned long reconnectTimer = 0;
unsigned long millisCheckTimer = 10000; // wait 10 sec to start timer
uint16_t remoteHeartbeatCounter = 0;
uint16_t toChangeLedOnTime = 0;
uint16_t toChangeLedOffTime = 0;
uint16_t ledOnTime = 100;
uint16_t ledOffTime = 4000;
bool wifiInterrupted = false;
bool ledStatus = false;
bool toSetEVSEcurrent = false;
bool toActivateEVSE = false;
bool toDeactivateEVSE = false;
bool toInitLog = false;
bool toSendStatus = false;
bool toSendSyslogToWs = false;
bool toReboot = false;
bool fsWorking = false;
bool deactivatedByRemoteHeartbeat = false;

//EVSE Modbus Registers
uint16_t evseAmpsConfig;    //Register 1000
uint16_t evseAmpsOutput;    //Register 1001
uint16_t evseVehicleState;  //Register 1002
uint16_t evseAmpsPP;        //Register 1003
uint16_t evseTurnOff;       //Register 1004
uint16_t evseFirmware;      //Register 1005
uint16_t evseEvseState;     //Register 1006
uint16_t evseRcdStatus;     //Register 1007
s_addEvseData addEvseData;

//Settings
bool dontUseWsAuthentication = false;
bool dontUseLED = false;
bool resetCurrentAfterCharge = false;
bool inAPMode = false;
bool inFallbackMode = false;
bool isWifiConnected = false;
String lastUsername = "";
String lastUID = "";
char * deviceHostname = NULL;
Syslog slog;

//Others
String msg = ""; //WS communication

//////////////////////////////////////////////////////////////////////////////////////////
///////       Auxiliary Functions
//////////////////////////////////////////////////////////////////////////////////////////

void ICACHE_FLASH_ATTR updateRemoteHeartbeat() {
  if (remoteHeartbeatCounter >= 60) {
    if (evseAmpsConfig > 0) {
      currentBeforeRemoteTimeout = evseAmpsConfig;
      toSetEVSEcurrent = true;
      currentToSet = 0;
    }
    Serial.println("Deactivated by lack of remote heartbeat");
    deactivatedByRemoteHeartbeat = true;
  }
  else {
    if (deactivatedByRemoteHeartbeat) {
      toSetEVSEcurrent = true;
      currentToSet = currentBeforeRemoteTimeout;
    }
    deactivatedByRemoteHeartbeat = false;
  }
  remoteHeartbeatCounter ++;
  millisRemoteHeartbeat = millis() + 1000;
}

uint16_t get16bitOfFloat32 (float float_number, uint8_t offset){
  union {
    float f_number;
    uint16_t uint16_arr[2];
  } union_for_conv;  
  union_for_conv.f_number = float_number;
  uint16_t ret = union_for_conv.uint16_arr[offset];
  return ret;
}

void ICACHE_FLASH_ATTR handleLed() {
  if (currentMillis >= previousLedAction && config.getEvseLedConfig(0) != 1) {
    if (ledStatus == false) {
      if (currentMillis >= previousLedAction + ledOffTime) {
        digitalWrite(config.getEvseLedPin(0), HIGH);
        ledStatus = true;
        previousLedAction = currentMillis;
      }
    }
    else {
      if (currentMillis >= previousLedAction + ledOnTime) {
        digitalWrite(config.getEvseLedPin(0), LOW);
        ledStatus = false;
        previousLedAction = currentMillis;
      }
    }
  }
}

void ICACHE_FLASH_ATTR changeLedStatus() {
  if (ledOnTime != toChangeLedOnTime) {
    ledOnTime = toChangeLedOnTime;
  }
  if (ledOffTime != toChangeLedOffTime) {
    ledOffTime = toChangeLedOffTime;
  }
}

void ICACHE_FLASH_ATTR changeLedTimes(uint16_t onTime, uint16_t offTime) {
  if (ledOnTime != onTime) {
    toChangeLedOnTime = onTime;
  }
  if (ledOffTime != offTime) {
    toChangeLedOffTime = offTime;
  }
}

String ICACHE_FLASH_ATTR printIP(IPAddress address) {
  return (String)address[0] + "." + (String)address[1] + "." + (String)address[2] + "." + (String)address[3];
}




unsigned long ICACHE_FLASH_ATTR getChargingTime() {
  unsigned long iTime;
  if (vehicleCharging == true) {
    //iTime = millis() - millisStartCharging;
    iTime = (ntp.getUtcTimeNow() - startChargingTimestamp) * 1000;

  }
  else {
    //iTime = millisStopCharging - millisStartCharging;
    iTime = (stopChargingTimestamp - startChargingTimestamp) * 1000;
  }
  return iTime;
}

bool ICACHE_FLASH_ATTR resetUserData() {
  Dir userdir = SPIFFS.openDir("/P/");
  while(userdir.next()){
    slog.logln(userdir.fileName());
    SPIFFS.remove(userdir.fileName());
  }
  delay(10);
  return true;
}

bool ICACHE_FLASH_ATTR factoryReset() {
  if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Factory Reset...");
  SPIFFS.remove("/config.json");
  initLogFile();
  if (resetUserData()) {
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] ...successfully done - going to reboot");
  }
  toReboot = true;
  delay(100);
  return true;
}

bool ICACHE_FLASH_ATTR reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(config.getWifiSsid(), config.getWifiPass(), 0);
  if(config.getSystemDebug()) slog.log(ntp.iso8601DateTime() + "[Info] Trying to reconnect WiFi without given BSSID: ");
  if(config.getSystemDebug()) slog.logln(config.getWifiSsid());
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////       RFID Functions
//////////////////////////////////////////////////////////////////////////////////////////
void ICACHE_FLASH_ATTR sendStatus() {
  fsWorking = true;
  struct ip_info info;
  FSInfo fsinfo;
  if (!SPIFFS.info(fsinfo)) {
    slog.log(ntp.iso8601DateTime() + "[ WARN ] Error getting info on SPIFFS, trying another way");
  }
  delay(10);
  fsWorking = false;
  StaticJsonDocument<1000> jsonDoc;
  jsonDoc["command"] = "status";
  jsonDoc["heap"] = ESP.getFreeHeap();
  jsonDoc["availsize"] = ESP.getFreeSketchSpace();
  jsonDoc["flashsize"] = ESP.getFlashChipSize();
  jsonDoc["cpu"] = ESP.getCpuFreqMHz();
  jsonDoc["uptime"] = ntp.getDeviceUptimeString();
  jsonDoc["devicetime"] = ntp.getDeviceUptimeString();
  
  #ifdef ESP8266
  jsonDoc["chipid"] = String(ESP.getChipId(), HEX);
  jsonDoc["availspiffs"] = fsinfo.totalBytes - fsinfo.usedBytes;
  jsonDoc["spiffssize"] = fsinfo.totalBytes;
  jsonDoc["hardwarerev"] = "ESP8266";
  #endif


  #ifdef ESP8266
  if (inAPMode) {
    wifi_get_ip_info(SOFTAP_IF, &info);
    struct softap_config conf;
    wifi_softap_get_config(&conf);
    jsonDoc["ssid"] = String(reinterpret_cast<char*>(conf.ssid));
    jsonDoc["dns"] = printIP(WiFi.softAPIP());
    jsonDoc["mac"] = WiFi.softAPmacAddress();
  }
  else {
    wifi_get_ip_info(STATION_IF, &info);
    struct station_config conf;
    wifi_station_get_config(&conf);
    jsonDoc["ssid"] = String(reinterpret_cast<char*>(conf.ssid));
    jsonDoc["rssi"] = String(WiFi.RSSI());
    jsonDoc["dns"] = printIP(WiFi.dnsIP());
    jsonDoc["mac"] = WiFi.macAddress();
  }
  IPAddress ipaddr = IPAddress(info.ip.addr);
  IPAddress gwaddr = IPAddress(info.gw.addr);
  IPAddress nmaddr = IPAddress(info.netmask.addr);

  jsonDoc["ip"] = printIP(ipaddr);
  jsonDoc["netmask"] = printIP(nmaddr);
  jsonDoc["hostname"] = wifi_station_get_hostname();

  #endif

  jsonDoc["gateway"] = printIP(gwaddr);
  jsonDoc["evse_amps_conf"] = evseAmpsConfig;          //Reg 1000
  jsonDoc["evse_amps_out"] = evseAmpsOutput;           //Reg 1001
  jsonDoc["evse_vehicle_state"] = evseVehicleState;    //Reg 1002
  jsonDoc["evse_pp_limit"] = evseAmpsPP;               //Reg 1003
  jsonDoc["evse_turn_off"] = evseTurnOff;              //Reg 1004
  jsonDoc["evse_firmware"] = evseFirmware;             //Reg 1005
  jsonDoc["evse_state"] = evseEvseState;               //Reg 1006
  jsonDoc["evse_rcd"] = 0;                             //Reg 1007
  jsonDoc["evse_amps_afterboot"] = addEvseData.evseAmpsAfterboot;  //Reg 2000
  jsonDoc["evse_modbus_enabled"] = addEvseData.evseModbusEnabled;  //Reg 2001
  jsonDoc["evse_amps_min"] = addEvseData.evseAmpsMin;              //Reg 2002
  jsonDoc["evse_analog_input"] = addEvseData.evseAnIn;             //Reg 2003
  jsonDoc["evse_amps_poweron"] = addEvseData.evseAmpsPowerOn;      //Reg 2004
  jsonDoc["evse_2005"] = addEvseData.evseReg2005;                  //Reg 2005
  jsonDoc["evse_sharing_mode"] = addEvseData.evseShareMode;        //Reg 2006
  jsonDoc["evse_pp_detection"] = addEvseData.evsePpDetection;      //Reg 2007
  
  //serializeJsonPretty(jsonDoc, Serial);  //Debugging
  size_t len = measureJson(jsonDoc);
  AsyncWebSocketMessageBuffer * buffer = ws.makeBuffer(len); //  creates a buffer (len + 1) for you.
  if (buffer) {
    serializeJson(jsonDoc, (char *)buffer->get(), len + 1);
    ws.textAll(buffer);
  }
}

// Send Scanned SSIDs to websocket clients as JSON object
void ICACHE_FLASH_ATTR printScanResult(int networksFound) {
  DynamicJsonDocument jsonDoc(3500);
  jsonDoc["command"] = "ssidlist";
  JsonArray jsonScanArray = jsonDoc.createNestedArray("list");
  for (int i = 0; i < networksFound; ++i) {
    JsonObject item = jsonScanArray.createNestedObject();
    // Print SSID for each network found
    item["ssid"] = WiFi.SSID(i);
    item["bssid"] = WiFi.BSSIDstr(i);
    item["rssi"] = WiFi.RSSI(i);
    item["channel"] = WiFi.channel(i);
    item["enctype"] = WiFi.encryptionType(i);
    #ifdef ESP8266
    item["hidden"] = WiFi.isHidden(i) ? true : false;
    #endif
  }
  size_t len = measureJson(jsonDoc);
  serializeJson(jsonDoc, Serial);
  AsyncWebSocketMessageBuffer * buffer = ws.makeBuffer(len);
  if (buffer) {
    serializeJson(jsonDoc, (char *)buffer->get(), len + 1);
    ws.textAll(buffer);
  }
  WiFi.scanDelete();
}

//////////////////////////////////////////////////////////////////////////////////////////
///////       Log Functions
//////////////////////////////////////////////////////////////////////////////////////////
void ICACHE_FLASH_ATTR logLatest(String uid, String username) {
  if (!config.getSystemLogging()) {
    return;
  }
  fsWorking = true;
  delay(30);
  File logFile = SPIFFS.open("/latestlog.json", "r");
  if (!logFile) {
    // Can not open file create it.
    File logFile = SPIFFS.open("/latestlog.json", "w");
    StaticJsonDocument<35> jsonDoc;
    jsonDoc["type"] = "latestlog";
    jsonDoc.createNestedArray("list");
    deserializeJson(jsonDoc, logFile);
    logFile.close();
    logFile = SPIFFS.open("/latestlog.json", "w+");
  }
  if (logFile) {
    size_t size = logFile.size();
    std::unique_ptr<char[]> buf (new char[size]);
    logFile.readBytes(buf.get(), size);
    DynamicJsonDocument jsonDoc2(8704);
    DeserializationError error = deserializeJson(jsonDoc2, buf.get());
    JsonArray list = jsonDoc2["list"];
    if (error) {
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Impossible to read log file");
    }
    else {
      logFile.close();
      if (list.size() >= 50) {
        list.remove(0);
      }
      logFile = SPIFFS.open("/latestlog.json", "w");
      StaticJsonDocument<192> jsonDoc3;
      if (config.getEvseRemote(0)){
        jsonDoc3["uid"] = "remote";
        jsonDoc3["username"] = "remote";
      }
      else {
        jsonDoc3["uid"] = uid;
        jsonDoc3["username"] = username;
      }
      jsonDoc3["timestamp"] = ntp.getUtcTimeNow();
      jsonDoc3["duration"] = 0;
      jsonDoc3["energy"] = 0;
      jsonDoc3["rEnd"] = 0;
      list.add(jsonDoc3);


      serializeJson(jsonDoc2, logFile);
      /* 
      size_t logfileSize;
      String jsonSizeCalc = "";
      serializeJson(jsonDoc2, jsonSizeCalc);
      size_t jsonSize = serializeJson(jsonDoc2, jsonSizeCalc);
      
      for (int i = 0; i < 2; i++) {
        logfileSize = serializeJson(jsonDoc2, logFile);
        if (logfileSize == jsonSizeCalc.length()) {
          if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ LOG ] LogFile verified!");
          Serial.print("logfileSize: ");
          Serial.println(logfileSize);
          break;
        }
        else {
          slog.logln(ntp.iso8601DateTime() + "[ LOG ] Error while writing LogFile... Trying 3 times");
          delay(50);
        }
      } */
    }
    logFile.close();
  }
  else {
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Cannot create Logfile");
  }
  //SPIFFS.end();
  delay(100);
  fsWorking = false;
}

void ICACHE_FLASH_ATTR readLogAtStartup() {
  fsWorking = true;
  delay(30);
  File logFile = SPIFFS.open("/latestlog.json", "r");
  size_t size = logFile.size();
  std::unique_ptr<char[]> buf (new char[size]);
  logFile.readBytes(buf.get(), size);
  DynamicJsonDocument jsonDoc(6500);
  DeserializationError error = deserializeJson(jsonDoc, buf.get());
  JsonArray list = jsonDoc["list"];
  if (error) {
    logFile.close();
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Impossible to read log file");
    slog.log(ntp.iso8601DateTime() + "[System] Impossible to parse Log file: ");
    slog.logln(error.c_str());
  }
  else {
    logFile.close();
    lastUID = list[(list.size()-1)]["uid"].as<String>();
    lastUsername = list[(list.size()-1)]["username"].as<String>();
    startChargingTimestamp = list[(list.size()-1)]["timestamp"];
    vehicleCharging = true;
    fsWorking = false;
  }
  return;
}

void ICACHE_FLASH_ATTR updateLog(bool incomplete) {
  if (!config.getSystemLogging()) {
    return;
  }
  if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ LOG ] Update Logfile...");
  fsWorking = true;
  delay(30);
  File logFile = SPIFFS.open("/latestlog.json", "r");
  size_t size = logFile.size();
  std::unique_ptr<char[]> buf (new char[size]);
  logFile.readBytes(buf.get(), size);
  DynamicJsonDocument jsonDoc(6500);
  DeserializationError error = deserializeJson(jsonDoc, buf.get());
  JsonArray list = jsonDoc["list"];
  if (error) {
    logFile.close();
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Impossible to update log file");
    slog.log(ntp.iso8601DateTime() + "[System] Impossible to parse Log file: ");
    slog.logln(error.c_str());
  }
  else {
    logFile.close();
    const char* uid = list[(list.size()-1)]["uid"];
    const char* username = list[(list.size()-1)]["username"];
    long timestamp = (long)list[(list.size()-1)]["timestamp"];
    list.remove(list.size() - 1); // delete newest log
   
    StaticJsonDocument<270> jsonDoc2;
    jsonDoc2["uid"] = uid;
    jsonDoc2["username"] = username;
    jsonDoc2["timestamp"] = timestamp;
    if (!incomplete) {
      jsonDoc2["duration"] = getChargingTime();
      if (config.getWifiWmode() && getChargingTime() > 360000000) jsonDoc2["duration"] = String("e");
    }
    else {
      jsonDoc2["duration"] = String("e");
      jsonDoc2["energy"] = String("e");
      jsonDoc2["price"] = String("e");
    }
    list.add(jsonDoc2);
    logFile = SPIFFS.open("/latestlog.json", "w");
    if (logFile) {
      String jsonSizeCalc = "";
      serializeJson(jsonDoc, jsonSizeCalc);
      size_t logfileSize;

      for (int i = 0; i < 2; i++) {
        logfileSize = serializeJson(jsonDoc, logFile);
        if (logfileSize == jsonSizeCalc.length()) {
          if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ LOG ] LogFile verified!");
          break;
        }
        else {
          slog.logln(ntp.iso8601DateTime() + "[ LOG ] Error while writing LogFile... Trying 3 times");
          delay(50);
        }
      }
    }
  }
  delay(100);
  fsWorking = false;
}

bool ICACHE_FLASH_ATTR initLogFile() {
  bool ret = true;
  fsWorking = true;
  if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Going to delete Log File...");
  File logFile = SPIFFS.open("/latestlog.json", "w");
  if (logFile) {
    StaticJsonDocument<35> jsonDoc;
    jsonDoc["type"] = "latestlog";
    jsonDoc.createNestedArray("list");
    serializeJson(jsonDoc, logFile);
    logFile.close();
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] ... Success!");
    ret = true;
  }
  else {
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] ... Failure!");
    ret = false;
  }
  delay(100);
  fsWorking = false;
  return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////       EVSE Modbus functions
//////////////////////////////////////////////////////////////////////////////////////////
bool ICACHE_FLASH_ATTR updateEvseData() {
  if (currentMillis > (lastModbusAction + config.getEvseUpdateInterval(0))) {
    if (config.getEvseReg1000(0) && config.getEvseReg2000(0)) {
      if (!updateHelper1000) {
        updateHelper1000 = true;
        return queryEVSE(false);
      }
      else {
        updateHelper1000 = false;
        return getAdditionalEVSEData();
      }
    }
    else if (config.getEvseReg1000(0)) {
      return queryEVSE(false);
    }
    else if (config.getEvseReg2000(0)) {
      return getAdditionalEVSEData();
    }
  }
  return true;
}

bool ICACHE_FLASH_ATTR queryEVSE(bool startup = false) {
  if (noEVSE) {
    slog.logln(ntp.iso8601DateTime() + "[ ERR ] No EVSE detected!");
    lastModbusAction = millis();
    return false;
  }
  uint8_t result;
  evseNode.clearTransmitBuffer();
  evseNode.clearResponseBuffer();
  result = evseNode.readHoldingRegisters(0x03E8, 7);  // read 7 registers starting at 0x03E8 (1000)

  if (config.getEvseLedConfig(0) != 1) changeLedTimes(100, 10000);

  if (result != 0) {
    if (evseErrorCount > 2) {
      evseVehicleState = 0;
      evseStatus = 0;
      if (config.getEvseLedConfig(0) == 3) changeLedTimes(300, 300);
    }
    evseErrorCount ++;
    mbErrCount ++;
    slog.log(ntp.iso8601DateTime() + "[ModBus] Error ");
    slog.log(result, true);
    slog.logln(" occured while getting EVSE Register 1000+");
    evseNode.clearTransmitBuffer();
    evseNode.clearResponseBuffer();
    lastModbusAction = millis();
    return false;
  }
  evseErrorCount = 0;
  mbReadCount ++;
  // register successfully read
  /*
  if(config.getSystemDebug()) slog.log("[ModBus] got EVSE Register 1000+ successfully - Last action ");
  if(config.getSystemDebug()) slog.log((millis() - lastModbusAction));
  if(config.getSystemDebug()) slog.log(" ms ago - MB errors total: ");
  if(config.getSystemDebug()) slog.log(mbErrCount);
  if(config.getSystemDebug()) slog.log("/");
  if(config.getSystemDebug()) slog.logln(mbReadCount); */
  lastModbusAction = millis();

  // register successfully read
  // process answer
  for (int i = 0; i < 7; i++) {
    switch(i) {
    case 0:
      evseAmpsConfig = evseNode.getResponseBuffer(i);     //Register 1000
      break;
    case 1:
      evseAmpsOutput = evseNode.getResponseBuffer(i);     //Register 1001
      break;
    case 2:
      evseVehicleState = evseNode.getResponseBuffer(i);   //Register 1002
      break;
    case 3:
      evseAmpsPP = evseNode.getResponseBuffer(i);          //Register 1003
      break;
    case 4:
      evseTurnOff = evseNode.getResponseBuffer(i);          //Register 1004
      break;
    case 5:
      evseFirmware = evseNode.getResponseBuffer(i);        //Register 1005
      break;
    case 6:
      evseEvseState = evseNode.getResponseBuffer(i);      //Register 1006
      break;
    }
  }

  // Globals
  if (evseFirmware > 17) {
    if (reg2005DefaultValues != 672) {
      reg2005DefaultValues = 32; // bit5 - clear RCD Error after < 30s
      reg2005DefaultValues += 128; // bit7 - high resolution
      reg2005DefaultValues += 512; // bit9 - pilot auto recover delay
    }
    if (!highResolution) highResolution = true;
  }
  else {
    if (reg2005DefaultValues != 32) {
      reg2005DefaultValues = 32; // bit5 - clear RCD Error after < 30s
    }
    if (highResolution) highResolution = false;
  }

  // Normal Mode
  if (!config.getEvseAlwaysActive(0) && !timerActive) {
    if (evseVehicleState == 0) {
      evseStatus = 0; //modbus communication failed
      if (config.getEvseLedConfig(0) == 3) changeLedTimes(300, 300);
    }
    if (evseEvseState == 3) {     //EVSE not Ready
      if (evseVehicleState == 2 ||
          evseVehicleState == 3 ||
          evseVehicleState == 4) {
        evseStatus = 2; //vehicle detected
        if (config.getEvseLedConfig(0) == 3) changeLedTimes(300, 2000);
      }
      else {
        evseStatus = 1; // EVSE deactivated
      }
      if (vehicleCharging == true && manualStop == false) {   //vehicle interrupted charging
        stopChargingTimestamp = ntp.getUtcTimeNow();
        vehicleCharging = false;
        if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Vehicle interrupted charging");
        updateLog(false);
      }
      evseActive = false;
      return true;
    }

    if (evseVehicleState == 1) {
      evseStatus = 1;  // ready
    }
    else if (evseVehicleState == 2) {
      evseStatus = 2; //vehicle detected
      if (config.getEvseLedConfig(0) == 3) changeLedTimes(300, 2000);
    }
    else if (evseVehicleState == 3 || evseVehicleState == 4) {
      evseStatus = 3; //charging
      if (startup) evseActive = true;
      if (config.getEvseLedConfig(0) == 3) changeLedTimes(2000, 1000);
    }
    else if (evseVehicleState == 5) {
      evseStatus = 5;
    }
  }
  
  // Always Active Mode
  else {
    if (evseVehicleState == 5) {   //ERROR
      evseStatus = 5;
    }
    if (evseEvseState == 1) { // Steady 12V
      if (vehicleCharging) { // EV interrupted charging
        stopChargingTimestamp = ntp.getUtcTimeNow();
        vehicleCharging = false;
        toDeactivateEVSE = true;
        lastUID = "vehicle";
        lastUsername = "vehicle";
        if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Vehicle interrupted charging");
      }
      evseStatus = 1; // ready
      evseActive = true;
    }
    else if (evseEvseState == 2) { // PWM is being generated
      if (evseVehicleState == 2) { // EV is present
        if (vehicleCharging) {  // EV interrupted charging
          stopChargingTimestamp = ntp.getUtcTimeNow();
          vehicleCharging = false;
          toDeactivateEVSE = true;
          lastUID = "vehicle";
          lastUsername = "vehicle";
          if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Vehicle interrupted charging");
        }
        evseStatus = 2; //vehicle detected
        evseActive = true;
        if (config.getEvseLedConfig(0) == 3) changeLedTimes(300, 2000);
      }
      else if (evseVehicleState == 3 || evseVehicleState == 4) {  // EV is charging
        if (!vehicleCharging) { // EV starts charging
          if (!startup) {
            toActivateEVSE = true;
          }
          startChargingTimestamp = ntp.getUtcTimeNow();
          vehicleCharging = true;
          lastUID = "vehicle";
          lastUsername = "vehicle";
          if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Vehicle started charging");
        }
        evseStatus = 3; //charging
        evseActive = true;
        if (config.getEvseLedConfig(0) == 3) changeLedTimes(2000, 1000);
      }
    }
    else if (evseEvseState == 3) {     //EVSE not Ready
      if (evseVehicleState == 2 ||
          evseVehicleState == 3 ||
          evseVehicleState == 4) {
        evseStatus = 2; //vehicle detected
        if (config.getEvseLedConfig(0) == 3) changeLedTimes(300, 2000);
        if (vehicleCharging && evseAmpsConfig == 0) { //Current Set to 0 - deactivate
          //millisStopCharging = millis();
          stopChargingTimestamp = ntp.getUtcTimeNow();
          vehicleCharging = false;
          toDeactivateEVSE = true;
          lastUID = "API";
          lastUsername = "API";
          if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] API interrupted charging");
        }
      }
      else {
        evseStatus = 1; // EVSE deactivated
      }
      if (vehicleCharging == true && manualStop == false) {   //vehicle interrupted charging
        //millisStopCharging = millis();
        stopChargingTimestamp = ntp.getUtcTimeNow();
        vehicleCharging = false;
        lastUID = "vehicle";
        lastUsername = "vehicle";
        if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Vehicle interrupted charging");
        updateLog(false);
      }
      evseActive = false;
      return true;
    }
  }
  return true;
}

bool ICACHE_FLASH_ATTR getAdditionalEVSEData() {
  if (noEVSE) {
    slog.logln(ntp.iso8601DateTime() + "[ ERR ] No EVSE detected!");
    return false;
  }
  // Getting additional Modbus data
  evseNode.clearTransmitBuffer();
  evseNode.clearResponseBuffer();
  uint8_t result = evseNode.readHoldingRegisters(0x07D0, 10);  // read 10 registers starting at 0x07D0 (2000)
  
  if (result != 0) {
    if (evseErrorCount > 2) {
      evseVehicleState = 0;
      evseStatus = 0;
      if (config.getEvseLedConfig(0) == 3) changeLedTimes(300, 300);
    }
    // error occured
    evseErrorCount ++;
    mbErrCount ++;
    evseVehicleState = 0;
    slog.log(ntp.iso8601DateTime() + "[ModBus] Error ");
    slog.log(result, true);
    slog.logln(" occured while getting EVSE Register 2000+");
    evseNode.clearTransmitBuffer();
    evseNode.clearResponseBuffer();
    if (config.getEvseLedConfig(0) == 3) changeLedTimes(300, 300);
    lastModbusAction = millis();
    return false;
  }
    evseErrorCount = 0;
    mbReadCount ++;
    // register successfully read
    /*
    if(config.getSystemDebug()) slog.log("[ModBus] got EVSE Register 2000+ successfully - Last action ");
    if(config.getSystemDebug()) slog.log((millis() - lastModbusAction));
    if(config.getSystemDebug()) slog.log(" ms ago - MB errors total: ");
    if(config.getSystemDebug()) slog.log(mbErrCount);
    if(config.getSystemDebug()) slog.log("/");
    if(config.getSystemDebug()) slog.logln(mbReadCount); */
    lastModbusAction = millis();

    //process answer
    for (int i = 0; i < 10; i++) {
      switch(i) {
      case 0:
        addEvseData.evseAmpsAfterboot  = evseNode.getResponseBuffer(i);    //Register 2000
        config.setEvseCurrentAfterBoot(0, addEvseData.evseAmpsAfterboot);
        break;
      case 1:
        addEvseData.evseModbusEnabled = evseNode.getResponseBuffer(i);     //Register 2001
        break;
      case 2:
        addEvseData.evseAmpsMin = evseNode.getResponseBuffer(i);           //Register 2002
        break;
      case 3:
        addEvseData.evseAnIn = evseNode.getResponseBuffer(i);             //Reg 2003
        break;
      case 4:
        addEvseData.evseAmpsPowerOn = evseNode.getResponseBuffer(i);      //Reg 2004
        break;
      case 5:
        addEvseData.evseReg2005 = evseNode.getResponseBuffer(i);          //Reg 2005
        break;
      case 6:
        addEvseData.evseShareMode = evseNode.getResponseBuffer(i);        //Reg 2006
        break;
      case 7:
        addEvseData.evsePpDetection = evseNode.getResponseBuffer(i);       //Register 2007
        break;
      case 9:
        addEvseData.evseBootFirmware = evseNode.getResponseBuffer(i);       //Register 2009
        break;
      }
    }
  return true;
}

bool ICACHE_FLASH_ATTR activateEVSE() {
  if (noEVSE) {
    slog.logln(ntp.iso8601DateTime() + "[ ERR ] No EVSE detected!");
    return false;
  }
  if (!config.getEvseAlwaysActive(0) && !timerActive) {   //Normal Mode
    static uint16_t iTransmit;
    if (evseEvseState == 3 &&
      evseVehicleState != 0) {    //no modbus error occured
      iTransmit = reg2005DefaultValues + 8192;         // disable EVSE after charge

      uint8_t result;
      evseNode.clearTransmitBuffer();
      evseNode.setTransmitBuffer(0, iTransmit); // set word 0 of TX buffer (bits 15..0)
      result = evseNode.writeMultipleRegisters(0x07D5, 1);  // write register 0x07D5 (2005)

      if (result != 0) {
        // error occured
        slog.log(ntp.iso8601DateTime() + "[ModBus] Error ");
        slog.log(result, true);
        slog.logln(" occured while activating EVSE - trying again...");
        if (config.getEvseLedConfig(0) == 3) changeLedTimes(300, 300);
        delay(500);
        return false;
      }

      //millisStartCharging = millis();
      startChargingTimestamp = ntp.getUtcTimeNow();
      manualStop = false;
      // register successfully written
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ModBus] EVSE successfully activated");
    }
  }
  else {
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ModBus] EVSE already active");
  }

  
  toActivateEVSE = false;
  evseActive = true;
  logLatest(lastUID, lastUsername);
  vehicleCharging = true;

  sendEVSEdata();
  return true;
}

bool ICACHE_FLASH_ATTR deactivateEVSE(bool logUpdate) {
  if (noEVSE) {
    slog.logln(ntp.iso8601DateTime() + "[ ERR ] No EVSE detected!");
    return false;
  }
  if (!config.getEvseAlwaysActive(0) && !timerActive) {   //Normal Mode
    //New ModBus Master Library
    static uint16_t iTransmit = reg2005DefaultValues + 16384;  // deactivate evse

    uint8_t result;

    evseNode.clearTransmitBuffer();
    evseNode.setTransmitBuffer(0, iTransmit); // set word 0 of TX buffer (bits 15..0)
    result = evseNode.writeMultipleRegisters(0x07D5, 1);  // write register 0x07D5 (2005)

    if (result != 0) {
      // error occured
      slog.log(ntp.iso8601DateTime() + "[ModBus] Error ");
      slog.log(result, true);
      slog.logln(" occured while deactivating EVSE - trying again...");
      if (config.getEvseLedConfig(0) == 3) changeLedTimes(300, 300);
      delay(500);
      return false;
    }

    // register successfully written
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ModBus] EVSE successfully deactivated");
    evseActive = false;
    stopChargingTimestamp = ntp.getUtcTimeNow();
  }
  manualStop = true;
  if (logUpdate) {
    updateLog(false);
  }
  vehicleCharging = false;
  toDeactivateEVSE = false;
  
  if (config.getEvseResetCurrentAfterCharge(0) == true) {
    currentToSet = config.getEvseCurrentAfterBoot(0);
    toSetEVSEcurrent = true;
  }
  sendEVSEdata();

  if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] EVSE successfully deactivated");
  return true;
}

bool ICACHE_FLASH_ATTR setEVSEcurrent() {  // telegram 1: write EVSE current
  //New ModBus Master Library

  if (highResolution) {
    if (currentToSet < 64) { // Current given in low resolution
      currentToSet = currentToSet * 100;
    }
  }
  else {
    if (currentToSet > 64) { // Current given in high resolution
      currentToSet = currentToSet / 100;
    }
  }

  if (config.getEvseRemote(0)) {
    toSetEVSEcurrent = false;
    if (currentToSet == evseAmpsConfig) return true; // no oLED action
  }
  else {
    toSetEVSEcurrent = false;
    if (currentToSet == evseAmpsConfig) return true; // oLED action but register is already set
  }

  if (setEVSERegister(1000,currentToSet) == false) {
    // error occured
    //slog.logln(ntp.iso8601DateTime() + "[ ModBus ] Error occured while setting current in EVSE - trying again in a few seconds...");
    return false;
  }
  else {
    // register successfully written
    //if(config.getSystemDebug()) slog.log(ntp.iso8601DateTime() + "[ ModBus ] Register 1000 successfully set to ");
    //if(config.getSystemDebug()) slog.logln(currentToSet);
    evseAmpsConfig = currentToSet;  //foce update in WebUI
    sendEVSEdata();               //foce update in WebUI
    toSetEVSEcurrent = false;
  }
  return true;
}


bool ICACHE_FLASH_ATTR setEVSERegister(uint16_t reg, uint16_t val) {
  if (noEVSE) {
    slog.logln(ntp.iso8601DateTime() + "[ ERR ] No EVSE detected!");
    return false;
  }



  for (int i = 0; i < 5; i++) {
    uint8_t result;
    evseNode.clearTransmitBuffer();
    evseNode.setTransmitBuffer(0, val); // set word 0 of TX buffer (bits 15..0)
    result = evseNode.writeMultipleRegisters(reg, 1);  // write given register

    if (result != 0) {
      // error occured
      slog.log(ntp.iso8601DateTime() + "[ModBus] Error ");
      slog.log(result, true);
      slog.logln(" occured while setting EVSE Register " + (String)reg + " to " + (String)val);
      if (config.getEvseLedConfig(0) == 3) changeLedTimes(300, 300);
      delay((i+1) * 50);
    }
    else {
      // register successfully written
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ModBus] Register " + (String)reg + " successfully set to " + (String)val);
      millisUpdateEvse = millis() + 100;
      return true;
    }
  }
  millisUpdateEvse = millis() + 5000;
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////       Websocket Functions
//////////////////////////////////////////////////////////////////////////////////////////
void ICACHE_FLASH_ATTR sendEVSEdata() {
    StaticJsonDocument<520> jsonDoc;
    jsonDoc["command"] = "getevsedata";
    jsonDoc["evse_vehicle_state"] = evseStatus;
    jsonDoc["evse_active"] = evseActive;
    if (evseAmpsConfig > 64) {
      jsonDoc["evse_current_limit"] = evseAmpsConfig / 100.0;
    }
    else {
      jsonDoc["evse_current_limit"] = evseAmpsConfig;
    }
    jsonDoc["evse_slider_status"] = sliderStatus;
    jsonDoc["evse_rse_value"] = config.getEvseRseValue(0);
    if (getChargingTime() > 360000000) {
      jsonDoc["evse_charging_time"] = 0;
    }
    else {
      jsonDoc["evse_charging_time"] = getChargingTime();
    }
    jsonDoc["evse_always_active"] = config.getEvseAlwaysActive(0);
    jsonDoc["evse_remote_controlled"] = config.getEvseRemote(0);
    jsonDoc["evse_timer_active"] = timerActive;
    jsonDoc["ap_mode"] = inAPMode;
    jsonDoc["evse_disabled_by_remote_hearbeat"] = deactivatedByRemoteHeartbeat;
    size_t len = measureJson(jsonDoc);
    AsyncWebSocketMessageBuffer * buffer = ws.makeBuffer(len);
    if (buffer) {
      serializeJson(jsonDoc, (char *)buffer->get(), len + 1);
      ws.textAll(buffer);
    }
}

void ICACHE_FLASH_ATTR sendTime() {
  StaticJsonDocument<100> jsonDoc;
  jsonDoc["command"] = "gettime";
  jsonDoc["epoch"] = now();
  jsonDoc["timezone"] = config.getNtpTimezone();
  size_t len = measureJson(jsonDoc);
  AsyncWebSocketMessageBuffer * buffer = ws.makeBuffer(len);
  if (buffer) {
    serializeJson(jsonDoc, (char *)buffer->get(), len + 1);
    ws.textAll(buffer);
  }
}

void ICACHE_FLASH_ATTR sendStartupInfo(AsyncWebSocketClient * client) {
  uint8_t opmode;
  if (config.getEvseAlwaysActive(0)) {
    opmode = 1;
  }
  else if (config.getEvseRemote(0)) {
    opmode = 2;
  }
  else { // Normal Mode
    opmode = 0;
  }
  String hwrev = "";
  String message = "";

  #ifdef ESP8266
  hwrev = "ESP8266";
  //String message = "{\"command\":\"startupinfo\",\"hw_rev\":\"ESP8266\",\"sw_rev\":\"" + swVersion + "\",\"pp_limit\":\"" +  + "\",\"language\":\"" + config.getSystemLanguage() + "\",\"opmode\":" + opmode + ",\"highResolution\":\"" + highResolution + "\"}"; 
  #endif

  StaticJsonDocument<500> jsonDoc;
  jsonDoc["command"] = "startupinfo";
  jsonDoc["hw_rev"] = hwrev;
  jsonDoc["sw_rev"] = swVersion;
  jsonDoc["pp_limit"] = (String)evseAmpsPP;
  jsonDoc["language"] = config.getSystemLanguage();
  jsonDoc["opmode"] = opmode;
  jsonDoc["highResolution"] = highResolution;
  jsonDoc["debug"] = config.getSystemDebug();

  serializeJson(jsonDoc, message);
  client->text(message);
}

void ICACHE_FLASH_ATTR sendEvseTimer(AsyncWebSocketClient * client) {
  File timerFile = SPIFFS.open("/timer.json", "r");
  String file;

  if (!timerFile) {
    return;
  }
  while(timerFile.available()) {
    file+= char(timerFile.read());
  }
  timerFile.close();
  client->text(file);
}

void ICACHE_FLASH_ATTR onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_ERROR) {
    String errText = "[ WARN ] WebSocket error [" + String(server->url()) + "] [" + String(client->id()) + "]";
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + errText);
    //if (config.getSystemDebug()) Serial.printf("[ WARN ] WebSocket[%s][%u] error(%u): %s\r\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);    
  }
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      //the whole message is in a single frame and we got all of it's data
      for (size_t i = 0; i < info->len; i++) {
        msg += (char) data[i];
      }
      StaticJsonDocument<1800> jsonDoc;
      DeserializationError error = deserializeJson(jsonDoc, msg);
      if (error) {
        if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ WARN ] Couldn't parse WebSocket message");
        msg = "";
        return;
      }
      processWsEvent(jsonDoc, client);
    }
    else {
      //message is comprised of multiple frames or the frame is split into multiple packets
      if(config.getSystemDebug()) slog.log(ntp.iso8601DateTime() + "[ Websocket ] more than one Frame!");
      for (size_t i = 0; i < len; i++) {
        msg += (char) data[i];
      }
      if (info->final && (info->index + len) == info->len) {
        StaticJsonDocument<1800> jsonDoc;
        DeserializationError error = deserializeJson(jsonDoc, msg);
        if (error) {
          if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ WARN ] Couldn't parse WebSocket message");
          msg = "";
          return;
        }
        processWsEvent(jsonDoc, client);
      }
    }
  }
}

void ICACHE_FLASH_ATTR sendSyslogToWs() {
  DynamicJsonDocument doc(15000);
  JsonArray array = doc.createNestedArray("syslog_export");
  size_t size = syslogDeque.size();
  for (size_t i = 0; i < size; i++) {
    array.add(syslogDeque.at(i));
  }
  size_t len = measureJson(doc);
  AsyncWebSocketMessageBuffer * buffer = ws.makeBuffer(len);
  if (buffer) {
    serializeJson(doc, (char *)buffer->get(), len + 1);
    ws.textAll(buffer);
  }

  toSendSyslogToWs = false;
}

void ICACHE_FLASH_ATTR processWsEvent(JsonDocument& root, AsyncWebSocketClient * client) {
  const char * command = root["command"];
  if (strcmp(command, "remove") == 0) {
    const char* uid = root["uid"];
    String filename = "/P/";
    filename += uid;
    fsWorking = true;
    SPIFFS.remove(filename);
    delay(10);
    fsWorking = false;
  }
  else if (strcmp(command, "configfile") == 0) {
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Try to update config.json...");
    String configString;
    serializeJson(root, configString);

    if (config.checkUpdateConfig(configString, setEVSERegister) && config.updateConfig(configString)) {
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Success - going to reboot now");
      if (vehicleCharging) {
        deactivateEVSE(true);
        delay(100);
      }
      #ifdef ESP8266
      ESP.reset();
      #endif
    }
    else {
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Could not save config.json");
    }
  }
  else if (strcmp(command, "status") == 0) {
    toSendStatus = true;
  }
  else if (strcmp(command, "userfile") == 0) {
    const char* uid = root["uid"];
    String filename = "/P/";
    filename += uid;
    File userFile = SPIFFS.open(filename, "w+");
    if (userFile) {
      userFile.print(msg);
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Userfile written!");
    }
    userFile.close();
    ws.textAll("{\"command\":\"result\",\"resultof\":\"userfile\",\"result\": true}");
  }
  else if (strcmp(command, "latestlog") == 0) {
    if (!fsWorking) {
      fsWorking = true;
      File logFile = SPIFFS.open("/latestlog.json", "r");
      if (logFile) {
        size_t len = logFile.size();
        AsyncWebSocketMessageBuffer * buffer = ws.makeBuffer(len);
        if (buffer) {
          logFile.readBytes((char *)buffer->get(), len + 1);
          ws.textAll(buffer);
        }
        logFile.close();
      }
      else {
        slog.logln(ntp.iso8601DateTime() + "[System] Error while reading log file");
      }
      delay(10);
      fsWorking = false;
    }
    else {
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ ERROR ] File system is already working...");
    }
  }
  else if (strcmp(command, "scan") == 0) {
    #ifdef ESP8266
    WiFi.scanNetworksAsync(printScanResult, true);
    #else
    int networks = WiFi.scanNetworks();
    printScanResult(networks);
    #endif
  }
  else if (strcmp(command, "gettime") == 0) {
    sendTime();
  }
  else if (strcmp(command, "settime") == 0) {
    unsigned long t = root["epoch"];
    //t += (config.getNtpTimezone * 3600);
    setTime(t);
    sendTime();
  }
  else if (strcmp(command, "getconf") == 0) {
    ws.textAll(config.getConfigJson());
  }
  else if (strcmp(command, "getevsedata") == 0) {
    //evseQueryTimeOut = millis() + 10000; //Timeout for pushing data in loop
    sendEVSEdata();
    //evseSessionTimeOut = false;
  }
  else if (strcmp(command, "setcurrent") == 0) {
    currentToSet = root["current"];
    if(config.getSystemDebug()) slog.log(ntp.iso8601DateTime() + "[ WebSocket ] Call setEVSECurrent() ");
    if(config.getSystemDebug()) slog.logln(currentToSet);
    toSetEVSEcurrent = true;
  }
  else if (strcmp(command, "activateevse") == 0) {
    toActivateEVSE = true;
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ WebSocket ] Activate EVSE via WebSocket");
    lastUID = "GUI";
    lastUsername = "GUI";
  }
  else if (strcmp(command, "deactivateevse") == 0) {
    toDeactivateEVSE = true;
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ WebSocket ] Deactivate EVSE via WebSocket");
    lastUID = "GUI";
    lastUsername = "GUI";
  }
  else if (strcmp(command, "setevsereg") == 0) {
    uint16_t reg = atoi(root["register"]);
    uint16_t val = atoi(root["value"]);
    setEVSERegister(reg, val);
  }
  else if (strcmp(command, "factoryreset") == 0) {
    factoryReset();
  }
  else if (strcmp(command, "resetuserdata") == 0) {
    if (resetUserData()) {
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ WebSocket ] User Data Reset successfully done");
    }
  }
  else if (strcmp(command, "initlog") == 0) {
    if(config.getSystemDebug()) slog.log(ntp.iso8601DateTime() + "[System] Websocket Command \"initlog\"...");
    toDeactivateEVSE = true;
    toInitLog = true;
  }
  else if (strcmp(command, "getstartup") == 0) {
    sendStartupInfo(client);
  }
  else if (strcmp(command, "getevsetimer") == 0) {
    sendEvseTimer(client);
  }
  else if (strcmp(command, "timer") == 0) {
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Try to update timer.json...");
    File timerFile = SPIFFS.open("/timer.json", "w+");
    if (timerFile) {
      serializeJsonPretty(root, timerFile);
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ DEBUG ] Timer file written!");
    }
    timerFile.close();
    ws.textAll("{\"command\":\"result\",\"resultof\":\"userfile\",\"result\": true}");
  }
  else if (strcmp(command, "getsyslog") == 0) {
    toSendSyslogToWs = true;
  }

  msg = "";
}

//////////////////////////////////////////////////////////////////////////////////////////
///////       Setup Functions
//////////////////////////////////////////////////////////////////////////////////////////
bool ICACHE_FLASH_ATTR connectSTA(const char* ssid, const char* password) {
  delay(100);
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password, 0);
  slog.log(ntp.iso8601DateTime() + "[Info] Trying to connect WiFi: ");
  slog.log(ssid);

  unsigned long now = millis();
  uint8_t timeout = 20;  // seconds
  do {
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
    delay(500);
    slog.log(".");
  }
  while (millis() - now < timeout * 1000);
  slog.logln("");

  if (WiFi.status() == WL_CONNECTED) {
    isWifiConnected = true;
    return true;
  }
  slog.logln(ntp.iso8601DateTime() + "[ WARN ] Couldn't connect in time");
  return false;
}

bool ICACHE_FLASH_ATTR startAP(const char * ssid, const char * password = NULL) {
  inAPMode = true;
  
  #ifdef ESP8266
  WiFi.hostname(config.getSystemHostname());
  #else
  WiFi.setHostname(config.getSystemHostname());
  #endif

  WiFi.mode(WIFI_AP);
  slog.log(ntp.iso8601DateTime() + "[Info] Configuring access point... ");
  bool success = WiFi.softAP(ssid, password);
  if (success) {
    slog.logln("Ready");
  }
  else {
    slog.logln("Failed!");
  }
  // Access Point IP
  IPAddress myIP = WiFi.softAPIP();
  slog.log(ntp.iso8601DateTime() + "[Info] AP IP address: ");
  slog.logln(myIP.toString());
  slog.log(ntp.iso8601DateTime() + "[Info] AP SSID:");
  slog.logln(String(ssid));
  isWifiConnected = success;

  if (!MDNS.begin(config.getSystemHostname())) {
    slog.logln(ntp.iso8601DateTime() + "[System] Error setting up MDNS responder!");
  }

  return success;
}

bool ICACHE_FLASH_ATTR loadConfiguration(String configString = "") {
  slog.logln(ntp.iso8601DateTime() + "[System] Loading Config File on Startup...");
  if (configString == "") {
    if (!config.loadConfig()) return false;
  }
  else {
    if (!config.loadConfig(configString)) return false;
  }
  config.loadConfiguration();
  
  slog.begin(&ws, config.getSystemDebug(), &syslogDeque);

  if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Check for old config version and renew it");
  config.renewConfigFile();

  delay(300); // wait a few milliseconds to prevent voltage drop...

#ifdef ESP8266
  SecondSer.begin(9600);
  evseNode.begin(1, SecondSer);
  #else

  FirstSer.begin(9600, SERIAL_8N1, 22, 21);   //EVSE
  SecondSer.begin(9600, SERIAL_8N1, 34, 14);  //SDM

  //Check connected devices
  //EVSE:
  if (checkUart(&FirstSer, 1)) { //EVSE -> UART1
    evseNode.begin(1, FirstSer);
    slog.logln(ntp.iso8601DateTime() + "[ Modbus ] EVSE detected at UART 1");
  }
  else if (checkUart(&SecondSer, 1)) { //EVSE -> UART2
    evseNode.begin(1, SecondSer);
    slog.logln(ntp.iso8601DateTime() + "[ Modbus ] EVSE detected at UART 2");
  }
  else {
    noEVSE = true;
  }

  //SDM:
  #endif

  if (config.getSystemDebug()) {
    slog.logln(ntp.iso8601DateTime() + "[System] Debug Mode: ON!");
  }
  else {
    slog.logln(ntp.iso8601DateTime() + "[System] Debug Mode: OFF!");
  }

  if (!config.getSystemWsauth()) {
    ws.setAuthentication("admin", config.getSystemPass());
    if(config.getSystemDebug()) slog.log(ntp.iso8601DateTime() + "[ Websocket ] Use Basic Authentication for Websocket");
  }
  #ifdef ESP8266
  server.addHandler(new SPIFFSEditor("admin", config.getSystemPass()));
  #else
  server.addHandler(new SPIFFSEditor(SPIFFS, "admin", config.getSystemPass()));
  #endif


  queryEVSE(true);
  while (evseErrorCount != 0) {
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ Modbus ] Error getting EVSE data!");
    delay(500);
    queryEVSE(true);
    if (evseErrorCount > 2) {
      break;
    }
  }
  delay(50);

  getAdditionalEVSEData();
  while (evseErrorCount != 0) {
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ Modbus ] Error getting additional EVSE data!");
    delay(500);
    getAdditionalEVSEData();
    if (evseErrorCount > 2) {
      break;
    }
  }
  delay(50);

  if (evseAmpsConfig == 0 && 
    !config.getEvseAlwaysActive(0) && !config.getEvseRemote(0)) {
      currentToSet = config.getEvseCurrentAfterBoot(0);
      toSetEVSEcurrent = true;
  }

  if (config.getEvseAlwaysActive(0)) {
    evseActive = true;
    if (addEvseData.evseReg2005 != reg2005DefaultValues) {
      delay(50);
      setEVSERegister(2005, reg2005DefaultValues);
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[Info] EVSE register 2005 set to 0 -> Always Active Mode");
    }
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[Info] EVSE-WiFi runs in always active mode");
  }

  if (config.getWifiWmode() == 1) {
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[Info] EVSE-WiFi is running in AP Mode ");
    WiFi.disconnect(true);
    return startAP(config.getWifiSsid(), config.getWifiPass());
  }

  WiFi.disconnect(true);

  if (config.getWifiStaticIp() == true) {
    IPAddress clientip;
    IPAddress subnet;
    IPAddress gateway;
    IPAddress dns;

    clientip.fromString(config.getWifiIp());
    subnet.fromString(config.getWiFiSubnet());
    gateway.fromString(config.getWiFiGateway());
    dns.fromString(config.getWiFiDns());

    WiFi.config(clientip, gateway, subnet, dns);
  }

  WiFi.hostname(config.getSystemHostname());

  if (!connectSTA(config.getWifiSsid(), config.getWifiPass())) {
    return false;
  }
  if (!MDNS.begin(config.getSystemHostname())) {
    slog.logln(ntp.iso8601DateTime() + "[System] Error setting up MDNS responder!");
  }

  //slog.logln("");
  slog.log(ntp.iso8601DateTime() + "[Info] Client IP address: ");
  slog.logln(WiFi.localIP().toString());

  //Check internet connection
  delay(100);
  if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ NTP ] NTP Server - set up NTP");
  const char * ntpserver = config.getNtpIp();
  IPAddress timeserverip;
  WiFi.hostByName(ntpserver, timeserverip);
  String ip = printIP(timeserverip);
  if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ NTP ] IP: " + ip);
  uint8_t tz = config.getNtpTimezone();
  if (config.getNtpDst()) {
    tz = tz + 1;
    if(config.getSystemDebug()) slog.log(ntp.iso8601DateTime() + "[ NTP ] Timezone: ");
    if(config.getSystemDebug()) slog.logln(tz);
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ NTP ] DST on");
  }
  else {
    if(config.getSystemDebug()) slog.log(ntp.iso8601DateTime() + "[ NTP ] Timezone: ");
    if(config.getSystemDebug()) slog.logln(tz);
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ NTP ] DST off");
  }
  ntp.Ntp(config.getNtpIp(), tz, 3600);   //use NTP Server, timeZone, update every x sec
  
  delay(1000);

  // Handle boot while charging is active
  return true;
}

void ICACHE_FLASH_ATTR setWebEvents() {
  server.on("/index.htm", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse * response = request->beginResponse_P(200, "text/html", WEBSRC_INDEX_HTM, WEBSRC_INDEX_HTM_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse * response = request->beginResponse_P(200, "text/javascript", WEBSRC_SCRIPT_JS, WEBSRC_SCRIPT_JS_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/lang.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse * response = request->beginResponse_P(200, "text/javascript", WEBSRC_LANG_JS, WEBSRC_LANG_JS_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/fonts/glyph.woff", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse * response = request->beginResponse_P(200, "font/woff", WEBSRC_GLYPH_WOFF, WEBSRC_GLYPH_WOFF_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });
 
  server.on("/fonts/glyph.woff2", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse * response = request->beginResponse_P(200, "font/woff", WEBSRC_GLYPH_WOFF2, WEBSRC_GLYPH_WOFF2_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/required/required.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse * response = request->beginResponse_P(200, "text/css", WEBSRC_REQUIRED_CSS, WEBSRC_REQUIRED_CSS_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/required/required.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse * response = request->beginResponse_P(200, "text/javascript", WEBSRC_REQUIRED_JS, WEBSRC_REQUIRED_JS_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/status_charging.svg", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse * response = request->beginResponse_P(200, "image/svg+xml", WEBSRC_STATUS_CHARGING_SVG, WEBSRC_STATUS_CHARGING_SVG_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/status_detected.svg", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse * response = request->beginResponse_P(200, "image/svg+xml", WEBSRC_STATUS_DETECTED_SVG, WEBSRC_STATUS_DETECTED_SVG_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/status_ready.svg", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse * response = request->beginResponse_P(200, "image/svg+xml", WEBSRC_STATUS_READY_SVG, WEBSRC_STATUS_READY_SVG_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse * response = request->beginResponse_P(200, "image/x-icon", WEBSRC_FAVICON_ICO, WEBSRC_FAVICON_ICO_LEN);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  //
  //  HTTP API
  //
  //getParameters
  if (config.getSystemApi()) {
    server.on("/getParameters", HTTP_GET, [](AsyncWebServerRequest * request) {
      AsyncResponseStream *response = request->beginResponseStream("application/json");
      StaticJsonDocument<500> jsonDoc;
      jsonDoc["type"] = "parameters";
      JsonArray list = jsonDoc.createNestedArray("list");
      JsonObject items = list.createNestedObject();
      items["vehicleState"] = evseStatus;
      items["evseState"] = evseActive;
      items["maxCurrent"] = config.getSystemMaxInstall();

      if (highResolution) {
        items["actualCurrent"] = evseAmpsConfig / 100;
        items["actualCurrentMA"] = evseAmpsConfig;
      }
      else {
        items["actualCurrent"] = evseAmpsConfig;
      }
      items["duration"] = getChargingTime();
      items["alwaysActive"] = config.getEvseAlwaysActive(0);
      items["lastActionUser"] = lastUsername;
      items["lastActionUID"] = lastUID;

      remoteHeartbeatCounter = 0; // Reset Heartbeat Counter

      serializeJson(jsonDoc, *response);
      request->send(response);
    });


    //getLog 
    server.on("/getLog", HTTP_GET, [](AsyncWebServerRequest * request) {
      File logFile = SPIFFS.open("/latestlog.json", "r");
      if (!logFile) {
        request->send(200, "text/plain", "E0_could not read log file - corrupted data or log file does not exist");
      }
      else{
        AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/latestlog.json", "application/json");
        request->send(response);
      }
      logFile.close();
    }); 

    //setCurrent (0,233)
    server.on("/setCurrent", HTTP_GET, [](AsyncWebServerRequest * request) {
        awp = request->getParam(0);
        if (awp->name() == "current") {
          if ((atoi(awp->value().c_str()) <= config.getSystemMaxInstall() && atoi(awp->value().c_str()) >= 6) || 
            (atoi(awp->value().c_str()) <= config.getSystemMaxInstall() * 100 && atoi(awp->value().c_str()) >= 600) || atoi(awp->value().c_str()) == 0) {
            currentToSet = atoi(awp->value().c_str());
            toSetEVSEcurrent = true;
            if (setEVSEcurrent()) {
              request->send(200, "text/plain", "S0_set current to given value");
            }
            else {
              request->send(200, "text/plain", "E0_could not set current - internal error");
            }
          }
          else {
            if (atoi(awp->value().c_str()) >= config.getSystemMaxInstall()) {
              currentToSet = config.getSystemMaxInstall();
              toSetEVSEcurrent = true;
              if (setEVSEcurrent()) {
                request->send(200, "text/plain", "S0_set current to maximum value");
              }
            }
            else {
              request->send(200, "text/plain", ("E1_could not set current - give a value between 6 and " + (String)config.getSystemMaxInstall()));
            }
          }
        }
        else {
          request->send(200, "text/plain", "E2_could not set current - wrong parameter");
        }
    });

    //setStatus
    server.on("/setStatus", HTTP_GET, [](AsyncWebServerRequest * request) {
      awp = request->getParam(0);
      if (awp->name() == "active" && config.getEvseAlwaysActive(0) == false && timerActive == false) {
        if (strcmp(awp->value().c_str(), "true") == 0) {
          lastUID = "API";
          lastUsername = "API";
          if (!evseActive) {
            if (activateEVSE()) {
              request->send(200, "text/plain", "S0_EVSE successfully activated");
            }
            else {
              request->send(200, "text/plain", "E0_could not activate EVSE - internal error!");
            }
          }
          else {
            request->send(200, "text/plain", "E3_could not activate EVSE - EVSE already activated!");
          }
        }
        else if (strcmp(awp->value().c_str(), "false") == 0) {
          lastUID = "API";
          lastUsername = "API";
          if (evseActive) {
            toDeactivateEVSE = true;
            request->send(200, "text/plain", "S0_EVSE successfully deactivated");
          }
          else { 
            request->send(200, "text/plain", "E3_could not deactivate EVSE - EVSE already deactivated!");
          }
        }
        else {
          request->send(200, "text/plain", "E1_could not process - give a valid value (true/false)");
        }
      }
      else {
        request->send(200, "text/plain", "E2_could not process - wrong parameter or EVSE-WiFi runs in always active mode");
      }
    });

    //doReboot
    server.on("/doReboot", HTTP_GET, [](AsyncWebServerRequest * request) {
      awp = request->getParam(0);
      if (awp->name() == "reboot") {
        if (strcmp(awp->value().c_str(), "true") == 0) {
          toReboot = true;
          request->send(200, "text/plain", "S0_EVSE-WiFi is going to reboot now...");
        }
        else {
          request->send(200, "text/plain", "E1_could not do reboot - wrong value");
        }
      }
      else {
        request->send(200, "text/plain", "E2_could not do reboot - wrong parameter");
      }
    });

    //setRegister (0,233)
    server.on("/setRegister", HTTP_GET, [](AsyncWebServerRequest * request) {
      awp = request->getParam(0);
      awp2 = request->getParam(1);
      if (awp->name() == "reg") {
        if ((atoi(awp->value().c_str()) >= 1000 && atoi(awp->value().c_str()) <= 1007) || 
            (atoi(awp->value().c_str()) >= 2000 && atoi(awp->value().c_str()) <= 2017)) {
          if (awp2->name() == "val") {
            if (atoi(awp2->value().c_str()) >= 0 && atoi(awp2->value().c_str()) <= 65535) {
              uint16_t reg = atoi(awp->value().c_str());
              uint16_t val = atoi(awp2->value().c_str());
              if (setEVSERegister(reg, val)){
                request->send(200, "text/plain", "S0_EVSE Register successfully set");
              }
              else {
                request->send(200, "text/plain", "E0_could not set EVSE register - internal error");
              }
            }
            else {
              request->send(200, "text/plain", "E0_could not set EVSE register - invalid value");
            }
          }
          else {
            request->send(200, "text/plain", "E1_could not set EVSE register - invalid parameter");
          }
        }
        else {
          request->send(200, "text/plain", "E0_could not set EVSE register - invalid register");
        }
      }
      else {
        request->send(200, "text/plain", "E1_could not set EVSE register - invalid parameter");
      }
    });

    //setConfig
    server.on("/setConfig", HTTP_GET, [](AsyncWebServerRequest * request) {
      awp = request->getParam(0);
      if (awp->name() == "config") {
          request->send(200, "text/plain", "E1_could not set registers and config - wrong value");
      }
      else {
        request->send(200, "text/plain", "E2_could not set registers and config - wrong parameter");
      }
    });
  }
    //evseHost
    server.on("/evseHost", HTTP_GET, [](AsyncWebServerRequest * request) {
      AsyncResponseStream *response = request->beginResponseStream("application/json");
      StaticJsonDocument<340> jsonDoc;
      jsonDoc["type"] = "evseHost";
      JsonArray list = jsonDoc.createNestedArray("list");
      JsonObject item = list.createNestedObject();

      #ifdef ESP8266
      struct ip_info info;
      if (inAPMode) {
        wifi_get_ip_info(SOFTAP_IF, &info);
        struct softap_config conf;
        wifi_softap_get_config(&conf);
        item["ssid"] = String(reinterpret_cast<char*>(conf.ssid));
        item["dns"] = printIP(WiFi.softAPIP());
        item["mac"] = WiFi.softAPmacAddress();
      }
      else {
        wifi_get_ip_info(STATION_IF, &info);
        struct station_config conf;
        wifi_station_get_config(&conf);
        item["ssid"] = String(reinterpret_cast<char*>(conf.ssid));
        item["rssi"] = String(WiFi.RSSI());
        item["dns"] = printIP(WiFi.dnsIP());
        item["mac"] = WiFi.macAddress();
      }
      #endif
      item["uptime"] = ntp.getUptimeSec();
      if (config.getEvseRemote(0)) {
        item["opMode"] = "remote";
      }
      else if (config.getEvseAlwaysActive(0)) {
        item["opMode"] = "alwaysActive";
      }
      else {
        item["opMode"] = "normal";
      }
      item["firmware"] = swVersion;
      serializeJson(jsonDoc, *response);
      request->send(response);
    });
}

void ICACHE_FLASH_ATTR fallbacktoAPMode() {
  if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[Info] EVSE-WiFi is running in Fallback AP Mode");
  WiFi.disconnect(true);
  if (startAP("EVSE-WiFi-Fallback")) {
    slog.logln(ntp.iso8601DateTime() + "[System] Fallback Mode set successfully!");
    inFallbackMode = true;
  }
  else {
    slog.logln(ntp.iso8601DateTime() + "[System] Fallback mode failed!");
  }
}

void ICACHE_FLASH_ATTR startWebserver() {
  // Start WebSocket Plug-in and handle incoming message on "onWsEvent" function
  server.addHandler(&ws);
  ws.onEvent(onWsEvent);
  server.onNotFound([](AsyncWebServerRequest * request) {
    AsyncWebServerResponse *response = request->beginResponse(404, "text/plain", "Not found");
    request->send(response);
  });

  setWebEvents();

  // HTTP basic authentication
  server.on("/login", HTTP_GET, [](AsyncWebServerRequest * request) {
      if (!request->authenticate("admin", config.getSystemPass())) {
        return request->requestAuthentication();
      }
      request->send(200, "text/plain", "Success");
  });

  server.rewrite("/", "/index.htm");
  server.begin();
}

void PAPP_callback(uint32_t papp)
{
    PAPP_now = papp;
    PAPP_updated = true;
    Serial.print("Got PAPP "); 
    Serial.println(papp);
}

void ADPS_callback(uint32_t adps)
{
    // ADPS ? Stop charging for ADPS_STOP_DURATION seconds
#define ADPS_STOP_DURATION 60
    ADPS_stop_until = millis() + ADPS_STOP_DURATION * 1000; 
    slog.logln("Got adps" + adps);
}

//////////////////////////////////////////////////////////////////////////////////////////
///////       Setup
//////////////////////////////////////////////////////////////////////////////////////////
void ICACHE_FLASH_ATTR setup() {
  Serial.begin(9600);
  if(config.getSystemDebug()) slog.logln("");
  if(config.getSystemDebug()) slog.log(ntp.iso8601DateTime() + "[Info] EVSE-WiFi - version ");
  if(config.getSystemDebug()) slog.log(swVersion);
  delay(500);

  SPI.begin();
  SPIFFS.begin();

  if (!loadConfiguration()) {
    slog.logln("[ WARNING ] Going to fallback mode!");
    fallbacktoAPMode();
  }

  // Setup LED
  if (config.getEvseLedConfig(0) != 1) {
    pinMode(config.getEvseLedPin(0), OUTPUT);
    changeLedTimes(100, 10000); // Heartbeat by default
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] LED pin set");
  }

  //Activate the button pin with pullup in any setup to prevent bouncing pin state
    pinMode(config.getButtonPin(0), INPUT_PULLUP);
    if(config.getSystemDebug()) slog.log(ntp.iso8601DateTime() + "[System] Internal pullup for button pin set: ");
    if(config.getSystemDebug()) slog.logln(config.getButtonPin(0));

  //Factory Reset when button pressed for 20 sec after boot
  if (digitalRead(config.getButtonPin(0)) == LOW) {
    pinMode(config.getEvseLedPin(0), OUTPUT);
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Button Pressed while boot!");
    digitalWrite(config.getEvseLedPin(0), HIGH);
    unsigned long millisBefore = millis();
    int button = config.getButtonPin(0);
    while (digitalRead(button) == LOW) {  
      if (millis() > (millisBefore + 20000)) {
        factoryReset();
        slog.logln(ntp.iso8601DateTime() + "[System] System has been reset to factory settings!");
        digitalWrite(config.getEvseLedPin(0), LOW);
      }
      delay(1000);
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[System] Button is pressed...");
    }
  }

  now();
  startWebserver();
  if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ SYSTEM ] End of setup routine");
  if (config.getEvseRemote(0)) sliderStatus = false;

  MDNS.addService("http", "tcp", 80);

  mqtt_PAPP.setCallback(PAPP_callback);
  mqtt_ADPS.setCallback(ADPS_callback);

  mqtt.subscribe(&mqtt_PAPP);
  mqtt.subscribe(&mqtt_ADPS);


  config.printConfig();
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 1 second...");
       mqtt.disconnect();
       delay(1000);  // wait 1 second
       retries--;
       if (retries == 0) {
           // XXX report MQTT failure
           return;
       }
  }
  Serial.println("MQTT Connected!");
}

static void update_maximum_current()
{
    // Handle ADPS which overrides everything
    if (millis() < ADPS_stop_until) {
        Serial.print("ADPS -> stopping charge for next seconds: "); Serial.println((ADPS_stop_until - millis()) / 1000);
        currentToSet = 0;
        setEVSEcurrent();
        return;
    }

    // Handle PAPP : always leave PAPP_MARGIN VA of room in power consumption
    if (!PAPP_updated) {
        return;
    }

    PAPP_updated = false;

    // Update maximum current value based on current house power consumption
    uint16_t max_current = 12 * 100;  // Maximum of 12A no matter what
    int32_t current_PAPP_headroom = PAPP_SUBSCRIBED - PAPP_now;
    if (current_PAPP_headroom < 0) current_PAPP_headroom = 0; // should be ADPS also...

    if (current_PAPP_headroom < PAPP_MARGIN) {
        Serial.print("current PAPP headroom is "); Serial.print(current_PAPP_headroom); Serial.println(" lower than margin, reducing current");
        uint32_t reduce_current_by = 100 * (PAPP_MARGIN - current_PAPP_headroom) / 230;
        Serial.print("evseAmpsConfig is "); Serial.print(evseAmpsConfig); Serial.print(" must reduce by "); Serial.println(reduce_current_by);
        if (evseAmpsConfig < reduce_current_by) {
            max_current = 0;
        } else {
            max_current = evseAmpsConfig - reduce_current_by;
        }
    }

    // Sanity checks
    if (max_current > 12 * 100)
        max_current = 12 * 100;

    if (max_current <= 64) {
        // This will be read as a low-resolution setting: set 0 instead
        max_current = 0;
    }

    // Update EVSE current if need be
    if (max_current != currentToSet) {
        Serial.print("Setting new current: "); Serial.println(max_current);
        currentToSet = max_current;
        toSetEVSEcurrent = true;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
///////       Loop
//////////////////////////////////////////////////////////////////////////////////////////
void IRAM_ATTR loop() {
  currentMillis = millis();
  unsigned long uptime = ntp.getUptimeSec();
  previousLoopMillis = currentMillis;
  changeLedStatus();

  // Ensure MQTT is connected
  MQTT_connect(); 

  //Reboot after 10 minutes in Fallback
  if (inFallbackMode && millis() > 600000) toReboot = true;

  if (uptime > 3888000) {   // auto restart after 45 days uptime
    if (vehicleCharging == false) {
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ UPDT ] Auto restarting...");
      delay(1000);
      toReboot = true;
    }
  }

  if (toReboot) {
    if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[ UPDT ] Rebooting...");
    delay(100);
    ESP.restart();
  }

  handleLed();

  if (config.getEvseRemote(0) && millisRemoteHeartbeat < millis()) {
    updateRemoteHeartbeat();
  }

  if (toActivateEVSE) {
    activateEVSE();
    delay(300);
  }
  if (toDeactivateEVSE) {
    deactivateEVSE(true);
    delay(300);
  }
  if (toInitLog) {
    if (initLogFile()) toInitLog = false;
  }
  
  updateEvseData();

  if (toSetEVSEcurrent && millisUpdateEvse < millis()) {
    setEVSEcurrent();
  }

  if (wifiInterrupted && reconnectTimer < millis()) {
    reconnectTimer = millis() + 30000; // 30 seconds
    reconnectWiFi();
  }

  if (!inAPMode && (WiFi.status() != WL_CONNECTED)) {
    wifiInterrupted = true;
  }
  else {
    if (wifiInterrupted) {
      if(config.getSystemDebug()) slog.logln(ntp.iso8601DateTime() + "[Info] WiFi connection successfully reconnected");
    }
    wifiInterrupted = false;
  }

  if (toSendStatus == true) {
    sendStatus();
    toSendStatus = false;
  }

  if (toSendSyslogToWs) {
    sendSyslogToWs();
  }

  mqtt.processPackets(100);

  update_maximum_current();
}

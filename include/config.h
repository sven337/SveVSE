#include "features.h"

#ifndef CONFIG_H_
#define CONFIG_H_

#include <Arduino.h>
#include <ArduinoJson.h>

#ifdef ESP8266
#include <FS.h>
#else
#include <SPIFFS.h>
#endif

#define ACTUAL_CONFIG_VERSION 1

struct s_wifiConfig {
    const char* bssid;
    const char* ssid;
    bool wmode;
    const char* pswd;
    bool staticip;
    const char* ip;
    const char* subnet;
    const char* gateway;
    const char* dns;
};

struct s_rfidConfig {
    bool userfid;
    bool usePN532;
    uint8_t sspin;
    int8_t rfidgain;
};

struct s_ntpConfig {
    int8_t timezone;
    const char* ntpip;
    bool dst;
};

struct s_buttonConfig {
    bool usebutton;
    uint8_t buttonpin;
};

struct s_systemConfig {
    const char* hostnm;
    const char* language;
    const char* adminpwd;
    bool wsauth;
    bool debug;
    bool disfbreboot;
    uint8_t maxinstall;
    uint8_t configversion;
    uint8_t evsecount;
    bool logging;
    bool api;
    uint16_t oledontime;
};

struct s_evseConfig {
    uint8_t mbid;
    bool alwaysactive;
    bool remote;
    uint8_t ledconfig;
    uint8_t drotation;
    bool resetcurrentaftercharge;
    uint8_t maxcurrent;
    float avgconsumption;
    bool rseActive;
    uint8_t rseValue;
    bool reg1000;
    bool reg2000;
    unsigned long updateInterval;
    uint8_t currentAfterBoot;   // Not in config.json - just for runtime
};

class EvseWiFiConfig {
public:
    bool ICACHE_FLASH_ATTR loadConfig(String = "");
    bool ICACHE_FLASH_ATTR loadConfiguration();
    bool ICACHE_FLASH_ATTR printConfigFile();
    bool ICACHE_FLASH_ATTR printConfig();
    bool ICACHE_FLASH_ATTR renewConfigFile();
    bool ICACHE_FLASH_ATTR checkUpdateConfig(String, bool (*setEVSERegister)(uint16_t reg, uint16_t val));
    bool ICACHE_FLASH_ATTR updateConfig(String);
    String ICACHE_FLASH_ATTR getConfigJson();
    bool ICACHE_FLASH_ATTR saveConfigFile(String jsonConfig);

// wifiConfig
    const char * ICACHE_FLASH_ATTR getWifiBssid();
    const char * ICACHE_FLASH_ATTR getWifiSsid();
    bool ICACHE_FLASH_ATTR getWifiWmode();
    const char * ICACHE_FLASH_ATTR getWifiPass();
    bool ICACHE_FLASH_ATTR getWifiStaticIp();
    const char * ICACHE_FLASH_ATTR getWifiIp();
    const char * ICACHE_FLASH_ATTR getWiFiSubnet();
    const char * ICACHE_FLASH_ATTR getWiFiGateway();
    const char * ICACHE_FLASH_ATTR getWiFiDns();

#if USE_METER
// meterConfig
    bool ICACHE_FLASH_ATTR getMeterActive(uint8_t meterId);
    const char * ICACHE_FLASH_ATTR getMeterType(uint8_t meterId);
    float ICACHE_FLASH_ATTR getMeterEnergyPrice(uint8_t meterId);
    uint8_t ICACHE_FLASH_ATTR getMeterPin(uint8_t meterId);
    uint16_t ICACHE_FLASH_ATTR getMeterImpKwh(uint8_t meterId);
    uint16_t ICACHE_RAM_ATTR getMeterImpLen(uint8_t meterId);
    uint8_t ICACHE_FLASH_ATTR getMeterPhaseCount(uint8_t meterId);
    uint8_t ICACHE_FLASH_ATTR getMeterFactor(uint8_t meterId);
    bool useSMeter;
    bool useMMeter;
    bool mMeterTypeSDM120;
    bool mMeterTypeSDM630;
#endif

// ntpConfig
    int8_t ICACHE_FLASH_ATTR getNtpTimezone();
    const char * ICACHE_FLASH_ATTR getNtpIp();
    bool ICACHE_FLASH_ATTR getNtpDst();

// buttonConfig
    bool ICACHE_FLASH_ATTR getButtonActive(uint8_t buttonId);
    uint8_t ICACHE_FLASH_ATTR getButtonPin(uint8_t buttonId);

// systemConfig
    const char * ICACHE_FLASH_ATTR getSystemHostname();
    const char * ICACHE_FLASH_ATTR getSystemLanguage();
    const char * ICACHE_FLASH_ATTR getSystemPass();
    bool ICACHE_FLASH_ATTR getSystemWsauth();
    bool ICACHE_FLASH_ATTR getSystemDebug();
    bool ICACHE_FLASH_ATTR getSystemDisableFallbackReboot();
    uint8_t ICACHE_FLASH_ATTR getSystemMaxInstall();
    uint8_t ICACHE_FLASH_ATTR getSystemConfigVersion();
    uint8_t ICACHE_FLASH_ATTR getSystemEvseCount();
    bool ICACHE_FLASH_ATTR getSystemLogging();
    bool ICACHE_FLASH_ATTR getSystemApi();
    uint16_t ICACHE_FLASH_ATTR getSystemOledOnTime();

// evrialConfig
    uint8_t ICACHE_FLASH_ATTR getEvseMbid(uint8_t evseId);
    bool ICACHE_FLASH_ATTR getEvseAlwaysActive(uint8_t evseId);
    bool ICACHE_FLASH_ATTR setEvseAlwaysActive(uint8_t evseId, bool aa);
    bool ICACHE_FLASH_ATTR getEvseRemote(uint8_t evseId);
    uint8_t ICACHE_FLASH_ATTR getEvseLedConfig(uint8_t evseId);
    uint8_t ICACHE_FLASH_ATTR getEvseDisplayRotation(uint8_t evseId);
    uint8_t ICACHE_FLASH_ATTR getEvseLedPin(uint8_t evseId);
    bool ICACHE_FLASH_ATTR getEvseResetCurrentAfterCharge(uint8_t evseId);
    uint8_t ICACHE_FLASH_ATTR getEvseMaxCurrent(uint8_t evseId);
    float ICACHE_FLASH_ATTR getEvseAvgConsumption(uint8_t evseId);
    uint8_t ICACHE_FLASH_ATTR getEvseCpIntPin(uint8_t evseId);
    bool ICACHE_FLASH_ATTR getEvseRseActive(uint8_t evseId);
    uint8_t ICACHE_FLASH_ATTR getEvseRsePin(uint8_t evseId);
    uint8_t ICACHE_FLASH_ATTR getEvseRseValue(uint8_t evseId);

    bool ICACHE_FLASH_ATTR getEvseReg1000(uint8_t evseId);
    bool ICACHE_FLASH_ATTR getEvseReg2000(uint8_t evseId);
    unsigned long ICACHE_FLASH_ATTR getEvseUpdateInterval(uint8_t evseId);

    uint16_t ICACHE_FLASH_ATTR getEvseCurrentAfterBoot(uint8_t evseId);
    bool ICACHE_FLASH_ATTR setEvseCurrentAfterBoot(uint8_t evseId, uint16_t current);

private:
    s_wifiConfig wifiConfig;
#if USE_METER
    s_meterConfig meterConfig[1];
#endif
    s_ntpConfig ntpConfig;
    s_buttonConfig buttonConfig[1];
    s_systemConfig systemConfig;
    s_evseConfig evseConfig[1];

    bool configLoaded;
    bool pre_0_4_Config;

protected:

};
#endif //CONFIG_H

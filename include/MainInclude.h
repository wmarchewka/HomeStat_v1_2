#ifndef __THERMOSTAT_INCLUDE
#define __THERMOSTAT_INCLUDE

#include <Arduino.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include <rom/rtc.h>

//define used in task scheduler
#define _TASK_TIMECRITICAL
#define _TASK_WDT_IDS
#define _TASK_PRIORITY
#define _TASK_STATUS_REQUEST

//dht defines
#define DHTTYPE DHT11 // DHT 11

String errorCodes[12] = {"0", "WIFI NO SSID AVAIL", "WIFI SCAN COMPLETED", "WIFI CONNECTED",
                         "WIFI CONNECT FAILED", "WIFI CONNECTION_LOST", "WIFI DISCONNECTED", "DHT TIMEOUT", "DHT CHECKSUM",
                         "TSTAT DETECT ERR", "WIFI IDLE STATUS"};

//eeprom settings memory location
const int ES_SSID PROGMEM PROGMEM = 0;  //size of 32
const int ES_SSIDPASSWORD PROGMEM = 32; //SIZE OF 32
const int ES_RESETCOUNTER PROGMEM = 65; //SIZE OF 32
const int ES_SSID_MD5 PROGMEM = 98;
const int ES_SSIDPASS_MD5 PROGMEM = 102;

//Modbus Registers Offsets (0-9999)
const int ANALOG_SENSOR_MB_HREG PROGMEM = 1;
const int TEMPERATURE_SENSOR_MB_HREG PROGMEM = 2;
const int HUMIDITY_SENSOR_MB_HREG PROGMEM = 3;
const int THERMOSTAT_HEAT_CALL_PULSE_VALUE_MB_HREG PROGMEM = 4;
const int THERMOSTAT_COOL_CALL_PULSE_VALUE_MB_HREG PROGMEM = 5;
const int THERMOSTAT_FAN_CALL_PULSE_VALUE_MB_HREG PROGMEM = 6;
const int DHT_STATUS_ERR_TIMEOUT_COUNTER_MB_HREG PROGMEM = 100;
const int DHT_STATUS_ERR_CHECKSUM_COUNTER_MB_HREG PROGMEM = 101;
const int DHT_STATUS_ERR_MB_HREG PROGMEM = 102;
const int BLINK_ERROR_CODE_MB_HREG PROGMEM = 103;
const int WIFI_STATUS_ERR_MB_HREG PROGMEM = 104;
const int THERMOSTAT_STATUS_ERR_MB_HREG PROGMEM = 105;
const int ESP_RESET_REASON_MB_HREG PROGMEM = 106;
const int ESP_CHIP_ID_HIGH_MB_HREG PROGMEM = 107;
const int ESP_CHIP_ID_LOW_MB_HREG PROGMEM = 108;
const int ESP_MEMORY_MB_HREG PROGMEM = 109;
const int WIFI_NOT_CONNECTED_MB_HREG PROGMEM = 110;
const int DHT_ROUTINE_TIME_MB_HREG PROGMEM = 111;
const int TIME_HH_MB_HREG PROGMEM = 112;
const int TIME_MM_MB_HREG PROGMEM = 113;
const int TIME_SS_MB_HREG PROGMEM = 114;
const int GOOD_PACKET_COUNTER_MB_REG PROGMEM = 115;
const int BAD_PACKET_COUNTER_MB_REG PROGMEM = 116;
const int MB_ROUTINE_TIME_MB_HREG PROGMEM = 117;
const int PROCESS_MODBUS_TIME_MB_HREG PROGMEM = 118;
const int THERM_DETECT_ROUTINE_TIME_MB_HREG PROGMEM = 119;
const int SCREEN_TIME_MB_HREG PROGMEM = 120;
const int ESP_BOOT_DEVICE_MB_HREG PROGMEM = 121;
const int ESP_RESET_COUNTER_MB_HREG PROGMEM = 122;
const int MB_START_DELAY_COUNTER_MB_REG PROGMEM = 123;
const int MB_OVERRUN_NEG_COUNTER_MB_REG PROGMEM = 124;
const int MB_OVERRUN_POS_COUNTER_MB_REG PROGMEM = 125;
const int NO_CLIENT_COUNTER_MB_REG PROGMEM = 126;
const int NOT_MODBUS_PACKET_COUNTER_MB_REG PROGMEM = 127;
const int LARGE_FRAME_COUNTER_MB_REG PROGMEM = 128;
const int FAILED_WRITE_COUNTER_MB_REG PROGMEM = 129;
const int NTP_LOOP_TIME_MB_HREG PROGMEM = 130;
const int ESP_MEMORY_LOW_POINT PROGMEM = 131;

//modbus COILS
const int HEAT_OVERRIDE_MB_COIL PROGMEM = 1;
const int HEAT_CONTROL_MB_COIL PROGMEM = 2;
const int COOL_OVERRIDE_MB_COIL PROGMEM = 3;
const int COOL_CONTROL_MB_COIL PROGMEM = 4;
const int FAN_OVERRIDE_MB_COIL PROGMEM = 5;
const int FAN_CONTROL_MB_COIL PROGMEM = 6;
const int THERMOSTAT_HEAT_CALL_MB_COIL PROGMEM = 7;
const int THERMOSTAT_COOL_CALL_MB_COIL PROGMEM = 8;
const int THERMOSTAT_FAN_CALL_MB_COIL PROGMEM = 9;
const int THERMOSTAT_STATUS_MB_COIL PROGMEM = 10;
const int ESP_RESTART_MB_COIL PROGMEM = 11;
const int ESP_CLEAR_SAVECRASH_DATA PROGMEM = 12;

//pin mappping to io expander
const int HEAT_OVERRIDE_PIN PROGMEM = 0;
const int HEAT_CONTROL_PIN PROGMEM = 1;
const int COOL_OVERRIDE_PIN PROGMEM = 2;
const int COOL_CONTROL_PIN PROGMEM = 3;
const int FAN_OVERRIDE_PIN PROGMEM = 4;
const int FAN_CONTROL_PIN PROGMEM = 5;
const int LED PROGMEM = 7;

//general variables

struct mqttDataStruct
{
    uint8_t pin_number;
    char *mqttTopic;
    char *payload; 
    uint16_t length;
};

mqttDataStruct mqttData[100];

bool glb_coolcall = false;
bool glb_DHT11debugOn = 1;
bool glb_fancall = false;
bool glb_heatcall = false;
bool glb_logDataDebug = false;
bool glb_OTA_Started = false;
bool glb_tempController = false;
bool glb_tstatDebugOn = false;
char glb_lcdTime[20] = {};
char glb_SSID[32] = {};
char glb_SSIDpassword[32] = {};
const bool COIL_OFF PROGMEM = false;
const bool COIL_ON PROGMEM = true;
const char *glb_mdnsName = "HOMESTAT";
const char *mqtt_server = "10.0.0.11";
const unsigned int ts_coolcall = 5;
const unsigned int ts_fancall = 6;
const unsigned int ts_heatcall = 4;
const unsigned int ts_humidity = 2;
const unsigned int ts_lightsensor = 3;
const unsigned int ts_temperature = 1;
const word glb_maxCoilSize PROGMEM = 256;
const word glb_maxHregSize PROGMEM = 256;
File fsUploadFile;
File glb_errorLog;
File glb_temperatureLog;
float glb_VCC = 0.0;
int glb_dataLogCount = 0;
int glb_dataServerCounter = 0;
int glb_heatRunTimeTotal = 0;
int glb_humidity = 0;
int glb_lightSensor = 0;
int glb_lowMemory = 80000;
int glb_TaskTimes[30] = {};
int glb_temperature = 0;
IPAddress glb_ipAddress; //ip address
long glb_wifiRSSI = 0;
String glb_dataLogPath = "/datalog.csv";
String glb_debugLogPath = "/debuglog.txt";
String glb_errorLogPath = "/errorlog.txt";
String glb_systemLogPath = "/systemlog.txt";
String glb_BootTime = "";
String glb_dhtStatusError = "";
String glb_testLED = "";
String glb_thermostatStatus = "";
String glb_timeDay = "";
String glb_timeHour = "";
String glb_TimeLong = "";
String glb_timeMin = "";
String glb_timeMonth = "";
String glb_timeSec = "";
String glb_TimeShort = "";
String glb_timeWeekDay = "";
String glb_timeYear = "";
uint32_t glb_freeHeap = 0;
uint32_t glb_resetCounter = 0;
uint64_t chipid = 0;
volatile word glb_coolPulseCounter = 0;
volatile word glb_coolPulseDuration = 0;
volatile word glb_fanPulseCounter = 0;
volatile word glb_fanPulseDuration = 0;
volatile word glb_heatPulseCounter = 0;
volatile word glb_heatPulseDuration = 0;
word glb_BlinkErrorCode = 0;
word glb_eepromHregCopy[glb_maxHregSize] = {};
word glb_errorDHT = 0;
word glb_errorThermostat = 0;
word glb_wifiNotConnectedCounter = 0;
word glb_WiFiStatus = 0;

//function declarations

bool FileSystem_DeleteFile(String);
bool FileSystem_PrintFile(String, bool);
bool LED_OnEnable();
String WebServer_getPage();
void ChipID_Acquire();
void DataServer_Process();
void DataServer_Setup();
void DeepSleepMode();
void DHT11_Sensor_Setup();
void DHT11_TempHumidity();
void ErrorCodes_Process();
void ESP_Restart();
void FileSystem_CreateHTML();
void FileSystem_DataLogCreate();
void FileSystem_DataLogSave();
void FileSystem_DebugDataSave(String);
void FileSystem_DebugLogCreate();
void FileSystem_ErrorLogCreate();
void FileSystem_ErrorLogSave(String);
void FileSystem_Format();
void FileSystem_ListDirectory();
void FileSystem_SystemDataSave(String);
void FileSystem_SystemLogCreate();
void handleSubmit();
void I2C_Setup();
void Interrupt_Detect_AC();
void IO_ControlPins();
void IO_Pins_Setup();
void LCD_DrawText(int, int, char *, uint16_t, uint16_t);
void LCD_Setup();
void LCD_Update();
void LED_Error();
void LED_Off();
void LED_On();
void LED_OnDisable();
void mDNS_Setup();
void MQTT_Callback(char *, byte *, unsigned int);
void MQTT_Data_Setup();
void MQTT_Publish();
void MQTT_Reconnect();
void MQTT_RunLoop();
void MQTT_Setup();
void OTA_Setup();
void OTA_Update();
void Reset_Reason();
void saveConfigCallback();
void selftestMcp();
void StartupPrinting_Setup();
void Tasks_Enable_Setup();
void TaskScheduler_Setup();
void TelnetServer_Process();
void TelnetServer_ProcessCommand();
void TelnetServer_Setup();
void testEspOutputPin(int, int);
void testMcpOutputPin(int, int);
void testVCC();
void Thermostat_ControlDisable();
void Thermostat_Detect();
void ThermostatMode_Setup();
void TimeRoutine();
void TimeSync_Setup();
void verbose_print_reset_reason(RESET_REASON);
void WebServer_HandleDataLog();
void WebServer_HandleDatalogUpload();
void WebServer_HandleErrorLog();
void WebServer_HandleFileUpload();
void WebServer_HandleInformation();
void WebServer_HandleNotFound();
void WebServer_HandleRoot();
void WebServer_Process();
void WebServer_Root();
void WebServer_Setup();
void Wifi_CheckStatus();
void Wifi_Setup();
void WiFiManager_Setup();

#endif

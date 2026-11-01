// =====================================================================
// CarController — نسخه «Fast» (بهینه‌سازی سرعت SIM800)
// تمام بهینه‌سازی‌ها با برچسب [OPT-..] در متن کد قابل جستجو هستند:
//  [OPT-3]  URC ثبت شبکه (+CREG/+CGREG) → تشخیص «همان لحظه» وصل شدن به شبکه
//  [OPT-4]  پولینگ پویا: تا قبل از ثبت شبکه هر ۳ ثانیه (به‌جای ۶۰ ثانیه) + COPS سریع تا پیدا شدن APN
//  [OPT-5]  warm-up فوری حامل SAPBR بلافاصله بعد از ثبت شبکه
//  [OPT-6]  حذف پنجره‌ی انتظار ۹۰ ثانیه‌ای GPS → LBS بلافاصله اجرا می‌شود
//  [OPT-7]  پاسخ‌گویی به تماس مجاز حتی در حین LBS (لغو LBS به‌جای از دست دادن تماس)
//  [OPT-8]  سریع‌تر شدن بوت (حذف تأخیرهای ثابت power-key/CPIN/ATA)
//  [OPT-10] بادریت خودکار تا 115200 (با بازگشت امن به 9600)
//  [OPT-11] حذف بلوک تکراری پردازش URC/فرمان‌ها در updateSim800
//  [OPT-12] پولینگ CSQ/CREG دیگر پاسخ‌های SAPBR را نمی‌بلعد
//  [OPT-14] CIPGSMLOC مستقیم وقتی حامل از قبل باز است (با retry خودکار)
//  [OPT-15] تشخیص سریع‌تر ماژول بی‌پاسخ (۱۰۰s به‌جای ۲۰۰s)
// =====================================================================
// SMS message type kept in this single sketch.
struct SimMsg {
  int idx;
  String st;
  String from;
  String date;
  String text;
};
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#if __has_include("esp_netif_net_stack.h")
#include "esp_netif_net_stack.h"   // esp_netif_get_netif_impl() در IDF 5.x اینجاست
#endif
#include <string.h>
#include <stdlib.h>
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "lwip/tcpip.h"
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <MFRC522.h>
#include <mbedtls/gcm.h>
#include <mbedtls/hkdf.h>
#include <mbedtls/md.h>
#include <mbedtls/base64.h>
#include <mbedtls/sha256.h>
// ===================== Pin Definitions (ESP32-S3) =====================
// GPIO26..32 : فلش/PSRAM  | GPIO33..37 : PSRAM اکتال (N8R8/N16R8)  | 19,20 : USB
// GPIO0,3,45,46 : strapping | 43,44 : UART0
#define OUTPUT_COUNT      5

#define PIN_DOOR_DRIVER   36
#define PIN_TRUNK         40
#define PIN_FUEL_PUMP     38
#define PIN_CAR_SWITCH    39
#define PIN_DOORS_INVERSE 37
#define PIN_BUZZER         1   // بازر/آژیر روی ESP32-S3 (از طریق ترانزیستور)
#define BUZZER_ON_LEVEL   HIGH // اگر بازر شما active-low است، این دو را جابه‌جا کنید
#define BUZZER_OFF_LEVEL  LOW
// هشدار: GPIO36/37 فقط روی بردهایی آزادند که PSRAM اکتال (R8) ندارند (مثلاً N8 / N16 / N4 / R2)

// ===================== SIM800 Module Pins =====================
#define SIM800_RX_PIN      18   // ESP32 RX  <- SIM800 TXD
#define SIM800_TX_PIN      17   // ESP32 TX  -> SIM800 RXD
#define SIM800_PWK_PIN      4   // Power Key (LOW pulse to power on)
#define SIM800_RI_PIN       5   // Ring Indicator input
#define SIM800_BAUD_DEFAULT 9600UL   // [OPT-10] نرخ پشتیبان (مثل قبل)
#define SIM800_BAUD_FAST    115200UL // [OPT-10] ابتدا امتحان می‌شود؛ اگر بی‌پاسخ بود به 9600 برمی‌گردیم

#define MAX_PHONE_NUMBERS   5
#define MAX_CALL_DURATION_MS 60000L

// جدید: حلقه‌ی SMS لوکیشن با فاصله‌ی قابل تنظیم بر حسب دقیقه (فرمان: Location <عدد 1..120>)
#define LOC_SMS_MIN_MINUTES          1
#define LOC_SMS_MAX_MINUTES          120
#define LOC_SMS_INTERVAL_DEFAULT_MS  60000UL   // «Location» بدون عدد = هر ۱ دقیقه
// جدید: تشخیص قطع آنتن GPS
#define GPS_ANTICUT_TIMEOUT_MS 60000UL   // 5 دقیقه
// جدید: OTA
#define OTA_HOSTNAME          "CarControl"
// ===================== GPS (NEO-6M) =====================
#define GPS_RX_PIN           7
#define GPS_TX_PIN           6
#define GPS_BAUD             9600
#define GPS_STALE_MS         120000UL   // 2 min without fix -> stale
#define GPS_FIX_TIMEOUT_MS   90000UL    // max wait for first fix

// ===================== Safe Zone =====================
#define SAFE_ZONE_CHECK_MS            5000UL
#define SAFE_ZONE_NO_FIX_TIMEOUT_MS   60000UL
#define SAFE_ZONE_CALL_GAP_MS         5000UL
#define SAFE_ZONE_CALL_TIMEOUT_MS     30000UL
#define SAFE_ZONE_CALL_POLL_MS        3000UL
// ===================== Pulse Timing =====================
#define DOOR_PULSE_MS   1000
#define TRUNK_PULSE_MS  1000
#define MAX_PULSES      5

// ===================== Default AP Settings =====================
#define DEFAULT_SSID      "CarControl"
#define DEFAULT_PASSWORD  "12345678"
#define DEFAULT_IP        "192.168.4.1"
#define HTTP_PORT         80
#define AP_CHANNEL        6
#define AP_MAX_CLIENTS    5

// ===================== Security Settings =====================
#define ACTIVATION_CODE       "DEPRECATED"
#define MAX_ALLOWED_DEVICES   20

// ===================== Global Objects =====================
WebServer server(HTTP_PORT);
DNSServer dnsServer;
Preferences preferences;

String ap_ssid;
String ap_password;
IPAddress ap_ip;

bool outputState[OUTPUT_COUNT] = {};
bool desiredState[OUTPUT_COUNT] = {};

String customOutputNames[OUTPUT_COUNT];

String allowedMACs[MAX_ALLOWED_DEVICES];
String allowedIPs[MAX_ALLOWED_DEVICES];
int allowedDevicesCount = 0;

String systemLanguage = "fa";
String nextLanguage = "fa";

int signalThreshold = 50;

bool weakAutoDisconnectEnabled = false;
bool weakDisconnectActive = false;
unsigned long lastWeakCheck = 0;

// Momentary pulse management
struct Pulse {
  bool active;
  int pin;
  unsigned long start;
  unsigned long duration;
  int resetOutputIndex;
};

Pulse pulses[MAX_PULSES];

// ---- انواع داده‌ی پیامک (باید قبل از اولین تابع تعریف شوند) ----
// SimMsg is defined at the beginning of this sketch; no separate header is needed.




// ===================== SIM800 Globals =====================
HardwareSerial sim800Serial(2);
// ===================== SMS Location Loop =====================
bool     locSmsActive = false;
String   locSmsTargetNumber = "";
unsigned long locSmsLastSend = 0;
int      locSmsSendCount = 0;              // شمارنده برای لاگ (حلقه تا فرمان Stop location ادامه دارد)
unsigned long locSmsIntervalMs = LOC_SMS_INTERVAL_DEFAULT_MS;   // فاصله‌ی ارسال لوکیشن (با فرمان Location <دقیقه> تنظیم می‌شود)

// ===================== Pending Status SMS =====================
// اگر Status هنگام تماس فعال درخواست شود، بعد از پایان تماس فقط یک‌بار ارسال می‌شود
bool     pendingStatusSms  = false;
String   pendingStatusFrom = "";

// ===================== Serial Command Console [PATCH-S] =====================
// lines typed in Serial Monitor are collected here and executed like SMS commands
String   serialCmdBuf = "";

// ===================== Pending Alarm SMS =====================
bool     pendingAlarmSms = false;
String   pendingAlarmSmsReason = "";

// ===================== GPS Anti-Cut =====================
bool     gpsAntiCutEnabled = false;
unsigned long gpsNoFixSince = 0;

// ===================== WiFi Presence =====================
bool     wifiUserWasConnected = false;
unsigned long lastWifiCheck = 0;
// ===================== GPS Safe Zone =====================
bool     safeZoneEnabled   = false;
double   safeZoneLat       = 0;
double   safeZoneLng       = 0;
uint32_t safeZoneRadiusM   = 100;
bool     safeZoneViolated  = false;
bool     safeZoneAlarmActive = false;
unsigned long safeZoneLastCheck = 0;
unsigned long safeZoneNoFixSince = 0;

int      safeZoneCallIdx   = 0;
unsigned long safeZoneCallStart = 0;
bool     safeZoneCallAnswered    = false;   // آیا تماس پاسخ داده شد؟
unsigned long safeZoneCallLastPoll = 0;      // زمان آخرین AT+CLCC

bool gpsSerialStarted = false;
// =====================================================

bool sim800Enabled = true;            // [CHANGE] همیشه فعال (چک‌باکس حذف شد)
bool sim800InternetEnabled = true;    // [CHANGE] همیشه فعال (چک‌باکس حذف شد)
bool sim800Ready   = false;
unsigned long sim800LastInit   = 0;
unsigned long sim800LastStatus = 0;
unsigned long sim800LastSignal = 0;
unsigned long sim800Baud       = 0;      // [OPT-10] بادریت فعال (0 = هنوز تشخیص داده نشده؛ در setup تشخیص می‌یافت)

int gsmCSQ = 99;
int gsmPercent = 0;
bool gprsRegistered = false;
bool gprsAttached   = false;
String sim800Carrier = "";
String sim800APN     = "";
String sim800IP      = "";
// شمارنده‌ی داده‌ی برنامه؛ SIM800 در حالت SAPBR آمار کل ترافیک اپراتور را ارائه نمی‌کند.
uint64_t sim800TxDataBytes = 0;
uint64_t sim800RxDataBytes = 0;

String phoneNumbers[MAX_PHONE_NUMBERS];

// ===================== MQTT Settings (CONFIGURATION ONLY) =====================
// No MQTT connection, AT commands, or remote vehicle control are activated by this menu.
// Broker authentication and transport will be implemented/tested separately.
struct MqttConfig {
  bool requestedEnabled = false;   // saved intent only; not used by SIM800 in this build
  bool tlsRequested = true;        // requirement for a future transport implementation
  String host = "smartcarlock.ir";
  uint16_t port = 8883;
  String deviceId = "";
  String username = "";
  String password = "";           // stored in NVS; never rendered back in HTML
  String commandKey = "";         // separate 64-character hex HMAC key; provisioned by admin
  String topicPrefix = "smartcar";
  uint16_t keepAliveSec = 60;
  uint16_t positionIntervalSec = 60;
  uint16_t heartbeatIntervalSec = 180;
};
MqttConfig mqttConfig;
// HTML CSRF token removed; local API now uses a bearer token.




// =================== SIM800 Native MQTT Transport ===================
// Security: no plaintext fallback; certificate trust store must be provisioned
// on the SIM800 and SSL server certificate verification must be available.
// This modem family has firmware variants WITHOUT native MQTT AT commands.
enum MqttStep : uint8_t {
  MQ_DOWN, MQ_DISC, MQ_PROBE, MQ_ROOT, MQ_CLOCK, MQ_VERIFY, MQ_NO_CLIENT,
  MQ_SSL, MQ_CID, MQ_URL, MQ_CLIENT, MQ_USER, MQ_PASS, MQ_KEEP, MQ_CLEAN,
  MQ_CONNECT, MQ_SUB, MQ_READY, MQ_PUB, MQ_CHECK
};
MqttStep mqttStep = MQ_DOWN;
bool mqttWaiting = false;
bool mqttConnected = false;
bool mqttRootSeen = false;
bool mqttClockValid = false;
bool mqttGotOk = false;
bool mqttPubAwaitUrc = false;
bool mqttSubAwaitUrc = false;
bool mqttPubAcked = false;
bool mqttSubAcked = false;
uint32_t mqttStepSince = 0;
uint32_t mqttLastAttempt = 0;
uint32_t mqttBackoffMs = 15000;
uint32_t mqttLastPosition = 0;
uint32_t mqttLastHeartbeat = 0;
uint32_t mqttLastCheck = 0;
uint32_t mqttLastSequence = 0;
String mqttLastError = "Disabled";
String mqttCurrentTopic;
String mqttPendingPayload;
String mqttPublishKind;
String mqttAckPayloads[8];
uint8_t mqttAckHead = 0, mqttAckCount = 0;
void mqttTick();
bool mqttOnModemLine(const String& line);
void mqttResetTransport(const String& why);
void mqttIncoming(const String& line);
String mqttStateText();

// ===================== SMS commands / Security / Location =====================
#define SMS_CMD_COUNT    7      // Location, Lock, Unlock, Alarm_on, Alarm_off, Stop location, Status
#define SEC_GRACE_MS     300000UL   // 5 دقیقه
#define SEC_DEBOUNCE_MS  30000UL    // قطعی آنتن باید ۳۰ ثانیه ادامه داشته باشد

String smsCmd[SMS_CMD_COUNT];

bool carLocked = false;         // قفل فعال (پمپ و سوئیچ نمی‌توانند روشن شوند)
bool secLocked = false;         // این قفل توسط سیستم امنیتی اعمال شده
bool savedFuel = true, savedSwitch = true;
bool alarmManual = false;

bool securityEnabled = false;
bool securitySuppressed = false;
int  secState = 0;              // 0 عادی | 1 مهلت+بازر | 2 کاربر وصل شد | 3 قفل امنیتی
unsigned long secGraceStart = 0;
unsigned long gsmBadSince = 0;

int  locPhase = 0;              // 0 بیکار | 1 منتظر SAPBR | 2 منتظر CIPGSMLOC
unsigned long locStart = 0;
String locRequester = "";
bool locWanted = false;                 // مرحله‌ی بیرون‌آوردن اینترنت برای Location است؟
unsigned long bearerLastFail = 0;
unsigned long sim800LastBearerTry = 0;
unsigned long sim800LastStorageCheck = 0;
String locResult = "";
bool locDone = false;
bool locFastRetry = false;   // [OPT-14] مسیر سریع CIPGSMLOC با حامل باز؛ برای retry یک‌باره در صورت شکست

bool ringDetected = false;
bool callActive = false;
unsigned long callStart = 0;
String activeCaller = "";
// ===================== GPS State =====================
TinyGPSPlus   gps;
HardwareSerial gpsSerial(1);
bool   gpsEnabled     = true;         // [CHANGE] همیشه فعال (چک‌باکس حذف شد)
int    gpsSite        = 0;              // 0=Google, 1=Neshan, 2=Balad
bool   gpsFix         = false;
double gpsLat         = 0;
double gpsLng         = 0;
unsigned long gpsLastFixMs = 0;
unsigned long gpsFirstSearchMs = 0;
int    gpsSatellites  = 0;
float  gpsHdop        = 0;

volatile bool g_smsOk      = false;   // +CMGS: <n> دریافت شد
volatile bool g_smsFail    = false;   // +CMS ERROR دریافت شد
String sim800RxBuf = "";

const uint8_t outputPins[OUTPUT_COUNT] = {
    PIN_DOOR_DRIVER,
    PIN_TRUNK,
    PIN_FUEL_PUMP,
    PIN_CAR_SWITCH,
    PIN_DOORS_INVERSE
};

// ===================== Function Prototypes =====================
void loadAllSettings();
void initPrefsNamespaces();
// Mobile API declarations
void sclLoadToken();
void sclPing();
void sclPair();
void sclPhones();
void sclRevokePhone();
void sclState();
void sclConfig();
void sclAction();
void sclApSave();
void sclReply(int code,const String& data);
void setupWiFiAP();
void setupDNSServer();
void setupWebServer();
// OTA
void setupOTA();
// SMS helpers
bool sendSms(const String& to, const String& text);
// WiFi reset
void checkWifiConnection();
void handleSerialCommands();   // [PATCH-S] serial command console
void resetAllSecurity();
void handleToggle();
void handleStatus();
void handleGetRSSI();
void handleSaveMqttSettings();
void loadMqttConfig();
bool saveMqttConfig(const MqttConfig& c);
String defaultMqttDeviceId();
bool mqttSafeToken(const String& value, size_t maxLength, bool allowSlash = false);
bool mqttValidHost(const String& host);

void setOutput(int index, bool state);
void setDoors(bool lock);
void triggerTrunk();
void initPulses();
void startPulse(int pin, unsigned long duration, int resetOutputIndex = -1);
void cancelPulse(int pin);
void handlePulses();

int getClientRSSI();
int rssiToPercentage(int rssi);


IPAddress stringToIP(String ipStr);
bool ipIsUsable(IPAddress ip);

void loadDevices();
void saveDevices();
int findDeviceByMAC(String mac);
void addOrUpdateDevice(String mac, String ip);
String getMacAddressString(const uint8_t* mac);
// کش IP→MAC برای کاهش فشار روی lwIP
struct MacCacheEntry {
  IPAddress ip;
  String    mac;
  unsigned long ts;
};
static MacCacheEntry macCache[AP_MAX_CLIENTS];
static int macCacheIdx = 0;
#define MAC_CACHE_TTL_MS 60000UL   // 60 ثانیه
String getMacByIP(IPAddress ip);
bool isRequestAuthorized();

void handleSetWeakAutoDisconnect();
void saveWeakAutoSetting();
void handleWeakSignalProtection();

void loadCustomNames();
void saveCustomName(int idx);
void handleSetOutputName();

void saveLanguageSettings();
String t(const char* key);
String getOutputName(int idx);

void handleSetSignalThreshold();
void saveSignalThreshold();

// --- SIM800 ---
void loadSim800Config();
void saveSim800Config();
void setupSim800();
void powerOnSim800();
void updateSim800();
bool sim800Probe();
String sim800Exchange(const String& cmd, unsigned long timeout);
bool sim800SendAT(String cmd, String expected, unsigned long timeout);
bool sim800WaitFor(const char* token, unsigned long timeout, String* out);
void sim800PumpRx();
void sim800ProcessLine(const String& line);
String sim800CleanAndQueue(const String& raw);
void updateSim800Status();
void updateGsmSignal();
void detectAndSetAPN();
String apnForOperator(String op);
void handleSim800URC(String line);
void handleIncomingCall(String number);
void handleIncomingSMS(int index);
void processSMSCommand(String from, String body);
bool isPhoneAllowed(String number);
String normalizePhone(String number);
void startLocation(const String& who);
void startBearer(bool wantLoc);
void bearerFail(const String& msg);
void locFinish(const String& msg);
void sim800CheckStorage();
bool cmdMatch(int i, const String& upBody);
String secStateName();
String jsonEscape(const String& in);
bool asciiOnly(const String& s);

bool sim800ConfigureSms();
void sim800VerifySmsConfig();
void sim800DrainSim();
void sim800PollSms();
bool sim800HandleMessage(const String& from, const String& text);
int  smsClassify(String body);
String smsCmdNormalized(int i);
String smsCollapseSpaces(const String& in);
String buildStatusText();
bool   sendSmsChunked(const String& to, const String& text);
bool   sendStatusSms(const String& from);
void inboxLoad();
void inboxSave();

void applyLock(bool lockDoors, bool bySecurity);   // [CHANGE] پروتوتایپ با تعریف واقعی هماهنگ شد
void releaseLock(bool restore);
void unlockCar();
void saveLockState();
void loadLockState();
bool authorizedClientConnected();
bool gsmHealthy();
void updateSecurity();
void updateBuzzer();

// ===================== RFID RC522 (READ ONLY) =====================
#define RFID_SCK 12
#define RFID_MISO 13
#define RFID_MOSI 11
#define RFID_SS 10
#define RFID_RST 9
#define RFID_TRIGGER_OUT 41
#define RFID_DONE_IN 42
#define RFID_SLOT_COUNT 2
MFRC522 rfidReader(RFID_SS, RFID_RST);
MFRC522::MIFARE_Key rfidSectorKey;
struct RfidEntry { String name, secret, key720, uid, sha256; bool valid=false; };
RfidEntry rfidEntries[RFID_SLOT_COUNT];
uint8_t rfidAesKeys[RFID_SLOT_COUNT][32] = {};
bool rfidReady=false;
int rfidStage=0; // 0 first presentation, 1 waiting for removal, 2 second presentation, 3 waiting for GPIO42 LOW
bool rfidPresent=false;
bool rfidDoneHighSeen=false; // Require GPIO42 HIGH then LOW before restarting cycle
unsigned long rfidSince=0,rfidLastSeen=0,rfidLastPoll=0;
String rfidCurrentUid;
const uint8_t RFID_BLOCKS[45]={4,5,6,8,9,10,12,13,14,16,17,18,20,21,22,24,25,26,28,29,30,32,33,34,36,37,38,40,41,42,44,45,46,48,49,50,52,53,54,56,57,58,60,61,62};

bool rfidDecode(const String &v,uint8_t *buf,size_t cap,size_t &out){
  out=0; return mbedtls_base64_decode(buf,cap,&out,(const unsigned char*)v.c_str(),v.length())==0;
}
bool rfidDerive(const RfidEntry &e,uint8_t key[32]){
  if(e.name.isEmpty())return false;
  if(e.key720.length()){
    uint8_t full[720];size_t n=0;
    bool ok=rfidDecode(e.key720,full,sizeof(full),n)&&n==720;
    if(ok)memcpy(key,full,32);
    memset(full,0,sizeof(full));return ok;
  }
  uint8_t secret[64];size_t n=0;
  if(!rfidDecode(e.secret,secret,sizeof(secret),n)||n!=32)return false;
  String info="tag:"+e.name;
  const char *salt="RC522-WIN-v1";
  int rc=mbedtls_hkdf(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
    (const unsigned char*)salt,strlen(salt),secret,n,
    (const unsigned char*)info.c_str(),info.length(),key,32);
  memset(secret,0,sizeof(secret));return rc==0;
}
String rfidHex(const uint8_t *bytes,size_t n){
  const char* hex="0123456789ABCDEF";String v;v.reserve(n*2);
  for(size_t i=0;i<n;i++){v+=hex[bytes[i]>>4];v+=hex[bytes[i]&15];}return v;
}
// WakeupA detects an already halted card; PICC_IsNewCardPresent alone cannot do that.
bool rfidSelect(String &uid){
  byte atqa[2],len=sizeof(atqa);
  MFRC522::StatusCode st=rfidReader.PICC_WakeupA(atqa,&len);
  if(st!=MFRC522::STATUS_OK && st!=MFRC522::STATUS_COLLISION)return false;
  if(!rfidReader.PICC_ReadCardSerial())return false;
  uid=rfidHex(rfidReader.uid.uidByte,rfidReader.uid.size);return true;
}
void rfidFinishCard(){rfidReader.PCD_StopCrypto1();rfidReader.PICC_HaltA();}
bool rfidReadRaw(uint8_t raw[720]){
  for(int i=0;i<45;i++){
    if(i%3==0){
      if(rfidReader.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,RFID_BLOCKS[i],&rfidSectorKey,&rfidReader.uid)!=MFRC522::STATUS_OK)return false;
    }
    byte data[18],len=sizeof(data);
    if(rfidReader.MIFARE_Read(RFID_BLOCKS[i],data,&len)!=MFRC522::STATUS_OK)return false;
    memcpy(raw+16*i,data,16);
  }
  return true;
}
bool rfidVerify(const uint8_t raw[720],int slot,const String &uid){
  RfidEntry &e=rfidEntries[slot];
  if(!e.valid)return false;
  if(e.uid.length() && !uid.equalsIgnoreCase(e.uid))return false;
  if(e.sha256.length()){
    uint8_t digest[32];
    if(mbedtls_sha256(raw,720,digest,0)!=0)return false;
    if(!rfidHex(digest,32).equalsIgnoreCase(e.sha256))return false;
  }
  size_t size=((size_t)raw[0]<<8)|raw[1];if(size>690)return false;
  uint8_t plain[690];mbedtls_gcm_context g;mbedtls_gcm_init(&g);
  int rc=mbedtls_gcm_setkey(&g,MBEDTLS_CIPHER_ID_AES,rfidAesKeys[slot],256);
  if(rc==0)rc=mbedtls_gcm_auth_decrypt(&g,690,raw+2,12,
    (const unsigned char*)e.name.c_str(),e.name.length(),raw+704,16,raw+14,plain);
  mbedtls_gcm_free(&g);
  bool ok=rc==0;
  if(ok)for(size_t i=size;i<690;i++)if(plain[i]!=0){ok=false;break;}
  memset(plain,0,sizeof(plain));return ok;
}
int rfidAuthenticatePresented(const String &uid){
  uint8_t raw[720];
  if(!rfidReadRaw(raw))return -1;
  for(int i=0;i<RFID_SLOT_COUNT;i++)if(rfidVerify(raw,i,uid))return i;
  return -1;
}
void rfidLoad(){
  Preferences pref;pref.begin("rfid",true);
  for(int i=0;i<RFID_SLOT_COUNT;i++){
    String p="t"+String(i);
    RfidEntry &e=rfidEntries[i];
    e.name=pref.getString((p+"n").c_str(),"");
    e.secret=pref.getString((p+"s").c_str(),"");
    e.key720=pref.getString((p+"k").c_str(),"");
    e.uid=pref.getString((p+"u").c_str(),"");
    e.sha256=pref.getString((p+"h").c_str(),"");
    e.valid=rfidDerive(e,rfidAesKeys[i]);
  }
  pref.end();
}
bool rfidConfigured(){
  for(int i=0;i<RFID_SLOT_COUNT;i++)if(!rfidEntries[i].valid)return false;
  return true;
}
void rfidReset(){
  Preferences pref;pref.begin("rfid",false);pref.clear();pref.end();
  for(int i=0;i<RFID_SLOT_COUNT;i++){rfidEntries[i]=RfidEntry();memset(rfidAesKeys[i],0,32);}
  rfidStage=0;rfidPresent=false;rfidDoneHighSeen=false;rfidCurrentUid="";
  digitalWrite(RFID_TRIGGER_OUT,LOW);
}
void rfidSetup(){
  pinMode(RFID_TRIGGER_OUT,OUTPUT);digitalWrite(RFID_TRIGGER_OUT,LOW);
  pinMode(RFID_DONE_IN,INPUT_PULLDOWN);
  SPI.begin(RFID_SCK,RFID_MISO,RFID_MOSI,RFID_SS);
  rfidReader.PCD_Init();
  for(int i=0;i<6;i++)rfidSectorKey.keyByte[i]=0xFF;
  rfidLoad();rfidReady=true;
}
// Registration: verifies the physical card BEFORE committing the entry.
// Encode field-specific validation failures as JSON for the RFID form.
String rfidJsonEscape(const String &s){
  String o; o.reserve(s.length()+12);
  for(size_t i=0;i<s.length();i++){
    char c=s[i];
    if(c=='\\'||c=='"'){o+='\\';o+=c;}
    else if(c=='\n')o+="\\n";
    else if(c=='\r')o+="\\r";
    else if((uint8_t)c>=32)o+=c;
  }
  return o;
}
void rfidFormError(int status,const String &field,const String &message){
  server.send(status,"application/json; charset=utf-8", "{\"ok\":false,\"field\":\""+rfidJsonEscape(field)+"\",\"message\":\""+rfidJsonEscape(message)+"\"}");
}
void rfidRegister(){
  if(!isRequestAuthorized()){rfidFormError(403,"general","دسترسی مجاز نیست.");return;}
  if(!server.hasArg("slot")||!server.hasArg("name")){rfidFormError(400,"name","نام تگ و شماره جایگاه لازم است.");return;}
  int slot=server.arg("slot").toInt();
  if(slot<0||slot>=RFID_SLOT_COUNT){rfidFormError(400,"general","شماره جایگاه نامعتبر است.");return;}
  if(rfidEntries[slot].valid){rfidFormError(409,"general","این جایگاه قبلاً ثبت شده است. برای پاک‌کردن از RFIDRESET استفاده کنید.");return;}
  String name=server.arg("name"),secret=server.arg("secret"),key=server.arg("key720");
  String uid=server.arg("uid"),sha=server.arg("sha256");
  name.trim();secret.trim();key.trim();uid.trim();sha.trim();
  if(name.isEmpty()||name.length()>100){rfidFormError(400,"name","نام تگ باید بین ۱ تا ۱۰۰ نویسه باشد.");return;}
  if(secret.isEmpty()&&key.isEmpty()){rfidFormError(400,"secret","یکی از secret_b64 یا key720_b64 را وارد کنید.");return;}
  if(!secret.isEmpty()&&!key.isEmpty()){rfidFormError(400,"key720","فقط یکی از دو نوع کلید را وارد کنید.");return;}
  if(uid.length() && (uid.length()<8||uid.length()>20)){rfidFormError(400,"uid","UID باید ۸ تا ۲۰ نویسه هگزادسیمال باشد.");return;}
  for(size_t i=0;i<uid.length();i++)if(!isxdigit((unsigned char)uid[i])){rfidFormError(400,"uid","UID فقط باید شامل ارقام 0-9 و حروف A-F باشد.");return;}
  if(sha.length()!=0&&sha.length()!=64){rfidFormError(400,"sha256","SHA256 باید دقیقاً ۶۴ نویسه باشد.");return;}
  for(size_t i=0;i<sha.length();i++)if(!isxdigit((unsigned char)sha[i])){rfidFormError(400,"sha256","SHA256 باید هگزادسیمال باشد.");return;}
  RfidEntry candidate;candidate.name=name;candidate.secret=secret;candidate.key720=key;
  candidate.uid=uid;candidate.sha256=sha;candidate.valid=true;
  uint8_t aes[32];
  if(!rfidDerive(candidate,aes)){rfidFormError(400,key.isEmpty()?"secret":"key720","کلید Base64 نامعتبر است یا طول آن با نوع کلید مطابقت ندارد.");return;}
  RfidEntry previous=rfidEntries[slot];uint8_t oldKey[32];memcpy(oldKey,rfidAesKeys[slot],32);
  rfidEntries[slot]=candidate;memcpy(rfidAesKeys[slot],aes,32);memset(aes,0,32);
  String presented;bool valid=false;
  if(rfidSelect(presented)){
    uint8_t raw[720];valid=rfidReadRaw(raw)&&rfidVerify(raw,slot,presented);
    rfidFinishCard();
  }
  if(!valid){rfidEntries[slot]=previous;memcpy(rfidAesKeys[slot],oldKey,32);
    rfidFormError(409,"general","تگ شناسایی نشد یا اعتبارسنجی AES-GCM/UID/SHA256 موفق نبود. کارت را روی خواننده نگه دارید و دوباره امتحان کنید.");return;}
  Preferences pref;pref.begin("rfid",false);String p="t"+String(slot);
  pref.putString((p+"n").c_str(),name);pref.putString((p+"s").c_str(),secret);
  pref.putString((p+"k").c_str(),key);pref.putString((p+"u").c_str(),uid);
  pref.putString((p+"h").c_str(),sha);pref.end();
  rfidPresent=false;rfidStage=0;
  server.send(200,"application/json; charset=utf-8","{\"ok\":true,\"message\":\"تگ با موفقیت بررسی و ذخیره شد.\"}");
}

// RFID state machine:
// 0: first presentation >=2 seconds -> unlock all + door pulse
// 1: wait for actual removal (debounced 650 ms)
// 2: second, separate presentation >5 seconds -> GPIO41 HIGH
// 3: wait for GPIO42 HIGH, then LOW before restarting at stage 0
void rfidTick(){
  if(!rfidReady || !rfidConfigured()){
    digitalWrite(RFID_TRIGGER_OUT,LOW);
    return;
  }
  unsigned long now=millis();
  bool doneHigh=(digitalRead(RFID_DONE_IN)==HIGH);
  if(rfidStage==3){
    if(doneHigh){
      rfidDoneHighSeen=true;
      digitalWrite(RFID_TRIGGER_OUT,LOW);
    }else if(rfidDoneHighSeen){
      // GPIO42 completed its HIGH -> LOW cycle; a fresh tag presentation is required.
      rfidStage=0;
      rfidDoneHighSeen=false;
      rfidPresent=false;
      rfidCurrentUid="";
      rfidLastSeen=0;
      rfidSince=0;
      rfidLastPoll=now;
    }
    return;
  }
  // External completion input HIGH inhibits new RFID actions as well.
  if(doneHigh){
    digitalWrite(RFID_TRIGGER_OUT,LOW);
    rfidStage=3;
    rfidDoneHighSeen=true;
    rfidPresent=false;
    rfidCurrentUid="";
    return;
  }
  if(now-rfidLastPoll<120)return;
  rfidLastPoll=now;
  String uid;
  bool detected=rfidSelect(uid);
  if(detected){
    bool same=rfidPresent && uid==rfidCurrentUid;
    if(!same){
      int match=rfidAuthenticatePresented(uid);
      if(match>=0){
        rfidPresent=true;
        rfidSince=millis();
        rfidCurrentUid=uid;
        rfidLastSeen=millis();
      }else{
        rfidPresent=false;
        rfidCurrentUid="";
      }
    }else{
      rfidLastSeen=millis();
    }
    rfidFinishCard();
  }
  // The card must be continuously detectable; tolerate brief reader misses.
  if(rfidPresent && millis()-rfidLastSeen>650){
    rfidPresent=false;
    rfidCurrentUid="";
    if(rfidStage==1)rfidStage=2;
  }
  if(!rfidPresent)return;
  unsigned long held=millis()-rfidSince;
  if(rfidStage==0 && held>=2000){
    unlockCar();
    setDoors(false);
    rfidStage=1;
  }else if(rfidStage==2 && held>5000){
    digitalWrite(RFID_TRIGGER_OUT,HIGH);
    rfidStage=3;
    rfidDoneHighSeen=false;
    rfidPresent=false;
    rfidCurrentUid="";
  }
}


void handleGetSim800Status();
void handleSetSim800Config();
void handleDetectAPN();

// GPS
void setupGPS();
void updateGPS();
String gpsMapUrl();
String gpsStatusJson();
bool  gpsAvailable();
double haversineMeters(double lat1, double lng1, double lat2, double lng2);
void   updateSafeZone();
void   triggerSafeZoneAlarm(const String& reason);
void   updateSafeZoneCalls();

// GPS config & web handlers
void loadGpsConfig();
void saveGpsConfig();
void handleSetGpsConfig();
void handleGetGpsStatus();

// ===================== GPS & Safe Zone Config =====================
void loadGpsConfig() {
  preferences.begin("sim800", true);
  // [CHANGE] چک‌باکس Enable GPS حذف شد - GPS همیشه فعال است
  gpsEnabled  = true;
  gpsSite     = preferences.getInt("gSite", 0);
  safeZoneEnabled = preferences.getBool("szEn", false);
  safeZoneLat = preferences.getString("szLat", "0").toDouble();
  safeZoneLng = preferences.getString("szLng", "0").toDouble();
  safeZoneRadiusM = preferences.getUInt("szRad", 100);
  gpsAntiCutEnabled = preferences.getBool("gpsAC", false);
  preferences.end();
  if (gpsSite < 0 || gpsSite > 2) gpsSite = 0;
  if (safeZoneRadiusM < 1 || safeZoneRadiusM > 100000) safeZoneRadiusM = 100;
  if (safeZoneEnabled && safeZoneLat == 0 && safeZoneLng == 0) safeZoneEnabled = false;
}

void saveGpsConfig() {
  preferences.begin("sim800", false);
  preferences.putBool("gEn", gpsEnabled);
  preferences.putInt("gSite", gpsSite);
  preferences.putBool("szEn", safeZoneEnabled);
  preferences.putString("szLat", String(safeZoneLat, 6));
  preferences.putString("szLng", String(safeZoneLng, 6));
  preferences.putUInt("szRad", safeZoneRadiusM);
  preferences.putBool("gpsAC", gpsAntiCutEnabled);
  preferences.end();
}

void handleGetGpsStatus() {
  if (!isRequestAuthorized()) { server.send(403, "application/json", "{\"success\":false}"); return; }
  server.send(200, "application/json", gpsStatusJson());
}

void handleSetGpsConfig() {
  if (!isRequestAuthorized()) { server.send(403, "application/json", "{\"success\":false}"); return; }
  bool changed = false;

  // [CHANGE] چک‌باکس Enable GPS حذف شد - GPS همیشه فعال و غیرقابل غیرفعال‌سازی است
  gpsEnabled = true;
  if (server.hasArg("site")) {
    int s = server.arg("site").toInt();
    if (s >= 0 && s <= 2 && s != gpsSite) { gpsSite = s; changed = true; }
  }
  // Safe zone enable: no longer blocked by 0 coords; runtime handles it
  if (server.hasArg("safeEnabled")) {
    bool en = (server.arg("safeEnabled") == "1");
    if (en != safeZoneEnabled) {
      safeZoneEnabled = en;
      changed = true;
      if (!en) {
        safeZoneAlarmActive = false;
        safeZoneCallIdx = 0;
        safeZoneNoFixSince = 0;
        safeZoneCallAnswered = false;
      }
    }
  }
  if (server.hasArg("safeLat")) {
    double v = server.arg("safeLat").toDouble();
    if (v >= -90 && v <= 90 && v != safeZoneLat) { safeZoneLat = v; changed = true; }
  }
  if (server.hasArg("safeLng")) {
    double v = server.arg("safeLng").toDouble();
    if (v >= -180 && v <= 180 && v != safeZoneLng) { safeZoneLng = v; changed = true; }
  }
  if (server.hasArg("safeRadius")) {
    long v = server.arg("safeRadius").toInt();
    if (v >= 1 && v <= 100000 && (uint32_t)v != safeZoneRadiusM) { safeZoneRadiusM = (uint32_t)v; changed = true; }
  }
    if (server.hasArg("gpsAntiCut")) {
    bool en = (server.arg("gpsAntiCut") == "1");
    if (en != gpsAntiCutEnabled) { gpsAntiCutEnabled = en; changed = true; }
  }
  if (changed) saveGpsConfig();

  // Optional warning if safe zone enabled but coords unset
  String extra = "";
  if (safeZoneEnabled && safeZoneLat == 0 && safeZoneLng == 0) {
    extra = ",\"warning\":\"Safe zone enabled but coordinates not set - alarm will not trigger until set\"";
  }
  server.send(200, "application/json", "{\"success\":true" + extra + "}");
}

// Safe Zone prototypes (must be before setup/loop)
double haversineMeters(double lat1, double lng1, double lat2, double lng2);
void   updateSafeZone();
void   triggerSafeZoneAlarm(const String& reason);
void   updateSafeZoneCalls();
void   checkSafeZoneCallStatus();   // جدید

// ===================== Setup =====================
void setup() {
  // خروجی‌ها را قبل از هر چیز در حالت امن LOW قرار بده (جلوگیری از گلیچ)
  for (int i = 0; i < OUTPUT_COUNT; i++) {
    digitalWrite(outputPins[i], LOW);
    pinMode(outputPins[i], OUTPUT);
    digitalWrite(outputPins[i], LOW);
  }

  digitalWrite(PIN_BUZZER, BUZZER_OFF_LEVEL);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, BUZZER_OFF_LEVEL);

  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Car Controller (ESP32-S3) Starting ===");

  initPrefsNamespaces();
  initPulses();

  loadAllSettings();
  loadDevices();
  loadCustomNames();
  loadSim800Config();
  loadMqttConfig();
  loadGpsConfig();
  loadLockState();

  for (int i = 0; i < OUTPUT_COUNT; i++) {
    if (i == 4) continue;
    if (i == 1) { desiredState[i] = false; continue; }
    desiredState[i] = outputState[i];
  }

  rfidSetup();
  setupWiFiAP();
  // No captive DNS or captive HTML portal.
  sclLoadToken();
  setupWebServer();
  setupSim800();
  setupGPS();
  // OTA disabled in mobile API build; legacy shared password was insecure.
  Serial.print("AP IP: ");   Serial.println(WiFi.softAPIP());

  Serial.print("AP IP: ");   Serial.println(WiFi.softAPIP());
  Serial.print("SSID: ");    Serial.println(ap_ssid);
  Serial.print("Language: "); Serial.println(systemLanguage);
  Serial.print("Signal Threshold: "); Serial.println(String(signalThreshold) + "%");
  Serial.print("Weak Auto Disconnect: "); Serial.println(weakAutoDisconnectEnabled ? "ON" : "OFF");
  Serial.println("Mobile JSON API active");

  // [PATCH-S] راهنمای کنسول سریال
  Serial.println("Serial console ready; Location command disabled");
  Serial.println("Serial console: Location/Stop location unavailable; other commands configured");
}

void loop() {
  // OTA disabled for security.

  handleSerialCommands();           // [PATCH-S] خواندن فرمان‌های Serial Monitor

  // DNS portal disabled.
  server.handleClient();

  handlePulses();
  rfidTick();
  updateSim800();
  mqttTick();
  updateGPS();
  updateSafeZone();
  updateSafeZoneCalls();
  updateSecurity();
  updateBuzzer();

  // ← بازه سیگنال: ۱۰ ثانیه اگر auto فعال، ۳۰ ثانیه اگر غیرفعال
  unsigned long wifiInterval = weakAutoDisconnectEnabled ? 10000UL : 30000UL;
  if (millis() - lastWeakCheck >= wifiInterval) {
    lastWeakCheck = millis();
    handleWeakSignalProtection();
  }

  // ← بررسی اتصال کاربر مجاز به AP
  if (millis() - lastWifiCheck >= 2000UL) {
    lastWifiCheck = millis();
    checkWifiConnection();
  }

  delay(1);
}

// ساخت namespace ها (در اولین بوت، begin(readOnly) روی namespace ناموجود خطا می‌دهد)
void initPrefsNamespaces() {
  const char* spaces[] = {"carctrl", "devices", "sim800", "mqttcfg"};
  for (int i = 0; i < 4; i++) {
    preferences.begin(spaces[i], false);
    preferences.end();
  }
}

// ===================== OTA =====================
void setupOTA() {
  MDNS.begin(OTA_HOSTNAME);
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(ACTIVATION_CODE);
  ArduinoOTA.setPort(3232);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA: start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA: end");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready (credentials redacted)");
}

// ===================== SMS send (non-blocking-safe) =====================
bool sendSms(const String& to, const String& text) {
  if (!sim800Ready || locPhase != 0 || mqttWaiting) {
    Serial.println("SMS: skipped (ready=" + String(sim800Ready?"Y":"N") +
                   " locPhase=" + String(locPhase) + ")");
    return false;
  }

  sim800RxBuf = "";
  g_smsOk     = false;
  g_smsFail   = false;

  sim800Serial.print("AT+CMGS=\"");
  sim800Serial.print(to);
  sim800Serial.print("\"\r");

  // ===== انتظار برای prompt > با تشخیص خطا =====
  unsigned long st = millis();
  bool promptFound = false;
  bool cmdRejected = false;
  while (millis() - st < 5000) {
    // ابتدا بدون پردازش newline، مستقیم از Serial بخوان (برای جلوگیری از merge با URC)
    while (sim800Serial.available()) {
      char c = (char)sim800Serial.read();
      if (c == '>') {
        promptFound = true;
        // Never discard pending bytes: other lines may be URCs (including MQTT).
        // Remaining bytes are parsed by sim800PumpRx after SMS submission.
        break;
      }
      sim800RxBuf += c;
      if (sim800RxBuf.indexOf("ERROR") >= 0 || sim800RxBuf.indexOf("+CMS") >= 0) {
        Serial.println("SMS: AT+CMGS rejected (response redacted)");
        sim800RxBuf = "";
        cmdRejected = true;
        break;
      }
      if (sim800RxBuf.length() > 256) sim800RxBuf = "";
    }
    if (promptFound || cmdRejected) break;
    delay(10);
  }

  if (sim800RxBuf.length()) {
    String remaining=sim800RxBuf; sim800RxBuf="";
    sim800CleanAndQueue(remaining); // preserve URCs seen before the CMGS prompt
  }
  if (cmdRejected) return false;
  if (!promptFound) {
    Serial.println("SMS: no prompt");
    return false;
  }

  // ===== ارسال متن + Ctrl+Z =====
  sim800Serial.print(text);
  sim800Serial.write(26);

  // ===== انتظار برای +CMGS یا +CMS ERROR =====
  st = millis();
  while (millis() - st < 15000) {   // ۱۵ ثانیه کافی است
    sim800PumpRx();
    if (g_smsOk)   { g_smsOk   = false; Serial.println("SMS sent"); return true;  }
    if (g_smsFail) { g_smsFail = false; Serial.println("SMS failed (+CMS ERROR)"); return false; }
    delay(10);
  }
  Serial.println("SMS: response timeout");
  return false;
}

// ===================== WiFi presence & reset =====================
void resetAllSecurity() {
  bool hadAlarm = safeZoneAlarmActive || alarmManual || secLocked || carLocked;

  // ----- Safe Zone alarm -----
  safeZoneAlarmActive  = false;
  safeZoneCallIdx      = 0;
  safeZoneCallAnswered = false;
  safeZoneViolated     = false;
  safeZoneNoFixSince   = 0;
  gpsNoFixSince        = 0;

  // ----- Manual alarm -----
  alarmManual = false;

  // ----- Security runtime -----
  secState      = 0;
  gsmBadSince   = 0;
  secGraceStart = 0;
  secLocked     = false;

  // ===== مورد 4 و 6: آزادسازی کامل خودرو =====
  carLocked = false;
  setOutput(2, true);  desiredState[2] = true;   // پمپ بنزین ON
  setOutput(3, true);  desiredState[3] = true;   // سوئیچ خودرو ON
  if (outputState[0]) setDoors(false);           // فقط اگر درب‌ها واقعاً قفل بودند، پالس باز شدن

  // ----- Pending alarm SMS -----
  pendingAlarmSms       = false;
  pendingAlarmSmsReason = "";

  // Keep periodic GPS SMS reporting active across unlock operations.

  // ----- End active call -----
  if (callActive && locPhase == 0) {
    sim800SendAT("ATH", "OK", 1500);
    callActive   = false;
    activeCaller = "";
  }

  saveLockState();
  updateBuzzer();

  if (hadAlarm) Serial.println("WiFi: all alarms cleared, car fully unlocked (fuel+switch ON, doors open)");
}

void checkWifiConnection() {
  static unsigned long firstConnectedAt = 0;

  bool nowConnected = authorizedClientConnected();

  // [PATCH 6] قبلاً این خط در هر چرخه‌ی ۲ ثانیه‌ای اجرا می‌شد و firstConnectedAt
  // دائماً Refresh می‌شد؛ در نتیجه شرط ۳ ثانیه هیچ‌وقت true نمی‌شد و reset اجرا نمی‌شد.
  if (nowConnected && !wifiUserWasConnected && firstConnectedAt == 0) {
    firstConnectedAt = millis();       // شروع پنجره تثبیت (فقط یک‌بار)
  }

  // فقط اگر حداقل ۳ ثانیه پایدار وصل مانده باشد، reset کن
  if (nowConnected && !wifiUserWasConnected && firstConnectedAt != 0 &&
      millis() - firstConnectedAt >= 3000UL) {
    Serial.println("WiFi: authorized device connected (stable) -> reset alarms");
    resetAllSecurity();
    wifiUserWasConnected = true;
    firstConnectedAt = 0;
  }

  if (!nowConnected) {
    if (wifiUserWasConnected) Serial.println("WiFi: authorized device disconnected");
    wifiUserWasConnected = false;
    firstConnectedAt = 0;
  }
}
// ===================== GPS =====================
void setupGPS() {
  if (!gpsEnabled) return;
  if (gpsSerialStarted) { gpsFirstSearchMs = millis(); return; }
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gpsSerialStarted = true;
  gpsFirstSearchMs = millis();
  Serial.println("GPS: serial started on pins RX=" + String(GPS_RX_PIN) + " TX=" + String(GPS_TX_PIN));
}

void updateGPS() {
  if (!gpsEnabled) return;
  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());

  if (gps.location.isValid() && gps.location.age() < 5000) {
    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
    gpsLastFixMs = millis();
    gpsFix = true;
    gpsSatellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
    gpsHdop = gps.hdop.isValid() ? gps.hdop.hdop() : 0;
  } else if (gpsFix && millis() - gpsLastFixMs > GPS_STALE_MS) {
    gpsFix = false;
    Serial.println("GPS: fix stale");
  }
}

bool gpsAvailable() { return gpsEnabled && gpsFix; }
double haversineMeters(double lat1, double lng1, double lat2, double lng2);
void   updateSafeZone();
void   triggerSafeZoneAlarm(const String& reason);
void   updateSafeZoneCalls();

String gpsMapUrl() {
  if (!gpsFix) return "";
  String lat = String(gpsLat, 6);
  String lng = String(gpsLng, 6);
  if (gpsSite == 0) return "https://maps.google.com/?q=" + lat + "," + lng;
  if (gpsSite == 1) return "https://neshan.org/maps/@" + lat + "," + lng + ",16z";
  return "https://balad.ir/location?latitude=" + lat + "&longitude=" + lng;
}

String gpsStatusJson() {
  String j = "{";
  j += "\"enabled\":"       + String(gpsEnabled ? "true" : "false") + ",";
  j += "\"fix\":"           + String(gpsFix ? "true" : "false") + ",";
  j += "\"lat\":"           + String(gpsLat, 6) + ",";
  j += "\"lng\":"           + String(gpsLng, 6) + ",";
  j += "\"sats\":"          + String(gpsSatellites) + ",";
  j += "\"hdop\":"          + String(gpsHdop, 2) + ",";
  j += "\"site\":"          + String(gpsSite) + ",";
  j += "\"gpsAntiCut\":" + String(gpsAntiCutEnabled ? "true" : "false") + ",";
  j += "\"url\":\""         + jsonEscape(gpsMapUrl()) + "\",";
  j += "\"safeZoneEnabled\":"+ String(safeZoneEnabled ? "true" : "false") + ",";
  j += "\"safeZoneLat\":"   + String(safeZoneLat, 6) + ",";
  j += "\"safeZoneLng\":"   + String(safeZoneLng, 6) + ",";
  j += "\"safeZoneRadius\":"+ String(safeZoneRadiusM) + ",";
  j += "\"safeZoneViolated\":"+ String(safeZoneViolated ? "true" : "false") + ",";
  j += "\"safeZoneAlarm\":" + String(safeZoneAlarmActive ? "true" : "false");
  if (gpsEnabled && !gpsFix) {
    unsigned long elapsed = millis() - gpsFirstSearchMs;
    j += ",\"searching\":" + String(elapsed < GPS_FIX_TIMEOUT_MS ? "true" : "false");
  }
  j += "}";
  return j;
}

// ===================== Safe Zone =====================
double haversineMeters(double lat1, double lng1, double lat2, double lng2) {
  const double R = 6371000.0;
  double phi1 = lat1 * DEG_TO_RAD;
  double phi2 = lat2 * DEG_TO_RAD;
  double dphi = (lat2 - lat1) * DEG_TO_RAD;
  double dlng = (lng2 - lng1) * DEG_TO_RAD;
  double a = sin(dphi/2) * sin(dphi/2) +
             cos(phi1) * cos(phi2) * sin(dlng/2) * sin(dlng/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

void updateSafeZone() {
  // ===================== GPS Anti-Cut (مستقل از Safe Zone) =====================
  static unsigned long lastAc = 0;
  if (gpsEnabled && gpsAntiCutEnabled && (millis() - lastAc > 5000UL)) {
    lastAc = millis();
    // «تازه» = همین الان fix معتبر داریم (نه ۲ دقیقه پس از قطع)
    bool freshFix = gps.location.isValid() && gps.location.age() < 5000;

    if (!freshFix) {
      if (gpsNoFixSince == 0) {
        gpsNoFixSince = millis();
        Serial.println("GPS-AC: no fresh fix - timer started");
      } else if (millis() - gpsNoFixSince > GPS_ANTICUT_TIMEOUT_MS && !safeZoneAlarmActive) {
        triggerSafeZoneAlarm("GPS antenna cut (no fix for " +
                             String(GPS_ANTICUT_TIMEOUT_MS / 1000UL) + "s)");
      }
    } else {
      if (gpsNoFixSince != 0) Serial.println("GPS-AC: fresh fix restored - timer reset");
      gpsNoFixSince = 0;
    }
  }

  // ===================== Safe Zone =====================
  // اگر GPS کلاً خاموش شد، آلارم را پاک کن
  if (!gpsEnabled) {
    if (safeZoneAlarmActive) {
      Serial.println("Safe zone: cleared (GPS disabled)");
      safeZoneAlarmActive  = false;
      safeZoneCallIdx      = 0;
      safeZoneCallAnswered = false;
    }
    safeZoneViolated   = false;
    safeZoneNoFixSince = 0;
    gpsNoFixSince      = 0;
    return;
  }

  // ←←← اصلاح حیاتی: اگر Safe Zone خاموش است، آلارم Anti-Cut را پاک نکن
  if (!safeZoneEnabled) {
    safeZoneViolated   = false;
    safeZoneNoFixSince = 0;
    return;
  }

  if (millis() - safeZoneLastCheck < SAFE_ZONE_CHECK_MS) return;
  safeZoneLastCheck = millis();

  if (!gpsFix) {
    if (safeZoneNoFixSince == 0) safeZoneNoFixSince = millis();
    else if (millis() - safeZoneNoFixSince > SAFE_ZONE_NO_FIX_TIMEOUT_MS && !safeZoneAlarmActive) {
      triggerSafeZoneAlarm("GPS signal lost for " + String(SAFE_ZONE_NO_FIX_TIMEOUT_MS / 1000) + "s");
    }
    return;
  }
  safeZoneNoFixSince = 0;

  if (safeZoneLat == 0 && safeZoneLng == 0) return;

  double dist = haversineMeters(gpsLat, gpsLng, safeZoneLat, safeZoneLng);
  if (dist > safeZoneRadiusM) {
    safeZoneViolated = true;
    if (!safeZoneAlarmActive) {
      triggerSafeZoneAlarm("Left safe zone (" + String((int)dist) + "m from center)");
    }
  } else {
    safeZoneViolated = false;
  }
}

void triggerSafeZoneAlarm(const String& reason) {
  if (safeZoneAlarmActive) return;
  safeZoneAlarmActive  = true;
  safeZoneCallIdx      = 0;
  safeZoneCallStart    = 0;
  safeZoneCallAnswered = false;
  safeZoneCallLastPoll = 0;

  Serial.println("!!! SAFE ZONE ALARM: " + reason);

  if (callActive) {
    sim800SendAT("ATH", "OK", 1500);
    callActive   = false;
    activeCaller = "";
  }

  // [OPT-13] آلارم از مکان‌یابی LBS مهم‌تر است: LBS لغو شود تا SMS/تماس آلارم بدون تأخیر اجرا شود
  locPhase = 0; locWanted = false; locDone = false; locResult = ""; locRequester = "";
  locFastRetry = false;

  applyLock(false, false);
  alarmManual = true;
  updateBuzzer();

  pendingAlarmSms       = true;
  pendingAlarmSmsReason = reason;

}

// =====================================================================
//    Safe Zone Calls (v2): 30s per number, AT+CLCC detection,
//    hang up previous call on alarm, stuck-detection.
// =====================================================================
void checkSafeZoneCallStatus() {
  if (locPhase != 0) return;
  if (!sim800Ready) return;

  String r = sim800Exchange("AT+CLCC", 1500);

  int idx = r.indexOf("+CLCC:");
  if (idx < 0) {
    if (callActive) {
      Serial.println("SAFE-ZONE: call ended (CLCC shows no active call)");
      callActive = false;
      activeCaller = "";
      safeZoneCallStart = millis();
    }
    return;
  }

  // +CLCC: <idx>,<dir>,<stat>,<mode>,<mpty>,"<number>",<type>
  // stat: 0=active(answer)  1=held  2=dialing  3=alerting  4=incoming  5=waiting
  int c1 = r.indexOf(',', idx);
  int c2 = (c1 > 0) ? r.indexOf(',', c1 + 1) : -1;
  int c3 = (c2 > 0) ? r.indexOf(',', c2 + 1) : -1;
  if (c2 > 0 && c3 > 0) {
    int stat = r.substring(c2 + 1, c3).toInt();
    if (stat == 0 && !safeZoneCallAnswered) {
      Serial.println("SAFE-ZONE: call answered; holding until user hangs up");
      safeZoneCallAnswered = true;
      safeZoneCallIdx = MAX_PHONE_NUMBERS;   // no more numbers
    }
  }
}

void updateSafeZoneCalls() {
  if (mqttWaiting) return;  
  if (!safeZoneAlarmActive) return;
  if (!sim800Enabled) {
    static bool lg1 = false;
    if (!lg1) { lg1 = true; Serial.println("SAFE-ZONE: SIM800 DISABLED -> cannot call"); }
    return;
  }
  if (!sim800Ready) {
    static bool lg2 = false;
    if (!lg2) { lg2 = true; Serial.println("SAFE-ZONE: SIM800 NOT READY -> waiting"); }
    return;
  }
  if (locPhase != 0) return;   // don't interrupt LBS state machine

  unsigned long now = millis();

  // --- active call handling ---
  if (callActive) {
    if (safeZoneCallAnswered) return;   // answered - hold until user hangs up

    // stuck detection: callActive never resolved
    if (now - safeZoneCallStart > (SAFE_ZONE_CALL_TIMEOUT_MS + 60000UL)) {
      Serial.println("SAFE-ZONE: callActive stuck >90s -> forcing reset");
      sim800SendAT("ATH", "OK", 1500);
      callActive = false;
      activeCaller = "";
      safeZoneCallStart = now;
      return;
    }

    // poll call status every 3s
    if (now - safeZoneCallLastPoll >= SAFE_ZONE_CALL_POLL_MS) {
      safeZoneCallLastPoll = now;
      checkSafeZoneCallStatus();
    }
    if (safeZoneCallAnswered) return;
    if (!callActive) return;   // call ended between polls

    // 30s timeout -> hangup, next number
    if (now - callStart >= SAFE_ZONE_CALL_TIMEOUT_MS) {
      Serial.println("SAFE-ZONE: 30s no answer -> hanging up, next number");
      sim800SendAT("ATH", "OK", 2000);
      callActive = false;
      activeCaller = "";
      safeZoneCallStart = now;
    }
    return;
  }

  // --- all numbers tried -> wait for user ---
  if (safeZoneCallIdx >= MAX_PHONE_NUMBERS) return;

  // --- gap between calls ---
  if (safeZoneCallStart > 0 && now - safeZoneCallStart < SAFE_ZONE_CALL_GAP_MS) return;

  // --- skip empty numbers ---
  while (safeZoneCallIdx < MAX_PHONE_NUMBERS && phoneNumbers[safeZoneCallIdx].length() < 7) {
    safeZoneCallIdx++;
  }
  if (safeZoneCallIdx >= MAX_PHONE_NUMBERS) {
    static bool lg3 = false;
    if (!lg3) { lg3 = true; Serial.println("SAFE-ZONE: no valid phone numbers -> waiting for user (SMS/WiFi)"); }
    return;
  }

  // --- dial ---
  // قبل از شماره‌گیری: اگر SMS آلارم هنوز ارسال نشده، حالا بفرست
if (pendingAlarmSms && sim800Ready && locPhase == 0) {
  pendingAlarmSms = false;
  String msg = "ALARM: " + pendingAlarmSmsReason;
  if (gpsAvailable()) msg += "\nLocation: " + gpsMapUrl();
  for (int i = 0; i < MAX_PHONE_NUMBERS; i++) {
    if (phoneNumbers[i].length() >= 7) sendSms(phoneNumbers[i], msg);
  }
  Serial.println("SAFE-ZONE: alarm SMS sent to all authorized numbers before calls");
}

// --- dial ---
String num = phoneNumbers[safeZoneCallIdx];
  Serial.println("SAFE-ZONE: dialing authorized contact (number redacted)");

  sim800Serial.print("ATD");
  sim800Serial.print(num);
  sim800Serial.print(";\r");
  sim800Serial.flush();

  callActive            = true;
  safeZoneCallAnswered  = false;
  safeZoneCallLastPoll  = now;
  callStart             = now;
  activeCaller          = num;
  safeZoneCallIdx++;
  safeZoneCallStart     = now;
}
// ===================== Pulse Management =====================
void initPulses() {
  for (int i = 0; i < MAX_PULSES; i++) {
    pulses[i].active = false;
    pulses[i].pin = -1;
    pulses[i].start = 0;
    pulses[i].duration = 0;
    pulses[i].resetOutputIndex = -1;
  }
}

void startPulse(int pin, unsigned long duration, int resetOutputIndex) {
  cancelPulse(pin);

  for (int i = 0; i < MAX_PULSES; i++) {
    if (!pulses[i].active) {
      pulses[i].active = true;
      pulses[i].pin = pin;
      pulses[i].start = millis();
      pulses[i].duration = duration;
      pulses[i].resetOutputIndex = resetOutputIndex;
      digitalWrite(pin, HIGH);
      return;
    }
  }
}

void cancelPulse(int pin) {
  for (int i = 0; i < MAX_PULSES; i++) {
    if (pulses[i].active && pulses[i].pin == pin) {
      digitalWrite(pin, LOW);
      pulses[i].active = false;
      pulses[i].pin = -1;
      pulses[i].duration = 0;
      pulses[i].resetOutputIndex = -1;
    }
  }
}

void handlePulses() {
  unsigned long now = millis();

  for (int i = 0; i < MAX_PULSES; i++) {
    if (pulses[i].active && (now - pulses[i].start >= pulses[i].duration)) {
      if (pulses[i].pin >= 0) digitalWrite(pulses[i].pin, LOW);

      if (pulses[i].resetOutputIndex >= 0 && pulses[i].resetOutputIndex < OUTPUT_COUNT) {
        outputState[pulses[i].resetOutputIndex] = false;
        desiredState[pulses[i].resetOutputIndex] = false;
      }

      pulses[i].active = false;
      pulses[i].pin = -1;
      pulses[i].duration = 0;
      pulses[i].resetOutputIndex = -1;
    }
  }
}

// قفل/باز کردن درها (مشترک بین وب و SMS)
void setDoors(bool lock) {
  outputState[0] = lock;
  desiredState[0] = lock;

  cancelPulse(PIN_DOOR_DRIVER);
  cancelPulse(PIN_DOORS_INVERSE);
  digitalWrite(PIN_DOOR_DRIVER, LOW);
  digitalWrite(PIN_DOORS_INVERSE, LOW);

  startPulse(lock ? PIN_DOOR_DRIVER : PIN_DOORS_INVERSE, DOOR_PULSE_MS, -1);
}

// پالس صندوق عقب (مشترک بین وب و SMS)
void triggerTrunk() {
  outputState[1] = true;
  desiredState[1] = false;
  startPulse(PIN_TRUNK, TRUNK_PULSE_MS, 1);
}

// ===================== Weak Signal Protection =====================
void saveWeakAutoSetting() {
  preferences.begin("carctrl", false);
  preferences.putBool("weakAuto", weakAutoDisconnectEnabled);
  preferences.end();
}

void handleSetWeakAutoDisconnect() {
  if (!isRequestAuthorized()) {
    server.send(403, "application/json", "{\"success\":false,\"message\":\"Unauthorized\"}");
    return;
  }

  if (!server.hasArg("enabled")) {
    server.send(400, "application/json", "{\"success\":false,\"message\":\"Missing enabled\"}");
    return;
  }

  bool enabled = (server.arg("enabled") == "1" || server.arg("enabled") == "true");

  if (enabled != weakAutoDisconnectEnabled) {
    weakAutoDisconnectEnabled = enabled;
    saveWeakAutoSetting();

    if (weakAutoDisconnectEnabled) {
      for (int i = 0; i < OUTPUT_COUNT; i++) {
        if (i == 4) continue;
        if (i == 1) { desiredState[i] = false; continue; }
        desiredState[i] = outputState[i];
      }
      weakDisconnectActive = false;
      Serial.println("Weak signal auto disconnect enabled.");
    } else {
      weakDisconnectActive = false;
      Serial.println("Weak signal auto disconnect disabled.");
    }
  }

  String json = "{\"success\":true,\"weakAuto\":" + String(weakAutoDisconnectEnabled ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void handleWeakSignalProtection() {
  if (!weakAutoDisconnectEnabled) {
    weakDisconnectActive = false;
    return;
  }

  bool clientConnected = (WiFi.softAPgetStationNum() > 0);
  int percent = 0;

  if (clientConnected) {
    int rssi = getClientRSSI();
    percent = rssiToPercentage(rssi);
  }

  if (clientConnected && percent >= signalThreshold) {
    if (weakDisconnectActive) {
      Serial.println("Signal recovered - restoring relays");

      for (int i = 0; i < OUTPUT_COUNT; i++) {
        if (i == 4) continue;
        if (i == 1) { desiredState[i] = false; continue; }
        if (outputState[i] != desiredState[i]) setOutput(i, desiredState[i]);
      }

      weakDisconnectActive = false;
    }
  } else {
    if (!weakDisconnectActive) {
      Serial.println("Weak/no signal - cutting active relays");

      for (int i = 0; i < OUTPUT_COUNT; i++) {
        if (i == 4) continue;
        if (i == 1) { desiredState[i] = false; continue; }
        desiredState[i] = outputState[i];
      }

      for (int i = 0; i < OUTPUT_COUNT; i++) {
        if (i == 4) continue;
        if (outputState[i]) setOutput(i, LOW);
      }

      weakDisconnectActive = true;
    } else {
      for (int i = 0; i < OUTPUT_COUNT; i++) {
        if (i == 4) continue;
        if (outputState[i]) setOutput(i, LOW);
      }
    }
  }
}

// ===================== Device Authorization Functions =====================
void loadDevices() {
  preferences.begin("devices", true);

  allowedDevicesCount = preferences.getInt("count", 0);
  if (allowedDevicesCount > MAX_ALLOWED_DEVICES) allowedDevicesCount = MAX_ALLOWED_DEVICES;
  if (allowedDevicesCount < 0) allowedDevicesCount = 0;

  for (int i = 0; i < allowedDevicesCount; i++) {
    String macKey = "mac" + String(i);
    String ipKey = "ip" + String(i);

    allowedMACs[i] = preferences.getString(macKey.c_str(), "");
    allowedIPs[i] = preferences.getString(ipKey.c_str(), "");
    allowedMACs[i].toUpperCase();
  }

  preferences.end();
  Serial.println("Authorized devices loaded: " + String(allowedDevicesCount));
}

void saveDevices() {
  preferences.begin("devices", false);
  preferences.putInt("count", allowedDevicesCount);

  for (int i = 0; i < allowedDevicesCount; i++) {
    String macKey = "mac" + String(i);
    String ipKey = "ip" + String(i);
    preferences.putString(macKey.c_str(), allowedMACs[i]);
    preferences.putString(ipKey.c_str(), allowedIPs[i]);
  }

  preferences.end();
}

int findDeviceByMAC(String mac) {
  mac.trim();
  mac.toUpperCase();
  if (mac.length() == 0) return -1;

  for (int i = 0; i < allowedDevicesCount; i++) {
    if (allowedMACs[i] == mac) return i;
  }
  return -1;
}

void addOrUpdateDevice(String mac, String ip) {
  mac.trim();
  mac.toUpperCase();
  ip.trim();

  if (mac.length() == 0 || ip.length() == 0) return;

  int idx = findDeviceByMAC(mac);

  if (idx >= 0) {
    if (allowedIPs[idx] != ip) {
      allowedIPs[idx] = ip;
      saveDevices();
      Serial.println("Authorized device IP updated: " + mac + " -> " + ip);
    }
    return;
  }

  if (allowedDevicesCount >= MAX_ALLOWED_DEVICES) {
    for (int i = 0; i < MAX_ALLOWED_DEVICES - 1; i++) {
      allowedMACs[i] = allowedMACs[i + 1];
      allowedIPs[i] = allowedIPs[i + 1];
    }
    allowedDevicesCount = MAX_ALLOWED_DEVICES - 1;
  }

  allowedMACs[allowedDevicesCount] = mac;
  allowedIPs[allowedDevicesCount] = ip;
  allowedDevicesCount++;

  saveDevices();
  Serial.println("New device authorized: " + mac + " / " + ip);
}

String getMacAddressString(const uint8_t* mac) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

// یافتن MAC کلاینت از روی IP:
//  1) جدول ARP لایه lwIP (روی core 2.x و 3.x کار می‌کند و چند کلاینت را پشتیبانی می‌کند)
//  2) در صورت شکست، اگر فقط یک کلاینت وصل است، MAC همان
String getMacByIP(IPAddress ip) {
  // ← چک کش
  unsigned long now = millis();
  for (int i = 0; i < AP_MAX_CLIENTS; i++) {
    if (macCache[i].mac.length() > 0 && macCache[i].ip == ip &&
        now - macCache[i].ts < MAC_CACHE_TTL_MS) {
      return macCache[i].mac;
    }
  }
  esp_netif_t* apNetif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  if (apNetif) {
    struct netif* n = (struct netif*)esp_netif_get_netif_impl(apNetif);
    if (n) {
      ip4_addr_t a;
      IP4_ADDR(&a, ip[0], ip[1], ip[2], ip[3]);
      struct eth_addr* eth = NULL;
      const ip4_addr_t* ipRet = NULL;
#ifdef LOCK_TCPIP_CORE
      LOCK_TCPIP_CORE();
#endif
      s8_t r = etharp_find_addr(n, &a, &eth, &ipRet);
      String result = "";
      if (r >= 0 && eth) result = getMacAddressString(eth->addr);
#ifdef UNLOCK_TCPIP_CORE
      UNLOCK_TCPIP_CORE();
#endif
      if (result.length() > 0) {
    macCache[macCacheIdx].ip  = ip;
    macCache[macCacheIdx].mac = result;
    macCache[macCacheIdx].ts  = now;
    macCacheIdx = (macCacheIdx + 1) % AP_MAX_CLIENTS;
  }
  return result;
    }
  }

  wifi_sta_list_t wifi_sta_list;
  if (esp_wifi_ap_get_sta_list(&wifi_sta_list) == ESP_OK) {
    if (wifi_sta_list.num == 1) {
      return getMacAddressString(wifi_sta_list.sta[0].mac);
    }
  }

  return "";
}


// ========= Mobile-only local API (no HTML pages) =========
// Each vehicle has ONE installation secret, shared only with trusted owners.
// Each phone receives its OWN random 256-bit bearer token and chooses its OWN app PIN.
// Set a unique, random secret (at least 12 characters, preferably 24+) BEFORE flashing.
// Do not commit the actual secret to a public repository.
#ifndef SCL_PAIRING_SECRET
#define SCL_PAIRING_SECRET "CHANGE_ME_BEFORE_FLASH"
#endif
static const uint8_t SCL_MAX_PAIRED_PHONES = 5;
String sclTokenHashes[SCL_MAX_PAIRED_PHONES]; // SHA-256 only; never store plaintext tokens.
String sclPhoneLabels[SCL_MAX_PAIRED_PHONES];
unsigned long sclLastAuthorizedAt=0;
unsigned long sclPairFailures=0;
unsigned long sclLastFailed=0;
const char* sclHeaders[]={"X-SCL-Token"};
String sclShaHex(const String& value) {
  unsigned char digest[32];
  mbedtls_sha256(reinterpret_cast<const unsigned char*>(value.c_str()), value.length(), digest, 0);
  char hex[65];
  for (int i=0;i<32;i++) sprintf(hex+2*i, "%02x", digest[i]);
  hex[64]='\0';
  return String(hex);
}
bool sclConstantTimeEqual(const String& x,const String& y){
  if (x.length()!=y.length()) return false;
  unsigned char diff=0;
  for(size_t i=0;i<x.length();i++) diff |= (unsigned char)(x[i]^y[i]);
  return diff==0;
}
bool sclValidHash(const String& value){
  if(value.length()!=64)return false;
  for(size_t i=0;i<value.length();i++)if(!isxdigit((unsigned char)value[i]))return false;
  return true;
}
uint8_t sclPairedCount(){
  uint8_t n=0;
  for(uint8_t i=0;i<SCL_MAX_PAIRED_PHONES;i++)if(sclValidHash(sclTokenHashes[i]))n++;
  return n;
}
int sclFreePhoneSlot(){
  for(uint8_t i=0;i<SCL_MAX_PAIRED_PHONES;i++)if(!sclValidHash(sclTokenHashes[i]))return i;
  return -1;
}
void sclLoadToken(){
  for(uint8_t i=0;i<SCL_MAX_PAIRED_PHONES;i++){
    sclTokenHashes[i]="";
    sclPhoneLabels[i]="";
  }
  Preferences p;
  if(!p.begin("sclauth",false))return;
  for(uint8_t i=0;i<SCL_MAX_PAIRED_PHONES;i++){
    String key="t"+String(i), label="n"+String(i);
    String hash=p.getString(key.c_str(),"");
    if(sclValidHash(hash)){
      sclTokenHashes[i]=hash;
      sclPhoneLabels[i]=p.getString(label.c_str(),"Phone "+String(i+1));
    }
  }
  // Backward-compatible, non-destructive migration from previous one-phone version.
  // If NVS write fails, retain legacy hash and do NOT remove the old key.
  String legacy=p.getString("tokenSha","");
  if(sclValidHash(legacy) && !sclValidHash(sclTokenHashes[0])){
    sclTokenHashes[0]=legacy;
    sclPhoneLabels[0]="Original phone";
    if(p.putString("t0",legacy)==64){
      p.putString("n0",sclPhoneLabels[0]);
      p.remove("tokenSha");
    }
  }
  p.end();
}
bool isRequestAuthorized(){
  String token=server.header("X-SCL-Token");
  if(!sclValidHash(token))return false;
  String computed=sclShaHex(token);
  bool match=false;
  // Compare against all valid phone hashes; no plaintext secrets in flash.
  for(uint8_t i=0;i<SCL_MAX_PAIRED_PHONES;i++){
    if(sclValidHash(sclTokenHashes[i])){
      match = sclConstantTimeEqual(computed,sclTokenHashes[i]) || match;
    }
  }
  if(match)sclLastAuthorizedAt=millis();
  return match;
}
void sclReply(int code,const String& data){
  server.sendHeader("Cache-Control","no-store");
  server.sendHeader("X-Content-Type-Options","nosniff");
  server.send(code,"application/json; charset=utf-8",data);
}
bool sclRequireAuth(){
  if(isRequestAuthorized())return true;
  sclReply(401,"{\"ok\":false,\"error\":\"unauthorized\"}");return false;
}
void sclPing(){
  sclReply(200,"{\"api\":1,\"platform\":\"esp32-s3\",\"deviceId\":\""+jsonEscape(mqttConfig.deviceId)+"\",\"paired\":"+
    String(sclPairedCount()>0?"true":"false")+",\"pairedCount\":"+String(sclPairedCount())+
    ",\"pairCapacity\":"+String(SCL_MAX_PAIRED_PHONES)+"}");
}
void sclPair(){
  if(String(SCL_PAIRING_SECRET)=="CHANGE_ME_BEFORE_FLASH" || strlen(SCL_PAIRING_SECRET)<12){
    sclReply(503,"{\"ok\":false,\"error\":\"pairing_not_provisioned\"}");return;
  }
  // Enrollment is NOT limited to the first 10 minutes after boot.
  // Possession of the private installation secret AND local AP access is required.
  if(sclPairFailures>=5 && (uint32_t)(millis()-sclLastFailed)<60000UL){
    sclReply(429,"{\"ok\":false,\"error\":\"try_later\"}");return;
  }
  String code=server.arg("code");
  if(!sclConstantTimeEqual(code,String(SCL_PAIRING_SECRET))){
    sclPairFailures++;sclLastFailed=millis();
    sclReply(403,"{\"ok\":false,\"error\":\"invalid_code\"}");return;
  }
  sclPairFailures=0;
  const int slot=sclFreePhoneSlot();
  if(slot<0){sclReply(409,"{\"ok\":false,\"error\":\"phone_limit_reached\"}");return;}
  String label=server.arg("name");
  label.trim();
  if(label.isEmpty())label="Phone "+String(slot+1);
  if(label.length()>96){sclReply(400,"{\"ok\":false,\"error\":\"invalid_name\"}");return;}
  for(size_t i=0;i<label.length();i++){
    if((unsigned char)label[i]<32){sclReply(400,"{\"ok\":false,\"error\":\"invalid_name\"}");return;}
  }
  char raw[65];
  for(int i=0;i<8;i++)snprintf(raw+8*i,9,"%08lx",(unsigned long)esp_random());
  raw[64]=0;
  String token(raw), hash=sclShaHex(token);
  Preferences p;
  if(!p.begin("sclauth",false)){sclReply(500,"{\"ok\":false,\"error\":\"storage_unavailable\"}");return;}
  const String hashKey="t"+String(slot), nameKey="n"+String(slot);
  // Persist the hash before exposing the raw token to the phone.
  bool saved=p.putString(hashKey.c_str(),hash)==64;
  if(saved)p.putString(nameKey.c_str(),label);
  p.end();
  if(!saved){sclReply(500,"{\"ok\":false,\"error\":\"storage_failed\"}");return;}
  sclTokenHashes[slot]=hash;
  sclPhoneLabels[slot]=label;
  sclReply(200,"{\"ok\":true,\"token\":\""+token+"\",\"slot\":"+String(slot)+
    ",\"pairedCount\":"+String(sclPairedCount())+"}");
}
void sclPhones(){
  if(!sclRequireAuth())return;
  String j="{\"ok\":true,\"capacity\":"+String(SCL_MAX_PAIRED_PHONES)+
    ",\"count\":"+String(sclPairedCount())+",\"phones\":[";
  bool first=true;
  for(uint8_t i=0;i<SCL_MAX_PAIRED_PHONES;i++){
    if(!sclValidHash(sclTokenHashes[i]))continue;
    if(!first)j+=",";
    first=false;
    j+="{\"slot\":"+String(i)+",\"name\":\""+jsonEscape(sclPhoneLabels[i])+"\"}";
  }
  j+="]}";
  sclReply(200,j);
}
void sclRevokePhone(){
  if(!sclRequireAuth())return;
  String index=server.arg("slot");
  if(index.length()!=1 || index[0]<'0' || index[0]>=('0'+SCL_MAX_PAIRED_PHONES)){
    sclReply(400,"{\"ok\":false,\"error\":\"invalid_slot\"}");return;
  }
  const uint8_t slot=(uint8_t)(index[0]-'0');
  if(!sclValidHash(sclTokenHashes[slot])){
    sclReply(404,"{\"ok\":false,\"error\":\"phone_not_found\"}");return;
  }
  Preferences p;
  if(!p.begin("sclauth",false)){sclReply(500,"{\"ok\":false,\"error\":\"storage_unavailable\"}");return;}
  String tokenKey="t"+String(slot), nameKey="n"+String(slot);
  // Prevent an old one-phone key from resurrecting after revoking slot 0.
  if(slot==0 && sclValidHash(p.getString("tokenSha","")) && !p.remove("tokenSha")){
    p.end();
    sclReply(500,"{\"ok\":false,\"error\":\"storage_failed\"}");return;
  }
  bool removed=p.remove(tokenKey.c_str());
  if(removed)p.remove(nameKey.c_str());
  p.end();
  if(!removed){sclReply(500,"{\"ok\":false,\"error\":\"storage_failed\"}");return;}
  sclTokenHashes[slot]="";
  sclPhoneLabels[slot]="";
  sclLastAuthorizedAt=0;
  sclReply(200,"{\"ok\":true,\"pairedCount\":"+String(sclPairedCount())+"}");
}
void sclState(){
  if(!sclRequireAuth())return;
  String j="{\"ok\":true,\"outputs\":[";
  for(int i=0;i<4;i++){if(i)j+=",";j+=outputState[i]?"true":"false";}
  j+="],\"names\":[";
  for(int i=0;i<4;i++){if(i)j+=",";j+="\""+jsonEscape(getOutputName(i))+"\"";}
  j+="],\"weakAuto\":"+String(weakAutoDisconnectEnabled?"true":"false");
  j+=",\"wifiPercent\":"+String(rssiToPercentage(getClientRSSI()));
  j+=",\"gsmPercent\":"+String(gsmPercent);
  j+=",\"internet\":"+String(sim800IP.length()>0?"true":"false");
  j+=",\"gps\":"+gpsStatusJson();
  j+=",\"mqtt\":{\"enabled\":"+String(mqttConfig.requestedEnabled?"true":"false")+",\"state\":\""+jsonEscape(mqttStateText())+"\"}";
  j+=",\"carLocked\":"+String(carLocked?"true":"false")+"}";
  sclReply(200,j);
}
void sclConfig(){
  if(!sclRequireAuth())return;
  String j="{\"ok\":true,\"ap\":{\"ssid\":\""+jsonEscape(ap_ssid)+"\",\"ip\":\""+ap_ip.toString()+"\",\"threshold\":"+String(signalThreshold)+",\"weakAuto\":"+String(weakAutoDisconnectEnabled?"true":"false")+"}";
  j+=",\"gps\":"+gpsStatusJson();
  j+=",\"sim\":{\"apn\":\""+jsonEscape(sim800APN)+"\",\"security\":"+String(securityEnabled?"true":"false")+",\"phones\":[";
  for(int i=0;i<MAX_PHONE_NUMBERS;i++){if(i)j+=",";j+="\""+jsonEscape(phoneNumbers[i])+"\"";}
  j+="],\"commands\":[";
  for(int i=0;i<SMS_CMD_COUNT;i++){if(i)j+=",";j+="\""+jsonEscape(smsCmd[i])+"\"";}
  j+="]},\"mqtt\":{\"enabled\":"+String(mqttConfig.requestedEnabled?"true":"false")+",\"tls\":"+String(mqttConfig.tlsRequested?"true":"false");
  j+=",\"host\":\""+jsonEscape(mqttConfig.host)+"\",\"port\":"+String(mqttConfig.port);
  j+=",\"deviceId\":\""+jsonEscape(mqttConfig.deviceId)+"\",\"username\":\""+jsonEscape(mqttConfig.username)+"\"";
  j+=",\"topicPrefix\":\""+jsonEscape(mqttConfig.topicPrefix)+"\",\"keepAlive\":"+String(mqttConfig.keepAliveSec);
  j+=",\"positionInterval\":"+String(mqttConfig.positionIntervalSec)+",\"heartbeatInterval\":"+String(mqttConfig.heartbeatIntervalSec);
  j+=",\"hasPassword\":"+String(mqttConfig.password.length()>0?"true":"false")+",\"hasCommandKey\":"+String(mqttConfig.commandKey.length()==64?"true":"false")+"}}";
  sclReply(200,j);
}
void sclAction(){
  if(!sclRequireAuth())return;
  const String action=server.arg("action");
  if(action=="door_lock")setDoors(true);
  else if(action=="door_unlock")setDoors(false);
  else if(action=="trunk")triggerTrunk();
  // Ignition/fuel controls are intentionally not exposed without a validated vehicle-stopped interlock.
  else {sclReply(403,"{\"ok\":false,\"error\":\"unsupported_or_unsafe_action\"}");return;}
  sclReply(200,"{\"ok\":true,\"action\":\""+action+"\"}");
}
void sclApSave(){
  if(!sclRequireAuth())return;
  String ssid=server.arg("ssid"),pass=server.arg("password"),ipStr=server.arg("ip");
  if(ssid.length()<1||ssid.length()>32){sclReply(400,"{\"ok\":false,\"error\":\"ssid\"}");return;}
  if(pass.length()>0&&(pass.length()<12||pass.length()>63)){sclReply(400,"{\"ok\":false,\"error\":\"password_min_12\"}");return;}
  IPAddress newIp=ap_ip;
  if(ipStr.length()){newIp=stringToIP(ipStr);if(newIp==INADDR_NONE||!ipIsUsable(newIp)){sclReply(400,"{\"ok\":false,\"error\":\"ip\"}");return;}}
  if(server.hasArg("threshold")){
    String v=server.arg("threshold");int t=v.toInt();
    if(t<0||t>100||v!=String(t)){sclReply(400,"{\"ok\":false,\"error\":\"threshold\"}");return;}
    signalThreshold=t;saveSignalThreshold();
  }
  if(server.hasArg("weakAuto")){
    bool enabled=server.arg("weakAuto")=="1";
    if(enabled!=weakAutoDisconnectEnabled){weakAutoDisconnectEnabled=enabled;saveWeakAutoSetting();weakDisconnectActive=false;}
  }
  bool reboot=ssid!=ap_ssid||newIp!=ap_ip||(pass.length()&&pass!=ap_password);
  Preferences p;if(p.begin("carctrl",false)){
    p.putString("ssid",ssid);p.putString("ip",newIp.toString());
    if(pass.length())p.putString("pwd",pass);p.end();
  }else{sclReply(500,"{\"ok\":false}");return;}
  sclReply(200,String("{\"ok\":true,\"rebootRequired\":")+(reboot?"true":"false")+"}");
  // Avoid rebooting mid-request; ask operator to power cycle after updating Wi-Fi credentials.
}






// ===================== Output Name Functions =====================
void loadCustomNames() {
  preferences.begin("carctrl", true);
  for (int i = 0; i < OUTPUT_COUNT; i++) {
    String key = "name" + String(i);
    customOutputNames[i] = preferences.getString(key.c_str(), "");
  }
  preferences.end();
}

void saveCustomName(int idx) {
  if (idx < 0 || idx >= OUTPUT_COUNT) return;

  preferences.begin("carctrl", false);
  String key = "name" + String(idx);
  preferences.putString(key.c_str(), customOutputNames[idx]);
  preferences.end();
}

void handleSetOutputName() {
  if (!isRequestAuthorized()) {
    server.send(403, "application/json", "{\"success\":false,\"message\":\"Unauthorized\"}");
    return;
  }

  if (!server.hasArg("pin") || !server.hasArg("name")) {
    server.send(400, "application/json", "{\"success\":false,\"message\":\"Missing parameters\"}");
    return;
  }

  int idx = server.arg("pin").toInt();
  String newName = server.arg("name");
  newName.trim();

  if (idx < 0 || idx >= OUTPUT_COUNT) {
    server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid pin\"}");
    return;
  }

  if (newName.length() == 0 || newName.length() > 60) {   // UTF-8: ~30 نویسه فارسی
    server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid name length\"}");
    return;
  }

  customOutputNames[idx] = newName;
  saveCustomName(idx);

  Serial.println("Output " + String(idx) + " renamed to: " + newName);
  server.send(200, "application/json", "{\"success\":true,\"name\":\"" + jsonEscape(newName) + "\"}");
}

// ===================== Language Functions =====================
void saveLanguageSettings() {
  preferences.begin("carctrl", false);
  preferences.putString("lang", systemLanguage);
  preferences.putString("nextLang", systemLanguage);
  preferences.end();
}











String getOutputName(int idx) {
  if (idx < 0 || idx >= OUTPUT_COUNT) return "";

  if (customOutputNames[idx].length() > 0) return customOutputNames[idx];

  if (systemLanguage == "en") {
    static const char* namesEn[OUTPUT_COUNT] = {"Doors", "Trunk", "Fuel Pump", "Car Switch", "Doors Inverse"};
    return namesEn[idx];
  }

  if (systemLanguage == "ar") {
    static const char* namesAr[OUTPUT_COUNT] = {"الأبواب", "صندوق السيارة", "مضخة الوقود", "مفتاح السيارة", "عكس الأبواب"};
    return namesAr[idx];
  }

  static const char* namesFa[OUTPUT_COUNT] = {"دربها", "صندوق عقب", "پمپ بنزین", "سوئیچ خودرو", "معکوس دربها"};
  return namesFa[idx];
}

// ===================== Translations (جدول) =====================
struct Tr { const char* k; const char* en; const char* ar; const char* fa; };

static const Tr TR[] = {
  {"mainTitle", "Car Control", "التحكم بالسيارة", "کنترل خودرو"},
  {"signal", "Signal", "الإشارة", "سیگنال"},
  {"autoDisconnectBefore", "Auto relay control with weak signal (below",
   "إيقاف وتشغيل المرحلات تلقائيًا عند ضعف الإشارة (أقل من",
   "قطع و وصل خودکار رله‌ها با سیگنال ضعیف (کمتر از"},
  {"autoDisconnectAfter", "%)", "%)", "٪)"},
  {"settingsBtn", "Settings", "الإعدادات", "تنظیمات"},
  {"on", "ON", "تشغيل", "روشن"},
  {"off", "OFF", "إيقاف", "خاموش"},
  {"editNameTitle", "Edit Output Name", "تحرير اسم المخرج", "تغییر نام خروجی"},
  {"saveLabel", "Save", "حفظ", "ذخیره"},
  {"cancelLabel", "Cancel", "إلغاء", "انصراف"},
  {"authTitle", "Device Activation", "تفعيل الجهاز", "فعال‌سازی دستگاه"},
  {"authInfo", "This device is not authorized. Enter the activation code to register its MAC address and IP address.",
   "هذا الجهاز غير مصرح به. أدخل رمز التفعيل لتسجيل عنوان MAC وعنوان IP.",
   "این دستگاه مجاز نیست. برای ثبت نشانی مک و نشانی آی‌پی، کد فعال‌سازی را وارد کنید."},
  {"authMac", "Device MAC Address", "عنوان MAC للجهاز", "نشانی مک دستگاه"},
  {"authIp", "Device IP Address", "عنوان IP للجهاز", "نشانی آی‌پی دستگاه"},
  {"authPlaceholder", "Activation code", "رمز التفعيل", "کد فعال‌سازی"},
  {"authBtn", "Activate Device", "تفعيل الجهاز", "فعال‌سازی دستگاه"},
  {"authWarning", "Important: If your phone uses a randomized MAC address, disable it for this Wi-Fi network and select the device's real MAC address. Otherwise activation may be required again.",
   "ملاحظة مهمة: إذا كان هاتفك يستخدم عنوان MAC عشوائيًا، فقم بتعطيله لهذه الشبكة واختر عنوان الجهاز الحقيقي. وإلا قد تحتاج إلى التفعيل مرة أخرى.",
   "نکته مهم: اگر گوشی شما از نشانی مک تصادفی استفاده می‌کند، برای این شبکه وای‌فای آن را غیرفعال کنید و نشانی مک واقعی دستگاه را انتخاب کنید. در غیر این صورت ممکن است دوباره نیاز به فعال‌سازی باشد."},
  {"successTitle", "Device activated successfully!", "تم تفعيل الجهاز بنجاح!", "دستگاه با موفقیت فعال شد!"},
  {"successText", "The MAC and IP addresses of this device were saved. Future connections will be checked against them.",
   "تم حفظ عنواني MAC وIP لهذا الجهاز. سيتم التحقق منهما في الاتصالات القادمة.",
   "نشانی مک و آی‌پی این دستگاه ذخیره شد. اتصالات بعدی با همین اطلاعات بررسی می‌شوند."},
  {"successLink", "Go to car control page", "الانتقال إلى صفحة التحكم بالسيارة", "رفتن به صفحه کنترل خودرو"},
  {"errorTitle", "Error", "خطأ", "خطا"},
  {"retry", "Try again", "إعادة المحاولة", "تلاش مجدد"},
  {"errorWrongCode", "Activation code is incorrect.", "رمز التفعيل غير صحيح.", "کد فعال‌سازی اشتباه است."},
  {"errorNoMac", "Error: Could not identify device MAC address. Reconnect or disable randomized MAC.",
   "خطأ: لم يتم التعرف على عنوان MAC للجهاز. أعد الاتصال أو عطّل عنوان MAC العشوائي.",
   "خطا: نشانی مک دستگاه شناسایی نشد. دوباره متصل شوید یا نشانی مک تصادفی را غیرفعال کنید."},
  {"settingsTitle", "Access Point Settings", "إعدادات نقطة الوصول", "تنظیمات نقطه دسترسی"},
  {"ssidLabel", "Wi-Fi Network Name (SSID)", "اسم شبكة الواي فاي (SSID)", "نام شبکه وای‌فای (SSID)"},
  {"pwdLabel", "Wi-Fi Password (minimum 8 characters)", "كلمة مرور الواي فاي (8 أحرف على الأقل)", "رمز عبور وای‌فای (حداقل ۸ نویسه)"},
  {"ipLabel", "IP Address (e.g. 192.168.4.1)", "عنوان الإنترنت (IP) مثل 192.168.4.1", "نشانی آی‌پی (مانند 192.168.4.1)"},
  {"saveBtn", "Save and Reboot", "حفظ وإعادة التشغيل", "ذخیره و راه‌اندازی مجدد"},
  {"backLink", "Back to Control Page", "العودة إلى صفحة التحكم", "بازگشت به صفحه کنترل"},
  {"settingsWarning", "Changing settings reboots the device. You must reconnect to the new Wi-Fi network.",
   "تغيير الإعدادات يعيد تشغيل الجهاز. يجب إعادة الاتصال بشبكة الواي فاي الجديدة.",
   "تغییر تنظیمات باعث راه‌اندازی مجدد دستگاه می‌شود. باید دوباره به شبکه وای‌فای جدید متصل شوید."},
  {"noChangeTitle", "No changes applied", "لم يتم تطبيق أي تغييرات", "هیچ تغییری اعمال نشد"},
  {"rebootTitle", "Rebooting...", "إعادة التشغيل...", "راه‌اندازی مجدد..."},
  {"rebootSaved", "Settings saved", "تم حفظ الإعدادات", "تنظیمات ذخیره شد"},
  {"rebootText", "Device will reboot in", "سيتم إعادة تشغيل الجهاز خلال", "دستگاه در"},
  {"seconds", "seconds", "ثانية", "ثانیه راه‌اندازی مجدد می‌شود"},
  {"signalThresholdTitle", "Signal Threshold", "حد الإشارة", "آستانه سیگنال"},
  {"signalThresholdDesc", "If automatic signal protection is enabled, outputs turn off below this percentage and turn back on when signal recovers.",
   "إذا تم تفعيل حماية الإشارة التلقائية، فسيتم إيقاف المخرجات عندما تكون الإشارة أقل من هذه النسبة، ثم إعادة تشغيلها عند تحسن الإشارة.",
   "اگر محافظت خودکار سیگنال فعال باشد، خروجی‌ها در کمتر از این درصد خاموش می‌شوند و با بهبود سیگنال دوباره روشن می‌شوند."},
  {"signalThresholdLabel", "Weak signal threshold (%)", "حد الإشارة الضعيفة (%)", "آستانه سیگنال ضعیف (٪)"},
  {"signalThresholdSave", "Save Threshold", "حفظ الحد", "ذخیره آستانه"},
  {"signalThresholdSaved", "Threshold saved successfully.", "تم حفظ الحد بنجاح.", "آستانه با موفقیت ذخیره شد."},
  {"signalThresholdInvalid", "Invalid value. Enter a number between 0 and 100.", "قيمة غير صالحة. أدخل رقمًا بين 0 و100.", "مقدار نامعتبر است. عددی بین ۰ تا ۱۰۰ وارد کنید."},
  {"menuTitle", "Settings", "الإعدادات", "تنظیمات"},
  {"menuAccessPoint", "Access Point Settings", "إعدادات نقطة الوصول", "تنظیمات نقطه دسترسی"},
  {"menuSim800", "SIM800", "SIM800", "SIM800"},
  {"menuMqtt", "MQTT / Cloud", "MQTT / السحابة", "MQTT / سرور ابری"},
  {"mqttTitle", "MQTT Cloud Settings", "إعدادات MQTT السحابية", "تنظیمات MQTT سرور ابری"},
  {"mqttSaved", "Settings saved; reconnect scheduled.", "تم حفظ الإعدادات؛ ستتم إعادة الاتصال.", "تنظیمات ذخیره شد؛ اتصال مجدد انجام می‌شود."},
  {"mqttNote", "Secure native MQTT (SIM800 firmware-dependent). Requires a provisioned trusted CA certificate and valid modem clock; there is no insecure fallback.", "اتصال MQTT الآمن يتطلب شهادة جذر موثوقة وساعة مضبوطة ووظائف MQTT في البرنامج الثابت.", "ارتباط MQTT وابسته به فریمور SIM800 است و به گواهی CA معتبر و ساعت درست نیاز دارد؛ اتصال ناامن فعال نمی‌شود."},
  {"mqttEnabled", "Enable secure MQTT", "تفعيل MQTT الآمن", "فعال‌سازی MQTT امن"},
  {"mqttHost", "Broker address", "عنوان وسيط MQTT", "آدرس سرور MQTT"},
  {"mqttPort", "Broker port", "منفذ MQTT", "پورت سرور MQTT"},
  {"mqttDeviceId", "Unique vehicle/device ID", "معرّف المركبة الفريد", "شناسه یکتای خودرو / دستگاه"},
  {"mqttUsername", "MQTT username", "اسم مستخدم MQTT", "نام کاربری MQTT"},
  {"mqttPassword", "MQTT password", "كلمة مرور MQTT", "رمز عبور MQTT"},
  {"mqttKeepSecret", "Leave blank to keep the stored password", "اتركه فارغاً للاحتفاظ بكلمة المرور", "برای حفظ رمز ذخیره‌شده خالی بگذارید"},
  {"mqttClearSecret", "Clear stored password", "حذف كلمة المرور المحفوظة", "حذف رمز ذخیره‌شده"},
  {"mqttTls", "Require TLS and certificate verification", "TLS والتحقق من شهادة الخادم", "الزام TLS و اعتبارسنجی گواهی سرور"},
  {"mqttTopic", "MQTT topic prefix", "بادئة مواضيع MQTT", "پیشوند Topic های MQTT"},
  {"mqttKeepAlive", "MQTT keep-alive (seconds)", "مهلة MQTT بالثواني", "Keepalive برحسب ثانیه"},
  {"mqttPositionInterval", "GPS reporting interval (seconds)", "فاصل إرسال GPS بالثواني", "فاصله ارسال موقعیت GPS (ثانیه)"},
  {"mqttHeartbeat", "Heartbeat / status interval (seconds)", "فاصل إرسال الحالة بالثواني", "فاصله ارسال وضعیت دستگاه (ثانیه)"},
  {"mqttStatus", "Connection state", "حالة الاتصال", "وضعیت اتصال"},
  {"mqttNotConnected", "Not implemented; settings only", "غير مفعّل؛ إعدادات فقط", "پیاده‌سازی نشده؛ فقط تنظیمات"},
  {"mqttTopicExample", "Planned topics", "المواضيع المخططة", "تاپیک‌های پیشنهادی"},
  {"mqttSave", "Save MQTT settings", "حفظ إعدادات MQTT", "ذخیره تنظیمات MQTT"},
  {"mqttSavedPassword", "A password is stored (hidden)", "كلمة مرور محفوظة (مخفية)", "رمز عبور ذخیره شده (مخفی)"},
  {"mqttNoPassword", "No password stored", "لا توجد كلمة مرور", "رمز عبور ذخیره نشده"},
  {"mqttGprsNote", "The cellular APN is configured separately in SIM800 settings. TLS support must be tested against the SIM800 firmware before connecting to the server.", "يتم ضبط APN من إعدادات SIM800. يجب اختبار TLS قبل الاتصال بالخادم.", "APN اینترنت در تنظیمات SIM800 است. پشتیبانی TLS باید پیش از اتصال واقعی با نسخه فریمور SIM800 آزمایش شود."},
  {"mqttLocalWarning", "The local web panel uses HTTP and NVS may be unencrypted. Change the default access point password before entering live MQTT credentials.", "الواجهة المحلية تستخدم HTTP وقد لا تكون NVS مشفرة. غيّر كلمة مرور نقطة الوصول الافتراضية.", "پنل محلی با HTTP باز می‌شود و NVS ممکن است رمزگذاری نشده باشد. پیش از وارد کردن رمز واقعی MQTT، رمز پیش‌فرض Wi-Fi را تغییر دهید."},
  {"sim800Title", "SIM800 Settings", "إعدادات SIM800", "تنظیمات SIM800"},
  {"backToMenu", "Back to Settings Menu", "العودة إلى قائمة الإعدادات", "بازگشت به منوی تنظیمات"},
  {"comingSoon", "This section will be available soon.", "سيكون هذا القسم متاحًا قريبًا.", "این بخش به زودی در دسترس خواهد بود."},
  {"autoDetect", "Auto Detect APN", "كشف تلقائي APN", "تشخیص خودکار APN"},
  {"phoneNumbersTitle", "Authorized Phone Numbers", "أرقام الهواتف المصرح بها", "شماره‌های مجاز"},
  {"phoneNumbersHint", "Only these numbers can send SMS commands or make calls. Commands: ON, OFF, STATUS, ON0..OFF3",
   "فقط هذه الأرقام يمكنها إرسال أوامر SMS أو الاتصال. الأوامر: ON, OFF, STATUS, ON0..OFF3",
   "فقط این شماره‌ها می‌توانند فرمان SMS بفرستند یا تماس بگیرند. فرمان‌ها: ON, OFF, STATUS, ON0..OFF3"},
  {"customInitTitle", "Custom AT Commands (one per line)", "أوامر AT مخصصة (سطر واحد لكل أمر)", "دستورات AT سفارشی (هر خط یک فرمان)"},
  {"customInitHint", "These commands are sent after module init.", "تُرسل هذه الأوامر بعد تهيئة الوحدة.", "این دستورات پس از راه‌اندازی ماژول ارسال می‌شوند."},
  {"saving", "Saving...", "جارٍ الحفظ...", "در حال ذخیره..."},
  {"saved", "Saved successfully", "تم الحفظ بنجاح", "با موفقیت ذخیره شد"},
  {"saveFailed", "Save failed", "فشل الحفظ", "ذخیره ناموفق"},
  {"gpsEnabled", "GPS Enabled", "GPS مفعّل", "GPS فعال"},
  {"gpsSearching", "Searching for satellite...", "جارٍ البحث عن القمر الصناعي...", "در حال جستجوی ماهواره..."},
  {"gpsNoFix", "No GPS fix (check antenna)", "لا يوجد إشارة GPS (تحقق من الهوائي)", "بدون fix (آنتن را بررسی کنید)"},
  {"gpsMapSite", "Map Site", "موقع الخريطة", "سایت نقشه"},
    {"gpsTitle", "GPS Settings", "إعدادات GPS", "تنظیمات GPS"},
  {"safeZoneTitle", "Safe Zone", "المنطقة الآمنة", "محدوده امن"},
  {"safeZoneEnable", "Enable Safe Zone", "تفعيل المنطقة الآمنة", "فعال‌سازی محدوده امن"},
  {"safeZoneLat", "Latitude", "خط العرض", "عرض جغرافیایی"},
  {"safeZoneLng", "Longitude", "خط الطول", "طول جغرافیایی"},
  {"safeZoneRadius", "Radius (meters)", "نصف القطر (متر)", "شعاع (متر)"},
  {"useCurrentLocation", "Use Current Location", "استخدام الموقع الحالي", "استفاده از موقعیت فعلی"},
};

String t(const char* key) {
  const int n = sizeof(TR) / sizeof(TR[0]);
  for (int i = 0; i < n; i++) {
    if (strcmp(TR[i].k, key) == 0) {
      if (systemLanguage == "en") return String(TR[i].en);
      if (systemLanguage == "ar") return String(TR[i].ar);
      return String(TR[i].fa);
    }
  }
  return String(key);
}

// ===================== Signal Threshold =====================
void saveSignalThreshold() {
  preferences.begin("carctrl", false);
  preferences.putInt("sigThresh", signalThreshold);
  preferences.end();
  Serial.println("Signal threshold saved: " + String(signalThreshold) + "%");
}

void handleSetSignalThreshold() {
  if (!isRequestAuthorized()) {
    server.send(403, "application/json", "{\"success\":false,\"message\":\"Unauthorized\"}");
    return;
  }

  if (!server.hasArg("value")) {
    server.send(400, "application/json", "{\"success\":false,\"message\":\"Missing value\"}");
    return;
  }

  int val = server.arg("value").toInt();

  if (val >= 0 && val <= 100) {
    signalThreshold = val;
    saveSignalThreshold();
    server.send(200, "application/json", "{\"success\":true,\"threshold\":" + String(signalThreshold) + "}");
  } else {
    server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid value\"}");
  }
}

// ===================== RSSI Logic =====================
// RSSI کلاینتی که IP درخواست را دارد (در صورت نیافتن، اولین کلاینت)
int getClientRSSI() {
  wifi_sta_list_t wifi_sta_list;
  esp_err_t err = esp_wifi_ap_get_sta_list(&wifi_sta_list);
  if (err == ESP_OK && wifi_sta_list.num > 0) {
    int best = -127;
    for (int i = 0; i < wifi_sta_list.num; i++) {
      if (wifi_sta_list.sta[i].rssi > best) best = wifi_sta_list.sta[i].rssi;
    }
    return best;
  }
  return 0;
}

int rssiToPercentage(int rssi) {
  if (rssi == 0) return 0;
  int quality;
  if (rssi <= -100) quality = 0;
  else if (rssi >= -50) quality = 100;
  else quality = 2 * (rssi + 100);
  return constrain(quality, 0, 100);
}

// ===================== Settings Storage =====================
void loadAllSettings() {
  preferences.begin("carctrl", true);

  ap_ssid = preferences.getString("ssid", DEFAULT_SSID);
  ap_password = preferences.getString("pwd", DEFAULT_PASSWORD);

  String ipStr = preferences.getString("ip", DEFAULT_IP);
  ap_ip = stringToIP(ipStr);
  if (ap_ip == INADDR_NONE || !ipIsUsable(ap_ip)) ap_ip = stringToIP(DEFAULT_IP);

  systemLanguage = preferences.getString("lang", "fa");
  nextLanguage = preferences.getString("nextLang", "fa");

  signalThreshold = preferences.getInt("sigThresh", 50);
  if (signalThreshold < 0 || signalThreshold > 100) signalThreshold = 50;

  weakAutoDisconnectEnabled = preferences.getBool("weakAuto", false);

  if (systemLanguage != "en" && systemLanguage != "fa" && systemLanguage != "ar") systemLanguage = "fa";
  if (nextLanguage != "en" && nextLanguage != "fa" && nextLanguage != "ar") nextLanguage = "fa";

  preferences.end();

  // اعتبارسنجی SSID/Password (در غیر این صورت softAP بدون رمز بالا می‌آید یا شکست می‌خورد)
  if (ap_ssid.length() == 0 || ap_ssid.length() > 32) ap_ssid = DEFAULT_SSID;
  if (ap_password.length() < 8 || ap_password.length() > 63) ap_password = DEFAULT_PASSWORD;
}

// ===================== Wi-Fi Access Point =====================
void setupWiFiAP() {
  WiFi.persistent(false);          // جلوگیری از نوشتن بی‌مورد روی فلش
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_AP);

  bool ok = WiFi.softAP(ap_ssid.c_str(), ap_password.c_str(), AP_CHANNEL, 0, AP_MAX_CLIENTS);
  if (!ok) {
    Serial.println("softAP FAILED - retrying with defaults");
    ok = WiFi.softAP(DEFAULT_SSID, DEFAULT_PASSWORD, AP_CHANNEL, 0, AP_MAX_CLIENTS);
  }
  delay(150);

  // IP و DHCP بعد از softAP تنظیم می‌شود (ترتیب مطمئن‌تر)
  for (int attempt = 0; attempt < 3; attempt++) {
    WiFi.softAPConfig(ap_ip, ap_ip, IPAddress(255, 255, 255, 0));
    delay(100);
    if (WiFi.softAPIP() == ap_ip) break;
    Serial.println("softAPConfig retry...");
  }

  WiFi.setSleep(false);            // تاخیر کمتر و پایداری بیشتر وب‌سرور

  Serial.print("Access Point ");
  Serial.println(ok ? "started" : "FAILED");
  Serial.print("AP IP (actual): ");
  Serial.println(WiFi.softAPIP());
}

void setupDNSServer() {
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", WiFi.softAPIP());
  Serial.println("DNS server started");
}

// ===================== Web Server =====================
void setupWebServer(){
  server.collectHeaders(sclHeaders,1);
  server.on("/api/v1/ping", HTTP_GET, sclPing);
  server.on("/api/v1/pair", HTTP_POST, sclPair);
  server.on("/api/v1/phones", HTTP_GET, sclPhones);
  server.on("/api/v1/phones/revoke", HTTP_POST, sclRevokePhone);
  server.on("/api/v1/state", HTTP_GET, sclState);
  server.on("/api/v1/config", HTTP_GET, sclConfig);
  server.on("/api/v1/action", HTTP_POST, sclAction);
  server.on("/api/v1/config/ap", HTTP_POST, sclApSave);
  server.on("/api/v1/config/gps", HTTP_POST, handleSetGpsConfig);
  server.on("/api/v1/config/sim", HTTP_POST, handleSetSim800Config);
  server.on("/api/v1/config/mqtt", HTTP_POST, handleSaveMqttSettings);
  server.on("/api/v1/config/output-name", HTTP_POST, handleSetOutputName);
  server.on("/api/v1/rfid/register", HTTP_POST, rfidRegister);
  server.onNotFound([](){sclReply(404,"{\"ok\":false,\"error\":\"not_found\"}");});
  server.begin();
}










void handleSaveMqttSettings() {
  if (mqttWaiting) { server.send(409,"text/plain","Modem busy with MQTT; retry saving in a few seconds"); return; }
  if (!isRequestAuthorized()) { server.send(403, "text/plain", "Forbidden"); return; }
  // Changes are saved first; transport reconnect is scheduled, not done in HTTP handler.
  MqttConfig newCfg = mqttConfig;
  newCfg.requestedEnabled = server.hasArg("enabled") && server.arg("enabled") == "1";
  newCfg.tlsRequested = server.hasArg("tls") && server.arg("tls") == "1";
  if (!server.hasArg("host") || !server.hasArg("port") ||
      !server.hasArg("deviceId") || !server.hasArg("topicPrefix") ||
      !server.hasArg("username") || !server.hasArg("keepAlive") ||
      !server.hasArg("positionInterval") || !server.hasArg("heartbeatInterval")) {
    server.send(400, "text/plain", "Missing fields"); return;
  }
  newCfg.host = server.arg("host"); newCfg.host.trim();
  newCfg.deviceId = server.arg("deviceId"); newCfg.deviceId.trim();
  newCfg.topicPrefix = server.arg("topicPrefix"); newCfg.topicPrefix.trim();
  newCfg.username = server.arg("username"); newCfg.username.trim();
  if (!mqttValidHost(newCfg.host) ||
      !mqttSafeToken(newCfg.deviceId, 32) ||
      !mqttSafeToken(newCfg.topicPrefix, 64, true) ||
      newCfg.username.length() > 64 || !asciiOnly(newCfg.username) || newCfg.username.indexOf('"') >= 0 || newCfg.username.indexOf('\\') >= 0 || newCfg.username.indexOf('\n') >= 0 || newCfg.username.indexOf('\r') >= 0) {
    server.send(400, "text/plain", "Invalid host, device ID, topic or username"); return;
  }
  // Only unsigned decimal digits are accepted; String::toInt() alone is too permissive.
  auto parseRange = [](const String& v, uint32_t minV, uint32_t maxV, uint16_t &out) -> bool {
    if (v.length() == 0 || v.length() > 5) return false;
    for (size_t i = 0; i < v.length(); ++i)
      if (v[i] < '0' || v[i] > '9') return false;
    unsigned long n = strtoul(v.c_str(), nullptr, 10);
    if (n < minV || n > maxV) return false;
    out = (uint16_t)n;
    return true;
  };
  if (!parseRange(server.arg("port"), 1, 65535, newCfg.port) ||
      !parseRange(server.arg("keepAlive"), 60, 600, newCfg.keepAliveSec) ||
      !parseRange(server.arg("positionInterval"), 10, 3600, newCfg.positionIntervalSec) ||
      !parseRange(server.arg("heartbeatInterval"), 30, 3600, newCfg.heartbeatIntervalSec)) {
    server.send(400, "text/plain", "Invalid numeric range"); return;
  }
  // Blank password means KEEP existing secret; explicit checkbox clears it.
  if (server.hasArg("clearPassword") && server.arg("clearPassword") == "1")
    newCfg.password = "";
  else if (server.hasArg("password") && server.arg("password").length() > 0) {
    String candidate = server.arg("password");
    if (candidate.length() > 96 || !asciiOnly(candidate) || candidate.indexOf('"') >= 0 || candidate.indexOf('\\') >= 0 || candidate.indexOf('\n') >= 0 || candidate.indexOf('\r') >= 0) {
      server.send(400, "text/plain", "Invalid password"); return;
    }
    newCfg.password = candidate;
  }
  if (server.hasArg("clearCommandKey") && server.arg("clearCommandKey") == "1")
    newCfg.commandKey="";
  else if (server.hasArg("commandKey") && server.arg("commandKey").length()>0) {
    String key=server.arg("commandKey");
    if (key.length()!=64) {server.send(400,"text/plain","Command HMAC key must be 64 hex characters");return;}
    for (size_t i=0;i<key.length();i++)
      if (!isxdigit((unsigned char)key[i])) {server.send(400,"text/plain","Command key must be hex");return;}
    key.toLowerCase(); newCfg.commandKey=key;
  }
  // Sequence IDs belong to one device ID; changing it requires fresh broker sequence space.
  if (newCfg.deviceId != mqttConfig.deviceId && preferences.begin("mqttcfg",false)) {
    preferences.remove("lastSeq"); preferences.end(); mqttLastSequence=0;
  }
  if (!saveMqttConfig(newCfg)) { server.send(500, "text/plain", "NVS save failed"); return; }
  mqttConfig = newCfg;
  mqttResetTransport("Configuration changed");
  sclReply(200, "{\"ok\":true}");
}



void handleToggle() {
  if (!isRequestAuthorized()) {
    server.send(403, "application/json", "{\"success\":false, \"message\":\"Unauthorized\"}");
    return;
  }

  if (!server.hasArg("pin")) {
    server.send(400, "application/json", "{\"success\":false}");
    return;
  }

  int idx = server.arg("pin").toInt();
  if (idx == 4) idx = 0;

  if (idx < 0 || idx >= OUTPUT_COUNT) {
    server.send(400, "application/json", "{\"success\":false}");
    return;
  }

  if (idx == 0) {
    bool newLockState = !outputState[0];
    setDoors(newLockState);

    // اگر Safe Zone فعال بود و کاربر درب را از پنل وب باز کرد → کنترل بدست گرفته
    if (!newLockState && safeZoneAlarmActive) {
        safeZoneAlarmActive  = false;
        safeZoneCallIdx      = 0;
        safeZoneCallAnswered = false;
        alarmManual = false;
        if (callActive) {
            if (locPhase == 0) sim800SendAT("ATH", "OK", 1500);
            callActive = false;
            activeCaller = "";
        }
        updateBuzzer();
        Serial.println("Safe zone alarm CLEARED via web door unlock");
    }

    server.send(200, "application/json",
                "{\"success\":true,\"pin\":0,\"state\":" +
                String(outputState[0] ? 1 : 0) + "}");
    return;
}

  if (idx == 1) {
    triggerTrunk();
    server.send(200, "application/json", "{\"success\":true,\"pin\":1,\"state\":1}");
    return;
  }

  bool newState = !outputState[idx];
  setOutput(idx, newState);
  desiredState[idx] = outputState[idx];

  server.send(200, "application/json",
              "{\"success\":true,\"pin\":" + String(idx) + ",\"state\":" + String(outputState[idx] ? 1 : 0) + "}");
}

void handleStatus() {
  if (!isRequestAuthorized()) {
    server.send(403, "application/json", "{\"success\":false, \"message\":\"Unauthorized\"}");
    return;
  }

  String json = "{\"states\":[";
  for (int i = 0; i < 4; i++) {
    json += outputState[i] ? "1" : "0";
    if (i < 3) json += ",";
  }
  json += "],\"weakAuto\":";
  json += weakAutoDisconnectEnabled ? "true" : "false";
  json += "}";

  server.send(200, "application/json", json);
}





void handleGetRSSI() {
  if (!isRequestAuthorized()) {
    server.send(403, "application/json", "{\"rssi\":0, \"percent\":0}");
    return;
  }
  int rssi = getClientRSSI();
  int percent = rssiToPercentage(rssi);
  server.send(200, "application/json", "{\"rssi\":" + String(rssi) + ",\"percent\":" + String(percent) + "}");
}



// ===================== Output Control =====================
void setOutput(int index, bool state) {
  if (index < 0 || index >= OUTPUT_COUNT) return;

  if (index == 0) {
    bool oldState = outputState[0];
    outputState[0] = state;

    cancelPulse(PIN_DOOR_DRIVER);
    cancelPulse(PIN_DOORS_INVERSE);
    digitalWrite(PIN_DOOR_DRIVER, LOW);
    digitalWrite(PIN_DOORS_INVERSE, LOW);

    if (oldState != state) {
      Serial.print(getOutputName(0));
      Serial.print(" ");
      Serial.println(state ? t("on") : t("off"));
    }
    return;
  }

  if (index == 1) {
    bool oldState = outputState[1];

    cancelPulse(PIN_TRUNK);
    digitalWrite(PIN_TRUNK, LOW);

    outputState[1] = false;
    desiredState[1] = false;

    if (oldState) {
      Serial.print(getOutputName(1));
      Serial.print(" ");
      Serial.println(t("off"));
    }
    return;
  }

  if (index == 4) return;

  if ((index == 2 || index == 3) && state && carLocked) {
    Serial.println("Output blocked: car is LOCKED");
    return;
  }

  cancelPulse(outputPins[index]);

  bool oldState = outputState[index];
  outputState[index] = state;
  digitalWrite(outputPins[index], state ? HIGH : LOW);

  if (oldState != state) {
    Serial.print(getOutputName(index));
    Serial.print(" ");
    Serial.println(state ? t("on") : t("off"));
  }
}

// ===================== IP Address Parser =====================
IPAddress stringToIP(String ipStr) {
  ipStr.trim();
  int parts[4] = {0, 0, 0, 0};
  int part = 0;
  int count = 0;
  int digits = 0;
  bool valid = true;

  for (int i = 0; i <= (int)ipStr.length(); i++) {
    char c = ipStr[i];
    if (c == '.' || c == '\0') {
      if (digits == 0 || count >= 4) { valid = false; break; }
      parts[count++] = part;
      part = 0;
      digits = 0;
    } else if (c >= '0' && c <= '9') {
      part = part * 10 + (c - '0');
      digits++;
      if (part > 255) { valid = false; break; }
    } else {
      valid = false;
      break;
    }
  }

  if (valid && count == 4) return IPAddress(parts[0], parts[1], parts[2], parts[3]);
  return INADDR_NONE;
}

// IP قابل‌استفاده برای AP (نه 0.x ، نه loopback، نه multicast، نه شبکه/broadcast)
bool ipIsUsable(IPAddress ip) {
  if (ip[0] == 0 || ip[0] == 127 || ip[0] >= 224) return false;
  if (ip[3] == 0 || ip[3] == 255) return false;
  return true;
}

// ===================== Settings Menu Page =====================


// ===================== Placeholder page =====================




// ===================== SIM800 Page =====================


// ===================== Main Page =====================


// ===================== Auth / Success / Error Pages =====================






// ===================== AP Settings Page =====================


// =====================================================================
//                              SIM800
// =====================================================================
#define URC_QUEUE_SIZE 8
String urcQueue[URC_QUEUE_SIZE];
uint8_t urcHead = 0, urcTail = 0;

bool sim800SerialStarted = false;
bool sim800PowerTried    = false;
bool sim800ForceInit     = false;
uint8_t sim800InitFails  = 0;
uint8_t sim800NoReply    = 0;

bool smsDrained = false;                 // پیامک‌های قدیمیِ قبل از بوت یک‌بار جمع شده‌اند؟
unsigned long sim800LastSmsPoll = 0;
unsigned long sim800LastSmsVerify = 0;

// صف فرمان‌هایی که باید بعد از پایان عملیات طولانی (اینترنت/لوکیشن) اجرا شوند
#define PEND_MAX 4
String pendFrom[PEND_MAX];
String pendBody[PEND_MAX];
int pendCount = 0;

const char* DEFAULT_CMDS[SMS_CMD_COUNT] = {"Location", "Lock", "Unlock", "Alarm_on", "Alarm_off", "Stop location", "Status"};
const char* CMD_KEYS[SMS_CMD_COUNT]     = {"cLoc", "cLock", "cUnl", "cAon", "cAof", "cStpL", "cStat"};

// ===================== Helpers =====================
String jsonEscape(const String& in) {
  String o;
  o.reserve(in.length() + 8);
  for (unsigned int i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"') o += "\\\"";
    else if (c == '\\') o += "\\\\";
    else if (c == '\n') o += "\\n";
    else if (c == '\r') o += "\\r";
    else if (c == '\t') o += "\\t";
    else if ((uint8_t)c < 0x20) o += ' ';
    else o += c;
  }
  return o;
}

bool asciiOnly(const String& s) {
  for (unsigned int i = 0; i < s.length(); i++) {
    uint8_t c = (uint8_t)s[i];
    if (c < 32 || c > 126) return false;
  }
  return true;
}

// استخراج رشته‌های داخل "..." به ترتیب
int collectQuoted(const String& line, String* arr, int maxN) {
  int cnt = 0, from = 0;
  while (cnt < maxN) {
    int a = line.indexOf('"', from);
    if (a < 0) break;
    int b = line.indexOf('"', a + 1);
    if (b < 0) break;
    arr[cnt++] = line.substring(a + 1, b);
    from = b + 1;
  }
  return cnt;
}

// ===================== Config Load/Save =====================
void loadSim800Config() {
  preferences.begin("sim800", true);
  // [CHANGE] چک‌باکس‌های Enable SIM800 / Enable GPRS حذف شدند - هر دو همیشه فعال‌اند
  sim800Enabled   = true;
  sim800InternetEnabled = true;
  sim800APN       = preferences.getString("apn", "");
  sim800Baud      = preferences.getULong("baud", 0);   // [OPT-10] 0 = تشخیص خودکار در اولین init
  securityEnabled = preferences.getBool("secEn", false);
  for (int i = 0; i < MAX_PHONE_NUMBERS; i++) {
    String k = "ph" + String(i);
    phoneNumbers[i] = preferences.getString(k.c_str(), "");
  }
  for (int i = 0; i < SMS_CMD_COUNT; i++) smsCmd[i] = preferences.getString(CMD_KEYS[i], DEFAULT_CMDS[i]);
  preferences.end();
}

void saveSim800Config() {
  preferences.begin("sim800", false);
  preferences.putBool("enabled", sim800Enabled);
  preferences.putBool("netEn", sim800InternetEnabled);
  preferences.putString("apn", sim800APN);
  preferences.putULong("baud", sim800Baud);            // [OPT-10] بادریت فعال برای بوت بعدی
  preferences.putBool("secEn", securityEnabled);
  for (int i = 0; i < MAX_PHONE_NUMBERS; i++) {
    String k = "ph" + String(i);
    preferences.putString(k.c_str(), phoneNumbers[i]);
  }
  for (int i = 0; i < SMS_CMD_COUNT; i++) preferences.putString(CMD_KEYS[i], smsCmd[i]);
  preferences.end();
}

// ===================== Lock state (persistent) =====================
void saveLockState() {
  preferences.begin("carctrl", false);
  preferences.putBool("locked", carLocked);
  preferences.putBool("secLk", secLocked);
  preferences.putBool("svFuel", savedFuel);
  preferences.putBool("svSw", savedSwitch);
  preferences.end();
}

void loadLockState() {
  preferences.begin("carctrl", true);
  carLocked   = preferences.getBool("locked", false);
  secLocked   = preferences.getBool("secLk", false);
  savedFuel   = preferences.getBool("svFuel", true);
  savedSwitch = preferences.getBool("svSw", true);
  preferences.end();
  if (carLocked && secLocked) secState = 3;
  if (carLocked) Serial.println("Car is LOCKED (restored from memory)");
}

// lockDoors: آیا درب‌ها هم قفل شوند؟
// bySecurity: آیا قفل امنیتیِ قطع آنتن است؟ (secLocked را ست می‌کند)
void applyLock(bool lockDoors, bool bySecurity) {
  if (!carLocked) {
    savedFuel   = outputState[2];
    savedSwitch = outputState[3];
  }
  carLocked = true;
  if (bySecurity) secLocked = true;    // ← مهم برای تغییر 6

  desiredState[2] = false;
  desiredState[3] = false;
  setOutput(2, false);
  setOutput(3, false);

  if (lockDoors) setDoors(true);

  saveLockState();
  Serial.println(String(bySecurity ? "LOCK (security)" : "LOCK (command)") +
                 String(lockDoors ? " + doors" : ""));
}

// فرمان Unlock: فقط پمپ بنزین و سوئیچ روشن می‌شوند
void unlockCar() {
  carLocked = false;
  secLocked = false;

  setOutput(2, true);  desiredState[2] = true;
  setOutput(3, true);  desiredState[3] = true;

  // ← مورد 2: setDoors(false) حذف شد — Unlock هیچ پالسی روی درب نمی‌زند
  // (چون آلارم درب‌ها را قفل نکرده، پس Unlock هم نباید به درب‌ها دست بزند)

  safeZoneAlarmActive  = false;
  safeZoneCallIdx      = 0;
  safeZoneCallAnswered = false;
  safeZoneViolated     = false;
  safeZoneNoFixSince   = 0;

  alarmManual = false;
  updateBuzzer();

  secState      = 0;
  secGraceStart = 0;
  gsmBadSince   = 0;
  securitySuppressed = true;   // ← مورد 8: تا برگشت GSM از قفل امنیتی جلوگیری کن

  if (callActive) {
    if (locPhase == 0) sim800SendAT("ATH", "OK", 1500);
    callActive   = false;
    activeCaller = "";
  }

  // Keep periodic GPS SMS reporting active.

  saveLockState();
  Serial.println("UNLOCK: fuel+switch ON, all security reset");
}


void releaseLock(bool restore) {
  bool was = carLocked;
  carLocked = false;
  secLocked = false;
  if (was && restore) {
    if (savedFuel)   { setOutput(2, true); desiredState[2] = true; }
    if (savedSwitch) { setOutput(3, true); desiredState[3] = true; }
  }
  saveLockState();
  Serial.println("Lock released");
}

// ===================== Buzzer + Security =====================
bool authorizedClientConnected(){
  return sclPairedCount()>0 && sclLastAuthorizedAt>0 && (uint32_t)(millis()-sclLastAuthorizedAt)<30000UL && WiFi.softAPgetStationNum()>0;
}

bool gsmHealthy() {
  return sim800Enabled && sim800Ready && gprsRegistered && gsmCSQ != 99;
}

void updateBuzzer() {
  static bool last = false;
  bool on = alarmManual || (secState == 1 && ((millis() / 500) % 2 == 0));
  if (on != last) {
    digitalWrite(PIN_BUZZER, on ? BUZZER_ON_LEVEL : BUZZER_OFF_LEVEL);
    last = on;
  }
}

// 0=عادی  1=مهلت (بازر)  2=کاربر وصل شد (تا برگشت آنتن)  3=قفل امنیتی
void updateSecurity() {
  static unsigned long last = 0;
  if (millis() - last < 1000) return;
  last = millis();

  if (!securityEnabled) {
    if (secState == 3 && secLocked) releaseLock(true);
    secState    = 0;
    gsmBadSince = 0;
    return;
  }

  // ← جدید (مورد 8): پس از Unlock دستی، تا برگشت GSM امنیت غیرفعال بماند
  if (securitySuppressed) {
    if (gsmHealthy()) {
      securitySuppressed = false;
      Serial.println("SECURITY: GSM back - suppression lifted");
    } else {
      secState = 0;
      gsmBadSince = 0;
      return;
    }
  }

  bool ok   = gsmHealthy();
  bool user = authorizedClientConnected();

  switch (secState) {
    case 0:
      if (ok) {
        gsmBadSince = 0;
      } else {
        if (gsmBadSince == 0) gsmBadSince = millis();
        else if (millis() - gsmBadSince >= SEC_DEBOUNCE_MS) {
          if (user) {
            secState = 2;
          } else {
            secState = 1;
            secGraceStart = millis();
            applyLock(false, false);   // فقط قطع پمپ+سوئیچ، بدون قفل درب، secLocked ست نمی‌شود
            updateBuzzer();
            Serial.println("SECURITY: GSM lost - grace, fuel+switch CUT, buzzer blinking");
          }
        }
      }
      break;

    case 1:
      if (ok) {
        secState = 0;
        gsmBadSince = 0;
        if (secLocked) releaseLock(true);
        Serial.println("SECURITY: GSM back - grace cancelled");
      } else if (user) {
        secState = 2;
        if (secLocked) releaseLock(true);
        Serial.println("SECURITY: user connected - grace cancelled");
      } else if (millis() - secGraceStart >= SEC_GRACE_MS) {
        applyLock(true, true);   // doors locked + secLocked set
        secState = 3;
        Serial.println("SECURITY: grace expired - LOCKED (doors+fuel+switch)");
      }
      break;

    case 2:
      if (ok) { secState = 0; gsmBadSince = 0; }
      break;

    case 3:
      if (ok || user) {
        if (secLocked) releaseLock(true);
        secState = ok ? 0 : 2;
        gsmBadSince = 0;
        Serial.println("SECURITY: released");
      }
      break;
  }
}

// ===================== URC queue / line helpers =====================
void urcPush(const String& l) {
  uint8_t n = (urcTail + 1) % URC_QUEUE_SIZE;
  if (n == urcHead) return;
  urcQueue[urcTail] = l;
  urcTail = n;
}

bool urcPop(String& l) {
  if (urcHead == urcTail) return false;
  l = urcQueue[urcHead];
  urcQueue[urcHead] = "";
  urcHead = (urcHead + 1) % URC_QUEUE_SIZE;
  return true;
}

bool isUrcLine(const String& l) {
  return l == "RING" || l.startsWith("+CLIP:") || l.startsWith("+CMTI:") ||
         l == "NO CARRIER" || l == "BUSY" || l == "NO ANSWER" ||
         l.startsWith("+PDP: DEACT");
}

// [OPT-3] پردازش مشترک خطوط +CREG / +CGREG (هم فرم URC «+CREG: 1» هم فرم پاسخ «+CREG: 0,1»)
// نتیجه: به‌محض وصل شدن به شبکه، پرچم‌ها همان لحظه آپدیت می‌شوند؛
// دیگر لازم نیست تا چرخه‌ی ۶۰ ثانیه‌ای بعدی برای دیدن وضعیت شبکه صبر کنیم.
bool sim800ParseRegLine(const String& line) {
  bool creg  = line.startsWith("+CREG:");
  bool cgreg = line.startsWith("+CGREG:");
  if (!creg && !cgreg) return false;
  int c = line.indexOf(',');
  int st = (c >= 0) ? line.substring(c + 1).toInt() : line.substring(7).toInt();
  bool reg = (st == 1 || st == 5);
  if (creg) {
    gprsRegistered = reg;
  } else {
    gprsAttached = reg;
    if (reg) gprsRegistered = true;
  }
  // [OPT-5] با وصل شدن شبکه، اجازه‌ی warm-up فوری حامل اینترنت (SAPBR) صادر می‌شود
  if (reg) sim800LastBearerTry = 0;
  return true;
}

String sim800CleanAndQueue(const String& raw) {
  String out = "";
  int pos = 0, len = raw.length();
  while (pos < len) {
    int nl = raw.indexOf('\n', pos);
    String line;
    if (nl < 0) { line = raw.substring(pos); pos = len; }
    else        { line = raw.substring(pos, nl); pos = nl + 1; }
    line.trim();
    if (line.length() == 0) continue;

    if (mqttOnModemLine(line)) continue;
    if (line.startsWith("+CMGS:"))      { g_smsOk   = true; continue; }
    if (line.startsWith("+CMS ERROR"))  { g_smsFail = true; continue; }
    // [OPT-3] ثبت فوری شبکه (پرچم‌های CREG/CGREG همان لحظه آپدیت می‌شوند)
    sim800ParseRegLine(line);   // [OPT-3] فقط پرچم‌ها را آپدیت می‌کند؛ خط در پاسخ باقی می‌ماند
    if (isUrcLine(line)) urcPush(line);
    else out += line + "\n";
  }
  return out;
}

void locFinish(const String& msg) {
  locResult = msg;
  locDone = true;
  locPhase = 0;
  locWanted = false;

  // [PATCH 1] درخواست از طریق SMS: بلافاصله پاسخ بده (نه منتظر چرخه 10 ثانیه‌ای)
  if (locRequester.startsWith("sms:")) {
    String to = (locSmsTargetNumber.length() > 0) ? locSmsTargetNumber
                                                  : locRequester.substring(4);
    // ارسال فوری؛ اگر الآن ممکن نبود (مثلاً تماس فعال)، حلقه‌ی ۱۰ ثانیه‌ای تحویلش می‌دهد
    if (sim800Ready && to.length() > 0 && !callActive) {
      if (sendSms(to, msg)) {
        locSmsLastSend = millis();   // جلوگیری از ارسال فوری دوباره در حلقه
        locResult = "";              // [PATCH 5] تحویل شد؛ در حلقه تکرار نشود
      }
    }
    locRequester = "";
  }

}

void bearerFail(const String& msg) {
  Serial.print("SIM800: "); Serial.println(msg);
  bearerLastFail = millis();
  sim800IP = "";
  if (locWanted) locFinish(msg);
  else locPhase = 0;
  locWanted = false;
}

// خطوطی که فرمان همزمان ندارد: URC یا پاسخ مراحل اینترنت/مکان‌یابی
void sim800ProcessLine(const String& line) {
  if (mqttOnModemLine(line)) return;
  if (isUrcLine(line)) { urcPush(line); return; }
  if (line.startsWith("AT")) return;
  // [OPT-3] URC های ثبت شبکه: +CREG: <stat> / +CGREG: <stat>
  if (sim800ParseRegLine(line)) return;
  if (line.startsWith("+CMGS:"))      { g_smsOk   = true; return; }
  if (line.startsWith("+CMS ERROR"))  { g_smsFail = true; return; }

  if (locPhase == 1) {                       // پاسخ AT+SAPBR=1,1
    if (line == "OK") {
      sim800Serial.print("AT+SAPBR=2,1\r");
      locPhase = 3;
      locStart = millis();
    } else if (line.indexOf("ERROR") >= 0) {
      bearerFail("No internet (GPRS bearer failed)");
    }
  } else if (locPhase == 3) {                // پاسخ AT+SAPBR=2,1
    if (line.startsWith("+SAPBR:")) {
      int c1 = line.indexOf(',');
      int c2 = (c1 >= 0) ? line.indexOf(',', c1 + 1) : -1;
      int st = (c1 >= 0) ? line.substring(c1 + 1, c2 > 0 ? c2 : line.length()).toInt() : 0;
      int q1 = line.indexOf('"');
      int q2 = (q1 >= 0) ? line.indexOf('"', q1 + 1) : -1;
      String ip = (q1 >= 0 && q2 > q1) ? line.substring(q1 + 1, q2) : "";
      if (st == 1 && ip.length() > 0 && ip != "0.0.0.0") {
        sim800IP = ip;
        Serial.println("SIM800: internet connected");
        locPhase = 0;
        locWanted = false;
      } else {
        bearerFail("No internet (no IP from operator)");
      }
    } else if (line.indexOf("ERROR") >= 0) {
      bearerFail("No internet");
    }
  }
}

void sim800PumpRx() {
  while (sim800Serial.available()) {
    char c = (char)sim800Serial.read();

    if (c == '\n') {
      String line = sim800RxBuf;
      sim800RxBuf = "";
      line.trim();
      if (line.length() == 0) continue;

      sim800ProcessLine(line);
    } else if (c != '\r') {
      sim800RxBuf += c;
      if (sim800RxBuf.length() > 512) sim800RxBuf = "";
    }
  }
}

String sim800Exchange(const String& cmd, unsigned long timeout) {
  if (mqttWaiting) return ""; // never consume another command's MQTT response
  sim800PumpRx();
  String resp = sim800RxBuf;
  sim800RxBuf = "";

  sim800Serial.print(cmd);
  sim800Serial.print('\r');

  unsigned long start = millis();
  while (millis() - start < timeout) {
    while (sim800Serial.available()) resp += (char)sim800Serial.read();
    // قبل: if (resp.endsWith("OK\r\n") || resp.indexOf("ERROR") >= 0) break;
    if (resp.indexOf("OK\r\n") >= 0 || resp.indexOf("ERROR") >= 0) break;   // ← اصلاح
    delay(5);
  }
  return sim800CleanAndQueue(resp);
}

bool sim800SendAT(String cmd, String expected, unsigned long timeout) {
  String r = sim800Exchange(cmd, timeout);
  if (expected.length() == 0) return true;
  return r.indexOf(expected) >= 0;
}

bool sim800WaitFor(const char* token, unsigned long timeout, String* out) {
  String buf = "";
  unsigned long start = millis();
  bool found = false;
  while (millis() - start < timeout) {
    while (sim800Serial.available()) buf += (char)sim800Serial.read();
    if (buf.indexOf(token) >= 0) { found = true; break; }
    if (buf.indexOf("ERROR") >= 0) break;
    delay(5);
  }
  if (out) *out = buf;
  return found;
}

bool sim800Probe() {
  for (int i = 0; i < 5; i++) {
    if (sim800SendAT("AT", "OK", 500)) return true;
    delay(100);
  }
  return false;
}

// ===================== SMS parsing =====================

bool smsPduSeen = false;     // ماژول پیام را به صورت PDU (هگز) داده است

int pduHex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool looksLikePdu(const String& s) {
  if (s.length() < 20 || (s.length() & 1)) return false;
  for (unsigned int i = 0; i < s.length(); i++) if (pduHex(s[i]) < 0) return false;
  return true;
}

char gsm7Char(int v) {
  if (v == 0x00) return '@';
  if (v == 0x02) return '$';
  if (v == 0x11) return '_';
  if (v == 0x0A || v == 0x0D) return (char)v;
  if (v >= 0x20 && v <= 0x7E) {
    if (v == 0x24 || v == 0x40 || (v >= 0x5B && v <= 0x60) || (v >= 0x7B && v <= 0x7E)) return '?';
    return (char)v;
  }
  return '?';
}

char gsm7Ext(int v) {
  switch (v) {
    case 0x14: return '^';  case 0x28: return '{';  case 0x29: return '}';
    case 0x2F: return '\\'; case 0x3C: return '[';  case 0x3D: return '~';
    case 0x3E: return ']';  case 0x40: return '|';
    default: return '?';
  }
}

// باز کردن septet های 7-بیتی GSM
String gsm7Unpack(const uint8_t* d, int avail, int septets, int skip) {
  String t = "";
  bool esc = false;
  for (int i = 0; i < septets; i++) {
    int bit = i * 7, bi = bit / 8, sh = bit % 8;
    if (bi >= avail) break;
    int v = d[bi] >> sh;
    if (sh > 1 && bi + 1 < avail) v |= d[bi + 1] << (8 - sh);
    v &= 0x7F;
    if (i < skip) continue;
    if (esc) { esc = false; t += gsm7Ext(v); }
    else if (v == 0x1B) esc = true;
    else t += gsm7Char(v);
  }
  return t;
}

void appendUtf8(String& t, uint16_t cp) {
  if (cp < 0x80) t += (char)cp;
  else if (cp < 0x800) { t += (char)(0xC0 | (cp >> 6)); t += (char)(0x80 | (cp & 0x3F)); }
  else { t += (char)(0xE0 | (cp >> 12)); t += (char)(0x80 | ((cp >> 6) & 0x3F)); t += (char)(0x80 | (cp & 0x3F)); }
}

// رمزگشایی PDU از نوع SMS-DELIVER
bool pduDecode(String pdu, String& from, String& date, String& text) {
  pdu.trim();
  pdu.toUpperCase();
  if (!looksLikePdu(pdu)) return false;
  int nb = pdu.length() / 2;
  if (nb > 200) return false;
  uint8_t b[200];
  for (int i = 0; i < nb; i++) b[i] = (uint8_t)(pduHex(pdu[2 * i]) * 16 + pduHex(pdu[2 * i + 1]));

  int p = 0;
  int smsc = b[p++];
  p += smsc;
  if (p + 3 >= nb) return false;
  int first = b[p++];
  if ((first & 0x03) != 0) return false;                 // فقط SMS-DELIVER
  bool udhi = (first & 0x40) != 0;
  int oaLen = b[p++];
  int oaType = b[p++];
  int oaBytes = (oaLen + 1) / 2;
  if (p + oaBytes + 10 > nb) return false;

  String num = "";
  if ((oaType & 0x70) == 0x50) {                         // فرستنده الفبایی
    num = gsm7Unpack(b + p, oaBytes, (oaLen * 4) / 7, 0);
  } else {
    for (int i = 0; i < oaBytes; i++) {
      int lo = b[p + i] & 0x0F, hi = b[p + i] >> 4;
      if ((int)num.length() < oaLen) num += (lo <= 9) ? (char)('0' + lo) : (lo == 0x0A ? '*' : '#');
      if ((int)num.length() < oaLen) num += (hi <= 9) ? (char)('0' + hi) : (hi == 0x0A ? '*' : '#');
    }
    if ((oaType & 0x70) == 0x10) num = "+" + num;
  }
  p += oaBytes;
  p++;                                                    // PID
  int dcs = b[p++];

  String d = "";
  for (int i = 0; i < 7; i++) { d += (char)('0' + (b[p + i] & 0x0F)); d += (char)('0' + (b[p + i] >> 4)); }
  date = d.substring(0, 2) + "/" + d.substring(2, 4) + "/" + d.substring(4, 6) + "," +
         d.substring(6, 8) + ":" + d.substring(8, 10) + ":" + d.substring(10, 12);
  p += 7;

  int udl = b[p++];
  if (p > nb) return false;

  String t = "";
  if ((dcs & 0x0C) == 0x08) {                             // UCS2
    for (int i = 0; i + 1 < udl && p + i + 1 < nb; i += 2) appendUtf8(t, (uint16_t)((b[p + i] << 8) | b[p + i + 1]));
  } else if ((dcs & 0x0C) == 0x04) {                      // 8-bit
    for (int i = 0; i < udl && p + i < nb; i++) t += (char)b[p + i];
  } else {                                                // 7-bit
    int skip = 0;
    if (udhi) { int udhl = b[p]; skip = ((udhl + 1) * 8 + 6) / 7; }
    t = gsm7Unpack(b + p, nb - p, udl, skip);
  }

  from = num;
  text = t;
  return true;
}

// اگر فرستنده خالی و متن شبیه PDU بود، رمزگشایی کن
bool tryDecodePdu(String& from, String& date, String& text) {
  if (from.length() > 0) return false;
  String t = text;
  t.trim();
  String f, d, x;
  if (!pduDecode(t, f, d, x)) return false;
  from = f; date = d; text = x;
  return true;
}


// خروجی AT+CMGL="ALL":  +CMGL: <idx>,"<st>","<from>","<alpha>","<date>"\n<text>
int simParseCmgl(const String& r, SimMsg* out, int maxN) {
  int n = -1;
  int pos = 0, len = r.length();
  while (pos < len) {
    int nl = r.indexOf('\n', pos);
    String line;
    if (nl < 0) { line = r.substring(pos); pos = len; }
    else        { line = r.substring(pos, nl); pos = nl + 1; }
    line.trim();
    if (line.length() == 0) continue;

    if (line.startsWith("+CMGL:")) {
      n++;
      if (n >= maxN) { n = maxN - 1; break; }
      int comma = line.indexOf(',');
      out[n].idx = line.substring(6, comma > 0 ? comma : line.length()).toInt();
      String q[6];
      int c = collectQuoted(line, q, 6);
      out[n].st   = (c > 0) ? q[0] : "";
      out[n].from = (c > 1) ? q[1] : "";
      out[n].date = (c > 2) ? q[c - 1] : "";
      out[n].text = "";
    } else if (line == "OK") {
      // پایان
    } else if (n >= 0) {
      if (out[n].text.length() > 0) out[n].text += "\n";
      out[n].text += line;
    }
  }
  for (int i = 0; i <= n && i < maxN; i++) {
    if (tryDecodePdu(out[i].from, out[i].date, out[i].text)) smsPduSeen = true;
  }
  return n + 1;
}

// ===================== SMS config (CNMI/CPMS/CMGF) =====================
bool sim800ConfigureSms() {
  bool ok = true;
  ok &= sim800SendAT("AT+CMGF=1", "OK", 1000);          // حالت متنی (نه PDU)
  sim800SendAT("AT+CSCS=\"GSM\"", "OK", 1000);
  sim800SendAT("AT+CPMS=\"SM\",\"SM\",\"SM\"", "OK", 2500);
  ok &= sim800SendAT("AT+CSMP=17,167,0,0", "OK", 1000); // [PATCH 4] پارامترهای SMS (بسیاری از ماژول‌های SIM800L بدون این فرمان SMS نمی‌فرستند)
  ok &= sim800SendAT("AT+CNMI=2,1,0,0,0", "OK", 1000);   // اعلان +CMTI برای هر پیام جدید
  return ok;
}

void sim800VerifySmsConfig() {
  if (!sim800Ready || callActive || locPhase != 0) return;
  String m = sim800Exchange("AT+CMGF?", 1200);
  String r = sim800Exchange("AT+CNMI?", 1200);
  // بعض فریمورها پاسخ را با فاصله و بعضی بدون فاصله پس از ':' می‌دهند.
  m.replace(" ", "");
  r.replace(" ", "");
  bool textMode = (m.indexOf("+CMGF:1") >= 0);
  bool notifyMode = (r.indexOf("+CNMI:2,1") >= 0);
  if (!textMode || !notifyMode) {
    Serial.println("SIM800: SMS config mismatch (CMGF=" + String(textMode ? "OK" : "BAD") +
                   ", CNMI=" + String(notifyMode ? "OK" : "BAD") + ") - reconfiguring once");
    sim800ConfigureSms();
  }
}

// ===================== Init =====================
void powerOnSim800() {
  Serial.println("SIM800: power key pulse");
  pinMode(SIM800_PWK_PIN, OUTPUT);
  digitalWrite(SIM800_PWK_PIN, HIGH);
  delay(100);
  digitalWrite(SIM800_PWK_PIN, LOW);
  delay(1200);
  digitalWrite(SIM800_PWK_PIN, HIGH);
  // [OPT-8] به‌جای انتظار ثابت ۳ ثانیه، به‌محض اولین پاسخ AT ادامه بده (حداکثر ۸ ثانیه)
  unsigned long st = millis();
  while (millis() - st < 8000UL) {
    if (sim800SendAT("AT", "OK", 400)) return;
    delay(200);
  }
}

bool sim800HandleMessage(const String& from, const String& text);

// پیامک‌های موجود روی سیم‌کارت (قبل از بوت) را در حافظه‌ی ESP32 ذخیره و از سیم‌کارت پاک می‌کند (اجرا نمی‌شوند)
void sim800DrainSim() {
  // پاک‌سازی همه پیام‌های قدیمی روی سیم‌کارت (بدون خواندن، بدون ذخیره، بدون اجرا)
  // Never bulk-delete received SMS. Pending commands survive a reboot.
  // Old SMS remain stored and are processed by the periodic poll after initialization.
}

void setupSim800() {
  if (!sim800Enabled) return;

  sim800LastInit = millis();
  Serial.println("SIM800: initializing...");

  if (!sim800SerialStarted) {
    pinMode(SIM800_PWK_PIN, OUTPUT);
    digitalWrite(SIM800_PWK_PIN, HIGH);
    pinMode(SIM800_RI_PIN, INPUT_PULLUP);
    sim800Serial.setRxBufferSize(2048);   // [OPT-10] بافر بزرگ‌تر برای بادریت بالاتر (قبلاً 1024)
    sim800SerialStarted = true;
  }

  // [OPT-10] تشخیص خودکار بادریت: اول بادریت ذخیره‌شده، بعد 115200، در نهایت 9600.
  // اگر ماژول (مثلاً بعد از قطع برق) به autobaud برگشته باشد، ارسال مکرر «AT» نرخ را سنکرون می‌کند.
  // اگر ماژول روی نرخ دیگری قفل باشد، بی‌پاسخ می‌مانیم و به گزینه‌ی بعدی می‌رویم — بدون ریسک.
  bool alive = false;
  {
    unsigned long cand[3] = { sim800Baud, SIM800_BAUD_FAST, SIM800_BAUD_DEFAULT };
    for (int i = 0; i < 3 && !alive; i++) {
      if (cand[i] == 0) continue;
      bool dup = false;
      for (int j = 0; j < i; j++) if (cand[j] == cand[i]) dup = true;
      if (dup) continue;
      sim800Serial.begin(cand[i], SERIAL_8N1, SIM800_RX_PIN, SIM800_TX_PIN);
      sim800Serial.setTimeout(200);
      delay(300);                       // [OPT-8] قبلاً 1500ms ثابت
      alive = sim800Probe();
      if (alive) {
        if (sim800Baud != cand[i]) {
          sim800Baud = cand[i];
          saveSim800Config();           // برای بوت بعدی ذخیره شود
          Serial.println("SIM800: baud locked @ " + String(sim800Baud));
        }
        // برای نرخ‌های بالا ماژول روی همین نرخ قفل می‌شود تا autobaud وسط کار نلغزد
        if (cand[i] != SIM800_BAUD_DEFAULT) sim800SendAT("AT+IPR=" + String(cand[i]), "OK", 1000);
      }
    }
  }

  if (!alive && (!sim800PowerTried || sim800InitFails >= 3)) {
    sim800PowerTried = true;
    sim800InitFails = 0;
    powerOnSim800();
    alive = sim800Probe();
  }

  if (!alive) {
    sim800InitFails++;
    Serial.println("SIM800: no response (check power 4V/2A, wiring, baud)");
    return;
  }

  sim800SendAT("ATE0",       "OK", 1000);
  sim800SendAT("AT+CMEE=1",  "OK", 1000);
  // [OPT-3] حالت URC ثبت شبکه: به‌محض وصل/قطع شدن، +CREG/+CGREG خودش می‌آید؛
  // دیگر لازم نیست تا ۶۰ ثانیه برای چرخه‌ی بعدی پولینگ صبر کنیم تا وصل شدن دیده شود
  sim800SendAT("AT+CREG=1",  "OK", 1000);
  sim800SendAT("AT+CGREG=1", "OK", 1000);
  sim800SendAT("AT+CSCLK=0", "OK", 1000);
  sim800SendAT("AT+CMIC=0,15", "OK", 1000);   // gain میکروفن
  sim800SendAT("AT+CLIP=1",  "OK", 1000);

  // منتظر آماده شدن سیم‌کارت (تنظیمات پیامک قبل از آن ممکن است رد شود)
  bool simReady = false;
  for (int i = 0; i < 12 && !simReady; i++) {
    String pin = sim800Exchange("AT+CPIN?", 1200);
    simReady = (pin.indexOf("READY") >= 0);
    if (!simReady) delay(500);   // [OPT-8] قبلاً 1000ms — سیم‌کارت زودتر آماده می‌شود
  }
  Serial.print("SIM800 SIM status: "); Serial.println(simReady ? "READY" : "NOT READY");

  // [PATCH C4] AT+CLIP=1 was originally sent BEFORE the SIM was READY and its result
  // was never checked. If it silently failed there is no +CLIP URC at all and the
  // firmware can never decide answer/reject. Re-apply and verify now that SIM is READY.
  if (simReady) {
    if (!sim800SendAT("AT+CLIP=1", "OK", 1000)) {
      Serial.println("SIM800: WARNING AT+CLIP=1 failed -> caller ID unavailable, incoming calls cannot be authorized");
    }
  }

  bool smsOk = false;
  for (int i = 0; i < 3 && !smsOk; i++) {
    smsOk = sim800ConfigureSms();
    if (!smsOk) delay(500);
  }
  String csca = sim800Exchange("AT+CSCA?", 1000);
  csca.replace("\n", " "); csca.trim();
  Serial.println("SIM800 SMSC checked");
  Serial.println(smsOk ? "SIM800: SMS configured" : "SIM800: SMS config incomplete (will retry)");

  if (!smsDrained) {
    sim800DrainSim();
    smsDrained = true;
  }

  sim800Ready = true;
  sim800InitFails = 0;
  sim800NoReply = 0;
  locPhase = 0;
  Serial.println("SIM800: ready");
  updateSim800Status();
  updateGsmSignal();
}

// ===================== Main update =====================
void updateSim800() {
  if (!sim800Enabled) {
    if (sim800Ready) { sim800Ready = false; locPhase = 0; }
    return;
  }

  if (!sim800Ready) {
    if (sim800ForceInit || millis() - sim800LastInit > 30000UL) {
      sim800ForceInit = false;
      setupSim800();
    }
    return;
  }

  sim800PumpRx();
  // MQTT owns the AT response channel until its in-flight command completes.
  // URCs are buffered and processed by the normal SIM800 path afterwards.
  if (mqttWaiting) return;

    if (locDone) {
    locDone = false;
    Serial.println("Location request completed");
    locRequester = "";
  }

  if (locPhase != 0) {
    unsigned long lim = (locPhase == 1) ? 90000UL : 60000UL;
    if (millis() - locStart > lim) {
      if (locPhase == 2) locFinish("Location timeout");
      else bearerFail("No internet (timeout)");
    }
  }

  bool busy = (locPhase != 0);

  // [PATCH C5] While GPRS bearer is busy, call state must stay healthy:
  //  - NO CARRIER / BUSY / NO ANSWER: clear callActive immediately (no AT command sent)
  //  - +CLIP از شماره‌ی «مجاز»: [OPT-7] مکان‌یابی لغو و تماس در همان چرخه پاسخ داده می‌شود
  //    (قبلاً تماس رد می‌شد و تا ۹۰ ثانیه بی‌پاسخ می‌ماند!)
  //  - RING / +CLIP غیرمجاز یا نامعتبر: مثل قبل، زنگ ادامه دارد و کاری نمی‌شود
  //  - all other URCs are kept for the normal cycle
  if (busy) {
    String keep[URC_QUEUE_SIZE]; int keepN = 0; String cl; int cg = 0;
    while (cg++ < 16 && urcPop(cl)) {
      if (cl == "NO CARRIER" || cl == "BUSY" || cl == "NO ANSWER") {
        handleSim800URC(cl);
      } else if (cl.startsWith("+CLIP:") && !callActive) {
        // [OPT-7] تماس مجاز در حین LBS: مکان‌یابی لغو می‌شود تا تماس سریع وصل شود
        int q1 = cl.indexOf('"');
        int q2 = (q1 >= 0) ? cl.indexOf('"', q1 + 1) : -1;
        String caller = (q1 >= 0 && q2 > q1) ? cl.substring(q1 + 1, q2) : "";
        if (caller.length() > 0 && isPhoneAllowed(caller)) {
          Serial.println("SIM800: authorized call during GPRS -> aborting location, answering call");
          locPhase = 0; locWanted = false; locDone = false; locResult = ""; locRequester = "";
          locFastRetry = false;
          if (keepN < URC_QUEUE_SIZE) keep[keepN++] = cl;   // برای پاسخ‌گویی در همین چرخه
        }
      } else if (keepN < URC_QUEUE_SIZE) {
        keep[keepN++] = cl;
      }
    }
    for (int i = 0; i < keepN; i++) urcPush(keep[i]);
  }

  busy = (locPhase != 0);   // [OPT-7] ممکن است بعد از لغو LBS تغییر کند
  // [OPT-11] بلوک تکراری قبلی (پردازش دوباره‌ی URC ها و فرمان‌های منتظر در دو if جدا) حذف شد

    if (!busy) {
    // URC ها
    String l;
    int guard = 0;
    while (guard++ < 16 && locPhase == 0 && urcPop(l)) handleSim800URC(l);

    // دستورات منتظر
    if (locPhase == 0 && pendCount > 0) {
      String f = pendFrom[0], b = pendBody[0];
      for (int i = 1; i < pendCount; i++) { pendFrom[i-1] = pendFrom[i]; pendBody[i-1] = pendBody[i]; }
      pendCount--;
      processSMSCommand(f, b);
    }

    // ← تغییر 10: ارسال SMS آلارم به همه شماره‌های مجاز
    if (pendingAlarmSms && sim800Ready && locPhase == 0 && !callActive) {
      pendingAlarmSms = false;
      String msg = "ALARM: " + pendingAlarmSmsReason;
      String loc = gpsAvailable() ? gpsMapUrl() : "";
      if (loc.length() > 0) msg += "\nLocation: " + loc;
      for (int i = 0; i < MAX_PHONE_NUMBERS; i++) {
        if (phoneNumbers[i].length() >= 7) {
          sendSms(phoneNumbers[i], msg);
        }
      }
    }

    // ← جدید: Status که هنگام تماس/GPRS درخواست شده بود، بعد از آزاد شدن فقط یک‌بار ارسال شود
    if (pendingStatusSms && sim800Ready && locPhase == 0 && !callActive) {
      pendingStatusSms = false;
      Serial.println("Deferred status sending");
      sendStatusSms(pendingStatusFrom);
    }

    // ← تغییر 7: حلقه ارسال SMS لوکیشن با فاصله‌ی قابل تنظیم (فرمان: Location <عدد 1..120 دقیقه>)
    // [PATCH 5] نتیجه‌ی GPS فقط یک‌بار ارسال می‌شود (locResult بعد از ارسال پاک می‌شود)
    // GPS SMS loop is stopped only by an explicit Stop location command.
    if (locSmsActive && sim800Ready && locPhase == 0 && !callActive) {
      if (locSmsLastSend == 0 || millis() - locSmsLastSend >= locSmsIntervalMs) {
        locSmsLastSend = millis();
        String msg;
        if (gpsAvailable()) {
          msg = "Location: " + gpsMapUrl();
        } else {
          msg = "Location: GPS fix unavailable";
        }
        locSmsSendCount++;
        Serial.println("LOCATION SMS #" + String(locSmsSendCount) + " (interval " + String(locSmsIntervalMs / 60000UL) + " min)");
        sendSms(locSmsTargetNumber, msg);
        if (!gpsAvailable()) {
          locResult = "";                  // [PATCH 5] یک‌بار مصرف: نتیجه‌ی کهنه دوباره نفرست
        }
      }
    }

  // [OPT-4] بازه‌ی پویا: تا وقتی شبکه ثبت نشده هر ۳ ثانیه CREG/CGREG/COPS پرسیده می‌شود
  // تا ثبت شبکه «همان لحظه» دیده شود (قبلاً تا ۶۰ ثانیه تأخیر)؛ بعد از ثبت، هر ۶۰ ثانیه.
  // [OPT-12] شرط locPhase==0: پولینگ وضعیت/سیگنال دیگر پاسخ‌های SAPBR را
  // نمی‌بلعد (باگ قبلی: CSQ هر ۲۰ ثانیه می‌توانست پاسخ LBS را بخورد → گیر کردن Location تا ۶۰-۹۰s)
  if (locPhase == 0 && millis() - sim800LastStatus > (gprsRegistered ? 60000UL : 3000UL)) { sim800LastStatus = millis(); updateSim800Status(); }
  if (sim800Ready && locPhase == 0 && millis() - sim800LastSignal > (gprsRegistered ? 20000UL : 5000UL)) { sim800LastSignal = millis(); updateGsmSignal(); }
  if (sim800Ready && locPhase == 0 && millis() - sim800LastSmsPoll > 8000UL) { sim800LastSmsPoll = millis(); sim800PollSms(); }  // 8s → 60s
if (sim800Ready && locPhase == 0 && millis() - sim800LastSmsVerify > 300000UL) { sim800LastSmsVerify = millis(); sim800VerifySmsConfig(); sim800CheckStorage(); }  // 2min → 5min

    if (sim800InternetEnabled && sim800Ready && locPhase == 0 && gprsRegistered && gprsAttached && sim800APN.length() > 0 &&
        sim800IP.length() == 0 &&
        (bearerLastFail == 0 || millis() - bearerLastFail > 60000UL) &&
        (sim800LastBearerTry == 0 || millis() - sim800LastBearerTry > 60000UL)) {
      startBearer(false);
    }
  }

    // فقط تماس‌های غیر safe-zone را با تایم‌اوت ببند.
// تماس Safe-Zone چه پاسخ‌داده‌شده چه در حال زنگ، توسط updateSafeZoneCalls مدیریت می‌شود.
if (callActive && !safeZoneAlarmActive && millis() - callStart > MAX_CALL_DURATION_MS) {
    if (locPhase == 0) sim800SendAT("ATH", "OK", 1500);
    callActive = false;
    activeCaller = "";
    Serial.println("SIM800: call timeout");
}
}

// ===================== Internet (GPRS bearer) + GPS-only Location =====================
void startBearer(bool wantLoc) {
  if (!sim800InternetEnabled) {
    if (wantLoc && locRequester.length() > 0) locFinish("Internet disabled in SIM800 settings");
    return;
  }
  locWanted = wantLoc;
  locStart = millis();
  sim800LastBearerTry = millis();

  // GPRS bearer only; no cell-tower location requests.
  locWanted = false;
  locFastRetry = false;
  sim800Exchange("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", 1500);
  sim800Exchange("AT+SAPBR=3,1,\"APN\",\"" + sim800APN + "\"", 1500);

  String r = sim800Exchange("AT+SAPBR=2,1", 1500);
  int i = r.indexOf("+SAPBR:");
  if (i >= 0 && r.indexOf(",1,", i) > 0) {
    int q1 = r.indexOf('"', i);
    int q2 = (q1 >= 0) ? r.indexOf('"', q1 + 1) : -1;
    if (q1 >= 0 && q2 > q1) sim800IP = r.substring(q1 + 1, q2);
    locWanted = false;
    locPhase = 0;
    return;
  }

  sim800Serial.print("AT+SAPBR=1,1\r");
  locPhase = 1;
}

void startLocation(const String& who) {
  locRequester = who;
  locResult = gpsAvailable() ? ("Location: " + gpsMapUrl()) : "Location: GPS fix unavailable";
  locDone = true;
  // SMS loop delivers GPS coordinates on its own schedule.
}

// ===================== URC handling =====================
void handleSim800URC(String line) {
  if (line == "RING") {
    ringDetected = true;
    Serial.println("SIM800: RING");   // [PATCH C2] make incoming rings visible in Serial Monitor
  } else if (line.startsWith("+CLIP:")) {
    int q1 = line.indexOf('"');
    int q2 = line.indexOf('"', q1 + 1);
    if (q1 >= 0 && q2 > q1) handleIncomingCall(line.substring(q1 + 1, q2));
  } else if (line.startsWith("+CMTI:")) {
    int comma = line.lastIndexOf(',');
    if (comma >= 0) {
      int idx = line.substring(comma + 1).toInt();
      Serial.println("SIM800: new SMS notification");
      if (idx > 0) handleIncomingSMS(idx);
    }
  } else if (line == "NO CARRIER" || line == "BUSY" || line == "NO ANSWER") {
    bool wasActive = callActive;
    callActive = false;
    activeCaller = "";
    ringDetected = false;
    // شروع تایمر فاصله تا شماره بعدی در updateSafeZoneCalls
    if (wasActive) safeZoneCallStart = millis();
    Serial.println("SAFE-ZONE: call ended by SIM800 (NO CARRIER/BUSY/NO ANSWER) -> next");
} else if (line.startsWith("+PDP: DEACT")) {
    sim800IP = "";
    sim800LastBearerTry = 0;
    bearerLastFail = 0;
  }
}

void handleIncomingCall(String caller) {
  if (callActive) return;

  // [PATCH C1] No usable caller ID received -> NEVER reject the call.
  // Rejecting (ATH) is what produces the "busy" tone the caller hears.
  // If the number is empty/garbled, just let the phone ring it out.
  String nrm = normalizePhone(caller);
  if (nrm.length() < 7) {
    Serial.println("SIM800: incoming call without usable caller ID");
    return;
  }

  // ← تغییر 1: در حالت آلارم تماس رد نمی‌شود
  // (فقط ثبت می‌شود، تصمیم پاسخ/رد بر اساس مجاز بودن شماره)
  if (safeZoneAlarmActive) {
    Serial.println("SAFE-ZONE: incoming call under evaluation");
  }

  if (isPhoneAllowed(caller)) {
    Serial.println("SIM800: answering authorized call");
    delay(100);   // [OPT-8] قبلاً 300ms — ۲۰۰ms سریع‌تر وصل شدن تماس
    bool aok = sim800SendAT("ATA", "OK", 3000);                        // [PATCH C3] gate on real result
    if (!aok) { delay(400); aok = sim800SendAT("ATA", "OK", 3000); }   // [PATCH C3] one retry
    if (aok) {
      callActive   = true;
      callStart    = millis();
      activeCaller = caller;
    } else {
      // [PATCH C3] never fake the state: previously callActive was set unconditionally,
      // so a failed ATA blocked all further call handling for MAX_CALL_DURATION_MS (60s)
      Serial.println("SIM800: ATA failed twice -> not faking callActive, next rings may retry");
    }
  } else {
    // [PATCH C2] log the FULL reason so the cause is visible in Serial Monitor
    int storedCnt = 0;
    for (int i = 0; i < MAX_PHONE_NUMBERS; i++) if (phoneNumbers[i].length() >= 7) storedCnt++;
    Serial.println("SIM800: rejecting unauthorized call");
    // Sensitive caller number not logged.
    if (storedCnt == 0) {
      Serial.println("SIM800: !!! whitelist is EMPTY -> EVERY caller is rejected (busy tone). Set phone numbers in web page /sim800 !!!");
    } else {
      Serial.print("SIM800: whitelist (" + String(storedCnt) + "):");
      // Phone whitelist contents intentionally not printed.
    }
    delay(100);   // [OPT-8]
    sim800SendAT("ATH", "OK", 1500);   // reject only after a POSITIVE mismatch, never on empty ID
  }
}

// ===================== Incoming SMS =====================
// +CMGR: "REC UNREAD","<sender>","","<time>"\n<body>\nOK
void handleIncomingSMS(int index) {
  String r = sim800Exchange("AT+CMGR=" + String(index), 3000);

  int h = r.indexOf("+CMGR:");
  if (h < 0) {
    // Preserve unread SMS on transient UART/network failure.
    Serial.println("SIM800: CMGR gave nothing for index " + String(index));
    return;
  }

  int nlHdr = r.indexOf('\n', h);
  String header = (nlHdr >= 0) ? r.substring(h, nlHdr) : r.substring(h);
  String q[6];
  int c = collectQuoted(header, q, 6);
  String sender = (c > 1) ? q[1] : "";
  String date   = (c > 2) ? q[c - 1] : "";

  String body = (nlHdr >= 0) ? r.substring(nlHdr + 1) : "";
  int okIdx = body.lastIndexOf("\nOK");
  if (okIdx >= 0) body = body.substring(0, okIdx);
  else if (body.endsWith("OK\n")) body = body.substring(0, body.length() - 3);
  body.trim();
  // Incomplete CMGR response must not be processed or deleted.
  if (r.indexOf("\nOK", nlHdr >= 0 ? nlHdr : h) < 0 && !r.endsWith("OK\r\n")) {
    Serial.println("SIM800: incomplete SMS read; retained for retry");
    return;
  }

  if (tryDecodePdu(sender, date, body)) smsPduSeen = true;
  if (smsPduSeen) {
    smsPduSeen = false;
    Serial.println("SIM800: module was in PDU mode - switching to text mode");
    sim800SendAT("AT+CMGF=1", "OK", 1000);
  }

  // حذف فوری از سیم‌کارت، سپس اجرای فرمان در صورت مجاز بودن
  if (sim800HandleMessage(sender, body)) {
    sim800SendAT("AT+CMGD=" + String(index), "OK", 1500);
  }
}

// پیام را در ESP32 ذخیره می‌کند و (در صورت مجاز بودن فرستنده) اجرا می‌کند
int smsClassify(String body);

void cancelActiveCallAndLocation() {
  if (callActive) {
    if (locPhase == 0) sim800SendAT("ATH", "OK", 1500);
    callActive   = false;
    activeCaller = "";
  }
  // GPS SMS loop deliberately remains active. No location state is changed here.
}

bool sim800HandleMessage(const String& from, const String& text) {
  // Do not acknowledge or delete a valid command until it has been executed.
  // If the modem is busy, leave the message in SIM storage for the next poll.
  if (!isPhoneAllowed(from)) return true; // discard unauthorized SMS
  String normalized = text; normalized.trim(); normalized.toUpperCase();
  if (normalized == "RFIDRESET") { rfidReset(); return true; }
  int cls = smsClassify(text);
  if (cls < 0) return true; // unknown command, no action
  if (callActive || locPhase != 0 || pendCount > 0) return false;
  if (cls == 6) { // STATUS: only acknowledge when the reply was submitted successfully
    return sendStatusSms(from);
  }
  processSMSCommand(from, text);
  return true;
}

// مسیر پشتیبان: همه‌ی پیامک‌های روی سیم‌کارت را بخوان، ذخیره/پاک/اجرا کن
void sim800PollSms() {
  // CMGL changes unread messages to READ on some SIM800 firmware versions.
  // Therefore poll ALL messages and re-read each one with CMGR.
  String r = sim800Exchange("AT+CMGL=\"ALL\"", 4000);
  if (r.indexOf("+CMGL:") < 0) return;
  SimMsg msgs[8];
  int n = simParseCmgl(r, msgs, 8);
  for (int i = 0; i < n; i++) handleIncomingSMS(msgs[i].idx);
}

// فشرده‌سازی فاصله‌های تکراری به یک فاصله؛ تا «stop  location» هم مثل «stop location» عمل کند
String smsCollapseSpaces(const String& in) {
  String o;
  o.reserve(in.length());
  bool prevSp = false;
  for (unsigned int j = 0; j < in.length(); j++) {
    char c = in[j];
    if (c == ' ' || c == '\t') { if (!prevSp) { o += ' '; prevSp = true; } }
    else { o += c; prevSp = false; }
  }
  return o;
}

// نسخه‌ی استانداردشده‌ی فرمان شماره‌ی i (بزرگ‌نویسه + فاصله‌های تکی)
String smsCmdNormalized(int i) {
  String k = smsCmd[i];
  k.trim();
  k.toUpperCase();
  return smsCollapseSpaces(k);
}

bool cmdMatch(int i, const String& upBody) {
  String k = smsCmdNormalized(i);
  if (k.length() == 0) return false;
  return k == smsCollapseSpaces(upBody);
}

// 0..6 فرمان‌های سفارشی | 10 فرمان قدیمی ON/OFF | -1 ناشناخته
int smsClassify(String body) {
  body.trim();
  body.toUpperCase();
  for (int i = 0; i < SMS_CMD_COUNT; i++) if (cmdMatch(i, body)) return i;

  // Location با عدد (فاصله‌ی ارسال بر حسب دقیقه): «location 15» → cls 0
  String k0 = smsCmdNormalized(0);
  String cb = smsCollapseSpaces(body);
  if (k0.length() > 0 && cb.startsWith(k0 + " ")) {
    String arg = cb.substring(k0.length() + 1);
    arg.trim();
    bool digits = (arg.length() > 0 && arg.length() <= 3);
    for (unsigned int j = 0; j < arg.length() && digits; j++) if (!isDigit(arg[j])) digits = false;
    if (digits) return 0;
  }

  if (body.startsWith("ON") || body.startsWith("OFF")) {
    bool on = body.startsWith("ON");
    String num = body.substring(on ? 2 : 3);
    num.trim();
    if (num.length() == 0) return 10;
    if (num.length() == 1 && num[0] >= '0' && num[0] <= '3') return 10;
  }
  return -1;
}

// ===================== Status Command (فرمان Status) =====================
// متن کامل وضعیت؛ لوکیشن فقط یک‌بار در همین متن می‌آید و هیچ حلقه‌ای شروع نمی‌شود
String maskAllowedNumber(const String& number) {
  String digits;
  for (unsigned int i = 0; i < number.length(); ++i)
    if (isDigit(number[i])) digits += number[i];
  if (digits.length() <= 8) return "****";
  return digits.substring(0, 4) + "****" + digits.substring(digits.length() - 4);
}

String buildStatusText() {
  String st = "STATUS:\n";
  st += "GSM: " + String(sim800Ready ? gsmPercent : 0) + "%\n";
  st += "Internet: ";
  st += (sim800InternetEnabled && gprsAttached && sim800IP.length() > 0) ? "YES\n" : "NO\n";
  st += "GPS: ";
  st += gpsAvailable() ? gpsMapUrl() : "Unavailable";
  st += "\nDoors: " + String(outputState[0] ? "LOCKED" : "OPEN");
  st += "\nIgnition: " + String(outputState[3] ? "ON" : "OFF");
  st += "\nFuel: " + String(outputState[2] ? "ON" : "OFF");
  // Trunk is a momentary output; current output state is not a physical open/closed sensor.
  st += "\nTrunk output: " + String(outputState[1] ? "ACTIVE" : "IDLE");
  st += "\nAllowed numbers:";
  int count = 0;
  for (int i = 0; i < MAX_PHONE_NUMBERS; ++i) {
    if (phoneNumbers[i].length() >= 7) {
      st += "\n" + String(++count) + ") " + maskAllowedNumber(phoneNumbers[i]);
    }
  }
  if (!count) st += " none";
  return st;
}

// ارسال متن چندخطی/بلند در تکه‌های مجاز (هر پیامک GSM حداکثر ۱۶۰ نویسه)
bool sendSmsChunked(const String& to, const String& text) {
  const unsigned int MAXCH = 150;   // حاشیه‌ی امن زیر ۱۶۰
  String rest = text;
  bool allOk = true;
  while (rest.length() > 0) {
    if (rest.length() <= MAXCH) {
      if (!sendSms(to, rest)) allOk = false;
      break;
    }
    int cut = -1;
    for (int i = (int)MAXCH; i >= (int)MAXCH / 2; i--) {
      if (i < (int)rest.length() && rest[i] == '\n') { cut = i; break; }
    }
    if (cut < 0) cut = (int)MAXCH;
    if (!sendSms(to, rest.substring(0, cut))) allOk = false;
    rest = (rest[cut] == '\n') ? rest.substring(cut + 1) : rest.substring(cut);
  }
  return allOk;
}

// ارسال وضعیت: برای سریال روی Serial چاپ می‌شود، برای شماره‌ی مجاز SMS می‌شود
bool sendStatusSms(const String& from) {
  if (from == "SERIAL") return false;
  return sendSmsChunked(from, buildStatusText());
}

String secStateName() {
  if (!securityEnabled) return "OFF";
  switch (secState) {
    case 1: return "GRACE";
    case 2: return "USER-OK";
    case 3: return "LOCKED";
    default: return "NORMAL";
  }
}


// cls: 0=Location 1=Lock 2=Unlock 3=Alarm_on 4=Alarm_off 5=Stop location 6=Status 10=ON/OFF legacy
void processSMSCommand(String from, String body) {
  int cls = smsClassify(body);
  String up = body;
  up.trim();
  up.toUpperCase();
  Serial.println("SMS command class=" + String(cls));

  // Location [X]: شروع حلقه SMS لوکیشن با فاصله‌ی X دقیقه (1..120)؛ بدون عدد = ۱ دقیقه
  if (cls == 0) {
    int minutes = LOC_SMS_MIN_MINUTES;
    String k0 = smsCmdNormalized(0);
    String cb = smsCollapseSpaces(up);
    if (k0.length() > 0 && cb.startsWith(k0 + " ")) {
      String arg = cb.substring(k0.length() + 1);
      arg.trim();
      bool digits = (arg.length() > 0 && arg.length() <= 3);
      for (unsigned int j = 0; j < arg.length() && digits; j++) if (!isDigit(arg[j])) digits = false;
      if (digits) {
        long v = arg.toInt();
        if (v < LOC_SMS_MIN_MINUTES) v = LOC_SMS_MIN_MINUTES;
        if (v > LOC_SMS_MAX_MINUTES) v = LOC_SMS_MAX_MINUTES;
        minutes = (int)v;
      }
    }
    locSmsIntervalMs   = (unsigned long)minutes * 60000UL;
    locSmsActive       = true;
    locSmsTargetNumber = from;
    locSmsLastSend     = 0;   // ارسال فوری در اولین چرخه
    locSmsSendCount    = 0;
    Serial.println("Location SMS loop enabled");
    startLocation("sms:" + from);
    return;
  }

  if (cls == 1) {   // Lock
    applyLock(false, false);     // [CHANGE] فقط قطع سوئیچ و پمپ بنزین - درب‌ها و صندوق عقب دست نمی‌خورند
    Serial.println("LOCK executed (fuel+switch only, doors/trunk untouched)");
    return;
  }

  if (cls == 2) {   // Unlock
    unlockCar();                 // شامل لغو مهلت ۵ دقیقه (تغییر 8)
    Serial.println("UNLOCK executed");
    return;
  }

  // Status (cls==3) حذف شد — تغییر 9

  if (cls == 3) {   // Alarm_on
  alarmManual = true;
  updateBuzzer();
  pendingAlarmSms       = true;                          // ← جدید
  pendingAlarmSmsReason = "Manual alarm activated";      // ← جدید
  Serial.println("ALARM ON");
  return;
}

  if (cls == 4) {   // Alarm_off
    alarmManual = false;
    safeZoneAlarmActive  = false;   // ← خاموش کردن آلارم Safe Zone
    safeZoneCallIdx      = 0;
    safeZoneCallAnswered = false;
    updateBuzzer();
    Serial.println("ALARM OFF");
    return;
  }

  if (cls == 5) {   // Stop location: توقف کامل حلقه‌ی ارسال لوکیشن
    bool wasActive = locSmsActive;
    locSmsActive       = false;
    locSmsTargetNumber = "";
    locSmsSendCount    = 0;
    if (wasActive) Serial.println("Location SMS loop stopped");
    else           Serial.println("Stop location: loop was not active");
    return;
  }

  if (cls == 6) {   // Status: گزارش یک‌باره‌ی وضعیت (لوکیشن فقط یک‌بار، بدون شروع حلقه)
    if (from == "SERIAL") {
      Serial.println("Status details available via authorized SMS only");
      return;
    }
    // اگر تماس فعال یا LBS در جریان است، بعد از آزاد شدن کانال فقط یک‌بار ارسال شود
    if (callActive || locPhase != 0) {
      pendingStatusSms  = true;
      pendingStatusFrom = from;
      Serial.println("STATUS: busy (call/GPRS) -> deferred");
      return;
    }
    sendStatusSms(from);
    return;
  }

  // legacy ON/OFF
  if (cls == 10) {
    bool on = up.startsWith("ON");
    String num = up.substring(on ? 2 : 3);
    num.trim();
    int idx = (num.length() == 1) ? num.toInt() : 0;
    if (idx == 0) { setDoors(on); Serial.println(on ? "Doors ON" : "Doors OFF"); }
    else if (idx == 1) {
      if (on) { triggerTrunk(); Serial.println("Trunk pulse"); }
      else Serial.println("Trunk momentary; use ON1");
    }
    else if (on && carLocked) Serial.println("Output blocked: car locked");
    else { setOutput(idx, on); desiredState[idx] = on; Serial.println("Output " + String(idx) + (on ? " ON" : " OFF")); }
    return;
  }
  Serial.println("Unknown SMS command");
}

// ===================== [PATCH-S] Serial Monitor Command Console =====================
// علت سکوت Serial Monitor این بود که اسکچ اصلاً از Serial ورودی نمی‌خواند.
// حالا خطوط تایپ‌شده در Serial Monitor دقیقاً مثل پیامک طبقه‌بندی و اجرا می‌شوند:
//  - فرمان‌های Location و Stop location فقط از SMS قابل اجرا هستند
//  - بقیه فرمان‌ها (Lock/Unlock/Alarm/ON/OFF) از همان processSMSCommand عبور می‌کنند
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (serialCmdBuf.length() < 120) serialCmdBuf += c;
      continue;
    }
    serialCmdBuf.trim();
    if (serialCmdBuf.length() == 0) {
      serialCmdBuf = "";
      continue;
    }
    int cls = smsClassify(serialCmdBuf);
    if (cls == 0 || cls == 5) {
      Serial.println("Location control is SMS-only; serial request ignored");
    } else {
      processSMSCommand("SERIAL", serialCmdBuf);
    }
    serialCmdBuf = "";
  }
}

// ===================== Status / Signal =====================
void sim800CheckStorage() {
  for (int pass = 0; pass < 2; pass++) {
    String r = sim800Exchange("AT+CPMS?", 1500);
    int i = r.indexOf("+CPMS:");
    if (i < 0) return;
    int c1 = r.indexOf(',', i);
    int c2 = (c1 >= 0) ? r.indexOf(',', c1 + 1) : -1;
    int c3 = (c2 >= 0) ? r.indexOf(',', c2 + 1) : -1;
    if (c1 < 0 || c2 < 0) return;
    int used  = r.substring(c1 + 1, c2).toInt();
    int total = r.substring(c2 + 1, c3 > 0 ? c3 : r.length()).toInt();
    if (total <= 0 || used < total - 2) return;
    Serial.println("SIM800: SIM SMS storage nearly full (" + String(used) + "/" + String(total) + ") - cleaning");
    // Never bulk-delete unread SMS: report capacity issue instead.
    return;
  }
}

void updateSim800Status() {
  if (!sim800Ready) return;

  static unsigned long lastCops = 0;   // ← تعریف قبل از استفاده (رفع خطای ۱)
  String r;

  // ===== CREG: ثبت شبکه تلفنی =====
  r = sim800Exchange("AT+CREG?", 1500);
  int i = r.indexOf("+CREG:");
  if (i >= 0) {
    sim800NoReply = 0;                 // ← هر AT موفق شمارنده را صفر می‌کند
    int c = r.indexOf(',', i);
    int nl = r.indexOf('\n', i);
    if (c > 0 && (nl < 0 || c < nl)) {
      int st = r.substring(c + 1).toInt();
      gprsRegistered = (st == 1 || st == 5);
    }
  }

  // ===== COPS: نام اپراتور =====
  // [OPT-4] تا وقتی APN هنوز پیدا نشده هر ۵ ثانیه پرسیده می‌شود (قبلاً ۳۰ ثانیه)،
  // چون پیدا شدن APN پیش‌نیاز warm-up اینترنت و GPRS سریع است
  if (millis() - lastCops > (sim800APN.length() > 0 ? 30000UL : 5000UL)) {
    lastCops = millis();
    r = sim800Exchange("AT+COPS?", 2000);
    int q1 = r.indexOf('"');
    int q2 = (q1 >= 0) ? r.indexOf('"', q1 + 1) : -1;
    if (q1 >= 0 && q2 > q1) sim800Carrier = r.substring(q1 + 1, q2);
    else if (r.indexOf("+COPS:") >= 0) sim800Carrier = "";
  }

  // ===== CGREG: ثبت شبکه بسته‌ای (جایگزین CGATT) =====
  String gr = sim800Exchange("AT+CGREG?", 1000);
  int gi = gr.indexOf("+CGREG:");
  if (gi >= 0) {
    sim800NoReply = 0;
    int gc = gr.indexOf(',', gi);
    if (gc > 0) {
      int gst = gr.substring(gc + 1).toInt();
      gprsRegistered = (gst == 1 || gst == 5);
    }
  }

  String attachResponse = sim800Exchange("AT+CGATT?", 1200);
  if (attachResponse.indexOf("+CGATT:") >= 0)
    gprsAttached = (attachResponse.indexOf("+CGATT: 1") >= 0 || attachResponse.indexOf("+CGATT:1") >= 0);
  if (!gprsRegistered || !gprsAttached) sim800IP = "";

  if (gprsRegistered && sim800APN.length() == 0 && sim800Carrier.length() > 0) detectAndSetAPN();
}   // ← این آکولاد ممکن است در نسخه‌ی شما جا افتاده باشد
void updateGsmSignal() {
  if (!sim800Ready) return;

  String r = sim800Exchange("AT+CSQ", 1000);
  int idx = r.indexOf("+CSQ:");
  if (idx >= 0) {
    sim800NoReply = 0;
    int comma = r.indexOf(',', idx);
    if (comma > 0) {
      gsmCSQ = r.substring(idx + 5, comma).toInt();
      gsmPercent = (gsmCSQ == 99 || gsmCSQ < 0) ? 0 : constrain(gsmCSQ * 100 / 31, 0, 100);
    }
  } else if (++sim800NoReply >= 5) {   // [OPT-15] قبلاً 10 → ماژول بی‌پاسخ ~۲۰۰s بعد ری‌استارت می‌شد؛ حالا ~۱۰۰s
    Serial.println("SIM800: module stopped responding - re-init");
    sim800Ready = false;
    sim800NoReply = 0;
    sim800LastInit = millis();
    locPhase = 0;
    sim800IP = "";
  }
}

void detectAndSetAPN() {
  String apn = apnForOperator(sim800Carrier);
  if (apn.length() > 0) {
    sim800APN = apn;
    saveSim800Config();
    sim800IP = "";
    Serial.println("SIM800: APN configured");
  }
}

String apnForOperator(String op) {
  op.toUpperCase();
  if (op.indexOf("MCI") >= 0 || op.indexOf("TCI") >= 0 || op.indexOf("HAMRAH") >= 0 || op.indexOf("43211") >= 0) return "mcinet";
  if (op.indexOf("IRANCELL") >= 0 || op.indexOf("MTN") >= 0 || op.indexOf("43235") >= 0) return "mtnirancell";
  if (op.indexOf("RIGHTEL") >= 0 || op.indexOf("43220") >= 0) return "rightel";
  if (op.indexOf("TALIYA") >= 0) return "taliya";
  if (op.indexOf("SHATEL") >= 0) return "shatel";
  return "";
}

// ===================== Phone whitelist =====================
String normalizePhone(String n) {
  n.trim();
  n.replace(" ", ""); n.replace("-", "");
  n.replace("(", ""); n.replace(")", "");
  n.replace("+", "");
  if (n.startsWith("00")) n = n.substring(2);
  return n;
}

bool isPhoneAllowed(String num) {
  String n = normalizePhone(num);
  if (n.length() < 7) return false;
  for (int i = 0; i < MAX_PHONE_NUMBERS; i++) {
    if (phoneNumbers[i].length() == 0) continue;
    String a = normalizePhone(phoneNumbers[i]);
    if (a.length() < 7) continue;
    if (a == n) return true;
    String a2 = a, n2 = n;
    if (a2.startsWith("0")) a2 = a2.substring(1);
    if (n2.startsWith("0")) n2 = n2.substring(1);
    if (a2 == n2) return true;
    String a3 = a2, n3 = n2;
    if (a3.startsWith("98") && a3.length() > 10) a3 = a3.substring(2);
    if (n3.startsWith("98") && n3.length() > 10) n3 = n3.substring(2);
    if (a3 == n3 && a3.length() >= 9) return true;
  }
  return false;
}

// ===================== Web handlers =====================
void handleGetSim800Status() {
  if (!isRequestAuthorized()) {
    server.send(403, "application/json", "{\"success\":false}");
    return;
  }
  long left = 0;
  if (secState == 1) {
    unsigned long el = millis() - secGraceStart;
    left = (el >= SEC_GRACE_MS) ? 0 : (long)((SEC_GRACE_MS - el) / 1000UL);
  }
  String json = "{";
  json += "\"enabled\":"    + String(sim800Enabled ? "true" : "false") + ",";
  json += "\"ready\":"      + String(sim800Ready ? "true" : "false") + ",";
  json += "\"gsmPercent\":" + String(gsmPercent) + ",";
  json += "\"internetEnabled\":" + String(sim800InternetEnabled ? "true" : "false") + ",";
  json += "\"txKB\":" + String((double)sim800TxDataBytes / 1024.0, 2) + ",";
  json += "\"rxKB\":" + String((double)sim800RxDataBytes / 1024.0, 2) + ",";
  json += "\"csq\":"        + String(gsmCSQ) + ",";
  json += "\"registered\":" + String(gprsRegistered ? "true" : "false") + ",";
  json += "\"attached\":"   + String(gprsAttached ? "true" : "false") + ",";
  json += "\"carrier\":\""  + jsonEscape(sim800Carrier) + "\",";
  json += "\"apn\":\""      + jsonEscape(sim800APN) + "\",";
  json += "\"ip\":\""       + jsonEscape(sim800IP) + "\",";
  json += "\"callActive\":" + String(callActive ? "true" : "false") + ",";
  json += "\"busy\":"       + String(locPhase != 0 ? "true" : "false") + ",";
  json += "\"secEn\":"      + String(securityEnabled ? "true" : "false") + ",";
  json += "\"sec\":"        + String(secState) + ",";
  json += "\"left\":"       + String(left) + ",";
  json += "\"locked\":"     + String(carLocked ? "true" : "false") + ",";
  json += "\"alarm\":"      + String(alarmManual ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetSim800Config() { if (!isRequestAuthorized()) {server.send(403, "application/json", "{\"success\":false,\"message\":\"Unauthorized\"}");return;}
  bool changed = false;

  // [CHANGE] چک‌باکس‌های Enable SIM800 / Enable GPRS حذف شدند - همیشه فعال و غیرقابل غیرفعال‌سازی
  sim800Enabled = true;
  sim800InternetEnabled = true;
  if (server.hasArg("apn")) { String a = server.arg("apn"); a.trim();if (a.length() > 40) a = a.substring(0, 40);if (a != sim800APN) { 
    sim800APN = a; changed = true; sim800IP = ""; sim800LastBearerTry = 0; bearerLastFail = 0; } }
  if (server.hasArg("sec")) { bool s = (server.arg("sec") == "1");if (s != securityEnabled) { securityEnabled = s; gsmBadSince = 0;
      if (!s && secState != 3) secState = 0; changed = true; } } String newCmd[SMS_CMD_COUNT];
  for (int k = 0; k < SMS_CMD_COUNT; k++) {newCmd[k] = smsCmd[k];String arg = "c" + String(k); if (server.hasArg(arg)) {String v = server.arg(arg); v.trim();
      if (v.length() > 0 && v.length() <= 30 && asciiOnly(v)) newCmd[k] = v;  } }  for (int a = 0; a < SMS_CMD_COUNT; a++)
       { for (int b = a + 1; b < SMS_CMD_COUNT; b++) { String x = newCmd[a], y = newCmd[b];  x.toUpperCase(); y.toUpperCase();
      if (x == y) { server.send(200, "application/json", "{\"success\":false,\"message\":\"Duplicate command\"}");  return; } } }
  for (int k = 0; k < SMS_CMD_COUNT; k++) {if (newCmd[k] != smsCmd[k]) { smsCmd[k] = newCmd[k]; changed = true; } } for (int k = 0; k < MAX_PHONE_NUMBERS; k++) {
    String arg = "ph" + String(k); if (server.hasArg(arg)) {String p = server.arg(arg); p.trim();if (p.length() > 20) p = p.substring(0, 20);
     if (p != phoneNumbers[k]) { phoneNumbers[k] = p; changed = true; } } }
  if (changed) { saveSim800Config();if (sim800Enabled && !sim800Ready) sim800ForceInit = true; }server.send(200, "application/json", "{\"success\":true}");}

void handleDetectAPN() {
  if (!isRequestAuthorized()) {
    server.send(403, "application/json", "{\"success\":false}");
    return;
  }
  if (sim800Enabled && sim800Ready && locPhase == 0) updateSim800Status();

  detectAndSetAPN();
  bool ok = sim800APN.length() > 0;
  server.send(200, "application/json",
              String("{\"success\":") + (ok ? "true" : "false") +
              ",\"apn\":\"" + jsonEscape(sim800APN) + "\",\"carrier\":\"" + jsonEscape(sim800Carrier) + "\"}");
}



// ============================================================================
// MQTT configuration and secure SIM800 native-MQTT transport.
// Avoid exposing credentials via GET, serial monitor, JSON endpoints, or URLs.
// ============================================================================
String defaultMqttDeviceId() {
  char id[20] = {0};
  snprintf(id, sizeof(id), "SCL-%012llX",
           (unsigned long long)(ESP.getEfuseMac() & 0xFFFFFFFFFFFFULL));
  return String(id);
}

bool mqttSafeToken(const String& value, size_t maxLength, bool allowSlash) {
  if (value.length() == 0 || value.length() > maxLength) return false;
  if (allowSlash && (value[0] == '/' || value[value.length()-1] == '/')) return false;
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '_' || c == '-' ||
        (allowSlash && c == '/')) continue;
    return false;
  }
  if (allowSlash && value.indexOf("//") >= 0) return false;
  return true;
}

bool mqttValidHost(const String& host) {
  if (host.length() == 0 || host.length() > 120 ||
      host.startsWith(".") || host.endsWith(".") || host.indexOf("..") >= 0) return false;
  for (size_t i = 0; i < host.length(); ++i) {
    const char c = host[i];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '.' || c == '-') continue;
    return false;
  }
  return true;
}

void loadMqttConfig() {
  MqttConfig cfg;
  if (preferences.begin("mqttcfg", true)) {
    cfg.requestedEnabled = preferences.getBool("enabled", false);
    cfg.tlsRequested = preferences.getBool("tls", true);
    cfg.host = preferences.getString("host", "smartcarlock.ir");
    cfg.port = preferences.getUShort("port", 8883);
    cfg.deviceId = preferences.getString("deviceId", defaultMqttDeviceId());
    cfg.username = preferences.getString("user", "");
    cfg.password = preferences.getString("pass", "");
    cfg.commandKey = preferences.getString("cmdKey", "");
    mqttLastSequence = preferences.getULong("lastSeq",0);
    cfg.topicPrefix = preferences.getString("topic", "smartcar");
    cfg.keepAliveSec = preferences.getUShort("keepAlive", 60);
    cfg.positionIntervalSec = preferences.getUShort("gpsEvery", 60);
    cfg.heartbeatIntervalSec = preferences.getUShort("heartbeat", 180);
    preferences.end();
  }
  if (!mqttValidHost(cfg.host)) cfg.host = "smartcarlock.ir";
  if (!mqttSafeToken(cfg.deviceId, 32)) cfg.deviceId = defaultMqttDeviceId();
  if (!mqttSafeToken(cfg.topicPrefix, 64, true)) cfg.topicPrefix = "smartcar";
  if (cfg.port == 0) cfg.port = cfg.tlsRequested ? 8883 : 1883;
  if (cfg.keepAliveSec < 60 || cfg.keepAliveSec > 600) cfg.keepAliveSec = 60;
  if (cfg.positionIntervalSec < 10 || cfg.positionIntervalSec > 3600) cfg.positionIntervalSec = 60;
  if (cfg.heartbeatIntervalSec < 30 || cfg.heartbeatIntervalSec > 3600) cfg.heartbeatIntervalSec = 180;
  if (cfg.username.indexOf('"') >= 0 || cfg.username.indexOf('\\') >= 0 || !asciiOnly(cfg.username)) cfg.username = "";
  if (cfg.password.indexOf('"') >= 0 || cfg.password.indexOf('\\') >= 0 || !asciiOnly(cfg.password)) cfg.password = "";
  if (cfg.commandKey.length() != 64) cfg.commandKey = "";
  else for (size_t i=0;i<cfg.commandKey.length();i++) if (!isxdigit((unsigned char)cfg.commandKey[i])) { cfg.commandKey=""; break; }
  mqttConfig = cfg;
}

bool saveMqttConfig(const MqttConfig& cfg) {
  if (!preferences.begin("mqttcfg", false)) return false;
  bool ok = true;
  ok &= preferences.putBool("enabled", cfg.requestedEnabled) == 1;
  ok &= preferences.putBool("tls", cfg.tlsRequested) == 1;
  ok &= preferences.putString("host", cfg.host) == cfg.host.length();
  ok &= preferences.putUShort("port", cfg.port) == 2;
  ok &= preferences.putString("deviceId", cfg.deviceId) == cfg.deviceId.length();
  // putString("") returns 0 even after a successful empty-string write in ESP32 Arduino;
  // validate blank fields by reading them back instead of interpreting 0 as failure.
  preferences.putString("user", cfg.username);
  ok &= preferences.getString("user", "\x01") == cfg.username;
  preferences.putString("pass", cfg.password);
  ok &= preferences.getString("pass", "\x01") == cfg.password;
  preferences.putString("cmdKey", cfg.commandKey);
  ok &= preferences.getString("cmdKey", "\x01") == cfg.commandKey;
  ok &= preferences.putString("topic", cfg.topicPrefix) == cfg.topicPrefix.length();
  ok &= preferences.putUShort("keepAlive", cfg.keepAliveSec) == 2;
  ok &= preferences.putUShort("gpsEvery", cfg.positionIntervalSec) == 2;
  ok &= preferences.putUShort("heartbeat", cfg.heartbeatIntervalSec) == 2;
  preferences.end();
  return ok;
}




// ============================================================================
// SIM800 NATIVE MQTT: a cooperative AT state machine (no waiting/delay loops).
// Commands: SIM800 MQTT Application Note v1.03; supported by SOME firmware only.
// Broker ACL MUST bind credentials to topicPrefix/deviceId/# per device.
// TLS is never downgraded to cleartext. SIM800 root CA must be pre-provisioned.
// ============================================================================
String mqttRoot() { return mqttConfig.topicPrefix + "/" + mqttConfig.deviceId; }
String mqttStateText() {
  if (!mqttConfig.requestedEnabled) return "Disabled";
  if (mqttConnected && mqttStep == MQ_READY) return "Connected (TLS requested, verify broker logs)";
  return mqttLastError;
}
void mqttResetTransport(const String& why) {
  mqttWaiting = false;
  mqttConnected = false;
  mqttStep = MQ_DOWN;
  mqttLastAttempt = 0;
  mqttBackoffMs = 15000;
  mqttLastError = why;
  mqttPubAwaitUrc = false;
  mqttSubAwaitUrc = false;
  mqttPubAcked = false;
  mqttSubAcked = false;
  mqttPublishKind = "";
  mqttPendingPayload = "";
}
void mqttFail(const String& reason) {
  mqttWaiting = false;
  mqttConnected = false;
  mqttStep = MQ_DOWN;
  mqttLastAttempt = millis();
  mqttLastError = reason;
  mqttPubAwaitUrc = false;
  mqttSubAwaitUrc = false;
  mqttPubAcked = false;
  mqttSubAcked = false;
  if (mqttBackoffMs < 300000UL) mqttBackoffMs = (mqttBackoffMs >= 150000UL ? 300000UL : mqttBackoffMs * 2UL);
  Serial.println("MQTT: reconnect scheduled (" + reason + ")");
}
void mqttCommand(const String& cmd, MqttStep next, uint32_t timeout = 5000) {
  // mqttStep denotes the command currently waiting for a result.
  mqttStep = next;
  mqttWaiting = true;
  mqttGotOk = false;
  mqttPubAcked = false;
  mqttSubAcked = false;
  mqttStepSince = millis();
  // Never echo AT+SMCONF PASSWORD nor payloads to USB Serial.
  sim800Serial.print(cmd);
  sim800Serial.print('\r');
  (void)timeout;
}
void mqttAdvance() {
  switch (mqttStep) {
    case MQ_DISC: mqttCommand("AT+SMCONF=?", MQ_PROBE); break;
    case MQ_PROBE: mqttRootSeen=false; mqttCommand("AT+SSLDEROOT?", MQ_ROOT); break;
    case MQ_ROOT: mqttClockValid=false; mqttCommand("AT+CCLK?", MQ_CLOCK); break;
    case MQ_CLOCK: mqttCommand("AT+SSLOPT=0,0", MQ_VERIFY); break;
    case MQ_VERIFY: mqttCommand("AT+SSLOPT=1,0", MQ_NO_CLIENT); break;
    case MQ_NO_CLIENT: mqttCommand("AT+SMSSL=1", MQ_SSL); break;
    case MQ_SSL: mqttCommand("AT+SMCONF=\"CID\",1", MQ_CID); break;
    case MQ_CID: mqttCommand("AT+SMCONF=\"URL\",\"" + mqttConfig.host + ":" + String(mqttConfig.port) + "\"", MQ_URL); break;
    case MQ_URL: mqttCommand("AT+SMCONF=\"CLIENTID\",\"" + mqttConfig.deviceId + "\"", MQ_CLIENT); break;
    case MQ_CLIENT: mqttCommand("AT+SMCONF=\"USERNAME\",\"" + mqttConfig.username + "\"", MQ_USER); break;
    case MQ_USER: mqttCommand("AT+SMCONF=\"PASSWORD\",\"" + mqttConfig.password + "\"", MQ_PASS); break;
    case MQ_PASS: mqttCommand("AT+SMCONF=\"KEEPALIVE\"," + String(mqttConfig.keepAliveSec), MQ_KEEP); break;
    case MQ_KEEP: mqttCommand("AT+SMCONF=\"CLEANSS\",1", MQ_CLEAN); break;
    case MQ_CLEAN: mqttCommand("AT+SMCONN", MQ_CONNECT); break;
    case MQ_CONNECT: mqttSubAwaitUrc=true; mqttCommand("AT+SMSUB=\"" + mqttRoot() + "/cmd\",1", MQ_SUB); break;
    default: break;
  }
}
// MQTT /cmd UTF-8/ASCII payload: SEQ|OP|MAC32
// MAC32 = first 16 bytes of HMAC-SHA256, lowercase hex, with a separate
// 32-byte commandKey (configured on the MQTT menu). Input signed as:
// "deviceId|SEQ|OP". Sequence strictly increases and survives reboot.
bool mqttCheckSignature(const String& seq, const String& op, const String& gotMac) {
  if (mqttConfig.commandKey.length()!=64 || gotMac.length()!=32) return false;
  uint8_t key[32];
  for (int i=0;i<32;i++) {
    char b[3]={mqttConfig.commandKey[2*i],mqttConfig.commandKey[2*i+1],0};
    key[i]=(uint8_t)strtoul(b,nullptr,16);
  }
  const mbedtls_md_info_t* sha=mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!sha) return false;
  String m=mqttConfig.deviceId+"|"+seq+"|"+op;
  unsigned char h[32]={0};
  if (mbedtls_md_hmac(sha,key,32,(const unsigned char*)m.c_str(),m.length(),h)!=0) return false;
  uint8_t diff=0;
  static const char* hex="0123456789abcdef";
  for (int i=0;i<16;i++) {
    diff |= (uint8_t)(hex[h[i]>>4] ^ gotMac[2*i]);
    diff |= (uint8_t)(hex[h[i]&15] ^ gotMac[2*i+1]);
  }
  return diff==0;
}
bool mqttParseCommand(const String& payload, uint32_t& seq, String& command) {
  if (payload.length()<38 || payload.length()>100) return false;
  int a=payload.indexOf('|');
  int b=payload.indexOf('|',a+1);
  if (a<1 || a>10 || b<=a+1 || payload.indexOf('|',b+1)>=0) return false;
  String n=payload.substring(0,a);
  for (size_t i=0;i<n.length();i++) if (!isDigit(n[i])) return false;
  unsigned long raw=strtoul(n.c_str(),nullptr,10);
  if (raw==0 || raw>0xFFFFFFFFUL) return false;
  String op=payload.substring(a+1,b);
  for (size_t i=0;i<op.length();i++) {
    char c=op[i]; if (!((c>='A' && c<='Z') || c=='_')) return false;
  }
  if (!mqttCheckSignature(n,op,payload.substring(b+1))) return false;
  seq=(uint32_t)raw;command=op;return true;
}
void mqttQueueAck(uint32_t seq, const char* result) {
  String a=String(seq)+"|"+String(result);
  if (mqttAckCount==8) return;
  mqttAckPayloads[(mqttAckHead+mqttAckCount)%8]=a;
  mqttAckCount++;
}
bool mqttVehicleIsParked() {
  // GPS alone is NOT a safety-rated motion detector. A remote command here
  // only actuates door locking pulses, NEVER fuel, ignition, or immobilization.
  return gpsAvailable() && gps.speed.isValid() && gps.speed.age() < 3000 &&
         gps.speed.kmph() < 1.0 && !outputState[3] && !callActive;
}
void mqttHandleRemoteCommand(const String& payload) {
  if (!mqttConnected || !mqttConfig.tlsRequested) return;
  uint32_t seq=0; String command;
  if (!mqttParseCommand(payload,seq,command)) return;
  if (mqttAckCount>=8) return; // fail closed if ack queue is full
  if (seq<=mqttLastSequence) { mqttQueueAck(seq,"duplicate"); return; }
  const char* outcome="denied";
  bool parked=mqttVehicleIsParked();
  if (command=="STATUS" || command=="LOCATION" || command=="ALARM_OFF") outcome="ok";
  else if (command=="DOOR_LOCK" || command=="DOOR_UNLOCK" || command=="TRUNK")
    outcome=parked ? "ok" : "not_parked";
  // Fuel/ignition/immobilizer commands are deliberately NOT accepted remotely.
  // Commit anti-replay sequence BEFORE actuation. A power loss in between
  // may cause a skipped operation; never retry a potentially unsafe pulse.
  if (!preferences.begin("mqttcfg",false)) {mqttFail("NVS unavailable");return;}
  size_t bytes=preferences.putULong("lastSeq",seq);
  uint32_t saved=preferences.getULong("lastSeq",0);
  preferences.end();
  if (bytes!=4 || saved!=seq) {mqttFail("Sequence commit failed");return;}
  mqttLastSequence=seq;
  if (!strcmp(outcome,"ok")) {
    if (command=="STATUS" || command=="LOCATION") {
      mqttLastHeartbeat=0; mqttLastPosition=0;
    } else if (command=="ALARM_OFF") {
      alarmManual=false;safeZoneAlarmActive=false;updateBuzzer();
    } else if (command=="DOOR_LOCK") setDoors(true);
    else if (command=="DOOR_UNLOCK") setDoors(false);
    else if (command=="TRUNK") triggerTrunk();
  }
  mqttQueueAck(seq,outcome);
}
void mqttIncoming(const String& line) {
  // +SMPUBLISH: <packet_id>,"topic",<msgLen>,"message"
  if (!mqttConnected || line.length()>512) return;
  int a=line.indexOf('"');
  int b=(a>=0)?line.indexOf('"',a+1):-1;
  if (a<0 || b<0) return;
  if (line.substring(a+1,b)!=mqttRoot()+"/cmd") return;
  int want=0; // parse length declared in +SMPUBLISH
  // SIM800 can include a space after the comma; parse the token properly below.
  int lenStart=b+1;
  while (lenStart<(int)line.length() && (line[lenStart]==',' || line[lenStart]==' ')) lenStart++;
  int lenEnd=line.indexOf(',',lenStart);
  if (lenEnd<0) return;
  want=line.substring(lenStart,lenEnd).toInt();
  int q1=line.indexOf('"',lenEnd+1);
  int q2=line.lastIndexOf('"');
  if (want<1 || want>180 || q1<0 || q2<=q1 || (q2-q1-1)!=want) return;
  mqttHandleRemoteCommand(line.substring(q1+1,q2));
}
void mqttCompleteSubscription() {
  if (!mqttSubAcked || !mqttGotOk || mqttStep!=MQ_SUB) return;
  mqttWaiting=false; mqttSubAwaitUrc=false; mqttConnected=true;
  mqttStep=MQ_READY; mqttBackoffMs=15000;
  mqttLastHeartbeat=0; mqttLastPosition=0;
  mqttLastError="Connected";
}
void mqttCompletePublish() {
  if (!mqttPubAcked || !mqttGotOk || mqttStep!=MQ_PUB) return;
  mqttWaiting=false; mqttPubAwaitUrc=false; mqttStep=MQ_READY;
  if (mqttPublishKind=="ack" && mqttAckCount) {
    mqttAckPayloads[mqttAckHead]="";
    mqttAckHead=(mqttAckHead+1)%8; mqttAckCount--;
  }
  if (mqttPublishKind=="gps") mqttLastPosition=millis();
  if (mqttPublishKind=="status") mqttLastHeartbeat=millis();
}
bool mqttOnModemLine(const String& line) {
  if (line.startsWith("+SMPUBLISH:")) { mqttIncoming(line); return true; }
  if (line.startsWith("+SMSUB:")) {
    if (mqttStep==MQ_SUB && mqttWaiting && mqttSubAwaitUrc) {
      int comma=line.lastIndexOf(',');
      if (comma>=0 && line.substring(comma+1).toInt()==0) {
        mqttSubAcked=true;
        mqttCompleteSubscription();
      } else mqttFail("MQTT subscription rejected");
    }
    return true;
  }
  if (line.startsWith("+SMPUB:")) {
    if (mqttStep==MQ_PUB && mqttWaiting && mqttPubAwaitUrc) {
      int comma=line.lastIndexOf(',');
      if (comma>=0 && line.substring(comma+1).toInt()==0) {
        mqttPubAcked=true;
        mqttCompletePublish();
      } else mqttFail("MQTT publish rejected");
    }
    return true;
  }
  if (!mqttWaiting) return false;
  // Distinguish modem lines while running a native-MQTT AT transaction.
  if (line.startsWith("+SSLDEROOT:")) mqttRootSeen=true;
  else if (mqttStep==MQ_ROOT && (line.indexOf(".crt")>=0 || line.indexOf(".cer")>=0 || line.indexOf(".der")>=0)) mqttRootSeen=true;
  if (mqttStep==MQ_CLOCK && line.startsWith("+CCLK:")) {
    int q=line.indexOf('"');
    if (q>=0 && line.length()>q+5) {
      int yr=line.substring(q+1,q+3).toInt();
      mqttClockValid=(yr>=24 && yr<=50);
    }
  }
  if (line=="OK") {
    mqttGotOk=true;
    if (mqttStep==MQ_ROOT && !mqttRootSeen) { mqttFail("SIM800 trusted root CA missing"); return true; }
    if (mqttStep==MQ_CLOCK && !mqttClockValid) { mqttFail("SIM800 clock not synchronized"); return true; }
    if (mqttStep==MQ_SUB) { mqttCompleteSubscription(); return true; }
    if (mqttStep==MQ_PUB) { mqttCompletePublish(); return true; }
    if (mqttStep==MQ_CHECK) {
      // +SMSTATE: 1 is parsed before OK as special case.
      if (!mqttConnected) { mqttFail("MQTT disconnected"); return true; }
      mqttWaiting=false; mqttStep=MQ_READY; mqttLastCheck=millis(); return true;
    }
    mqttWaiting=false;
    mqttAdvance();
    return true;
  }
  if (mqttStep==MQ_CHECK && line.startsWith("+SMSTATE:")) {
    mqttConnected=(line.substring(line.indexOf(':')+1).toInt()==1);
    return true;
  }
  if ((line=="ERROR" || line.startsWith("+CME ERROR")) && mqttStep==MQ_DISC) {
    mqttWaiting=false; mqttAdvance(); return true; // no prior MQTT session
  }
  if (line=="ERROR" || line.startsWith("+CME ERROR") || line.startsWith("+CMS ERROR")) {
    mqttFail("Modem rejected MQTT/TLS command (check firmware and CA)");
    return true;
  }
  return false;
}
// Compact ASCII records avoid unsupported escaping of JSON quotes inside
// AT+SMPUB="topic",qos,retain,"message" on various SIM800 firmware builds.
String mqttStatusPayload() {
  String j="gsm="+String(gsmPercent)+";net=";
  j+=(gprsAttached && sim800IP.length()>0 ? "1" : "0");
  j+=";doors="+String(outputState[0]?"1":"0");
  j+=";ignition="+String(outputState[3]?"1":"0");
  j+=";fuel="+String(outputState[2]?"1":"0");
  j+=";trunkPulse="+String(outputState[1]?"1":"0");
  j+=";alarm="+String(alarmManual?"1":"0");
  return j;
}
String mqttGpsPayload() {
  if (!gpsAvailable()) return "fix=0";
  return "fix=1;lat="+String(gpsLat,6)+";lon="+String(gpsLng,6)+
         ";speed="+String(gps.speed.isValid()?gps.speed.kmph():0,1);
}
void mqttPublish(const String& kind, const String& payload) {
  String topic=mqttRoot()+"/"+(kind=="ack"?"ack":(kind=="gps"?"telemetry":"status"));
  if (topic.length()>100 || payload.length()>400) return;
  mqttPublishKind=kind;
  mqttPubAwaitUrc=true;
  mqttCommand("AT+SMPUB=\""+topic+"\",1,0,\""+payload+"\"",MQ_PUB,10000);
}
void mqttTick() {
  if (!mqttConfig.requestedEnabled) {
    if (mqttConnected || mqttWaiting) mqttResetTransport("Disabled");
    return;
  }
  if (!mqttConfig.tlsRequested || mqttConfig.port!=8883) {
    mqttLastError="TLS required on port 8883"; return; // never downgrade
  }
  if (mqttConfig.username.length()==0 || mqttConfig.password.length()==0) {
    mqttLastError="MQTT credentials missing"; return;
  }
  if (!sim800Ready || !gprsRegistered || !gprsAttached || sim800IP.length()==0) {
    mqttConnected=false; mqttWaiting=false; mqttStep=MQ_DOWN;
    mqttLastError="Waiting for SIM800 GPRS bearer"; return;
  }
  if (mqttWaiting) {
    // The RX pump is called from updateSim800 once on each loop.
    unsigned long timeout=(mqttStep==MQ_CONNECT?35000UL: (mqttStep==MQ_PUB||mqttStep==MQ_SUB?15000UL:7000UL));
    if (millis()-mqttStepSince>timeout) mqttFail("MQTT AT timeout");
    return;
  }
  // SMS, calls, and the legacy bearer manager have higher AT priority.
  if (callActive || locPhase!=0 || urcHead!=urcTail || pendCount>0 || pendingAlarmSms || pendingStatusSms) return;
  if (mqttStep==MQ_DOWN) {
    if (mqttLastAttempt && (uint32_t)(millis()-mqttLastAttempt)<mqttBackoffMs) return;
    mqttLastAttempt=millis(); mqttLastError="Checking SIM800 MQTT/TLS";
    // Disconnect stale session from earlier configurations; do not wait for OK.
    // Using SMSTATE? first is safer than a blind disconnect; here use native probe.
    mqttCommand("AT+SMDISC",MQ_DISC);
    return;
  }
  if (mqttStep!=MQ_READY) return;
  if (mqttAckCount>0) { mqttPublish("ack",mqttAckPayloads[mqttAckHead]); return; }
  if (!mqttLastHeartbeat || millis()-mqttLastHeartbeat >= (uint32_t)mqttConfig.heartbeatIntervalSec*1000UL) {
    mqttPublish("status",mqttStatusPayload()); return;
  }
  if (!mqttLastPosition || millis()-mqttLastPosition >= (uint32_t)mqttConfig.positionIntervalSec*1000UL) {
    mqttPublish("gps",mqttGpsPayload()); return;
  }
  if (millis()-mqttLastCheck>60000UL) {
    mqttConnected=false; // wait for confirmed +SMSTATE: 1
    mqttCommand("AT+SMSTATE?",MQ_CHECK);
  }
}

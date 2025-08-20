
// ESP32「禽畜雲端管理」教學範例
// Lab1: 感測資料上傳 (MQTT)
// Lab2: 儀表板+告警（由 Node-RED 完成）
// Lab3: 風扇控制（手動/自動）+ 斷線保護 + LWT
//
// 硬體建議:
//  - DHT22 連到 GPIO4（可改 PIN_DHT）
//  - 水流量感測器 YF-S201 訊號腳到 GPIO27（可改 PIN_FLOW），5V 供電，訊號需與 ESP32 共地
//  - 繼電器模組控制腳到 GPIO25（可改 PIN_RELAY），示範接 12V 風扇（注意功率與安全）
//
// 需安裝函式庫：
//  - PubSubClient (Nick O'Leary)
//  - DHT sensor library (Adafruit) + Adafruit Unified Sensor
//  - ArduinoJson (>=6.x)
//  - NTPClient (arduino-libraries)
//

#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

/********* 使用者需設定區 *********/
const char* WIFI_SSID     = "你的WiFi網路名稱";
const char* WIFI_PASSWORD = "你的WiFi網路密碼";

// MQTT Broker (可用本機 Mosquitto 或雲端 Broker)
const char* MQTT_HOST = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "";   // 若無帳密可留空 ""
const char* MQTT_PASS = "";   // 若無帳密可留空 ""

// Topic 規劃：請與 Node-RED flow 對應
const char* TOPIC_SENSOR       = "xxxxxx/sensor";
const char* TOPIC_CMD_FAN      = "xxxxxx/cmd/fan";
const char* TOPIC_STATE_FAN    = "xxxxxx/state/fan";
const char* TOPIC_STATE_ONLINE = "xxxxxx/state/online";

/********* 腳位與感測器設定 *********/
#define PIN_DHT   4
#define DHTTYPE   DHT22

#define PIN_FLOW  27     // YF-S201 訊號輸入腳位（需外接 10k 下拉電阻較穩）
#define PIN_RELAY 25     // 繼電器控制腳位（依模組可能為 HIGH/LOW 有效）
#define RELAY_ACTIVE_LEVEL HIGH  // 若模組為低電平啟動，改為 LOW

// 是否啟用水流量感測器（教學可先關閉，只上傳溫濕度）
#define USE_FLOW_SENSOR 1

// YF-S201 校正參數（常見約 450 脈衝 / 公升，實際請校正）
const float FLOW_PULSES_PER_LITER = 450.0f;

/********* 自動控制邏輯參數（可由 MQTT 指令覆寫） *********/
float g_thresh_temp_c = 30.0f;  // 自動模式的溫度開啟風扇閾值
enum Mode { MODE_AUTO, MODE_MANUAL };
Mode  g_mode = MODE_AUTO;
bool  g_manual_on = false;

/********* 全域變數 *********/
WiFiClient espClient;
PubSubClient mqtt(espClient);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 8 * 3600 /*台灣時區+8*/, 60 * 1000 /*更新間隔*/);

DHT dht(PIN_DHT, DHTTYPE);

volatile uint32_t g_flow_pulses = 0;   // 中斷計數
unsigned long lastFlowCalcMs = 0;
float lastFlowLpm = 0.0f;

unsigned long lastPubMs = 0;
const unsigned long PUB_INTERVAL_MS = 5000; // 每 5 秒上傳一次

unsigned long lastStatePubMs = 0;
const unsigned long STATE_PUB_INTERVAL_MS = 10000; // 每 10 秒回報一次狀態（避免過於頻繁）

unsigned long lastMqttConnectAttempt = 0;

/********* 中斷服務函式（flow sensor） *********/
#if USE_FLOW_SENSOR
void IRAM_ATTR isr_flow() {
  g_flow_pulses++;
}
#endif

/********* WiFi 連線 *********/
void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect timeout.");
  }
}

/********* 取得毫秒級 Unix 時間戳 *********/
uint64_t epochMillis() {
  // NTPClient 提供秒級，這裡轉毫秒；若尚未同步，回傳 millis-based 的相對時間
  if (timeClient.isTimeSet()) {
    return (uint64_t)timeClient.getEpochTime() * 1000ULL;
  } else {
    return (uint64_t)millis();
  }
}

/********* 風扇控制 *********/
void setFan(bool on) {
  digitalWrite(PIN_RELAY, on ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL);
}

/********* 回報風扇狀態 *********/
void publishFanState() {
  StaticJsonDocument<192> doc;
  doc["mode"] = (g_mode == MODE_AUTO) ? "auto" : "manual";
  bool fanIsOn = (digitalRead(PIN_RELAY) == RELAY_ACTIVE_LEVEL);
  doc["on"] = fanIsOn;
  doc["thresh_temp_c"] = g_thresh_temp_c;
  doc["last_cmd_ts"] = epochMillis();

  uint8_t buf[192];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_STATE_FAN, buf, n, true); // 保留最後狀態（retain）
}

/********* 收到 MQTT 指令 *********/
void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  String t = topic;
  String s; s.reserve(len);
  for (unsigned int i=0;i<len;i++) s += (char)payload[i];
  Serial.printf("MQTT message [%s]: %s\n", t.c_str(), s.c_str());

  if (t == TOPIC_CMD_FAN) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, s);
    if (err) {
      Serial.println("JSON parse error for cmd/fan.");
      return;
    }
    const char* mode = doc["mode"] | "auto";
    if (String(mode) == "auto") g_mode = MODE_AUTO;
    else g_mode = MODE_MANUAL;

    if (doc.containsKey("on")) g_manual_on = doc["on"];
    if (doc.containsKey("thresh_temp_c")) g_thresh_temp_c = doc["thresh_temp_c"];

    // 立即回報一次狀態
    publishFanState();
  }
}

/********* MQTT 連線 *********/
void connectMqtt() {
  if (mqtt.connected()) return;
  unsigned long now = millis();
  if (now - lastMqttConnectAttempt < 3000) return; // 3 秒重試間隔
  lastMqttConnectAttempt = now;

  String clientId = String("esp32-poultry-") + String((uint32_t)ESP.getEfuseMac(), HEX);

  // 設定 LWT：離線時發布 'offline'（retain=true）
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(60);
  bool ok;
  if (strlen(MQTT_USER) > 0) {
    ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS,
                      TOPIC_STATE_ONLINE, 1, true, "offline");
  } else {
    ok = mqtt.connect(clientId.c_str(), NULL, NULL,
                      TOPIC_STATE_ONLINE, 1, true, "offline");
  }

  if (ok) {
    Serial.println("MQTT connected.");

    // 上線訊息（retain）
    mqtt.publish(TOPIC_STATE_ONLINE, "online", true);

    mqtt.subscribe(TOPIC_CMD_FAN, 1);
    // 初次回報狀態
    publishFanState();
  } else {
    Serial.printf("MQTT connect failed, rc=%d\n", mqtt.state());
  }
}

/********* 讀取感測器 *********/
bool readSensors(float& tempC, float& rh, float& lpm) {
  tempC = dht.readTemperature();  // °C
  rh    = dht.readHumidity();     // %
  Serial.print(tempC);
  Serial.print(",");
  Serial.println(rh);
  bool ok = !(isnan(tempC) || isnan(rh));

  lpm = NAN;
#if USE_FLOW_SENSOR
  // 每次呼叫計算期間內的流量，這裡以 5 秒的 PUB_INTERVAL 為基準
  unsigned long now = millis();
  unsigned long intervalMs = now - lastFlowCalcMs;
  if (intervalMs >= PUB_INTERVAL_MS) {
    noInterrupts();
    uint32_t pulses = g_flow_pulses;
    g_flow_pulses = 0;
    interrupts();
    lastFlowCalcMs = now;
#if 0 // Original design   
    // L/min = (pulses / (pulses/L)) / (intervalMin)
    // intervalMin = intervalMs / 60000
    float l = (float)pulses / FLOW_PULSES_PER_LITER;
    lpm = l / ((float)intervalMs / 60000.0f);
#else // My design: using button module to simulate flow counter
    #define FLOW_PER_PULSE  0.02 // L
    float l = (float)pulses * FLOW_PER_PULSE;
    lpm = l / ((float)intervalMs / 60000.0f);
#endif
    lastFlowLpm = lpm;
    Serial.print("new lpm: ");
    Serial.println(lpm);
  } else {
    lpm = lastFlowLpm; // 使用上一個週期的值
    Serial.print("lpm: ");
    Serial.println(lpm);
  }
#endif
  return ok;
}

/********* 上傳感測資料 *********/
void publishSensor(float tempC, float rh, float lpm) {
  StaticJsonDocument<256> doc;
  doc["ts"] = epochMillis();
  doc["temp_c"] = tempC;
  doc["rh"] = rh;
#if USE_FLOW_SENSOR
  if (!isnan(lpm)) doc["flow_lpm"] = lpm;
#endif

  uint8_t buf[256];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_SENSOR, buf, n, false); // 不保留
  Serial.printf("PUB %s\n", buf);
}

/********* 自動化控制（本地保護邏輯） *********/
void localControl(float tempC) {
  bool fanOn = (digitalRead(PIN_RELAY) == RELAY_ACTIVE_LEVEL);

  bool desired = fanOn;
  if (g_mode == MODE_MANUAL && mqtt.connected()) {
    desired = g_manual_on;  // 僅在連線時允許手動模式；斷線自動保護
  } else {
    // 自動模式（或斷線保護）：以溫度閾值控制
    if (!isnan(tempC)) {
      desired = (tempC >= g_thresh_temp_c);
    }
  }

  if (desired != fanOn) {
    setFan(desired);
    publishFanState();  // 狀態改變即時回報（本地到雲）
  }
}

/********* 設定 *********/
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_RELAY, OUTPUT);
  setFan(false);

#if USE_FLOW_SENSOR
  pinMode(PIN_FLOW, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), isr_flow, RISING);
#endif

  dht.begin();
  connectWifi();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  connectMqtt();

  timeClient.begin();
}

/********* 主迴圈 *********/
void loop() {
  connectWifi();
  if (!mqtt.connected()) connectMqtt();
  mqtt.loop();

  timeClient.update();

  unsigned long now = millis();

  // 讀取感測器 & 本地控制
  static float tempC = NAN, rh = NAN, lpm = NAN;
  if (now - lastPubMs >= PUB_INTERVAL_MS) {
    lastPubMs = now;
    bool ok = readSensors(tempC, rh, lpm);
    if (ok) publishSensor(tempC, rh, lpm);
  }

  // 本地控制（每個迴圈都檢查）
  localControl(tempC);

  // 週期性回報風扇狀態（避免儀表板不同步）
  if (now - lastStatePubMs >= STATE_PUB_INTERVAL_MS) {
    lastStatePubMs = now;
    publishFanState();
  }

  delay(5);
}

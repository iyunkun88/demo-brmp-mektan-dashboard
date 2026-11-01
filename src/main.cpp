#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <time.h>

// ==================== CONFIGURATION ====================
const char* ssid          = "Redmi Note 10 Pro"; 
const char* password_wifi = "sekjalakukoqlali"; 

const char* mqtt_server    = "a8ec7c7188494f8db03295616f3f4dbf.s1.eu.hivemq.cloud";
const int   mqtt_port      = 8883;
const char* mqtt_user      = "Ondokun32-cicurug";
const char* mqtt_pass      = "BRMPc1curu9";
const char* mqtt_topic_pub = "KP_Cicurug/GH03";
const char* mqtt_topic_sub = "esp32/irrigation/control";
const char* mqtt_topic_stat= "esp32/irrigation/status";

#define SLAVE_SENSOR_2  2
#define REG_TEMP        1
#define REG_HUM         2

#define RELAY_PIN       25   // Output ke Coil Kontaktor A1
#define OPTO_PIN        32   // Input dari Output Modul Optocoupler 220V
#define LED_WIFI_PIN    12 
#define RX_PIN          16
#define TX_PIN          17
#define RTS_PIN         27 

// Pin Sensor Analog CWT (Kabel Kuning via Voltage Divider 10k/20k)
#define SOIL_HUM_PIN    34 

const char* ntpServer          = "pool.ntp.org";
const long  gmtOffset_sec      = 25200; // GMT+7 (WIB)
const int   daylightOffset_sec = 0;

// ==================== GLOBAL OBJECTS ====================
LiquidCrystal_I2C lcd(0x27, 20, 4);
ModbusMaster node;
HardwareSerial modbusSerial(2);

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ==================== VARIABLES ====================
bool relayState  = false;       
bool autoMode    = false;       
int  autoSubMode = 0; // 0: AUTO_SENSOR (Soil Only), 1: AUTO_TIMER          

float humThreshold = 60.0; // Threshold Kelembaban Tanah (%) dari Web Dashboard

int startHour = 7,  startMinute = 0;
int stopHour  = 7,  stopMinute  = 15;

float currentTemp = 0.0; // Monitoring Only (RS485)
float currentHum  = 0.0; // Soil Moisture Sensor CWT Analog
float currentEnvHum = 0.0; // Deklarasi variabel kelembaban lingkungan RS485

unsigned long previousSensorMillis = 0;
unsigned long lastDisplayUpdate    = 0;
unsigned long lastNetworkCheck     = 0;

bool lastOptoState = HIGH;

// ==================== RS485 FUNCTIONS (MONITORING ONLY) ====================
void preTransmission()  { digitalWrite(RTS_PIN, HIGH); }
void postTransmission() { digitalWrite(RTS_PIN, LOW);  }

float ReadXYMD02(uint8_t slaveID, uint16_t REG) {
  uint8_t result;
  int retry = 0;
  const int maxRetry = 2;

  node.begin(slaveID, modbusSerial);

  while (retry < maxRetry) {
    while (modbusSerial.available() > 0) {
      modbusSerial.read();
    }
    delay(10);

    result = node.readInputRegisters(REG, 1);

    if (result == node.ku8MBSuccess) {
      int16_t rawData = (int16_t)node.getResponseBuffer(0);
      return (float)rawData / 10.0;
    }

    retry++;
    yield();
    delay(20);
  }
  
  return 0.0;
}

// ==================== PEMBACAAN SENSOR ANALOG CWT (KENDALI UTAMA) ====================
float readCWTSoilHumidity() {
  analogSetPinAttenuation(SOIL_HUM_PIN, ADC_11db);
  
  // 1. Sampling 30x untuk meredam noise ADC ESP32
  long sumADC = 0;
  for (int i = 0; i < 30; i++) {
    sumADC += analogRead(SOIL_HUM_PIN);
    delay(3);
  }
  int rawADC = sumADC / 30;

  // 2. Kalkulasi Tegangan Pin & Rekonstruksi Sinyal Sensor
  float vOut = (rawADC / 4095.0) * 3.3; 
  float vSensor = vOut * 1.5; 
  
  // ================= PARAMETER KALIBRASI BARU =================
  // vDry diturunkan ke 0.30V agar tanah kering terbaca di kisaran 10% - 15%
  float vDry = 0.30; // Batas Bawah (Murni Kering Oven)
  float vWet = 0.80; // Batas Atas (Basah Maksimal Jenuh Air)

  // 3. Hitung persentase kelembaban
  float rawHumidity = ((vSensor - vDry) / (vWet - vDry)) * 100.0;
  
  // Clamping Rentang 0 - 100%
  if (rawHumidity > 100.0) rawHumidity = 100.0;
  if (rawHumidity < 0.0)   rawHumidity = 0.0;

  // 4. Exponential Moving Average (EMA Filter) agar pergerakan angka halus
  static float filteredHumidity = 10.0; // Default diinisialisasi 10%
  filteredHumidity = (filteredHumidity * 0.7) + (rawHumidity * 0.3);

  //Serial.printf("[DEBUG] rawADC: %d | vPin: %.3fV | vSensor: %.3fV | SoilHum: %.2f%%\n", 
                //rawADC, vOut, vSensor, filteredHumidity);

  return filteredHumidity;
}

// ==================== CONTROL & LOGIC FUNCTIONS ====================
void setRelay(bool newState, String triggerSource) {
  if (relayState != newState) {
    relayState = newState;
    
    // Kontrol Relay Kontaktor
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    
    Serial.print("[RELAY] Status Berubah -> ");
    Serial.print(relayState ? "ON (KONTAKTOR AKTIF)" : "OFF (KONTAKTOR MATI)");
    Serial.print(" | Trigger: ");
    Serial.println(triggerSource);
    
    // Kirim Feedback Status Realtime ke Web Dashboard
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      StaticJsonDocument<128> docStatus;
      docStatus["relay"]  = relayState ? 1 : 0;
      docStatus["valve"]  = relayState ? 1 : 0;
      docStatus["mode"]   = autoMode ? (autoSubMode == 0 ? "AUTO_SENSOR" : "AUTO_TIMER") : "MANUAL";
      docStatus["source"] = triggerSource;
      docStatus["device_status"] = "ONLINE";
      
      String statusPayload;
      serializeJson(docStatus, statusPayload);
      client.publish(mqtt_topic_stat, statusPayload.c_str());
      client.publish(mqtt_topic_pub, statusPayload.c_str());
    }
  }
}

bool checkTimerMatch() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return false;
  
  int currentMinutesFromMidnight = (timeinfo.tm_hour * 60) + timeinfo.tm_min;
  int startMinutesFromMidnight   = (startHour * 60) + startMinute;
  int stopMinutesFromMidnight    = (stopHour * 60) + stopMinute;

  if (startMinutesFromMidnight < stopMinutesFromMidnight) {
    return (currentMinutesFromMidnight >= startMinutesFromMidnight && currentMinutesFromMidnight < stopMinutesFromMidnight);
  } else {
    return (currentMinutesFromMidnight >= startMinutesFromMidnight || currentMinutesFromMidnight < stopMinutesFromMidnight);
  }
}

// ==================== MQTT CALLBACK ====================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) != mqtt_topic_sub) return;

  char message[length + 1];
  for (unsigned int i = 0; i < length; i++) message[i] = (char)payload[i];
  message[length] = '\0';

  Serial.print("[MQTT Control] Payload: ");
  Serial.println(message);

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (!error) {
    // 1. Reset Darurat
    if (doc.containsKey("reset") && doc["reset"].as<bool>() == true) {
      autoMode = false;
      setRelay(false, "System_Reset_Web");
      return;
    }

    // 2. Kontrol Relay Manual dari Web
    if (doc.containsKey("relay")) {
      autoMode = false;
      int rState = doc["relay"];
      setRelay(rState == 1, "Web_Manual_JSON");
      return;
    }

    // 3. Switch Mode (MANUAL / AUTO_SENSOR / AUTO_TIMER)
    if (doc.containsKey("mode")) {
      String modeStr = doc["mode"].as<String>();
      if (modeStr == "AUTO_SENSOR") { autoMode = true; autoSubMode = 0; }
      else if (modeStr == "AUTO_TIMER") { autoMode = true; autoSubMode = 1; }
      else if (modeStr == "MANUAL") { autoMode = false; }
    }

    // 4. Parameter Auto Sensor (Hanya Mengambil Parameter Kelembaban dari Web)
    if (doc.containsKey("hum_min"))   humThreshold = doc["hum_min"].as<float>();
    if (doc.containsKey("hum_limit")) humThreshold = doc["hum_limit"].as<float>();

    // 5. Parameter Timer Jam ("HH:MM")
    if (doc.containsKey("time_start")) {
      String tStart = doc["time_start"].as<String>();
      startHour   = tStart.substring(0, 2).toInt();
      startMinute = tStart.substring(3, 5).toInt();
    }
    if (doc.containsKey("time_end")) {
      String tEnd = doc["time_end"].as<String>();
      stopHour   = tEnd.substring(0, 2).toInt();
      stopMinute = tEnd.substring(3, 5).toInt();
    }
  } else {
    String strMsg = String(message);
    strMsg.trim();
    strMsg.toUpperCase();

    if (strMsg == "1" || strMsg == "ON") {
      autoMode = false;
      setRelay(true, "Web_Manual_Text");
    } else if (strMsg == "0" || strMsg == "OFF") {
      autoMode = false;
      setRelay(false, "Web_Manual_Text");
    }
  }
}

// ==================== KONEKSI NETWORK NON-BLOCKING ====================
void handleNetwork() {
  unsigned long now = millis();
  
  if (now - lastNetworkCheck >= 15000) {
    lastNetworkCheck = now;

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Reconnecting in background...");
      WiFi.disconnect();
      WiFi.begin(ssid, password_wifi);
    } 
    else if (!client.connected()) {
      Serial.println("[MQTT] Connecting HiveMQ...");
      String clientId = "ESP32Client-" + String(random(0xffff), HEX);
      const char* willTopic   = mqtt_topic_stat;
      const char* willPayload = "{\"device_status\":\"OFFLINE\",\"relay\":0}";

      if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass, willTopic, 1, true, willPayload)) {
        Serial.println("[MQTT] Terhubung!");
        client.subscribe(mqtt_topic_sub);
        client.publish(willTopic, "{\"device_status\":\"ONLINE\"}", true);
      }
    }
  }

  if (WiFi.status() == WL_CONNECTED && client.connected()) {
    client.loop();
  }
}

// ==================== SETUP ====================
void setup() {
  digitalWrite(RELAY_PIN, LOW); 
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
  relayState = false; //

  pinMode(LED_WIFI_PIN, OUTPUT);
  digitalWrite(LED_WIFI_PIN, LOW);
  
  pinMode(OPTO_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Starting...");

  modbusSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  WiFi.begin(ssid, password_wifi);
  
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setKeepAlive(30);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  lcd.clear();
  Serial.println("--- SETUP COMPLETED ---");
}

// ==================== MAIN LOOP ====================
void loop() {
  // 1. Kelola Network secara Non-Blocking
  handleNetwork();

  digitalWrite(LED_WIFI_PIN, (WiFi.status() == WL_CONNECTED) ? HIGH : LOW);

  // ==================== BACA 1 TOMBOL MANUAL (OFFLINE ONLY) ====================
  if (WiFi.status() != WL_CONNECTED) {
    
    // Optocoupler Active-LOW: LOW saat tombol ditekan (220V masuk)
    bool currentBtnState = digitalRead(OPTO_PIN); 
    
    static bool lastBtnState = HIGH;
    static unsigned long lastDebounceTime = 0;

    // Filter Debounce 50ms untuk meredam noise fisik tombol
    if (currentBtnState != lastBtnState) {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > 50) {
      static bool buttonHandled = false;

      // Terdeteksi TEKANAN BARU (Transition HIGH -> LOW)
      if (currentBtnState == LOW && !buttonHandled) {
        buttonHandled = true;
        
        // 1. Matikan mode Auto
        autoMode = false; 

        // 2. Toggle status kontaktor (Balikkan nilai: ON jadi OFF, OFF jadi ON)
        relayState = !relayState; 
        digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);

        if (relayState) {
          Serial.println("[MANUAL SWITCH] Tombol Ditekan -> Kontaktor ON!");
        } else {
          Serial.println("[MANUAL SWITCH] Tombol Ditekan -> Kontaktor OFF!");
        }
      } 
      // Saat tombol dilepas
      else if (currentBtnState == HIGH) {
        buttonHandled = false;
      }
    }
    lastBtnState = currentBtnState;
  }

  // 3. Baca Sensor (Setiap 5 Detik)
  unsigned long currentMillis = millis();
  if (currentMillis - previousSensorMillis >= 5000) {
    previousSensorMillis = currentMillis;

    // Read Temp RS485 (HANYA UNTUK MONITORING)
    currentTemp   = ReadXYMD02(SLAVE_SENSOR_2, REG_TEMP);
    currentEnvHum = ReadXYMD02(SLAVE_SENSOR_2, REG_HUM);
    
    // Read Soil Humidity CWT Analog (KENDALI UTAMA RELAY)
    currentHum = readCWTSoilHumidity();

    Serial.print("Temp Lingkungan: ");
    Serial.print(currentTemp);
    Serial.print(" C | Soil Moisture: ");
    Serial.print(currentHum);
    Serial.print("% | Thresh: ");
    Serial.println(humThreshold);

    // Kirim Log Sensor ke MQTT
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      StaticJsonDocument<256> doc;
      doc["soil_hum"] = currentHum;     // Kelembaban Tanah CWT (Pin 34)
      doc["temp"]     = currentTemp;    // Suhu Udara RS485 (REG 1)
      doc["env_hum"]  = currentEnvHum;  // Kelembaban Udara RS485 (REG 2)
      doc["relay"]    = relayState ? 1 : 0;
      doc["status"]   = "ONLINE";

      char buffer[256];
      serializeJson(doc, buffer);
      client.publish("KP_Cicurug/GH03", buffer);
    }

    // Evaluasi Auto Mode (KENDALI SEPENUHNYA OLEH SOIL SENSOR CWT)
    if (autoMode) {
      if (autoSubMode == 0) { 
        // Jika kelembaban tanah KURANG DARI ATAU SAMA DENGAN Threshold dari Web Dashboard -> RELAY ON
        if (currentHum <= humThreshold) {
          setRelay(true, "Auto_SoilSensor");
        } else {
          setRelay(false, "Auto_SoilSensor");
        }
      } 
      else if (autoSubMode == 1) { 
        // Auto Timer Jam
        if (checkTimerMatch()) {
          setRelay(true, "Auto_Timer");
        } else {
          setRelay(false, "Auto_Timer");
        }
      }
    }
  }

  // ==================== 4. UPDATE LCD 20X4 (BERGANTIAN HALAMAN) ====================
  static unsigned long lastPageSwitch = 0;
  static int lcdPage = 1; // 1: Hal Utama (5 detik), 2: Hal Lingkungan (2 detik)
  
  // Tentukan durasi berdasarkan halaman aktif (Halaman 1 = 5s, Halaman 2 = 2s)
  unsigned long pageDuration = (lcdPage == 1) ? 5000 : 2000;

  // Cek pewaktuan perpindahan halaman
  if (millis() - lastPageSwitch >= pageDuration) {
    lastPageSwitch = millis();
    lcdPage = (lcdPage == 1) ? 2 : 1; // Alternasi Halaman 1 <-> 2
    lcd.clear(); // Bersihkan layar setiap kali berganti halaman
  }

  // --- HALAMAN 1: MONITOR UTAMA (TAMPIL 5 DETIK) ---
  if (lcdPage == 1) {
    // Baris 1: Status WiFi
    lcd.setCursor(0, 0);
    lcd.print("Status:");
    lcd.print(WiFi.status() == WL_CONNECTED ? "ONLINE" : "OFFLINE");

    // Baris 2: Kelembaban Tanah CWT
    lcd.setCursor(0, 1);
    lcd.print("Soil Moist:");
    if (currentHum < 10.0) lcd.print(" ");
    lcd.print(currentHum, 1);
    lcd.print("%");

    // Baris 3: Status Valve / Kontaktor
    lcd.setCursor(0, 2);
    lcd.print("Valve : ");
    lcd.print(relayState ? "ON" : "OFF");

    // Baris 4: Informasi Mode Sistem
    lcd.setCursor(0, 3);
    lcd.print("Mode : ");
    lcd.print(!autoMode ? "MANUAL    " : "AUTOMATIC ");
  }
  // --- HALAMAN 2: SUHU & KELEMBABAN LINGKUNGAN (TAMPIL 2 DETIK) ---
  else {
    // Baris 1: Suhu Udara RS485
    lcd.setCursor(0, 0);
    lcd.print("Temp : ");
    if (currentTemp < 10.0) lcd.print(" ");
    lcd.print(currentTemp, 1);
    lcd.print((char)223); // Simbol Derajat C
    lcd.print("C   ");

    // Baris 2: Kelembaban Udara RS485
    lcd.setCursor(0, 1);
    lcd.print("Humi : ");
    if (currentEnvHum < 10.0) lcd.print(" ");
    lcd.print(currentEnvHum, 1); // Variabel kelembaban udara RS485
    lcd.print("%   ");

    // Baris 3 & 4: Kosongkan
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(0, 3);
    lcd.print("                    ");
  }
}
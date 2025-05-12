#include <Arduino.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_SGP30.h"
#include <Preferences.h> 
#include <PubSubClient.h> 
#include <Wire.h>
#include <WiFi.h>        
#include <WiFiClientSecure.h>
#include <SdsDustSensor.h>

// --- SABİT AYARLAR ---
#define SHT31_I2C_ADDR 0x44 // SHT31 I2C adresi
#define SDS_RX_PIN 16       // ESP32 RX <- SDS TX
#define SDS_TX_PIN 17       // ESP32 TX -> SDS RX

// WiFi Ayarları
const char* ssid = "Burak";           // WiFi Adı
const char* password = "burak1234";   // WiFi Şifresi

// MQTT Ayarları
const char* mqtt_server = "164e3ba85ffa4014a1a1f22ed2dc46ee.s1.eu.hivemq.cloud";      // HiveMQ Cloud URL
const int mqtt_port = 8883;                                                           // MQTT portu (SSL/TLS için 8883)
const char* mqtt_client_id = "ESP32_IAQ_Monitor_Burak";                               // Broker'da benzersiz olmalı
const char* mqtt_topic_temp = "iaq/livingroom/temperature";                           // MQTT topicler
const char* mqtt_topic_hum = "iaq/livingroom/humidity";
const char* mqtt_topic_tvoc = "iaq/livingroom/tvoc";
const char* mqtt_topic_eco2 = "iaq/livingroom/eco2";
const char* mqtt_topic_pm25 = "iaq/livingroom/pm25";
const char* mqtt_topic_pm10 = "iaq/livingroom/pm10";
const char* mqtt_user = "esp32_iaq";                                                  // Broker username
const char* mqtt_password = "Burak2255";                                              // Broker password

// Zamanlama Ayarları (milisaniye cinsinden)
const unsigned long measurementInterval = 10000;        // Her 10 saniyede bir ölçüm yap ve yayınla
const unsigned long baselineSaveInterval = 3600000;     // Her 1 saatte bir baseline kaydet (3600 * 1000)
const unsigned long sdsReadInterval = 30000;            // SDS011'i ne sıklıkla okuyacağımız (30 saniye)
const unsigned long sdsWarmUpTime = 15000;              // SDS011 fanının ısınması için bekleme süresi (15 saniye)
// --- SABİT AYARLAR ---


// Sensör Nesneleri
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_SGP30 sgp;
SdsDustSensor sds(Serial2);            // VEYA SdsDustSensor sds(Serial2); - Seri portu constructor'da ver
HardwareSerial& sdsSerial = Serial2;   // Serial2 referansı kalabilir veya doğrudan kullanılabilir

// NVS Yönetimi için Nesne
Preferences preferences;

// WiFi ve MQTT için Nesneleri
WiFiClientSecure espClientSecure;
PubSubClient client(espClientSecure);

// Zamanlama için Değişkenler
unsigned long lastMeasurementTime = 0;
unsigned long lastBaselineSaveTime = 0;
unsigned long lastSdsReadAttempt = 0;   // Son SDS okuma denemesi zamanı
bool sdsNeedsReading = false;           // SDS okuması gerekiyor mu
unsigned long sdsWakeUpTime = 0;        // SDS'in uyandırıldığı zaman

// --- FONKSİYON PROTOTİPLERİ ---
void setup_wifi();
void connect_mqtt();
void publish_mqtt_data(float temp, float hum, uint16_t tvoc, uint16_t eco2, float pm25, float pm10);
uint32_t getAbsoluteHumidity(float temperature, float humidity);
bool manage_and_read_sds011(float &pm25_ref, float &pm10_ref);
void load_sgp_baseline();
void save_sgp_baseline();
// --- FONKSİYON PROTOTİPLERİ ---


// Mutlak Nem Hesabı
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // Eğer sıcaklık veya nem geçerli değilse, 0 döndür.
    if (isnan(temperature) || isnan(humidity)) {
        return 0;
    }
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

// WiFi Bağlantı Fonksiyonu
void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        retries++;
        if (retries > 20) {     // Yaklaşık 10 saniye bekle, olmazsa sistemi yeniden başlat
             Serial.println("\nWiFi connection failed, restarting...");
             ESP.restart();
        }
    }

    randomSeed(micros());       // MQTT Client ID için

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

// MQTT Bağlantı ve Yeniden Bağlanma Fonksiyonu
void connect_mqtt() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection (secure)...");
        String clientId = mqtt_client_id;
        clientId += String(random(0xffff), HEX);

        // Bağlanmayı dene (Kullanıcı adı ve şifre ile)
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
            Serial.println("connected");
             // Gerekirse abone ol
             // client.subscribe("some/topic");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            // Hata kodları için PubSubClient belgelerine bakabilirsin.
            // -4: Connection refused: Bad username or password
            // -2: Connection refused: Identifier rejected (Client ID sorunu)
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

// MQTT'ye Veri Yayınlama Fonksiyonu
void publish_mqtt_data(float temp, float hum, uint16_t tvoc, uint16_t eco2, float pm25, float pm10) {
    // Eğer MQTT bağlantısı kopmuşsa yeniden bağlanmayı dene
    if (!client.connected()) {
        Serial.println("MQTT disconnected! Attempting reconnect before publish...");
        connect_mqtt();
        delay(100); // Bağlantının oturması için kısa bekleme
        if (!client.connected()){
            Serial.println("Reconnect failed, skipping publish!");
            return; // Bağlanamazsa fonksiyondan çık
        }
        Serial.println("Reconnected!");
    }

    // Değerleri string'e çevirmek için buffer değişkeni
    char buffer[10];

    // Sıcaklık değerini kontrol et ve gönder
    if (!isnan(temp)) {
        dtostrf(temp, 1, 2, buffer);
        client.publish(mqtt_topic_temp, buffer);
    }
    // Nem değerini kontrol et ve gönder
    if (!isnan(hum)) {
        dtostrf(hum, 1, 2, buffer);
        client.publish(mqtt_topic_hum, buffer);
    }
    // TVOC değerni kontrol et ve gönder
    if (tvoc < 60000) {
       snprintf(buffer, sizeof(buffer), "%d", tvoc);
       client.publish(mqtt_topic_tvoc, buffer);
    }
    // eCO2 değerini kontrol et ve gönder
    if (eco2 >= 400 && eco2 < 60000) {
       snprintf(buffer, sizeof(buffer), "%d", eco2);
       client.publish(mqtt_topic_eco2, buffer);
    }
    // PM2.5 değerini kontrol et ve gönder
    if (pm25 >= 0.0) { // Geçerli okuma varsa (hata durumunda -1.0 geliyordu)
        dtostrf(pm25, 1, 1, buffer); // 1 ondalık basamak
        client.publish(mqtt_topic_pm25, buffer);
    }
    // PM10 değerini kontrol et ve gönder
    if (pm10 >= 0.0) { // Geçerli okuma varsa
        dtostrf(pm10, 1, 1, buffer);
        client.publish(mqtt_topic_pm10, buffer);
    }
}

// NVS'den Baseline Yükleme Fonksiyonu
void load_sgp_baseline() {
    preferences.begin("sgp30-base", true);          // Alanını aç (true = read-only)
    uint32_t baseline_eco2 = preferences.getUInt("baselineECO2", 0);
    uint32_t baseline_tvoc = preferences.getUInt("baselineTVOC", 0);
    preferences.end();                              // Alanı kapat

    if (baseline_eco2 != 0 && baseline_tvoc != 0) {
        Serial.println("Found baseline values in NVS. Restoring them.");
        Serial.print("eCO2 Baseline: 0x"); Serial.println(baseline_eco2, HEX);
        Serial.print("TVOC Baseline: 0x"); Serial.println(baseline_tvoc, HEX);
        sgp.setIAQBaseline(baseline_eco2, baseline_tvoc);
    } else {
        Serial.println("No baseline found in NVS or values are zero. SGP30 will start fresh calibration.");
        // İlk çalıştırmada veya NVS boşsa burası çalışır.
    }
}

// NVS'ye Baseline Kaydetme Fonksiyonu
void save_sgp_baseline() {
    uint16_t current_eco2_baseline, current_tvoc_baseline;

    if (sgp.getIAQBaseline(&current_eco2_baseline, &current_tvoc_baseline)) {
        // 0x0000 veya 0xFFFF gibi geçersiz baseline'ları kaydetmemek için
        if (current_eco2_baseline != 0 && current_eco2_baseline != 0xFFFF &&
            current_tvoc_baseline != 0 && current_tvoc_baseline != 0xFFFF)
        {
            preferences.begin("sgp30-base", false);         // Alanı aç (false = read/write)
            preferences.putUInt("baselineECO2", current_eco2_baseline);
            preferences.putUInt("baselineTVOC", current_tvoc_baseline);
            preferences.end();                              // Alanı kapat
            Serial.print("Saved baseline values to NVS -> ");
            Serial.print("eCO2: 0x"); Serial.print(current_eco2_baseline, HEX);
            Serial.print(" / TVOC: 0x"); Serial.println(current_tvoc_baseline, HEX);
        } else {
             Serial.println("Skipping baseline save, received potentially invalid values.");
        }

    } else {
        Serial.println("Failed to get baseline values from SGP30.");
    }
}

// SDS011 Okuma ve Yönetme Fonksiyonu
bool manage_and_read_sds011(float &pm25_ref, float &pm10_ref) {
    unsigned long currentTime = millis();
    bool readSuccess = false;

    // Adım 1: Okuma zamanı geldi mi ve sensör zaten uyanık değil mi?
    if (!sdsNeedsReading && (currentTime - lastSdsReadAttempt >= sdsReadInterval)) {
        Serial.println("SDS011: Waking up sensor...");
        // --- DÜZELTME: WorkingStateResult nesnesini al ve isOk() / isWorking() ile kontrol et ---
        WorkingStateResult wakeupResult = sds.wakeup(); // Nesneyi değişkene ata
        if (!wakeupResult.isOk()) {                     // Genel iletişim hatası var mı?
            Serial.print("SDS011: Wakeup command failed! Error: ");
            Serial.println(wakeupResult.statusToString()); // Hatayı string olarak yazdır
        } else if (!wakeupResult.isWorking()) {         // Başarılı ama hala uyuyor mu?
            Serial.println("SDS011: Wakeup command sent, but sensor didn't confirm it's working.");
        } else {
            Serial.println("SDS011: Sensor woke up successfully.");
        }
        // --- /DÜZELTME ---
        sdsWakeUpTime = currentTime;
        sdsNeedsReading = true;
        lastSdsReadAttempt = currentTime; // Deneme zamanını güncelle
  }

  // Adım 2: Sensör uyanık mı ve ısınma süresi geçti mi?
  if (sdsNeedsReading && (currentTime - sdsWakeUpTime >= sdsWarmUpTime)) {
      Serial.println("SDS011: Warm-up complete. Querying data...");

      // --- DÜZELTME: readMeasurement yerine queryPm kullan ---
      // Measurement measurement = sds.readMeasurement(); // Eski kod
      PmResult pmResult = sds.queryPm(); // Yeni kod: Sensöre sorgu gönder
      // --- /DÜZELTME ---

      // --- DÜZELTME: PmResult kontrolü ---
      // if (measurement.status == SdsDustSensor::Status::Ok) { // Eski kod
      if (pmResult.isOk()) { // Yeni kod: Sonuç başarılı mı?
          pm25_ref = pmResult.pm25;
          pm10_ref = pmResult.pm10;
          Serial.printf("SDS011: Read success -> PM2.5=%.1f ug/m3, PM10=%.1f ug/m3\n", pm25_ref, pm10_ref);
          readSuccess = true;
        } else {
          Serial.print("SDS011: Query failed! Status: ");
          // Serial.println(SdsDustSensor::statusToString(measurement.status)); // Eski kod
          Serial.println(pmResult.statusToString()); // Yeni kod: Durumu string olarak yazdır
          pm25_ref = -1.0; // Hata durumunda varsayılan değerler
          pm10_ref = -1.0;
          readSuccess = false;
        }
      // --- /DÜZELTME ---

      Serial.println("SDS011: Putting sensor back to sleep.");
      // --- DÜZELTME: WorkingStateResult nesnesini al ve isOk() / isWorking() ile kontrol et ---
      WorkingStateResult sleepResult = sds.sleep();   // Nesneyi değişkene ata
       if (!sleepResult.isOk()) {                      // Genel iletişim hatası var mı?
          Serial.print("SDS011: Sleep command failed! Error: ");
          Serial.println(sleepResult.statusToString()); // Hatayı string olarak yazdır
        } else if (sleepResult.isWorking()) {           // Başarılı ama hala çalışıyor mu?
           Serial.println("SDS011: Sleep command sent, but sensor didn't confirm it's sleeping.");
       } else {
            Serial.println("SDS011: Sensor went to sleep successfully.");
       }
      // --- /DÜZELTME ---
      sdsNeedsReading = false; // Okuma tamamlandı veya başarısız oldu, bir sonraki döngüye kadar bekle
  }

  return readSuccess;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("\n\n--- Indoor Air Quality Monitor ---");

    // WiFi Bağlantısı
    setup_wifi();

    // MQTT Ayarları
    client.setServer(mqtt_server, mqtt_port);

    // TLS/SSL bağlantısı için sertifika doğrulamasını pasif hale getir.(şimdilik)
    espClientSecure.setInsecure();
    
    // I2C Başlatma
    Wire.begin(); 

    // SHT3x Sensör Başlatma
    Serial.println("Initializing SHT31...");
    if (!sht31.begin(SHT31_I2C_ADDR)) {
        Serial.println("Couldn't find SHT31 sensor!");
        while (1) delay(10); // Başlamazsa sonsuz döngü
    } else {
        Serial.println("SHT31 Found!");
    }

    // SGP30 Sensör Başlatma
    Serial.println("Initializing SGP30...");
    if (!sgp.begin()){
        Serial.println("Couldn't find SGP30 sensor!");
        while (1) delay(10); // Başlamazsa sonsuz döngü
    } else {
         Serial.println("SGP30 Found!");
         // SGP30'un iç algoritmasını başlatmak için IAQ init çağrısı
         if (!sgp.IAQinit()) {
            Serial.println("SGP30 IAQinit failed!");
            // Burada durmak yerine devam edilebilir, belki sonraki okumalarda düzelir.
         } else {
             Serial.println("SGP30 IAQ init success.");
         }
    }

    // --- SDS011 Sensör Başlatma ---
    Serial.print("Initializing SDS011 on Serial2 (RX:");
    Serial.print(SDS_RX_PIN); Serial.print(", TX:"); Serial.print(SDS_TX_PIN); Serial.print(")...");
    // ÖNCE Serial2 portunu başlat
    sdsSerial.begin(9600, SERIAL_8N1, SDS_RX_PIN, SDS_TX_PIN);
    sds.begin();
    sds.sleep(); // Bu fonksiyon hala olmalı
    Serial.println(" OK (Started in sleep mode).");

    // NVS'den Baseline Yükle
    load_sgp_baseline();

    // Zamanlayıcıları başlat
    lastMeasurementTime = 0;
    lastBaselineSaveTime = millis();                        // İlk kaydı 1 saat sonra yapması için şimdiki zamanı ata
    lastSdsReadAttempt = millis() - sdsReadInterval + 5000; // İlk SDS okumasının başlangıçtan kısa bir süre sonra tetiklenmesini sağla
    
    Serial.println("Setup complete. Starting measurements...");
}


void loop()
{
    // MQTT bağlantısını kontrol et ve gerekirse yeniden bağlan
    if (!client.connected()) {
        connect_mqtt();
    }
    // MQTT istemcisinin arka plan işlerini yapmasına izin ver (gelen mesajları alma vb.)
    client.loop();

    // SDS011 Okuma Fonksiyonunu Çağır
    float pm25_value = -1.0; // Bu döngüdeki okuma için varsayılan
    float pm10_value = -1.0;
    manage_and_read_sds011(pm25_value, pm10_value); // SDS'i yönet, okursa değerleri güncelle

    // Ölçüm zamanını kontrol et (Non-blocking delay)
    unsigned long currentTime = millis();
    if (currentTime - lastMeasurementTime >= measurementInterval) {
        lastMeasurementTime = currentTime; // Zamanlayıcıyı güncelle

        Serial.println("---------------------");
        Serial.print(currentTime / 1000); Serial.println("s: Reading sensors...");

        // --- SHT31 Okuma ---
        float temperature = sht31.readTemperature();
        float humidity = sht31.readHumidity();

        bool sht31_ok = true; // SHT31 okuma durumu
        if (isnan(temperature)) {
            Serial.println("Failed to read temperature from SHT31!");
            sht31_ok = false;
        }
        if (isnan(humidity)) {
            Serial.println("Failed to read humidity from SHT31!");
            sht31_ok = false;
        }

        if(sht31_ok) {
             Serial.print("Temperature: "); Serial.print(temperature); Serial.print(" *C\t");
             Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
        }

        // --- SGP30 için Mutlak Nem Ayarı ---
        // Sadece geçerli SHT31 verisi varsa mutlak nemi hesapla ve ayarla
        if (sht31_ok) {
            uint32_t absHumidity = getAbsoluteHumidity(temperature, humidity);
             if (absHumidity > 0) { // Geçerli mutlak nem değeri hesaplandıysa
                sgp.setHumidity(absHumidity);
             }
        } else {
             Serial.println("Cannot set humidity for SGP30 due to SHT31 read errors.");
        }


        // --- SGP30 Okuma ---
        uint16_t tvoc = 0;  // Başlangıç değeri ata
        uint16_t eco2 = 0;  // Başlangıç değeri ata
        bool sgp30_ok = sgp.IAQmeasure();
        if (!sgp30_ok) {
            Serial.println("SGP30 measurement failed!");
        } else { 
            tvoc = sgp.TVOC;
            eco2 = sgp.eCO2;
            Serial.print("TVOC: "); Serial.print(sgp.TVOC); Serial.print(" ppb\t");
            Serial.print("eCO2: "); Serial.print(sgp.eCO2); Serial.println(" ppm");
        }

        // Fonksiyona tüm sensör değerlerini (pm25_value ve pm10_value dahil) gönder
        publish_mqtt_data(sht31_ok ? temperature : NAN, // SHT31 ok ise sıcaklık, değilse NaN
                          sht31_ok ? humidity : NAN,    // SHT31 ok ise nem, değilse NaN
                          sgp30_ok ? tvoc : 0,          // SGP30 ok ise tvoc, değilse 0
                          sgp30_ok ? eco2 : 0,          // SGP30 ok ise eco2, değilse 0
                          pm25_value,                   // SDS'den okunan PM2.5 değeri (-1.0 olabilir)
                          pm10_value);                  // SDS'den okunan PM10 değeri (-1.0 olabilir)
        Serial.println("Data published via MQTT (if sensors read successfully).");

    }

    // --- Baseline kaydetme zamanını kontrol et---
    if (currentTime - lastBaselineSaveTime >= baselineSaveInterval) {
        lastBaselineSaveTime = currentTime; // Zamanlayıcıyı güncelle
        Serial.println("Attempting to save SGP30 baseline...");
        save_sgp_baseline();
    }
}
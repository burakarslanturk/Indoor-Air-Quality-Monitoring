#ifndef CONFIG_H
#define CONFIG_H

// --- SABİT AYARLAR ---
#define SHT31_I2C_ADDR 0x44 // SHT31 I2C adresi
#define SDS_RX_PIN 16       // ESP32 RX <- SDS TX
#define SDS_TX_PIN 17       // ESP32 TX -> SDS RX

// WiFi Ayarları
extern const char* ssid;
extern const char* password;

// MQTT Ayarları
extern const char* mqtt_server;
extern const int mqtt_port;
extern const char* mqtt_client_id;
extern const char* mqtt_topic_telemetry;
extern const char* mqtt_user;
extern const char* mqtt_password;

// Zamanlama Ayarları (milisaniye cinsinden)
const unsigned long MEASUREMENT_INTERVAL = 10000;        // Her 10 saniyede bir ölçüm yap ve yayınla
const unsigned long BASELINE_SAVE_INTERVAL = 3600000;    // Her 1 saatte bir baseline kaydet (3600 * 1000)
const unsigned long SDS_READ_INTERVAL = 30000;           // SDS011'i ne sıklıkla okuyacağımız (30 saniye)
const unsigned long SDS_WARM_UP_TIME = 15000;            // SDS011 fanının ısınması için bekleme süresi (15 saniye)
const int WIFI_CONNECT_RETRIES = 20;                     // WiFi bağlantı deneme sayısı
const int MQTT_RECONNECT_DELAY_MS = 5000;                // MQTT yeniden bağlanma denemeleri arası bekleme

#endif // CONFIG_H 
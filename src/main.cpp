#include <Arduino.h>
#include "config.h"
#include "wifi_manager.h"
#include "mqtt_handler.h"
#include "sensor_manager.h"

// WiFi Ayarları (config.h'den extern ile geliyor, burada tanımlanıyor)
const char* ssid = "Burak";           // WiFi Adı
const char* password = "burak1234";   // WiFi Şifresi

// MQTT Ayarları (config.h'den extern ile geliyor, burada tanımlanıyor)
const char* mqtt_server = "164e3ba85ffa4014a1a1f22ed2dc46ee.s1.eu.hivemq.cloud";      // HiveMQ Cloud URL
const int mqtt_port = 8883;                                                           // MQTT portu (SSL/TLS için 8883)
const char* mqtt_client_id = "ESP32_IAQ_Monitor_Burak";                               // Broker'da benzersiz olmalı
const char* mqtt_topic_temp = "iaq/ofis/temperature";                                 // MQTT topicler
const char* mqtt_topic_hum = "iaq/ofis/humidity";
const char* mqtt_topic_tvoc = "iaq/ofis/tvoc";
const char* mqtt_topic_eco2 = "iaq/ofis/eco2";
const char* mqtt_topic_pm25 = "iaq/ofis/pm25";
const char* mqtt_topic_pm10 = "iaq/ofis/pm10";
const char* mqtt_user = "esp32_iaq";                                                  // Broker username
const char* mqtt_password = "Burak2255";                                              // Broker password

// Zamanlama için Değişkenler (Sadece ölçüm aralığı için olan main'de kalacak)
unsigned long lastMeasurementTime = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("\n\n--- Indoor Air Quality Monitor (Modular) ---");

    // WiFi Bağlantısı
    setup_wifi();

    // MQTT Ayarları
    setup_mqtt_client();
    
    // Sensörleri Başlat
    setup_sensors();
    
    // Ana ölçüm zamanlayıcısını başlat
    lastMeasurementTime = 0;    // İlk ölçümün hemen yapılmasını sağlamak için 0 veya millis() - MEASUREMENT_INTERVAL
    
    Serial.println("Setup complete. Starting measurements...");
}

void loop()
{
    // MQTT bağlantısını yönet ve client loop'unu çalıştır
    connect_mqtt_if_needed();
    mqtt_loop();

    // Sensör verilerini periyodik olarak oku
    unsigned long currentTime = millis();
    if (currentTime - lastMeasurementTime >= MEASUREMENT_INTERVAL) {
        lastMeasurementTime = currentTime; 

        Serial.println("---------------------");
        Serial.print(currentTime / 1000); Serial.println("s: Reading sensors...");

        SensorData currentSensorData;
        manage_sensors_and_read_data(currentSensorData);  // sensor_manager'dan verileri al

        // Verileri MQTT ile yayınla
        publish_mqtt_data(currentSensorData.temperature, 
                          currentSensorData.humidity,    
                          currentSensorData.tvoc,          
                          currentSensorData.eco2,                 
                          currentSensorData.pm25,                   
                          currentSensorData.pm10);
                                         
        Serial.println("Data potentially published via MQTT.");
    }

    // Sensörler için periyodik diğer görevleri çalıştır (baseline kaydetme)
    periodic_sensor_tasks(); 
}
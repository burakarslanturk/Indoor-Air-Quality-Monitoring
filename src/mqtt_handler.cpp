#include "mqtt_handler.h"
#include "config.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <ArduinoJson.h> // For JSON payload

// WiFi ve MQTT için Nesneler
WiFiClient espClient;
PubSubClient client(espClient);

// MQTT Client Ayarları
void setup_mqtt_client() {
    client.setServer(mqtt_server, mqtt_port);
    // TLS/SSL bağlantısı için sertifika doğrulamasını pasif hale getir.(şimdilik)
    // Daha güvenli bir yöntem için buraya CA sertifikası eklenmeli.
    //espClient.setInsecure(); 
    randomSeed(micros());   // MQTT Client ID eşsiz yapmak için.
}

// MQTT Bağlantı ve Yeniden Bağlanma Fonksiyonu
void connect_mqtt_if_needed() {
    // Sadece bağlantı yoksa bağlanmayı dene
    if (!client.connected()) {
        Serial.print("Attempting MQTT connection (secure)... ClientID: ");
        String clientId = mqtt_client_id;
        clientId += String(random(0xffff), HEX);
        Serial.print(clientId);

        // Bağlanmayı dene (Kullanıcı adı ve şifre ile)
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
            Serial.println(" ...connected");
             // Gerekirse abone ol
             // client.subscribe("some/topic");
        } else {
            Serial.print(" ...failed, rc=");
            Serial.print(client.state());
            Serial.print(" try again in "); Serial.print(MQTT_RECONNECT_DELAY_MS / 1000); Serial.println(" seconds");
            // Hata kodları için PubSubClient belgelerine bakabilirsin.
            // -4: Connection refused: Bad username or password
            // -2: Connection refused: Identifier rejected (Client ID sorunu)
            // Hemen yeniden denemek yerine bir sonraki loop turunda denenecek.
            // Eğer acil bağlanması gerekiyorsa burada bir delay ve retry mekanizması kurulabilir
            // ancak loop içinde periyodik kontrol genellikle daha iyi.
        }
    }
}

// MQTT'ye Veri Yayınlama Fonksiyonu
void publish_mqtt_data(float temp, float hum, uint16_t tvoc, uint16_t eco2, float pm25, float pm10) {
    // Önce bağlantıyı kontrol et, yoksa bağlanmayı dene
    connect_mqtt_if_needed(); 
    delay(100); // MQTT bağlantısının oturması veya deneme sonrası için kısa bir bekleme
                // connect_mqtt_if_needed blocking olmadığı için burası kritik olabilir.
                // Daha iyisi, publish denemeden önce client.connected() tekrar kontrol edilebilir.

    if (!client.connected()){
        Serial.println("MQTT still not connected, skipping publish!");
        return; // Bağlanamazsa fonksiyondan çık
    }

    // JSON belgesi oluştur
    StaticJsonDocument<256> doc; // JSON kapasitesini ihtiyaca göre ayarlayın

    // Sıcaklık değerini kontrol et ve JSON belgesine ekle
    if (!isnan(temp)) {
        doc["temperature"] = temp;
    }
    
    // Nem değerini kontrol et ve JSON belgesine ekle
    if (!isnan(hum)) {
        doc["humidity"] = hum;
    }

    // TVOC değerini kontrol et ve JSON belgesine ekle
    if (tvoc < 60000) { 
       doc["tvoc"] = tvoc;
    }

    // eCO2 değerini kontrol et ve JSON belgesine ekle
    if (eco2 >= 400 && eco2 < 60000) { 
       doc["eco2"] = eco2;
    }

    // PM2.5 değerini kontrol et ve JSON belgesine ekle
    if (pm25 >= 0.0) { 
        doc["pm25"] = pm25;
    }

    // PM10 değerini kontrol et ve JSON belgesine ekle
    if (pm10 >= 0.0) { 
        doc["pm10"] = pm10;
    }

    // JSON'u bir string'e serialize et
    char jsonBuffer[256];
    size_t n = serializeJson(doc, jsonBuffer);

    // JSON payload'ını yayınla
    if (n > 0) {
        client.publish(mqtt_topic_telemetry, jsonBuffer, n);
        Serial.print("Published JSON to ThingsBoard: ");
        Serial.println(jsonBuffer);
    } else {
        Serial.println("Failed to serialize JSON or JSON is empty.");
    }
}

void mqtt_loop() {
    if (client.connected()) { // Sadece bağlıysa loop çağır, bazı kütüphaneler sorun çıkarabilir
        client.loop();
    }
} 
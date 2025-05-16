#include "mqtt_handler.h"
#include "config.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Arduino.h>

// WiFi ve MQTT için Nesneler
WiFiClientSecure espClientSecure;
PubSubClient client(espClientSecure);

// MQTT Client Ayarları
void setup_mqtt_client() {
    client.setServer(mqtt_server, mqtt_port);
    // TLS/SSL bağlantısı için sertifika doğrulamasını pasif hale getir.(şimdilik)
    // Daha güvenli bir yöntem için buraya CA sertifikası eklenmeli.
    espClientSecure.setInsecure(); 
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

    // Değerleri string'e çevirmek için buffer değişkeni
    char buffer[16];

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
    if (tvoc < 60000) { // 0 da geçerli bir TVOC olabilir, SGP30'un başlangıç değeri.
       snprintf(buffer, sizeof(buffer), "%d", tvoc);
       client.publish(mqtt_topic_tvoc, buffer);
    }
    // eCO2 değerini kontrol et ve gönder
    // SGP30 için eCO2 minimum 400ppm'dir. 0 genellikle başlangıç/hata durumudur.
    if (eco2 >= 400 && eco2 < 60000) { 
       snprintf(buffer, sizeof(buffer), "%d", eco2);
       client.publish(mqtt_topic_eco2, buffer);
    }
    // PM2.5 değerini kontrol et ve gönder
    if (pm25 >= 0.0) { // Geçerli okuma varsa (hata durumunda -1.0 geliyordu)
        dtostrf(pm25, 1, 1, buffer);
        client.publish(mqtt_topic_pm25, buffer);
    }
    // PM10 değerini kontrol et ve gönder
    if (pm10 >= 0.0) { // Geçerli okuma varsa
        dtostrf(pm10, 1, 1, buffer);
        client.publish(mqtt_topic_pm10, buffer);
    }
}

void mqtt_loop() {
    if (client.connected()) { // Sadece bağlıysa loop çağır, bazı kütüphaneler sorun çıkarabilir
        client.loop();
    }
} 
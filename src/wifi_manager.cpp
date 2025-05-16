#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "wifi_manager.h"

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
        if (retries > WIFI_CONNECT_RETRIES) { // config.h'den
             Serial.println("\nWiFi connection failed, restarting...");
             ESP.restart();
        }
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
} 
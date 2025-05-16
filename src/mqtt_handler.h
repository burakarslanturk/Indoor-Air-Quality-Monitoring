#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <Arduino.h>

// Fonksiyon prototipleri
void setup_mqtt_client();
void connect_mqtt_if_needed();
void publish_mqtt_data(float temp, float hum, uint16_t tvoc, uint16_t eco2, float pm25, float pm10);
void mqtt_loop(); // client.loop() için sarmalayıcı

#endif // MQTT_HANDLER_H 
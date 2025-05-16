#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h> // Gerekli olabilir

// Sensör verilerini tutmak için bir yapı (struct) tanımlayabiliriz
struct SensorData {
    float temperature;
    float humidity;
    uint16_t tvoc;
    uint16_t eco2;
    float pm25;
    float pm10;
    bool sht31_ok;
    bool sgp30_ok;
    bool sds011_ok;
};

void setup_sensors();
void manage_sensors_and_read_data(SensorData &data); // Tüm sensör okumalarını yapar ve veriyi doldurur
void periodic_sensor_tasks(); // Baseline kaydetme gibi periyodik işler için

// Bu fonksiyonlar artık sensor_manager içinde dahili olacak veya 
// manage_sensors_and_read_data tarafından yönetilecek.
// uint32_t getAbsoluteHumidity(float temperature, float humidity); // Dahili olabilir
// bool manage_and_read_sds011(float &pm25_ref, float &pm10_ref); // manage_sensors_and_read_data içinde hallolacak
// void load_sgp_baseline(); // setup_sensors içinde çağrılacak
// void save_sgp_baseline(); // periodic_sensor_tasks içinde çağrılacak

#endif // SENSOR_MANAGER_H 
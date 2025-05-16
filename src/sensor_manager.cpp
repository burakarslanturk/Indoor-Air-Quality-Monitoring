#include "sensor_manager.h"
#include "config.h" 
#include "Adafruit_SHT31.h"
#include "Adafruit_SGP30.h"
#include <Preferences.h> 
#include <SdsDustSensor.h>
#include <Wire.h>

// Sensör Nesneleri 
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_SGP30 sgp;
SdsDustSensor sds(Serial2);           // Serial2'yi doğrudan kullanıyoruz
HardwareSerial& sdsSerial = Serial2;  // Referans olarak da tutabiliriz, sds(Serial2) yeterli olabilir.
                                      // SdsDustSensor kütüphanesi Serial2'yi doğrudan alıyorsa bu satıra gerek kalmaz.
                                      // Şimdilik bırakalım, sds.begin() öncesi sdsSerial.begin() için.

// NVS Yönetimi için Nesne
Preferences preferences;

// SDS011 ve Baseline Kaydetme Zamanlaması için Değişkenler 
static unsigned long lastBaselineSaveTime = 0;
static unsigned long lastSdsReadAttempt = 0;   
static bool sdsNeedsReading = false;          
static unsigned long sdsWakeUpTime = 0;        

// --- Dahili Yardımcı Fonksiyon Prototipleri ---
static uint32_t getAbsoluteHumidity(float temperature, float humidity);
static bool read_sds011_data(float &pm25_ref, float &pm10_ref); // manage_and_read_sds011'in parçası
static void load_sgp_baseline_internal();
static void save_sgp_baseline_internal();


void setup_sensors() {
    // I2C Başlatma (SHT31 ve SGP30 için ortak)
    Wire.begin(); 

    // SHT3x Sensör Başlatma
    Serial.println("Initializing SHT31...");
    if (!sht31.begin(SHT31_I2C_ADDR)) {
        Serial.println("Couldn't find SHT31 sensor!");
        while (1) delay(10); 
    } else {
        Serial.println("SHT31 Found!");
    }

    // SGP30 Sensör Başlatma
    Serial.println("Initializing SGP30...");
    if (!sgp.begin()){
        Serial.println("Couldn't find SGP30 sensor!");
        while (1) delay(10); 
    } else {
         Serial.println("SGP30 Found!");
         if (!sgp.IAQinit()) {
            Serial.println("SGP30 IAQinit failed!");
         } else {
             Serial.println("SGP30 IAQ init success.");
         }
    }

    // SDS011 Sensör Başlatma
    Serial.print("Initializing SDS011 on Serial2 (RX:");
    Serial.print(SDS_RX_PIN); Serial.print(", TX:"); Serial.print(SDS_TX_PIN); Serial.print(")...");
    sdsSerial.begin(9600, SERIAL_8N1, SDS_RX_PIN, SDS_TX_PIN); // Serial2'yi SDS için başlat
    sds.begin(); // Kütüphanenin kendi begin'i, Serial portunu tekrar başlatmayabilir.
    sds.sleep(); 
    Serial.println(" OK (Started in sleep mode).");

    // NVS'den Baseline Yükle
    load_sgp_baseline_internal();

    // Zamanlayıcıları başlat
    lastBaselineSaveTime = millis(); 
    lastSdsReadAttempt = millis() - SDS_READ_INTERVAL + 5000; // İlk okumayı erken tetikle
}

void manage_sensors_and_read_data(SensorData &data) {
    unsigned long currentTime = millis();
    data.sht31_ok = false;
    data.sgp30_ok = false;
    data.sds011_ok = false;
    data.temperature = NAN;
    data.humidity = NAN;
    data.tvoc = 0;
    data.eco2 = 0;
    data.pm25 = -1.0;
    data.pm10 = -1.0;

    // --- SHT31 Okuma ---
    float temp_sht = sht31.readTemperature();
    float hum_sht = sht31.readHumidity();

    if (!isnan(temp_sht) && !isnan(hum_sht)) {
        data.temperature = temp_sht;
        data.humidity = hum_sht;
        data.sht31_ok = true;
        Serial.print("SHT31: Temp="); Serial.print(data.temperature); Serial.print(" *C, Hum="); Serial.print(data.humidity); Serial.println(" %");
        
        // SGP30 için Mutlak Nem Ayarı
        uint32_t absHumidity = getAbsoluteHumidity(data.temperature, data.humidity);
        if (absHumidity > 0) {
            sgp.setHumidity(absHumidity);
        } else {
            Serial.println("SGP30: Invalid absolute humidity, not setting.");
        }
    } else {
        Serial.println("SHT31: Failed to read data.");
        if (isnan(temp_sht)) Serial.println("SHT31: Failed to read temperature!");
        if (isnan(hum_sht)) Serial.println("SHT31: Failed to read humidity!");
    }

    // --- SGP30 Okuma ---
    if (sgp.IAQmeasure()) {
        data.tvoc = sgp.TVOC;
        data.eco2 = sgp.eCO2;
        data.sgp30_ok = true;
        Serial.print("SGP30: TVOC="); Serial.print(data.tvoc); Serial.print(" ppb, eCO2="); Serial.print(data.eco2); Serial.println(" ppm");
    } else {
        Serial.println("SGP30: Measurement failed!");
    }

    // --- SDS011 Okuma Yönetimi ---
    data.sds011_ok = read_sds011_data(data.pm25, data.pm10);
    if (data.sds011_ok) {
        Serial.printf("SDS011: Read success -> PM2.5=%.1f ug/m3, PM10=%.1f ug/m3\n", data.pm25, data.pm10);
    } else {
        // read_sds011_data fonksiyonu kendi loglamasını yapar, burada sadece okuma yapılmadığı belirtilebilir.
        // Serial.println("SDS011: Failed to read data in this cycle.");
    }
}

void periodic_sensor_tasks() {
    unsigned long currentTime = millis();
    // Baseline kaydetme zamanını kontrol et
    if (currentTime - lastBaselineSaveTime >= BASELINE_SAVE_INTERVAL) {
        lastBaselineSaveTime = currentTime; 
        Serial.println("SensorManager: Attempting to save SGP30 baseline...");
        save_sgp_baseline_internal();
    }
}

// --- Dahili Yardımcı Fonksiyonlar ---
static uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    if (isnan(temperature) || isnan(humidity)) {
        return 0;
    }
    const float absoluteHumidity_val = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature));
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity_val);
    return absoluteHumidityScaled;
}

static void load_sgp_baseline_internal() {
    preferences.begin("sgp30-base", true); 
    uint32_t baseline_eco2 = preferences.getUInt("baselineECO2", 0);
    uint32_t baseline_tvoc = preferences.getUInt("baselineTVOC", 0);
    preferences.end();

    if (baseline_eco2 != 0 && baseline_tvoc != 0) {
        Serial.println("SensorManager: Found SGP30 baseline in NVS. Restoring.");
        Serial.print("  eCO2 Baseline: 0x"); Serial.println(baseline_eco2, HEX);
        Serial.print("  TVOC Baseline: 0x"); Serial.println(baseline_tvoc, HEX);
        sgp.setIAQBaseline(baseline_eco2, baseline_tvoc);
    } else {
        Serial.println("SensorManager: No SGP30 baseline in NVS or values are zero. Starting fresh calibration.");
    }
}

static void save_sgp_baseline_internal() {
    uint16_t current_eco2_baseline, current_tvoc_baseline;
    if (sgp.getIAQBaseline(&current_eco2_baseline, &current_tvoc_baseline)) {
        if (current_eco2_baseline != 0 && current_eco2_baseline != 0xFFFF &&
            current_tvoc_baseline != 0 && current_tvoc_baseline != 0xFFFF) {
            preferences.begin("sgp30-base", false); 
            preferences.putUInt("baselineECO2", current_eco2_baseline);
            preferences.putUInt("baselineTVOC", current_tvoc_baseline);
            preferences.end();
            Serial.print("SensorManager: Saved SGP30 baseline to NVS -> ");
            Serial.print("eCO2: 0x"); Serial.print(current_eco2_baseline, HEX);
            Serial.print(" / TVOC: 0x"); Serial.println(current_tvoc_baseline, HEX);
        } else {
             Serial.println("SensorManager: Skipping SGP30 baseline save, received potentially invalid values.");
        }
    } else {
        Serial.println("SensorManager: Failed to get SGP30 baseline values.");
    }
}

static bool read_sds011_data(float &pm25_ref, float &pm10_ref) {
    unsigned long currentTime = millis();
    bool readSuccess = false;
    pm25_ref = -1.0; // Varsayılan hata değeri
    pm10_ref = -1.0; // Varsayılan hata değeri

    // Okuma zamanı geldi mi ve sensör zaten uyanık mı?
    if (!sdsNeedsReading && (currentTime - lastSdsReadAttempt >= SDS_READ_INTERVAL)) {
        Serial.println("SDS011: Waking up sensor...");
        WorkingStateResult wakeupResult = sds.wakeup();
        if (!wakeupResult.isOk()) {
            Serial.print("SDS011: Wakeup command failed! Error: ");
            Serial.println(wakeupResult.statusToString());
        } else if (!wakeupResult.isWorking()) {
            Serial.println("SDS011: Wakeup command sent, but sensor didn't confirm it's working.");
        } else {
            Serial.println("SDS011: Sensor woke up successfully.");
        }
        sdsWakeUpTime = currentTime;
        sdsNeedsReading = true;
        lastSdsReadAttempt = currentTime; 
    }

    // Sensör uyanık mı ve ısınma süresi geçti mi?
    if (sdsNeedsReading && (currentTime - sdsWakeUpTime >= SDS_WARM_UP_TIME)) {
        Serial.println("SDS011: Warm-up complete. Querying data...");
        PmResult pmResult = sds.queryPm(); 
        if (pmResult.isOk()) { 
            pm25_ref = pmResult.pm25;
            pm10_ref = pmResult.pm10;
            // Başarı logu manage_sensors_and_read_data içinde atılacak
            readSuccess = true;
        } else {
            Serial.print("SDS011: Query failed! Status: ");
            Serial.println(pmResult.statusToString());
            // pm25_ref ve pm10_ref zaten -1.0 olarak ayarlı
        }
        Serial.println("SDS011: Putting sensor back to sleep.");
        WorkingStateResult sleepResult = sds.sleep();
        if (!sleepResult.isOk()) {
            Serial.print("SDS011: Sleep command failed! Error: ");
            Serial.println(sleepResult.statusToString());
        } else if (sleepResult.isWorking()) {
            Serial.println("SDS011: Sleep command sent, but sensor didn't confirm it's sleeping.");
        } else {
            Serial.println("SDS011: Sensor went to sleep successfully.");
        }
        sdsNeedsReading = false; 
    }
    return readSuccess;
} 
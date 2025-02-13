/*
 *   9-Eksen AccGyroMagne Veri Okuma örneği,
 *
 *   Bu örnekte temel konfigürasyon ayarları yapılmaktadır.
 *   Sensörlerden gelen İvmeölçerden(Acc) X,Y,Z Dönüölçerden(Gyro) X,Y,Z, Manyetometre(Magne) X,Y,Z ve Sıcaklık C ve F cinsinden sıcaklık değerlerini
 *   seri port ekranına yazdırmaktadır.
 *
 *   Bu algılayıcı I2C haberleşme protokolü ile çalışmaktadır.
 *
 *   Bu örnek Deneyap 9-Eksen Ataletsel Ölçüm Birimi için oluşturulmuştur
 *      ------> https://docs.deneyapkart.org/tr/content/contentDetail/deneyap-modul-deneyap-9-eksen-ataletsel-olcum-biri <------
 *      ------> https://github.com/deneyapkart/deneyap-9-eksen-ataletsel-olcum-birimi-arduino-library <------
 */

#include <LSM6DSL.h>
#include <Adafruit_MMC56x3.h>
#include <ArduinoJson.h>

#define IMU_ADDRESS 0x6B

/* Assign a unique ID to this sensor at the same time */
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);
JsonDocument doc;
LSM6DSL imu(LSM6DSL_MODE_I2C, 0x6B);

void setup() { 

    Wire.begin();
    Wire.setClock(400000); //400khz clock
    Serial.begin(115200);

    imu.settings.gyroSampleRate=104;
    imu.settings.accelSampleRate=104;

    imu.begin();    
    while (!mmc.begin(MMC56X3_DEFAULT_ADDRESS, &Wire));                                         // begin(slaveAdress) fonksiyonu ile cihazların haberleşmesi başlatılması
}

void loop() {                             // X-eksen akselerometre verisi okuma
    JsonArray accelData=doc["ACC"].to<JsonArray>();
    JsonArray gyroData=doc["GYR"].to<JsonArray>();
    JsonArray magData=doc["MAG"].to<JsonArray>();
  
    sensors_event_t event;
    mmc.getEvent(&event);
    accelData.add(imu.readFloatAccelX());
    accelData.add(imu.readFloatAccelY());
    accelData.add(imu.readFloatAccelZ());
    gyroData.add(imu.readFloatGyroX());
    gyroData.add(imu.readFloatGyroY());
    gyroData.add(imu.readFloatGyroZ());
    magData.add(event.magnetic.x);
    magData.add(event.magnetic.y);
    magData.add(event.magnetic.z);
    serializeJson(doc,Serial);
    Serial.println();

}

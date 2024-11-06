#ifndef SENSOR_CPU_H
#define SENSOR_CPU_H

#include <Adafruit_BMP3XX.h>
#include <ICM20948_WE.h>
#include <Wire.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "FRAMBuffer.h"
#include "FlightStateManager.h"
#include "ESPNow_Transmitter.h"

#define BMP_CS 2            // Pino SPI para o BMP388
#define ICM_CS 1            // Pino SPI para o ICM20948
#define FRAM_I2C_ADDR 0x50  // Endereço I2C da FRAM
#define FRAM_SIZE 32768     // Tamanho total da FRAM em bytes

struct TelemetryData {
    uint32_t packetNumber;
    uint32_t timestamp;
    float temperature;
    float pressure;
    float accX, accY, accZ;
    float gyrX, gyrY, gyrZ;
    float magX, magY, magZ;
};

class SensorCPU {
public:
    SensorCPU(const uint8_t *receiverMacAddress) : bmp(), myICM(ICM_CS, spi), flightStateManager(), packetNumber(0), framBuffer(), espNowTransmitter(receiverMacAddress) {}

    void begin() {
        Serial.begin(115200);

        // Configuração do BMP388
        if (!bmp.begin_SPI(BMP_CS)) {
            Serial.println("Falha ao inicializar o BMP388!");
            while (1);
        }

        // Configuração do ICM20948
        if (!myICM.init()) {
            Serial.println("Falha ao inicializar o ICM20948!");
            while (1);
        }
        if (!myICM.initMagnetometer()) {
            Serial.println("Magnetômetro não responde");
        } else {
            Serial.println("Magnetômetro conectado");
        }

        // Inicializa a FRAM e carrega o estado
        if (!framBuffer.begin()) {
            Serial.println("Falha ao inicializar a FRAM.");
            while (1);
        }

        // Inicializa o ESP-NOW
        if (!espNowTransmitter.begin()) {
            Serial.println("Falha ao inicializar ESP-NOW");
            while (1);
        }

        float initialPressure = framBuffer.getInitialPressure();
        uint8_t flightState = framBuffer.getFlightState();

        Serial.print("Pressão inicial: ");
        Serial.println(initialPressure);
        Serial.print("Estado do voo: ");
        Serial.println(flightState);

        // Criação do semáforo e fila
        xSemaphore = xSemaphoreCreateBinary();
        if (xSemaphore == NULL) {
            Serial.println("Falha ao criar o semáforo!");
            while (1);
        }
        xSemaphoreGive(xSemaphore);

        dataQueue = xQueueCreate(10, sizeof(TelemetryData));
        if (dataQueue == NULL) {
            Serial.println("Falha ao criar a fila de dados!");
            while (1);
        }

        // Criação das tarefas para leitura dos sensores, armazenamento na FRAM e envio via ESP-NOW
        xTaskCreatePinnedToCore(SensorCPU::readBMP388, "Read BMP388", 2048, this, 1, NULL, 1);
        xTaskCreatePinnedToCore(SensorCPU::readICM20948, "Read ICM20948", 2048, this, 1, NULL, 1);
        xTaskCreatePinnedToCore(SensorCPU::storeToFRAM, "Store to FRAM", 2048, this, 1, NULL, 1);
        xTaskCreatePinnedToCore(SensorCPU::sendTelemetryESPNow, "Send ESP-NOW", 2048, this, 1, NULL, 1);
    }

private:
    Adafruit_BMP3XX bmp;
    ICM20948_WE myICM;
    FRAMBuffer<TelemetryData> framBuffer;
    FlightStateManager flightStateManager;
    ESPNowTransmitter<TelemetryData> espNowTransmitter;
    SemaphoreHandle_t xSemaphore;
    QueueHandle_t dataQueue;
    volatile uint32_t packetNumber;
    static bool spi;

    static void readBMP388(void* parameter) {
        SensorCPU* self = static_cast<SensorCPU*>(parameter);
        for (;;) {
            if (self->bmp.performReading()) {
                float temperature = self->bmp.temperature;
                float pressure = self->bmp.pressure / 100.0;  // Conversão para hPa
                self->packetNumber++;

                xSemaphoreGive(self->xSemaphore);
            }
            vTaskDelay(100);  // 10 Hz
        }
    }

    static void readICM20948(void* parameter) {
        SensorCPU* self = static_cast<SensorCPU*>(parameter);
        for (;;) {
            if (xSemaphoreTake(self->xSemaphore, portMAX_DELAY) == pdTRUE) {
                self->myICM.readSensor();
                xyzFloat gValue = self->myICM.getGValues();
                xyzFloat gyr = self->myICM.getGyrValues();
                xyzFloat magValue = self->myICM.getMagValues();
                float temp = self->myICM.getTemperature();
                float resultantG = self->myICM.getResultantG(gValue);
                uint32_t timestamp = millis();

                TelemetryData data;
                data.packetNumber = self->packetNumber;
                data.timestamp = timestamp;
                data.temperature = self->bmp.temperature;
                data.pressure = self->bmp.pressure / 100.0;
                data.accX = gValue.x;
                data.accY = gValue.y;
                data.accZ = gValue.z;
                data.gyrX = gyr.x;
                data.gyrY = gyr.y;
                data.gyrZ = gyr.z;
                data.magX = magValue.x;
                data.magY = magValue.y;
                data.magZ = magValue.z;

                if (xQueueSend(self->dataQueue, &data, portMAX_DELAY) != pdPASS) {
                    Serial.println("Falha ao enviar dados para a fila");
                }

                self->flightStateManager.updateState(gValue.x, gValue.y, gValue.z, self->bmp.pressure / 100.0, gyr.x, gyr.y, gyr.z, timestamp);
            }
            vTaskDelay(100);  // 10 Hz
        }
    }

    static void storeToFRAM(void* parameter) {
        SensorCPU* self = static_cast<SensorCPU*>(parameter);
        TelemetryData data;
        for (;;) {
            if (xQueueReceive(self->dataQueue, &data, portMAX_DELAY) == pdTRUE) {
                self->framBuffer.writeData(data);
                self->readTelemetryDataFromFRAM(data.packetNumber);
            }
        }
    }

    static void sendTelemetryESPNow(void* parameter) {
        SensorCPU* self = static_cast<SensorCPU*>(parameter);
        TelemetryData data;
        for (;;) {
            if (xQueueReceive(self->dataQueue, &data, portMAX_DELAY) == pdTRUE) {
                self->espNowTransmitter.sendTelemetry(data);
            }
        }
    }

    void readTelemetryDataFromFRAM(uint32_t packetNumber) {
        TelemetryData data;
        if (framBuffer.readData(data)) {
            Serial.print("Pacote: ");
            Serial.print(data.packetNumber);
            Serial.print(", Tempo: ");
            Serial.print(data.timestamp);
            Serial.print(" ms, Temperatura: ");
            Serial.print(data.temperature);
            Serial.print(" C, Pressão: ");
            Serial.print(data.pressure);
            Serial.print(" hPa, Aceleração: X=");
            Serial.print(data.accX);
            Serial.print(" Y=");
            Serial.print(data.accY);
            Serial.print(" Z=");
            Serial.print(data.accZ);
            Serial.print(" Giroscópio: X=");
            Serial.print(data.gyrX);
            Serial.print(" Y=");
            Serial.print(data.gyrY);
            Serial.print(" Z=");
            Serial.print(data.gyrZ);
            Serial.print(" Magnetômetro: X=");
            Serial.print(data.magX);
            Serial.print(" Y=");
            Serial.print(data.magY);
            Serial.print(" Z=");
            Serial.println(data.magZ);
        } else {
            Serial.println("Falha ao ler o pacote da FRAM.");
        }
    }
};

bool SensorCPU::spi = true;

#endif // SENSOR_CPU_H

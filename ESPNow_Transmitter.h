#ifndef ESPNOW_TRANSMITTER_H
#define ESPNOW_TRANSMITTER_H

#include <esp_now.h>
#include <WiFi.h>

template <typename T>
class ESPNowTransmitter {
public:
    ESPNowTransmitter(const uint8_t *receiverMacAddress) {
        memcpy(macAddress, receiverMacAddress, 6);
    }

    bool begin() {
        WiFi.mode(WIFI_STA);
        if (esp_now_init() != ESP_OK) {
            Serial.println("Erro ao inicializar ESP-NOW");
            return false;
        }
        esp_now_register_send_cb(onDataSent);

        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, macAddress, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("Erro ao adicionar peer ESP-NOW");
            return false;
        }
        return true;
    }

    void sendTelemetry(const T &data) {
        esp_err_t result = esp_now_send(macAddress, (uint8_t *)&data, sizeof(T));
        if (result == ESP_OK) {
            Serial.println("Dados enviados com sucesso via ESP-NOW");
        } else {
            Serial.println("Falha ao enviar dados via ESP-NOW");
        }
    }

private:
    uint8_t macAddress[6];

    static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
        Serial.print("Status do envio: ");
        Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");
    }
};

#endif // ESPNOW_TRANSMITTER_H

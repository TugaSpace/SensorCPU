#ifndef FRAM_BUFFER_H
#define FRAM_BUFFER_H

#include <Wire.h>

#define FRAM_I2C_ADDR 0x50   // Endereço I2C da FRAM MB85RC256V
#define FRAM_SIZE 32768      // Tamanho total da FRAM em bytes
#define POINTERS_SIZE 10     // Tamanho reservado para ponteiros e último pacote
#define STATE_SIZE 5         // Tamanho reservado para as variáveis de estado (pressão inicial e estado do voo)
#define INIT_FLAG_VALUE 0xA5 // Valor da flag de inicialização

template <typename T>
class FRAMBuffer {
public:
    FRAMBuffer() : writeAddress(0), readAddress(0), lastPacketNumber(0), isFull(false) {}

    // Inicializa a FRAM e carrega os ponteiros, número do último pacote
    bool begin() {
        Wire.begin();
        Wire.beginTransmission(FRAM_I2C_ADDR);
        if (Wire.endTransmission() != 0) {
            Serial.println("Erro ao inicializar a FRAM.");
            return false;
        }

        // Carrega a flag de inicialização
        Wire.beginTransmission(FRAM_I2C_ADDR);
        Wire.write(0); // Endereço inicial para a flag de inicialização
        Wire.endTransmission();
        Wire.requestFrom(FRAM_I2C_ADDR, 1);

        uint8_t initFlag = 0;
        if (Wire.available()) {
            initFlag = Wire.read();
        }

        // Verifica se a FRAM já foi inicializada
        if (initFlag == INIT_FLAG_VALUE) {
            loadPointers(); // Carrega os ponteiros da FRAM
        } else {
            // Inicializa a FRAM pela primeira vez
            writeAddress = 0;
            readAddress = 0;
            lastPacketNumber = 0;
            isFull = false;
            savePointers(); // Salva os ponteiros e a flag de inicialização
        }
        return true;
    }

    // Grava um pacote de dados na FRAM com comportamento FIFO e salva o estado dos ponteiros
    bool writeData(const T &data) {
        uint8_t dataBuffer[sizeof(T)];
        memcpy(dataBuffer, &data, sizeof(T));

        Wire.beginTransmission(FRAM_I2C_ADDR);
        Wire.write((writeAddress + POINTERS_SIZE + STATE_SIZE) >> 8); // Endereço alto ajustado
        Wire.write((writeAddress + POINTERS_SIZE + STATE_SIZE) & 0xFF); // Endereço baixo ajustado
        Wire.write(dataBuffer, sizeof(dataBuffer));

        if (Wire.endTransmission() != 0) {
            Serial.println("Erro ao gravar na FRAM.");
            return false;
        }

        // Atualiza o endereço de gravação com wrap-around
        writeAddress += sizeof(T);
        if (writeAddress >= FRAM_SIZE - POINTERS_SIZE - STATE_SIZE) {
            writeAddress = 0;
        }

        // FIFO: Se a FRAM estiver cheia, movemos o ponteiro de leitura
        if (isFull) {
            readAddress += sizeof(T);
            if (readAddress >= FRAM_SIZE - POINTERS_SIZE - STATE_SIZE) {
                readAddress = 0;
            }
        }

        // Atualiza o número do último pacote
        lastPacketNumber++;

        // Detecta quando a FRAM está cheia
        if (writeAddress == readAddress) {
            isFull = true;
        }

        savePointers(); // Salva os ponteiros e o número do último pacote após a gravação
        return true;
    }

    // Lê o pacote de dados mais antigo da FRAM com comportamento FIFO
    bool readData(T &data) {
        if (isEmpty()) {
            Serial.println("FRAM está vazia.");
            return false;
        }

        Wire.beginTransmission(FRAM_I2C_ADDR);
        Wire.write((readAddress + POINTERS_SIZE + STATE_SIZE) >> 8); // Endereço alto ajustado
        Wire.write((readAddress + POINTERS_SIZE + STATE_SIZE) & 0xFF); // Endereço baixo ajustado
        Wire.endTransmission();

        Wire.requestFrom(FRAM_I2C_ADDR, sizeof(T));

        if (Wire.available() == sizeof(T)) {
            uint8_t dataBuffer[sizeof(T)];
            for (int i = 0; i < sizeof(T); i++) {
                dataBuffer[i] = Wire.read();
            }
            memcpy(&data, dataBuffer, sizeof(T));

            // Atualiza o endereço de leitura com wrap-around
            readAddress += sizeof(T);
            if (readAddress >= FRAM_SIZE - POINTERS_SIZE - STATE_SIZE) {
                readAddress = 0;
            }

            // Após ler um registro, a FRAM não está mais cheia
            isFull = false;

            savePointers(); // Salva os ponteiros após a leitura
            return true;
        } else {
            Serial.println("Erro ao ler da FRAM.");
            return false;
        }
    }

    // Verifica se a FRAM está vazia
    bool isEmpty() const {
        return (!isFull && (writeAddress == readAddress));
    }

private:
    uint16_t writeAddress;
    uint16_t readAddress;
    uint32_t lastPacketNumber; // Número do último pacote gravado
    bool isFull;

    // Salva os ponteiros de leitura, gravação e número do último pacote na FRAM
    void savePointers() {
        Wire.beginTransmission(FRAM_I2C_ADDR);
        Wire.write(0); // Endereço inicial para salvar os ponteiros e a flag
        Wire.write(INIT_FLAG_VALUE); // Flag de inicialização
        Wire.write(writeAddress >> 8);
        Wire.write(writeAddress & 0xFF);
        Wire.write(readAddress >> 8);
        Wire.write(readAddress & 0xFF);
        Wire.write(isFull ? 1 : 0);
        Wire.write(lastPacketNumber >> 24);
        Wire.write((lastPacketNumber >> 16) & 0xFF);
        Wire.write((lastPacketNumber >> 8) & 0xFF);
        Wire.write(lastPacketNumber & 0xFF);
        Wire.endTransmission();
    }

    // Carrega os ponteiros de leitura, gravação e número do último pacote da FRAM
    void loadPointers() {
        Wire.beginTransmission(FRAM_I2C_ADDR);
        Wire.write(0); // Endereço inicial para carregar os ponteiros e a flag
        Wire.endTransmission();

        Wire.requestFrom(FRAM_I2C_ADDR, POINTERS_SIZE);
        if (Wire.available() == POINTERS_SIZE) {
            Wire.read(); // Leitura da flag de inicialização
            writeAddress = (Wire.read() << 8) | Wire.read();
            readAddress = (Wire.read() << 8) | Wire.read();
            isFull = Wire.read() == 1;
            lastPacketNumber = (Wire.read() << 24) | (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
        }
    }
};

#endif // FRAM_BUFFER_H

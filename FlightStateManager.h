#ifndef FLIGHT_STATE_MANAGER_H
#define FLIGHT_STATE_MANAGER_H

#include <Arduino.h>
#include <math.h>

enum FlightState {
    PRE_LAUNCH = 0,
    LAUNCH,
    APOGEE,
    PARACHUTE_DEPLOYMENT,
    TERMINAL_VELOCITY,
    LANDING
};

// Classe para gerenciar o estado do voo baseado nos dados do acelerômetro, BMP388, giroscópio e magnetômetro
class FlightStateManager {
public:
    FlightStateManager() : currentState(PRE_LAUNCH), maxAltitude(0.0), maxAcceleration(0.0), parachuteDeploymentTime(0) {}

    // Atualiza o estado do voo com base nos dados dos sensores
    void updateState(float accX, float accY, float accZ, float altitude, float gyrX, float gyrY, float gyrZ, unsigned long currentTime) {
        float totalAcceleration = sqrt(accX * accX + accY * accY + accZ * accZ);
        float totalRotation = sqrt(gyrX * gyrX + gyrY * gyrY + gyrZ * gyrZ);

        // Atualiza a aceleração máxima
        if (totalAcceleration > maxAcceleration) {
            maxAcceleration = totalAcceleration;
        }

        switch (currentState) {
            case PRE_LAUNCH:
                // No pré-lançamento, a aceleração total deve estar próxima de 1G (9,8 m/s²)
                if (fabs(totalAcceleration - 9.8) > ACCELERATION_THRESHOLD) {
                    currentState = LAUNCH;
                    Serial.println("Estado: Lançamento");
                }
                break;

            case LAUNCH:
                // Durante o lançamento, a aceleração deve ser significativamente maior que 1G
                if (altitude > maxAltitude) {
                    maxAltitude = altitude; // Atualiza a altitude máxima
                }
                if (totalAcceleration < 1.0) {
                    currentState = APOGEE;
                    Serial.println("Estado: Apogeu");
                }
                break;

            case APOGEE:
                // No apogeu, a aceleração deve ser muito próxima de zero e a altitude deve ser máxima
                if (totalAcceleration > 2.0 || altitude < maxAltitude - ALTITUDE_THRESHOLD) {
                    currentState = PARACHUTE_DEPLOYMENT;
                    parachuteDeploymentTime = currentTime;
                    Serial.println("Estado: Desdobramento do Paraquedas");
                }
                break;

            case PARACHUTE_DEPLOYMENT:
                // Durante o desdobramento do paraquedas, a aceleração deve aumentar e depois estabilizar
                if (fabs(totalAcceleration - 9.8) < ACCELERATION_THRESHOLD && totalRotation < ROTATION_THRESHOLD) {
                    currentState = TERMINAL_VELOCITY;
                    Serial.println("Estado: Velocidade Terminal");
                }
                break;

            case TERMINAL_VELOCITY:
                // Em velocidade terminal, a aceleração deve estar próxima de zero e a descida estabilizada
                if (totalAcceleration > 15.0) {
                    currentState = LANDING;
                    Serial.println("Estado: Pouso");
                }
                break;

            case LANDING:
                // No pouso, haverá uma aceleração brusca, seguida de estabilização
                // O estado final será mantido, sem mudança
                break;
        }
    }

    // Retorna o estado atual do voo
    FlightState getCurrentState() const {
        return currentState;
    }

    // Retorna a altitude máxima atingida
    float getMaxAltitude() const {
        return maxAltitude;
    }

    // Retorna a aceleração máxima registrada
    float getMaxAcceleration() const {
        return maxAcceleration;
    }

    // Retorna o tempo de desdobramento do paraquedas
    unsigned long getParachuteDeploymentTime() const {
        return parachuteDeploymentTime;
    }

private:
    FlightState currentState;
    float maxAltitude;               // Altitude máxima registrada durante o voo
    float maxAcceleration;           // Aceleração máxima registrada durante o voo
    unsigned long parachuteDeploymentTime; // Tempo de desdobramento do paraquedas

    static constexpr float ACCELERATION_THRESHOLD = 1.0; // Tolerância para detectar mudanças de estado (m/s²)
    static constexpr float ALTITUDE_THRESHOLD = 5.0;     // Tolerância para mudança de altitude significativa (metros)
    static constexpr float ROTATION_THRESHOLD = 1.0;     // Tolerância para detectar rotação estabilizada (rad/s)
};

#endif // FLIGHT_STATE_MANAGER_H

#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
//MCP2515 mcp2515(5); // CS no ESP32S
MCP2515 mcp2515(10); // Arduino NANO

void setup() {
  Serial.begin(115200);
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("Simulador de ECU OBD-II iniciado");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Verifica se é uma solicitação OBD-II padrão (ID 0x7DF)
    if (canMsg.can_id == 0x7DF && canMsg.can_dlc >= 3 && canMsg.data[1] == 0x01) {
      byte pid = canMsg.data[2];
      struct can_frame response;
      response.can_id  = 0x7E8; // ID de resposta padrão
      response.can_dlc = 8;
      response.data[1] = 0x41; // Resposta ao modo 0x01
      response.data[2] = pid;

      switch (pid) {
        case 0x00: { // PIDs suportados [0x01 - 0x28]
            response.data[0] = 0x06; // Número de bytes de dados a seguir
            response.data[1] = 0x41; // Resposta ao modo 0x01
            response.data[2] = 0x00; // PID 0x00
            response.data[3] = 0x1E; // PIDs x01-x08
            response.data[4] = 0x7B; // PIDs x09-x10
            response.data[5] = 0x98; // PIDs x11-x18
            response.data[6] = 0x01; // Ativa PID 0x20
            response.data[7] = 0x00;
            Serial.println("Respondido ao PID: 0x00 (PIDs 0x0C e 0x0D suportados)");
            break;
          }

        case 0x20: { // PIDs suportados [0x29 - 0x40]
          response.data[0] = 0x06; // Número de bytes de dados a seguir
          response.data[1] = 0x41; // Resposta ao modo 0x01
          response.data[2] = 0x20; // PID 0x20
          response.data[3] = 0x00;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x01; // Ativa PID 0x40
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x20");
          break;
        }

        case 0x40: { // PIDs suportados [0x41 - 0x60]
          response.data[0] = 0x06; // Número de bytes de dados a seguir
          response.data[1] = 0x41; // Resposta ao modo 0x01
          response.data[2] = 0x40; // PID 0x40
          response.data[3] = 0x40; // Ativa PID 0x42
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x40");
          break;
        }

        case 0x04: { // Calculated Engine Load -> A/2.55
            // Simulando 50% de carga do motor
            uint8_t raw = static_cast<uint8_t>(round(50.0 * 2.55)); // ≈128
            response.data[0] = 0x03;      // Número de bytes de dados a seguir (1 byte de A mais dois bytes de cabeçalho)
            response.data[3] = raw;       // Byte A
            response.data[4] = 0x00;
            response.data[5] = 0x00;
            response.data[6] = 0x00;
            response.data[7] = 0x00;
            Serial.println("Respondido ao PID: 0x04 (Calculated Engine Load)");
            break;
          }

        case 0x05: { // Temperatura do líquido de arrefecimento
          uint8_t temp = 90; // Temperatura simulada em °C
          response.data[0] = 0x03; // Número de bytes de dados a seguir
          response.data[3] = temp + 40; // Fórmula: A - 40 = temp
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x05 (Temperatura do líquido de arrefecimento)");
          break;
        }

        case 0x06: { // Short Term Fuel Trim (STFT) — Bank 1 -> (A/1.28 - 100)
          // Simulando 0% de ajuste de curto prazo
          uint8_t raw = static_cast<uint8_t>(round((0 + 100) * 1.28)); // =128
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x06 (STFT Bank 1)");
          break;
        }

        case 0x07: { // Long Term Fuel Trim (LTFT) — Bank 1 -> (A/1.28 - 100)
          // Simulando 0% de ajuste de longo prazo
          uint8_t raw = static_cast<uint8_t>(round((0 + 100) * 1.28)); // =128
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x07 (LTFT Bank 1)");
          break;
        }

        case 0x0A: { // Fuel Pressure (gauge) -> 3 × A
          // Simulando pressão de combustível de 102 kPa
          uint8_t raw = 34; // 34*3 = 102 kPa
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x0A (Fuel Pressure)");
          break;
        }

        case 0x0B: { // Intake Manifold Absolute Pressure -> A
          // Simulando pressão de 40 kPa
          uint8_t raw = 40;
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x0B (Intake Manifold Pressure)");
          break;
        }

        case 0x0C: { // RPM do motor
          uint16_t rpm = 3000; // Valor de RPM simulado
          uint16_t value = rpm * 4;
          response.data[0] = 0x04; // Número de bytes de dados a seguir
          response.data[3] = (value >> 8) & 0xFF;
          response.data[4] = value & 0xFF;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x0C (RPM)");
          break;
        }

        case 0x0D: { // Velocidade do veículo
          uint8_t speed = 60; // Velocidade simulada em km/h
          response.data[0] = 0x03; // Número de bytes de dados a seguir
          response.data[3] = speed;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x0D (Velocidade)");
          break;
        }

        case 0x0F: { // Intake Air Temperature -> A - 40
          // Simulando 25 °C de temperatura do ar de admissão
          uint8_t raw = 25 + 40; // =65
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x0F (Intake Air Temperature)");
          break;
        }

        case 0x10: { // Mass Air Flow (MAF) -> (256*A + B) / 100
          // Simulando fluxo de 10 g/s
          uint16_t maf_raw = 10 * 100;          // =1000
          uint8_t A = (maf_raw >> 8) & 0xFF;    // =3
          uint8_t B = maf_raw & 0xFF;           // =232
          response.data[0] = 0x04;              
          response.data[3] = A;
          response.data[4] = B;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x10 (Mass Air Flow)");
          break;
        }

        case 0x11: { // Throttle Position -> A/2.55
          // Simulando 25% de abertura
          uint8_t raw = static_cast<uint8_t>(round(25.0 * 2.55)); // =64
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x11 (Throttle Position)");
          break;
        }

        case 0x14: { // Oxygen Sensor 1: Voltage (A/200) & STFT (B/1.28 - 100)
          // Simulando 0.75 V e 0% de trim
          uint8_t A = static_cast<uint8_t>(round(0.75 * 200)); // =150
          uint8_t B = static_cast<uint8_t>(round((0 + 100) * 1.28)); // =128
          response.data[0] = 0x04;
          response.data[3] = A;  
          response.data[4] = B;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x14 (O2 Sensor 1)");
          break;
        }

        case 0x15: { // Oxygen Sensor 2: Voltage (A/200) & STFT (B/1.28 - 100)
          // Mesmo valor que o sensor 1
          uint8_t A = static_cast<uint8_t>(round(0.75 * 200)); // =150
          uint8_t B = static_cast<uint8_t>(round((0 + 100) * 1.28)); // =128
          response.data[0] = 0x04;
          response.data[3] = A;
          response.data[4] = B;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x15 (O2 Sensor 2)");
          break;
        }

        case 0x42: { // Control Module Voltage -> (256*A + B) / 1000
          // Simulando 12.6 V de tensão da ECU
          uint16_t volt_raw = static_cast<uint16_t>(round(12.6 * 1000)); // =12600
          uint8_t A = (volt_raw >> 8) & 0xFF;    // =49
          uint8_t B = volt_raw & 0xFF;           // =56
          response.data[0] = 0x04;
          response.data[3] = A;
          response.data[4] = B;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x42 (Control Module Voltage)");
          break;
        }


        default: {
          // PID não suportado: não responder
          Serial.print("PID não suportado: 0x");
          Serial.println(pid, HEX);
          return;
        }
      }

      mcp2515.sendMessage(&response);
    }
  }
}
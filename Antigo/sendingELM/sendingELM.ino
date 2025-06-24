  #include <SPI.h>
#include <mcp2515.h>

#define trigPin 5

struct can_frame canMsg;
MCP2515 mcp2515(10); // Arduino NANO

//Dados enviados
#define intervaloCapturaDado 250 
int dataIndex = 0;

float engLoad[10] = {50, 90, 23, 75, 13, 46, 88, 44, 35, 68}; // [%] Carga do motor - 0 a 100 
float EngCoolTemp[10] = {125, -20, 45, 13, 96, 32, 67, 103, 19, 75}; // [°C] Temperatura do líquido de arrefecimento - -40 a 215
float STFT[10] = {-10, 55, -80, 12, -40, 9, -15, 23, -9, 35}; // [%] Ajuste de curto prazo de Combustível - -100 a 99,2
float LTFT[10] = {-15, -22, 8, -13, 4, -9, 5, 60, 10, -4}; // [%] Ajuste de longo prazo de Combustível - -100 a 99,2
float FuelPressure[10] = {102, 235, 88, 345, 157, 298, 45, 188, 277, 66}; // [kPa] Pressão de combustível - 0 a 765
float IntManifAbsPres[10] = {40, 87, 33, 125, 57, 98, 12, 77, 145, 24}; // [kPa] Intake Manifold Absolute Pressure - 0 a 255
float rpm[10] = {3000, 1250, 8765, 3422, 987, 5433, 7654, 2346, 4322, 6543}; // [RPM] RPM Motor - 0 a 16.383,75
float VehiSpeed[10] = {60, 112, 46, 88, 32, 77, 124, 54, 99, 23}; // [km/h] Velocidade - 0 a 255
float IntAirTemp[10] = {25, -12, 38, 116, -32, -9, 20, 31, -4, 201}; // [°C] Temperatura do ar de admissão - -40 a 215
float MAF[10] = {10, 33, 88, 124, 46, 98, 156, 624, 277, 187}; // [g/s] Mass air flow - 0 a 655,35
float ThrPos[10] = {25, 12, 88, 34, 57, 9, 79, 24, 46, 67}; // [%] Throttle Position - 0 a 100
float VOS1[10] = {0.75, 0.23, 0.98, 0.45, 0.67, 0.12, 0.89, 0.34, 0.56, 0.78}; // [V] Voltage Oxygem Sensor 1 - 0 a 1,275
float STFT1[10] = {0, 60, -12, 82, -47, 10, -80, 3, -18, 41}; // [%] Short Term Fuel Trim Oxygem Sensor 1 - -100 a 99,2
float VOS2[10] = {0.75, 0.34, 0.87, 0.52, 0.91, 0.23, 0.68, 0.45, 0.79, 0.12}; // [V] Voltage Oxygem Sensor 2 - 0 a 1,275
float STFT2[10] = {0, -38, 13, -5, 58, -3, 7, -12, 44, -66}; // [%] Short Term Fuel Trim Oxygem Sensor 2 - -100 a 99,2
float CMV[10] = {12, 17, 22, 13, 40, 5, 33, 52, 38, 9}; // [V] Tensão da ECU - 0 a 65,535
bool trigger = false;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("Simulador de ECU OBD-II iniciado");

  pinMode(trigPin, INPUT);
  delay(5000); // Evitar interferências de nível lógico alto
}

void loop() {
  trigger = digitalRead(trigPin);

  Serial.print("DI: ");
  Serial.print(dataIndex);
  Serial.print(", Trig: ");
  Serial.println(trigger);

  if (trigger){
    if (dataIndex < 9){
      dataIndex++;
      delay(intervaloCapturaDado);
    } else{
      dataIndex = 0;
      delay(intervaloCapturaDado);
    }
  }


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
          break;
        }

        case 0x04: { // Calculated Engine Load -> A/2.55
            uint8_t raw = static_cast<uint8_t>(round(engLoad[dataIndex] * 2.55)); // ≈128
            response.data[0] = 0x03;      // Número de bytes de dados a seguir (1 byte de A mais dois bytes de cabeçalho)
            response.data[3] = raw;       // Byte A
            response.data[4] = 0x00;
            response.data[5] = 0x00;
            response.data[6] = 0x00;
            response.data[7] = 0x00;
            break;
        }

        case 0x05: { // Temperatura do líquido de arrefecimento
          response.data[0] = 0x03; // Número de bytes de dados a seguir
          response.data[3] = EngCoolTemp[dataIndex] + 40; // Fórmula: A - 40 = temp
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x06: { // Short Term Fuel Trim (STFT) — Bank 1 -> (A/1.28 - 100)
          uint8_t raw = static_cast<uint8_t>(round((STFT[dataIndex] + 100) * 1.28)); // =128
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x07: { // Long Term Fuel Trim (LTFT) — Bank 1 -> (A/1.28 - 100)
          uint8_t raw = static_cast<uint8_t>(round((LTFT[dataIndex] + 100) * 1.28)); // =128
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x0A: { // Fuel Pressure (gauge) -> 3 × A
          uint8_t raw = FuelPressure[dataIndex]/3;
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x0B: { // Intake Manifold Absolute Pressure -> A
          uint8_t raw = IntManifAbsPres[dataIndex];
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x0C: { // RPM do motor 3000
          uint16_t value = rpm[dataIndex] * 4;
          response.data[0] = 0x04; // Número de bytes de dados a seguir
          response.data[3] = (value >> 8) & 0xFF;
          response.data[4] = value & 0xFF;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x0D: { // Velocidade do veículo
          response.data[0] = 0x03; // Número de bytes de dados a seguir
          response.data[3] = VehiSpeed[dataIndex];
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x0F: { // Intake Air Temperature -> A - 40          
          uint8_t raw = IntAirTemp[dataIndex] + 40; // =65
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x10: { // Mass Air Flow (MAF) -> (256*A + B) / 100
          uint16_t maf_raw = MAF[dataIndex] * 100;          // =1000
          uint8_t A = (maf_raw >> 8) & 0xFF;    // =3
          uint8_t B = maf_raw & 0xFF;           // =232
          response.data[0] = 0x04;              
          response.data[3] = A;
          response.data[4] = B;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x11: { // Throttle Position -> A/2.55
          uint8_t raw = static_cast<uint8_t>(round(ThrPos[dataIndex] * 2.55)); // =64
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x14: { // Oxygen Sensor 1: Voltage (A/200) & STFT (B/1.28 - 100)
          uint8_t A = static_cast<uint8_t>(round(VOS1[dataIndex] * 200)); // =150
          uint8_t B = static_cast<uint8_t>(round((STFT1[dataIndex] + 100) * 1.28)); // =128
          response.data[0] = 0x04;
          response.data[3] = A;  
          response.data[4] = B;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x15: { // Oxygen Sensor 2: Voltage (A/200) & STFT (B/1.28 - 100)
          // Mesmo valor que o sensor 1
          uint8_t A = static_cast<uint8_t>(round(VOS2[dataIndex] * 200)); // =150
          uint8_t B = static_cast<uint8_t>(round((STFT2[dataIndex] + 100) * 1.28)); // =128
          response.data[0] = 0x04;
          response.data[3] = A;
          response.data[4] = B;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          break;
        }

        case 0x42: { // Control Module Voltage -> (256*A + B) / 1000
          uint16_t volt_raw = static_cast<uint16_t>(round(CMV[dataIndex] * 1000)); // =12600
          uint8_t A = (volt_raw >> 8) & 0xFF;    // =49
          uint8_t B = volt_raw & 0xFF;           // =56
          response.data[0] = 0x04;
          response.data[3] = A;
          response.data[4] = B;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
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
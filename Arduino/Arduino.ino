#include <SPI.h>
#include <mcp2515.h>
 
#define trigPin 5

struct can_frame canMsg;
MCP2515 mcp2515(10); // Arduino NANO

// Dados enviados
#define intervaloCapturaDado 250 
int dataIndex = 0;
bool manualMode = false;

// Expansão dos vetores para 11 posições, inicializando a posição 10 com 0
float engLoad[11]          = {50, 90, 23, 75, 13, 46, 88, 44, 35, 68, 0};  // [%] Carga do motor
float EngCoolTemp[11]      = {125, -10, 45, 13, 96, 32, 67, 103, 19, 75, 0}; // [°C] Temperatura do líquido de arrefecimento
float STFT[11]             = {-10, 55, -80, 12, -40, 9, -15, 23, -9, 35, 0}; // [%] Ajuste de curto prazo de combustível
float LTFT[11]             = {-15, -22, 8, -13, 4, -9, 5, 60, 10, -4, 0};  // [%] Ajuste de longo prazo de combustível
float FuelPressure[11]     = {102, 235, 88, 345, 157, 298, 45, 188, 277, 66, 0};  // [kPa] Pressão de combustível
float IntManifAbsPres[11]  = {40, 87, 33, 125, 57, 98, 12, 77, 145, 24, 0};    // [kPa] Pressão absoluta do coletor
float rpm[11]              = {3000, 1250, 8765, 3422, 987, 5433, 7654, 2346, 4322, 6543, 0}; // [RPM] RPM do motor
float VehiSpeed[11]        = {60, 112, 46, 88, 32, 77, 124, 54, 99, 23, 0};     // [km/h] Velocidade do veículo
float IntAirTemp[11]       = {25, -12, 38, 116, -32, -9, 20, 31, -4, 201, 0};   // [°C] Temperatura do ar de admissão
float MAF[11]              = {10, 33, 88, 124, 46, 98, 156, 624, 277, 187, 0};  // [g/s] Vazão de massa de ar
float ThrPos[11]           = {25, 12, 88, 34, 57, 9, 79, 24, 46, 67, 0};       // [%] Posição do acelerador
float VOS1[11]             = {0.75, 0.23, 0.98, 0.45, 0.67, 0.12, 0.89, 0.34, 0.56, 0.78, 0}; // [V] Tensão sensor oxigênio 1
float STFT1[11]            = {0, 60, -12, 82, -47, 10, -80, 3, -18, 41, 0};    // [%] Ajuste de curto prazo sensor 1
float VOS2[11]             = {0.75, 0.34, 0.87, 0.52, 0.91, 0.23, 0.68, 0.45, 0.79, 0.12, 0}; // [V] Tensão sensor oxigênio 2
float STFT2[11]            = {0, -38, 13, -5, 58, -3, 7, -12, 44, -66, 0};     // [%] Ajuste de curto prazo sensor 2
float CMV[11]              = {12, 17, 22, 13, 40, 5, 33, 52, 38, 9, 0};        // [V] Tensão da ECU

void setup() {
  Serial.begin(115200);
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("Simulador de ECU OBD-II iniciado");
  
  pinMode(trigPin, INPUT);
}

void loop() {
  Serial.println(dataIndex);
  // Verifica se há dados na Serial (rotina manual)
  if (Serial.available()) {
    String linha = Serial.readStringUntil('\n');
    int vals[7];
    int startIndex = 0;
    int endIndex = 0;
    // Parse dos 7 valores: triggerManual, engLoad, EngCoolTemp, FuelPressure, IntManifAbsPres, rpm, VehiSpeed
    for (int i = 0; i < 6; i++) {
      endIndex = linha.indexOf(',', startIndex);
      if (endIndex == -1) {
        // Caso não encontre mais vírgulas, pega o restante
        vals[i] = linha.substring(startIndex).toInt();
        break;
      } else {
        vals[i] = linha.substring(startIndex, endIndex).toInt();
        startIndex = endIndex + 1;
      }
    }
    // Último valor
    vals[6] = linha.substring(startIndex).toInt();

    int manualTrig = vals[0];
    if (manualTrig == 1) {
      // Ativa modo manual
      manualMode = true;
      dataIndex = 10; // 11ª posição dos vetores

      // Atualiza os 6 PIDs na posição 10
      engLoad[10]         = (float)vals[1];
      EngCoolTemp[10]     = (float)vals[2];
      FuelPressure[10]    = (float)vals[3];
      IntManifAbsPres[10] = (float)vals[4];
      rpm[10]             = (float)vals[5];
      VehiSpeed[10]       = (float)vals[6];
    } else {
      // Desativa modo manual
      manualMode = false;
      dataIndex = 0;
    }
  }

  // Rotina automática de incremento de dataIndex (somente se não estiver em modo manual)
  if (!manualMode) {
    bool trigger = digitalRead(trigPin);
    if (trigger) {
      if (dataIndex < 9) {
        dataIndex++;
        Serial.println(dataIndex);
      } else {
        dataIndex = 0;
      }
      delay(intervaloCapturaDado);
    }
  }

  // Ajuste de debug (opcional)
  // Serial.print("DI: ");
  // Serial.print(dataIndex);
  // Serial.print(", ModoManual: ");
  // Serial.println(manualMode);

  // Leitura de mensagens CAN e resposta a requisições OBD-II
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
          response.data[1] = 0x41;
          response.data[2] = 0x00;
          response.data[3] = 0x1E;
          response.data[4] = 0x7B;
          response.data[5] = 0x98;
          response.data[6] = 0x01;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x00 (PIDs 0x0C e 0x0D suportados)");
          break;
        }

        case 0x20: { // PIDs suportados [0x29 - 0x40]
          response.data[0] = 0x06;
          response.data[1] = 0x41;
          response.data[2] = 0x20;
          response.data[3] = 0x00;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x01;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x20");
          break;
        }

        case 0x40: { // PIDs suportados [0x41 - 0x60]
          response.data[0] = 0x06;
          response.data[1] = 0x41;
          response.data[2] = 0x40;
          response.data[3] = 0x40;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x40");
          break;
        }

        case 0x04: { // Calculated Engine Load -> A/2.55
          uint8_t raw = static_cast<uint8_t>(round(engLoad[dataIndex] * 2.55));
          response.data[0] = 0x03;
          response.data[3] = raw;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x04 (Calculated Engine Load)");
          break;
        }

        case 0x05: { // Temperatura do líquido de arrefecimento
          response.data[0] = 0x03;
          response.data[3] = EngCoolTemp[dataIndex] + 40;
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x05 (Temperatura do líquido de arrefecimento)");
          break;
        }

        case 0x06: { // Short Term Fuel Trim (STFT) — Bank 1 -> (A/1.28 - 100)
          uint8_t raw = static_cast<uint8_t>(round((STFT[dataIndex] + 100) * 1.28));
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
          uint8_t raw = static_cast<uint8_t>(round((LTFT[dataIndex] + 100) * 1.28));
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
          uint8_t raw = FuelPressure[dataIndex] / 3;
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
          uint8_t raw = IntManifAbsPres[dataIndex];
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
          uint16_t value = rpm[dataIndex] * 4;
          response.data[0] = 0x04;
          response.data[3] = (value >> 8) & 0xFF;
          response.data[4] = value & 0xFF;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x0C (RPM)");
          break;
        }

        case 0x0D: { // Velocidade do veículo
          response.data[0] = 0x03;
          response.data[3] = VehiSpeed[dataIndex];
          response.data[4] = 0x00;
          response.data[5] = 0x00;
          response.data[6] = 0x00;
          response.data[7] = 0x00;
          Serial.println("Respondido ao PID: 0x0D (Velocidade)");
          break;
        }

        case 0x0F: { // Intake Air Temperature -> A - 40          
          uint8_t raw = IntAirTemp[dataIndex] + 40;
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
          uint16_t maf_raw = MAF[dataIndex] * 100;
          uint8_t A = (maf_raw >> 8) & 0xFF;
          uint8_t B = maf_raw & 0xFF;
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
          uint8_t raw = static_cast<uint8_t>(round(ThrPos[dataIndex] * 2.55));
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
          uint8_t A = static_cast<uint8_t>(round(VOS1[dataIndex] * 200));
          uint8_t B = static_cast<uint8_t>(round((STFT1[dataIndex] + 100) * 1.28));
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
          uint8_t A = static_cast<uint8_t>(round(VOS2[dataIndex] * 200));
          uint8_t B = static_cast<uint8_t>(round((STFT2[dataIndex] + 100) * 1.28));
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
          uint16_t volt_raw = static_cast<uint16_t>(round(CMV[dataIndex] * 1000));
          uint8_t A = (volt_raw >> 8) & 0xFF;
          uint8_t B = volt_raw & 0xFF;
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

// Código do arduino aparentemente ok, rotina de testes automática funcionando. Falta implementar a rotina manual no ESP pra ver se ele vai enviar certo os dados.

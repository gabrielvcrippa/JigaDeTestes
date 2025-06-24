#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
// Pinos de ADC para leitura dos sinais manuais
const int pin1 = 34; // Carga do motor – PID 04 (0 a 100%)
const int pin2 = 35; // Temp. líquido de arrefecimento – PID 05 (-40 a 215 °C)
const int pin3 = 32; // Pressão do combustível – PID 0A (0 a 765 kPa)
const int pin4 = 33; // Pressão Abs. Coletor – PID 0B (0 a 255 kPa)
const int pin5 = 25; // Rotação do motor – PID 0C (0 a 16 383 RPM)
const int pin6 = 26; // Velocidade – PID 0D (0 a 255 km/h)

const int Tx = 17; // UART2 TX para enviar dados ao Arduino

#define trigPin 5
#define BT_DISCOVER_TIME 10000  // 10 segundos de escaneamento

const char* elmPIN    = "1234";
size_t      elmPINlen = strlen(elmPIN);

// Dados enviados
#define intervaloCapturaDado 250
int dataIndex = 0;
int dataError = 0;
int totalError = 0;

// Arrays estendidos para 11 posições
float engLoad[11]          = {50, 90, 23, 75, 13, 46, 88, 44, 35, 68, 0};  
float EngCoolTemp[11]      = {125, -20, 45, 13, 96, 32, 67, 103, 19, 75, 0};
float STFT[11]             = {-10, 55, -80, 12, -40, 9, -15, 23, -9, 35, 0};
float LTFT[11]             = {-15, -22, 8, -13, 4, -9, 5, 60, 10, -4, 0};
float FuelPressure[11]     = {102, 235, 88, 345, 157, 298, 45, 188, 277, 66, 0};
float IntManifAbsPres[11]  = {40, 87, 33, 125, 57, 98, 12, 77, 145, 24, 0};
float rpm[11]              = {3000, 1250, 8765, 3422, 987, 5433, 7654, 2346, 4322, 6543, 0};
float VehiSpeed[11]        = {60, 112, 46, 88, 32, 77, 124, 54, 99, 23, 0};
float IntAirTemp[11]       = {25, -12, 38, 116, -32, -9, 20, 31, -4, 201, 0};
float MAF[11]              = {10, 33, 88, 124, 46, 98, 156, 624, 277, 187, 0};
float ThrPos[11]           = {25, 12, 88, 34, 57, 9, 79, 24, 46, 67, 0};
float VOS1[11]             = {0.75, 0.23, 0.98, 0.45, 0.67, 0.12, 0.89, 0.34, 0.56, 0.78, 0};
float STFT1[11]            = {0, 60, -12, 82, -47, 10, -80, 3, -18, 41, 0};
float VOS2[11]             = {0.75, 0.34, 0.87, 0.52, 0.91, 0.23, 0.68, 0.45, 0.79, 0.12, 0};
float STFT2[11]            = {0, -38, 13, -5, 58, -3, 7, -12, 44, -66, 0};
float CMV[11]              = {12, 17, 22, 13, 40, 5, 33, 52, 38, 9, 0};

float erro[16] = {0};

bool menu = false;
bool ativaTrigger = false;
bool rotinaAutomatica = false;
bool BTconectado = false;
bool primeiraExecucao = true;

struct DadosRecebidos {
  float conv04;
  float conv05;
  float conv06;
  float conv07;
  float conv0A;
  float conv0B;
  float conv0C;
  float conv0D;
  float conv0F;
  float conv10;
  float conv11;
  float conv14a;
  float conv14b;
  float conv15a;
  float conv15b;
  float conv42;
};

void setup() {
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);

  Serial.begin(115200);
  delay(500);

  // Inicializa UART2 (TX no GPIO17) para enviar ao Arduino
  Serial2.begin(115200, SERIAL_8N1, -1, Tx);

  // Configura ADC (resolução e atenuação)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Bluetooth SPP
  if (!SerialBT.begin("ESP32_OBD", true)) {
    Serial.println("❌ Falha ao iniciar Bluetooth SPP");
    while (1) delay(1000);
  }
  SerialBT.setPin(elmPIN, elmPINlen);
  Serial.println("✅ Bluetooth iniciado");
  Serial.println("Envie 'b' para buscar dispositivos Bluetooth:");
}

void loop() {
  if (primeiraExecucao) {
    primeiraExecucao = false;
    esperaComandoInicial();  // Travado até receber o primeiro 'b'
    conectaBTelm();          // Tenta conectar pela primeira vez
    BTconectado = true;
  }

  if (BTconectado) {
    Serial.println("Selecionar modo:\n 1 - Teste Automático \n 2 - Teste Manual \n b - Reconectar Bluetooth");

    digitalWrite(trigPin, LOW);
    menu = false;
    dataError = 0;
    totalError = 0;

    while (!menu) {
      if (Serial.available()) {
        int read = Serial.read();
        if (read == '1') {
          Serial.println("Iniciando teste automático");
          initLeitura(); // Necessário para não haver erro inicial de leitura
          ativaTrigger = true;
          rotinaAutomatica = true;
          for (int i = 0; i < 10; ++i) {
            DadosRecebidos dados = recebeDados(ativaTrigger, rotinaAutomatica);
            calculaErro(dados);
          }
          if (totalError > 0) {
            Serial.printf("Existem %d erros\n", totalError);
          } else {
            Serial.println("Teste automático bem sucedido!\n");
          }
          menu = true;
          ativaTrigger = false;
        } 
        else if (read == '2') {
          // --- TESTE MANUAL ---
          Serial.println("===== Iniciando teste manual ======");
          delay(1000);
          ativaTrigger = false;
          rotinaAutomatica = false;
          // Informa ao Arduino para entrar em modo manual: trigger = 1
          // Os valores reais virão logo após.
          while (true) {
            // Verifica se usuário digitou '0' para sair do teste manual
            if (Serial.available() > 0) {
              char c = Serial.read();
              if (c == '0') {
                // Envia trigger=0 e zeros para parar o modo manual no Arduino
                Serial2.print("0,0,0,0,0,0,0\n");
                break;
              }
            }

            // 1) Lê ADCs e faz o mapeamento
            int pin1Bruto = analogRead(pin1);
            int pin2Bruto = analogRead(pin2);
            int pin3Bruto = analogRead(pin3);
            int pin4Bruto = analogRead(pin4);
            int pin5Bruto = analogRead(pin5);
            int pin6Bruto = analogRead(pin6);

            int val1 = map(pin1Bruto, 0, 4095, 0, 100);      // engLoad
            int val2 = map(pin2Bruto, 0, 4095, -40, 215);   // EngCoolTemp
            int val3 = map(pin3Bruto, 0, 4095, 0, 765);     // FuelPressure
            int val4 = map(pin4Bruto, 0, 4095, 0, 255);     // IntManifAbsPres
            int val5 = map(pin5Bruto, 0, 4095, 0, 16383);   // rpm
            int val6 = map(pin6Bruto, 0, 4095, 0, 255);     // VehiSpeed

            // // 2) Envia pela Serial2: "1,val1,val2,val3,val4,val5,val6\n"
            // Serial2.print("1,");
            // Serial2.print(val1); Serial2.print(",");
            // Serial2.print(val2); Serial2.print(",");
            // Serial2.print(val3); Serial2.print(",");
            // Serial2.print(val4); Serial2.print(",");
            // Serial2.print(val5); Serial2.print(",");
            // Serial2.print(val6); Serial2.print("\n");

            // 3) Lê do ELM327 apenas os 6 PIDs que serão alterados e dá print
            DadosRecebidos dados = recebeDados(ativaTrigger, rotinaAutomatica); 
          }

          menu = true;
        } else if (read == 'b'){
            conectaBTelm();
            loop();
        }
      }
    }
  }
}

// ------------------- FUNÇÕES AUXILIARES -------------------

void esperaComandoInicial() {
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'b') {
        break;
      }
    }
  }
}

void checaComandoSerial() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      BTconectado = false;
      Serial.println("Reiniciando busca de dispositivos Bluetooth...");
      conectaBTelm(); // Sua função de conexão
    }
  }
}

void conectaBTelm() {
  Serial.printf("🔍 Escaneando dispositivos...\n");
  BTScanResults* results = SerialBT.discover(BT_DISCOVER_TIME);
  if (!results) {
    Serial.println("❌ Erro no escaneamento Bluetooth");
    conectaBTelm();
    return;
  }

  int count = results->getCount();
  Serial.printf("Dispositivos encontrados: %d\n", count);

  // Primeiro mostra os ELM327 (MAC começando com 66)
  Serial.println("----- ELM327 (MAC 66:) -----");
  bool foundELM = false;
  for (int i = 0; i < count; ++i) {
    BTAdvertisedDevice* dev = results->getDevice(i);
    String mac = dev->getAddress().toString();
    if (mac.startsWith("66:")) {
      Serial.printf(" [%d] %s  [%s] (ELM327)\n",
        i,
        dev->haveName() ? dev->getName().c_str() : "<sem nome>",
        mac.c_str()
      );
      foundELM = true;
    }
  }

  // Depois mostra outros dispositivos
  if (foundELM) {
    Serial.println("\n----- Outros dispositivos -----");
  }
  for (int i = 0; i < count; ++i) {
    BTAdvertisedDevice* dev = results->getDevice(i);
    String mac = dev->getAddress().toString();
    if (!mac.startsWith("66:")) {
      Serial.printf(" [%d] %s  [%s]\n",
        i,
        dev->haveName() ? dev->getName().c_str() : "<sem nome>",
        mac.c_str()
      );
    }
  }

  Serial.println("\nDigite o índice do ELM327 para conectar:");
  while (Serial.available()) Serial.read();
  while (!Serial.available()) { delay(10); }
  int sel = Serial.parseInt();
  if (sel < 0 || sel >= count) {
    Serial.println("Seleção inválida.");
    conectaBTelm();
    return;
  }

  BTAdvertisedDevice* chosen = results->getDevice(sel);
  Serial.printf("Conectando a %s [%s] …\n",
    chosen->haveName() ? chosen->getName().c_str() : "<sem nome>",
    chosen->getAddress().toString().c_str()
  );

  if (!SerialBT.connect(chosen->getAddress())) {
    Serial.println("❌ Falha ao conectar");
    conectaBTelm();
    return;
  }

  Serial.println("✅ Conectado ao ELM327!");
  BTconectado = true;
  initializeELM();
}

void initializeELM() {
  const char* cmds[] = {
    "ATZ", "ATE0", "ATS0", "ATL0", "ATH0", "ATSP0", "ATAT2", "ATST10"
  };
  for (auto &c : cmds) {
    Serial.printf("AT→ %s\n", c);
    SerialBT.print(c); SerialBT.print("\r");
    delay(300);
    String r = SerialBT.readStringUntil('>');
    r.trim();
    Serial.printf("  ← %s\n", r.c_str());
    delay(150);
  }
}

String readELMResponse() {
  String response = "";
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    while (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '>') {
        response.trim();
        return response;
      }
      response += c;
    }
  }
  response.trim();
  return response.length() ? response : "NO DATA";
}

void initLeitura() {
  /* 
    A primeira leitura do ELM327 nunca retorna valores corretos, esta função
    faz uma leitura inicial de um PID arbitrário para que a inicialização do
    ELM não interfira no resultado do teste automatizado
  */ 
  while (SerialBT.available()) SerialBT.read();
  SerialBT.print("01 0D\r");
  readELMResponse();
  delay(1000);
}

DadosRecebidos recebeDados(bool ativaTrigger, bool rotinaAutomatica) {
   /* 
    Recebimento e conversão dos PIDs. No teste automatizado, um trigger 
    sincroniza a varredura dos arrays do ESP32 e Arduino NANO, de modo 
    que o ESP saiba qual dado será enviado para que o erro seja calculado
  */ 
  DadosRecebidos dados;
  if (ativaTrigger) { digitalWrite(trigPin, LOW); }
  while (SerialBT.available()) SerialBT.read();

  if (rotinaAutomatica){
    // PID 06
    SerialBT.print("01 06\r");
    String resp06 = readELMResponse();
    dados.conv06 = (strtoul(resp06.substring(4).c_str(), NULL, 16) * 100.0 / 128.0) - 100;
    Serial.printf("Short FT: %.1f %%\n", dados.conv06);

    // PID 07
    SerialBT.print("01 07\r");
    String resp07 = readELMResponse();
    dados.conv07 = (strtoul(resp07.substring(4).c_str(), NULL, 16) * 100.0 / 128.0) - 100;
    Serial.printf("Long FT: %.1f %%\n", dados.conv07);

    // PID 0F
    SerialBT.print("01 0F\r");
    String resp0F = readELMResponse();
    {
      int16_t raw = strtol(resp0F.substring(4).c_str(), NULL, 16);
      dados.conv0F = raw - 40;
    }
    Serial.printf("IAT: %.1f °C\n", dados.conv0F);

    // PID 10
    SerialBT.print("01 10\r");
    String resp10 = readELMResponse();
    {
      uint16_t val = strtoul(resp10.substring(4).c_str(), NULL, 16);
      dados.conv10 = val / 100.0;
    }
    Serial.printf("MAF: %.2f g/s\n", dados.conv10);

    // PID 11
    SerialBT.print("01 11\r");
    String resp11 = readELMResponse();
    dados.conv11 = strtoul(resp11.substring(4).c_str(), NULL, 16) * 100.0 / 255.0;
    Serial.printf("Throttle: %.1f %%\n", dados.conv11);

    // PID 14
    SerialBT.print("01 14\r");
    String resp14 = readELMResponse();
    {
      uint16_t o2 = strtoul(resp14.substring(4).c_str(), NULL, 16);
      dados.conv14a = (o2 >> 8) * 0.005;
      dados.conv14b = ((o2 & 0xFF) * 100.0 / 128.0) - 100;
    }
    Serial.printf("O2 Sensor 1: %.3f V, %.1f %%\n", dados.conv14a, dados.conv14b);

    // PID 15
    SerialBT.print("01 15\r");
    String resp15 = readELMResponse();
    {
      uint16_t o2 = strtoul(resp15.substring(4).c_str(), NULL, 16);
      dados.conv15a = (o2 >> 8) * 0.005;
      dados.conv15b = ((o2 & 0xFF) * 100.0 / 128.0) - 100;
    }
    Serial.printf("O2 Sensor 2: %.3f V, %.1f %%\n", dados.conv15a, dados.conv15b);

    // PID 42
    SerialBT.print("01 42\r");
    String resp42 = readELMResponse();
    dados.conv42 = strtoul(resp42.substring(4).c_str(), NULL, 16) / 1000.0;
    Serial.printf("ECU Voltage: %.2f V\n", dados.conv42);
  }

  SerialBT.print("01 04\r");
  String resp04 = readELMResponse();
  dados.conv04 = strtoul(resp04.substring(4).c_str(), NULL, 16) * 100.0 / 255.0;
  Serial.printf("Recebido PID 04 (Carga do Motor): %.1f %%\n", dados.conv04);

  // PID 05 – EngCoolTemp
  SerialBT.print("01 05\r");
  String resp05 = readELMResponse();
  dados.conv05 = strtoul(resp05.substring(4).c_str(), NULL, 16) - 40.0;
  Serial.printf("Recebido PID 05 (Temp. Líq. Arref.): %.1f °C\n", dados.conv05);

  // PID 0A – FuelPressure
  SerialBT.print("01 0A\r");
  String resp0A = readELMResponse();
  dados.conv0A = strtoul(resp0A.substring(4).c_str(), NULL, 16) * 3.0;
  Serial.printf("Recebido PID 0A (Pressão Comb.): %.1f kPa\n", dados.conv0A);

  // PID 0B – IntManifAbsPres
  SerialBT.print("01 0B\r");
  String resp0B = readELMResponse();
  dados.conv0B = strtoul(resp0B.substring(4).c_str(), NULL, 16);
  Serial.printf("Recebido PID 0B (Pressão Coletor): %.1f kPa\n", dados.conv0B);

  // PID 0C – rpm
  SerialBT.print("01 0C\r");
  String resp0C = readELMResponse();
  {
    uint16_t raw0C = strtoul(resp0C.substring(4).c_str(), NULL, 16);
    dados.conv0C = raw0C / 4.0;
    Serial.printf("Recebido PID 0C (RPM): %.0f RPM\n", dados.conv0C);
  }

  // PID 0D – VehiSpeed
  SerialBT.print("01 0D\r");
  String resp0D = readELMResponse();
  dados.conv0D = strtoul(resp0D.substring(4).c_str(), NULL, 16);
  Serial.printf("Recebido PID 0D (Velocidade): %.0f km/h\n", dados.conv0D);

  Serial.println("-----------------------------");


  if (ativaTrigger) { digitalWrite(trigPin, HIGH); }
  delay(intervaloCapturaDado);
  return dados;
}

float calculaErro(const DadosRecebidos& d) {
  dataError = 0;
  erro[0]  = abs((d.conv04  - engLoad[dataIndex])        / engLoad[dataIndex]);
  erro[1]  = abs((d.conv05  - EngCoolTemp[dataIndex])    / EngCoolTemp[dataIndex]);
  erro[2]  = abs((d.conv06  - STFT[dataIndex])           / STFT[dataIndex]);
  erro[3]  = abs((d.conv07  - LTFT[dataIndex])           / LTFT[dataIndex]);
  erro[4]  = abs((d.conv0A  - FuelPressure[dataIndex])   / FuelPressure[dataIndex]);
  erro[5]  = abs((d.conv0B  - IntManifAbsPres[dataIndex]) / IntManifAbsPres[dataIndex]);
  erro[6]  = abs((d.conv0C  - rpm[dataIndex])            / rpm[dataIndex]);
  erro[7]  = abs((d.conv0D  - VehiSpeed[dataIndex])      / VehiSpeed[dataIndex]);
  erro[8]  = abs((d.conv0F  - IntAirTemp[dataIndex])     / IntAirTemp[dataIndex]);
  erro[9]  = abs((d.conv10  - MAF[dataIndex])            / MAF[dataIndex]);
  erro[10] = abs((d.conv11  - ThrPos[dataIndex])         / ThrPos[dataIndex]);
  erro[11] = abs((d.conv14a - VOS1[dataIndex])           / VOS1[dataIndex]);
  erro[12] = abs((d.conv14b - STFT1[dataIndex])          / STFT1[dataIndex]);
  erro[13] = abs((d.conv15a - VOS2[dataIndex])           / VOS2[dataIndex]);
  erro[14] = abs((d.conv15b - STFT2[dataIndex])          / STFT2[dataIndex]);
  erro[15] = abs((d.conv42  - CMV[dataIndex])            / CMV[dataIndex]);

  for (int i = 0; i < 16; ++i) {
    if (erro[i] > 0.1) {
      dataError++;
      totalError++;
    }
  }

  Serial.printf("Erros: %d\n\n=====================================\n", dataError);

  // Atualiza índice circular
  if (++dataIndex >= 10) dataIndex = 0;
  return totalError;
}

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define trigPin 5
#define BT_DISCOVER_TIME 10000  // 10 segundos de escaneamento

const char* elmPIN    = "1234";
size_t      elmPINlen = strlen(elmPIN);

//Dados enviados
int dataIndex = 0;
int dataError = 0;
int totalError = 0;

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


float erro[16] = {0};

bool menu = false;
bool ativaTrigger = false;

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

  if (!SerialBT.begin("ESP32_OBD", true)) {
    Serial.println("❌ Falha ao iniciar Bluetooth SPP");
    while (1) delay(1000);
  }
  SerialBT.setPin(elmPIN, elmPINlen);
  Serial.println("✅ Bluetooth SPP iniciado como mestre");

  Serial.printf("🔍 Escaneando dispositivos (%d ms)...\n", BT_DISCOVER_TIME);
  BTScanResults* results = SerialBT.discover(BT_DISCOVER_TIME);
  if (!results) {
    Serial.println("❌ Erro no escaneamento Bluetooth");
    while (1) delay(1000);
  }

  int count = results->getCount();
  Serial.printf("Dispositivos encontrados: %d\n", count);
  for (int i = 0; i < count; ++i) {
    BTAdvertisedDevice* dev = results->getDevice(i);
    Serial.printf(" [%d] %s  [%s]\n",
      i,
      dev->haveName() ? dev->getName().c_str() : "<sem nome>",
      dev->getAddress().toString().c_str()
    );
  }

  Serial.println("Digite o índice do ELM327 para conectar:");
  while (!Serial.available()) { delay(10); }
  int sel = Serial.parseInt();
  if (sel < 0 || sel >= count) {
    Serial.println("Seleção inválida. Reinicie.");
    while (1) delay(1000);
  }
  BTAdvertisedDevice* chosen = results->getDevice(sel);
  Serial.printf("Conectando a %s [%s] …\n",
    chosen->haveName() ? chosen->getName().c_str() : "<sem nome>",
    chosen->getAddress().toString().c_str()
  );

  if (!SerialBT.connect(chosen->getAddress())) {
    Serial.println("❌ Falha ao conectar");
    while (1) delay(1000);
  }
  Serial.println("✅ Conectado ao ELM327!");
  initializeELM();
}

void loop() {
  Serial.println("Selecionar modo:\n 1 - Teste Automático \n 2 - Teste Manual");

  digitalWrite(trigPin, LOW);
  menu = false;
  dataError = 0;
  totalError = 0;

  while (!menu) {
    if (Serial.available()) {
      if (Serial.read() == '1') {
        Serial.println("Iniciando teste automático");
        initLeitura(); // Necessário para não haver erro inicial de leitura
        ativaTrigger = true;
        for (int i = 0; i < 10; ++i) {
          DadosRecebidos dados = recebeDados(ativaTrigger);
          calculaErro(dados);
        }
        if (totalError > 0){
          Serial.printf("Existem %d erros\n", totalError);
        } else {
          Serial.println("Teste automático bem sucedido!\n");
        }
        menu = true;
        ativaTrigger = false;
      }
      if (Serial.read() == '2'){ // Não entra nesse if, trocar por case?
        Serial.println("Iniciando teste manual");
        ativaTrigger = false;
        for (int i = 0; i < 10; ++i) {
          DadosRecebidos dados = recebeDados(ativaTrigger);
        }
        menu = true;
      }
    }
  }
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

void initLeitura(){
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

DadosRecebidos recebeDados(bool ativaTrigger) {
   /* 
    Recebimento e conversão dos PIDs. No teste automatizado, um trigger 
    sincroniza a varredura dos arrays do ESP32 e Arduino NANO, de modo 
    que o ESP saiba qual dado será enviado para que o erro seja calculado
  */ 
  DadosRecebidos dados;
  if (ativaTrigger){digitalWrite(trigPin, LOW);}
  while (SerialBT.available()) SerialBT.read();

  // PID 04
  SerialBT.print("01 04\r");
  String resp = readELMResponse();
  dados.conv04 = strtoul(resp.substring(4).c_str(), NULL, 16) * 100.0 / 255.0;
  Serial.printf("Carga do Motor: %.1f %%\n", dados.conv04);

  // PID 05
  SerialBT.print("01 05\r");
  resp = readELMResponse();
  dados.conv05 = strtoul(resp.substring(4).c_str(), NULL, 16) - 40.0;
  Serial.printf("Temperatura: %.1f °C\n", dados.conv05);

  // PID 06
  SerialBT.print("01 06\r");
  resp = readELMResponse();
  dados.conv06 = (strtoul(resp.substring(4).c_str(), NULL, 16) * 100.0 / 128.0) - 100;
  Serial.printf("Short FT: %.1f %%\n", dados.conv06);

  // PID 07
  SerialBT.print("01 07\r");
  resp = readELMResponse();
  dados.conv07 = (strtoul(resp.substring(4).c_str(), NULL, 16) * 100.0 / 128.0) - 100;
  Serial.printf("Long FT: %.1f %%\n", dados.conv07);

  // PID 0A
  SerialBT.print("01 0A\r");
  resp = readELMResponse();
  dados.conv0A = strtoul(resp.substring(4).c_str(), NULL, 16) * 3.0;
  Serial.printf("Fuel Pressure: %.1f kPa\n", dados.conv0A);

  // PID 0B
  SerialBT.print("01 0B\r");
  resp = readELMResponse();
  dados.conv0B = strtoul(resp.substring(4).c_str(), NULL, 16);
  Serial.printf("Pressão Coletor: %.1f kPa\n", dados.conv0B);

  // PID 0C
  SerialBT.print("01 0C\r");
  resp = readELMResponse();
  {
    uint16_t val = strtoul(resp.substring(4).c_str(), NULL, 16);
    dados.conv0C = val / 4.0;
  }
  Serial.printf("RPM: %.0f\n", dados.conv0C);

  // PID 0D
  SerialBT.print("01 0D\r");
  resp = readELMResponse();
  dados.conv0D = strtoul(resp.substring(4).c_str(), NULL, 16);
  Serial.printf("Velocidade: %.0f km/h\n", dados.conv0D);

  // PID 0F
  SerialBT.print("01 0F\r");
  resp = readELMResponse();
  {
    int16_t raw = strtol(resp.substring(4).c_str(), NULL, 16);
    dados.conv0F = raw - 40;
  }
  Serial.printf("IAT: %.1f °C\n", dados.conv0F);

  // PID 10
  SerialBT.print("01 10\r");
  resp = readELMResponse();
  {
    uint16_t val = strtoul(resp.substring(4).c_str(), NULL, 16);
    dados.conv10 = val / 100.0;
  }
  Serial.printf("MAF: %.2f g/s\n", dados.conv10);

  // PID 11
  SerialBT.print("01 11\r");
  resp = readELMResponse();
  dados.conv11 = strtoul(resp.substring(4).c_str(), NULL, 16) * 100.0 / 255.0;
  Serial.printf("Throttle: %.1f %%\n", dados.conv11);

  // PID 14
  SerialBT.print("01 14\r");
  resp = readELMResponse();
  {
    uint16_t o2 = strtoul(resp.substring(4).c_str(), NULL, 16);
    dados.conv14a = (o2 >> 8) * 0.005;
    dados.conv14b = ((o2 & 0xFF) * 100.0 / 128.0) - 100;
  }
  Serial.printf("O2 Sensor 1: %.3f V, %.1f %%\n", dados.conv14a, dados.conv14b);

  // PID 15
  SerialBT.print("01 15\r");
  resp = readELMResponse();
  {
    uint16_t o2 = strtoul(resp.substring(4).c_str(), NULL, 16);
    dados.conv15a = (o2 >> 8) * 0.005;
    dados.conv15b = ((o2 & 0xFF) * 100.0 / 128.0) - 100;
  }
  Serial.printf("O2 Sensor 2: %.3f V, %.1f %%\n", dados.conv15a, dados.conv15b);

  // PID 42
  SerialBT.print("01 42\r");
  resp = readELMResponse();
  dados.conv42 = strtoul(resp.substring(4).c_str(), NULL, 16) / 1000.0;
  Serial.printf("ECU Voltage: %.2f V\n", dados.conv42);

  if (ativaTrigger){digitalWrite(trigPin, HIGH);}
  delay(1000);
  return dados;
}

float calculaErro(const DadosRecebidos& d) {
  dataError = 0;
  erro[0]  = abs((d.conv04  - engLoad[dataIndex])       / engLoad[dataIndex]);
  erro[1]  = abs((d.conv05  - EngCoolTemp[dataIndex])   / EngCoolTemp[dataIndex]);
  erro[2]  = abs((d.conv06  - STFT[dataIndex])          / STFT[dataIndex]);
  erro[3]  = abs((d.conv07  - LTFT[dataIndex])          / LTFT[dataIndex]);
  erro[4]  = abs((d.conv0A  - FuelPressure[dataIndex])  / FuelPressure[dataIndex]);
  erro[5]  = abs((d.conv0B  - IntManifAbsPres[dataIndex]) / IntManifAbsPres[dataIndex]);
  erro[6]  = abs((d.conv0C  - rpm[dataIndex])           / rpm[dataIndex]);
  erro[7]  = abs((d.conv0D  - VehiSpeed[dataIndex])     / VehiSpeed[dataIndex]);
  erro[8]  = abs((d.conv0F  - IntAirTemp[dataIndex])    / IntAirTemp[dataIndex]);
  erro[9]  = abs((d.conv10  - MAF[dataIndex])           / MAF[dataIndex]);
  erro[10] = abs((d.conv11 - ThrPos[dataIndex])         / ThrPos[dataIndex]);
  erro[11] = abs((d.conv14a - VOS1[dataIndex])          / VOS1[dataIndex]);
  erro[12] = abs((d.conv14b - STFT1[dataIndex])         / STFT1[dataIndex]);
  erro[13] = abs((d.conv15a - VOS2[dataIndex])          / VOS2[dataIndex]);
  erro[14] = abs((d.conv15b - STFT2[dataIndex])         / STFT2[dataIndex]);
  erro[15] = abs((d.conv42 - CMV[dataIndex])            / CMV[dataIndex]);

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

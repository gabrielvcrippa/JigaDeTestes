#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define trigPin 5
#define BT_DISCOVER_TIME 10000  // 10 segundos de escaneamento

const char* elmPIN    = "1234";
size_t      elmPINlen = strlen(elmPIN);

//Dados enviados
int dataIndex = 0;
int dataError = 0;

float engLoad[10] = {50, 90, 23, 75, 13, 46, 88, 5, 35, 68}; // [%] Carga do motor - 0 a 100 
float EngCoolTemp[10] = {125, 87, 45, 13, 96, 32, 67, 103, 19, 75}; // [°C] Temperatura do líquido de arrefecimento - -40 a 215
float STFT[10] = {-10, 5, -8, 12, -3, 9, -15, 3, -9, 5}; // [%] Ajuste de curto prazo de Combustível - -100 a 99,2
float LTFT[10] = {-15, -2, 8, -13, 4, -9, 5, -1, 10, -4}; // [%] Ajuste de longo prazo de Combustível - -100 a 99,2
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

float erro[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

bool menu = false;





void setup() {
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW); 

  Serial.begin(115200);
  delay(500);

  // Inicia o Bluetooth como mestre e define o PIN
  if (!SerialBT.begin("ESP32_OBD", true)) {
    Serial.println("❌ Falha ao iniciar Bluetooth SPP");
    while (1) delay(1000);
  }
  SerialBT.setPin(elmPIN, elmPINlen);
  Serial.println("✅ Bluetooth SPP iniciado como mestre");

  // Escaneia dispositivos
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

  // Usuário escolhe o índice do dispositivo
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

  // Conecta ao dispositivo selecionado
  if (!SerialBT.connect(chosen->getAddress())) {
    Serial.println("❌ Falha ao conectar");
    while (1) delay(1000);
  }
  Serial.println("✅ Conectado ao ELM327!");
  initializeELM();
}

void loop() {
  Serial.printf("Selecionar modo: \n 1 - Teste Automático \n");

  digitalWrite(trigPin, LOW);
  menu = false;
  dataError = 0;

  while (menu == false) {
    if(Serial.available() > 0){
    switch (Serial.read()) {
      case '1':
        Serial.println("Iniciando rotina automática");
        for (int i = 0; i <= 9; ++i) {
          recebeDados();
        }
        menu = true;     
        break;
      case 'o':
        break;
      }
    }
  }
}

void initializeELM() {
  // Sequência de comandos AT para configurar o ELM327
  const char* cmds[] = {
    "ATZ",    // Reset
    "ATE0",   // Echo Off
    "ATS0",   // Spaces Off
    "ATL0",   // Linefeeds Off
    "ATH0",   // Headers Off
    "ATSP0",  // Protocolo Automático
    "ATAT2",  // Temporização Adaptativa Agressiva
    "ATST10"  // Tempo de Espera de 10 ms
  };
  for (auto &c : cmds) {
    Serial.printf("AT→ %s\n", c);
    SerialBT.print(c);
    SerialBT.print("\r");
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
  while (millis() - startTime < 2000) {  // Aguarda até 2 segundos por uma resposta
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
  if (response.length() == 0) {
    return "NO DATA";
  }
  return response;
}

void recebeDados(){
  digitalWrite(trigPin, LOW); 
  // Limpa o buffer antes de cada leitura
  while (SerialBT.available()) SerialBT.read();


  // Solicita e lê Carga do Motor (PID 04)
  SerialBT.print("01 04\r");
  String resp04 = readELMResponse();
  float conv04 = strtoul(resp04.substring(4).c_str(), NULL, 16) * 100.0 / 255.0; // Variável pronta para uso
  Serial.printf("Carga do Motor: %.1f %%\n", conv04);

  // Solicita e lê Temperatura do Líquido de Arrefecimento (PID 05)
  SerialBT.print("01 05\r");
  String resp05 = readELMResponse();
  float conv05 = strtoul(resp05.substring(4).c_str(), NULL, 16) - 40.0; // Fórmula: A - 40
  Serial.printf("Temperatura: %.1f °C\n", conv05);

  // Solicita e lê Short Term Fuel Trim (PID 06)
  SerialBT.print("01 06\r");
  String resp06 = readELMResponse();
  float conv06 = (strtoul(resp06.substring(4).c_str(), NULL, 16) * 100.0 / 128.0) - 100; // Fórmula: (A*100/128)-100
  Serial.printf("Short FT: %.1f %%\n", conv06);

  // Solicita e lê Long Term Fuel Trim (PID 07)
  SerialBT.print("01 07\r");
  String resp07 = readELMResponse();
  float conv07 = (strtoul(resp07.substring(4).c_str(), NULL, 16) * 100.0 / 128.0) - 100; // Fórmula: (A*100/128)-100
  Serial.printf("Long FT: %.1f %%\n", conv07); // Ex: "-2.3 %" ou "+7.8 %"

  // Solicita e lê Fuel Pressure (PID 0A)
  SerialBT.print("01 0A\r");
  String resp0A = readELMResponse();
  float conv0A = strtoul(resp0A.substring(4).c_str(), NULL, 16) * 3; // kPa
  Serial.printf("Fuel Pressure: %.1f kPa\n", conv0A);

  // Solicita e lê Pressão do Coletor (PID 0B)
  SerialBT.print("01 0B\r");
  String resp0B = readELMResponse();
  float conv0B = strtoul(resp0B.substring(4).c_str(), NULL, 16); // kPa
  Serial.printf("Pressão Coletor: %.1f kPa\n", conv0B);

  // Solicita e lê RPM (PID 0C)
  SerialBT.print("01 0C\r");
  String resp0C = readELMResponse();
  float conv0C = strtoul(resp0C.substring(4).c_str(), NULL, 16) / 4.0; // Fórmula: (A*256+B)/4
  Serial.printf("RPM: %.0f\n", conv0C);

  // Solicita e lê Velocidade (PID 0D)
  SerialBT.print("01 0D\r");
  String resp0D = readELMResponse();
  float conv0D = strtoul(resp0D.substring(4).c_str(), NULL, 16); // km/h (valor direto)
  Serial.printf("Velocidade: %d km/h\n", conv0D);

  // Solicita e lê Intake Air Temperature (PID 0F)
  SerialBT.print("01 0F\r");
  String resp0F = readELMResponse();
  long rawValue = strtol(resp0F.substring(4).c_str(), NULL, 16); // Usando strtol para signed
  float conv0F = rawValue - 40; // Fórmula: A - 40
  Serial.printf("IAT: %.1f °C\n", conv0F);

  // Solicita e lê Mass Air Flow (PID 10)
  SerialBT.print("01 10\r");
  String resp10 = readELMResponse();
  float conv10 = strtoul(resp10.substring(4).c_str(), NULL, 16) / 100.0; // Fórmula: (A*256+B)/100
  Serial.printf("MAF: %.2f g/s\n", conv10);

  // Throttle Position (PID 11)
  SerialBT.print("01 11\r");
  String resp11 = readELMResponse();
  float conv11 = strtoul(resp11.substring(4).c_str(), NULL, 16) * 100.0 / 255.0;
  Serial.printf("Throttle: %.1f %%\n", conv11);

  // Oxygen Sensor 1 (PID 14)
  SerialBT.print("01 14\r");
  String resp14 = readELMResponse();
  unsigned int oxygen1 = strtoul(resp14.substring(4).c_str(), NULL, 16);
  float conv14a = (oxygen1 >> 8) * 0.005;
  float conv14b = ((oxygen1 & 0xFF) * 100.0 / 128.0) - 100;
  Serial.printf("O2 Sensor 1: %.3f V, %.1f %%\n", conv14a, conv14b);

  // Oxygen Sensor 2 (PID 15)
  SerialBT.print("01 15\r");
  String resp15 = readELMResponse();
  unsigned int oxygen2 = strtoul(resp15.substring(4).c_str(), NULL, 16);
  float conv15a = (oxygen2 >> 8) * 0.005;
  float conv15b = ((oxygen2 & 0xFF) * 100.0 / 128.0) - 100;
  Serial.printf("O2 Sensor 2: %.3f V, %.1f %%\n", conv15a, conv15b);

  // ECU Voltage (PID 42)
  SerialBT.print("01 42\r");
  String resp42 = readELMResponse();
  float conv42 = strtoul(resp42.substring(4).c_str(), NULL, 16) / 1000.0;
  Serial.printf("ECU Voltage: %.2f V\n", conv42);

  Serial.print("DI: ");
  Serial.println(dataIndex);

  digitalWrite(trigPin, HIGH);
  calculaErro(conv04, conv05, conv06, conv07, conv0A, conv0B, conv0C, conv0D,
         conv0F, conv10, conv11, conv14a, conv14b, conv15a, conv15b, conv42);
  delay(1000);  // Aguarda 1 segundo antes da próxima leitura
}

void calculaErro(float conv04, float conv05, float conv06, float conv07, float conv0A, 
                 float conv0B, float conv0C, float conv0D, float conv0F, float conv10,
                 float conv11, float conv14a, float conv14b, float conv15a, float conv15b, 
                 float conv42) {

  erro[0] = abs((conv04 - engLoad[dataIndex]) / engLoad[dataIndex]);
  erro[1] = abs((conv05 - EngCoolTemp[dataIndex]) / EngCoolTemp[dataIndex]);
  erro[2] = abs((conv06 - STFT[dataIndex]) / STFT[dataIndex]);
  erro[3] = abs((conv07 - LTFT[dataIndex]) / LTFT[dataIndex]);
  erro[4] = abs((conv0A - FuelPressure[dataIndex]) / FuelPressure[dataIndex]);
  erro[5] = abs((conv0B - IntManifAbsPres[dataIndex]) / IntManifAbsPres[dataIndex]);
  erro[6] = abs((conv0C - rpm[dataIndex]) / rpm[dataIndex]);
  erro[7] = abs((conv0D - VehiSpeed[dataIndex]) / VehiSpeed[dataIndex]);
  erro[8] = abs((conv0F - IntAirTemp[dataIndex]) / IntAirTemp[dataIndex]);
  erro[9] = abs((conv10 - MAF[dataIndex]) / MAF[dataIndex]);
  erro[10] = abs((conv11 - ThrPos[dataIndex]) / ThrPos[dataIndex]);
  erro[11] = abs((conv14a - VOS1[dataIndex]) / VOS1[dataIndex]);
  erro[12] = abs((conv14b - STFT1[dataIndex]) / STFT1[dataIndex]);
  erro[13] = abs((conv15a - VOS2[dataIndex]) / VOS2[dataIndex]);
  erro[14] = abs((conv15b - STFT2[dataIndex]) / STFT2[dataIndex]);
  erro[15] = abs((conv42 - CMV[dataIndex]) / CMV[dataIndex]);

  // Checa se erro é maior de 10%
  for (int i = 0; i < 16; i++) {
      if (erro[i] > 0.1) { 
          dataError++;
      }
  }
  Serial.printf("Erros: %d \n", dataError);
  Serial.printf("\n ===================================== \n");
  if (dataIndex >= 9){
    dataIndex = 0;
  }else{
    dataIndex++;
  }
}
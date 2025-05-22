#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define BT_DISCOVER_TIME 10000  // 10 segundos de escaneamento

const char* elmPIN    = "1234";
size_t      elmPINlen = strlen(elmPIN);

void setup() {
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
  int conv0D = strtoul(resp0D.substring(4).c_str(), NULL, 16); // km/h (valor direto)
  Serial.printf("Velocidade: %d km/h\n", conv0D);

  // Solicita e lê Intake Air Temperature (PID 0F)
  SerialBT.print("01 0F\r");
  String resp0F = readELMResponse();
  float conv0F = strtoul(resp0F.substring(4).c_str(), NULL, 16) - 40; // Fórmula: A - 40
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



  delay(1000);  // Aguarda 1 segundo antes da próxima leitura
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

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
  Serial.printf("Resposta Carga do Motor: %s\n", resp04.c_str());

  // Solicita e lê Temperatura do Líquido de Arrefecimento (PID 05)
  SerialBT.print("01 05\r");
  String resp05 = readELMResponse();
  Serial.printf("Resposta Temperatura: %s\n", resp05.c_str());

  // Solicita e lê Short Term Fuel Trim (PID 06)
  SerialBT.print("01 06\r");
  String resp06 = readELMResponse();
  Serial.printf("Resposta Short Fuel Trim: %s\n", resp06.c_str());

  // Solicita e lê Long Term Fuel Trim (PID 07)
  SerialBT.print("01 07\r");
  String resp07 = readELMResponse();
  Serial.printf("Resposta Long Fuel Trim: %s\n", resp07.c_str());

  // Solicita e lê Fuel Pressure (PID 0A)
  SerialBT.print("01 0A\r");
  String resp0A = readELMResponse();
  Serial.printf("Resposta Fuel Pressure: %s\n", resp0A.c_str());

  // Solicita e lê Intake Manifold Absolute Pressure (PID 0B)
  SerialBT.print("01 0B\r");
  String resp0B = readELMResponse();
  Serial.printf("Resposta Intake Manifold Absolute Pressure: %s\n", resp0B.c_str());

  // Solicita e lê RPM (PID 0C)
  SerialBT.print("01 0C\r");
  String resp0C = readELMResponse();
  Serial.printf("Resposta RPM: %s\n", resp0C.c_str());

  // Solicita e lê Velocidade (PID 0D)
  SerialBT.print("01 0D\r");
  String resp0D = readELMResponse();
  Serial.printf("Resposta Velocidade: %s\n", resp0D.c_str());

  // Solicita e lê Intake Air Temperature (PID 0F)
  SerialBT.print("01 0F\r");
  String resp0F = readELMResponse();
  Serial.printf("Resposta Intake Air Temperature: %s\n", resp0F.c_str());

  // Solicita e lê Mass Air Flow (PID 10)
  SerialBT.print("01 10\r");
  String resp10 = readELMResponse();
  Serial.printf("Resposta Mass Air Flow: %s\n", resp10.c_str());

  // Solicita e lê Throttle position (PID 11)
  SerialBT.print("01 11\r");
  String resp11 = readELMResponse();
  Serial.printf("Resposta Throttle position: %s\n", resp11.c_str());

  // Solicita e lê Oxygen 1 (PID 14)
  SerialBT.print("01 14\r");
  String resp14 = readELMResponse();
  Serial.printf("Resposta Oxygen 1: %s\n", resp14.c_str());

  // Solicita e lê Oxygen 2 (PID 15)
  SerialBT.print("01 15\r");
  String resp15 = readELMResponse();
  Serial.printf("Resposta Oxygen 2: %s\n", resp15.c_str());

  // Solicita e lê ECU Voltage (PID 42)
  SerialBT.print("01 42\r");
  String resp42 = readELMResponse();
  Serial.printf("Resposta ECU Votage: %s\n", resp42.c_str());



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

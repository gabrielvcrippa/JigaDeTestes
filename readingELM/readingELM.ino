#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define BT_DISCOVER_TIME 10000  // 10 s de scan

const char* elmPIN    = "1234";
size_t      elmPINlen = strlen(elmPIN);

void setup() {
  Serial.begin(115200);
  delay(500);

  // 1. Inicia Bluetooth Classic como mestre e define PIN
  if (!SerialBT.begin("ESP32_OBD", true)) {
    Serial.println("❌ Falha ao iniciar Bluetooth SPP");
    while (1) delay(1000);
  }
  SerialBT.setPin(elmPIN, elmPINlen);
  Serial.println("✅ Bluetooth SPP iniciado como mestre");

  // 2. Escaneia dispositivos
  Serial.printf("🔍 Escaneando dispositivos (%d ms)...\n", BT_DISCOVER_TIME);
  BTScanResults* results = SerialBT.discover(BT_DISCOVER_TIME);  // scan síncrono :contentReference[oaicite:0]{index=0}
  if (!results) {
    Serial.println("❌ Erro no scan Bluetooth");
    while (1) delay(1000);
  }

  int count = results->getCount();
  Serial.printf("Dispositivos encontrados: %d\n", count);
  for (int i = 0; i < count; ++i) {
    // Usa ponteiro em vez de referência abstrata
    BTAdvertisedDevice* dev = results->getDevice(i);
    Serial.printf(" [%d] %s  [%s]\n",
      i,
      dev->haveName() ? dev->getName().c_str() : "<sem nome>",
      dev->getAddress().toString().c_str()
    );
  }

  // 3. Usuário escolhe índice
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

  // 4. Conecta via SPP pelo MAC do device escolhido :contentReference[oaicite:1]{index=1}
  if (!SerialBT.connect(chosen->getAddress())) {
    Serial.println("❌ Falha ao conectar");
    while (1) delay(1000);
  }
  Serial.println("✅ Conectado ao ELM327!");
  initializeELM();
}

void loop() {
  // 5. Solicita e lê RPM (PID 0C) :contentReference[oaicite:2]{index=2}
  while (SerialBT.available()) {
    SerialBT.read();
  }
  SerialBT.print("01 0C\r");
  delay(200);
  String resp0C = SerialBT.readStringUntil('>');
  resp0C.trim();
  Serial.printf("Resposta RPM raw: %s\n", resp0C.c_str());
  delay(1000);


  while (SerialBT.available()) {
    SerialBT.read();
  }
  SerialBT.print("01 0D\r");
  delay(200);
  String resp0D = SerialBT.readStringUntil('>');
  resp0D.trim();
  Serial.printf("Resposta velocidade raw: %s\n", resp0D.c_str());
  delay(1000);


  while (SerialBT.available()) {
    SerialBT.read();
  }
  SerialBT.print("01 05\r");
  delay(200);
  String resp05 = SerialBT.readStringUntil('>');
  resp05.trim();
  Serial.printf("Resposta Temperatura raw: %s\n", resp05.c_str());
  delay(1000);
}



void initializeELM() {
  // Sequência de AT-commands padrão
  const char* cmds[] = {"ATZ", "ATE0", "ATL0", "ATS0", "ATSP0"};
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
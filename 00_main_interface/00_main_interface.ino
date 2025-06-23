// ===================================================================================
// PROJETO COMPLETO - VERSÃO UNIFICADA E FINAL
// ===================================================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include "FS.h"
#include "Free_Fonts.h"
#include "BluetoothSerial.h"

// --- OBJETOS GLOBAIS ---
TFT_eSPI tft = TFT_eSPI();
BluetoothSerial SerialBT;

// --- DEFINIÇÕES GLOBAIS ---
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320
#define TABELA_X 10
#define TABELA_WIDTH (SCREEN_WIDTH * 0.6)
#define BOTOES_WIDTH (SCREEN_WIDTH * 0.3)
#define BTN_HEIGHT 50
#define ESPACAMENTO 20
#define MSG_Y 30
#define TABELA_Y 70
#define PAG_BTN_HEIGHT 28
#define PAG_Y_POS (SCREEN_HEIGHT - PAG_BTN_HEIGHT - 5)
#define TABELA_HEIGHT (PAG_Y_POS - TABELA_Y - 10)
#define BOTOES_X (TABELA_X + TABELA_WIDTH + 10)
#define BTN_PROCURAR_Y TABELA_Y
#define BTN_CONECTAR_Y (BTN_PROCURAR_Y + BTN_HEIGHT + ESPACAMENTO)
#define MENSAGEM_ERRO_Y (BTN_CONECTAR_Y + BTN_HEIGHT + 10)
#define PAG_BTN_WIDTH 40
#define PAG_BTN_PREV_X (TABELA_X + 20)
#define PAG_BTN_NEXT_X (TABELA_X + TABELA_WIDTH - PAG_BTN_WIDTH - 20)
#define PAG_TEXT_Y (PAG_Y_POS + PAG_BTN_HEIGHT / 2)
#define CINZA_ESCURO 0x39E7
#define CINZA_CLARO  TFT_DARKGREY
#define LARANJA      TFT_ORANGE
#define VERDE        TFT_GREEN
#define AZUL         TFT_BLUE
#define VERMELHO     TFT_RED
#define CALIBRATION_FILE "/final_calibration_file"
#define MENUBAR_HEIGHT 40
#define TABLE_Y (MENUBAR_HEIGHT + 5)
#define TABLE_ROW_HEIGHT 48
#define NUM_PIDS 15
#define PIDS_PER_TABLE 5
#define trigPin 5
#define intervaloCapturaDado 250

// --- ESTRUTURAS E ESTADOS GLOBAIS ---
enum ScreenState { TELA_BLUETOOTH, TELA_MENU_TESTES, TELA_TESTE_AUTO, TELA_TESTE_MANUAL };
ScreenState estadoAtual = TELA_BLUETOOTH;

struct DispositivoBluetooth { String nome; BTAddress* pAddress; };
struct DadosRecebidos {
  float conv04, conv05, conv06, conv07, conv0A, conv0B, conv0C, conv0D, conv0F,
        conv10, conv11, conv14a, conv14b, conv15a, conv15b, conv42;
};

bool btConectado = false;
bool testIsPaused = false;
int dataIndex = 0;
int totalError = 0;

// Arrays de gabarito para o teste automático
float engLoad[11]         = {50, 90, 23, 75, 13, 46, 88, 44, 35, 68, 0};
float EngCoolTemp[11]     = {125, -20, 45, 13, 96, 32, 67, 103, 19, 75, 0};
float STFT[11]            = {-10, 55, -80, 12, -40, 9, -15, 23, -9, 35, 0};
float LTFT[11]            = {-15, -22, 8, -13, 4, -9, 5, 60, 10, -4, 0};
float FuelPressure[11]    = {102, 235, 88, 345, 157, 298, 45, 188, 277, 66, 0};
float IntManifAbsPres[11] = {40, 87, 33, 125, 57, 98, 12, 77, 145, 24, 0};
float rpm[11]             = {3000, 1250, 8765, 3422, 987, 5433, 7654, 2346, 4322, 6543, 0};
float VehiSpeed[11]       = {60, 112, 46, 88, 32, 77, 124, 54, 99, 23, 0};
float IntAirTemp[11]      = {25, -12, 38, 116, -32, -9, 20, 31, -4, 201, 0};
float MAF[11]             = {10, 33, 88, 124, 46, 98, 156, 624, 277, 187, 0};
float ThrPos[11]          = {25, 12, 88, 34, 57, 9, 79, 24, 46, 67, 0};
float VOS1[11]            = {0.75, 0.23, 0.98, 0.45, 0.67, 0.12, 0.89, 0.34, 0.56, 0.78, 0};
float STFT1[11]           = {0, 60, -12, 82, -47, 10, -80, 3, -18, 41, 0};
float VOS2[11]            = {0.75, 0.34, 0.87, 0.52, 0.91, 0.23, 0.68, 0.45, 0.79, 0.12, 0};
float STFT2[11]           = {0, -38, 13, -5, 58, -3, 7, -12, 44, -66, 0};
float CMV[11]             = {12, 17, 22, 13, 40, 5, 33, 52, 38, 9, 0};
float erro[16] = {0};

uint16_t calData[5] = { 342, 3492, 279, 3535, 7 };
DispositivoBluetooth* dispositivosEncontrados = nullptr;
int numDispositivosEncontrados = 0;
int dispositivoSelecionado = -1;
int paginaAtual = 0;
int itensPorPagina = 6;
int totalPaginas = 0;

const int pinAdc1 = 34; const int pinAdc2 = 35; const int pinAdc3 = 32;
const int pinAdc4 = 33; const int pinAdc5 = 25; const int pinAdc6 = 26;
const char* pidNames[NUM_PIDS] = {
  "Carga Motor", "Temp. Liq.", "Short Term F.T.", "Long Term F.T.", "Pres. Combus.",
  "Pres. Coletor", "RPM", "Velocidade", "Temp. Ar Adm.", "Fluxo de Ar",
  "Pos. Borbol.", "Tensao O2(B1S1)", "Ajuste O2(B1S1)", "Tensao O2(B1S2)", "Ajuste O2(B1S2)"
};

// ===================================================================================
// SEÇÃO DE FUNÇÕES DE DESENHO E UI
// ===================================================================================
void desenhaBotao(int x, int y, int w, int h, const char* texto, uint16_t cor);
void desenhaPaginacao();
void desenhaListaDispositivos();
void mostrarTelaBluetooth(const char* msgErro = "");
void mostrarTelaMenuTestes();
void desenhaMenuBarTeste();
void desenhaTabelasBase();
void atualizaValoresTabelas(const DadosRecebidos& dados, const float erros[]);
void mostrarTelaTesteAuto();
void mostrarTelaTesteManual();

// ===================================================================================
// SEÇÃO DE LÓGICA E CONTROLE
// ===================================================================================
void touch_calibrate();
void procurarDispositivos();
String readELMResponse();
DadosRecebidos recebeDados(bool ativaTrigger, bool rotinaAutomatica);
void calculaErro(const DadosRecebidos& d);
void initLeitura();
void initializeELM();
void conectarDispositivoSelecionado();
void finalizarTeste();
void handleTouch_TelaBluetooth(uint16_t x, uint16_t y);
void handleTouch_TelaMenuTestes(uint16_t x, uint16_t y);
void handleTouch_TelaTesteAuto(uint16_t x, uint16_t y);
void handleTouch_TelasDeTeste(uint16_t x, uint16_t y);
void iniciarTesteAutomatico();
void iniciarTesteManual();
void sincronizarIndices();


// ===================================================================================
// IMPLEMENTAÇÃO DAS FUNÇÕES
// ===================================================================================

void desenhaBotao(int x, int y, int w, int h, const char* texto, uint16_t cor) {
  tft.fillRoundRect(x, y, w, h, 8, cor);
  tft.setTextColor(TFT_WHITE, cor);
  tft.setFreeFont(FSSB9);
  tft.setTextDatum(CC_DATUM);
  tft.drawString(texto, x + w / 2, y + h / 2);
}

void desenhaPaginacao() {
  totalPaginas = (numDispositivosEncontrados + itensPorPagina - 1) / itensPorPagina;
  if (totalPaginas == 0) totalPaginas = 1;
  tft.fillRect(PAG_BTN_PREV_X + PAG_BTN_WIDTH, PAG_Y_POS, PAG_BTN_NEXT_X - (PAG_BTN_PREV_X + PAG_BTN_WIDTH), PAG_BTN_HEIGHT, CINZA_ESCURO);
  uint16_t corBotaoEsquerda = (paginaAtual > 0) ? LARANJA : CINZA_CLARO;
  desenhaBotao(PAG_BTN_PREV_X, PAG_Y_POS, PAG_BTN_WIDTH, PAG_BTN_HEIGHT, "<", corBotaoEsquerda);
  uint16_t corBotaoDireita = (paginaAtual < totalPaginas - 1) ? LARANJA : CINZA_CLARO;
  desenhaBotao(PAG_BTN_NEXT_X, PAG_Y_POS, PAG_BTN_WIDTH, PAG_BTN_HEIGHT, ">", corBotaoDireita);
  String textoPagina = "Pagina " + String(paginaAtual + 1) + " / " + String(totalPaginas);
  tft.setTextColor(TFT_WHITE); tft.setFreeFont(FSSB9); tft.setTextDatum(CC_DATUM);
  tft.drawString(textoPagina, TABELA_X + TABELA_WIDTH / 2, PAG_TEXT_Y);
}

void desenhaListaDispositivos() {
  int listaStartY = TABELA_Y + 25;
  int alturaDisponivel = TABELA_HEIGHT - 30;
  int alturaLinha = alturaDisponivel / itensPorPagina;
  tft.fillRect(TABELA_X + 1, listaStartY, TABELA_WIDTH - 2, alturaDisponivel + 5, CINZA_ESCURO);
  int indiceInicial = paginaAtual * itensPorPagina;
  int indiceFinal = min(indiceInicial + itensPorPagina, numDispositivosEncontrados);
  tft.setFreeFont(FSSB9);
  tft.setTextDatum(ML_DATUM);
  for (int i = indiceInicial; i < indiceFinal; i++) {
    int yPosItem = listaStartY + (i - indiceInicial) * alturaLinha;
    String nomeExibido = dispositivosEncontrados[i].nome;
    if (i == dispositivoSelecionado) {
      tft.fillRect(TABELA_X + 1, yPosItem, TABELA_WIDTH - 2, alturaLinha, AZUL);
      tft.setTextColor(TFT_WHITE, AZUL);
    } else {
      tft.fillRect(TABELA_X + 1, yPosItem, TABELA_WIDTH - 2, alturaLinha, CINZA_ESCURO);
      tft.setTextColor(TFT_WHITE, CINZA_ESCURO);
    }
    tft.drawString(nomeExibido.substring(0, 25), TABELA_X + 10, yPosItem + (alturaLinha / 2));
  }
}

void mostrarTelaBluetooth(const char* msgErro) {
  tft.fillScreen(CINZA_ESCURO);
  tft.setFreeFont(FSSB9);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(LARANJA, CINZA_ESCURO);
  tft.drawString("Conecte ao dispositivo ELM", SCREEN_WIDTH / 2, MSG_Y);
  tft.drawRect(TABELA_X, TABELA_Y, TABELA_WIDTH, TABELA_HEIGHT, TFT_WHITE);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_YELLOW, CINZA_ESCURO);
  tft.drawString("Dispositivos:", TABELA_X + 5, TABELA_Y + 5);
  desenhaListaDispositivos();
  desenhaPaginacao();
  desenhaBotao(BOTOES_X, BTN_PROCURAR_Y, BOTOES_WIDTH, BTN_HEIGHT, "Procurar", VERDE);
  desenhaBotao(BOTOES_X, BTN_CONECTAR_Y, BOTOES_WIDTH, BTN_HEIGHT, "Conectar", AZUL);
  if (strlen(msgErro) > 0) {
    tft.setTextColor(VERMELHO, CINZA_ESCURO);
    tft.drawString(msgErro, BOTOES_X, MENSAGEM_ERRO_Y);
  }
}

void mostrarTelaMenuTestes() {
  tft.fillScreen(CINZA_ESCURO);
  tft.setTextColor(LARANJA);
  tft.setTextDatum(TC_DATUM);
  tft.setFreeFont(FSSB9);
  tft.drawString("Conectado!", SCREEN_WIDTH / 2, MSG_Y);
  tft.drawString("Selecione o modo de teste:", SCREEN_WIDTH / 2, MSG_Y + 30);
  int btnWidth = SCREEN_WIDTH * 0.6;
  int btnY = SCREEN_HEIGHT / 2 - BTN_HEIGHT;
  desenhaBotao((SCREEN_WIDTH - btnWidth) / 2, btnY, btnWidth, BTN_HEIGHT, "Teste Automático", AZUL);
  desenhaBotao((SCREEN_WIDTH - btnWidth) / 2, btnY + BTN_HEIGHT + ESPACAMENTO, btnWidth, BTN_HEIGHT, "Teste Manual", VERDE);
}

void desenhaMenuBarTeste() {
  tft.fillRect(0, 0, SCREEN_WIDTH, MENUBAR_HEIGHT, LARANJA);
  tft.fillTriangle(10, MENUBAR_HEIGHT / 2, 30, 10, 30, MENUBAR_HEIGHT - 10, TFT_BLACK);
  uint16_t iconColor = TFT_BLACK;
  int iconSize = MENUBAR_HEIGHT * 0.6;
  int iconCenterX = SCREEN_WIDTH - 25;
  int iconCenterY = MENUBAR_HEIGHT / 2;
  if (testIsPaused) {
    int playWidth = iconSize * 0.8;
    int playHeight = iconSize;
    tft.fillTriangle(iconCenterX - playWidth / 2, iconCenterY - playHeight / 2, iconCenterX + playWidth / 2, iconCenterY, iconCenterX - playWidth / 2, iconCenterY + playHeight / 2, iconColor);
  } else {
    int barWidth = iconSize / 3.5;
    int barHeight = iconSize;
    int barSpacing = iconSize / 4;
    tft.fillRect(iconCenterX - barSpacing / 2 - barWidth, iconCenterY - barHeight / 2, barWidth, barHeight, iconColor);
    tft.fillRect(iconCenterX + barSpacing / 2,            iconCenterY - barHeight / 2, barWidth, barHeight, iconColor);
  }
}

void desenhaTabelasBase() {
  int tableWidth = SCREEN_WIDTH / 3;
  tft.setTextDatum(TC_DATUM);
  tft.setFreeFont(FSSB9);
  for (int i = 0; i < 3; i++) {
    int tableX = i * tableWidth;
    tft.setTextColor(TFT_YELLOW);
    tft.drawString("PID", tableX + tableWidth / 4, TABLE_Y + 5);
    tft.drawString("VALOR", tableX + (tableWidth / 4) * 3, TABLE_Y + 5);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(TL_DATUM);
    for (int j = 0; j < PIDS_PER_TABLE; j++) {
      int rowY = TABLE_Y + 30 + (j * TABLE_ROW_HEIGHT);
      int pidIndex = i * PIDS_PER_TABLE + j;
      if (pidIndex < NUM_PIDS) {
        tft.drawString(pidNames[pidIndex], tableX + 5, rowY);
        tft.drawFastHLine(tableX, rowY + TABLE_ROW_HEIGHT - 5, tableWidth - 2, CINZA_CLARO);
      }
    }
    tft.drawFastVLine(tableX + tableWidth/2 -2, TABLE_Y + 28, 250, CINZA_CLARO);
  }
}

void atualizaValoresTabelas(const DadosRecebidos& dados, const float erros[]) {
  int tableWidth = SCREEN_WIDTH / 3;
  tft.setTextDatum(TR_DATUM);
  tft.setFreeFont(FSS9);
  float valoresLidos[] = { dados.conv04, dados.conv05, dados.conv06, dados.conv07, dados.conv0A, dados.conv0B, dados.conv0C, dados.conv0D, dados.conv0F, dados.conv10, dados.conv11, dados.conv14a, dados.conv14b, dados.conv15a, dados.conv15b };
  float valoresGabarito[] = { engLoad[dataIndex], EngCoolTemp[dataIndex], STFT[dataIndex], LTFT[dataIndex], FuelPressure[dataIndex], IntManifAbsPres[dataIndex], rpm[dataIndex], VehiSpeed[dataIndex], IntAirTemp[dataIndex], MAF[dataIndex], ThrPos[dataIndex], VOS1[dataIndex], STFT1[dataIndex], VOS2[dataIndex], STFT2[dataIndex] };
  for (int i = 0; i < 3; i++) {
    int tableX = i * tableWidth;
    for (int j = 0; j < PIDS_PER_TABLE; j++) {
      int rowY = TABLE_Y + 30 + (j * TABLE_ROW_HEIGHT);
      int pidIndex = i * PIDS_PER_TABLE + j;
      if (pidIndex < NUM_PIDS) {
        tft.fillRect(tableX + tableWidth/2, rowY, tableWidth/2-2, TABLE_ROW_HEIGHT - 6, CINZA_ESCURO);
        uint16_t corValor = (erros[pidIndex] > 0.1) ? VERMELHO : VERDE;
        tft.setTextColor(corValor);
        String valorStr = String(valoresLidos[pidIndex], 1) + "/" + String(valoresGabarito[pidIndex], 1);
        tft.drawString(valorStr, tableX + tableWidth - 5, rowY);
      }
    }
  }
}

void mostrarTelaTesteAuto() {
  tft.fillScreen(CINZA_ESCURO);
  desenhaMenuBarTeste();
  desenhaTabelasBase();
}

void mostrarTelaTesteManual() {
    tft.fillScreen(CINZA_ESCURO);
    tft.setTextColor(LARANJA);
    tft.setTextDatum(TC_DATUM);
    tft.setFreeFont(FSSB9);
    tft.drawString("Teste Manual Ativo", SCREEN_WIDTH / 2, MSG_Y);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Verifique o Monitor Serial para os resultados.", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
    desenhaBotao(10, 10, 100, 40, "Voltar", CINZA_CLARO);
}


void touch_calibrate() {
  uint16_t calData[5];
  uint8_t calDataOK = 0;
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    fs::File f = SPIFFS.open(CALIBRATION_FILE, "r");
    if (f) {
      if (f.readBytes((char*)calData, sizeof(calData)) == sizeof(calData)) calDataOK = 1;
      f.close();
    }
  }
  if (calDataOK) {
    tft.setTouch(calData);
  } else {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0, 2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println("Toque nos cantos para calibrar");
    tft.calibrateTouch(calData, TFT_RED, TFT_BLACK, 15);
    fs::File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char*)calData, sizeof(calData));
      f.close();
    }
    tft.setTouch(calData);
  }
}

void procurarDispositivos() {
  desenhaBotao(BOTOES_X, BTN_PROCURAR_Y, BOTOES_WIDTH, BTN_HEIGHT, "Procurando...", CINZA_CLARO);
  if(dispositivosEncontrados != nullptr){
    for (int i = 0; i < numDispositivosEncontrados; i++) {
      delete dispositivosEncontrados[i].pAddress;
    }
    delete[] dispositivosEncontrados;
    dispositivosEncontrados = nullptr;
  }
  numDispositivosEncontrados = 0;
  dispositivoSelecionado = -1;
  paginaAtual = 0;
  BTScanResults* results = SerialBT.discover(10000);
  if (results) {
    numDispositivosEncontrados = results->getCount();
    if (numDispositivosEncontrados > 0) {
      dispositivosEncontrados = new DispositivoBluetooth[numDispositivosEncontrados];
      for(int i=0; i < numDispositivosEncontrados; i++){
        BTAdvertisedDevice* dev = results->getDevice(i);
        dispositivosEncontrados[i].nome = dev->haveName() ? dev->getName().c_str() : dev->getAddress().toString().c_str();
        dispositivosEncontrados[i].pAddress = new BTAddress(dev->getAddress());
      }
    }
  }
  results->~BTScanResults();
  desenhaBotao(BOTOES_X, BTN_PROCURAR_Y, BOTOES_WIDTH, BTN_HEIGHT, "Procurar", VERDE);
  desenhaListaDispositivos();
  desenhaPaginacao();
}

String readELMResponse() {
  String response = "";
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '>') { response.trim(); return response; }
      response += c;
    }
  }
  response.trim();
  return response.length() ? response : "NO DATA";
}

DadosRecebidos recebeDados(bool ativaTrigger, bool rotinaAutomatica) {
    DadosRecebidos dados = {0}; // Inicializa para garantir que não haja lixo de memória
    while (SerialBT.available()) SerialBT.read();

    if (rotinaAutomatica){
        SerialBT.print("01 06\r"); String r = readELMResponse(); if(r!="NO DATA") dados.conv06 = (strtoul(r.substring(4).c_str(),NULL,16)*100.0/128.0)-100;
        SerialBT.print("01 07\r"); r = readELMResponse(); if(r!="NO DATA") dados.conv07 = (strtoul(r.substring(4).c_str(),NULL,16)*100.0/128.0)-100;
        SerialBT.print("01 0F\r"); r = readELMResponse(); if(r!="NO DATA") dados.conv0F = strtol(r.substring(4).c_str(),NULL,16) - 40;
        SerialBT.print("01 10\r"); r = readELMResponse(); if(r!="NO DATA") { uint16_t val = strtoul(r.substring(4).c_str(),NULL,16); dados.conv10 = val / 100.0; }
        SerialBT.print("01 11\r"); r = readELMResponse(); if(r!="NO DATA") dados.conv11 = strtoul(r.substring(4).c_str(),NULL,16) * 100.0/255.0;
        SerialBT.print("01 14\r"); r = readELMResponse(); if(r!="NO DATA"){uint16_t o2=strtoul(r.substring(4).c_str(),NULL,16);dados.conv14a=(o2>>8)*0.005;dados.conv14b=((o2&0xFF)*100.0/128.0)-100;}
        SerialBT.print("01 15\r"); r = readELMResponse(); if(r!="NO DATA"){uint16_t o2=strtoul(r.substring(4).c_str(),NULL,16);dados.conv15a=(o2>>8)*0.005;dados.conv15b=((o2&0xFF)*100.0/128.0)-100;}
        SerialBT.print("01 42\r"); r = readELMResponse(); if(r!="NO DATA") dados.conv42 = strtoul(r.substring(4).c_str(),NULL,16)/1000.0;
    }

    SerialBT.print("01 04\r"); String r4 = readELMResponse(); if (r4 != "NO DATA") dados.conv04 = strtoul(r4.substring(4).c_str(), NULL, 16) * 100.0 / 255.0;
    SerialBT.print("01 05\r"); String r5 = readELMResponse(); if (r5 != "NO DATA") dados.conv05 = strtoul(r5.substring(4).c_str(), NULL, 16) - 40;
    SerialBT.print("01 0A\r"); String rA = readELMResponse(); if (rA != "NO DATA") dados.conv0A = strtoul(rA.substring(4).c_str(), NULL, 16) * 3.0;
    SerialBT.print("01 0B\r"); String rB = readELMResponse(); if (rB != "NO DATA") dados.conv0B = strtoul(rB.substring(4).c_str(), NULL, 16);
    SerialBT.print("01 0C\r"); String rC = readELMResponse(); if (rC != "NO DATA") { uint16_t raw = strtoul(rC.substring(4).c_str(), NULL, 16); dados.conv0C = raw / 4.0; }
    SerialBT.print("01 0D\r"); String rD = readELMResponse(); if (rD != "NO DATA") dados.conv0D = strtoul(rD.substring(4).c_str(), NULL, 16);    
    return dados;
}

void calculaErro(const DadosRecebidos& d) {
  int dataError=0;
  float valoresGabarito[]={engLoad[dataIndex],EngCoolTemp[dataIndex],STFT[dataIndex],LTFT[dataIndex],FuelPressure[dataIndex],IntManifAbsPres[dataIndex],rpm[dataIndex],VehiSpeed[dataIndex],IntAirTemp[dataIndex],MAF[dataIndex],ThrPos[dataIndex],VOS1[dataIndex],STFT1[dataIndex],VOS2[dataIndex],STFT2[dataIndex],CMV[dataIndex]};
  float valoresLidos[]={d.conv04,d.conv05,d.conv06,d.conv07,d.conv0A,d.conv0B,d.conv0C,d.conv0D,d.conv0F,d.conv10,d.conv11,d.conv14a,d.conv14b,d.conv15a,d.conv15b,d.conv42};
  
  for(int i=0; i < 16; ++i){
    erro[i] = 0;
    if(abs(valoresGabarito[i]) > 0.01) { 
      erro[i] = abs((valoresLidos[i] - valoresGabarito[i]) / valoresGabarito[i]);
    } else {
      erro[i] = (abs(valoresLidos[i]) > 0.01) ? 1.0 : 0.0;
    }
    if(erro[i] > 0.1) {
      dataError++;
      totalError++;
    }
  }
  Serial.printf("Passo %d - Erros neste passo: %d\n", dataIndex + 1, dataError);
  // A linha de incremento foi REMOVIDA daqui.
}

void initLeitura() {
  while (SerialBT.available()) SerialBT.read();
  SerialBT.print("01 0D\r");
  readELMResponse();
  delay(1000);
}

void initializeELM() {
  const char* cmds[] = { "ATZ", "ATE0", "ATS0", "ATL0", "ATH0", "ATSP0", "ATAT2", "ATST10" };
  for (auto& c : cmds) {
    SerialBT.print(String(c) + "\r");
    delay(200);
    SerialBT.readStringUntil('>');
  }
}

void conectarDispositivoSelecionado() {
  if (dispositivoSelecionado < 0 || dispositivoSelecionado >= numDispositivosEncontrados) {
    mostrarTelaBluetooth("Selecione um disp.");
    return;
  }
  desenhaBotao(BOTOES_X, BTN_CONECTAR_Y, BOTOES_WIDTH, BTN_HEIGHT, "Conectando...", CINZA_CLARO);
  DispositivoBluetooth* dev = &dispositivosEncontrados[dispositivoSelecionado];
  btConectado = SerialBT.connect(*dev->pAddress);
  if (btConectado) {
    initializeELM();
    estadoAtual = TELA_MENU_TESTES;
    mostrarTelaMenuTestes();
  } else {
    mostrarTelaBluetooth("Falha ao conectar.");
  }
}

void finalizarTeste() {
    digitalWrite(trigPin, LOW); 
    dataIndex = 0;
    totalError = 0;
    testIsPaused = false;
    estadoAtual = TELA_MENU_TESTES;
    mostrarTelaMenuTestes();
}

void handleTouch_TelaBluetooth(uint16_t x, uint16_t y) {
  if ((x > BOTOES_X) && (x < (BOTOES_X + BOTOES_WIDTH)) && (y > BTN_PROCURAR_Y) && (y < (BTN_PROCURAR_Y + BTN_HEIGHT))) {
    procurarDispositivos();
  } else if ((x > BOTOES_X) && (x < (BOTOES_X + BOTOES_WIDTH)) && (y > BTN_CONECTAR_Y) && (y < (BTN_CONECTAR_Y + BTN_HEIGHT))) {
    conectarDispositivoSelecionado();
  } else if ((x > PAG_BTN_PREV_X) && (x < (PAG_BTN_PREV_X + PAG_BTN_WIDTH)) && (y > PAG_Y_POS) && (y < (PAG_Y_POS + PAG_BTN_HEIGHT))) {
    if (paginaAtual > 0) { paginaAtual--; desenhaListaDispositivos(); desenhaPaginacao(); }
  } else if ((x > PAG_BTN_NEXT_X) && (x < (PAG_BTN_NEXT_X + PAG_BTN_WIDTH)) && (y > PAG_Y_POS) && (y < (PAG_Y_POS + PAG_BTN_HEIGHT))) {
    if (paginaAtual < totalPaginas - 1) { paginaAtual++; desenhaListaDispositivos(); desenhaPaginacao(); }
  } else if ((x > TABELA_X) && (x < (TABELA_X + TABELA_WIDTH)) && (y > (TABELA_Y + 25)) && (y < (PAG_Y_POS - 5))) {
    int listaStartY = TABELA_Y + 25;
    int alturaDisponivel = TABELA_HEIGHT - 30;
    int alturaLinha = alturaDisponivel / itensPorPagina;
    int itemClicadoNaPagina = (y - listaStartY) / alturaLinha;
    int novoIndiceSelecionado = (paginaAtual * itensPorPagina) + itemClicadoNaPagina;
    if (novoIndiceSelecionado < numDispositivosEncontrados) {
      dispositivoSelecionado = (dispositivoSelecionado == novoIndiceSelecionado) ? -1 : novoIndiceSelecionado;
      desenhaListaDispositivos();
    }
  }
}

void handleTouch_TelaMenuTestes(uint16_t x, uint16_t y) {
    int btnWidth = SCREEN_WIDTH * 0.6;
    int btnY_auto = SCREEN_HEIGHT / 2 - BTN_HEIGHT;
    int btnY_manual = btnY_auto + BTN_HEIGHT + ESPACAMENTO;
    int btnX = (SCREEN_WIDTH - btnWidth) / 2;
    if ((x > btnX) && (x < btnX + btnWidth) && (y > btnY_auto) && (y < btnY_auto + BTN_HEIGHT)) {
        iniciarTesteAutomatico();
    } else if ((x > btnX) && (x < btnX + btnWidth) && (y > btnY_manual) && (y < btnY_manual + BTN_HEIGHT)) {
        iniciarTesteManual();
    }
}

void handleTouch_TelaTesteAuto(uint16_t x, uint16_t y) {
  if (x < 60 && y < MENUBAR_HEIGHT) {
    finalizarTeste();
  } else if (x > (SCREEN_WIDTH - 60) && y < MENUBAR_HEIGHT) {
    testIsPaused = !testIsPaused;
    desenhaMenuBarTeste();
  }
}

void handleTouch_TelasDeTeste(uint16_t x, uint16_t y) {
    if ((x > 10) && (x < 110) && (y > 10) && (y < 50)) {
        finalizarTeste();
    }
}

void iniciarTesteAutomatico() {
    estadoAtual = TELA_TESTE_AUTO;
    testIsPaused = false;
    dataIndex = 0;
    totalError = 0;
    
    digitalWrite(trigPin, LOW);
    delay(100);

    mostrarTelaTesteAuto();
    initLeitura();
    
    for(int i=0; i<10; i++){
        while(testIsPaused){
            uint16_t tx,ty;
            if(tft.getTouch(&tx,&ty)){
                handleTouch_TelaTesteAuto(tx,ty);
                while(tft.getTouch(&tx,&ty)){}
            }
            if(estadoAtual!=TELA_TESTE_AUTO)return;
            delay(100);
        }
        if(estadoAtual!=TELA_TESTE_AUTO)return;
        
        digitalWrite(trigPin, LOW);
        
        DadosRecebidos dados=recebeDados(true,true);
        
        // Calcula o erro para o índice ATUAL
        calculaErro(dados);
        
        // Mostra os resultados na tela (dados e gabarito do índice ATUAL)
        atualizaValoresTabelas(dados,erro);
        
        // Agora, incrementa o índice para o PRÓXIMO passo
        if (++dataIndex >= 10) dataIndex = 0;

        // Sinaliza para o Arduino avançar
        digitalWrite(trigPin, HIGH);
        delay(intervaloCapturaDado);
    }
    
    tft.setTextDatum(CC_DATUM);
    tft.setFreeFont(FSSB9);
    tft.setTextColor(TFT_WHITE,CINZA_ESCURO);
    String msgFinal="Teste Concluido! Erros: "+String(totalError);
    tft.drawString(msgFinal,SCREEN_WIDTH/2,SCREEN_HEIGHT/2);
    
    uint16_t tx, ty;
    while(!tft.getTouch(&tx, &ty)){}
    finalizarTeste();
}

void iniciarTesteManual() {
    estadoAtual=TELA_TESTE_MANUAL;
    mostrarTelaTesteManual();
    Serial.println("\n===== MODO TESTE MANUAL ATIVO =====");
    Serial.println("Pressione 'Voltar' na tela para sair.");
    while(estadoAtual==TELA_TESTE_MANUAL){
        uint16_t tx,ty;
        if(tft.getTouch(&tx,&ty)){
            handleTouch_TelasDeTeste(tx,ty);
            while(tft.getTouch(&tx,&ty)){}
        }
        if(estadoAtual!=TELA_TESTE_MANUAL)break;
        DadosRecebidos dados=recebeDados(false,false);
        Serial.printf("Carga Motor: %.1f %%, Temp: %.1f C\n",dados.conv04,dados.conv05);
    }
}


// ===================================================================================
// SETUP E LOOP PRINCIPAL
// ===================================================================================
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(3);

  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);

  if (!SPIFFS.begin(true)) {
    Serial.println("Falha ao montar SPIFFS.");
    return;
  }
  
  touch_calibrate();

  const char* elmPIN = "1234";
  uint8_t elmPINlen = strlen(elmPIN);
  if (!SerialBT.begin("ESP32_OBD", true)) {
    Serial.println("Falha ao iniciar Bluetooth em modo Master.");
  }
  SerialBT.setPin(elmPIN, elmPINlen);
  
  estadoAtual = TELA_BLUETOOTH;
  mostrarTelaBluetooth();
}

void loop() {
  uint16_t tx, ty;
  if (tft.getTouch(&tx, &ty)) {
    switch (estadoAtual) {
      case TELA_BLUETOOTH:    handleTouch_TelaBluetooth(tx, ty);   break;
      case TELA_MENU_TESTES:  handleTouch_TelaMenuTestes(tx, ty);  break;
      case TELA_TESTE_AUTO:   handleTouch_TelaTesteAuto(tx, ty);   break;
      case TELA_TESTE_MANUAL: handleTouch_TelasDeTeste(tx, ty);    break; 
    }
    while (tft.getTouch(&tx, &ty)) {}
  }
}
#include <TFT_eSPI.h>
#include <SPI.h>
#include "FS.h"
#include "Free_Fonts.h"
#include "BluetoothSerial.h"
#include <vector>

TFT_eSPI tft = TFT_eSPI();
BluetoothSerial SerialBT;

// --- DEFINIÇÕES GLOBAIS ---
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320
#define TABELA_X 10
#define TABELA_WIDTH (SCREEN_WIDTH * 0.6)
#define BOTOES_WIDTH (SCREEN_WIDTH * 0.3)
#define BTN_HEIGHT 50
#define BTN_HEIGHT_LARGE 60
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
#define CINZA_ESCURO TFT_BLACK
#define CINZA_CLARO  TFT_DARKGREY
#define LARANJA      TFT_ORANGE
#define VERDE        TFT_GREEN
#define AZUL         0x34BF
#define VERMELHO     TFT_RED
#define NUM_AMOSTRAS 1000
#define CALIBRATION_FILE "/final_calibration_file"
#define MENUBAR_HEIGHT 40
#define TABLE_Y (MENUBAR_HEIGHT + 5)
#define TABLE_ROW_HEIGHT 54
#define NUM_PIDS 15
#define PIDS_PER_TABLE 5
#define trigPin 5
#define intervaloCapturaDado 250
const int Tx_pin_to_Arduino = 17;

// --- ESTRUTURAS E ESTADOS GLOBAIS ---
enum ScreenState { TELA_BLUETOOTH, TELA_MENU_TESTES, TELA_TESTE_AUTO, TELA_TESTE_MANUAL, TELA_RELATORIO, TELA_SIMULADOR };
ScreenState estadoAtual = TELA_BLUETOOTH;

struct DispositivoBluetooth { String nome; BTAddress* pAddress; };
struct DadosRecebidos {
  float conv04, conv05, conv06, conv07, conv0A, conv0B, conv0C, conv0D, conv0F,
        conv10, conv11, conv14a, conv14b, conv15a, conv15b, conv42;
};
struct ErroDetalhado {
  String nomePid;
  float valorMedido;
  float valorGabarito;
};
std::vector<ErroDetalhado> errosDetalhados;

bool btConectado = false;
bool testIsPaused = false;
int dataIndex = 0;
int totalError = 0;
int paginaAtualRelatorio = 0;
int itensPorPaginaRelatorio = 6;
int totalPaginasRelatorio = 0;

// Arrays de gabarito para o teste automático
float engLoad[11]         = {50, 90, 23, 75, 13, 46, 88, 44, 35, 68, 0};
float EngCoolTemp[11]     = {125, -10, 45, 13, 96, 32, 67, 103, 19, 75, 0};
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
  "Carga Motor", "Temp. Liq.", "STFT", "LTFT", "Press. Comb.",
  "Pres. Coletor", "RPM", "Velocidade", "Temp. Ar Adm.", "Fluxo de Ar",
  "Pos. Borbol.", "Tensao1 O2", "Ajuste1 O2", "Tensao2 O2", "Ajuste2 O2"
};

// ===================================================================================
// SEÇÃO DE FUNÇÕES DE DESENHO E UI
// ===================================================================================
void desenhaBotao(int x, int y, int w, int h, const char* texto, uint16_t cor);
void desenhaPaginacao();
void desenhaListaDispositivos();
void desenhaMenuBarTeste();
void desenhaTabelasBase();
void desenhaMenuBarRelatorio();
void desenhaMenuBarBluetooth();
void desenhaListaErros();
void desenhaPaginacaoRelatorio();
void desenhaRingGauge(int x, int y, int r, const char* rotulo, float valor, float minVal, float maxVal, bool isRpm, uint16_t corValor);
void drawThickArc(int x, int y, int r, int thickness, int start_angle, int sweep_angle, uint16_t color);

void mostrarTelaBluetooth(const char* msgErro = "");
void mostrarTelaMenuTestes();
void mostrarTelaTesteAuto();
void mostrarTelaRelatorio();

void atualizaValoresTabelas(const DadosRecebidos& dados, const float erros[]);

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
void desconectarEVoltarParaBusca();
void finalizarTeste();
void handleTouch_TelaBluetooth(uint16_t x, uint16_t y);
void handleTouch_TelaMenuTestes(uint16_t x, uint16_t y);
void handleTouch_TelaTesteAuto(uint16_t x, uint16_t y);
void handleTouch_TelaTesteManual(uint16_t x, uint16_t y);
void handleTouch_TelaRelatorio(uint16_t x, uint16_t y);
void handleTouch_TelaSimulador(uint16_t x, uint16_t y);
void iniciarTesteAutomatico();
void iniciarTesteManual();
void iniciarModoSimulador();
int lerAdcFiltrado(int pino);


// ===================================================================================
// IMPLEMENTAÇÃO DAS FUNÇÕES
// ===================================================================================

int lerAdcFiltrado(int pino) {
  long acumulador = 0;
  for (int i = 0; i < NUM_AMOSTRAS; i++) {
    acumulador += analogRead(pino);
  }
  return acumulador / NUM_AMOSTRAS;
}

void desconectarEVoltarParaBusca() {
  if (btConectado) {
    SerialBT.disconnect();
  }
  btConectado = false;
  dispositivoSelecionado = -1; // Reseta a seleção
  
  // Limpa a lista de dispositivos encontrados para forçar uma nova busca
  if(dispositivosEncontrados != nullptr){
    for (int i = 0; i < numDispositivosEncontrados; i++) {
      delete dispositivosEncontrados[i].pAddress;
    }
    delete[] dispositivosEncontrados;
    dispositivosEncontrados = nullptr;
    numDispositivosEncontrados = 0;
  }
  
  estadoAtual = TELA_BLUETOOTH;
  mostrarTelaBluetooth(); // Volta para a tela de conexão
}

void desenhaBotao(int x, int y, int w, int h, const char* texto, uint16_t cor) {
  tft.fillRoundRect(x, y, w, h, 8, cor); // Desenha o preenchimento do botão
  tft.drawRoundRect(x, y, w, h, 8, TFT_WHITE); // Desenha a borda branca ao redor do botão
  tft.setTextColor(TFT_WHITE, cor);
  tft.setFreeFont(FSSB9);
  tft.setTextDatum(CC_DATUM);
  tft.drawString(texto, x + w / 2, y + h / 2);
}

void drawThickArc(int x, int y, int r, int thickness, int start_angle, int sweep_angle, uint16_t color) {
  start_angle -= 90; // Converte os ângulos para o sistema de coordenadas da biblioteca (0 é na direita, sentido horário)
  
  // Desenha o arco por partes
  for (int i = start_angle; i < start_angle + sweep_angle; i++) {
    float rad = i * 0.0174532925; // Converte graus para radianos
    float outer_x = x + (r * cos(rad));
    float outer_y = y + (r * sin(rad));
    float inner_x = x + ((r - thickness) * cos(rad));
    float inner_y = y + ((r - thickness) * sin(rad));
    tft.drawLine(outer_x, outer_y, inner_x, inner_y, color);
  }
}

void desenhaRingGauge(int x, int y, int r, const char* rotulo, float valor, float minVal, float maxVal) {
  // 1. Desenha o anel de contorno cinza
  tft.drawCircle(x, y, r, CINZA_CLARO);
  tft.drawCircle(x, y, r - 1, CINZA_CLARO);
  tft.drawCircle(x, y, r - 2, CINZA_CLARO);
  drawThickArc(x, y, r, 3, 0, 360, CINZA_CLARO); // Limpa a área do anel com a cor de fundo

  // 2. Desenha o arco inicial
  int sweepAngle = map(valor, minVal, maxVal, 0, 360);
  drawThickArc(x, y, r, 3, 0, sweepAngle, LARANJA);
  
  // 3. Escreve o rótulo
  tft.setFreeFont(FSS9); 
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(BC_DATUM); 
  tft.drawString(rotulo, x, y - r - 5);
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
  tft.fillScreen(TFT_BLACK);
  desenhaMenuBarBluetooth();

  tft.setFreeFont(FSSB9);
  tft.setTextColor(TFT_WHITE, LARANJA);
  tft.setTextDatum(CC_DATUM);
  tft.drawString("Conectar ao Scanner", SCREEN_WIDTH / 2, MENUBAR_HEIGHT / 2);

  // --- POSIÇÕES ATUALIZADAS PARA 3 BOTÕES ---
  const int NOVO_TABELA_Y = MENUBAR_HEIGHT + 20;
  const int ALTURA_BOTAO = 60;
  const int ESPACO_TOTAL_BOTOES = (ALTURA_BOTAO * 3);
  const int ESPACO_LIVRE = TABELA_HEIGHT - ESPACO_TOTAL_BOTOES;
  const int ESPACO_ENTRE_BOTOES = ESPACO_LIVRE / 2;
  
  const int y_buscar = NOVO_TABELA_Y;
  const int y_conectar = y_buscar + ALTURA_BOTAO + ESPACO_ENTRE_BOTOES;
  const int y_simulador = y_conectar + ALTURA_BOTAO + ESPACO_ENTRE_BOTOES;

  tft.drawRect(TABELA_X, NOVO_TABELA_Y, TABELA_WIDTH, TABELA_HEIGHT, TFT_WHITE);

  if (numDispositivosEncontrados > 0) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("Selecione o scanner abaixo:", TABELA_X + 5, NOVO_TABELA_Y + 5);
  }

  desenhaListaDispositivos();
  desenhaPaginacao();
  
  // --- Desenho dos 3 botões ---
  desenhaBotao(BOTOES_X, y_buscar, BOTOES_WIDTH, ALTURA_BOTAO, "Buscar", LARANJA);
  desenhaBotao(BOTOES_X, y_conectar, BOTOES_WIDTH, ALTURA_BOTAO, "Conectar", CINZA_CLARO);
  desenhaBotao(BOTOES_X, y_simulador, BOTOES_WIDTH, ALTURA_BOTAO, "Modo Simulador", AZUL);
  
  if (strlen(msgErro) > 0) {
    // Calcula a posição Y correta para a mensagem de erro, abaixo do último botão
    const int y_erro = y_simulador + ALTURA_BOTAO + 10; // 10 pixels de margem

    tft.setTextColor(VERMELHO, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    String erroStr(msgErro);
    int newlineIndex = erroStr.indexOf('\n');
    if (newlineIndex != -1) {
      String linha1 = erroStr.substring(0, newlineIndex);
      String linha2 = erroStr.substring(newlineIndex + 1);
      // Usa a nova variável y_erro para posicionar o texto
      tft.drawString(linha1, BOTOES_X, y_erro);
      tft.drawString(linha2, BOTOES_X, y_erro + 25);
    } else {
      // Usa a nova variável y_erro para posicionar o texto
      tft.drawString(erroStr, BOTOES_X, y_erro);
    }
  }
}

void mostrarTelaMenuTestes() {
  tft.fillScreen(CINZA_ESCURO); // Limpa a tela e desenha o cabeçalho laranja
  desenhaMenuBarTeste();

  // Cabeçalho
  tft.setTextColor(TFT_WHITE, LARANJA);
  tft.setTextDatum(CC_DATUM);
  tft.setFreeFont(FSSB9);
  tft.drawString("Menu", SCREEN_WIDTH / 2, MENUBAR_HEIGHT / 2);

  // --- Definições para os novos botões ---
  int btnWidth = SCREEN_WIDTH * 0.6;
  int btnHeight = 60;
  int btnY_auto = MENUBAR_HEIGHT + (SCREEN_HEIGHT - MENUBAR_HEIGHT - (btnHeight * 2 + ESPACAMENTO)) / 2;
  int btnY_manual = btnY_auto + btnHeight + ESPACAMENTO;
  int btnX = (SCREEN_WIDTH - btnWidth) / 2;

  desenhaBotao(btnX, btnY_auto, btnWidth, btnHeight, "Modo automatizado", LARANJA);
  desenhaBotao(btnX, btnY_manual, btnWidth, btnHeight, "Modo manual", LARANJA);
}

void desenhaMenuBarTeste() {
  tft.fillRect(0, 0, SCREEN_WIDTH, MENUBAR_HEIGHT, LARANJA);
  tft.fillTriangle(10, MENUBAR_HEIGHT / 2, 30, 10, 30, MENUBAR_HEIGHT - 10, TFT_BLACK);
}

void desenhaMenuBarBluetooth() {
  tft.fillRect(0, 0, SCREEN_WIDTH, MENUBAR_HEIGHT, LARANJA);
}

void desenhaMenuBarRelatorio() {
  // Desenha a barra laranja do cabeçalho
  tft.fillRect(0, 0, SCREEN_WIDTH, MENUBAR_HEIGHT, LARANJA);
  // Desenha o ícone de seta para voltar (triângulo)
  tft.fillTriangle(10, MENUBAR_HEIGHT / 2, 30, 10, 30, MENUBAR_HEIGHT - 10, TFT_BLACK);
}

void desenhaMenuBarSimulador() {
  // Desenha a barra azul e a seta de voltar
  tft.fillRect(0, 0, SCREEN_WIDTH, MENUBAR_HEIGHT, AZUL);
  tft.fillTriangle(10, MENUBAR_HEIGHT / 2, 30, 10, 30, MENUBAR_HEIGHT - 10, TFT_BLACK);
}

void desenhaTabelasBase() {
  int tableWidth = SCREEN_WIDTH / 3; // Largura de uma das 3 colunas principais

  for (int i = 0; i < 3; i++) {
    int tableX = i * tableWidth;
    int pidColumnWidth = tableWidth * 0.59; 

    // ---- Linhas horizontais da grade ----
    for (int j = 0; j < PIDS_PER_TABLE; j++) {
      int rowY = TABLE_Y + (j * TABLE_ROW_HEIGHT);
      tft.drawFastHLine(tableX, rowY + TABLE_ROW_HEIGHT, tableWidth - 1, CINZA_CLARO);
    }

    // ---- Linhas Verticais ----
    // Linha que separa as 3 colunas principais
    if (i > 0) {
        tft.drawFastVLine(tableX, TABLE_Y, TABLE_ROW_HEIGHT * PIDS_PER_TABLE + 30, CINZA_CLARO);
        tft.drawFastVLine(tableX + 1, TABLE_Y, TABLE_ROW_HEIGHT * PIDS_PER_TABLE + 30, CINZA_CLARO);
    }
  }
}

void desenhaListaErros() {
  int yTabela = MENUBAR_HEIGHT + 50;
  int xNome = 10;
  int xMedido = 200;
  int xEnviado = 330;
  
  tft.fillRect(0, yTabela, SCREEN_WIDTH, SCREEN_HEIGHT - yTabela - PAG_BTN_HEIGHT - 10, CINZA_ESCURO);

  tft.setFreeFont(FSSB9);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(LARANJA);
  tft.drawString("Sensor", xNome, yTabela);
  tft.drawString("Valor Medido", xMedido, yTabela);
  tft.drawString("Valor Enviado", xEnviado, yTabela);
  tft.drawFastHLine(5, yTabela + 25, SCREEN_WIDTH - 10, CINZA_CLARO);

  int indiceInicial = paginaAtualRelatorio * itensPorPaginaRelatorio;
  int indiceFinal = min(indiceInicial + itensPorPaginaRelatorio, (int)errosDetalhados.size());

  tft.setFreeFont(FSS9);
  tft.setTextColor(TFT_WHITE);
  int yItem = yTabela + 35;
  int alturaLinha = 20;

  for (int i = indiceInicial; i < indiceFinal; i++) {
    const auto& erro = errosDetalhados[i];
    int yPosItem = yItem + (i - indiceInicial) * alturaLinha;
    tft.drawString(erro.nomePid, xNome, yPosItem);
    tft.drawString(String(erro.valorMedido, 1), xMedido, yPosItem);
    tft.drawString(String(erro.valorGabarito, 1), xEnviado, yPosItem);
  }
}

void desenhaPaginacaoRelatorio() {
  totalPaginasRelatorio = (errosDetalhados.size() + itensPorPaginaRelatorio - 1) / itensPorPaginaRelatorio;
  if (totalPaginasRelatorio == 0) totalPaginasRelatorio = 1;

  // Posições dos botões na parte inferior
  int pagXPrev = (SCREEN_WIDTH / 2) - PAG_BTN_WIDTH - 60;
  int pagXNext = (SCREEN_WIDTH / 2) + 60;

  // Limpa a área de texto da página antiga
  tft.fillRect(pagXPrev + PAG_BTN_WIDTH, PAG_Y_POS, pagXNext - (pagXPrev + PAG_BTN_WIDTH), PAG_BTN_HEIGHT, CINZA_ESCURO);

  uint16_t corBotaoEsquerda = (paginaAtualRelatorio > 0) ? LARANJA : CINZA_CLARO;
  desenhaBotao(pagXPrev, PAG_Y_POS, PAG_BTN_WIDTH, PAG_BTN_HEIGHT, "<", corBotaoEsquerda);
  
  uint16_t corBotaoDireita = (paginaAtualRelatorio < totalPaginasRelatorio - 1) ? LARANJA : CINZA_CLARO;
  desenhaBotao(pagXNext, PAG_Y_POS, PAG_BTN_WIDTH, PAG_BTN_HEIGHT, ">", corBotaoDireita);
  
  // Desenha o texto da página "n / n"
  String textoPagina = String(paginaAtualRelatorio + 1) + " / " + String(totalPaginasRelatorio);
  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(FSSB9);
  tft.setTextDatum(CC_DATUM);
  tft.drawString(textoPagina, SCREEN_WIDTH / 2, PAG_TEXT_Y);
}

void iniciarModoSimulador() {
    estadoAtual = TELA_SIMULADOR;
    
    static int angulosAnteriores[6] = {0,0,0,0,0,0};

    // --- DESENHO INICIAL DA TELA ---
    tft.fillScreen(TFT_BLACK);
    desenhaMenuBarSimulador(); // Usa a barra de menu azul

    // Adiciona o título "Modo Simulador" no cabeçalho
    tft.setTextColor(TFT_WHITE, AZUL); // Cor do texto sobre o header azul
    tft.setTextDatum(CC_DATUM);
    tft.setFreeFont(FSSB9);
    tft.drawString("Modo Simulador", SCREEN_WIDTH / 2, MENUBAR_HEIGHT / 2);

    const char* pidLabels[] = {"Carga do Motor", "Temp. Arref.", "Press. Combustivel", "Press. Coletor", "RPM", "Velocidade"};
    float minValues[] = {0, -40, 0, 0, 0, 0};
    float maxValues[] = {100, 215, 765, 255, 16384, 255};
    int numColunas = 3;
    int larguraColuna = SCREEN_WIDTH / numColunas;
    int yOffset = 15;
    int alturaLinha = (SCREEN_HEIGHT - MENUBAR_HEIGHT - yOffset) / 2;
    int raioGauge = 40;

    for (int i = 0; i < 6; i++) {
        int linha = i / numColunas;
        int coluna = i % numColunas;
        int centroX = (coluna * larguraColuna) + (larguraColuna / 2);
        int centroY = MENUBAR_HEIGHT + yOffset + (linha * alturaLinha) + (alturaLinha / 2);
        
        desenhaRingGauge(centroX, centroY, raioGauge, pidLabels[i], minValues[i], minValues[i], maxValues[i]);
        atualizaValorTexto(centroX, centroY, minValues[i], TFT_WHITE, (i == 4));
        angulosAnteriores[i] = 0;
    }

    // --- LOOP DE ATUALIZAÇÃO PRINCIPAL ---
    while(estadoAtual == TELA_SIMULADOR){
        uint16_t tx, ty;
        if(tft.getTouch(&tx, &ty)){
            handleTouch_TelaSimulador(tx,ty);
            while(tft.getTouch(&tx, &ty)){}
        }

        if (estadoAtual != TELA_SIMULADOR) {
            break; 
        }

        float valoresEnviados[6];
        valoresEnviados[0] = map(lerAdcFiltrado(pinAdc1), 0, 4095, 0, 100);
        valoresEnviados[1] = map(lerAdcFiltrado(pinAdc2), 0, 4095, -40, 215);
        valoresEnviados[2] = map(lerAdcFiltrado(pinAdc3), 0, 4095, 0, 765);
        valoresEnviados[3] = map(lerAdcFiltrado(pinAdc4), 0, 4095, 0, 255);
        valoresEnviados[4] = map(lerAdcFiltrado(pinAdc5), 0, 4095, 0, 16383);
        valoresEnviados[5] = map(lerAdcFiltrado(pinAdc6), 0, 4095, 0, 255);
        
        String commandToArduino = "1," + String((int)valoresEnviados[0]) + "," + String((int)valoresEnviados[1]) + "," + String((int)valoresEnviados[2]) + "," + String((int)valoresEnviados[3]) + "," + String((int)valoresEnviados[4]) + "," + String((int)valoresEnviados[5]) + "\n";
        Serial2.print(commandToArduino);
        
        for (int i = 0; i < 6; i++) {
            // ======================= CORREÇÃO ABAIXO =======================
            // Recalcula as coordenadas DENTRO deste loop para que elas existam neste escopo
            int linha = i / numColunas;
            int coluna = i % numColunas;
            int centroX = (coluna * larguraColuna) + (larguraColuna / 2);
            int centroY = MENUBAR_HEIGHT + yOffset + (linha * alturaLinha) + (alturaLinha / 2);
            // ===============================================================

            atualizaValorTexto(centroX, centroY, valoresEnviados[i], TFT_WHITE, (i == 4));

            int anguloNovo = map(valoresEnviados[i], minValues[i], maxValues[i], 0, 360);
            
            if (anguloNovo > angulosAnteriores[i]) {
                drawThickArc(centroX, centroY, raioGauge, 3, angulosAnteriores[i], anguloNovo - angulosAnteriores[i], AZUL);
            } else if (anguloNovo < angulosAnteriores[i]) {
                drawThickArc(centroX, centroY, raioGauge, 3, anguloNovo, angulosAnteriores[i] - anguloNovo, CINZA_CLARO);
            }
            
            angulosAnteriores[i] = anguloNovo;
        }
        delay(50);
    }
}

void atualizaValoresTabelas(const DadosRecebidos& dados, const float erros[]) {
  int tableWidth = SCREEN_WIDTH / 3;
  
  float valoresLidos[] = { dados.conv04, dados.conv05, dados.conv06, dados.conv07, dados.conv0A, dados.conv0B, dados.conv0C, dados.conv0D, dados.conv0F, dados.conv10, dados.conv11, dados.conv14a, dados.conv14b, dados.conv15a, dados.conv15b };

  for (int i = 0; i < 3; i++) {
    int tableX = i * tableWidth;
    int pidColumnWidth = tableWidth * 0.59;

    for (int j = 0; j < PIDS_PER_TABLE; j++) {
      int rowY = TABLE_Y + (j * TABLE_ROW_HEIGHT);
      int pidIndex = i * PIDS_PER_TABLE + j;
      if (pidIndex < NUM_PIDS) {
        
        // ---- 1. Limpa e Redesenha o Nome do PID ----
        tft.setTextFont(2);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(TL_DATUM);
        // Limpa a área do nome
        tft.fillRect(tableX + 2, rowY, pidColumnWidth - 4, TABLE_ROW_HEIGHT - 6, CINZA_ESCURO);
        // Redesenha o nome
        tft.drawString(pidNames[pidIndex], tableX + 5, rowY + 12);
        
        // ---- 2. Limpa e Redesenha o Valor ----
        tft.setTextDatum(TL_DATUM);
        // Limpa a área do valor
        tft.fillRect(tableX + pidColumnWidth + 2, rowY, (tableWidth - pidColumnWidth - 4), TABLE_ROW_HEIGHT - 6, CINZA_ESCURO);
        // Define a cor
        uint16_t corValor = (erros[pidIndex] > 0.1) ? VERMELHO : VERDE;
        tft.setTextColor(corValor);
        // Redesenha o valor
        String valorStr = String(valoresLidos[pidIndex], 1);
        tft.drawString(valorStr, tableX + pidColumnWidth + 5, rowY + 12);
      }
    }
  }
  
  tft.setFreeFont(nullptr); 
}

void atualizaValorTexto(int x, int y, float valor, uint16_t cor, bool isRpm) {
  char valorStr[7];
  if (isRpm) {
    sprintf(valorStr, "%.0f", valor);
  } else {
    sprintf(valorStr, "%.0f", valor);
  }

  tft.setFreeFont(FSSB12);
  tft.setTextColor(cor, CINZA_ESCURO); // Define a cor do texto e do fundo
  tft.setTextDatum(CC_DATUM);

  // Apaga a área antiga desenhando um retângulo com a cor de fundo
  tft.fillRect(x - 35, y - 10, 70, 20, CINZA_ESCURO);
  
  // Desenha o novo valor na área limpa
  tft.drawString(valorStr, x, y);
}

void atualizaRingGauges(const DadosRecebidos& dados) {
    // Parâmetros dos gauges
    int numLinhas = 2;
    int numColunas = 3;
    int larguraColuna = SCREEN_WIDTH / numColunas;
    int yOffset = 15;
    int alturaLinha = (SCREEN_HEIGHT - MENUBAR_HEIGHT - yOffset) / numLinhas;
    int raioGauge = 40;

    // Rótulos e faixas de valores para cada PID
    const char* pidLabels[] = { "Carga do Motor", "Temp. Arref.", "Press. Combustivel", "Press. Coletor", "RPM", "Velocidade" };
    float minValues[] = {0, -40, 0, 0, 0, 0};
    float maxValues[] = {100, 215, 765, 255, 16384, 255};

    // Array com os valores recebidos do ELM327
    float pidValues[] = { dados.conv04, dados.conv05, dados.conv0A, dados.conv0B, dados.conv0C, dados.conv0D };

    for (int i = 0; i < numLinhas; i++) {
        for (int j = 0; j < numColunas; j++) {
            int pidIndex = i * numColunas + j;
            int centroX = (j * larguraColuna) + (larguraColuna / 2);
            int centroY = MENUBAR_HEIGHT + yOffset + (i * alturaLinha) + (alturaLinha / 2);

            // Redesenha o gauge com o novo valor
            desenhaRingGauge(centroX, centroY, raioGauge, pidLabels[pidIndex],
                 pidValues[pidIndex], minValues[pidIndex], maxValues[pidIndex],
                 (pidIndex == 4), TFT_WHITE);
        }
    }
}

void mostrarTelaTesteAuto() {
  tft.fillScreen(CINZA_ESCURO);
  desenhaMenuBarTeste();
  tft.setTextColor(TFT_WHITE, LARANJA);
  tft.setTextDatum(CC_DATUM);
  tft.setFreeFont(FSSB9);
  tft.drawString("Modo Automatizado", SCREEN_WIDTH / 2, MENUBAR_HEIGHT / 2);
  tft.fillRect(0, 0, 40, MENUBAR_HEIGHT, LARANJA);
  desenhaTabelasBase();
}

void mostrarTelaRelatorio() {
  tft.fillScreen(CINZA_ESCURO);
  desenhaMenuBarRelatorio();
  tft.setTextColor(TFT_WHITE, LARANJA);
  tft.setTextDatum(CC_DATUM);
  tft.setFreeFont(FSSB9);
  tft.drawString("Resultados", SCREEN_WIDTH / 2, MENUBAR_HEIGHT / 2);
  
  int yOffset = MENUBAR_HEIGHT + 15; 
  
  tft.setFreeFont(FSSB9);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_WHITE);

  if (totalError == 0) {
    // Define o ponto de referência como Centro-Centro para o alinhamento
    tft.setTextDatum(CC_DATUM);
    // Desenha a mensagem no centro exato da tela
    tft.drawString("Teste finalizado! Total de 0 erros.", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
  } else {
    // Mantém o alinhamento antigo (Topo-Centro) para a tela com a lista de erros
    tft.setTextDatum(TC_DATUM);
    String msg = "Teste finalizado! Erros: " + String(totalError);
    tft.drawString(msg, SCREEN_WIDTH / 2, yOffset);
    
    desenhaListaErros();
    desenhaPaginacaoRelatorio();
  }
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
  // Posições e tamanhos dos botões
  const int NOVO_BOTOES_Y = MENUBAR_HEIGHT + 20;
  const int ALTURA_BOTAO = 60;

  // Etapa 2: Botão muda para "Buscando..." e fica cinza
  desenhaBotao(BOTOES_X, NOVO_BOTOES_Y, BOTOES_WIDTH, ALTURA_BOTAO, "Buscando...", CINZA_CLARO);

  // Limpa a lista de dispositivos anterior
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

  // Busca por dispositivos
  BTScanResults* results = SerialBT.discover(5000); // 5 segundos de busca
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
  
  // Etapa 3: Redesenha a tela inteira para atualizar tudo
  mostrarTelaBluetooth(); // Redesenha a tela base
  
  // Etapa 3 (continuação): Botão "Buscar" volta ao texto original, mas cinza
  desenhaBotao(BOTOES_X, NOVO_BOTOES_Y, BOTOES_WIDTH, ALTURA_BOTAO, "Buscar", CINZA_CLARO);
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
        SerialBT.print("01 0F\r"); r = readELMResponse(); if(r!="NO DATA") dados.conv0F = strtol(r.substring(4).c_str(), NULL, 16) - 40;
        SerialBT.print("01 10\r"); r = readELMResponse(); if(r!="NO DATA") { uint16_t val = strtoul(r.substring(4).c_str(),NULL,16); dados.conv10 = val / 100.0; }
        SerialBT.print("01 11\r"); r = readELMResponse(); if(r!="NO DATA") dados.conv11 = strtoul(r.substring(4).c_str(),NULL,16) * 100.0/255.0;
        SerialBT.print("01 14\r"); r = readELMResponse(); if(r!="NO DATA"){uint16_t o2=strtoul(r.substring(4).c_str(),NULL,16);dados.conv14a=(o2>>8)*0.005;dados.conv14b=((o2&0xFF)*100.0/128.0)-100;}
        SerialBT.print("01 15\r"); r = readELMResponse(); if(r!="NO DATA"){uint16_t o2=strtoul(r.substring(4).c_str(),NULL,16);dados.conv15a=(o2>>8)*0.005;dados.conv15b=((o2&0xFF)*100.0/128.0)-100;}
        SerialBT.print("01 42\r"); r = readELMResponse(); if(r!="NO DATA") dados.conv42 = strtoul(r.substring(4).c_str(),NULL,16)/1000.0;
    }

    SerialBT.print("01 04\r"); String r4 = readELMResponse(); if (r4 != "NO DATA") dados.conv04 = strtoul(r4.substring(4).c_str(), NULL, 16) * 100.0 / 255.0;
    SerialBT.print("01 05\r"); String r5 = readELMResponse(); if (r5 != "NO DATA") dados.conv05 = strtol(r5.substring(4).c_str(), NULL, 16) - 40;
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
  
  const char* pidNamesReport[16];
  for(int i=0; i<15; i++) pidNamesReport[i] = pidNames[i];
  pidNamesReport[15] = "Tensao Modulo";

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
      // Salva os detalhes do erro encontrado
      ErroDetalhado erroInfo;
      erroInfo.nomePid = pidNamesReport[i];
      erroInfo.valorMedido = valoresLidos[i];
      erroInfo.valorGabarito = valoresGabarito[i];
      errosDetalhados.push_back(erroInfo);
    }
  }
  Serial.printf("Passo %d - Erros neste passo: %d\n", dataIndex + 1, dataError);
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
    mostrarTelaBluetooth("Selecione um\ndispositivo");
    return;
  }

  // Calcula a posição correta do botão do meio no layout de 3 botões
  const int NOVO_TABELA_Y = MENUBAR_HEIGHT + 20;
  const int ALTURA_BOTAO = 60;
  const int ESPACO_TOTAL_BOTOES = (ALTURA_BOTAO * 3);
  const int ESPACO_LIVRE = TABELA_HEIGHT - ESPACO_TOTAL_BOTOES;
  const int ESPACO_ENTRE_BOTOES = ESPACO_LIVRE / 2;
  const int y_conectar = NOVO_TABELA_Y + ALTURA_BOTAO + ESPACO_ENTRE_BOTOES;

  // Usa as novas constantes para desenhar o botão "Conectando..." na posição correta
  desenhaBotao(BOTOES_X, y_conectar, BOTOES_WIDTH, ALTURA_BOTAO, "Conectando...", CINZA_CLARO);

  DispositivoBluetooth* dev = &dispositivosEncontrados[dispositivoSelecionado];
  btConectado = SerialBT.connect(*dev->pAddress);

  if (btConectado) {
    initializeELM();
    estadoAtual = TELA_MENU_TESTES;
    mostrarTelaMenuTestes();
  } else {
    // Redesenha a tela para mostrar a falha e reativar os botões
    mostrarTelaBluetooth("Falha ao conectar");
    // Reativa o botão conectar em laranja, já que a conexão falhou
    desenhaBotao(BOTOES_X, y_conectar, BOTOES_WIDTH, ALTURA_BOTAO, "Conectar", LARANJA);
  }
}

void finalizarTeste() {
  Serial2.print("0,0,0,0,0,0,0\n");
  digitalWrite(trigPin, LOW); 
  dataIndex = 0;
  totalError = 0;
  testIsPaused = false;
  estadoAtual = TELA_MENU_TESTES;
  mostrarTelaMenuTestes();
}

void handleTouch_TelaBluetooth(uint16_t x, uint16_t y) {
  // Posições e tamanhos atualizados para 3 botões
  const int NOVO_TABELA_Y = MENUBAR_HEIGHT + 20;
  const int ALTURA_BOTAO = 60;
  const int ESPACO_TOTAL_BOTOES = (ALTURA_BOTAO * 3);
  const int ESPACO_LIVRE = TABELA_HEIGHT - ESPACO_TOTAL_BOTOES;
  const int ESPACO_ENTRE_BOTOES = ESPACO_LIVRE / 2;
  const int y_buscar = NOVO_TABELA_Y;
  const int y_conectar = y_buscar + ALTURA_BOTAO + ESPACO_ENTRE_BOTOES;
  const int y_simulador = y_conectar + ALTURA_BOTAO + ESPACO_ENTRE_BOTOES;

  // Toque no botão "Buscar"
  if ((x > BOTOES_X) && (x < (BOTOES_X + BOTOES_WIDTH)) && (y > y_buscar) && (y < (y_buscar + ALTURA_BOTAO))) {
    procurarDispositivos();
  } 
  // Toque no botão "Conectar"
  else if ((x > BOTOES_X) && (x < (BOTOES_X + BOTOES_WIDTH)) && (y > y_conectar) && (y < (y_conectar + ALTURA_BOTAO))) {
    if (dispositivoSelecionado != -1) {
        conectarDispositivoSelecionado();
    }
  }
  // Toque no botão "Modo Simulador"
  else if ((x > BOTOES_X) && (x < (BOTOES_X + BOTOES_WIDTH)) && (y > y_simulador) && (y < (y_simulador + ALTURA_BOTAO))) {
    iniciarModoSimulador(); // Chama a nova função
  }
  // Toque na paginação
  else if ((x > PAG_BTN_PREV_X) && (x < (PAG_BTN_PREV_X + PAG_BTN_WIDTH)) && (y > PAG_Y_POS) && (y < (PAG_Y_POS + PAG_BTN_HEIGHT))) {
    if (paginaAtual > 0) { paginaAtual--; desenhaListaDispositivos(); desenhaPaginacao(); }
  } else if ((x > PAG_BTN_NEXT_X) && (x < (PAG_BTN_NEXT_X + PAG_BTN_WIDTH)) && (y > PAG_Y_POS) && (y < (PAG_Y_POS + PAG_BTN_HEIGHT))) {
    if (paginaAtual < totalPaginas - 1) { paginaAtual++; desenhaListaDispositivos(); desenhaPaginacao(); }
  } 
  // Toque na lista de dispositivos
  else if ((x > TABELA_X) && (x < (TABELA_X + TABELA_WIDTH)) && (y > NOVO_TABELA_Y) && (y < (PAG_Y_POS - 5))) {
    int listaStartY = NOVO_TABELA_Y + 25;
    int alturaDisponivel = TABELA_HEIGHT - 30;
    int alturaLinha = alturaDisponivel / itensPorPagina;
    int itemClicadoNaPagina = (y - listaStartY) / alturaLinha;
    int novoIndiceSelecionado = (paginaAtual * itensPorPagina) + itemClicadoNaPagina;

    if (novoIndiceSelecionado < numDispositivosEncontrados) {
      dispositivoSelecionado = (dispositivoSelecionado == novoIndiceSelecionado) ? -1 : novoIndiceSelecionado;
      desenhaListaDispositivos();

      if (dispositivoSelecionado != -1) {
        desenhaBotao(BOTOES_X, y_conectar, BOTOES_WIDTH, ALTURA_BOTAO, "Conectar", LARANJA);
      } else {
        desenhaBotao(BOTOES_X, y_conectar, BOTOES_WIDTH, ALTURA_BOTAO, "Conectar", CINZA_CLARO);
      }
    }
  }
}

void handleTouch_TelaMenuTestes(uint16_t x, uint16_t y) {
    // Adiciona a verificação para o botão voltar
    if (x < 60 && y < MENUBAR_HEIGHT) {
      desconectarEVoltarParaBusca();
      return; // Sai da função para não verificar os outros botões
    }

    // A lógica para os botões de modo de teste continua a mesma
    int btnWidth = SCREEN_WIDTH * 0.6;
    int btnHeight = 60;
    int btnY_auto = MENUBAR_HEIGHT + (SCREEN_HEIGHT - MENUBAR_HEIGHT - (btnHeight * 2 + ESPACAMENTO)) / 2;
    int btnY_manual = btnY_auto + btnHeight + ESPACAMENTO;
    int btnX = (SCREEN_WIDTH - btnWidth) / 2;
    
    if ((x > btnX) && (x < btnX + btnWidth) && (y > btnY_auto) && (y < btnY_auto + btnHeight)) {
        iniciarTesteAutomatico();
    } else if ((x > btnX) && (x < btnX + btnWidth) && (y > btnY_manual) && (y < btnY_manual + btnHeight)) {
        iniciarTesteManual();
    }
}

void handleTouch_TelaTesteAuto(uint16_t x, uint16_t y) {
  // Nenhuma interação nessa tela
}

void handleTouch_TelaRelatorio(uint16_t x, uint16_t y) {
  // 1. Verifica toque no botão de voltar (seta)
  if (x < 60 && y < MENUBAR_HEIGHT) {
    finalizarTeste();
    return;
  }

  if (totalError == 0) return;

  // 2. Verifica toque nos botões de paginação
  int pagXPrev = (SCREEN_WIDTH / 2) - PAG_BTN_WIDTH - 60;
  int pagXNext = (SCREEN_WIDTH / 2) + 60;

  if ((x > pagXPrev) && (x < (pagXPrev + PAG_BTN_WIDTH)) && (y > PAG_Y_POS) && (y < (PAG_Y_POS + PAG_BTN_HEIGHT))) {
    if (paginaAtualRelatorio > 0) {
      paginaAtualRelatorio--;
      desenhaListaErros();
      desenhaPaginacaoRelatorio();
    }
  } else if ((x > pagXNext) && (x < (pagXNext + PAG_BTN_WIDTH)) && (y > PAG_Y_POS) && (y < (PAG_Y_POS + PAG_BTN_HEIGHT))) {
    if (paginaAtualRelatorio < totalPaginasRelatorio - 1) {
      paginaAtualRelatorio++;
      desenhaListaErros();
      desenhaPaginacaoRelatorio();
    }
  }
}

void handleTouch_TelaTesteManual(uint16_t x, uint16_t y) {
  // Verifica o toque apenas no botão de voltar
  if (x < 60 && y < MENUBAR_HEIGHT) {
    finalizarTeste();
  }
}

void handleTouch_TelaSimulador(uint16_t x, uint16_t y) {
  // Verifica o toque apenas no botão de voltar (mesma área dos outros)
  if (x < 60 && y < MENUBAR_HEIGHT) {
    // Ao voltar, reseta o estado para a tela Bluetooth
    estadoAtual = TELA_BLUETOOTH;
    mostrarTelaBluetooth();
  }
}

void iniciarTesteAutomatico() {
    estadoAtual = TELA_TESTE_AUTO;
    testIsPaused = false;
    dataIndex = 0;
    totalError = 0;
    errosDetalhados.clear();
    paginaAtualRelatorio = 0;
    
    digitalWrite(trigPin, LOW);
    delay(100);

    mostrarTelaTesteAuto();
    initLeitura();
    
    for(int i=0; i<10; i++){
        dataIndex = i;
        if(estadoAtual!=TELA_TESTE_AUTO)return;
    
        digitalWrite(trigPin, LOW);
        
        DadosRecebidos dados=recebeDados(true,true);
        calculaErro(dados);
        atualizaValoresTabelas(dados,erro);

        digitalWrite(trigPin, HIGH);
        delay(intervaloCapturaDado);
    }
    
    digitalWrite(trigPin, LOW);
    
    estadoAtual = TELA_RELATORIO;
    mostrarTelaRelatorio();
}

void iniciarTesteManual() {
    estadoAtual = TELA_TESTE_MANUAL;

    static int angulosAnteriores[6] = {0,0,0,0,0,0};

    // --- PARÂMETROS E DESENHO INICIAL ---
    tft.fillScreen(CINZA_ESCURO);
    desenhaMenuBarTeste();
    tft.setTextColor(TFT_WHITE, LARANJA);
    tft.setTextDatum(CC_DATUM);
    tft.setFreeFont(FSSB9);
    tft.drawString("Modo Manual", SCREEN_WIDTH / 2, MENUBAR_HEIGHT / 2);

    const char* pidLabels[] = {"Carga do Motor", "Temp. Arref.", "Press. Combustivel", "Press. Coletor", "RPM", "Velocidade"};
    float minValues[] = {0, -40, 0, 0, 0, 0};
    float maxValues[] = {100, 215, 765, 255, 16384, 255};
    int numColunas = 3;
    int larguraColuna = SCREEN_WIDTH / numColunas;
    int yOffset = 15;
    int alturaLinha = (SCREEN_HEIGHT - MENUBAR_HEIGHT - yOffset) / 2;
    int raioGauge = 40;

    for (int i = 0; i < 6; i++) {
        int linha = i / numColunas;
        int coluna = i % numColunas;
        int centroX = (coluna * larguraColuna) + (larguraColuna / 2);
        int centroY = MENUBAR_HEIGHT + yOffset + (linha * alturaLinha) + (alturaLinha / 2);
        
        // Desenha o gauge estático usando o valor mínimo correto para cada PID
        desenhaRingGauge(centroX, centroY, raioGauge, pidLabels[i], minValues[i], minValues[i], maxValues[i]);
        atualizaValorTexto(centroX, centroY, minValues[i], TFT_WHITE, (i == 4));
        angulosAnteriores[i] = 0; // Reseta o estado (o ângulo para o valor mínimo é sempre 0)
    }

    // --- LOOP DE ATUALIZAÇÃO PRINCIPAL ---
    while(estadoAtual == TELA_TESTE_MANUAL){
        uint16_t tx, ty;
        if(tft.getTouch(&tx,&ty)){
            handleTouch_TelaTesteManual(tx,ty);
            while(tft.getTouch(&tx,&ty)){}
        }

        if (estadoAtual != TELA_TESTE_MANUAL) {
            break;
        }

        // Leitura de dados (semelhante a antes)
        float valoresEnviados[6];
        valoresEnviados[0] = map(lerAdcFiltrado(pinAdc1), 0, 4095, 0, 100);
        valoresEnviados[1] = map(lerAdcFiltrado(pinAdc2), 0, 4095, -40, 215);
        valoresEnviados[2] = map(lerAdcFiltrado(pinAdc3), 0, 4095, 0, 765);
        valoresEnviados[3] = map(lerAdcFiltrado(pinAdc4), 0, 4095, 0, 255);
        valoresEnviados[4] = map(lerAdcFiltrado(pinAdc5), 0, 4095, 0, 16383);
        valoresEnviados[5] = map(lerAdcFiltrado(pinAdc6), 0, 4095, 0, 255);
        String commandToArduino = "1," + String((int)valoresEnviados[0]) + "," + String((int)valoresEnviados[1]) + "," + String((int)valoresEnviados[2]) + "," + String((int)valoresEnviados[3]) + "," + String((int)valoresEnviados[4]) + "," + String((int)valoresEnviados[5]) + "\n";
        Serial2.print(commandToArduino);
        DadosRecebidos dados = recebeDados(false, false);
        float valoresLidos[] = {dados.conv04, dados.conv05, dados.conv0A, dados.conv0B, dados.conv0C, dados.conv0D};

        //valoresEnviados[0] = valoresEnviados[0] * 0.7; // APAGAR (forçando erro para validar se o texto fica vermelho no teste manual)

        // --- LÓGICA DE ATUALIZAÇÃO INCREMENTAL ---
        for (int i = 0; i < 6; i++) {
            // Calcula a posição do gauge
            int linha = i / numColunas;
            int coluna = i % numColunas;
            int centroX = (coluna * larguraColuna) + (larguraColuna / 2);
            int centroY = MENUBAR_HEIGHT + yOffset + (linha * alturaLinha) + (alturaLinha / 2);

            // Calcula erro e cor do texto
            float erro = 0;
            if (abs(valoresEnviados[i]) > 0.01) erro = abs((valoresLidos[i] - valoresEnviados[i]) / valoresEnviados[i]);
            else erro = (abs(valoresLidos[i]) > 0.01) ? 1.0 : 0.0;
            uint16_t corDoValor = (erro > 0.1) ? VERMELHO : VERDE;

            // 1. Atualiza o texto (anti-flicker)
            atualizaValorTexto(centroX, centroY, valoresLidos[i], corDoValor, (i == 4));

            // 2. Lógica incremental para o anel
            int anguloNovo = map(valoresLidos[i], minValues[i], maxValues[i], 0, 360);
            if (anguloNovo > angulosAnteriores[i]) {
                // Valor aumentou: desenha apenas o novo segmento em laranja
                drawThickArc(centroX, centroY, raioGauge, 3, angulosAnteriores[i], anguloNovo - angulosAnteriores[i], LARANJA);
            } else if (anguloNovo < angulosAnteriores[i]) {
                // Valor diminuiu: apaga o segmento antigo com a cor de fundo
                drawThickArc(centroX, centroY, raioGauge, 3, anguloNovo, angulosAnteriores[i] - anguloNovo, CINZA_CLARO);
            }
            // Se o ângulo for igual, não faz nada no anel.

            // 3. Guarda o ângulo atual para a próxima iteração
            angulosAnteriores[i] = anguloNovo;
        }
        delay(50); // Reduzimos o delay para uma resposta mais fluida
    }
}


// ===================================================================================
// SETUP E LOOP PRINCIPAL
// ===================================================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, -1, Tx_pin_to_Arduino);

  // Correção da tela de inicialização
  // const int pinoBacklight = 16; // Pino do LED do backlight
  // pinMode(pinoBacklight, OUTPUT);
  // digitalWrite(pinoBacklight, LOW); // Garante que a luz comece apagada

  tft.init();
  tft.setRotation(3);

  // Correção da tela de inicialização
  // tft.fillScreen(CINZA_ESCURO);
  // delay(50);
  // digitalWrite(pinoBacklight, HIGH);


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
      case TELA_TESTE_MANUAL: handleTouch_TelaTesteManual(tx, ty); break;
      case TELA_RELATORIO:    handleTouch_TelaRelatorio(tx, ty);   break;
      case TELA_SIMULADOR:    handleTouch_TelaSimulador(tx, ty);     break;
    }
    while (tft.getTouch(&tx, &ty)) {}
  }
}
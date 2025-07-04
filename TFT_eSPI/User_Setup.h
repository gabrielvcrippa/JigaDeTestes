// ===================================================================================
// ARQUIVO DE CONFIGURAÇÃO PARA TFT_eSPI
// Display: 480x320 (ILI9488) com Touch (XPT2046)
// ===================================================================================

#define USER_SETUP_INFO "ESP32_480x320_ILI9488_XPT2046"

// --- SEÇÃO 1: DRIVER DO DISPLAY ---
#define ILI9488_DRIVER


// --- SEÇÃO 2: PINOS DE CONEXÃO (ESP32) ---
#define TFT_CS      15  // Chip Select do Display
#define TFT_DC      2   // Data/Command (também chamado de RS)
#define TFT_RST     4   // Pino de Reset do Display
#define TFT_BL      16  // Pino de controle do Backlight (LED)

#define TFT_MOSI    23  // SPI Master Out, Slave In (envio de dados para o display)
#define TFT_SCLK    18  // SPI Serial Clock (sinal de clock do barramento)
#define TFT_MISO    19  // SPI Master In, Slave Out (leitura de dados do display)

#define TOUCH_CS    21  // Chip Select do Touchscreen (controlador XPT2046)

#define TFT_BACKLIGHT_ON HIGH // Nível para ligar o backlight (pode ser LOW dependendo da placa)


// --- SEÇÃO 3: FONTES ---
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT


// --- SEÇÃO 4: CONFIGURAÇÕES DO BARRAMENTO SPI ---
#define SPI_FREQUENCY       27000000 //55000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY 2500000 // Frequência mais baixa, ideal para o XPT2046
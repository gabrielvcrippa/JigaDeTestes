# Jiga de Testes OBD-II com ESP32 e Arduino

## Descrição Geral

Este projeto consiste em uma jiga de testes eletrônica projetada para validar o funcionamento de scanners automotivos padrão OBD-II. O sistema simula uma Unidade de Controle Eletrônico (ECU) de um veículo, gerando um conjunto de dados de diagnóstico (PIDs) que são transmitidos através de um barramento CAN.

A arquitetura é composta por três elementos principais:
1.  Um **ESP32** com um display TFT touchscreen, que atua como o cérebro do sistema e a interface principal para o usuário.
2.  Um **Arduino Nano**, que funciona como um simulador de ECU, gerando os dados dos sensores do veículo.
3.  Um scanner **ELM327**, que atua como ponte, lendo os dados do barramento CAN e os retransmitindo via Bluetooth para o ESP32.

O objetivo da jiga é permitir que o ESP32 compare os dados que ele espera que a ECU envie com os dados que ele de fato recebe de volta do scanner, validando toda a cadeia de comunicação.

## Arquitetura e Componentes

### 1. ESP32 - Firmware Principal (`00_main_interface`)

O ESP32 é o componente central do projeto, responsável por toda a lógica de controle e interação com o usuário. Suas principais funções são:

* **Gerenciamento da Interface Gráfica (UI):** Controla todas as telas do sistema no display TFT, incluindo a conexão Bluetooth, menu de seleção, telas de teste e relatórios.
* **Comunicação Bluetooth:** Gerencia a busca, seleção e conexão com o scanner ELM327, enviando requisições de PIDs e recebendo as respostas.
* **Modos de Operação:** Coordena três rotinas de teste distintas: manual, automatizada e simulação.
* **Modo Automatizado:** Compara os dados recebidos do scanner com um conjunto de valores pré-definidos (gabarito) e, ao final, gera um relatório detalhado de erros.
* **Modo Manual:** Permite ao usuário controlar em tempo real os valores de 6 PIDs através de potenciômetros. Neste modo, o ESP32 permanece conectado ao ELM327 para receber os dados de volta, compará-los com os valores enviados e exibir o resultado com um feedback de erro (verde/vermelho) na tela. O objetivo é validar todo o ciclo de comunicação da jiga.
* **Modo Simulação:** Permite ao usuário controlar em tempo real os valores de 6 PIDs através de potenciômetros. Neste modo, o Bluetooth do ESP32 permanece inativo e não se conecta ao ELM327. A tela da jiga serve apenas como um "painel de controle", mostrando os valores que estão sendo enviados ao Arduino, para que um dispositivo externo (como um celular) possa se conectar ao ELM327 e ser testado.
* **Comunicação Serial com Arduino:** No modo manual, o ESP32 envia os valores lidos dos potenciômetros para o Arduino via comunicação Serial (`Serial2`) para que a ECU simulada utilize esses dados.

### 2. Arduino - Simulador de ECU

O firmware do Arduino tem a única função de atuar como um simulador de ECU. Ele está conectado ao barramento CAN e é responsável por:

* **Geração de Dados CAN:** Criar e enviar mensagens no padrão CAN contendo as respostas para as solicitações de PIDs do OBD-II.
* **Operação:** Ele pode operar de três formas:
    * **Modo Automático:** Percorre um conjunto de dados pré-programados, alterando os valores dos PIDs em um ciclo definido.
    * **Modo Manual:** Recebe comandos do ESP32 via Serial e utiliza os valores fornecidos para gerar as respostas CAN, permitindo o controle em tempo real.
    * **Modo Simulador:** Atua de forma idêntica ao Modo Manual, recebendo comandos do ESP32 via Serial e gerando as respostas CAN. A diferença é que o destinatário final da informação (após passar pelo ELM327) é um aplicativo de scanner externo, e não a própria jiga.

### 3. Biblioteca TFT_eSPI (Modificada)

A interação com o display é gerenciada pela biblioteca `TFT_eSPI`, que é fundamental para desenhar todos os elementos gráficos e capturar os eventos de toque.

É importante notar que a biblioteca foi modificada para se adequar às necessidades específicas de calibração do touch screen utilizado neste projeto, com um arquivo de calibração (`CALIBRATION_FILE`) sendo gerado e lido para garantir a precisão do toque.

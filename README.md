# CarrinhoSeguidorPacheco
seguidor de linha liga lari time: pachecoins
ganhador Corrida 3º competição liga lari
o carrinho tem um codigo de musica que esta comentado















1.1. Inclusões e Dependências

    1.1.1. Inclusões de Bibliotecas (Linhas 1-3)

        Nota: Verifica se todas as bibliotecas são estritamente necessárias.

    1.1.2. Mapeamento de Pinos e Constantes (#define) (Linhas 27-37)

        Define a interface de hardware (Motores, LED, Botão).

1.2. Variáveis de Estado e Parâmetros

    1.2.1. Variáveis de Estado e Contadores (Linhas 5-13)

        Controle de parada, linha, e LED.

    1.2.2. Constantes de Controle PID (Linhas 14-19)

        Termos Kp​, Ki​, Kd​, Kr​ e variáveis de erro.

    1.2.3. Configurações de Velocidade (Linhas 20-23)

        Velocidades Máxima e Base dos motores A e B.

Capítulo 2. Inicialização do Sistema (setup)

2.1. Funções de Setup e Protótipos

    2.1.1. Prototipagem das Funções (Linhas 40-43)

    2.1.2. Início das Comunicações (Serial e Bluetooth) (Linhas 45-46)

2.2. Configuração de Hardware

    2.2.1. Setup dos Canais PWM (Motores e LED) (Linhas 48-52)

    2.2.2. Configuração e Inicialização dos Sensores QTR (Linhas 57-60)

    2.2.3. Anexação de Pinos PWM e Definição de Modos (Linhas 62-73)

2.3. Multitarefa (FreeRTOS)

    2.3.1. Criação da tarefaRGB (Core 1) (Linhas 76-83)

        Permite animações de LED não-bloqueantes.

Capítulo 3. Rotina Principal de Controle (loop)

3.1. Gerenciamento de Estado

    3.1.1. Lógica de Controle do LED (Bluetooth/Calibração) (Linhas 86-104)

    3.1.2. Botão de Calibração (Linhas 107-109)

    3.1.3. Botão Liga/Desliga (Linhas 147-164)

3.2. Processamento de Comandos Bluetooth

    3.2.1. Leitura de Comandos (Linhas 112-114)

    3.2.2. Troca de Modos (Manual/PID) (Linhas 116-126)

    3.2.3. Atualização de Constantes PID e Velocidades (Linhas 128-146)

    3.2.4. Controle em Modo Manual (Linhas 134-177)

        Direção e tempo de ação (Del).

3.3. Execução do Controle (Ligar/Desligar)

    3.3.1. Ativação do controle_PID() (Linhas 171-177)

        Execução apenas se onOff = true e manualPid = 0.

Capítulo 4. Funções de Controle e Lógica Central

4.1. Controle de Motor (Baixo Nível)

    4.1.1. Função controleMotores() (Linhas 240-271)

        Traduz velocidade (sinal e magnitude) para direção (GPIO) e PWM.

4.2. Lógica PID (Proporcional, Integral, Derivativo)

    4.2.1. Função errosPassados() (Linhas 275-279)

        Atualiza o histórico de erros (para I e D).

    4.2.2. Função errosSomatorio() (Linhas 282-286)

        Cálculo do somatório de erros (termo I).

    4.2.3. Função controle_PID() (Linhas 287-313)

        Leitura, Cálculo do Erro, PID, Lógica de Velocidade e Controle de Saída de Linha.

        Otimização: Revisar a lógica de saída de linha (ValorMaximoSensores <= 700).

4.3. Funções de Inicialização e Suporte

    4.3.1. Função calibracao() (Linhas 315-323)

        Rotina de calibração dos sensores.

Capítulo 5. Gerenciamento do LED RGB

5.1. Controle Direto do LED

    5.1.1. Função LedRGB() (Linhas 325-339)

        Controle de cor via PWM, com opção de intermitência.

5.2. Padrões de Animação (FreeRTOS)

    5.2.1. Implementação da tarefaRGB (Linhas 185-236)

        Casos de Estado (switch) para desligado (0), padrão (1), Rainbow (2) e Pulsar (3).

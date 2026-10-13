# O que foi feito (resumo técnico)

Trabalho feito sem acesso ao carrinho: só código, validado em simulação. Data: 30/09–01/10/2026.

## O problema

Carrinho seguidor de linha (ESP32-S3, array QTR-8A, ponte H TB6612, IMU BMI160), sem encoder. Objetivo:
reconhecer trechos da pista (zigue-zague, cruzamentos, curvas), gravar, e nas voltas seguintes andar mais rápido
e passar reto no zigue-zague. Dúvida do time: "como saber a posição do carrinho só com sensores, giro e PWM?"

## A resposta

Não existe posição (x,y) confiável sem encoder, e não precisa. O carrinho está preso a uma linha: basta saber
em qual segmento está e quando ele acaba. Três fontes, nenhuma depende de tempo:

1. Giroscópio integrado: ângulo de uma curva é invariante à velocidade.
2. Marcos do array: cruzamentos e barras acendem os 8 sensores; a ordem deles é fixa; cada um re-sincroniza.
3. Velocidade pela IMU em curva (v = a_lat/ω) calibra pwm→velocidade, usada para estimar distância nas retas.

## O que existe agora no repositório

| pasta | o que é |
|---|---|
| `lib/core/` | núcleo de controle em C++ puro (sem Arduino). Mesmo arquivo roda no ESP32 e no PC. |
| `lib/bmi160/` | driver I2C mínimo do BMI160 (±2000 dps, ±4 g, 1600 Hz). |
| `src/main.cpp` | firmware: cola entre hardware e núcleo. BLE, botão, LEDs, NVS (mapa e ganhos). |
| `test/test_core/` | 32 checagens unitárias do núcleo (g++, sem framework). |
| `sim/` | simulador: pista extraída da imagem publicada, física do carrinho, vídeo, varredura de ganhos. |
| `sim/ml/` | experimentos de ML (evolução dos ganhos, rede neural) e comparação. |
| `sim/tune_ble.py` | ajuste de ganhos no carrinho real por BLE, em laço fechado. |
| `docs/` | este arquivo e o manual de uso. |

## O núcleo (`lib/core`), em uma página

- **Seguir linha**: PID sobre a posição da linha (±1), ganho P parte fixo e parte proporcional ao PWM
  (quanto mais rápido, mais autoridade), D sobre o erro, e giroscópio como amortecedor.
- **Classificador de segmentos**: reta / curva esquerda / curva direita / zigue-zague / barra, por curvatura
  (ω/v, invariante à velocidade), com estado provisório: curva só confirma com ≥23° acumulados, reta só com
  ≥10 cm, para balanço do PID não virar segmento.
- **Mapa**: volta 1 grava (tipo, ângulo, duração, distância, curvatura máxima). Fecha a volta quando os 7
  primeiros segmentos se repetem (tipos, ângulos e distâncias). Volta 2+: casa segmentos ao vivo com o mapa,
  barras re-sincronizam, perfil de velocidade por curvatura (v = √(a_lat_max/κ)), freio antes da curva por
  distância de frenagem.
- **Zigue-zague**: detecta por reversões largas da posição da linha com rumo líquido constante; segura o rumo
  pelo giroscópio, corrige o centro da oscilação; sai quando a oscilação para, a linha foge ou a distância do
  mapa foi cumprida. Com mapa, entra na 1ª reversão.
- **Cruzamento**: 8 sensores pretos → segura rumo E taxa de giro (se estava em curva, continua a curva).
- **Perda de linha**: pivô para o último lado visto.
- **Aprendizado volta a volta**: volta limpa sobe o perfil 8 %; perdeu linha recua 15 % na hora. Teto +25 %.
- **Externo**: gancho para controlador externo (usado pelos experimentos de ML).

## Resultados (simulação, pista da competição)

- Volta de mapeamento 18,3 s → voltas com mapa 13,0–13,5 s → com aprendizado por volta chega a 11,5 s.
- Zigue-zague atravessado reto em todas as voltas, cruzamentos tratados, parada na chegada.
- 5 sementes de ruído diferentes: 5/5 completam. Afastamento máximo sensor–linha ≈ 4,7 cm.
- ML: evolução dos ganhos dá −19 % mas sobreajusta à física simulada; rede neural no lugar do PID é pior e
  instável. Conclusão: ML como otimizador de parâmetros com randomização de física, nunca como controlador.

## O que NÃO foi validado

- Compilação real com PlatformIO (não instalado aqui; só checagem contra stubs).
- Sinais e eixos da IMU, qual motor é o esquerdo, ganhos reais, aderência real.
- O zigue-zague da pista tem amplitude ±3 cm e o array tem meia-largura 3,3 cm: folga quase zero. É o ponto
  mais sensível para testar primeiro no carrinho.

## Decisões que valem registrar

- Tempo nunca é usado como referência de posição; só ângulo, marcos e distância estimada (com resync).
- Nenhuma peça nova: o time não vai adicionar encoder.
- Limiares em unidades físicas (rad, m, 1/m) para não depender da velocidade do dia.
- Mapa e ganhos persistem na NVS; `CL` apaga o mapa.

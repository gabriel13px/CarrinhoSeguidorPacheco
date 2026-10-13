# CarrinhoSeguidorPacheco

Seguidor de linha do time Pachecoins (Liga LARI). ESP32-S3 + array QTR-8A + ponte H TB6612 + IMU BMI160. Sem encoder.

Ganhador da Corrida da 3ª competição Liga LARI e da Dtech.

---

## ⚠️ Branch `caotico_neutro` — leia antes de testar

Branch de trabalho, **isolada e sem push**. A `main` não foi tocada. Para testar:
`git checkout caotico_neutro`, depois `pio run -t upload`.

**O firmware NÃO foi compilado com PlatformIO neste ambiente** — só checagem de sintaxe. O primeiro
`pio run` de vocês é o teste de verdade; se der erro, mandem a saída inteira (costuma ser de uma linha).

### O que mudou nesta branch (vs. o que estava no carrinho)

1. **`GG`/`PM` não resetam mais o ESP.** O crash era estouro de pilha da task do Bluetooth (`Stack canary
   ... BTC_TASK`): as respostas longas eram enviadas de dentro do callback BLE. Agora o comando só marca uma
   flag e o envio sai no `loop()`, com pausa entre linhas.
2. **Ele não "desliga" mais ao sair da linha.** Era `lost_time` curto demais (0,03 s) + recuperação parada.
   Agora `lost_time` = 0,15 s e, ao perder a linha, ele **avança em arco** pro último lado visto até
   reencontrar, como o código antigo. Só para com botão, `OF` ou chegada (`G:laps=N`). Ajustável por BLE:
   `G:lost_time=...` e `G:v_lost=...`.
3. **A telemetria de teste mudou** (o app precisa acompanhar). Com o carrinho parado no modo teste (`TS`),
   agora é `T:modo,seg,pos_linha,rumo,gyro_cru,acc_x,acc_y,omega` — o que serve parado é o giro e o
   acelerômetro, não `v_est` (que parado é sempre 0). A telemetria de corrida `R:` e o `LAP:` são separados e
   continuam iguais.
4. **Pinos e orientação** da fiação real já entraram como padrão: motor A/B (`A_HOR=37 A_ANTI=36 B_HOR=38
   B_ANTI=47`), sensores `{11,10,9,8,18,17,16,15}`, `BMI160=0x68`, `IMU_GYRO_SIGN=-1`. **Ainda não validado em
   pista** — confira no modo teste (abaixo).

### Primeiro teste, para destravar ("não dava uma volta")

Isolem o problema antes de ligar mapa e zigue-zague:
1. `TS` e confira a IMU: girando o carrinho pra **esquerda**, `gyro_cru` tem que ficar **positivo**; empurrando
   de lado pra esquerda, `acc_y` positivo. Se não, ajuste `IMU_GYRO_SIGN` / eixos em `src/main.cpp`.
2. `MA` (manual): `MF=120` vai pra frente, `ML=120` gira pra esquerda. Se trocado, `MOTOR_LEFT_IS_A` em `src/main.cpp`.
3. `PI`, `G:use_map=0`, `G:v_base=70`, calibra (botão), 2 s parado, liga. **Se nesse modo ele seguir a linha**,
   o núcleo e a orientação estão ok e o resto é só ajuste de ganho. Se nem assim, é orientação (giro ou motor).

Depois que seguir limpo em PID puro: `G:use_map=1` liga o mapa (1ª volta mapeia, demais correm rápido).

### Comandos BLE novos (além dos antigos `PI/MA/TS/ON/OF/MF...`)

`G:nome=valor` muda qualquer ganho na hora · `GG` lista os ganhos · `GS` grava na NVS · `TL=1`/`TL=0`
liga/desliga telemetria de corrida · `SV`/`LD`/`CL`/`PM` salva/carrega/apaga/imprime o mapa · `LB` fecha volta
manualmente. Tabela completa de ganhos e receitas por tipo de pista em
[`docs/MANUAL_DO_CARRINHO.md`](docs/MANUAL_DO_CARRINHO.md).

---

Manuais: [`docs/O_QUE_FOI_FEITO.md`](docs/O_QUE_FOI_FEITO.md) (resumo técnico) e
[`docs/MANUAL_DO_CARRINHO.md`](docs/MANUAL_DO_CARRINHO.md) (o que mudou, como compilar, conferir, correr e ajustar).

## O que tem aqui

```
lib/core/        núcleo de controle em C++ puro (sem Arduino). Mesmo código roda no ESP32 e no PC.
lib/bmi160/      driver I2C mínimo do BMI160 (gyro ±2000 dps, acc ±4 g, 1600 Hz).
src/main.cpp     firmware: cola entre hardware (QTR, IMU, motores, BLE, botão, LED, NVS) e o núcleo.
test/test_core/  testes unitários do núcleo (g++ puro, sem framework).
sim/             simulador: pista real extraída da imagem publicada + física do carrinho + vídeo.
```

## Ideia central

O carrinho não tem encoder, então não existe posição (x,y) confiável. Mas ele está preso numa linha: só
precisa saber **em qual segmento está** e **quando o segmento acaba**. Três fontes resolvem isso sem depender
de tempo nem de velocidade:

1. **Giroscópio integrado**: ângulo acumulado de uma curva não depende da velocidade. Reta = ω≈0,
   curva = ω num sentido só até fechar o ângulo, zigue-zague = ω alternando com média zero.
2. **Marcos do array**: cruzamentos e barras de largada/chegada acendem os 8 sensores de uma vez. A ordem
   deles é sempre a mesma. Cada marco re-sincroniza a posição no mapa.
3. **Velocidade sem encoder**: em curva, aceleração lateral = v·ω, logo v = a_lat/ω só com a IMU. Isso
   calibra uma tabela pwm→velocidade, usada para estimar distância nas retas (erro ~10 %, zerado no próximo marco).

**Volta 1 (mapeamento)**: segue linha devagar e grava a sequência de segmentos (tipo, ângulo, distância,
curvatura máxima). O fechamento da volta é detectado quando os primeiros 7 segmentos se repetem.
**Voltas seguintes**: usa o mapa para o perfil de velocidade (reta a fundo, freia antes de curva pela curvatura
gravada), entra no modo zigue-zague logo na 1ª reversão, e mantém o rumo pelo giroscópio nos cruzamentos.
O mapa é salvo na NVS: desligou e ligou, continua valendo (comando `CL` apaga).
**Aprendizado volta a volta**: volta limpa sobe o perfil de velocidade 8 %, perda de linha recua 15 % na hora
(`lap_boost`, `lap_backoff`, teto `boost_max`). Em ~5 voltas acha o limite da pista e da bateria do dia.

**Zigue-zague**: quando a posição da linha oscila com amplitude grande e o rumo líquido não muda, o carrinho
para de seguir a linha e segura o rumo pelo giroscópio, corrigindo só o centro da oscilação. Sai quando a
oscilação para (ou a linha foge para um lado, ou a distância do mapa foi cumprida).

## Simulador

```bash
cd sim
python3 track_from_image.py            # extrai a pista de ../../pistaGlobal.jpeg -> data/track.npz (uma vez)
./build_core.sh                        # compila lib/core (.so + testes nativos) e roda os testes
python3 run_sim.py --laps 3 --png out/sim.png --video out/sim.mp4 --events
python3 run_sim.py --live              # playback interativo (janela)
python3 dump_log.py --t0 18.3 --t1 19.2   # tabela passo a passo de uma janela de tempo
python3 sweep.py --grid a_lat_max=5,6.5,8 --grid a_brake=3.5,5    # varredura de ganhos
python3 -m unittest discover -s tests -v                            # testes de integração
```

Dependências: python3, numpy, scipy, matplotlib, PIL, g++, ffmpeg (vídeo). Sem PlatformIO.

O simulador modela: tração diferencial, atraso de 1ª ordem dos motores, queda de bateria, limite de aderência
lateral (μ·g, acima disso sub-esterça), 8 sensores com mancha de leitura e ruído, giroscópio com bias e ruído,
acelerômetro com ruído. Escala da pista: assume linha de 19 mm (→ 2,0 × 1,3 m, 11,7 m de volta).

O que o simulador **valida**: lógica de segmentação, fechamento de volta, sincronização por marcos,
zigue-zague, cruzamentos, perfil de velocidade, contagem de voltas, parada na chegada.
O que ele **não valida**: ganhos reais (kp, kd, kgyro), velocidade real, aderência real, orientação da IMU.
Isso tem que ser ajustado no carrinho, via BLE (`G:nome=valor`) e `sim/sweep.py` como guia.

## Firmware (não testado em hardware ainda)

`pio run` para compilar. Antes de correr:

1. Modo teste (`TS`) e conferir sinais da IMU: girar o carrinho para a esquerda => `gyro_z` positivo;
   empurrar para a esquerda => `acc_y` positivo. Senão, ajustar `IMU_*_SIGN`/`IMU_*_AXIS` em `main.cpp`.
2. Confirmar que motor A é o da esquerda (`MOTOR_LEFT_IS_A`).
3. Calibrar sensores (botão), deixar o carrinho **parado 2 s** (estima o bias do giroscópio) e ligar.

Comandos BLE (Nordic UART): `PI` `MA` `TS` `ON` `OF` (como antes), `G:nome=valor` (qualquer ganho de
`lf_gains_t`), `GS` salva ganhos, `SV`/`LD`/`CL`/`PM` salva/carrega/apaga/imprime o mapa, `LB` sync manual de
volta. Formato antigo `Kp=..Ki=..Kd=..Vmax=..Vmin=..` continua aceito.

Telemetria no modo teste (20 Hz): `T:modo,seg,pos_linha,rumo,omega,v_est,k,volta,idx/len,barras` e `S:s0..s7`.

## Experimento: ML contra o clássico (no simulador)

`sim/ml/` treina, com o mesmo orçamento, três versões e compara com o PID padrão em física nominal e perturbada
(`ml/train.py`, `ml/compare.py`). Avaliação: 2 voltas com o mapa pré-gravado; pontuação = progresso − tempo − penalidades.

| cenário | PID padrão | PID + ganhos por ES (600 voltas) | Rede neural do zero (640 voltas) | Rede partindo do PID (480 voltas) |
|---|---|---|---|---|
| nominal | 13,06 s (100 %) | **10,62 s** (100 %) | falhou (0 %) | 11,28 s (50 %) |
| bateria fraca (−15 % vel.) | 13,32 s (100 %) | **falhou (0 %)** | 14,17 s (50 %) | 12,94 s (100 %) |
| piso liso (μg 7) | 13,43 s (100 %) | 11,15 s (100 %) | falhou (0 %) | 11,51 s (50 %) |

(tempo médio de volta e taxa de conclusão em 2 sementes de ruído; mapa gravado em cada cenário pelo controlador padrão)

Leitura:
- **Otimizar ganhos do controlador clássico com ML (estratégia evolutiva) funciona**: −19 % no tempo de volta.
  Mas treinado só na física nominal, **sobreajusta**: com bateria 15 % mais fraca, falha sempre. Treino tem que
  randomizar a física (bateria, atraso do motor, aderência, posição do sensor) e validar no carrinho real.
- **Rede neural substituindo o PID**: do zero, não aprende com esse orçamento; partindo do PID, fica rápida mas
  completa só metade das voltas. Mesmo orçamento, pior resultado, zero garantia. Não vale no carrinho.
- **Conclusão**: ML entra como otimizador de parâmetros do controlador clássico (com randomização), não como controlador.

```bash
python3 ml/train.py --what gains --gens 40 --pop 24      # ES nos ganhos (out/ml/gains_best.npz)
python3 ml/train.py --what nn --gens 60 --pop 32 --init pid
python3 ml/compare.py --seeds 3                          # tabela out/ml/compare.md
python3 run_sim.py --laps 2 --preset-map out/ml/map_seed0.pkl --gains-npz out/ml/gains_best.npz --video out/ml/pid_es.mp4
```

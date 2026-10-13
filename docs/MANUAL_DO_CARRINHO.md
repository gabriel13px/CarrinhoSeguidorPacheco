# Manual: o que mudou e o que fazer

Para quem vai compilar, gravar e correr com o carrinho.

## 1. O que mudou em relação ao código antigo

- `src/main.cpp` foi reescrito. O PID que estava dentro dele agora vive em `lib/core/` (C++ puro, testado
  no simulador). O `main.cpp` só liga hardware ao núcleo.
- Loop de controle a 500 Hz, taxa fixa. Sem `Serial.printf` dentro do loop.
- IMU BMI160 lida por I2C (`lib/bmi160/`, SDA=GPIO2, SCL=GPIO1, endereço 0x69; se SDO estiver em GND, troque
  para 0x68 em `BMI160_ADDR`).
- PWM dos motores de 5 kHz para 20 kHz (tira o apito).
- Mapa da pista e ganhos ficam gravados na NVS (flash). Desligou e ligou, continuam.
- BLE: mesmo nome, mesmos UUIDs, mesmos comandos antigos. Comandos novos abaixo.
- `platformio.ini`: mesmas dependências de antes.
- Pastas novas: `lib/core`, `lib/bmi160`, `test/`, `sim/`, `docs/`. Nada disso atrapalha o `pio run`.

## 2. Compilar e gravar

```bash
pio run              # compila
pio run -t upload    # grava
pio device monitor   # serial 115200
```

Se der erro de compilação, mandar a saída inteira: é coisa de uma linha (versão de biblioteca).
Não rodar `pio test` no ESP32: os testes do núcleo são para PC (`sim/build_core.sh`).

## 3. Antes da primeira corrida (uma vez)

**a) Conferir a IMU.** Conectar por BLE, mandar `TS` (modo teste). Vem telemetria `T:` a 20 Hz:
`T:modo,seg,pos_linha,rumo,gyro_cru,acc_x,acc_y,omega_filtrado` (com o carrinho parado, `gyro_cru` e
`acc_x`/`acc_y` são o que importa para conferir a IMU; `v_est` saiu do modo teste porque parado é sempre 0).
- Gire o carrinho na mão para a **esquerda** (anti-horário visto de cima): `omega` tem que ser **positivo**.
- Se vier negativo: em `main.cpp`, `IMU_GYRO_SIGN` → `-1.0f`.
- Se `omega` quase não mexe ao girar no plano: eixo errado, troque `IMU_GYRO_AXIS` (0, 1 ou 2).
- Acelerômetro lateral: empurre o carrinho para a esquerda, `acc_y` positivo (não aparece no `T:`; se precisar
  conferir, imprima `ac[]` no serial). Se o carrinho for montado com a placa deitada normal, o padrão serve.
- Serial na partida mostra `[IMU] BMI160 ok` ou `NAO ENCONTRADO`. Sem IMU o carrinho segue linha, mas sem
  zigue-zague reto, sem mapa confiável.

**b) Conferir os motores.** Modo manual (`MA`), `MF=120` tem que ir para frente, `ML=120` girar para a
esquerda. Se girar para a direita, `MOTOR_LEFT_IS_A` → `0`.

**c) Conferir a posição da linha.** No modo teste, linha debaixo do sensor da esquerda do carrinho:
`pos_linha` negativa. Se positiva, os pinos do array estão em ordem invertida: inverta a lista em
`qtr.setSensorPins`.

## 4. Correr

1. Botão: calibra os sensores (passe o array sobre a linha e o fundo, uns 3 s). LED pulsa amarelo, depois verde.
2. **Deixe o carrinho parado 2 s** (ele mede o bias do giroscópio).
3. Botão de novo (ou `ON` por BLE): corre. Botão/`OF`: para.
4. Primeira volta é lenta (mapeamento). Quando o mapa fecha, BLE avisa `MAP:CLOSED` e ele acelera.
5. Cada volta sai `LAP:volta,tempo,perdas`. Volta limpa → ele sobe o ritmo 8 %. Perdeu linha → recua.
6. Para parar sozinho depois de N voltas: `G:laps=3` (0 = sem fim).

Mapa fica salvo. Na próxima tentativa na mesma pista ele já começa rápido. Pista diferente: `CL` apaga.

## 5. Comandos BLE

| comando | efeito |
|---|---|
| `PI` `MA` `TS` | modo PID / manual / teste (como antes) |
| `ON` `OF` | liga / desliga (como antes) |
| `MF=pwm` `MT=` `ML=` `MR=` `MS` | manual (como antes) |
| `G:nome=valor` | muda um ganho na hora, mesmo correndo (ex.: `G:kp=40`, `G:v_base=100`) |
| `GG` | lista todos os ganhos atuais |
| `GS` | grava os ganhos na NVS |
| `TL=1` / `TL=0` | liga/desliga telemetria de corrida (10 Hz): `R:t_volta,volta,modo,seg,pos,v_est,idx/len,barras,perdas,boost` |
| `SV` `LD` `CL` `PM` | salva / carrega / apaga / imprime o mapa |
| `LB` | força fechamento de volta agora (pista sem marca e sem repetição detectável) |
| `Kp=..Ki=..Kd=..Vmax=..Vmin=..` | formato do app antigo, aceito e convertido de escala |

Modos no `R:`: 0 parado, 1 seguindo linha, 2 zigue-zague, 3 cruzamento, 4 perdeu linha, 5 chegou.
Segmentos: 0 reta, 1 curva esq., 2 curva dir., 3 zigue-zague, 4 barra.

## 6. Ganhos que importam (e valores de partida)

| ganho | padrão | o que faz |
|---|---|---|
| `v_base` | 80 (pwm) | velocidade da volta de mapeamento e do PID puro. Seguro até 100; 110 já perde linha nas curvinhas |
| `kp`, `kp_v` | 40, 1.0 | P fixo + P proporcional ao pwm. Oscila → baixe `kp_v`; lento para centrar → suba |
| `kd` | 1.2 | amortece pelo erro |
| `kgyro` | 6 | amortece pelo giroscópio. Muito alto sub-esterça em curva |
| `a_lat_max` | 6.5 (m/s²) | quão rápido entra nas curvas com mapa. Derrapa → baixe |
| `a_brake`, `brake_margin` | 5, 0.12 m | frenagem assumida e folga antes da curva. Entra rápido demais → suba a margem |
| `v_straight` | 240 | pwm máximo nas retas com mapa |
| `v_zigzag` | 1.2 (m/s) | velocidade no zigue-zague |
| `lap_boost`, `lap_backoff` | 0.08, 0.15 | aprendizado volta a volta (0 desliga) |
| `use_map` | 1 | 0 = só PID, ignora mapa |
| `cycle_no_bar` | 1 | fecha volta por repetição mesmo sem marca na pista |
| `min_lap_time` | 8 (s) | pista pequena com volta < 8 s: baixe |
| `bar_min_black` | 7 | sensores pretos para contar como barra/cruzamento. Linha muito larga: 8 |
| `lost_time` | 0.15 (s) | tempo sem linha até entrar em recuperação. Entra fácil demais → suba; demora a reagir → baixe |
| `v_lost` | 90 (pwm) | agressividade do arco ao procurar a linha perdida (avança virando, não para no lugar) |

Ordem sugerida de ajuste no carrinho: 1) `v_base` até seguir limpo; 2) `kp_v`/`kd`/`kgyro` até não oscilar;
3) `a_lat_max` e `brake_margin` com mapa; 4) `v_zigzag`. Ou deixar `sim/tune_ble.py` fazer isso
(`pip install bleak`, `python3 sim/tune_ble.py --gains kp,kd,kgyro,a_lat_max,brake_margin --laps 2 --iters 20`).

## 7. Receitas por situação

### 7.1 Pista da competição (a da foto)
Nada a mudar: os padrões foram ajustados nela. Sequência no dia:
1. Tentativa 1: `CL` (apaga mapa velho), `G:laps=0`, calibra, 2 s parado, liga. Deixa dar 2–3 voltas: a 1ª
   mapeia (`MAP:CLOSED` no BLE), a 2ª já corre rápido, a 3ª um pouco mais (aprendizado por volta). `OF`.
2. Tentativas 2 e 3: só calibrar e ligar. Mapa e `boost` conquistado estão guardados? **O mapa sim, o boost
   não** (volta a 1,0 ao ligar). Para começar já no ritmo conquistado, veja o `boost` na última linha `R:` da
   tentativa anterior e mande `G:boost_max=1.25` (ou menor, se a última volta teve perda).
3. Se a regra for "largada e chegada na mesma marca, N voltas": `G:laps=N` e ele para sozinho na barra.
Se a linha de chegada for sensor da organização, nada muda no carrinho.

### 7.2 Pista de tamanho ou linha diferentes
| situação | o que mudar |
|---|---|
| volta curta (< 8 s) | `G:min_lap_time=3` (senão o mapa nunca fecha) |
| pista enorme (retas > 3 m) | `G:brake_margin=0.25` (erro de distância cresce com a reta) |
| mais de ~90 segmentos na volta | `LF_MAP_MAX` em `lf_core.h` → 256, recompilar |
| pista lisa sem marca nem cruzamento (oval, "8" sem cruzar) | `cycle_no_bar=1` já é padrão; se mesmo assim não fechar, `LB` na largada da 2ª volta |
| linha larga (3–4 cm) | nada; conferir no modo teste que uma linha normal acende ≤ 5 sensores. Se acender 6–7: `G:bar_min_black=8` |
| linha fina (< 1,5 cm) | `on_line` continua ok; só a calibração precisa de contraste bom |
| curvas mais fechadas que 5 cm de raio | `G:v_base=70` para a volta de mapeamento |
| piso escorregadio | `G:a_lat_max=5` |
Largura da linha e tamanho da pista não estão em lugar nenhum do código: o mapa é gravado no dia.

### 7.3 Treinar (fazer ele aprender a pista)
Não precisa de notebook. Calibra, liga, 1ª volta mapeia, 2ª em diante usa o mapa e sobe o ritmo a cada
volta limpa. Para ver acontecendo: `TL=1` e acompanhar `LAP:volta,tempo,perdas` e o último campo de `R:`
(boost). Mapa salvo sozinho em `MAP:CLOSED`; `PM` imprime; `CL` apaga para outra pista.

### 7.4 Só PID (sem mapa)
`G:use_map=0` e `G:lap_boost=0`. Velocidade é `v_base` fixa (80 padrão, 100 seguro na pista da competição,
110 já perde linha nas curvinhas de 5 cm). Freio por erro (`kv_e`) e por curvatura (`react_kappa`) continuam
ativos. `G:use_map=1` volta ao normal; o mapa gravado não é apagado.

### 7.5 Ganhos vindos do ES (simulador) ou ajustados no carrinho
No PC, com o simulador: `cd sim && python3 ml/train.py --what gains --gens 40 --pop 24` gera
`out/ml/gains_best.npz`. Levar ao carrinho:
- `python3 tune_ble.py --print out/ml/gains_best.npz` imprime os `G:...` para colar em qualquer terminal BLE;
- ou `python3 tune_ble.py --apply out/ml/gains_best.npz` manda e grava (`GS`).
Atenção: ganhos do simulador sobreajustam à física simulada (no teste, falharam com bateria fraca). Use como
ponto de partida e confirme com 2–3 voltas reais. Para afinar no carrinho de verdade:
`python3 tune_ble.py --gains kp,kd,kgyro,a_lat_max,brake_margin --laps 2 --iters 20` (cada tentativa é uma
corrida real; alguém recoloca o carrinho na largada). Voltar aos padrões: apagar a NVS de ganhos não tem
comando; recompilar/gravar restaura, ou mandar os valores da tabela da seção 6.

## 8. Sinais de que algo está errado

- Carrinho vira para o lado errado ao sair da linha: sinal do giro ou motor trocado (seção 3).
- Entra em "zigue-zague" (modo 2 no `R:`) em trecho normal: `zz_amp` muito baixo ou giro com sinal errado.
- `MAP:CLOSED` nunca aparece: pista com poucos segmentos ou volta < `min_lap_time`. Use `LB` na largada da
  2ª volta ou baixe `min_lap_time`.
- Perde a linha nos vértices do zigue-zague: é a folga de 3 mm do array. Baixe `v_zigzag`, suba `kp_zz`.
- Desliga/para ao sair da linha: era `lost_time` curto demais (corrigido para 0.15 s). Ele NÃO desliga ao
  perder a linha; entra em recuperação e avança em arco pro último lado visto até reencontrar, como o código
  antigo. Para parar só botão, `OF` ou chegada com `G:laps=N`.

## 9. Simulador no PC (opcional)

```bash
cd sim && ./build_core.sh && python3 run_sim.py --laps 3 --video out/sim.mp4
```
Pista nova: coloque a foto, rode `python3 track_from_image.py foto.jpeg`. O simulador valida lógica, não ganhos
reais.

// SeguidorPacheco — firmware ESP32-S3.
// Toda a lógica de controle está em lib/core (lf_core.h), o mesmo código testado no simulador (sim/).
// Este arquivo é só a "cola": sensores QTR, IMU BMI160, motores TB6612, BLE, botão, LEDs e NVS.
//
// ATENÇÃO (não testado em hardware): compile com `pio run`, confira sinais da IMU no modo teste (TS)
// antes de correr: girar o carrinho para a ESQUERDA (anti-horário visto de cima) => gyro_z POSITIVO;
// empurrar o carrinho para a ESQUERDA => acc_y POSITIVO. Se vier invertido, troque IMU_GYRO_SIGN / IMU_ACC_Y_SIGN.
#include <Arduino.h>
#include <QTRSensors.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <Wire.h>
#include "BMI160Raw.h"
#include "lf_core.h"

#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// -------- pinos (placa Pacheco V2) -----------------------------
#define PIN_A_HORARIO     37
#define PIN_A_ANTIHORARIO 36
#define PIN_B_HORARIO     38
#define PIN_B_ANTIHORARIO 47
#define PIN_APWM          41
#define PIN_BPWM          35
#define PIN_LED_R         13
#define PIN_LED_G         14
#define PIN_LED_B         12
#define PIN_EMISSOR_CONTROLE 21     // botão
#define PIN_EMISSOR_LINHA    7      // emissor IR do array
#define PIN_NEOPIXEL      48
#define PIN_I2C_SDA        2
#define PIN_I2C_SCL        1
#define BMI160_ADDR     0x68        // placa Pacheco: SDO em GND (0x69 se em VDDIO)

// -------- orientação / convenções ------------------------------
#define MOTOR_LEFT_IS_A   1         // motor A é o da esquerda (como no firmware antigo: vA = base - correcao)
#define IMU_GYRO_AXIS     2         // eixo do giroscópio que aponta para cima (0=x,1=y,2=z)
#define IMU_GYRO_SIGN     (-1.0f)   // +1 se girar à esquerda dá leitura positiva (invertido nesta montagem)
#define IMU_ACC_Y_AXIS    1         // eixo do acelerômetro que aponta para a esquerda do carrinho
#define IMU_ACC_Y_SIGN    (+1.0f)
#define IMU_ACC_X_AXIS    0         // eixo que aponta para a frente
#define IMU_ACC_X_SIGN    (+1.0f)

#define LOOP_US           2000      // período do loop de controle (500 Hz). 8 ADC + 12 bytes I2C cabem folgado.
#define PWM_FREQ_HZ       20000     // acima do audível; TB6612 aceita até 100 kHz
#define TELEM_MS          50        // telemetria BLE no modo teste (20 Hz)
#define RACE_TELEM_MS     100       // telemetria BLE durante a corrida (10 Hz), ligada com TL=1

Adafruit_NeoPixel neopixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
QTRSensors qtr;
const uint8_t QUANT_SENSORES = 8;
uint16_t sensorValores[QUANT_SENSORES];
BMI160Raw imu;
Preferences prefs;
lf_handle_t core = nullptr;

enum Modo { MODO_PID, MODO_MANUAL, MODO_TESTE };
Modo modoAtual = MODO_PID;
bool onOff = false, calibracaoAtiva = false, imuOk = false, ultimoHasMap = false;
bool telemCorrida = true;          // TL=0/1
volatile bool pedirGanhos = false, pedirMapa = false;   // GG/PM: enviados no loop, nunca no callback BLE
float ultGyro = 0, ultAccX = 0, ultAccY = 0;            // ultima leitura da IMU (telemetria de teste)
uint8_t ultimaVolta = 1; uint32_t inicioVoltaMs = 0; uint16_t perdasVolta = 0; uint8_t ultimoModo = 0;
int8_t estadoLed = 1;

enum DirManual { DIR_STOP, DIR_FRENTE, DIR_TRAS, DIR_ESQUERDA, DIR_DIREITA };
DirManual direcaoAtual = DIR_STOP;
int16_t pwmManual = 255;

// -------- protótipos -------------------------------------------
void calibracao();
void controleMotores(int motorA, int motorB);
void LedRGB(int r, int g, int b, int tempo, int loop);
void tarefaRGB(void* param);
void processarComando(const String& cmd);
void executarManual();
void salvarMapa();
bool carregarMapa();
void salvarGanhos();
bool carregarGanhos();
void enviarMapa();
void enviarTelemetria();
void enviarTelemetriaCorrida(const lf_telem_t& t);
void enviarGanhos();

// -------- BLE ---------------------------------------------------
BLEServer* pServer = nullptr;
BLECharacteristic* pTxChar = nullptr;
bool bleConectado = false;
static String bufferBle = "";

void bleSend(const String& msg) {
  if (!bleConectado || pTxChar == nullptr) return;
  const size_t MTU = 20;
  size_t len = msg.length(), offset = 0;
  while (offset < len) {
    size_t chunk = (len - offset < MTU) ? (len - offset) : MTU;
    pTxChar->setValue((uint8_t*)(msg.c_str() + offset), chunk);
    pTxChar->notify();
    offset += chunk;
    yield();
  }
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bleConectado = true; estadoLed = 1; LedRGB(0, 0, 255, 0, 1); Serial.println("BLE conectado"); }
  void onDisconnect(BLEServer*) override { bleConectado = false; bufferBle = ""; BLEDevice::startAdvertising(); }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string val = pChar->getValue();
    for (char c : val) {
      if (c == '\n' || c == '\r') {
        bufferBle.trim();
        if (bufferBle.length() > 0) { processarComando(bufferBle); bufferBle = ""; }
      } else bufferBle += c;
    }
  }
};

// -------- tabela de ganhos acessíveis por BLE ("G:nome=valor") --
struct GainField { const char* name; size_t off; bool is_u8; };
#define GF(n) {#n, offsetof(lf_gains_t, n), false}
#define GU(n) {#n, offsetof(lf_gains_t, n), true}
const GainField GAIN_FIELDS[] = {
  GF(kp), GF(ki), GF(kd), GF(kgyro), GF(kp_v), GF(kv_e), GF(react_kappa), GF(kh), GF(kh_gyro), GF(kp_zz), GF(ki_zz),
  GF(v_base), GF(v_max), GF(v_straight), GF(v_min_curve), GF(v_zigzag), GF(v_lost), GF(ramp_up), GF(ramp_down),
  GF(a_lat_max), GF(a_brake), GF(brake_margin), GF(k_pwm2v), GF(tau_motor), GF(lap_boost), GF(lap_backoff), GF(boost_max), GF(boost_min), GF(lost_tol),
  GF(kappa_curve), GF(kappa_straight), GF(t_curve), GF(t_straight), GF(min_seg_time), GF(min_curve_angle),
  GF(min_straight_dist), GF(zz_hyst), GF(zz_amp), GF(zz_max_turn), GF(zz_window), GF(zz_exit_time),
  GF(cross_hold_time), GF(cross_max_time), GF(lost_time), GF(min_lap_time), GF(angle_tol),
  GU(zz_min_swaps), GU(bar_min_black), GU(laps), GU(cycle_k), GU(cycle_no_bar), GU(use_map),
};
const int N_GAIN_FIELDS = sizeof(GAIN_FIELDS) / sizeof(GAIN_FIELDS[0]);

bool setGain(const String& name, float value) {
  lf_gains_t g; lf_get_gains(core, &g);
  for (int i = 0; i < N_GAIN_FIELDS; i++) {
    if (name == GAIN_FIELDS[i].name) {
      uint8_t* p = (uint8_t*)&g + GAIN_FIELDS[i].off;
      if (GAIN_FIELDS[i].is_u8) *p = (uint8_t)value; else *(float*)p = value;
      lf_set_gains(core, &g);
      return true;
    }
  }
  return false;
}

// -------- setup ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[BOOT] SeguidorPacheco (lf_core)");
  // motivo do ultimo reset: se o carrinho "desliga" ao sair da pista, veja aqui o que apareceu.
  // BROWNOUT = queda de tensao (motor puxando corrente / bateria fraca); TASK_WDT/INT_WDT = travou;
  // PANIC = crash; POWERON/SW = normal.
  { esp_reset_reason_t r = esp_reset_reason();
    const char* m = r == ESP_RST_BROWNOUT ? "BROWNOUT (queda de tensao)" :
                    r == ESP_RST_TASK_WDT ? "TASK_WDT (travou)" :
                    r == ESP_RST_INT_WDT  ? "INT_WDT (travou)" :
                    r == ESP_RST_PANIC    ? "PANIC (crash)" :
                    r == ESP_RST_POWERON  ? "POWERON (normal)" :
                    r == ESP_RST_SW       ? "SW (normal)" : "outro";
    Serial.printf("[BOOT] motivo do reset: %s (%d)\n", m, (int)r); }

  neopixel.begin(); neopixel.setBrightness(255); neopixel.show();

  core = lf_create();
  lf_gains_t g; lf_default_gains(&g);
  g.laps = 0;                       // sem fim: para pelo botão/BLE (ou G:laps=N)
  lf_set_gains(core, &g);

  prefs.begin("lf", false);
  if (carregarGanhos()) Serial.println("[NVS] ganhos carregados");
  if (carregarMapa()) { Serial.println("[NVS] mapa carregado"); ultimoHasMap = true; }

  BLEDevice::init("SeguidorPacheco");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* pService = pServer->createService(NUS_SERVICE);
  pTxChar = pService->createCharacteristic(NUS_TX_CHAR, BLECharacteristic::PROPERTY_NOTIFY);
  pTxChar->addDescriptor(new BLE2902());
  BLECharacteristic* pRxChar = pService->createCharacteristic(NUS_RX_CHAR, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxChar->setCallbacks(new RxCallbacks());
  pService->start();
  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(NUS_SERVICE);
  pAdv->setScanResponse(true);
  BLEDevice::startAdvertising();

  // motores e LED
  ledcSetup(0, PWM_FREQ_HZ, 8); ledcAttachPin(PIN_APWM, 0);
  ledcSetup(1, PWM_FREQ_HZ, 8); ledcAttachPin(PIN_BPWM, 1);
  ledcSetup(4, 5000, 8); ledcAttachPin(PIN_LED_R, 4);
  ledcSetup(5, 5000, 8); ledcAttachPin(PIN_LED_G, 5);
  ledcSetup(6, 5000, 8); ledcAttachPin(PIN_LED_B, 6);
  pinMode(PIN_A_HORARIO, OUTPUT); pinMode(PIN_A_ANTIHORARIO, OUTPUT);
  pinMode(PIN_B_HORARIO, OUTPUT); pinMode(PIN_B_ANTIHORARIO, OUTPUT);
  pinMode(PIN_EMISSOR_CONTROLE, INPUT_PULLUP);

  // sensores de linha: 1 amostra por sensor para caber no período do loop
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){11, 10, 9, 8, 18, 17, 16, 15}, QUANT_SENSORES);
  qtr.setEmitterPin(PIN_EMISSOR_LINHA);
  qtr.setSamplesPerSensor(1);

  // IMU
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  imuOk = imu.begin(Wire, BMI160_ADDR);
  Serial.printf("[IMU] BMI160 %s\n", imuOk ? "ok" : "NAO ENCONTRADO (gyro/acc = 0)");

  controleMotores(0, 0);
  LedRGB(255, 0, 0, 0, 1);
  xTaskCreatePinnedToCore(tarefaRGB, "tarefaRGB", 2048, NULL, 1, NULL, 1);
  Serial.println("[BOOT] pronto");
}

// -------- loop de controle a taxa fixa ----------------------------
void loop() {
  static uint32_t proximo = micros();
  static uint32_t ultimoTelem = 0;
  static bool botaoAnterior = false;

  // botão: calibra (1º toque) / liga-desliga (toques seguintes)
  bool botao = digitalRead(PIN_EMISSOR_CONTROLE) == LOW;
  if (botao && !botaoAnterior && modoAtual == MODO_PID) {
    if (!calibracaoAtiva) calibracao();
    else {
      onOff = !onOff;
      if (onOff) { LedRGB(0, 255, 0, 150, 2); lf_start(core); ultimaVolta = 1; inicioVoltaMs = millis(); perdasVolta = 0; bleSend("STATE:ON\n"); }
      else { lf_stop(core); controleMotores(0, 0); LedRGB(255, 0, 0, 200, 2); bleSend("STATE:OFF\n"); }
    }
    delay(30);   // debounce simples
  }
  botaoAnterior = botao;

  if (modoAtual == MODO_MANUAL) { executarManual(); delay(5); return; }

  // espera o próximo tick (sem bloquear BLE: ele roda no outro core)
  int32_t espera = (int32_t)(proximo - micros());
  if (espera > 0) { if (espera > 200) delayMicroseconds(espera - 100); return; }
  proximo += LOOP_US;
  static uint32_t ultimoTick = micros();
  uint32_t agora = micros();
  float dt = (agora - ultimoTick) * 1e-6f;
  ultimoTick = agora;

  // 1. sensores
  lf_frame_t f;
  if (calibracaoAtiva) qtr.readCalibrated(sensorValores);
  else { qtr.read(sensorValores); for (int i = 0; i < QUANT_SENSORES; i++) sensorValores[i] = (uint16_t)min(1000UL, (unsigned long)sensorValores[i] * 1000UL / 4095UL); }
  for (int i = 0; i < QUANT_SENSORES; i++) f.s[i] = sensorValores[i];
  float gy[3] = {0, 0, 0}, ac[3] = {0, 0, 0};
  if (imuOk) imu.read(gy, ac);
  f.gyro_z = IMU_GYRO_SIGN * gy[IMU_GYRO_AXIS];
  f.acc_y = IMU_ACC_Y_SIGN * ac[IMU_ACC_Y_AXIS];
  f.acc_x = IMU_ACC_X_SIGN * ac[IMU_ACC_X_AXIS];
  f.dt = dt;
  ultGyro = f.gyro_z; ultAccX = f.acc_x; ultAccY = f.acc_y;

  // 2. núcleo (IDLE quando parado: estima bias do giroscópio)
  lf_cmd_t cmd;
  lf_step(core, &f, &cmd);
  if (modoAtual == MODO_PID && onOff && calibracaoAtiva) {
#if MOTOR_LEFT_IS_A
    controleMotores(cmd.left, cmd.right);
#else
    controleMotores(cmd.right, cmd.left);
#endif
  } else controleMotores(0, 0);

  // 3. eventos: mapa fechou -> salva na NVS; chegou -> desliga
  lf_telem_t t; lf_get_telem(core, &t);
  if (t.has_map && !ultimoHasMap) { salvarMapa(); bleSend("MAP:CLOSED\n"); }
  ultimoHasMap = t.has_map;
  if (t.mode == LF_MODE_FINISHED && onOff) { onOff = false; controleMotores(0, 0); bleSend("STATE:FINISHED\n"); LedRGB(0, 255, 0, 100, 5); }

  // 4. eventos de volta (tempo real da volta anterior + quantas vezes perdeu a linha) e telemetria
  if (onOff) {
    if (t.mode == LF_MODE_LOST && ultimoModo != LF_MODE_LOST) perdasVolta++;
    ultimoModo = t.mode;
    if (t.lap != ultimaVolta) {
      char line[48]; snprintf(line, sizeof(line), "LAP:%d,%.2f,%d\n", ultimaVolta, (millis() - inicioVoltaMs) / 1000.0f, perdasVolta);
      bleSend(line); Serial.print(line);
      ultimaVolta = t.lap; inicioVoltaMs = millis(); perdasVolta = 0;
    }
    if (telemCorrida && millis() - ultimoTelem >= RACE_TELEM_MS) { ultimoTelem = millis(); enviarTelemetriaCorrida(t); }
  }
  if (modoAtual == MODO_TESTE && millis() - ultimoTelem >= TELEM_MS) { ultimoTelem = millis(); enviarTelemetria(); }

  // 5. respostas longas pedidas por BLE: enviadas aqui, nunca dentro do callback (senão reseta)
  if (pedirGanhos) { pedirGanhos = false; enviarGanhos(); }
  if (pedirMapa)   { pedirMapa = false;   enviarMapa(); }
}

// -------- comandos BLE ------------------------------------------
void processarComando(const String& cmd) {
  Serial.println("BLE RX: " + cmd);
  if (cmd == "PI") { modoAtual = MODO_PID; direcaoAtual = DIR_STOP; controleMotores(0, 0); bleSend("MODE:PID\n"); return; }
  if (cmd == "MA") { modoAtual = MODO_MANUAL; onOff = false; lf_stop(core); controleMotores(0, 0); bleSend("MODE:MANUAL\n"); return; }
  if (cmd == "TS") { modoAtual = MODO_TESTE; onOff = false; lf_stop(core); controleMotores(0, 0); bleSend("MODE:TEST\n"); return; }
  if (cmd == "ON") {
    if (modoAtual != MODO_PID || !calibracaoAtiva) { bleSend("ERR:CALIB\n"); return; }
    onOff = true; lf_start(core); ultimaVolta = 1; inicioVoltaMs = millis(); perdasVolta = 0; bleSend("STATE:ON\n"); return;
  }
  if (cmd == "OF") { onOff = false; lf_stop(core); controleMotores(0, 0); bleSend("STATE:OFF\n"); return; }
  if (cmd == "SV") { salvarMapa(); bleSend("MAP:SAVED\n"); return; }
  if (cmd == "LD") { bleSend(carregarMapa() ? "MAP:LOADED\n" : "MAP:NONE\n"); return; }
  if (cmd == "CL") { prefs.remove("map"); prefs.remove("mapn"); lf_set_map(core, nullptr, 0); ultimoHasMap = false; bleSend("MAP:CLEARED\n"); return; }
  if (cmd == "PM") { pedirMapa = true; return; }
  if (cmd == "GS") { salvarGanhos(); bleSend("GAINS:SAVED\n"); return; }
  if (cmd == "GG") { pedirGanhos = true; return; }                          // lista ganhos (enviado no loop)
  if (cmd.startsWith("TL=")) { telemCorrida = cmd.substring(3).toInt() != 0; bleSend(telemCorrida ? "TELEM:ON\n" : "TELEM:OFF\n"); return; }
  if (cmd == "LB") { lf_lap_boundary(core); bleSend("LAP:SYNC\n"); return; }   // sync manual de volta
  if (cmd.startsWith("G:")) {                       // G:kp=150
    int eq = cmd.indexOf('=');
    if (eq > 2 && setGain(cmd.substring(2, eq), cmd.substring(eq + 1).toFloat())) bleSend("OK:" + cmd.substring(2) + "\n");
    else bleSend("ERR:GAIN\n");
    return;
  }
  if (modoAtual == MODO_MANUAL && cmd.startsWith("M")) {
    String dirCmd = cmd.substring(0, 2);
    int eqIdx = cmd.indexOf('=');
    if (eqIdx >= 0) pwmManual = (int16_t)constrain((int)cmd.substring(eqIdx + 1).toInt(), 0, 255);
    if (dirCmd == "MF") direcaoAtual = DIR_FRENTE;
    else if (dirCmd == "MT") direcaoAtual = DIR_TRAS;
    else if (dirCmd == "ML") direcaoAtual = DIR_ESQUERDA;
    else if (dirCmd == "MR") direcaoAtual = DIR_DIREITA;
    else if (dirCmd == "MS") direcaoAtual = DIR_STOP;
    return;
  }
  // formato antigo do app: Kp=..Ki=..Kd=..Kr=..Vmax=..Vmin=..
  int idxKp = cmd.indexOf("Kp="), idxKi = cmd.indexOf("Ki="), idxKd = cmd.indexOf("Kd="), idxKr = cmd.indexOf("Kr=");
  int idxVmax = cmd.indexOf("Vmax="), idxVmin = cmd.indexOf("Vmin=");
  bool algum = false;
  if (idxVmax >= 0 && idxVmin > idxVmax) {
    setGain("v_max", cmd.substring(idxVmax + 5, idxVmin).toFloat());
    setGain("v_base", cmd.substring(idxVmin + 5).toFloat());
    algum = true;
  }
  if (idxKp >= 0 && idxKi > idxKp && idxKd > idxKi) {
    // escala antiga: erro em ±3500. Nova: erro em ±1 (x3500) e parte do P vem de kp_v·pwm (zera para manter o app antigo)
    setGain("kp", cmd.substring(idxKp + 3, idxKi).toFloat() * 3500.0f);
    setGain("kp_v", 0.0f);
    setGain("ki", cmd.substring(idxKi + 3, idxKd).toFloat() * 3500.0f);
    setGain("kd", cmd.substring(idxKd + 3, idxKr >= 0 ? idxKr : (idxVmax >= 0 ? idxVmax : cmd.length())).toFloat() * 3500.0f * 0.001f);
    algum = true;
  }
  bleSend(algum ? "OK:GAINS\n" : "ERR:CMD\n");
}

// -------- motores -------------------------------------------------
void controleMotores(int motorA, int motorB) {
  motorA = constrain(motorA, -255, 255); motorB = constrain(motorB, -255, 255);
  digitalWrite(PIN_A_HORARIO, motorA > 0); digitalWrite(PIN_A_ANTIHORARIO, motorA < 0); ledcWrite(0, abs(motorA));
  digitalWrite(PIN_B_HORARIO, motorB > 0); digitalWrite(PIN_B_ANTIHORARIO, motorB < 0); ledcWrite(1, abs(motorB));
}

void executarManual() {
  switch (direcaoAtual) {
    case DIR_FRENTE:   controleMotores(pwmManual, pwmManual); break;
    case DIR_TRAS:     controleMotores(-pwmManual, -pwmManual); break;
    case DIR_ESQUERDA: controleMotores(-pwmManual, pwmManual); break;
    case DIR_DIREITA:  controleMotores(pwmManual, -pwmManual); break;
    default:           controleMotores(0, 0); break;
  }
}

// -------- calibração ----------------------------------------------
void calibracao() {
  estadoLed = 3;
  Serial.println("Calibrando sensores (passe o array sobre a linha)...");
  for (uint16_t i = 0; i < 400; i++) qtr.calibrate();
  calibracaoAtiva = true;
  estadoLed = 1;
  LedRGB(0, 255, 0, 0, 1);
  bleSend("CALIB:DONE\n");
  Serial.println("Calibracao concluida. Deixe o carrinho PARADO 2 s antes de ligar (bias do giroscopio).");
}

// -------- NVS: mapa e ganhos ---------------------------------------
void salvarMapa() {
  static lf_map_entry_t buf[LF_MAP_MAX];
  int n = lf_get_map(core, buf, LF_MAP_MAX);
  prefs.putBytes("map", buf, n * sizeof(lf_map_entry_t));
  prefs.putInt("mapn", n);
  Serial.printf("[NVS] mapa salvo: %d entradas\n", n);
}

bool carregarMapa() {
  int n = prefs.getInt("mapn", 0);
  if (n <= 0 || n > LF_MAP_MAX) return false;
  static lf_map_entry_t buf[LF_MAP_MAX];
  if (prefs.getBytes("map", buf, n * sizeof(lf_map_entry_t)) != n * sizeof(lf_map_entry_t)) return false;
  lf_set_map(core, buf, n);
  return true;
}

void salvarGanhos() { lf_gains_t g; lf_get_gains(core, &g); prefs.putBytes("gains", &g, sizeof(g)); }

bool carregarGanhos() {
  lf_gains_t g;
  if (prefs.getBytesLength("gains") != sizeof(g)) return false;
  prefs.getBytes("gains", &g, sizeof(g));
  lf_set_gains(core, &g);
  return true;
}

// -------- telemetria ----------------------------------------------
void enviarMapa() {
  static lf_map_entry_t buf[LF_MAP_MAX];
  int n = lf_get_map(core, buf, LF_MAP_MAX);
  bleSend("MAP:" + String(n) + "\n");
  for (int i = 0; i < n; i++) {
    char line[64];
    snprintf(line, sizeof(line), "M%d:%d,%.2f,%.2f,%.2f,%.1f\n", i, buf[i].type, buf[i].angle, buf[i].duration, buf[i].dist, buf[i].kappa_max);
    bleSend(line);
    delay(15);
  }
}

void enviarTelemetria() {
  lf_telem_t t; lf_get_telem(core, &t);
  char line[120];
  // modo teste (carrinho parado na mão): pos da linha, rumo, giro cru e acelerometro X/Y para conferir a IMU
  snprintf(line, sizeof(line), "T:%d,%d,%.2f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
           t.mode, t.seg, t.line_pos, t.heading, ultGyro, ultAccX, ultAccY, t.omega_f);
  bleSend(line);
  String s = "S:";
  for (int i = 0; i < QUANT_SENSORES; i++) { s += sensorValores[i]; s += ','; }
  s += '\n';
  bleSend(s);
  Serial.print(line); Serial.print(s);
}

// durante a corrida, 10 Hz: R:t_volta,volta,modo,seg,pos_linha,v_est,idx/len,barras,perdas
void enviarTelemetriaCorrida(const lf_telem_t& t) {
  char line[96];
  snprintf(line, sizeof(line), "R:%.1f,%d,%d,%d,%.2f,%.2f,%d/%d,%d,%d,%.2f\n",
           t.t_lap, t.lap, t.mode, t.seg, t.line_pos, t.v_est, t.map_idx, t.map_len, t.bars, perdasVolta, t.boost);
  bleSend(line);
}

void enviarGanhos() {
  lf_gains_t g; lf_get_gains(core, &g);
  for (int i = 0; i < N_GAIN_FIELDS; i++) {
    const uint8_t* p = (const uint8_t*)&g + GAIN_FIELDS[i].off;
    char line[48];
    if (GAIN_FIELDS[i].is_u8) snprintf(line, sizeof(line), "G:%s=%d\n", GAIN_FIELDS[i].name, *p);
    else snprintf(line, sizeof(line), "G:%s=%.4g\n", GAIN_FIELDS[i].name, *(const float*)p);
    bleSend(line);
    delay(15);                      // dá tempo do stack BLE esvaziar a fila de notify (senão reseta)
  }
}

// -------- LED RGB ---------------------------------------------------
void LedRGB(int r, int g, int b, int tempo, int loop) {
  for (int i = 0; i < loop; i++) {
    ledcWrite(4, r); ledcWrite(5, g); ledcWrite(6, b);
    neopixel.setPixelColor(0, neopixel.Color(r, g, b)); neopixel.show();
    if (tempo > 0) {
      delay(tempo);
      ledcWrite(4, 0); ledcWrite(5, 0); ledcWrite(6, 0);
      neopixel.setPixelColor(0, 0); neopixel.show();
      delay(tempo);
    }
  }
}

void tarefaRGB(void* param) {
  while (true) {
    switch (estadoLed) {
      case 0: while (estadoLed == 0) { LedRGB(0, 0, 0, 0, 1); vTaskDelay(10 / portTICK_PERIOD_MS); } break;
      case 1: while (estadoLed == 1) vTaskDelay(10 / portTICK_PERIOD_MS); break;
      case 2:
        while (estadoLed == 2)
          for (int hue = 0; hue < 360 && estadoLed == 2; hue++) {
            float rad = hue * 3.14159f / 180.0f;
            LedRGB((int)((sinf(rad) + 1) * 127.5f), (int)((sinf(rad + 2.09439f) + 1) * 127.5f), (int)((sinf(rad + 4.18878f) + 1) * 127.5f), 0, 1);
            vTaskDelay(5 / portTICK_PERIOD_MS);
          }
        break;
      case 3:
        while (estadoLed == 3) {
          for (int i = 0; i <= 255 && estadoLed == 3; i += 5) { LedRGB(i, i, 0, 0, 1); vTaskDelay(10 / portTICK_PERIOD_MS); }
          for (int i = 255; i >= 0 && estadoLed == 3; i -= 5) { LedRGB(i, i, 0, 0, 1); vTaskDelay(10 / portTICK_PERIOD_MS); }
        }
        break;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

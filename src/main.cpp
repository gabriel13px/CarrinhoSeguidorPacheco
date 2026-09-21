#include <Arduino.h>
#include <QTRSensors.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_NeoPixel.h>


#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// -------- pinos -------------
#define PIN_A_HORARIO     38  
#define PIN_A_ANTIHORARIO 47  

#define PIN_B_HORARIO     37  
#define PIN_B_ANTIHORARIO 36


#define PIN_APWM          41  
#define PIN_BPWM          35   

//pinos para LEDs RGB
#define PIN_LED_R          13  
#define PIN_LED_G          14  
#define PIN_LED_B          12  

// Botão e emissor sensores
#define PIN_EMISSOR_CONTROLE         21  
#define PIN_EMISSOR_LINHA        7  

// LED RGB Neopixel 
#define PIN_NEOPIXEL      48
#define NEOPIXEL_COUNT     1
Adafruit_NeoPixel neopixel(NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// -------- sensores --------------------------------------------
QTRSensors qtr;
const uint8_t QUANT_SENSORES = 8;
uint16_t sensorValores[QUANT_SENSORES];
// -------- modos -----------------------------------------------
enum Modo { MODO_PID, MODO_MANUAL, MODO_TESTE };
Modo modoAtual = MODO_PID;

// -------- estado geral ----------------------------------------
bool onOff           = false;
bool calibracaoAtiva = false;
int8_t estadoLed     = 1;

// -------- PID -------------------------------------------------
float Kp = 0.28f, Ki = 0.015f, Kd = 3.68f, Kr = 0.0f;
int16_t ultimoErro = 0;
int16_t erros[10]  = {};

int16_t velocidadeMaximaA = 255;
int16_t velocidadeMaximaB = 255;
int16_t velocidadeBaseA   = 237;
int16_t velocidadeBaseB   = 237;

uint8_t contadorSaiuLinha = 0;
bool    warnOfflineSent   = false;

// -------- controle remoto ---------------------------
enum DirManual { DIR_STOP, DIR_FRENTE, DIR_TRAS, DIR_ESQUERDA, DIR_DIREITA };
DirManual direcaoAtual = DIR_STOP;
int16_t   pwmManual    = 255; 

// -------- temporização modo teste ----------------------------
uint32_t ultimoEnvioTeste   = 0;
const uint16_t INTERVALO_TESTE_MS = 100;

// -------- protótipos -----------------------------------------
void calibracao();
void controleMotores(int motorA, int motorB);
void controle_PID();
void LedRGB(int r, int g, int b, int tempo, int loop);
void tarefaRGB(void* param);
void processarComando(const String& cmd);
void executarManual();
void enviarDadosTeste();
// -------- BLE -------------------------------------------------
BLEServer*         pServer   = nullptr;
BLECharacteristic* pTxChar   = nullptr; 
bool bleConectado             = false;
static String bufferBle       = "";   

void bleSend(const String& msg) {
  if (!bleConectado || pTxChar == nullptr) return;
  const size_t MTU = 20;
  size_t len    = msg.length();
  size_t offset = 0;
  while (offset < len) {
    size_t chunk = (len - offset < MTU) ? (len - offset) : MTU;
    pTxChar->setValue((uint8_t*)(msg.c_str() + offset), chunk);
    pTxChar->notify();
    offset += chunk;
    yield(); 
  }
}
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConectado = true;
    estadoLed = 1;
    LedRGB(0, 0, 255, 0, 1);
    Serial.println("BLE conectado");
  }
  void onDisconnect(BLEServer* s) override {
    bleConectado = false;
    bufferBle = "";
    Serial.println("BLE desconectado — reiniciando advertising");
    BLEDevice::startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string val = pChar->getValue();
    for (char c : val) {
      if (c == '\n' || c == '\r') {
        bufferBle.trim();
        if (bufferBle.length() > 0) {
          processarComando(bufferBle);
          bufferBle = "";
        }
      } else {
        bufferBle += c;
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[BOOT] ESP32-S3 iniciando...");

  neopixel.begin();
  neopixel.setBrightness(255); 
  neopixel.show();

  BLEDevice::init("SeguidorPacheco");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(NUS_SERVICE);

  pTxChar = pService->createCharacteristic(
    NUS_TX_CHAR,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxChar->addDescriptor(new BLE2902());

  BLECharacteristic* pRxChar = pService->createCharacteristic(
    NUS_RX_CHAR,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRxChar->setCallbacks(new RxCallbacks());

  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(NUS_SERVICE);
  pAdv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("BLE iniciado");

  // ── motores ───────────────────────────────────────────────
  ledcSetup(0, 5000, 8); ledcAttachPin(PIN_APWM, 0);
  ledcSetup(1, 5000, 8); ledcAttachPin(PIN_BPWM, 1);

  // ── LED RGB ───────────────────────────────────────────────
  ledcSetup(4, 5000, 8); ledcAttachPin(PIN_LED_R, 4);
  ledcSetup(5, 5000, 8); ledcAttachPin(PIN_LED_G, 5);
  ledcSetup(6, 5000, 8); ledcAttachPin(PIN_LED_B, 6);

  pinMode(PIN_A_HORARIO,    OUTPUT);
  pinMode(PIN_A_ANTIHORARIO,OUTPUT);
  pinMode(PIN_B_HORARIO,    OUTPUT);
  pinMode(PIN_B_ANTIHORARIO,OUTPUT);
  pinMode(PIN_EMISSOR_CONTROLE, INPUT_PULLUP);

  // ── sensores ──────────────────────────────────────────────
  qtr.setTypeAnalog();
  //pinos sensores - - - - - - - -
  qtr.setSensorPins((const uint8_t[]){15,16,17,18,8,9,10,11}, QUANT_SENSORES);
  qtr.setEmitterPin(PIN_EMISSOR_LINHA);
  qtr.setTimeout(2600);

  controleMotores(0, 0);
  LedRGB(255, 0, 0, 0, 1);

  xTaskCreatePinnedToCore(tarefaRGB, "tarefaRGB", 2048, NULL, 1, NULL, 1);
  Serial.println("[BOOT] Setup concluido — aguardando BLE");
}

void loop() {
  static bool ultimoBleConectado = false;
  static bool ultimoOnOff        = false;
  static bool ultimoCalib        = false;
  if ((bleConectado != ultimoBleConectado)|| (onOff != ultimoOnOff)||(calibracaoAtiva != ultimoCalib)) {
    ultimoBleConectado = bleConectado;
    ultimoOnOff        = onOff;
    ultimoCalib        = calibracaoAtiva;
    if (!bleConectado) {
      estadoLed = calibracaoAtiva ? (onOff ? 2 : 1) : 1;
      if (!calibracaoAtiva) LedRGB(255, 0, 0, 0, 1);
      else if (!onOff)      LedRGB(255, 255, 255, 0, 1);
    }
  }

  // ── 2. Botão físico ───────────────────────────────────────
  if (digitalRead(PIN_EMISSOR_CONTROLE) == LOW) {
    if (!calibracaoAtiva && !onOff && modoAtual == MODO_PID) {
      calibracao();
    } else if (calibracaoAtiva && modoAtual == MODO_PID) {
      onOff = !onOff;
      if (onOff) {
        estadoLed = 1;
        LedRGB(255,0,0,300,3);
        LedRGB(255,127,0,300,1);
        LedRGB(0,255,0,300,1);
        bleSend("STATE:ON\n");
        Serial.println("corrida iniciada");
      } else {
        estadoLed = 1;
        LedRGB(255,0,0,200,2);
        bleSend("STATE:OFF\n");
        Serial.println("corrida parou");
      }
    }
  }

  switch (modoAtual) {
    case MODO_PID:
      if (onOff && calibracaoAtiva) controle_PID();
      else                          controleMotores(0, 0);
      break;
    case MODO_MANUAL:
      executarManual();
      break;
    case MODO_TESTE:
      enviarDadosTeste();
      break;
  }
}

void processarComando(const String& cmd) {
  Serial.println("BLE RX: " + cmd);

  if (cmd == "PI") {
    modoAtual = MODO_PID;
    direcaoAtual = DIR_STOP;
    controleMotores(0, 0);
    bleSend("MODE:PID\n");
    Serial.println("[OK] Modo PID ativado");
    return;
  }
  if (cmd == "MA") {
    modoAtual = MODO_MANUAL;
    onOff = false;
    controleMotores(0, 0);
    bleSend("MODE:MANUAL\n");
    bleSend("STATE:OFF\n");
    Serial.println("[OK] Modo Manual ativado");
    return;
  }
  if (cmd == "TS") {
    modoAtual = MODO_TESTE;
    controleMotores(0, 0);
    bleSend("MODE:TEST\n");
    Serial.println("[OK] Modo Teste de Sensor ativado");
    return;
  }

  if (cmd == "ON" && modoAtual == MODO_PID && calibracaoAtiva) {
    onOff = true;
    bleSend("STATE:ON\n");
    Serial.println("[OK] Carro LIGADO");
    return;
  }
  if (cmd == "ON" && modoAtual == MODO_PID && !calibracaoAtiva) {
    Serial.println("[IGNORADO] ON recebido mas calibracao nao foi feita");
    return;
  }
  if (cmd == "ON" && modoAtual != MODO_PID) {
    Serial.println("[IGNORADO] ON recebido fora do modo PID");
    return;
  }
  if (cmd == "OF") {
    onOff = false;
    controleMotores(0, 0);
    bleSend("STATE:OFF\n");
    Serial.println("[OK] Carro DESLIGADO");
    return;
  }

  if (modoAtual == MODO_MANUAL) {
    String dirCmd = cmd.substring(0, 2);
    int eqIdx = cmd.indexOf('=');
    if (eqIdx >= 0) {
      int val = cmd.substring(eqIdx + 1).toInt();
      pwmManual = constrain(val, 0, 255);
    }
    if      (dirCmd == "MF") { direcaoAtual = DIR_FRENTE;   Serial.printf("[OK] Manual: FRENTE  PWM=%d\n", pwmManual); }
    else if (dirCmd == "MT") { direcaoAtual = DIR_TRAS;     Serial.printf("[OK] Manual: TRAS    PWM=%d\n", pwmManual); }
    else if (dirCmd == "ML") { direcaoAtual = DIR_ESQUERDA; Serial.printf("[OK] Manual: ESQUERDA PWM=%d\n", pwmManual); }
    else if (dirCmd == "MR") { direcaoAtual = DIR_DIREITA;  Serial.printf("[OK] Manual: DIREITA  PWM=%d\n", pwmManual); }
    else if (dirCmd == "MS") { direcaoAtual = DIR_STOP;     Serial.println("[OK] Manual: PARAR"); }
    if (cmd.startsWith("M")) return;
  } else if (cmd.startsWith("M") && cmd != "MA") {
    Serial.println("[IGNORADO] Comando de direcao recebido fora do modo Manual");
    return;
  }

  int idxKp   = cmd.indexOf("Kp=");
  int idxKi   = cmd.indexOf("Ki=");
  int idxKd   = cmd.indexOf("Kd=");
  int idxKr   = cmd.indexOf("Kr=");
  int idxVmax = cmd.indexOf("Vmax=");
  int idxVmin = cmd.indexOf("Vmin=");

  if (idxVmax >= 0 && idxVmin > idxVmax) {
    velocidadeMaximaA = velocidadeMaximaB =
        (uint8_t)cmd.substring(idxVmax + 5, idxVmin).toInt();
    velocidadeBaseA = velocidadeBaseB =
        (uint8_t)cmd.substring(idxVmin + 5).toInt();
    Serial.printf("[OK] Velocidades: Vmax=%d Vmin=%d\n", velocidadeMaximaA, velocidadeBaseA);
  }
  if (idxKp >= 0 && idxKi > idxKp && idxKd > idxKi && modoAtual == MODO_PID) {
    Kp = cmd.substring(idxKp + 3, idxKi).toFloat();
    Ki = cmd.substring(idxKi + 3, idxKd).toFloat();
    Kd = cmd.substring(idxKd + 3, idxKr >= 0 ? idxKr : idxVmax).toFloat() * 2;
    if (idxKr >= 0)
      Kr = cmd.substring(idxKr + 3, idxVmax >= 0 ? idxVmax : cmd.length()).toFloat();
    Serial.printf("[OK] PID atualizado: Kp=%.3f Ki=%.3f Kd=%.3f Kr=%.3f\n", Kp, Ki, Kd, Kr);
  } else if (idxKp >= 0 && modoAtual != MODO_PID) {
    Serial.println("[IGNORADO] Constantes PID recebidas fora do modo PID");
  } else if (idxKp < 0 && idxVmax < 0) {
    Serial.println("[DESCONHECIDO] Comando nao reconhecido: " + cmd);
  }
}

// ==============================================================
void executarManual() {
  switch (direcaoAtual) {
    case DIR_FRENTE:   controleMotores( pwmManual,  pwmManual); break;
    case DIR_TRAS:     controleMotores(-pwmManual, -pwmManual); break;
    case DIR_ESQUERDA: controleMotores( pwmManual, -pwmManual); break;
    case DIR_DIREITA:  controleMotores(pwmManual,  pwmManual); break;
    default:           controleMotores(0, 0); break;
  }
}

void enviarDadosTeste() {
  uint32_t agora = millis();
  if (agora - ultimoEnvioTeste < INTERVALO_TESTE_MS) return;
  ultimoEnvioTeste = agora;

  uint32_t soma = 0;
  uint16_t pos  = 0;
  String msg = "S:";

  if (calibracaoAtiva) {
    pos = qtr.readLineBlack(sensorValores);
  } else {
    qtr.read(sensorValores);
    for (uint8_t i = 0; i < QUANT_SENSORES; i++) {
      sensorValores[i] = map(sensorValores[i], 0, 2600, 0, 1000);
    }
    pos = 0; 
  }

  for (uint8_t i = 0; i < QUANT_SENSORES; i++) {
    soma += sensorValores[i];
    msg += sensorValores[i];
    msg += ',';
  }
  msg += pos;
  msg += ',';
  msg += soma;
  msg += '\n';

  bleSend(msg);
  Serial.print(msg);
}

// ==============================================================
void controleMotores(int motorA, int motorB) {
  if (motorA > 0) {
    digitalWrite(PIN_A_HORARIO, HIGH); digitalWrite(PIN_A_ANTIHORARIO, LOW);
    ledcWrite(0, motorA);
  } else if (motorA < 0) {
    digitalWrite(PIN_A_HORARIO, LOW);  digitalWrite(PIN_A_ANTIHORARIO, HIGH);
    ledcWrite(0, -motorA);
  } else {
    digitalWrite(PIN_A_HORARIO, LOW);  digitalWrite(PIN_A_ANTIHORARIO, LOW);
    ledcWrite(0, 0);
  }
  if (motorB > 0) {
    digitalWrite(PIN_B_HORARIO, HIGH); digitalWrite(PIN_B_ANTIHORARIO, LOW);
    ledcWrite(1, motorB);
  } else if (motorB < 0) {
    digitalWrite(PIN_B_HORARIO, LOW);  digitalWrite(PIN_B_ANTIHORARIO, HIGH);
    ledcWrite(1, -motorB);
  } else {
    digitalWrite(PIN_B_HORARIO, LOW);  digitalWrite(PIN_B_ANTIHORARIO, LOW);
    ledcWrite(1, 0);
  }
}

// ==============================================================
void errosPassados(int erro) {
  for (int i = 9; i > 0; i--) erros[i] = erros[i - 1];
  erros[0] = erro;
}

int errosSomatorio(int qtd) {
  int soma = 0;
  for (int i = 0; i < qtd; i++) soma += erros[i];
  return soma;
}

// ==============================================================
void controle_PID() {
  uint16_t posicao = qtr.readLineBlack(sensorValores);
  int erro = 3500 - (int)posicao;
  errosPassados(erro);

  int32_t soma = 0;
  for (uint8_t i = 0; i < QUANT_SENSORES; i++) soma += sensorValores[i];

  if (soma <= 700) {
    contadorSaiuLinha++;
    if (contadorSaiuLinha > 10) {
      if (!warnOfflineSent) {
        bleSend("WARN:OFFLINE\n");
        warnOfflineSent = true;
      }
      if (ultimoErro > 0) controleMotores(velocidadeMaximaA, 0);
      else                controleMotores(0, velocidadeMaximaB);
    }
    return;
  }
  contadorSaiuLinha = 0;
  warnOfflineSent   = false;

  if (abs(erro) < (int)(1000 * Kr)) erro = 0;

  int P = erro;
  int I = errosSomatorio(5);
  int D = erro - ultimoErro;
  ultimoErro = erro;

  int correcao = (int)(P * Kp + I * Ki + D * Kd);
  int vA = constrain(velocidadeBaseA - correcao, -velocidadeMaximaA, velocidadeMaximaA);
  int vB = constrain(velocidadeBaseB + correcao, -velocidadeMaximaB, velocidadeMaximaB);

  controleMotores(vA, vB);
  Serial.printf("VA=%d VB=%d Pos=%d\n", vA, vB, posicao);
}

// ==============================================================
void calibracao() {
  estadoLed = 3;
  Serial.println("Calibrando...");
  for (uint16_t i = 0; i < 500; i++) qtr.calibrate();
  Serial.println("Calibracao concluida");
  estadoLed = 1;
  LedRGB(0, 255, 0, 0, 1);
  calibracaoAtiva = true;
  bleSend("CALIB:DONE\n");
}


void LedRGB(int r, int g, int b, int tempo, int loop) {
  for (int i = 0; i < loop; i++) {
    ledcWrite(4, r); ledcWrite(5, g); ledcWrite(6, b);

    neopixel.setPixelColor(0, neopixel.Color(r, g, b));
    neopixel.show();
    if (tempo > 0) {
      delay(tempo);
      ledcWrite(4, 0); ledcWrite(5, 0); ledcWrite(6, 0);
      neopixel.setPixelColor(0, 0);
      neopixel.show();
      delay(tempo);
    }
  }
}

void tarefaRGB(void* param) {
  while (true) {
    switch (estadoLed) {
      case 0:
        while (estadoLed == 0) { LedRGB(0,0,0,0,1); vTaskDelay(10/portTICK_PERIOD_MS); }
        break;
      case 1:
        while (estadoLed == 1) vTaskDelay(10/portTICK_PERIOD_MS);
        break;
      case 2:
        while (estadoLed == 2) {
          for (int hue = 0; hue < 360; hue++) {
            if (estadoLed != 2) break;
            float rad = hue * 3.14159f / 180.0f;
            LedRGB((int)((sinf(rad)            + 1) * 127.5f),
                   (int)((sinf(rad + 2.09439f) + 1) * 127.5f),
                   (int)((sinf(rad + 4.18878f) + 1) * 127.5f), 0, 1);
            vTaskDelay(5/portTICK_PERIOD_MS);
          }
        }
        break;
      case 3:
        while (estadoLed == 3) {
          for (int i = 0; i <= 255; i += 5) { if (estadoLed!=3) break; LedRGB(i,i,0,0,1); vTaskDelay(10/portTICK_PERIOD_MS); }
          for (int i = 255; i >= 0; i -= 5) { if (estadoLed!=3) break; LedRGB(i,i,0,0,1); vTaskDelay(10/portTICK_PERIOD_MS); }
        }
        break;
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

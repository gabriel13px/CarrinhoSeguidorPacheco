#include <Arduino.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"
#include <SPIFFS.h> 
#include "esp_timer.h"  
#include "esp_task_wdt.h"
File audio;
bool tocando = false;
bool parar = false;
bool tocar = false;
String musicaSelecionada = "/audio1.wav";
int pinoDAC = 26;
int Contadorwatchdogs = 0;
int ContadorParada = 0;
int ContadorSaiuDaLinha = 0;
int Errors[10] = {0,0,0,0,0,0,0,0,0,0};
int estadoLed= 1;


QTRSensors qtr;
const uint8_t QuantSensores = 8;
uint16_t SensorValores[QuantSensores];
BluetoothSerial SerialBT;

float Kp = 0.48;
float Ki = 0.06;
float Kd = 2.5;
float Kr = 0;
int P;
int I;
int D;
int R;
int UltimoErro = 0;

boolean OnOff = false;
boolean calibracaoAtiva = false;
int ManualPid = 0;// 0 = PID, 1 = Manual


 uint8_t VelocidadeMaximaA = 255;
 uint8_t VelocidadeMaximaB = 255;
 uint8_t VelocidadeBaseA = 255;
 uint8_t VelocidadeBaseB = 255;
 

// os sensores devem ser colocados na ordem
//17,18,13,14,27,25,33,32 gpios usados sensores -entrada digital
//26 para buzzer -saida pwm
//5,12 para botao - input pullup
//2,15,4 para led - saida pwm
//19,21,22,23 para ponte H - saidas pwm
//16 ir led

int AHorario =19;// esquerda horario
int AAntiHora =21;//esquerda anti horario
int BHorario =22;//direita horario 
int BAntiHora  =23;//direita anti horario

int LedRed = 15;
int LedGreen = 2;
int LedBlue = 4;
int botao = 5; 
int botaoCalibracao = 12;
TaskHandle_t musicaTaskHandle = NULL;
void calibracao();
void frente_freio(int motorA, int motorB);
void Controle_PID();
void PiscaPisca(int tempo, int vezes);
void LedRGB(int r, int g, int b, int tempo,int loop);
void pararMusica();
void tarefaMusica(void*param);
void tarefaRGB(void*param);
void delay_us_custom(uint64_t us);
void setup(){
  SPIFFS.begin(true);
  Serial.begin(115200);
  SerialBT.begin("SeguidorPacheco"); 
  //setup canais dos motores
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);
  ledcSetup(3, 5000, 8);
  //setup dos canais do led rgb
  ledcSetup(4, 5000, 8);
  ledcSetup(5, 5000, 8);
  ledcSetup(6, 5000, 8);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){17,18,13,14,27,25,33,32}, QuantSensores);
  qtr.setEmitterPin(16); 

  //setup pinos dos motores
  ledcAttachPin(AHorario, 0);
  ledcAttachPin(BAntiHora, 1);
  ledcAttachPin(AAntiHora, 2);
  ledcAttachPin(BHorario, 3);

  //setup pinos do led rgb
  ledcAttachPin(LedRed, 4);
  ledcAttachPin(LedGreen, 5);
  ledcAttachPin(LedBlue, 6);


   pinMode(botao, INPUT_PULLUP);
   pinMode(botaoCalibracao, INPUT_PULLUP);
  
  frente_freio(0,0);
  LedRGB(255, 0, 0,0,1);
  tocar = true;
  musicaSelecionada = "/audio1.wav";
//-----------------------multitarefas------------------
xTaskCreatePinnedToCore(
  tarefaRGB,
  "tarefaRGB",  
  2048,
  NULL,
  1,
  NULL,
  1); // Core 1
  xTaskCreatePinnedToCore(
    tarefaMusica,
    "tarefaMusica",   
    19000,
    NULL,
    1,
    &musicaTaskHandle,
    0);     
}




void loop(){
  // para a musica caso esteja no bluetooth e verifica estado do led
  if (!SerialBT.hasClient()) {
    if(calibracaoAtiva == false) {
      estadoLed = 1;
      LedRGB(255, 0, 0,0,1);
    }else{
      if(OnOff == true){
        estadoLed = 2;
      }else{
        estadoLed = 1;
      LedRGB(255, 255, 255,0,1);
      }
    }
    if (eTaskGetState(musicaTaskHandle) == eSuspended) {
      vTaskResume(musicaTaskHandle); 
      Serial.println("Bluetooth desconectado - Retomando a música.");
    }
  } else {
    estadoLed = 1;
    LedRGB(0, 0, 255,0,1);
    if (eTaskGetState(musicaTaskHandle) != eSuspended) {
      vTaskSuspend(musicaTaskHandle);  
      Serial.println("Bluetooth pareado - Suspendendo a música.");
    }
  }

  //botão calibração
  if(digitalRead(botaoCalibracao) == LOW && OnOff ==false && ManualPid == 0) {
    calibracao();
  }
  //verifica se o bluetooth está conectado e faz a troca das constantes ou troca para o modo manual
  if(SerialBT.available()){
    String EntradaSerial = SerialBT.readStringUntil(' '); 
    Serial.println("Recebido: " + EntradaSerial);
    // SerialBT.println("Recebido: " + EntradaSerial);
    EntradaSerial.trim();

     if (EntradaSerial.length() >= 2) {
      String modo = EntradaSerial.substring(0, 2);

      if (modo == "MA") {
        ManualPid = 1;
        Serial.println("Modo Manual ativado");
        // SerialBT.println("Modo Manual ativado");
      } else if (modo == "PI") {
        ManualPid = 0;
        Serial.println("Modo PID ativado");
        // SerialBT.println("Modo PID ativado");
      }
    }
    
    //leitura e atualização de constantes
    int KpIndex = EntradaSerial.indexOf("Kp=");
    int KiIndex = EntradaSerial.indexOf("Ki=");
    int KdIndex = EntradaSerial.indexOf("Kd=");
    int KrIndex = EntradaSerial.indexOf("Kr=");
    int VMaxIndex = EntradaSerial.indexOf("Vmax=");
    int VMinIndex = EntradaSerial.indexOf("Vmin=");
    String VMaxStr = EntradaSerial.substring(VMaxIndex + 5, VMinIndex);
    String VMinStr = EntradaSerial.substring(VMinIndex + 5);
    VelocidadeMaximaA = (uint8_t)VMaxStr.toInt();
    VelocidadeMaximaB = (uint8_t)VMaxStr.toInt();
    VelocidadeBaseA = (uint8_t)VMinStr.toInt();
    VelocidadeBaseB = (uint8_t)VMinStr.toInt();

    if (KpIndex >= 0 && KiIndex > KpIndex && KdIndex > KiIndex && ManualPid==0) {
      String kpStr = EntradaSerial.substring(KpIndex+3, KiIndex);
      String kiStr = EntradaSerial.substring(KiIndex + 3, KdIndex);
      String kdStr = EntradaSerial.substring(KdIndex + 3, KrIndex);
      String krStr = EntradaSerial.substring(KrIndex + 3, VMaxIndex);
      
      Kp = kpStr.toFloat();
      Ki = kiStr.toFloat();
      Kd = kdStr.toFloat();
      Kr = krStr.toFloat();


      Serial.printf("Constantes atualizadas:\nKp = %.2f\nKi = %.2f\nKd = %.2f\nKr = %.2f\n", Kp, Ki, Kd,Kr);
      // SerialBT.printf("Constantes atualizadas:\nKp = %.2f\nKi = %.2f\nKd = %.2f\n", Kp, Ki, Kd);
    } 
    // leitura e modo manual
    if (ManualPid == 1 && EntradaSerial.length() >= 3) {
      char comando = EntradaSerial.charAt(2);
      int DelIndex = EntradaSerial.indexOf("Del=");
      String DelStr = EntradaSerial.substring(DelIndex + 4,VMaxIndex);
      int Del = DelStr.toInt();
    
      switch (comando) {
        case 'F': // Frente
          frente_freio(VelocidadeMaximaA, VelocidadeMaximaB);
          Serial.println("Frente");
          // SerialBT.println("Frente");

          if (Del > 0) {
            delay(Del);
            frente_freio(0, 0);
          }
          break;
    
        case 'T': // Trás
          frente_freio(-VelocidadeMaximaA, -VelocidadeMaximaB);
          Serial.println("Tras");
          // SerialBT.println("Tras");
          if (Del > 0) {
            delay(Del);
            frente_freio(0, 0);
          }
          
          break;
    
        case 'L': // Esquerda
        frente_freio(VelocidadeMaximaA, -VelocidadeMaximaB);
          Serial.println("Esquerda");
          // SerialBT.println("Esquerda");
          if (Del > 0) {
            delay(Del);
            frente_freio(0, 0);
          }
          break;
    
        case 'R': // Direita
          frente_freio(-VelocidadeMaximaA, VelocidadeMaximaB);
          Serial.println("Direita");
          // SerialBT.println("Direita");
          if (Del > 0) {
            delay(Del);
            frente_freio(0, 0);
          }
          break;
    
        default: 
          frente_freio(0, 0);
          Serial.println("Parado");
          // SerialBT.println("Parado");
          break;
      }
    }

  }

  //botão liga e desliga do carro
  if(digitalRead(botao)==LOW && ManualPid == 0&& calibracaoAtiva==true) {
    OnOff = !OnOff;
    if(OnOff == true) {
      //tempo até o carro ligar
      estadoLed = 1;
      LedRGB(255,0,0,300,3);
      LedRGB(255,127,0,300,1);
      LedRGB(0,255,0,300,1);
      Serial.println("corrida iniciada");
      // SerialBT.println("corrida iniciada");
    }
    else {
      
      //tempo até o carro desligar
      estadoLed = 1;
      LedRGB(255,0,0,200,2);
      Serial.println("corrida Acabou");
      // SerialBT.println("corrida Acabou");
    }
  }

  //controle liga e desliga do carro
  if (OnOff == true&& ManualPid == 0&& calibracaoAtiva == true) {
    //musicaSelecionada = "/Corrida.wav";
    tocar = true;
    Controle_PID();
  }else{
    if (ManualPid == 0) {
      frente_freio(0, 0);
      //pararMusica();
    }
  }
}
void delay_us_custom(uint64_t us) {
  uint64_t start = esp_timer_get_time();
  while ((esp_timer_get_time() - start) < us) {
    esp_task_wdt_reset(); 
  }
}
void tarefaMusica(void*param){
  while (true) {
    if (tocar && !tocando) {
      audio = SPIFFS.open(musicaSelecionada, "r");
      if (!audio) {
        Serial.println("Erro ao abrir o arquivo");
        tocar = false;
        continue;
      }

      audio.seek(45); 
      tocando = true;
      parar = false;

      while (audio.available() && !parar) {
        byte valor = audio.read();
        dacWrite(pinoDAC, valor);
        delay_us_custom(55);
        if (++Contadorwatchdogs >= 100) {
          Contadorwatchdogs = 0;
          vTaskDelay(1);  
        }
      }

      audio.close();
      tocando = false;
      tocar = false;
      Serial.println(" Fim da música");
    }

     vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

void tarefaRGB(void*param){
  while (true) {
    switch (estadoLed) {
    {
      case 0: //estado desligado
      while (estadoLed == 0) {
        LedRGB(0, 0, 0,0,1);
        vTaskDelay(10 / portTICK_PERIOD_MS); 
      }
      break;
    case 1: // estado configuração manual
      while (estadoLed == 1) {
        
        vTaskDelay(10 / portTICK_PERIOD_MS); 
      }
      break;
    case 2: //rainbow
    while (estadoLed == 2) {
      int rr, gg, bb;
  for (int hue = 0; hue < 360; hue++) {
    if (estadoLed != 2) break;

    float rad = hue * 3.14159 / 180.0;
    rr = (int)((sin(rad) + 1) * 127.5);
    gg = (int)((sin(rad + 2.09439) + 1) * 127.5);
    bb = (int)((sin(rad + 4.18878) + 1) * 127.5);

    LedRGB(rr, gg, bb, 0, 1); 
    vTaskDelay(5 / portTICK_PERIOD_MS); 
  }
    }
      
      break;
      
    
      case 3: //estado pulsar calibração
      while (estadoLed == 3) {
        // Aumenta brilho do azul
        for (int i = 0; i <= 255; i += 5) {
          if (estadoLed != 3) break;
          LedRGB(i, i, 0, 0, 1);
          vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    
        // Diminui brilho do azul
        for (int i = 255; i >= 0; i -= 5) {
          if (estadoLed != 3) break;
          LedRGB(i, i, 0, 0, 1);
          vTaskDelay(10 / portTICK_PERIOD_MS);
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}
}
void pararMusica() {
  parar = true;
  delay(10); 
  if (audio) audio.close();
  tocando = false;
  Serial.println(" Música parada");
}

//controle dos motores
void frente_freio(int motorA, int motorB){
  if(motorA > 0){
    ledcWrite(0, motorA);
    ledcWrite(2, 0);
  }else if(motorA < 0){
    ledcWrite(0, 0);
    ledcWrite(2, -(motorA));
  }else{
    ledcWrite(0, 0);
    ledcWrite(2, 0);
  }
  if(motorB > 0){
    ledcWrite(1, motorB);
    ledcWrite(3, 0);
  }else if(motorB < 0){
    ledcWrite(1, 0);
    ledcWrite(3, -(motorB));
  }else{
    ledcWrite(1, 0);
    ledcWrite(3, 0);
  }
}

void past_errors (int error)
{
  for (int i = 9; i > 0; i--)
      Errors[i] = Errors[i-1];
  Errors[0] = error;
}


int errors_sum(int index, int Abs) {
  int sum = 0;
  for (int i = 0; i < index; i++) {
    if (Abs == 0)
      sum += abs(Errors[i]);  // para R
    else
      sum += Errors[i];       // para I
  }
  return sum;
}
//controle do PID

void Controle_PID(){
uint16_t posicao = qtr.readLineBlack(SensorValores);
//------------verificação linha chegada ---------------
//se está sobre a linha preta o valor do sensor é 1000
//se está fora da linha preta o valor do sensor é abaixo de 100
int ValorMaximoSensores = SensorValores[0]+SensorValores[1]+SensorValores[2]+SensorValores[3]+SensorValores[4]+SensorValores[5]+SensorValores[6]+SensorValores[7];
if(ValorMaximoSensores >= 7500){
  ContadorParada++;
  if(ContadorParada ==10){
  OnOff = false;
  frente_freio(0, 0);
  estadoLed = 1;
  LedRGB(255, 0, 0,300,2);
  }
}else{
  ContadorParada = 0;
}
//musicaSelecionada = "/Chegada.wav";
// tocar = true;
//PararMusica();
//------------------verificação se saiu da linha-----------------
if(ValorMaximoSensores <= 700){
  ContadorSaiuDaLinha++;
  if(UltimoErro > 0&& ContadorSaiuDaLinha > 10){
    frente_freio(VelocidadeMaximaA, 0);
  }else{
    frente_freio(0, VelocidadeMaximaB);
  }
}else{
  ContadorSaiuDaLinha = 0;
}
//--------------------------------------------------------
int erro = 3500 - posicao;
past_errors(erro);
//Serial.println(posicao);
P = erro;
I = errors_sum(5, 0);
D = erro - UltimoErro;
UltimoErro = erro;
R = errors_sum(5, 1); // tentativa de aplicar uma logica de redução de velocidade, provavelmente não  vai ser usado

//vai ser necessario alterar a logica dos motores
int VelocidadeMotor = (P*Kp) + (I*Ki) + (D*Kd);
int VelocidadeA = VelocidadeBaseA - VelocidadeMotor- (R*Kr);
int VelocidadeB = VelocidadeBaseB + VelocidadeMotor- (R*Kr);


if (VelocidadeA > VelocidadeMaximaA) {
  VelocidadeA = VelocidadeMaximaA;
}
if (VelocidadeB > VelocidadeMaximaB) {
  VelocidadeB = VelocidadeMaximaB;
}
if (VelocidadeA < -VelocidadeMaximaA) {
  VelocidadeA = -VelocidadeMaximaA;
}
if (VelocidadeB < -VelocidadeMaximaB) {
  VelocidadeB = -VelocidadeMaximaB;
}
frente_freio(VelocidadeA, VelocidadeB);
Serial.printf("VA=%d VB=%d Pos=%d\n", VelocidadeA, VelocidadeB, posicao);
}


void calibracao() {
   estadoLed = 3;
  Serial.println("Calibrando...");
  // SerialBT.println("Calibrando...");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  Serial.println("Calibracao concluida");
  // SerialBT.println("Calibracao concluida");
  estadoLed = 1;
  LedRGB(0, 255, 0,0,1);
  calibracaoAtiva = true;
}

void LedRGB(int r, int g, int b, int tempo,int loop) {
  for(int i = 0; i < loop; i++){
    if(tempo == 0){
      ledcWrite(4, r);
      ledcWrite(5, g);
      ledcWrite(6, b);
    }else{
      ledcWrite(4, r);
    ledcWrite(5, g);
    ledcWrite(6, b);
    delay(tempo);
    ledcWrite(4, 0);
    ledcWrite(5, 0);
    ledcWrite(6, 0);
    delay(tempo);
    }
  }
  
}


#include <Arduino.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"

uint8_t contadorParada = 0;
uint8_t contadorSaiuDaLinha = 0;
int16_t erros[15] = {0,0,0,0,0,0,0,0,0,0};
int8_t estadoLed= 1;

QTRSensors qtr;
const uint8_t quantSensores = 8;
uint16_t sensorValores[quantSensores];
BluetoothSerial serialBt;
float Kp = 0.28;
float Ki = 0.015;
float Kd = 3.68;
float Kr = 0;
int16_t P,I, D;
int16_t ultimoErro = 0;
boolean onOff = false;
boolean calibracaoAtiva = false;
int8_t manualPid = 0;// 0 = PID, 1 = Manual
int16_t velocidadeMaximaA = 255;
int16_t velocidadeMaximaB = 255;
int16_t velocidadeBaseA = 237;
int16_t velocidadeBaseB = 237;
 //----------pinos do esp--------------------------------------
// os sensores devem ser colocados na ordem
//17,18,13,14,27,25,33,32 gpios usados sensores -entrada digital
//5, para botao - input pullup
//2,15,4 para led - saida pwm
//19,21,22,23 para ponte H - saidas digitais
//26,12 pwm motor a e b
//16 ir led
//-------------------------------------------------------------
#define aHorario 21// esquerda horario
#define aAntiHora 19//esquerda anti horario
#define bHorario 23//direita horario 
#define bAntiHora  22//direita anti horario
#define APWM  26 // motor a pwm
#define BPWM  12 // motor b pwm
#define ledRed  15
#define ledGreen 2
#define ledBlue  4
#define botao  5 
//-----------prototipagem das funções----------
void calibracao();
void controleMotores(int motorA, int motorB);
void controle_PID();
void LedRGB(int r, int g, int b, int tempo,int loop);
void tarefaRGB(void*param);
//---------------------------------------------
void setup(){
  Serial.begin(115200);
  serialBt.begin("SeguidorPacheco"); 
  //----------setup canais dos motores--------
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);
  ledcSetup(3, 5000, 8);
  //-----setup dos canais do led rgb----------
  ledcSetup(4, 5000, 8);
  ledcSetup(5, 5000, 8);
  ledcSetup(6, 5000, 8);
//-----setup dos sensores---------------------
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){17,18,13,14,27,25,33,32}, quantSensores);
  qtr.setEmitterPin(16); 
  qtr.setTimeout(2600);

  ledcAttachPin(APWM, 0);
  ledcAttachPin(BPWM, 1);
  pinMode(aHorario, OUTPUT);
  pinMode(bAntiHora, OUTPUT);
  pinMode(aAntiHora, OUTPUT);
  pinMode(bHorario, OUTPUT);

  ledcAttachPin(ledRed, 4);
  ledcAttachPin(ledGreen, 5);
  ledcAttachPin(ledBlue, 6);


   pinMode(botao, INPUT_PULLUP);
  
  controleMotores(0,0);
  LedRGB(255, 0, 0,0,1);
//-----------------------multitarefas------------------
xTaskCreatePinnedToCore(
  tarefaRGB,
  "tarefaRGB",  
  2048,
  NULL,
  1,
  NULL,
  1);   
}

void loop(){
  //---para a musica caso esteja no bluetooth e verifica estado do led------
  if (!serialBt.hasClient()) {
    if(calibracaoAtiva == false) {
      estadoLed = 1;
      LedRGB(255, 0, 0,0,1);
    }else{
      if(onOff == true){
        estadoLed = 2;
      }else{
        estadoLed = 1;
      LedRGB(255, 255, 255,0,1);
      }
    }
  } else {
    estadoLed = 1;
    LedRGB(0, 0, 255,0,1);
  }

  //--------botão calibração---------------------------
  if(digitalRead(botao) == LOW && onOff ==false && manualPid == 0&& calibracaoAtiva==false) {
    calibracao();
  }
  //---verifica se o bluetooth está conectado e faz a troca das constantes ou troca para o modo manual--
  if(serialBt.available()){
    String EntradaSerial = serialBt.readStringUntil(' '); 
    Serial.println("Recebido: " + EntradaSerial);
    EntradaSerial.trim();

     if (EntradaSerial.length() >= 2) {
      String modo = EntradaSerial.substring(0, 2);

      if (modo == "MA") {
        manualPid = 1;
        Serial.println("Modo Manual ativado");
      } else if (modo == "PI") {
        manualPid = 0;
        Serial.println("Modo PID ativado");
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
    velocidadeMaximaA = (uint8_t)VMaxStr.toInt();
    velocidadeMaximaB = (uint8_t)VMaxStr.toInt();
    velocidadeBaseA = (uint8_t)VMinStr.toInt();
    velocidadeBaseB = (uint8_t)VMinStr.toInt();

    if (KpIndex >= 0 && KiIndex > KpIndex && KdIndex > KiIndex && manualPid==0) {
      String kpStr = EntradaSerial.substring(KpIndex+3, KiIndex);
      String kiStr = EntradaSerial.substring(KiIndex + 3, KdIndex);
      String kdStr = EntradaSerial.substring(KdIndex + 3, KrIndex);
      String krStr = EntradaSerial.substring(KrIndex + 3, VMaxIndex);
      
      Kp = kpStr.toFloat();
      Ki = kiStr.toFloat();
      Kd = (kdStr.toFloat())*2;
      Kr = krStr.toFloat();


      Serial.printf("Constantes atualizadas:\nKp = %.2f\nKi = %.2f\nKd = %.2f\nKr = %.2f\n", Kp, Ki, Kd,Kr);
    } 
    // ---------leitura e modo manual---------------------------
    if (manualPid == 1 && EntradaSerial.length() >= 3) {
      char comando = EntradaSerial.charAt(2);
      int DelIndex = EntradaSerial.indexOf("Del=");
      String DelStr = EntradaSerial.substring(DelIndex + 4,VMaxIndex);
      int Del = DelStr.toInt();
    
      switch (comando) {
        case 'F': // Frente
          controleMotores(velocidadeMaximaA, velocidadeMaximaB);
          Serial.println("Frente");
          if (Del > 0) {
            delay(Del);
            controleMotores(0, 0);
          }
          break;
    
        case 'T': // Trás
          controleMotores(-velocidadeMaximaA, -velocidadeMaximaB);
          Serial.println("Tras");
          if (Del > 0) {
            delay(Del);
            controleMotores(0, 0);
          }
          
          break;
    
        case 'L': // Esquerda
        controleMotores(velocidadeMaximaA, -velocidadeMaximaB);
          Serial.println("Esquerda");
          if (Del > 0) {
            delay(Del);
            controleMotores(0, 0);
          }
          break;
    
        case 'R': // Direita
          controleMotores(-velocidadeMaximaA, velocidadeMaximaB);
          Serial.println("Direita");
          if (Del > 0) {
            delay(Del);
            controleMotores(0, 0);
          }
          break;
    
        default: //Parar
          controleMotores(0, 0);
          Serial.println("Parado");
          break;
      }
    }

  }

  //-----botão liga e desliga do carro-------------------
  if(digitalRead(botao)==LOW && manualPid == 0&& calibracaoAtiva==true) {
    onOff = !onOff;
    if(onOff == true) {
      //------tempo até o carro ligar-----
      estadoLed = 1;
      LedRGB(255,0,0,300,3);
      LedRGB(255,127,0,300,1);
      LedRGB(0,255,0,300,1);
      Serial.println("corrida iniciada");
    }
    else {
      //------tempo até o carro desligar---
      estadoLed = 1;
      LedRGB(255,0,0,200,2);
      Serial.println("corrida Acabou");
    }
  }

  //-----controle liga e desliga do carro--------------------
  if (onOff == true&& manualPid == 0&& calibracaoAtiva == true) {
    //musicaSelecionada = "/Corrida.wav";
    // tocar = true;
    controle_PID();
  }else{
    if (manualPid == 0) {
      controleMotores(0, 0);
      //pararMusica();
    }
  }
}
//--------------------------------------------
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
        for (int i = 0; i <= 255; i += 5) {
          if (estadoLed != 3) break;
          LedRGB(i, i, 0, 0, 1);
          vTaskDelay(10 / portTICK_PERIOD_MS);
        }
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


void controleMotores(int motorA, int motorB){
  if(motorA > 0){
    digitalWrite(aHorario, HIGH);
    digitalWrite(aAntiHora, LOW);
    ledcWrite(0, motorA);
    // ledcWrite(0, motorA);
    // ledcWrite(2, 0);
  }else if(motorA < 0){
    digitalWrite(aHorario, LOW);
    digitalWrite(aAntiHora, HIGH);
    ledcWrite(0, -(motorA));
  }else{
    digitalWrite(aHorario, LOW);
    digitalWrite(aAntiHora, LOW);
    ledcWrite(0, 0);
  }
  if(motorB > 0){
    digitalWrite(bHorario, HIGH);
    digitalWrite(bAntiHora, LOW);
    ledcWrite(1, motorB);
  }else if(motorB < 0){
    digitalWrite(bHorario, LOW);
    digitalWrite(bAntiHora, HIGH);
    ledcWrite(1, -(motorB));
  }else{
    digitalWrite(bHorario, LOW);
    digitalWrite(bAntiHora, LOW);
    ledcWrite(1, 0);
  }
}

void errosPassados (int error)
{
  for (int i = 9; i > 0; i--)
      erros[i] = erros[i-1];
  erros[0] = error;
}


int errosSomatorio(int index, int Abs) {
  int sum = 0;
  for (int i = 0; i < index; i++) {
    if (Abs == 0)
      sum += abs(erros[i]);  
    else
      sum += erros[i];       // para I
  }
  return sum;
}

void controle_PID(){
  //-----------------leitura dos sensores-----------------
uint16_t posicao = qtr.readLineBlack(sensorValores);
int erro = 3500 - posicao;
errosPassados(erro);
//se está sobre a linha preta o valor do sensor é 1000
//se está fora da linha preta o valor do sensor é abaixo de 100
int ValorMaximoSensores = sensorValores[0]+sensorValores[1]+sensorValores[2]+sensorValores[3]+sensorValores[4]+sensorValores[5]+sensorValores[6]+sensorValores[7];
//------------verificação linha chegada ---------------
// if(ValorMaximoSensores >= 7500){
//   contadorParada++;
//   if(contadorParada ==10){
//   onOff = false;
//   controleMotores(0, 0);
//   estadoLed = 1;
//   LedRGB(255, 0, 0,300,2); 
//   }
// }else{
//   contadorParada = 0;
// }
//------------------verificação se saiu da linha-----------------
if(ValorMaximoSensores <= 700){
  contadorSaiuDaLinha++;
  if(ultimoErro > 0&& contadorSaiuDaLinha > 10){
    controleMotores(velocidadeMaximaA, 0);
  }else{
    controleMotores(0, velocidadeMaximaB);
  }
}else{
  contadorSaiuDaLinha = 0;
}
//------------------verificação 90 graus(teste)----------------

//-----------------ajuste de tolerancia em linha reta-----------------
if (abs(erro) < 1000*Kr) {  
  erro = 0;
}
//-----------------PID-----------------
P = erro;
I = errosSomatorio(5, 0);
D = erro - ultimoErro;
ultimoErro = erro;
//-------------------controle de velocidade-----------------
int VelocidadeMotor = (P*Kp) + (I*Ki) + (D*Kd);
int VelocidadeA = velocidadeBaseA - VelocidadeMotor;
int VelocidadeB = velocidadeBaseB + VelocidadeMotor;
if (VelocidadeA > velocidadeMaximaA) {
  VelocidadeA = velocidadeMaximaA;
}
if (VelocidadeB > velocidadeMaximaB) {
  VelocidadeB = velocidadeMaximaB;
}
if (VelocidadeA < -velocidadeMaximaA) {
  VelocidadeA = -velocidadeMaximaA;
}
if (VelocidadeB < -velocidadeMaximaB) {
  VelocidadeB = -velocidadeMaximaB;
}
controleMotores(VelocidadeA, VelocidadeB);
Serial.printf("VA=%d VB=%d Pos=%d\n", VelocidadeA, VelocidadeB, posicao);
}


void calibracao() {
   estadoLed = 3;
  Serial.println("Calibrando...");
  for (uint16_t i = 0; i < 500; i++) {
    qtr.calibrate();
  }
  Serial.println("Calibracao concluida");
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


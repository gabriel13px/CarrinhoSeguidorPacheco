#include <Arduino.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"
QTRSensors qtr;
const uint8_t QuantSensores = 8;
uint16_t SensorValores[QuantSensores];
float Kp = 0.64;
float Ki = 0;
float Kd = 0.54;
int P;
int I;
int D;
int UltimoErro = 0;
boolean OnOff = false;
boolean calibracaoAtiva = false;
BluetoothSerial SerialBT;
int ManualPid = 0;// 0 = PID, 1 = Manual
int TempoParada = 300;

 uint8_t VelocidadeMaximaA = 224;
 uint8_t VelocidadeMaximaB = 224;
 uint8_t VelocidadeBaseA = 214;
 uint8_t VelocidadeBaseB = 214;
 

// os sensores devem ser colocados na ordem
//17,18,13,14,27,26,25,33 gpios usados sensores  -entrada digital
//32 para buzzer -saida pwm
//5 para botao - input pullup
//2,15,4 para led - saida pwm
//19,21,22,23 para ponte H - saidas pwm
//16 ir led

int AHorario =19;// esquerda horario
int AAntiHora =21;//esquerda anti horario
int BHorario =22;//direita horario 
int BAntiHora  =23;//direita anti horario
//int LedIndicador = 2;

int LedRed = 15;
int LedGreen = 2;
int LedBlue = 4;

int botao = 5; 

int botaoCalibracao = 12;

void calibracao();
void frente_freio(int motorA, int motorB);
void Controle_PID();
void PiscaPisca(int tempo, int vezes);
void LedRGB(int r, int g, int b, int tempo,int loop);

void setup(){
  Serial.begin(115200);
  SerialBT.begin("Pacheco"); 
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
  qtr.setSensorPins((const uint8_t[]){17,18,13,14,27,26,25,33}, QuantSensores);
  qtr.setEmitterPin(16); // IR Pin

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
  // pinMode(LedIndicador, OUTPUT);
  delay(500);
  
  // boolean Espera = false;
  // while (Espera == false) {
  //   if(digitalRead(botao) == LOW) {
  //     calibracao();
  //     Espera = true;
  //   }
  // }
  frente_freio(0,0);
}

void loop(){
  if(digitalRead(botaoCalibracao) == LOW && OnOff ==false) {
    calibracao();
  }
  //verifica se o bluetooth está conectado e faz a troca das constantes outroca para o modo manual
  if(SerialBT.available()){
    String EntradaSerial = SerialBT.readStringUntil(' '); // Lê até Enter
    Serial.println("Recebido: " + EntradaSerial);
    // SerialBT.println("Recebido: " + EntradaSerial);
    EntradaSerial.trim();

     if (EntradaSerial.length() >= 2) {
      String modo = EntradaSerial.substring(0, 2);

      if (modo == "MA") {
        ManualPid = 1;
        Serial.println("Modo Manual ativado");
        SerialBT.println("Modo Manual ativado");
      } else if (modo == "PI") {
        ManualPid = 0;
        Serial.println("Modo PID ativado");
        SerialBT.println("Modo PID ativado");
      }
    }
    
    //leitura e atualização de constantes
    int KpIndex = EntradaSerial.indexOf("Kp=");
    int KiIndex = EntradaSerial.indexOf("Ki=");
    int KdIndex = EntradaSerial.indexOf("Kd=");
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
      String kdStr = EntradaSerial.substring(KdIndex + 3, VMaxIndex);
      
      Kp = kpStr.toFloat();
      Ki = kiStr.toFloat();
      Kd = kdStr.toFloat();

      Serial.printf("Constantes atualizadas:\nKp = %.2f\nKi = %.2f\nKd = %.2f\n", Kp, Ki, Kd);
      SerialBT.printf("Constantes atualizadas:\nKp = %.2f\nKi = %.2f\nKd = %.2f\n", Kp, Ki, Kd);
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
          SerialBT.println("Frente");

          if (Del > 0) {
            delay(Del);
            frente_freio(0, 0);
          }
          break;
    
        case 'T': // Trás
          frente_freio(-VelocidadeMaximaA, -VelocidadeMaximaB);
          Serial.println("Tras");
          SerialBT.println("Tras");
          if (Del > 0) {
            delay(Del);
            frente_freio(0, 0);
          }
          
          break;
    
        case 'L': // Esquerda
        frente_freio(VelocidadeMaximaA, -VelocidadeMaximaB);
          Serial.println("Esquerda");
          SerialBT.println("Esquerda");
          if (Del > 0) {
            delay(Del);
            frente_freio(0, 0);
          }
          break;
    
        case 'R': // Direita
          frente_freio(-VelocidadeMaximaA, VelocidadeMaximaB);
          Serial.println("Direita");
          SerialBT.println("Direita");
          if (Del > 0) {
            delay(Del);
            frente_freio(0, 0);
          }
          break;
    
        default: 
          frente_freio(0, 0);
          Serial.println("Parado");
          SerialBT.println("Parado");
          break;
      }
    }

  }

  //botão liga e desliga do carro
  if(digitalRead(botao)==LOW && ManualPid == 0&& calibracaoAtiva==true) {
    OnOff = !OnOff;
    if(OnOff == true) {
      //tempo até o carro ligar
      // PiscaPisca(400, 4);
      LedRGB(255,0,0,300,3);
      LedRGB(255,127,0,300,1);
      LedRGB(0,255,0,300,1);
      Serial.println("corrida iniciada");
      SerialBT.println("corrida iniciada");
    }
    else {
      
      //tempo até o carro desligar
      // PiscaPisca(200, 2);
      LedRGB(255,0,0,200,2);
      Serial.println("corrida Acabou");
      SerialBT.println("corrida Acabou");
    }
  }

  //controle liga e desliga do carro
  if (OnOff == true&& ManualPid == 0&& calibracaoAtiva == true) {
    Controle_PID();
  }else{
    if (ManualPid == 0) {
      frente_freio(0, 0);
    }
  }
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



//controle do PID
void Controle_PID(){
uint16_t posicao = qtr.readLineBlack(SensorValores);
//------------verificação linha chegada ---------------
//se está sobre a linha preta o valor do sensor é 1000
//se está fora da linha preta o valor do sensor é abaixo de 100
int ValorMaximoSensores = SensorValores[0]+SensorValores[1]+SensorValores[2]+SensorValores[3]+SensorValores[4]+SensorValores[5]+SensorValores[6]+SensorValores[7];
if(ValorMaximoSensores == 7000){
  
}
//--------------------------------------------------------
int erro = 3500 - posicao;
//Serial.println(posicao);
P = erro;
I = I + erro;
D = erro - UltimoErro;
UltimoErro = erro;

//vai ser necessario alterar a logica dos motores
int VelocidadeMotor = (P*Kp) + (I*Ki) + (D*Kd);
int VelocidadeA = VelocidadeBaseA - VelocidadeMotor;
int VelocidadeB = VelocidadeBaseB + VelocidadeMotor;


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
//SerialBT.printf("VA=%d VB=%d Pos=%d\n", VelocidadeA, VelocidadeB, posicao);

}


void calibracao() {
  Serial.println("Calibrando...");
  SerialBT.println("Calibrando...");
  // digitalWrite(LedIndicador, HIGH);
  LedRGB(255, 0, 0,0,1);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  Serial.println("Calibracao concluida");
  SerialBT.println("Calibracao concluida");
  delay(1000);
  // digitalWrite(LedIndicador, LOW);
  LedRGB(0, 0, 0,0,1);
  calibracaoAtiva = true;
}

// pisca o led indicador
// void PiscaPisca(int tempo, int vezes) {
//   for (int i = 0; i < vezes; i++) {
//     digitalWrite(LedIndicador, HIGH);
//     delay(tempo);
//     digitalWrite(LedIndicador, LOW);
//     delay(tempo);
//   }
// }

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
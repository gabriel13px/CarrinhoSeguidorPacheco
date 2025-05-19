#include <Arduino.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"
//parte de codigo de musica que não funcionou---------------------------------
// #include <SPIFFS.h> 
// #include "esp_timer.h"  
// #include "esp_task_wdt.h"
// File audio;
// bool tocando = false;
// bool parar = false;
// bool tocar = false;
// String musicaSelecionada = "/Corrida.wav";
// //int Contadorwatchdogs = 0;
// int pinoDAC = 26;
//----------------------------------------------------------------------------
int contadorParada = 0;
int contadorSaiuDaLinha = 0;
int erros[10] = {0,0,0,0,0,0,0,0,0,0};
int estadoLed= 1;

QTRSensors qtr;
const uint8_t quantSensores = 8;
uint16_t sensorValores[quantSensores];
BluetoothSerial serialBt;
float Kp = 0.48;
float Ki = 0.06;
float Kd = 2;
float Kr = 0;
int P,I, D, R;
int ultimoErro = 0;
boolean onOff = false;
boolean calibracaoAtiva = false;
int manualPid = 0;// 0 = PID, 1 = Manual
 uint8_t velocidadeMaximaA = 255;
 uint8_t velocidadeMaximaB = 255;
 uint8_t velocidadeBaseA = 255;
 uint8_t velocidadeBaseB = 255;
 //----------pinos do esp--------------------------------------
// os sensores devem ser colocados na ordem
//17,18,13,14,27,25,33,32 gpios usados sensores -entrada digital
//26 para buzzer -saida pwm
//5,12 para botao - input pullup
//2,15,4 para led - saida pwm
//19,21,22,23 para ponte H - saidas pwm
//16 ir led
//-------------------------------------------------------------
int aHorario =19;// esquerda horario
int aAntiHora =21;//esquerda anti horario
int bHorario =22;//direita horario 
int bAntiHora  =23;//direita anti horario
int ledRed = 15;
int ledGreen = 2;
int ledBlue = 4;
int botaoInicio = 5; 
int botaoCalibracao = 12;
// TaskHandle_t musicaTaskHandle = NULL;
//-----------prototipagem das funções----------
void calibracao();
void controleMotores(int motorA, int motorB);
void controle_PID();
void LedRGB(int r, int g, int b, int tempo,int loop);
// void pararMusica();
// void tarefaMusica(void*param);
void tarefaRGB(void*param);
// void delay_us_custom(uint64_t us);
//---------------------------------------------
void setup(){
  // SPIFFS.begin(true);
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
  qtr.setTimeout(2587);

  ledcAttachPin(aHorario, 0);
  ledcAttachPin(bAntiHora, 1);
  ledcAttachPin(aAntiHora, 2);
  ledcAttachPin(bHorario, 3);

  ledcAttachPin(ledRed, 4);
  ledcAttachPin(ledGreen, 5);
  ledcAttachPin(ledBlue, 6);


   pinMode(botaoInicio, INPUT_PULLUP);
   pinMode(botaoCalibracao, INPUT_PULLUP);
  
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
  // xTaskCreatePinnedToCore(
  //   tarefaMusica,
  //   "tarefaMusica",   
  //   19000,
  //   NULL,
  //   1,
  //   &musicaTaskHandle,
  //   0);     
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
    // if (eTaskGetState(musicaTaskHandle) == eSuspended) {
    //   vTaskResume(musicaTaskHandle); 
    //   Serial.println("Bluetooth desconectado - Retomando a música.");
    // }
  } else {
    estadoLed = 1;
    LedRGB(0, 0, 255,0,1);
    // if (eTaskGetState(musicaTaskHandle) != eSuspended) {
    //   vTaskSuspend(musicaTaskHandle);  
    //   Serial.println("Bluetooth pareado - Suspendendo a música.");
    // }
  }

  //--------botão calibração---------------------------
  if(digitalRead(botaoCalibracao) == LOW && onOff ==false && manualPid == 0) {
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
      Kd = kdStr.toFloat();
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
  if(digitalRead(botaoInicio)==LOW && manualPid == 0&& calibracaoAtiva==true) {
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
//-----------------funções de musica-------------------
// void delay_us_custom(uint64_t us) {
//   uint64_t start = esp_timer_get_time();
//   while ((esp_timer_get_time() - start) < us) {
//     esp_task_wdt_reset(); 
//   }
// }

// void tarefaMusica(void*param){
//   while (true) {
//     if (tocar && !tocando) {
//       audio = SPIFFS.open(musicaSelecionada, "r");
//       if (!audio) {
//         Serial.println("Erro ao abrir o arquivo");
//         tocar = false;
//         continue;
//       }
//       Serial.print(" Tocando: ");
//       Serial.println(musicaSelecionada);
//       audio.seek(46); 
//       tocando = true;
//       parar = false;
//       while (audio.available() && !parar) {
//         byte valor = audio.read();
//         dacWrite(pinoDAC, valor);
//         delay_us_custom(70);
//         if (++Contadorwatchdogs >= 100) {
//           Contadorwatchdogs = 0;
//           vTaskDelay(1);  
//         }
//       }
//       audio.close();
//       tocando = false;
//       tocar = false;
//       Serial.println(" Fim da música");
//     }
//      vTaskDelay(10 / portTICK_PERIOD_MS); 
//   }
// }

// void pararMusica() {
//   parar = true;
//   delay(10); 
//   if (audio) audio.close();
//   tocando = false;
//   Serial.println(" Música parada");
// }

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
if(ValorMaximoSensores >= 7500){
  contadorParada++;
  if(contadorParada ==10){
  onOff = false;
  controleMotores(0, 0);
  estadoLed = 1;
  LedRGB(255, 0, 0,300,2);
  }
}else{
  contadorParada = 0;
}
//musicaSelecionada = "/Chegada.wav";
// tocar = true;
//PararMusica();
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
//-----------------ajuste de tolerancia em linha reta-----------------
if (abs(erro) < 100) {  
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
  for (uint16_t i = 0; i < 400; i++) {
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


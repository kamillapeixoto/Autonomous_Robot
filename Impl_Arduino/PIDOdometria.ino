#include <digitalWriteFast.h>
#include <PID_v1.h>
#include <AFMotor.h>
#include <stdio.h>
#include <Utility.h>



// -*- Controle de Velocidade -*-
AF_DCMotor motorD(2);             //Seleciona o motor direito na porta 2
AF_DCMotor motorE(1);             //Seleciona o motor esquerdo na porta 1


int ps2D = 32, ps2E = 33;         //Pinos do Arduino conectados ao encoder
int countE = 0, countD = 0;
double velD = 0, velE = 0; // Velocidade em rad/s
int s1D = 0, s1E = 0;          // Primeiros valores lidos pelo encoder

unsigned long t = 0; unsigned long told = 0;  // Tempo real 
unsigned long tnew = 0, tRPI = 350000, espera = 0; // em us
double tvel = 50000;
double tPID = 100000;      // Periodo de amostragem tPID em us
int constLoop = 10; // constLoop = tRPI/tPID - tempo de comunicaçao
unsigned long tserial = tRPI - tPID*constLoop;
unsigned long esperaTot = 0, toldTot = 0;

// -*- PID -*-
double Kpe = 3, Kie = 8.5, Kde = 0;   // Ganhos da roda esquerda
double Kpd = 3, Kid = 8.3, Kdd = 0;   // Ganhos da roda direita
double SetpointD, InputD, EsfControleD,  SetpointE, InputE, EsfControleE; //  Variaveis relacionadas ao PID
double EsfControleD2 = 0, EsfControleE2 =0;

double EsfMinOND = 120, EsfMinOFFD = 200,EsfMinONE =120, EsfMinOFFE =180, velMinD = 0.8, velMinE = 0.8; // Valores minimos para que o robo nao pare
double fatorD = 1, fatorE = 1 ;

PID myPIDe(&InputE, &EsfControleE, &SetpointE, Kpe, Kie, Kde, DIRECT); // Declaraçao do PID esquerdo
PID myPIDd(&InputD, &EsfControleD, &SetpointD, Kpd, Kid, Kdd, DIRECT); // Declaraçao do PID direito

// -*- Odometria -*-

double r=0.034;                      //raio em metros
double l=0.147;                      //comprimento em metros

double x0=0, y0=0, theta0=0, theta; // posiçao inicial
double x, y;                        // posiçao atual
double x_est, y_est, theta_est;     // posiçao estimada

int dirD = 1, dirE =1;

// -*- Comunicação Serial -*-
String recebido = "";
String u1_str = "";
String u2_str = "";
double u1;
double u2;
int index =0;
int parar = 0;
 
float tin = 0, tout = 0;

// Vetores para enviar os dados
double x_vet [20],y_vet[20],theta_vet[20], velD_vet[20], velE_vet[20], EsfD_vet[20], EsfE_vet[20];
int amostras = 0;

void setup()
{
  Serial.begin(115200);
  
  // Parametros PID
  myPIDd.SetOutputLimits(-255, 255); myPIDe.SetOutputLimits(-255, 255);
  myPIDd.SetSampleTime(tPID); myPIDe.SetSampleTime(tPID);
  myPIDd.SetMode(AUTOMATIC); myPIDe.SetMode(AUTOMATIC);
  SetpointE = 0; SetpointD = 0;
  
  // Limpa buffer
  Serial.flush();

}

void loop()
{
 
  while(parar == 0){
      
    // -*- Controle de Velocidade -*-
    toldTot = micros();
    
    // -*- Odometria -*-
    // A posiçao enviada
    x_est     = x0+r/2*tRPI*cos(theta0)*(SetpointD + SetpointE)/1000000; // Divide por um milhao para transformar o periodo em s
    y_est     = y0+r/2*tRPI*sin(theta0)*(SetpointD + SetpointE)/1000000;
    theta_est = theta0+r/l*tRPI*(SetpointD - SetpointE)/1000000;
        
    // Envia dados para o Scilab calcular a nova referencia
    Serial.write(2);
    Serial.print(x_est);Serial.print(",");
    Serial.print(y_est);Serial.print(",");
    Serial.print(theta_est); 
    Serial.write(3);
    

  // PID e Odometria
  for (int count = 0; count <constLoop; count ++){
      told = micros();
      // -*- Controle de Velocidade -*-
  
      t = (micros() - told);
      
      // Conta os putPIDlsos durante tvel msegundos
      while (t < tvel){
          int s2D = digitalRead(ps2D);
          if (s2D != s1D) {
            s1D = s2D;
            countD = countD + 1;
          } // if s2D
          int s2E = digitalRead(ps2E);
          if (s2E != s1E) {
            s1E = s2E;
            countE = countE + 1;
          } //if s2E
                
          t = (micros() - told);
      } // while t<tveltPID

        // Calcula as velocidades e zera o contador
      velD = dirD*(countD * 157.08) / (t*1000);
      velE = dirE*(countE * 157.08) / (t*1000);
      countE = 0; countD = 0;


    // -*- PID -*-
     InputD = velD; InputE = velE;
     myPIDd.Compute(); myPIDe.Compute();
    

     //Escreve nos motores
     motorD.setSpeed(EsfControleD2); motorE.setSpeed(EsfControleE2);
     
      
 
    // -*- Odometria -*-        Referente ao ciclo do PID
    x    = x0+r/2*tPID*cos(theta0)*(velD + velE)/1000000; // Divide por um milhao para transformar o periodo em s
    y    = y0+r/2*tPID*sin(theta0)*(velD + velE)/1000000;
    theta = theta0+r/l*tPID*(velD - velE)/1000000;
    
    x0     = x; 
    y0     = y; 
    theta0 = theta;

    
    x_vet [amostras]    = x;
    y_vet [amostras]    = y;
    theta_vet[amostras] = theta;
    velD_vet[amostras]  = velD;
    velE_vet[amostras]  = velE;
    EsfD_vet[amostras]  = EsfControleD2;
    EsfE_vet[amostras]  = EsfControleE2;
    
    amostras = amostras +1;

    espera = tPID - (micros()-told);

    if (espera>0){
        delayMicroseconds(espera);  
    }// if (espera>0)
        
  } // for (int count = 0, i<constLoop, i++)
  
  
  
    // -*- Odometria -*-        Referente ao ciclo total
    x    = x0+r/2*tserial*cos(theta0)*(SetpointD + SetpointE)/1000000; // Divide por um milhao para transformar o periodo em s
    y    = y0+r/2*tserial*sin(theta0)*(SetpointD + SetpointE)/1000000;
    theta = theta0+r/l*tserial*(SetpointD - SetpointE)/1000000;
    
    x0     = x; 
    y0     = y; 
    theta0 = theta;

    
    x_vet [amostras]    = x;
    y_vet [amostras]    = y;
    theta_vet[amostras] = theta;
    velD_vet[amostras]  = velD;
    velE_vet[amostras]  = velE;
    EsfD_vet[amostras]  = EsfControleD2;
    EsfE_vet[amostras]  = EsfControleE2;
    
    amostras = amostras +1;
    
    
      
    // Le novas referencias
    byte lido = 0;
    while(lido == 0){
     tin = millis();
    // Le novas referencias
    if((Serial.available()>0)){
          recebido = leStringSerial();
          index = recebido.indexOf(',');
          if(index <= 0){ // Para quando recebe apenas uma variavel
              parar = 1;
              break;
          }else{
              u1_str = recebido.substring(0,index);
              u2_str = recebido.substring(index+1);
        
              SetpointD = atof(u1_str.c_str());
              SetpointE = atof(u2_str.c_str()); 
             
              dirD = sign(SetpointD);
              dirE = sign(SetpointE); 
               
              lido = 1;
              
            // Permite que o motor gire em ambos sentidos
             if (SetpointD > 0){
                 motorD.run(FORWARD);
                 dirD = 1;
             } else{
                 motorD.run(BACKWARD);
                 dirD = -1;
             }
               
             if (SetpointE > 0){
                 motorE.run(FORWARD);
                 dirE = 1;
             } else{
                 motorE.run(BACKWARD);
                 dirE = -1;
             } 
             
             
          } //else
          
     } // if((Serial.available()>0))
     tout = millis()- tin;
     if (tout > 20){
     lido = 1;
     }
    } //  while(lido == 0)
  
    
      
  
    esperaTot = tRPI - (micros()-toldTot);
    //Serial.println(micros()-told); 
    if (esperaTot>0){
        delayMicroseconds(esperaTot);
         
    }// if (espera>0)
    
    
  }//  while(parar == 0)
  
    // Se saiu do loop, desliga tudo
    motorD.run(RELEASE);motorE.run(RELEASE);
    SetpointD = 0;
    SetpointE = 0;
    
    // Se saiu do loop, desliga tudo
    motorD.run(RELEASE);motorE.run(RELEASE);
    SetpointD = 0;
    SetpointE = 0;
    
    // Se saiu do loop, desliga tudo
    motorD.run(RELEASE);motorE.run(RELEASE);
    SetpointD = 0;
    SetpointE = 0;
    
    // Envia os dados para conferencia
    for(int i = 0; i<amostras; i++){
      Serial.write(2);
      Serial.print(x_vet[i]);Serial.print(",");
      Serial.print(y_vet[i]);Serial.print(",");
      Serial.print(theta_vet[i]);Serial.print(","); 
      Serial.print(velD_vet[i]);Serial.print(",");
      Serial.print(velE_vet[i]);Serial.print(",");
      Serial.print(EsfD_vet[i]);Serial.print(",");
      Serial.println(EsfE_vet[i]);
      Serial.write(3);
    } //for(int i = 0; i<amostras; i+++){
    
    Serial.flush();
    Serial.end();
}  //void loop()



// Funçao que faz a leitura serial
String leStringSerial(){
  String conteudo = "";
  char caractere;  
  // Enquanto receber algo pela serial
  while(Serial.available() > 0) {
    // Lê byte da serial
    caractere = Serial.read();
    // Ignora caractere de quebra de linha
    if (caractere != '\n'){
      // Concatena valores
      conteudo.concat(caractere);
    }
    // Aguarda buffer serial ler próximo caractere
    delay(1);
  }  
  return conteudo;
}

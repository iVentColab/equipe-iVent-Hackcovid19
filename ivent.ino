//teste p usar no checando - manha -  16052020 - 2º dia do hackcovid19
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Addr, En, Rw, Rs, d4, d5, d6, d7, backlighpin, polarity

const int
    encoder=2,        // pino de interrupção do encoder
    M1 =5,            // out  - motor1  - pino 5 e 6 que levam informação de controle do arduino para a placa  controla o motor (módulo driver ponte H -L298N )
    M2 =6,          // out  - motor1   
    EnableM=7;        // out  - pino enable do motor. um PWM será aplicada nesse pino nos dando a possibilidade de controlar dessa forma a velocidade do motor  
 
// variáveis
int  
contador=0,                                   // conta as interrupções do encoder
     direcao=0,                                    // a indicação da direção do motor informa se o contador vai ser incrementado ou decrementado na interrupção 1
     sinalizador=0,
     prov, 
     S1=0,
     pp=0,
     primeiro=0;
 float    
     media=0,
     media2=0,
     agora=0,
     anterior=0;
 

     
     //setada durante a interrupção causada pelo no botao1 e retorna a 0 através tb do botao1 ou depois de alguns ciclos de operação(Ciclos)
void setup(){
  Serial.begin(9600);
   //display lcd 16x2
  lcd.begin (16,2);                              //SETA A QUANTIDADE DE COLUNAS(16) E O NÚMERO DE LINHAS(2) DO DISPLAY
  lcd.backlight();                               //LIGA O BACKLIGHT (LUZ DE FUNDO)                                 
 lcd.setBacklight(HIGH);                                      
  lcd.setCursor(5,0);  lcd.print("iVent");                     
 delay(1000); 
  lcd.clear(); 
  
 //botao - interrupção
  pinMode(encoder,INPUT);                        // pino2  - entrada do sinal do disco acoplado ao motor
 //interrupções
  attachInterrupt(digitalPinToInterrupt(encoder), interrompendo1, RISING);           // CONFIGURAÇÃO DA INTERRUPÇÃO NO PINO 2
  //pinos do motor
  pinMode(M1, OUTPUT); pinMode(M2, OUTPUT);  
  pinMode(EnableM, OUTPUT);                      // o pino enable habilita o funcionamento do motor. Pino pwm usado p controlar a velocidade do motor                                

}
void loop(){                          
              //vamos procurar uma "medida" de tempo p servir de parametro p comparação
if (pp==0){
 while (prov<20)                                                                             // 20 é um valor aleatorio so p botar o motor p girar
       {     
                lcd.setCursor(0,0);lcd.print("medindo");       
        
             direcao=1;  analogWrite(EnableM, 70);  digitalWrite(M1,1);  digitalWrite(M2,0);  //configura motor - velocidade arditraria, no codigo pra valer vai ser a velocidade escolhida pelo usuario
         
         if (sinalizador==1)                                  // chegou interupção?
            {    primeiro++; agora=millis(); media=agora-anterior; anterior = agora;}     // isso tem q acontecer pelo menos duas vezes p gerar o valor "anterior"
                 if (primeiro==2) {prov=25;}                   // depois de duas vezes... acabar com o loop  ,, poderiamos usar um goto pro proximo loop tb 
            }                                                  

          lcd.setCursor(0,0);lcd.print("media= ");    lcd.setCursor(9,0); lcd.print(media);   
 //JA TEMOS O VALOR MEDIO (media)-  saimos do loop acima com um valor p ter como parametro--
 // vamos comparar com o proximo
       
        while(prov<1000)  //aleatorio - so pra continuar rodando o motor
        {         
          if (sinalizador==1)    // ? interrupção  ---como o motor continua girando(configuações não foram alteradas), nova interrupção vai acontecer
            { 
              agora=millis(); media2=agora-anterior;      // mesma coisa. conseguir uma amostra do tempo entre interrupções
              //anterior = agora;    // só p constar ,essa linha pode até sair, sei lá
             lcd.setCursor(0,1);lcd.print("m2= ");    lcd.setCursor(5,1); lcd.print(media2);  
            //}      
//avaliação                  
float tolerancia=1.685;
float media3=media*tolerancia;
           if (media2> media3)  // comparando os dois tempos de intervalo entre interrupções, dando uma toleranciazinha
               {
                digitalWrite(M1,1);  digitalWrite(M2,1); prov=1000; pp=1;         // para o motor  - o valor 1000 é só p parar o while linha 43
                 lcd.setCursor(9,1);lcd.print("m3=");    lcd.setCursor(12,1); lcd.print(media3);  goto fim;
                }           
//caso contrario(inha 53), começa tudo de novo, ou seja, não chegamos na fresta do disco onde o tempo entre interrupções é bem maior
         }   
fim: 
 prov=0;
while(prov==0){
lcd.setCursor(15,0);lcd.print("X"); delay(300);
lcd.setCursor(15,0);lcd.print("O"); delay(300);}
        

           
}           
}
}   
              
 void interrompendo1()                    // encoder pino2  // função de interrupção1         
          {   sinalizador=1;
       if (direcao==1){contador ++;} 
       if (direcao==2){contador--;}}               


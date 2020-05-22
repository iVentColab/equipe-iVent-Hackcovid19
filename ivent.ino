//modificação noite 22052020 -

#include <NewPing.h>                     // biblioteca do sensor ultrasonico

#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Addr, En, Rw, Rs, d4, d5, d6, d7, backlighpin, polarity

#include <Thermistor.h> //INCLUSÃO DA BIBLIOTECA -LEITURA DE TEMPERATURA NO sensorTemp1 A3
 
Thermistor temp(3); //VARIÁVEL DO TIPO THERMISTOR, INDICANDO O PINO ANALÓGICO (A3) EM QUE O TERMISTOR ESTÁ CONECTADO

//#include <NTC_Thermistor.h>
//#include <AverageThermistor.h>

//Pinos digitais---------------------------

#define    Peep1            2            // out  - PEEP ELETRONICO  info vai p (módulo driver ponte H -L298N )
#define    Peep2            3            // out  -              ""
#define    Peep3            4
#define    EnablePeep5      5            // out  enable do PEEP ELETRONICO
#define    EnablePeep6      6            // out  enable do PEEP ELETRONICO
#define    M1               7            // out  - motor1  - pino 5 e 6 que levam informação de controle do arduino para a placa  controla o motor (módulo driver ponte H -L298N )
#define    M2               8            // out  - motor1   
#define    EnableM          9            // out  - pino enable do motor um PWM será aplicada nesse pino nos dando a possibilidade de controlar dessa forma a velocidade do motor  
#define    Ambu            10            // in - sensor óptico reflexivo (infravermelho) analógico - informa ao sistema se há um ambu no aparelho ou se está fora de posição                              
#define    Ambu11vcc       11
#define    Ambu12gnd       12
#define    encoder         13            // pino ligado a um sensor optico - conta os passos do motor atraves de um disco-encoder
#define    encoder14vcc    14
#define    encoder15gnd    15
#define    encoderPzero    16            // pino ligado a um sensor optico - avisa quando chega ao ponto zero
#define    encPzero17vcc   17
#define    encPzero18gnd   18 
#define    botaoA          19            // pino de interrupção pelo botao - quando acionado equipamento entra em estado de edição e retorna atraves de int pelo timer  
//#define    SDA             20            //PINOS DA PLACA I2C
//#define    SCL             21            //PINOS DA PLACA I2C     
#define    botaoA22gnd     22 
#define    ledEdit         23            // led  sinalizador do modo de edição dos parametros   
#define    ledEdit24gnd    24
#define    ledAlarme       25            // led ALTO BRILHO sinalizador de alarme  
#define    ledAlarme26gnd  26       
#define    botaoB          28            // responsável por sinal que instrui o sistema que a ligação da alimentação não veio de um recente queda/pico
#define    botaoB29gnd     29 
#define    buzzer          30            // aviso sonoro
#define    buzzer31gnd     31
#define    TRIGGER         32            // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define    ECHO            33            // Arduino pin tied to echo pin on the ultrasonic sensor.
#define    sonar34vcc      34
#define    sonar35gnd      35      
#define    pots36vcc       36   
#define    pots37gnd       37
#define    senTempA38vcc   38
#define    senTempA39gnd   39 
#define    senTempB40vcc   40
#define    senTempB41gnd   41 
#define    sCorrente42vcc  42
#define    sCorrente43gnd  43 
    
// pinos analógicos
#define    potFR           A0        // potenciometro responsável pela definição da velocidade do movimento do braço  - Knob AZUL
#define    potVC           A1        //    "              "        "      "      do curso do braço (VOLUME de ar injetada no paciente)  - Knob AMARELO 
#define    potIE           A2        //    "              "        "      "      da relação chamada - insp/exp ou I/E - Knob VERDE                    
#define    sensorTemp1     A3
//#define    sensorTemp2     A4
#define    sensorCorrente  A5
#define    sensorfluxo     A6
#define    sensorpressao   A7


#define    MAX_DISTANCE    120                   // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER, ECHO, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


// variáveis
int leituraAmbu,                                 // variável q sinaliza se há um Ambu no equipamento.
    leituraIE,leituraVC,leituraFR,               // valores analógico lido nos potenciometros              
    valorVC,valorIE, valorFR,                    // valor parametrizado da leitura feita nos potenciomentros                  
    valorV,                                      // é uma variante do valor V
    valorV2,
    guardadoVC = 0, 
    guardadoIE = 0,
    guardadoFR = 0,                              // variáveis  guardam o valor anterior de C,V eF                          
    marca=0,                                     // variável - indica se o programa acabou de iniciar ou não
    limiteUp = 30,                               // ponto zero do equipamento
    contador=0,                                  // conta as interrupções do encoder
    espera=0,                                    // variavel que guarda o resultado de calculos de tempo relativos à relação Insp/Exp              
    Ciclos=2,                                    // durante quantos ciclos do equip(respirações) a janela de edição de parametro fica abeerta disponivel depois de ter sido solicitada pelo botaoA
    nCiclos=0,                                   // contagem de ciclos p fechamento da janela de configuração do equip - estratégia de prevençao p manipuaçao acidental dos parametros nos pots
    direcao=0,                                   // a indicação da direção do motor informa se o contador vai ser incrementado ou decrementado na interrupção 1
    sinalizador=0, 
    Peep,                                        // valor da velocidade (pwm) do soprador p manter o peep do paciente
    cracha=0,
    S1=0, 
    brilho=7,
    Zero=0,
    TemperaturaAlarme=50,
    TemperaturaDesliga=80,
    control = 0;
    
//const double 
 //   celsius;
 //   temperature=0,
float     
    K,                                           // PD
    W,                                           // PD 
    frequencia=0,                              
    celsius,
    temperature=0;
    
unsigned long 
    esperando=0,
    t1 = 0, 
    t2 = 0, 
    t3 = 0, 
    t4 = 0, 
    t5=0 ;    //  variáveis associadas ao millis()   
//Thermistor* thermistor = NULL;

void setup(){

pinMode(Peep1, OUTPUT);            
pinMode(Peep2, OUTPUT);            
pinMode(Peep3, OUTPUT);
pinMode(EnablePeep5, OUTPUT);          
pinMode(EnablePeep6, OUTPUT);           
pinMode(M1, OUTPUT);            
pinMode(M2, OUTPUT);              
pinMode(EnableM, OUTPUT);              
pinMode(Ambu,INPUT);                                          
pinMode(Ambu11vcc, OUTPUT);
pinMode(Ambu12gnd, OUTPUT);
pinMode(encoder,INPUT);   
pinMode(encoder14vcc, OUTPUT);
pinMode(encoder15gnd, OUTPUT);
pinMode(encoderPzero,INPUT);                 
pinMode(encPzero17vcc, OUTPUT);   
pinMode(encPzero18gnd , OUTPUT);
pinMode(botaoA,INPUT_PULLUP);     
//pinMode(SDA             
//pinMode(SCL     
pinMode(botaoA22gnd , OUTPUT);       
pinMode(ledEdit, OUTPUT);            
pinMode(ledEdit24gnd, OUTPUT);   
pinMode(ledAlarme, OUTPUT);         
pinMode(ledAlarme26gnd, OUTPUT);         
pinMode(botaoB,INPUT_PULLUP);          
pinMode(botaoB29gnd, OUTPUT);      
pinMode(buzzer, OUTPUT);                     
pinMode(buzzer31gnd, OUTPUT);     
//pinMode(TRIGGER                   
//pinMode(ECHO                       
pinMode(sonar34vcc, OUTPUT);      
pinMode(sonar35gnd, OUTPUT);          
pinMode(pots36vcc , OUTPUT);        
pinMode(pots37gnd, OUTPUT);       
pinMode(senTempA38vcc, OUTPUT);   
pinMode(senTempA39gnd, OUTPUT);   
pinMode(senTempB40vcc, OUTPUT);  
pinMode(senTempB41gnd, OUTPUT);  
pinMode(sCorrente42vcc, OUTPUT);
pinMode(sCorrente43gnd, OUTPUT); 
pinMode(potFR,INPUT);                                             
pinMode(potVC,INPUT);                                           
pinMode(potIE,INPUT);                                                              
pinMode(sensorTemp1,INPUT);                                           
//pinMode(sensorTemp2,INPUT);                                           
pinMode(sensorCorrente,INPUT);                                           
pinMode(sensorfluxo,INPUT);                                           
pinMode(sensorpressao,INPUT);                                              

attachInterrupt(digitalPinToInterrupt(botaoA), interrompendo1, RISING);            // CONFIGURAÇÃO DA INTERRUPÇÃO NO PINO 3
lcd.begin (16,2);                              //SETA A QUANTIDADE DE COLUNAS(16) E O NÚMERO DE LINHAS(2) DO DISPLAY
lcd.backlight();                               //LIGA O BACKLIGHT (LUZ DE FUNDO)                                
Serial.begin(9600);  
                                          
digitalWrite(Ambu11vcc,1);
digitalWrite(Ambu12gnd,0);
digitalWrite(encoder14vcc,1);
digitalWrite(encoder15gnd,0);                
digitalWrite(encPzero17vcc,1);   
digitalWrite(encPzero18gnd ,0);
digitalWrite(botaoA22gnd ,0);                             
digitalWrite(ledEdit24gnd,0);          
digitalWrite(ledAlarme26gnd,0);               
           
digitalWrite(botaoB29gnd,0);                         
digitalWrite(buzzer31gnd,0);                         
digitalWrite(sonar34vcc,1);      
digitalWrite(sonar35gnd,0);          
digitalWrite(pots36vcc ,1);        
digitalWrite(pots37gnd,0);       
digitalWrite(senTempA38vcc,1);   
digitalWrite(senTempA39gnd,0);   
digitalWrite(senTempB40vcc,1);  
digitalWrite(senTempB41gnd,0);  
digitalWrite(sCorrente42vcc,1);
digitalWrite(sCorrente43gnd,0);                                                               


//iniciando o display c logo do iVent e piscadinha do led - só p tirar onda
  lcd.setBacklight(HIGH);                                      
  lcd.setCursor(5,0);  lcd.print("iVent");                     

  analogWrite(ledEdit, 0);  
  delay(300);
  analogWrite(ledEdit, brilho);
  digitalWrite(buzzer,1);
  delay(300); 
  
  digitalWrite(ledEdit,1); 
  digitalWrite(buzzer,0);
  delay(300); 
  analogWrite(ledEdit, brilho);  
  delay(1500); 
  lcd.clear();                
  }

byte cedilha[] = {
  B01111,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B01111,
  B01100
};

void loop()
{  
       if (marca==0)                                            // verificacão inicial do aparelho - acontece só uma vez
          {
          checandoQueda();
          Configureme();      // S1=1                           // configurações (braço parado inicialmente)                                    
          checandoPzero();                                           // checagem de posição do braço    
          Temperaturas();                                        // checa temperatura do motor e da placa controladora do motor
          marca=1;
          cracha=1;
          }                                                                                     
          
          AloAmbu();                                         // função q monitora o posicionamento correto do Ambu no aparelho

          atualiza();                                          // se a janela de edição estiver aberta atualiza os parametros
       
       if (S1==1&& nCiclos<= Ciclos)
       {
        nCiclos++;
        goto inicio; 
        }                               // conta Ciclos ---(S1 inicia 0) e se botaoA acionado S1=1, ou seja cod fica na linha 115 (tem um while no atualiza) esperando configuração nos pots
       if (S1==1&& nCiclos> Ciclos) 
       {
        nCiclos=0; 
        S1=0; 
        analogWrite(ledEdit, 0);            // fecha janela S1=0  
        } 
        
 inicio:
 
//.........................................INSPIRAÇÃO DO PACIENTE - em 2 fases A e B  

 //faseA
       
       t1 = millis();
          analogWrite(EnableM, valorV);                                  // configur motor          
          digitalWrite(M1,0);                                            // configur motor 
          digitalWrite(M2,1);                                            // configur motor            
       while ( contador< valorVC/2)                                      // enquanto o braço não atingir metade do curso escolhido
       {  
             // checaUltrasom              //verifica posição
             if   (digitalRead(encoder) == 0)                            // codigo luigi
             {
                if(control == 0)
                {
                 contador ++;
                }
             control = 1;
             } 
             else 
                 { control = 0;}   
             
             atualiza();                 // da uma passadinha p atualizar o display de leds
                    
       }
                                                                           
                                                                                              
 
 
  //faseB
       
   //PD       analogWrite(EnableM, valorV2/1.15);                                 
             
             analogWrite(EnableM, valorV2);                               //configura nova velocidade do motor p faseB - mais brando     
       while ( contador< valorVC)                                        // enquanto o braço não atingir o total do curso escolhido
       {     
             // checaUltrasom              //verifica posição
             if   (digitalRead(encoder) == 0)                            // codigo luigi
             {
                if(control == 0)
                {
                 contador ++;
                }
                control = 1;
             } 
             else 
                 { control = 0;}   
             atualiza();                                                 // da uma passadinha p atualizar o display de leds
       }                                 // acabado o loop paramos o motor                                                            

  // PD       K = 1 ;                      // K é o coeficiente de amortecimento, K grande = rápido desaceleração
 //  PD       W = valorVC;                  // W normaliza espaço temporal
  //´PD   while (contador< valorC)                                                             // depois de 95% do percurso aplica-se um comtrole derivativo da velocidade           
  // PD       {analogWrite(EnableM, valorV2 * exp((-valorC + W) * K));                       // velocidade é reduzida exponencialmente           
                     
          digitalWrite(M1,1);           // motor para
          digitalWrite(M2,1);
          // checaUltrasom              //verifica posição
//freadinha substituir por PID ou PD


          digitalWrite(M1,1); 
          digitalWrite(M2,0);  
          delay(130);        
     
          digitalWrite(M1,1); 
          digitalWrite(M2,1); 
          delay(40);   
                                                                                                                           
          t2=millis();     //encerrando contagem de descida
                         
         
//........................ MOTOR RETORNANDO A POSIÇÃO SUPERIOR - PONTO ZERO
        
          t3=millis();                                                                    //iniciando contagem de subida
 
// ativar motor brushles p enviar peep
 
// ....retornando braço p posição zero

           analogWrite(EnableM,100);    // configura motor                             // habilita o PWM com velocidade lenta a ser verificada empiricamente
           digitalWrite(M1,1);         // configura motor
           digitalWrite(M2,0);         // configura motor    
    
     while ( encoderPzero==0)
     {
      // checaUltrasom              //verifica posição
      }                                     // enquanto o braço não retornar pra próximo de zero
         
         digitalWrite(M1,1);  //desliga
         digitalWrite(M2,1);  //desliga
       
        //freadinha substituir por PID ou PD
        
           digitalWrite(M1,0); 
           digitalWrite(M2,1);  
           delay(130); 
           
           digitalWrite(M1,1); 
           digitalWrite(M2,1); 
           delay(1000);   
    
          digitalWrite(M1,1); 
          digitalWrite(M2,1);                                         // ao fim do loop desliga motor

           
//checagens por ciclo
       // checaUltrasom      verifica posição
       // verifica temperatura do motor
       // verifica temperatura da placa                 
      
          t4=millis();                                                                    //encerrando contagem de subida
 
 // tempo de espera                                                                       //delay de espera que completa o tempo faltante p fazer valer a relação i/e
 
          espera=(valorIE*(t2-t1)) - (t4-t3) ;         
          esperando=espera/20;                                                     
     
     for  (int i = 0; i <=20; i++) 
          {
          delay(esperando);
          atualiza();
          }                                                       // aqui poderiamos ter colocado um delay com o tempo esperando, mas preferimos dividi-lo arbitrariamente em 20 pedaços      
                                                                                          // a cada pedaço o procedimento da um pulinho em atualiza() e se a janela de edição estiver aberta atualiza os parametros e o display                                                                              
          t5=millis();

          frequencia= ((t5-t1)/1000);                                                       // valor em segundos p  apresentar no display

          if(frequencia==leituraFR)
          {
            lcd.setCursor(14,1);
            lcd.print(frequencia);
            }                                                                                // CHECA SE O VALOR SOLICITADO É IGUAL AO RESULTADO CONSEGUIDO
          else 
          {
            lcd.setCursor(14,1);lcd.print("ER");
          }                                                                                                                                                          
 
}

void AloAmbu()
{  

          int leituraAmbu = digitalRead(Ambu);
          
     if   (leituraAmbu!=1)
  { 
    while (leituraAmbu!=0)                                     // verifica se há Ambu no lugar certo, e se não estiver sinaliza
                                                 
          {lcd.clear();lcd.setCursor(0,0);
          lcd.print("AMBU AUSENTE");
          digitalWrite(buzzer,1);
          delay(400);
          lcd.clear();
          digitalWrite(buzzer,0);
          delay(400);
          leituraAmbu= digitalRead(Ambu);         
          }
   }
}
void Configureme()
{
          analogWrite(ledEdit,brilho);
          nCiclos=0; 
          S1=1;
          atualiza();
          marca=1;  
}

void checandoQueda()
{

  long contadorIni=0;
  lcd.clear();
esperando: 
     if (S1==0)           // então não ocorreu ainda a interupção botaoA -interrompendo1  
     {    if (contadorIni< 200000000){contadorIni++;}
          else
          {        
            lcd.setCursor(0,0);
            lcd.print(" PRECIONE BOTAO");
            lcd.setCursor(0,1);
            lcd.print("  PARA INICIAR  ");
            
            digitalWrite(buzzer,1);
            delay(1000);
            digitalWrite(buzzer,0);
            delay(1000);
          }
          goto esperando;       // entra em looping
     }
     else               // então já ocorreu a interupção botaoA -interrompendo1
     {
      S1=0;
     }                  // segue p o codigo
contadorIni=0;
lcd.clear();

}

void ultrasonico()
{    

 // tabela de correspondencia entre o valor escolhido pelo usuário e a medida em cm
       int valorPontoZero=0;
 
  /// sonar.ping_median(10);  faz 10 checagens e tira a média
    Serial.print(sonar.ping_cm());
    if (sonar.ping_cm()== valorPontoZero)
    {
     // tudo ok
      }
     if (sonar.ping_cm()!= valorPontoZero)
    {
     // deu ruim - 
     }
      
}
void Temperaturas()
{
    temperature = temp.getTemp(); //VARIÁVEL DO TIPO INTEIRO QUE RECEBE O VALOR DE TEMPERATURA CALCULADO PELA BIBLIOTECA
    celsius = temperature;     // se precisar fazer ajuste ou conversão é aqui que acontece
    Serial.print(celsius); //IMPRIME O TEXTO NO MONITOR SERIAL

// checando temperatura

  if (celsius > TemperaturaAlarme && celsius < TemperaturaDesliga) 
    {
    alarmeTemperatura();  //alarme de temperatura acima do normal .. funcionamento continua - porem alerta é disparado
    }

  if (celsius > TemperaturaDesliga)
    {
    desligandoAparelho();  // rotina de aviso que o aparelho vai ser desligado em tantos minutos. e que a manutenção deve ser acionada
    }
   
}

void alarmeTemperatura()
{
  //alarme de temperatura acima do normal .. funcionamento continua - porem alerta é disparado
}

void desligandoAparelho()
{
  // rotina de aviso que o aparelho vai ser desligado em tantos minutos. e que a manutenção deve ser acionada
}

void checandoPzero()
{                                                           // ROTINA DE INICIO DE TRABALHAO
          // escreve no display
 
          lcd.clear();
          lcd.setCursor(0,0);lcd.print(" REPOSICIONANDO ");                          
          lcd.setCursor(1,1);lcd.print("BRA");
          lcd.createChar(0, cedilha);
          lcd.setCursor(4,1);
          lcd.write(byte(0));
          lcd.print("O"); 
          lcd.setCursor(7,1);lcd.print("MECANICO");

           analogWrite(EnableM,70);    // configura motor                             // habilita o PWM com velocidade lenta a ser verificada empiricamente
           digitalWrite(M1,1);         // configura motor
           digitalWrite(M2,0);         // configura motor    
        long   contadore=0;
     while ( encoderPzero==0 && contadore<=2000000000) 
     {  
      // checaUltrasom              //verifica posição
      contadore++; 
      
      }                                     // enquanto o braço não retornar pra próximo de zero
         if (contadore> 2000000000)
         {  
          // checaUltrasom              //verifica posição
         digitalWrite(M1,1);  //desliga
         digitalWrite(M2,1);  //desliga
       
        //freadinha
        
           digitalWrite(M1,0); 
           digitalWrite(M2,1);  
           delay(130); 
           
           digitalWrite(M1,1); 
           digitalWrite(M2,1); 
           delay(1000);                                                    // espera um tempinho p iniciar o trabalho

          while(contadore=contadore)
          {
           lcd.clear();
           lcd.setCursor(0,0);lcd.print("ERRO - REINICIE "); 
           lcd.setCursor(0,1);lcd.print("   EQUIPAMENTO  ");
           digitalWrite (buzzer,1);
           delay(600);
           lcd.clear();
           digitalWrite (buzzer,0);
           delay(600);
           
          }

          }
             //freadinha
        
           digitalWrite(M1,0); 
           digitalWrite(M2,1);  
           delay(130); 
           // checaUltrasom              //verifica posição
           digitalWrite(M1,1); 
           digitalWrite(M2,1); 
           delay(1000);      
 }    
                                                              
void atualiza()
{                                                                          // ATUALIZA OS PARAMETROS 
       if (cracha==1)
        {
        goto pulo;
        }
        
       if (S1==1 && marca!=0) 
        {
        goto pulo;
        }
   
  while (S1==1 && marca==0)
    
 pulo:  {  
      
          leituraVC = analogRead(potVC);                       //valor ccorrente = volume
          valorVC = map(leituraVC, 0, 1023, 1, 40); 
          int valorDispVC = map(leituraVC, 0, 1023, 1, 1500);
          
          leituraIE = analogRead(potIE); 
          valorIE = map(leituraIE, 0, 1023, 1, 4);   
               
          leituraFR = analogRead(potFR); 
          valorFR = map(leituraFR, 0, 1023, 5,50);  
          valorV = map(valorFR, 80, 255, 80,200);                // calcuar, calibrar e verificar empiricamente 
          valorV2 = map(valorV, 80, 255, 80,160);    

             
      //  template do display  formato: exemplo
      //  lcd.setCursor(0,0);lcd.print("VC 1230 PF 20L/m"); 
      //  lcd.setCursor(0,1);lcd.print("I/E 1:2.45 FR 30" ); 
             
             lcd.setCursor(0,0);lcd.print("VC");
             lcd.setCursor(7,0);lcd.print("PF");
             lcd.setCursor(13,0);lcd.print("L/m");
             lcd.setCursor(0,1);lcd.print("I/E 1:");
             lcd.setCursor(11,1);lcd.print("FR"); 
             
 // ........... valor corrente - display 
                   
       if (valorVC != guardadoVC)
       {
        guardadoVC=valorVC;}; 
       }
       
       lcd.setCursor(4,0);
       if (valorVC <10  && guardadoVC>9 && guardadoVC<100){ lcd.print(" ");}
       if (valorVC <10  &&  guardadoVC>99) {lcd.print("  ");} 
       if (valorVC >10  && valorVC < 100 && guardadoVC>99);lcd.setCursor(4,0); lcd.print(" ");   
       lcd.setCursor(3,0); lcd.print(valorVC); 

  // ........... frequencia respiratoria - display 
        
       if (valorFR != guardadoFR)
       {
        guardadoFR=valorFR;
        } 
       lcd.setCursor(11,0);
       if (valorFR <10  && guardadoFR>9 && guardadoFR<100){lcd.print("  ");}    
       lcd.setCursor(10,0); lcd.print(valorFR); 

  //...................... relação ins/ex - display    
   
       if (valorIE != guardadoIE)
       {
        guardadoIE=valorIE;
        }
       lcd.setCursor(7,1);
       if (valorVC <10  && guardadoIE>9 && guardadoIE<100){ lcd.print(" ");}
       if (valorVC <10  &&  guardadoIE>99) {lcd.print("  ");} 
       if (valorVC >10  && valorVC < 100 && guardadoIE>99);lcd.setCursor(4,0); lcd.print(" ");   
       lcd.setCursor(6,1);lcd.print(valorIE);         

//...................... pico de fuxo PF - display
         
   //     lcd.setCursor(14,1);lcd.print(PF);                                     
  
      cracha=0;
}   
   

// interrupções
void interrompendo1()                   //botao pino3      // função de interrupção2
          { 
            if(S1==0)
            {
              S1=1;
              analogWrite(ledEdit, brilho);
              nCiclos=0; 
              }
              else
              {
                S1=0;
                analogWrite(ledEdit, 0);
               }
            delay(1000);
           }  



// portas de interrupção  2,3,18, 19 ,20,21 no arduino MEGA
// A PEEP ELETRONICA
// CALCULOS  E CALIBRAGEM DE PICO DE FLUXO ASSOCIADO A FREQUENCA RESPIRATORIA 
// sensor ultrasonico e seus calculos e calibragem
// resolver a questão de queda de corrente
// instalar a caixa gigante.
// finalizar e testar o encoder


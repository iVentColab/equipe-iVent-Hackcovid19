//modificação 3 -noite -  15052020 - 1º dia do hackcovid19

#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Addr, En, Rw, Rs, d4, d5, d6, d7, backlighpin, polarity

//Pinos digitais---------------------------
const int
    encoder=2,        // pino de interrupção do encoder
    botao1=3,         // pino de interrupção pelo botao - quando acionado equipamento entra em estado de edição e retorna atraves de int pelo timer
    buzzer= 4,        // aviso sonoro
    M1 =5,            // out  - motor1  - pino 5 e 6 que levam informação de controle do arduino para a placa  controla o motor (módulo driver ponte H -L298N )
    M2 =6,            // out  - motor1   
    EnableM=7,        // out  - pino enable do motor. um PWM será aplicada nesse pino nos dando a possibilidade de controlar dessa forma a velocidade do motor  
    C1=8,             // out  - soprador reversível.       PEEP ELETRONICO  info vai p (módulo driver ponte H -L298N )
    C2=9,             // out  -     "        "                  ""
    EnableC=10,       // out  enable do soprador reversivel     ""
    ledEdit=13,       // led sinalizador do modo de edição dos parametros
    //30 usado como ground do led
    //32 usado como positivo p sensor do Ambu
    //34 usado como ground p sensor do ambu  
    EnableAmbu = 38,  // out enable do sensor do Ambu   
    Ambu = 36,        // in - sensor óptico reflexivo (infravermelho) analógico - informa ao sistema se há um ambu no aparelho ou se está fora de posição                              
    //Peep = A15;       // peep eletronico  não implementado
    // portas 20 e 21  SDA e SCL da placa I2C
    
// pinos analógicos
    potV = A0,        // potenciometro responsável pela definição da velocidade do movimento do braço  - Knob AZUL
    potF = A2,        //    "              "        "      "      da relação chamada - insp/exp ou I/E - Knob VERDE
    potC = A1;        //    "              "        "      "      do curso do braço (VOLUME de ar injetada no paciente)  - Knob AMARELO                     

    // sensor de fluxo   - não implementado
    // sensor de pressão - não implementado

// variáveis
int leituraAmbu,                                  // variável q sinaliza se há um Ambu no equipamento.
    leituraC,leituraV,leituraF,                   // valores analógico lido nos potenciometros              
    valorC,valorV,                                // valor parametrizado da leitura feita nos potenciomentros                  
    valorV2,                                      // é uma variante do valor V
    guardadoC = 0, guardadoV = 0, guardadoF = 0,  // variáveis  guardam o valor anterior de C,V eF                          
    marca=0,                                      // variável - indica se o programa acabou de iniciar ou não
    limiteUp = 10,                                // ponto zero do equipamento
    contador=0,                                   // conta as interrupções do encoder
    espera=0,                                     // variavel que guarda o resultado de calculos de tempo relativos à relação Insp/Exp              
    Ciclos=2,                                     // durante quantos ciclos do equip(respirações) a janela de edição de parametro fica abeerta disponivel depois de ter sido solicitada pelo botao1
    nCiclos=0,                                    // contagem de ciclos p fechamento da janela de configuração do equip - estratégia de prevençao p manipuaçao acidental dos parametros nos pots
    direcao=0,                                    // a indicação da direção do motor informa se o contador vai ser incrementado ou decrementado na interrupção 1
    sinalizador=0, 
    Peep,                                         // valor da velocidade (pwm) do soprador p manter o peep do paciente
    cracha=0,
    S1=0,                                          //setada durante a interrupção causada pelo no botao1 e retorna a 0 através tb do botao1 ou depois de alguns ciclos de operação(Ciclos)
    contadore=0;
float     
    K, W, frequencia=0,valorF,                                       // PD
//unsigned long 
    t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5=0,       //  variáveis associadas ao millis()   

void setup(){
  Serial.begin(9600);
 //botao -interrupção
  pinMode(botao1, INPUT_PULLUP);                 // pino3  - botão p modo de edição dos parametros  
  pinMode(encoder,INPUT);                        // pino2  - entrada do sinal do disco acoplado ao motor
 //interrupções
  attachInterrupt(digitalPinToInterrupt(encoder), interrompendo1, RISING);           // CONFIGURAÇÃO DA INTERRUPÇÃO NO PINO 2
  attachInterrupt(digitalPinToInterrupt(botao1), interrompendo2, RISING);            // CONFIGURAÇÃO DA INTERRUPÇÃO NO PINO 3

  //ambu  
  pinMode(Ambu,INPUT);                           // pino 38- PORTA ANALOGICA PINO QUE MONITORA A PRESENÇA DO AMBU NO EQUIP
  pinMode(EnableAmbu,OUTPUT);                    // pino 36 - porta digital - HABILITA SENSOR DO AMBU
  pinMode(32,OUTPUT); digitalWrite(36,1);        // VAI SER USADO COMO POSITIVO P SENSOR DO AMBU
  pinMode(34,OUTPUT); digitalWrite(38,0);        // VAI SER USADO COMO GROUND P SENSOR DO AMBU

  //display lcd 16x2
  lcd.begin (16,2);                              //SETA A QUANTIDADE DE COLUNAS(16) E O NÚMERO DE LINHAS(2) DO DISPLAY
  lcd.backlight();                               //LIGA O BACKLIGHT (LUZ DE FUNDO)                                 
  
  // led sinalizador do modo de edição
  pinMode(30, OUTPUT); digitalWrite(30,0);
  pinMode(ledEdit, OUTPUT);                      // led - acende quando é aberta uma janela de tempo p edição dos parametros
  pinMode(12, OUTPUT); digitalWrite(12,LOW);     // ground do ledEdit
 
  //pinos do motor
  pinMode(M1, OUTPUT); pinMode(M2, OUTPUT);  
  pinMode(EnableM, OUTPUT);                      // o pino enable habilita o funcionamento do motor. Pino pwm usado p controlar a velocidade do motor                                

  //pinos do soprador
  pinMode(C1, OUTPUT); pinMode(C2, OUTPUT);  
  pinMode(EnableC, OUTPUT);                      // o pino enable habilita o funcionamento do motor. Pino pwm usado p controlar a velocidade do motor                                
 
 // pinos dos 3 potenciometros
  pinMode(potC, INPUT);pinMode(potF, INPUT);pinMode(potV, INPUT); 

 // sensor de fluxo

 // sensor de pressão
 
//iniciando o display c logo do iVent e piscadinha do led - só p tirar onda
  lcd.setBacklight(HIGH);                                      
  lcd.setCursor(5,0);  lcd.print("iVent");                     
  digitalWrite(ledEdit,1);  delay(300);  digitalWrite(ledEdit,0);  delay(1500); 
  lcd.clear();                }

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

void loop(){  
       if (marca==0)                                            // verificacão inicial do aparelho - acontece só uma vez
          {
          Configureme();      // S1=1                           // configurações (braço parado inicialmente)                                    
          checando();                                           // checagem de posição do braço    
          marca=1;
          cracha=1;
          }                                                                                     
          
      //    AloAmbu();                                         // função q monitora o posicionamento correto do Ambu no aparelho
      
          atualiza();                                          // se a janela de edição estiver aberta atualiza os parametros
       
       if (S1==1&& nCiclos<= Ciclos)
       {nCiclos++;goto inicio; }                               // conta Ciclos ---(S1 inicia 0) e se botao1 acionado S1=1, ou seja cod fica na linha 115 (tem um while no atualiza) esperando configuração nos pots
       if (S1==1&& nCiclos> Ciclos) 
       {nCiclos=0; S1=0;digitalWrite(ledEdit,LOW); }           // fecha janela S1=0
 inicio:
//.........................................INSPIRAÇÃO DO PACIENTE - em 2 fases A e B  
                                                                                          
          digitalWrite(C1,0);  digitalWrite(C2,1);analogWrite(EnableC, 255);             // em discussão-- soprador em força máxima(255)criando uma barreira de resistência à fuga da pressão enviada pelo Ambu - 

 //faseA                                                                                              
          contador=limiteUp;                                                             // coloca em contador um valor inicial de segurança
          t1 = millis();                                                                 // iniciando contagem de descida 
          direcao=1; // descendo                // configura motor                       // avisa ao encder a direçao atual p incrementação ou decrementação do contador
          analogWrite(EnableM, valorV);           // configura motor                     // habilita motor com pwm no valor de velocidade escolhido no potV
          digitalWrite(M1,0);  digitalWrite(M2,1);// configura motor                     // motor gira no sentido 1 enquanto o braço não atingir metade do curso escolhido
   while  (contador< valorC/2)                     
          {//ajudae();
          atualiza(); }  


                                                                                                                                                 
 //fase B 
          analogWrite(EnableM, valorV2);                                                 //configura velocidade menor nessa fase.                                                                                                                                                     
   while  (contador< valorC)                                                             // enquanto o braço não atingir , aproximadamente, 95% do percurso prescrito no pot C            
          { //ajudae();
          atualiza(); }                                                            
          digitalWrite(M1,1);  digitalWrite(M2,1);                                       // acabado o loop paramos o motor                                                            

//freadinha substituir por PID ou PD
          direcao=1;  //descendo
          digitalWrite(M1,1); digitalWrite(M2,0);  delay(100);        
          digitalWrite(M1,1); digitalWrite(M2,1); 
     
          digitalWrite(M1,1); digitalWrite(M2,1); delay(40);                                                                                                                    
          t2=millis();     //encerrando contagem de descida
                         
         
//........................ MOTOR RETORNANDO A POSIÇÃO SUPERIOR - PONTO ZERO
          t3=millis();                                                                    //iniciando contagem de subida
 
          digitalWrite(C1,0);  digitalWrite(C2,1);analogWrite(EnableC, Peep);             // soprador diminue a intensidade do fluxo - passa a atuar como mantenedor do ppep(pressão mínima de ar nos pulmões para q ele não colabe.
          //ajudae();
    while (contador> limiteUp)                                                            // enquanto a leitura da posição do braço indicar  não chegou ao ponto zero                                                                                   
          {direcao=2;     // subindo              // configura motor
          analogWrite(EnableM, valorV);            // configura motor                                             
          digitalWrite(M1,1);  digitalWrite(M2,0); // configura motor                      
          atualiza();}                                                               
          digitalWrite(M1,1); digitalWrite(M2,1);                                         // ao fim do loop desliga motor


//freadinha substituir por PID ou PD
          direcao=1;  //descendo
          digitalWrite(M1,0); digitalWrite(M2,1);  delay(100);        
          digitalWrite(M1,1); digitalWrite(M2,1);          
      
          //aqui cabe uma checagem                                                        // confirmar posição - a desenvolver
      
          t4=millis();                                                                    //encerrando contagem de subida
 // tempo de espera                                                                       //delay de espera que completa o tempo faltante p fazer valer a relação i/e
                                                                                          // temos que descobrir os pontos de inversão do cooler p maior eficiencia  
          espera=(valorF*(t2-t1)) - (t4-t3) ;         
          float xx=espera/20;                                                     
     for  (int i = 0; i <=20; i++) {
          delay(xx); {atualiza();}}                                                       // aqui poderiamos ter colocado um delay com o tempo xx, mas preferimos dividi-lo arbitrariamente em 20 pedaços      
                                                                                          // a cada pedaço o procedimento da um pulinho em atualiza() e se a janela de edição estiver aberta atualiza os parametros e o display                                                                              
          t5=millis();
          frequencia= (t5-t1)/1000;                                                       // valor em segundos p  apresentar no display
          lcd.setCursor(13,1);lcd.print("    ");   
          lcd.setCursor(13,1);lcd.print(frequencia);

}                                                                                                                                                               
 
 void ajudae(){                                                // auxiliar na procura de erros, copio e colo 
  lcd.clear();lcd.setCursor(0,0);lcd.print("contador=");lcd.setCursor(10,0); lcd.print(contador);
  lcd.setCursor(15,0); lcd.print(contadore);
 }

void AloAmbu(){  
          digitalWrite(EnableAmbu,1);
          int leituraAmbu = digitalRead(Ambu);
  // if   (leituraAmbu!=1) { 
    while (leituraAmbu!=0)                                     // verifica se há Ambu no lugar certo, e se não estiver sinaliza
                                                 
          {lcd.clear();lcd.setCursor(0,0);lcd.print("AMBU AUSENTE");
          delay(400);lcd.clear();delay(400);
          leituraAmbu= digitalRead(Ambu);         } }

void Configureme(){
          digitalWrite(ledEdit,HIGH);nCiclos=0; S1=1;
          atualiza();
          marca=1;   }
          
void checando(){

          lcd.clear();lcd.setCursor(0,0);lcd.print(" REPOSICIONANDO ");                          
          lcd.setCursor(1,1);lcd.print("BRA");
          lcd.createChar(0, cedilha);
          lcd.setCursor(4,1);
          lcd.write(byte(0));
          lcd.print("O"); 
          lcd.setCursor(7,1);lcd.print("MECANICO");

      
          analogWrite(EnableM, 70);                   // configura motor                  // habilita o PWM com velocidade do potenciometro V
          digitalWrite(M1,1);  digitalWrite(M2,0);    // configura motor 
          direcao=1;     // subindo                   // configura motor
          contadore=0;
    while (contador<20) 
          {contadore++;  delay(1);//ajudae();                                                                                                                                                   
       if (sinalizador==1){ contadore=0;sinalizador=0;} }                                                                                           
       
          digitalWrite(M1,1); digitalWrite(M2,1);         // desliga
                                                                                         
          digitalWrite(M1,0); digitalWrite(M2,1);  delay(100);                            //freadinha  substituir por PID
          digitalWrite(M1,1); digitalWrite(M2,1); 
          delay(1000);
          lcd.clear();                                    }                                   // espera um tempinho p iniciar o trabalho    
                                                                 
void atualiza(){                                                                          // ATUALIZA OS PARAMETROS 
       if (cracha==1){goto pulo;}
       if (S1==1 && marca!=0) {goto pulo;}
    while (S1==1 && marca==0)
 pulo:    {                                                                               
          leituraC = analogRead(potC); valorC = map(leituraC, 0, 1023, 1, 200); 
          leituraF = analogRead(potF); valorF = map(leituraF, 0, 1023, 1, 4);        
          leituraV = analogRead(potV); valorV = map(leituraV, 0, 1023, 60,255);  valorV2 = map(valorV, 80, 255, 60,100);       
             
             lcd.setCursor(0,0);lcd.print("Vol:");
             lcd.setCursor(8,0);lcd.print("i/e  1/"); 
             lcd.setCursor(0,1);lcd.print("Vel:");
             lcd.setCursor(8,1);lcd.print("Freq");  
                             
       if (valorC != guardadoC){guardadoC=valorC;}; lcd.setCursor(4,0);lcd.print("    ");    lcd.setCursor(4,0); lcd.print(valorC); 
       if (valorV != guardadoV){guardadoV=valorV;}; lcd.setCursor(4,1);lcd.print("    ");    lcd.setCursor(4,1); lcd.print(valorV); 
       if (valorF != guardadoF){guardadoF=valorF;}; lcd.setCursor(15,0);lcd.print("    ");   lcd.setCursor(15,0);lcd.print(valorF); 
                                                    lcd.setCursor(12,1);lcd.print("    ");   lcd.setCursor(13,1);lcd.print(frequencia);                                      };


      cracha=0;
          }
   
   
// interrupções
 void interrompendo1()                    // encoder pino2  // função de interrupção1         
          {
          sinalizador=1;
       if (direcao==1){contador ++;} 
       if (direcao==2){contador--;}
                            }                       

void interrompendo2()                   //botao pino3      // função de interrupção2
          { if (S1==0){S1=1;digitalWrite(ledEdit,HIGH);nCiclos=0; } else {S1=0;digitalWrite(ledEdit,LOW);}
            delay(200);
              }  







               

     

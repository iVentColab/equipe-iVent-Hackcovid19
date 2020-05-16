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
    t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5=0, ti=0, tf=0;    //  variáveis associadas ao millis()   

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

          lcd.clear();   
  

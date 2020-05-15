
//******************************************************************************************************************************
// início do projeto                               - manha 27042020
// finalização do código                           - noite 29042020  
// Testes e correções no aparelho                  - noite 30042020
// Aparelho funcionando                            - noite 01052020
// Aperfeiçoamento-1                               - noite 03052020  velocidade em 2 fases e sistema de proteção inicial e inclusão de sensor p detectar a ausencia do Ambu no aparelho
// Envio do cód p programadores envolvidos no proj - tarde 04052020
// Aperfeiçoamento 2                               - noite 04052020  implentação do pwm, arrays no display e retirada do sensor de Ambu
// Aperfeiçoamento 3                               - noite 06052020  implementação de mais dois potenciometros, 
//                                                                   1-substituimos a ideia de um algoritmo de desaceleração pronto e
//                                                                   e criamos um potenciometro que prescreve a taxa de desaceleração(isso fica a cargo da equipe médica)
//                                                                   2-potenciomento funcionando como encoder, ou seja todo o controle de posicionamento, que inclusive era feito pelos microsuitches,
//                                                                   agora é feito pelo encoder. vamos ver se funciona bem. 
//                                                                   3- alguns pinos foram mudados de endereço
//
// Aperfeiçoamento 4                               - manhã 07052020  com a remontagem do novo modelo reorganizamos a tabela do display
//                                                                   Ajustamos o controle de posicionamento do braço através do Encoder.
//                                                                   Ajustamos o controle de velocidade pelo pot V
//                                                                   Fizemos alguns testes com os parametros de desaceleração do pot 4, ainda não tá bom 
//                                                                   voltamos a trabalhar com a idéia de controle do tempo das ações, tipo: se um movimento que
//                                                                   deveria se realizar em um segundo não chega ao destino no tempo previsto , alguma ação é tomada.
//                                                                   Apareceram alguns problemas e retomamos isso amanhã 
//                                                                   substituimos o HIGH e o LOW da programação por 1 e 0 respectivamente p facilitar a escrita  
//Hardware                                         - manhã 08052020  Preparação e pigmentação do material plástico líquido
//                                                                   que deve fazer a cobertura plástica das peças externas de metal.  
//hardware                                         - manhã 09052020  Finalizada cobertura das duas peças do braço com pl[astico líquido
//   
// migração p arduino mega- ja estavamos quase sem portas
// implementando sensor optico reflexivo p sensoriamento do Ambu - esperando a peça chegar
// implementação de encoder óptico p controle de posição do braço  
// modelamos em 3d e imprimimos a peça ( rodela perfurada c adaptação p eixo do motor) do encoder com 72 passos 
// implementamos o código p cooler reversível como auxiliar na eliminação ou diminuição de gases tóxicos residuais no sistema 
// e que ainda serve como auxiliar p manutenção do peep no paciente
// eliminamos o potenciometro q cuidava da velocidade da segunda metade do percurso (mais lenta) e implementamos no lugar uma função (map) da velocidade da faseA
// sustituimos o display de 7 segmentos por um display lcd 16x2 (16 colunas e duas linhas)
// Elaboramos um design onde as informações aparecem completas no display e imediatamente ajustáveis conforme forem editadas
// foi implementado um código que mantém uma proporção exata do tempo da relação insp/exp programada pela equipe médica- grande avanço
// foi implementado um código que calcula, baseado nos valores de curso, velocidade e i/e a frequencia ou tempo do ciclo comleto e expõe no displaylcd (unid em segs)
// houve o entendimento da necessidade de implementação de um PID em dois lugares e isso já está sinalizado e estamos trabalhando nisso agora.  
// ATÉ 13/05/2020
// falta atualizar novas modificações    até 15/05/2020                                             
//******************************************************************************************************************************

// informações importantes p facilitar o entendimento:

//trata-se de um programa que pretende fazer um motor (e um braço acoplado a ele)  ter um comportamento de acordo com o desejado
//as ordens p o funcionamento do motor são:

// digitalWrite(M1,0); digitalWrite(M2,1); -para o motor girar no sentido de descida do braço
// digitalWrite(M1,1);  digitalWrite(M2,0); -para o motor girar no sentido de subida do braço
// digitalWrite(M1,1); digitalWrite(M2,1); - para o motor parar de girar 
// Esse braço mecânico comprime o Ambu que é um equipamento hospitalar que quando apertado expulsa o seu volume de ar para o paciente através de uma traqueia de plástico
// e uma mascara presa sobre a boca e o nariz do paciente.  
// Nesse conjunto, há válvulas que fazem, entre outras coisas, o trabalho de encaminhar o ar "usado" do paciente para fora do sistema.

// observação:é importante saber que a função do aparelho é "injetar" uma mistura gasosa hospitalar (ar enriquecido com O2) no paciente - inspiração. 
// A expiração do paciente é totalmente independente do aparelho, ou seja, o retorno do braço mecânico à posição superior não tem influência nenhuma
// sobre o paciente.
// No caso de não haver uma linha de oxigênio perto do paciente, ainda assim o aparelho tem utilidade pois ventila o pulmão do doente, não com tanta eficácia.

// outra coisa imortante
// temos agora 3 POTENCIÔMETROS,   que são responsáveis pelo " ajuste ou programação" do comportamento do equipamento que é FEITA PELA EQUIPE MÉDICA
// no hospital ou CTI
// o equipamento tem que dar conta de pacientes extremamente diferentes ou seja permitir ao médico ou fisioterapeuta encontrar nesses 4 parametros (potenciômetros) 
// a combinação ideal p cada paciente.

//Potenciometro.
// o pino central envia um valor analógico que é lido na porta do arduino como um numero que vai de 0 a 1023. 
//  Parâmetros:
// 1º - quantidade ou volume de ar injetada no paciente
// 2º - velocidade da fase A do giro do motor , a velocidade da fase b do percurso é ditada por uma outra função (map) da velocidade da fase A
// 3º - a relação entre tempo de inspiração e expiração - 
// próximos desafios devem aparecer quando o aparelho entrar em testes e quando a ANVISA fizer novas exigências
//  idéias boas são sempre bem vindas! Abraço!! 
 
//;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
//#include <TimerOne.h>
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
    EnableAmbu 40;    // enable do sensor do Ambu   
    //36 usado como positivo p sensor do Ambu
    //38 usado como ground p sensor do ambu 
    // portas 20 e 21  SDA e SCL da placa I2C
    
// pinos analógicos
    potV = A0,        // potenciometro responsável pela definição da velocidade do movimento do braço  - Knob AZUL
    potF = A2,        //    "              "        "      "      da relação chamada - insp/exp ou I/E - Knob VERDE
    potC = A1,        //    "              "        "      "      do curso do braço (VOLUME de ar injetada no paciente)  - Knob AMARELO                     
    Ambu = A14,       // in - sensor óptico reflexivo (infravermelho) analógico - informa ao sistema se há um ambu no aparelho ou se está fora de posição                              
    // Peep = A15;       - peep eletronico  não implementado
    // sensor de fluxo   - não implementado
    // sensor de pressão - não implementado

// variáveis
int leituraAmbu,                                  // variável q sinaliza se há um Ambu no equipamento.
    leituraC,leituraV,leituraF,                   // valores analógico lido nos potenciometros              
    valorC,valorV,valorF,                         // valor parametrizado da leitura feita nos potenciomentros                  
    valorV2,                                      // é uma variante do valor V
    guardadoC = 0, guardadoV = 0, guardadoF = 0,  // variáveis  guardam o valor anterior de C,V eF                          
    marca=0,                                      // variável - indica se o programa acabou de iniciar ou não
    limiteUp = 10,                                // ponto zero do equipamento
    contador=0,                                   // conta as interrupções do encoder
    frequencia=1,
    espera=0,                                     // variavel que guarda o resultado de calculos de tempo relativos à relação Insp/Exp              
    Ciclos=30,                                    // durante quantos ciclos do equip(respirações) a janela de edição de parametro fica abeerta disponivel depois de ter sido solicitada pelo botao1
    nCiclos=0,                                    // contagem de ciclos p fechamento da janela de configuração do equip - estratégia de prevençao p manipuaçao acidental dos parametros nos pots
    direcao=0,                                    // a indicação da direção do motor informa se o contador vai ser incrementado ou decrementado na interrupção 1
    sinalizador=0, 
    // Vpeep,                                     // valor analogico lido no potenciometro relativo ao peep eletronico - ainda não implemntado
    S1=0;                                         //setada durante a interrupção causada pelo no botao1 e retorna a 0 através tb do botao1 ou depois de alguns ciclos de operação(Ciclos)

unsigned long 
    t1 = 0, t2 = 0, t3 = 0, t4 = 0,ti=0, tf=0;    //  variáveis associadas ao millis()   

void setup(){
  Serial.begin(9600);
  
  //interrupções
  attachInterrupt(digitalPinToInterrupt(encoder), interrompendo1, RISING);           // CONFIGURAÇÃO DA INTERRUPÇÃO NO PINO 2
  attachInterrupt(digitalPinToInterrupt(botao1), interrompendo2, RISING);            // CONFIGURAÇÃO DA INTERRUPÇÃO NO PINO 3

  //ambu
  pinMode(Ambu,INPUT);                           // A14- PORTA ANALOGICA PINO QUE MONITORA A PRESENÇA DO AMBU NO EQUIP
  pinMode(EnableAmbu,OUTPUT);                    // pino 40 - porta digital - HABILITA SENSOR DO AMBU
  pinMode(36,OUTPUT); digitalWrite(36,1);        // VAI SER USADO COMO POSITIVO P SENSOR DO AMBU
  pinMode(38,OUTPUT); digitalWrite(38,0);        // VAI SER USADO COMO GROUND P SENSOR DO AMBU

  //display lcd 16x2
  lcd.begin (16,2);                              //SETA A QUANTIDADE DE COLUNAS(16) E O NÚMERO DE LINHAS(2) DO DISPLAY
  lcd.backlight();                               //LIGA O BACKLIGHT (LUZ DE FUNDO)                                 
  
  // led sinalizador do modo de edição
  pinMode(ledEdit, OUTPUT);                      // led - acende quando é aberta uma janela de tempo p edição dos parametros
  pinMode(12, OUTPUT); digitalWrite(12,LOW);     // ground do ledEdit
 
  //pinos do motor
  pinMode(M1, OUTPUT); pinMode(M2, OUTPUT);  
  pinMode(EnableM, OUTPUT);                      // o pino enable habilita o funcionamento do motor. Pino pwm usado p controlar a velocidade do motor                                

  //pinos do soprador
  pinMode(C1, OUTPUT); pinMode(C2, OUTPUT);  
  pinMode(EnableC, OUTPUT);                      // o pino enable habilita o funcionamento do motor. Pino pwm usado p controlar a velocidade do motor                                

  //botao -interrupção
  pinMode(botao1, INPUT_PULLUP);                 // pino3  - botão p modo de edição dos parametros
 
 // pinos dos 3 potenciometros
  pinMode(potC, INPUT);pinMode(potF, INPUT);pinMode(potV, INPUT); 

 // sensor de fluxo

 // sensor de pressão
 
//iniciando o display c logo do iVent e piscadinha do led - só p tirar onda
  lcd.setBacklight(HIGH);                                      
  lcd.setCursor(5,0);  lcd.print("iVent");                     
  digitalWrite(ledEdit,1);  delay(300);  digitalWrite(ledEdit,0);  delay(1500); 
  lcd.clear();                }

void loop(){  
       if (marca==0)                                             // verificacão inicial do aparelho - acontece só uma vez
          {
          Configureme();                                        // configurações (braço parado inicialmente)                                    
          checando();                                           // checagem de posição do braço    
          marca=1;
          }                                                                                     
       
          leituraAmbu= digitalRead(Ambu);                     // verifica a cada loop se o Ambu está no lugar certo
       if (leituraAmbu==LOW){CadeAmbu();}                     // se o Ambu não está, chama a rotina CadeAmbu()- (ta invertido aqui enquanto naõ resolvemos o HARDWARE)
      
          atualiza();                                         // se a janela de edição estiver aberta atualiza os parametros
       
       if (S1==1)                                             // conta Ciclos   --- S1=botao1 acionado
          {
       if (nCiclos<= Ciclos)
          {nCiclos++;digitalWrite(ledEdit,HIGH);              // incrementa ciclos e liga led sinalizador de modo de edição 
          }
     else
          {
          nCiclos=0; S1=0;digitalWrite(ledEdit,LOW);          // depois de alguns ciclos de operação a janela de edição se fecha 
          } }    
       
//.........................................INSPIRAÇÃO DO PACIENTE - em 2 fases A e B  
 //faseA  
          t1 = millis();                                                                 // iniciando contagem de descida                                                           
          digitalWrite(C1,0);  digitalWrite(C2,1);analogWrite(EnableM, 255);             // em discussão-- soprador em força máxima(255)criando uma barreira de resistência à fuga da pressão enviada pelo Ambu - 
                                                                                             
          contador=limiteUp;                                                             // coloca em contador um valor inicial de segurança

   while  contador< valorC/2)                     
          {direcao=1; // descendo                // configura motor                      // avisa ao encder a direçao atual p incrementação ou decrementação do contador
          analogWrite(EnableM, valorV);           // configura motor                     // habilita motor com pwm no valor de velocidade escolhido no potV
          digitalWrite(M1,0);  digitalWrite(M2,1);// configura motor                     // motor gira no sentido 1 enquanto o braço não atingir metade do curso escolhido
          atualiza(); }  

// obs :  a configuração do motor linhas 214,215,216 poderiam ficar fora(antes) do loop
//        e não precisariam ser repetidas na fase B já que são as mesmas, correto? num sei pq deu ruim quando eu fiz isso                                                                                                                                                    
 //fase B 
          analogWrite(EnableM, valorV2);                                                 //configura velocidade menor nessa fase.                                                                 
    while (contador< valorC)                                                             // enquanto o braço não atingir o fim do percurso prescrito no pot C            
          {direcao=1; 
          analogWrite(EnableM, valorV2);           // configura motor                    
          digitalWrite(M1,0);  digitalWrite(M2,1);                                       // motor gira no sentido 1 enquanto o braço não atingir metade do curso escolhido 
          atualiza(); }                                                            
          digitalWrite(M1,1);  digitalWrite(M2,1);                                       // acabado o loop paramos o motor                                                            

//freadinha  substituir por PID ou PD
          direcao=2;   //subindo                                                         // avisa ao encoder a direçao atual p incrementação ou decrementação do contador
          digitalWrite(M1,1);  digitalWrite(M2,0); delay(80);                            // estrategia provisoria de parada imediata do motor, dando um microgiro em sentido contrario                                                                                            
          digitalWrite(M1,1);  digitalWrite(M2,1); delay(40);                            // esperamos em breve implementar um PID                                                                                          
          t2=millis();     //encerrando contagem de descida
       
//........................ MOTOR RETORNANDO A POSIÇÃO SUPERIOR - PONTO ZERO
          t3=millis();                                                                   //iniciando contagem de subida
 
          digitalWrite(C1,0);  digitalWrite(C2,1);analogWrite(EnableM, peep);             // soprador diminue a intensidade do fluxo - passa a atuar como mantenedor do ppep(pressão mínima de ar nos pulmões para q ele não colabe.
          
    while (contador> limiteUp)                                                            // enquanto a leitura da posição do braço indicar  não chegou ao ponto zero                                                                                   
          {direcao=2;     // subindo              // configura motor
          analogWrite(EnableM, valorV);            // configura motor                                             
          digitalWrite(M1,1);  digitalWrite(M2,0); // configura motor                     // giro invertido   
          atualiza();}                                                               
          digitalWrite(M1,1); digitalWrite(M2,1);                                         // ao fim do loop desliga motor
//freadinha substituir por PID ou PD
          direcao=1;  //descendo
          digitalWrite(M1,0); digitalWrite(M2,1);  delay(100);        
          digitalWrite(M1,1); digitalWrite(M2,1);          
      
          //if (contador!=limiteUp){checando();}                                          // confirmar posição - a desenvolver
        
          t4=millis();                                                                    //encerrando contagem de subida
 // tempo de espera                                                                       //delay de espera que completa o tempo faltante p fazer valer a relação i/e
                                                                                          // temos que descobrir os pontos de inversão do cooler p maior eficiencia  
          espera=valorF*((t2-t1) - (t4-t3)) ;         
          float xx=espera/20;                                                     
     for  (int i = 0; i <=20; i++) {
          delay(xx); {atualiza();}}                                                       // aqui poderiamos ter colocado um delay com o tempo xx, mas preferimos dividi-lo arbitrariamente em 20 pedaços      
          frequencia= ((t2-t1)+(t4-t3))/1000;                                             // a cada pedaço o procedimento da um pulinho em atualiza() e se a janela de edição estiver aberta atualiza os parametros e o display                                                                              
}                                                                                         //Se não fosse feito isso, durante esse espaço de tempo as alteraçoes do usuário não seriam efetivadas e nem visualiadas                                                                       
void CadeAmbu(){  
    while (leituraAmbu==HIGH)    // acho que teremos que mudar p uma leitura analogica    // verifica se há Ambu no lugar certo, e se não estiver pisca um "A"
                                // porque acho o sensor não é digital, ainda não vi isso
          {lcd.clear();lcd.setCursor(0,0);lcd.print("AMBU AUSENTE");
          delay(400);lcd.clear();delay(400);
          leituraAmbu= digitalRead(Ambu);         } }

void Configureme(){
          digitalWrite(ledEdit,HIGH);nCiclos=0; S1=1;
          atualiza();
          marca=1;
}
void checando(){
          // acertar isso, e depois calibrar os parametros
          analogWrite(EnableM, 70);                   // configura motor                  // habilita o PWM com velocidade do potenciometro V
          digitalWrite(M1,1);  digitalWrite(M2,0);    // configura motor 
          direcao=2;     // subindo                   // configura motor
          int contadore=0;
    while (contadore<10)                                                                  // enquanto a leitura da posição do braço indicar  não chegou ao ponto zero(limiteUp)                                                                                   
          {contadore++;
          delay(75);  // delayzinho p tornar mais visiveis os testes
       if (sinalizador==1){ contadore=0;sinalizador=0;} }                                                                                           
       
          digitalWrite(M1,1); digitalWrite(M2,1);         // desliga
                                                                                         
          digitalWrite(M1,0); digitalWrite(M2,1);  delay(100);                            //freadinha  substituir por PID
          digitalWrite(M1,1); digitalWrite(M2,1); 
          delay(1000);                                }                                   // espera um tempinho p iniciar o trabalho    
                                                                 
void atualiza(){                                                                          // ATUALIZA OS PARAMETROS 
       if (S1==1 && marca!=0) {goto pulo;}
    while (S1==1 && marca==0)
 pulo:    {ti=millis();                                                                                
          leituraC = analogRead(potC); valorC = map(leituraC, 0, 1023, 1, 280); 
          leituraF = analogRead(potF); valorF = map(leituraF, 0, 1023, 1, 4);        
          leituraV = analogRead(potV); valorV = map(leituraV, 0, 1023, 60,255);  valorV2 = map(valorV, 80, 255, 60,150);       
             lcd.setCursor(13,1);lcd.print(frequencia);
             lcd.setCursor(0,0);lcd.print("Vol:");
             lcd.setCursor(8,0);lcd.print("i/e 1/"); 
             lcd.setCursor(0,1);lcd.print("Vel:");
             lcd.setCursor(8,1);lcd.print("Freq");                           
       if (valorC != guardadoC){guardadoC=valorC; lcd.setCursor(4,0);lcd.print("    ");    lcd.setCursor(4,0);lcd.print(valorC); };
       if (valorV != guardadoV){guardadoV=valorV; lcd.setCursor(4,1);lcd.print("    ");    lcd.setCursor(4,1);lcd.print(valorV); };
       if (valorF != guardadoF){guardadoF=valorF; lcd.setCursor(14,0);lcd.print(valorF);                                         };
          }
   }
// interrupções
 void interrompendo1()                    // encoder pino2  // função de interrupção1         
          {
       if (direcao==1){contador ++;  } 
       if (direcao==2){contador--;}
          sinalizador=1;}                       

void interrompendo2()                   //botao pino3      // função de interrupção2
          {
       if (S1==0){digitalWrite(ledEdit,HIGH);nCiclos=0; S1=1;} else {S1=0;digitalWrite(ledEdit,LOW);}}  

 
/*/

ISR: a ISR a ser chamada quando a interrupção ocorre; essa função deve não tomar nenhum parâmetro nem retornar nada. Essa função é chamada de rotina de serviço da interrupção ou ISR (do Inglês, interrupt service routine).
modo: define quando a interrupção deve ser ativada. Quatro constantes estão predefinidas como valores válidos:
LOW acionar a interrupção quando o estado do pino for LOW,
CHANGE acionar a interrupção quando o sempre estado do pino mudar
RISING acionar a interrupção quando o estado do pino for de LOW para HIGH apenas,
FALLING acionar a interrupção quando o estado do pino for de HIGH para LOW apenas.
Placas Due, Zero e MKR1000 suportam também:
HIGH acionar a interrupção quando o estado do pino for HIGH.
 
/*
// Timer1.initialize(500000); // Inicializa o Timer1 e configura para um período de 0,5 segundos
 // Timer1.attachInterrupt(uhu); // Configura a função uhu() como a função para ser chamada a cada interrupção do Timer1
  //Timer1.detachInterrupt(uhu); 

// PID......................    
  Setpoint = ;                                  //Hardcode the brigdness value
  myPID.SetMode(AUTOMATIC);                       //Turn the PID on
  myPID.SetTunings(Kp, Ki, Kd);                   //Adjust PID values
// ......................    
*/

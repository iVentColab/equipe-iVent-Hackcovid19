

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
// só p lembrar ou p quem não sabe. O potenciometro tem tres pinos onde são ligados tres fios. O pino da esquerda vai ligado no positivo 5v do arduino e o pino
// da direita vai ligado no negativo do arduino(isso pode ser ao contrário tb dependendo do projeto). De acordo com o movimento no botão(knob) 
// o pino central envia um valor analógico que é lido na porta do arduino como um numero que vai de 0 a 1023. 
// Esse numero é convertido "display"=(map( "leitura do potenciometro" ,0,1023, 1,9) proporcionalmente em um digito de 1 a 9 que aparece no dispay como guia p o usuário.
// e ao mesmo tempo pode haver uma outra conversão para parametros como velocidade do motor, tamanho de percurso, ou qualquer outro.
//  Parâmetros:
// 1º - quantidade ou volume de ar injetada no paciente
// 2º - velocidade da fase A do giro do motor , a velocidade da fase b do percurso é ditada por uma outra função (map) da velocidade da fase A
// 3º - a relação entre tempo de inspiração e expiração - resolvemos essa questão medidno o tempo gasto na inspiração completa e multiplicando pelo indice escolhido pela eq médica

// próximos desafios devem aparecer quando o aparelho entrar em teste e quando a ANVISA fizer novas exigências
//  idéias boas são sempre bem vindas! Abraço!! 
 
//;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
#include <AutoPID.h>                         //PID sinistrão
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Addr, En, Rw, Rs, d4, d5, d6, d7, backlighpin, polarity

//Pinos digitais---------------------------
const int
    encoder=2,        // pino de interrupção do encoder
    Ambu=3,           // in   - sensor óptico reflexivo (infravermelho) - informa ao sistema se há um ambu no aparelho ou se está fora de posição                              
    buzzer= 4,        // aviso sonoro
    M1 =5,            // out  - motor1  - pino 5 e 6 que levam informação de controle do arduino para a placa  controla o motor (módulo driver ponte H -L298N )
    M2 =6,            // out  - motor1   
    EnableM=7,        // out  - pino enable do motor. um PWM será aplicada nesse pino nos dando a possibilidade de controlar dessa forma a velocidade do motor  
    C1=8,             // out  - cooler reversível. auxiliar na eliminação dos gaes tóxicos  info vai p (módulo driver ponte H -L298N )
    C2=9,             // out  - cooler reversível. auxiliar na eliminação dos gaes tóxicos
    EnableC=10,       // pino enable do cooler reversivel
    // portas 20 e 21  SDA e SCL da placa I2C
          
// pinos analógicos
    potV = A0,        // potenciometro responsável pela definição da velocidade do movimento do braço  - Knob AZUL
    potF = A3,        //    "              "        "      "      da relação chamada - insp/exp ou I/E - Knob VERDE
    potC = A1;        //    "              "        "      "      do curso do braço (VOLUME de ar injetada no paciente)  - Knob AMARELO                     
     // sensor de fluxo   - não implementado
     // sensor de pressão - não implementado

//variáveis--------------------------- 
int leituraAmbu,                                  // variável q sinaliza se há um Ambu no equipamento.
    leituraC,leituraV,leituraF,                   // valores analógico lido nos potenciometros              
    valorC,valorV,valorF,                         // valor parametrizado da leitura feita nos potenciomentros                  
    valorV2,                                      // é uma variante do valor V
    guardadoC = 0, guardadoV = 0, guardadoF = 0,  // variáveis  guardam o valor anterior de C,V eF                          
    marca=0,                                      // variável - indica se o programa acabou de iniciar ou não
    limiteUp = 10,                                // ponto zero do equipamento
    contador=0,                                   // conta as interrupções do encoder
    frequencia=1,
    espera;                                       //variavel que guarda o resultado de calculos de tempo               
      
unsigned long 
    t1 = 0, t2 = 0, t3 = 0, t4 = 0;//  variáveis associada ao millis() 
                                    
void setup(){
  Serial.begin(9600);
  attachInterrupt(0, interrompendo, RISING);      // CONFIGURAÃO DA INTERRUPÇÃO NO PINO 2
  pinMode(Ambu,INPUT);                            // PINO QUE MONITORA A PRESENÇA DO AMBU NO EQUIP
  lcd.begin (16,2);                               //SETA A QUANTIDADE DE COLUNAS(16) E O NÚMERO DE LINHAS(2) DO DISPLAY
  lcd.backlight();                                //LIGA O BACKLIGHT (LUZ DE FUNDO)                                 

 //pinos do motor
  pinMode(M1, OUTPUT); pinMode(M2, OUTPUT);  
  pinMode(EnableM, OUTPUT);                       // o pino enable habilita o funcionamento do motor. Pino pwm usado p controlar a velocidade do motor                                

 // pinos dos 5 potenciometros
  pinMode(potC, INPUT);pinMode(potF, INPUT);pinMode(potV, INPUT);  

  lcd.setBacklight(HIGH);                                     //iniciando com o display
  lcd.setCursor(5,0);  lcd.print("iVent");
  delay(2000);                                                // tempo p visualização do logo do iVent , rs
  lcd.clear();                }

void interrompendo(){  contador ++;  }                        // função de interrupção

void loop(){  
    if (marca==0) {                                           // se certifica que esse procedimento só acontece uma vez quando o equipamento é ligado                                                               
       marca=1;
       checando();                                            // chama a rotina que faz a checagem se o braço está na posição inicial(ponto zero)
       atualizaDisplay();                                     
       delay(1000);}                                          // delayzinho de espera p começar
                     
       leituraAmbu= digitalRead(Ambu);                        // verifica todo o tempo se o Ambu está no lugar certo
       if (leituraAmbu==LOW){CadeAmbu();}                     // se o Ambu não está, chama a rotina CadeAmbu()- ta invertido enquanto naõ resolvemos o HARDWARE
       atualizaDisplay();
  
//.........................................INSPIRAÇÃO DO PACIENTE - em 2 fases A e B        
 //faseA  
          t1 = millis();                                                                 //iniciando contagem de descida                                                           
          contador=0;                                                                    // reinicia cotagem
  while ( contador< valorC/2)                                                            // enquanto o braço não atingir metade do curso escolhido
        { analogWrite(EnableM, valorV);                                                  // habilita motor com pwm no valor de velocidade escolhido no potV    
          digitalWrite(M1,0);  digitalWrite(M2,1);                                       // motor gira no sentido 1                
          atualizaDisplay();                                                             // da uma passadinha p atualizar o display 
          digitalWrite(C1,0);  digitalWrite(C2,1);analogWrite(EnableM, 255);     }        // cooler sopra c a máxima potencia p evitar perda de fluxo                                                                             
                                                            
 //fase B                                                                                                                      
  while ( contador< valorC)                                                              // enquanto o braço não atingir o fim do percurso prescrito no pot C            
        { analogWrite(EnableM, valorV2);                                                 // menor velocidade nessa fase.          
          digitalWrite(M1,0);  digitalWrite(M2,1);                                       // motor gira  no sentido 1               
          atualizaDisplay();     }                                                       // da uma passadinha p atualizar o display 
          digitalWrite(M1,1);  digitalWrite(M2,1);                                       // acabado o loop paramos o motor                                                            
//freadinha  substituir por PID
         digitalWrite(M1,1);  digitalWrite(M2,0); delay(80);                             // estrategia provisoria de parada imediata do motor, dando um microgiro em sentido contrario                 
         digitalWrite(M1,1);  digitalWrite(M2,1); delay(40);                             // esperamos em breve implementar um PID                                                                                          
         t2=millis();     //encerrando contagem de descida
       
//........................ MOTOR RETORNANDO A POSIÇÃO SUPERIOR - PONTO ZERO
          t3=millis();                                                                   //iniciando contagem de subida
          digitalWrite(C1,1);  digitalWrite(C2,0);analogWrite(EnableM, 255);              // cooler inverte e colabora p eliminar parte dos gases tóxicos a partir do y da traqueia
     while (  contador> limiteUp)                                                        // enquanto a leitura da posição do braço indicar  não chegou ao ponto zero                                                                                   
        { analogWrite(EnableM, 250);                                                     // habilita o PWM com velocidade do potenciometro V
          digitalWrite(M1,1);  digitalWrite(M2,0);                                       // giro invertido
          atualizaDisplay();  }                                                               
          digitalWrite(M1,1); digitalWrite(M2,1);                                        // desliga motor
//freadinha substituir por PID
          digitalWrite(M1,0); digitalWrite(M2,1);  delay(100); 
          digitalWrite(M1,1); digitalWrite(M2,1);          
          if (contador!=limiteUp){checando();}                                           // confirma posição
          t4=millis();                                                                   //encerrando contagem de subida
 // tempo de espera                                                                      //delay de espera que completa o tempo faltante p fazer valer a relação i/e
          digitalWrite(C1,0);  digitalWrite(C2,1);analogWrite(EnableM, 255);             // cooler volta a dgirar enviando ar p os pumões ajudando no ppep(manter um mínimo de ar nos pulmões p q ele não colabe.
                                                                                         // temos que descobrir os pontos de inversão do cooler p maior eficiencia  
          espera=valorF*(t2-t1) - (t4-t3) ;         
          float xx=espera/20;                                                     
          for (int i = 0; i <=20; i++) {delay(xx); atualizaDisplay();}                   // poderiamos ter colocado um delay com o tempo xx, mas preferimos dividi-lo arbitrariamente em 20 pedaços
          frequencia= ((t2-t1)+(t4-t3))/1000;               }                             // a cada pedaço o procedimento da um pulinho em atualizaDisplay() q atualiza qq mudança ocorrida nos potenciometros.                                                                              
                                                                                        //Se não fosse feito isso, durante esse espaço de tempo as alteraçoes do usuário não seriam efetivadas e nem visualiadas                                                                       
void CadeAmbu(){ 
   while (leituraAmbu==HIGH)                                                             // verifica se há Ambu no lugar certo, e se não estiver pisca um "A"
       {  lcd.clear();lcd.setCursor(0,0);lcd.print("AMBU AUSENTE");
          delay(400);lcd.clear();delay(400);
          leituraAmbu= digitalRead(Ambu);         } }

void checando(){
          digitalWrite(C1,1);  digitalWrite(C2,1);analogWrite(EnableM, 0);               // cooler reversível desliga// rotina de inicio 
 while (  contador> limiteUp)                                                            // enquanto a leitura da posição do braço indicar  não chegou ao ponto zero                                                                                   
       {  analogWrite(EnableM, 70);                                                      // habilita o PWM com velocidade do potenciometro V
          digitalWrite(M1,1);  digitalWrite(M2,0);                                       // giro invertido
          atualizaDisplay();                                                                  
          lcd.clear();lcd.setCursor(0,0);lcd.print("Preparando");lcd.setCursor(0,1);lcd.print("inicialização");}
          digitalWrite(M1,1); digitalWrite(M2,1); 
                                                                                         
          digitalWrite(M1,0); digitalWrite(M2,1);  delay(100);                           //freadinha  substituir por PID
          digitalWrite(M1,1); digitalWrite(M2,1); 
          delay(1000);                                }                                  // espera um tempinho p iniciar o trabalho    
   
 void atualizaDisplay(){                                                                 // rotina resp. pela atualização da leitura dos 3 potenciometros no display 
                                                                                                   
          leituraC = analogRead(potC); valorC = map(leituraC, 0, 1023, 1, 70); 
          leituraF = analogRead(potF); valorF = map(leituraF, 0, 1023, 1, 4);        
          leituraV = analogRead(potV); valorV = map(leituraV, 0, 1023, 80,255);  valorV2 = map(valorV, 80, 255, 80,180);       
             lcd.setCursor(13,1);lcd.print(frequencia);
             lcd.setCursor(0,0);lcd.print("Vol:");
             lcd.setCursor(8,0);lcd.print("i/e 1/"); 
             lcd.setCursor(0,1);lcd.print("Vel:");
             lcd.setCursor(8,1);lcd.print("Freq");                   
          if (valorC != guardadoC){guardadoC=valorC; lcd.setCursor(4,0);lcd.print("    ");    lcd.setCursor(4,0);lcd.print(valorC); };
          if (valorV != guardadoV){guardadoV=valorV; lcd.setCursor(4,1);lcd.print("    ");    lcd.setCursor(4,1);lcd.print(valorV); };
          if (valorF != guardadoF){guardadoF=valorF; lcd.setCursor(14,0);lcd.print(valorF);                                         };
   }         








               

     

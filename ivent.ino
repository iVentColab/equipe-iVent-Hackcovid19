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
// temos agora 5 POTENCIÔMETROS, sendo 4 deles acessáveis, que são responsáveis pelo " ajuste ou programação" do comportamento do equipamento que é FEITA PELA EQUIPE MÉDICA
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
// 2º - velocidade dessa injeção - e aí entra também o algoritmo que falaremos adiante
// 3º - a relação entre tempo de inspiração e expiração - que foi resolvida de forma simples no 3º potenciometro que simplesmente dita um tempo de espera 
// até completar o ciclo de acordo com a proporção desejada escolhida pelo fisioterapeuta.
// 4º - o quarto potenciometro dita a taxa de desaceleração do braço mecânico, assim deixamos de necessitar de um algoritmo fixo e esse comportamento fica a cargo
// da equipe médica que saberá ajustar de modo eficiente no sentido de dar ao paciente uma inspiração mais confortável.

// próximos desafios devem aparecer quando o aparelho entrar em teste e quando a ANVISA fizer novas exigências
//  idéias boas são sempre bem vindas! Abraço!!  


//Pinagem---------------------------
const int M1 =13;                // out  - motor1  - pino 12 e 13 que levam informação de controle do arduino para a placa  controla o motor (módulo driver ponte H -L298N )
const int M2 =12;                // out  - motor1   
const int EnableM=11;      // out  - pino enable do motor. um PWM será aplicada nesse pino nos dando a possibilidade de controlar dessa forma a velocidade do motor  
const int Ambu=9;                 // in   - sensor microsuitch - informa ao sistema se há um ambu no aparelho ou se está fora de posição 
byte display7s[7]={2,3,4,5,6,7,8};            //pins de 2 a 8   display de 7 segmentos
                              

                                      
const int potV = A1;        // potenciometro responsável pela definição da velocidade do movimento do braço  - Knob VERDE 
const int potF = A2;        //    "              "        "      "      do ciclo ou frequencia da respiração = tempo de espera pós subida - - Knob AZUL
const int potC = A3;        // potenciometro responsável pela definição do curso do braço (quantidade de ar injetada no paciente)  - Knob AMARELO
                            // pode ser entendido também como o CURSO ou tamanho do percurso do braço mecânico
const int potA = A0;        // pot responsável por enviar um valor de velocidade para a FASE B do movimento do motor (substituto do ALGORITMO- por isso o A
                            // é o potenciometro abaixo do pot de velocidade -knob Azul tb
const int potE = A4;        // pot acoplado ao eixo do motor- funciona como um encoder- conforme o motor gira o arduino recebe 
                            // a informação desse pot que nos dá a posição do motor e consequentemente da posição do braço mecânico.

//variáveis---------------------------
int leituraAmbu;            // variável q sinaliza se há um Ambu no equipamento.
int leituraC;               // variável - valor analógico lido no potenciometroC  
int leituraV;               //    "          "    "         "     potenciometroE  
int leituraF;               //    "          "    "         "     potenciometroF  
int leituraA;               //    "          "    "         "     potenciometroA
int leituraE;               //    "          "    "         "     potenciometroM
int valorC;                 // variável - valor parametrizado da leituraC -(de 1 a 9 - é o que vai ser visível no display de leds)
int valorV;                 // variável - valor parametrizado da leituraE                   "
int valorF;                 // variável - valor parametrizado da leituraF                   "
int valorA;                 // variável - valor parametrizado da leituraA                   "
int valorE;                 // variável - valor parametrizado da leituraM                   "
int VpotC ;
float VpotV ;
int VpotF ;
int VpotA ;
int VpotE ;
int marca=0;                // variável - indica se o programa acabou de iniciar ou não
int valorguardadoC=0;       // variavel que guarda o último valor escolhido anteriormente para o curso do braço   - C
int valorguardadoV=0;       //      "    "     "        "     "      "          "           "    velocidade de descida do braço mecanico - V
int valorguardadoF=0;       //      "    "     "        "     "      "          "           "    para o tamanho do período ou frequencia - F
int valorguardadoA=0;       //      "    "     "        "     "      "          "           "    velocidade da  FASE B  
int valorguardadoE=0;       //      "    "     "   o valor da posição do braço mecanico  - E de Encoder
byte valor;                 // variavel que guarda valor que é lançado no display de leds
int xx;                     // variavel que guarda o resultado de calculos de tempo 
int limiteUp= 100;          // valor provisorio do petenciometro uando esta no topo ---deveria ser mais proximo de 0 mas o pot ta ruim, rs
int velocidadeFaseA;
float variante=0;
unsigned long ultimoTime;
unsigned long timeAgora;    
                            // a b c d e f g  
byte seven_seg_digits[14][7]={{0,0,0,0,0,0,0},
                             {0,0,0,1,1,0,0}, //Digito 1
                             {1,0,1,1,0,1,1}, //Digito 2
                             {1,0,1,1,1,1,0}, //Digito 3
                             {1,1,0,1,1,0,0}, //Digito 4
                             {1,1,2,0,1,1,0}, //Digito 5
                             {1,1,0,0,1,1,1}, //Digito 6
                             {0,0,1,1,1,0,0}, //Digito 7
                             {1,1,1,1,1,1,1}, //Digito 8
                             {1,1,1,1,1,0,0}, //Digito 9
                             {1,1,1,1,1,0,1}, //Letra A de Ambu  - aparece piscando se o Ambu não está no lugar certo
                             {0,1,0,0,0,0,1}, //aviso de subida  - acende enquanto checa a posição inicial e se encaminha p o ponto zero
                             {0,0,0,0,0,0,0}, //zerado  - limpa o display
                             {1,1,1,0,0,1,1}};//Letra E de Erro

                             
void setup(){        
Serial.begin(9600);

 //pinos do motor
  pinMode(M1, OUTPUT); pinMode(M2, OUTPUT);  
  pinMode(EnableM, OUTPUT);    // o pino enable habilita o funcionamento do motor. e é por isso que ele é ligado num pino do arduino que 
                                      //pode ser usado como PWM - ligando e desligando o motor p controlar a sua velocidade
 
 // pinos dos 5 potenciometros
  pinMode(potC, INPUT);pinMode(potF, INPUT);pinMode(potV, INPUT);pinMode(potA, INPUT);pinMode(potE, INPUT);    
  
  // pinos do display de 7 segmentos
 for (int nP=0;nP<=6;nP++){pinMode(display7s[nP],OUTPUT); }                        
  }
void loop(){  
  
  if (marca==0) {                                        // se certifica que esse procedimento só acontece uma vez quando o equipamento é ligado                                                               
  checando();  marca=1;  atualiza();                     // chama a rotina que faz a checagem se o braço está na posição inicial(ponto zero)
  delay(1000);   }                                       // delayzinho de espera p começar
                    
  leituraAmbu= digitalRead(Ambu);                        // verifica todo o tempo se o Ambu está no lugar certo
  if (leituraAmbu==LOW){CadeAmbu();}                     //INVERTEMOS ENQUANTO NÃO ESTÁ RESOLVIDO O HARDWARE- deverá ser novamente invertido quando isso for sanado
                                                         // se o Ambu não está, chama a rotina CadeAmbu()
                                                       
 VpotC = valorguardadoC*100;                             // esse multiplicador está sendo ajustado empiricamente p encontrar o mínimo e o máximo mais razoável     
 VpotV = velocidadeFaseA ;                               // velocidade p FASE A do percurso
 VpotF = valorguardadoF*350 ;                            // esse multiplicador está sendo ajustado p encontrar os tempos de espera mais razoável       
 //VpotA                                                 // essa informaçaõ dá ao fisioterapeuta a possibilidade de alterar de acordo com a sua observação a desaceleração do braço mecanico
 VpotE = valorguardadoE;                                 //essa informação substitui a versão anterior dos microsuitches - 
                                                         // E de encoder  - através desse valor saberemos aonde o braço se encontra e até aonde queremos que ele vá

//.........................................INSPIRAÇÃO DO PACIENTE        
 //faseA  
                                                          
     while ( leituraE< VpotC/2)                                                         // enquanto o braço não atingir metade do curso escolhido
       {  analogWrite(EnableM, VpotV);                                           // velocidade escolhida para FASE A, de acordo com o potV    
          digitalWrite(M1,0);  digitalWrite(M2,1);                                    // motor gira                 
          leituraE = analogRead(potE);                                                  // a cada loop faz nova leitura no Encoder
          atualiza();                                                                   // da uma passadinha p atualizar o display de leds
      }
                                                            
 //fase B       
 //  velocidade da FASE A desacelerando de acordo com o valor pot A
                                                                                                                          
  while ( leituraE<VpotC)                                                             // enquanto o braço não atingir o fim do percurso prescrito no pot C            
        { analogWrite(EnableM, (VpotV)-variante);                              // velocidade inicial = velocidade da FASE A e desacelerando conforme taxa escolhida em potA         
          digitalWrite(M1,0);  digitalWrite(M2,1);                                  // motor gira                 
          if (VpotV-(variante+VpotA)>60){variante=variante+VpotA;}                    // variante vai crescendo a cada loop de acordo com taxa prescrita pelo potenciometra                                                       
          atualiza();                                                                 // da uma passadinha p atualizar o display de leds 
         }
        variante=0; 
        digitalWrite(M1,1);  digitalWrite(M2,1);                                     // acabado o loop paramos o motor
                                                            
//freadinha
        digitalWrite(M1,1);  digitalWrite(M2,0); delay(80);                           // dá uma micro girada em sentido contrário p evitar que o motor gire mais                   
        digitalWrite(M1,1);  digitalWrite(M2,1); delay(40);                           // do que devia.  Chegamos aos valores empiricamente.  Ainda estamos calibrando
                                                                                        // essa foi a maneira encontrada p conseguir uma certa precisão no giro do motor
          
//........................ MOTOR RETORNANDO A POSIÇÃO SUPERIOR - PONTO ZERO
     
             
     leituraE = analogRead(potE);                                                       // leitura da posição do braço no Encoder
     while ( leituraE> limiteUp)                                                        // enquanto a leitura da posição do braço indicar  não chegou ao ponto zero                                                                                   
       {   analogWrite(EnableM, VpotV);                                          // habilita o PWM com velocidade do potenciometro V
           digitalWrite(M1,1);  digitalWrite(M2,0);                                   // giro invertido
           leituraE = analogRead(potE);                                                 // a cada loop faz nova leitura no Encoder
           atualiza();                                                                  // da uma passadinha p atualizar o display
        }
           digitalWrite(M1,1); digitalWrite(M2,1);                                    // desliga motor
//freadinha
           digitalWrite(M1,0); digitalWrite(M2,1);  delay(100); 
           digitalWrite(M1,1); digitalWrite(M2,1);          
           
// tempo de espera 
          int espera =  VpotF;                                     // potF controla o tempo de espera final depois de descer. 
          xx=espera/20;                                            // Sua utilidade é construir os formatos padrões 1/1 ou 1/1,5 1/2 etc. na relação insp/exp    
          for (int i = 0; i <=20; i++) {delay(xx); atualiza();}}   // lembrando que espera é o valor do potenciometro F responsavel pela frequencia  ou tamanho do ciclo total
                                                                   // da mesma forma que a relação insp/exp
                                                                   // poderiamos ter colocado um delay com o tempo xx, mas preferimos dividi-lo arbitrariamente em 20 pedaços
                                                                   // para que a cada pedaço o procedimento desse um pulinho na rotina atualiza() que é quem atualiza alguma mudança
                                                                   // nos potenciometros. Se não fosse feito isso, durante esse espaço de tempo as alteraçoes do usuário não seriam visualiadas
                                                                   
 
void display7(int valor){                                          // rotina responsavel pela atualização do display de leds
 byte pino = 2;
  for (byte contadorSegmentos = 0; contadorSegmentos <=6; ++contadorSegmentos){    
    digitalWrite(pino, seven_seg_digits[valor][contadorSegmentos]);  
    ++pino;                                                                      
  }}
                    
void alertaInicio(){                                               // pisca "E" no display
  valor=13;  display7(valor);delay(300);
  valor=12;  display7(valor);delay(300);
                  }
           
void CadeAmbu(){
    
     while (leituraAmbu==HIGH)                                     // verifica todo o tempo se há Ambu no lugar certo, e se não estiver pisca um "A"
     {      valor=11;
            display7(valor);  
            leituraAmbu= digitalRead(Ambu);
     }                                                        
 } 
 
void checando(){                                                           // ROTINA DE INICIO DE TRABALHAO

     leituraE = analogRead(potE);                                          // leitura da posição do braço no Encoder
     while ( leituraE>limiteUp)                                            // enquanto o braço não retornar pra próximo de zero
       {   delay(100);
           analogWrite(EnableM,70);                                 // habilita o PWM com velocidade lenta a ser verificada empiricamente
           digitalWrite(M1,1);  digitalWrite(M2,0);                      // SUBINDO                       
           leituraE = analogRead(potE);                                    // a cada loop faz nova leitura no Encoder
           valor =11; display7(valor);
        } 
        //freadinha
           digitalWrite(M1,0); digitalWrite(M2,1);  delay(100); 
           digitalWrite(M1,1); digitalWrite(M2,1); 
           delay(1000);                                                    // espera um tempinho p iniciar o trabalho
 }
           
 void atualiza(){                                                          // rotina resp. pela atualização da leitura dos 5 potenciometros e envio p a rotina display7 
                                                                           // só não vai enviar o potE porque não interessa                                     

           leituraC = analogRead(potC); valorC = map(leituraC, 0, 1023, 1, 9);       
           leituraV = analogRead(potV); valorV = map(leituraV, 0, 1023, 1, 9);
           velocidadeFaseA = map(leituraV, 0, 1023, 80, 255);
           leituraF = analogRead(potF); valorF = map(leituraF, 0, 1023, 1, 9); 
           leituraA = analogRead(potA); valorA = map(leituraA, 0, 1023, 1, 9);
           VpotA=map(leituraA, 0, 1023, 0.05, 4);
           leituraE = analogRead(apotE); valorE = map(leituraE, 0, 1023, 1, 9);                    
                       
           if ((valorC != valorguardadoC) )  {valorguardadoC=valorC; valor=valorC; display7(valor);}
           if ((valorV != valorguardadoV) )  {valorguardadoV=valorV; valor=valorV; display7(valor);}
           if ((valorF != valorguardadoF) )  {valorguardadoF=valorF; valor=valorF; display7(valor);} 
           if ((valorA != valorguardadoA) )  {valorguardadoA=valorA; valor=valorA; display7(valor);} 
         }  
       
               

     

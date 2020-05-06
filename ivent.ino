//******************************************************************************************************************************
// início do projeto                               - manha 27042020
// finalização do código                           - noite 29042020  
// Testes e correções no aparelho                  - noite 30042020
// Aparelho funcionando                            - noite 01052020
// Aperfeiçoamento-1                               - noite 03052020  velocidade em 2 fases e sistema de proteção inicial e inclusão de sensor p detectar a ausencia do Ambu no aparelho
// Envio do cód p programadores envolvidos no proj - tarde 04052020
// Aperfeiçoamento 2                               - noite 04052020  implentação do pwm, arrays no display e retirada do sensor de Ambu
// Aperfeiçoamento 3                               - noite 06052020  implementação de mais dois potenciometros, 
                                                                     1-substituindo a ideia de um algoritmo de desaceleração
                                                                      e criando um potenciometro ue deixa a desaceleração do braço a cargo do médico ou fisioterapeuta
                                                                     2-potenciomento funcionando como encoder.
                                                                     os códigos foram alterados, tudo com relação aos microsuitches está sendo substituido
                                                                     3- alguns pinos foram mudados de posição
                                                                     
//******************************************************************************************************************************

// informações importantes p facilitar o entendimento:

//trata-se de um programa que pretende fazer um motor ter um comportamento de acordo com o desejado
//as ordens p de funcionamento do motor são:

// digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); -para o motor girar no sentido de descida
// digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); -para o motor girar no sentido de subida
// digitalWrite(IN1,HIGH); digitalWrite(IN2,HIGH); - o motor para de girar 
//@@@@@@@@@@@@@ podemos implementar o uso do PWM que o arduino disponibiliza especialmente para isso @@@@@@@@@@@@@


// observação:é importante saber que a função do aparelho é "injetar" uma mistura gasosa hospitalar (ar enriquecido com O2) no paciente - inspiração. 
// A expiração do paciente é totalmente independente do aparelho, ou seja, o retorno do braço mecânico à posição superior não tem influência nenhuma
// sobre o paciente.

// outra coisa imortante
// temos 3 POTENCIÔMETROS responsaveis pela " ajuste ou programação" do comportamento do equip que é FEITA PELO FISIOTERAPEUTA ou médico no hospital ou CTI
// o equipamento tem que dar conta de pacientes extremamente diferentes ou seja permitir ao médico ou fisioterapeuta encontrar nesses 3 parametros (potenciômetros) 
// a combinação ideal p cada paciente.

//  Parâmetros:
// 1º - quantidade ou volume de ar injetada no paciente
// 2º - velocidade dessa injeção - e aí entra também o algoritmo que falaremos adiante
// 3º - a relação entre tempo de inspiração e expiração - que foi resolvida de forma simples no 3º potenciometro que simplesmente entra em espera 
// durante o tempo para completar o ciclo de acordo com a proporção desejada escolhida pelo fisioterapeuta.
//
// O Algoritmo:
// Nosso musculatura respiratória não funciona de modo linear, ou seja, é preciso haver uma curva que descreve aceleração, estabilização, desaceleração 
// e parada do motor de acordo com um modelo matemático da respiração humana.  
// Isso significará uma respiração mais confortável ao paciente.
//
// Até aqui fizemos uma programação baseada em observação visual e uma análise rudimentar de gráficos apresentados por outras universidade que tem 
// maior conhecimento dessa curva( principalmente da desaceleração do motor )
// simplificamos essa curva, porque percebemos que ela se parecia muito com duas retas e assim dividimos a movimentação do motor em duas fases com velocidades diferentes
// Podemos melhorar muito isso! 
//@@@@@@@@@@@@@  devemos nos dedicar a esse assunto prioritariamente.      @@@@@@@@@@@@@

//problemas resolvidos hoje 04/05/2020  
//-  tratar algumas variáveis, como as do display, como arrays. 
//- criar uns loops com for pra simplificar o codigo   
//- controlar a velocidade do motor pelos pinos de pwm do arduino - estávamos controlando de outra maneira

//  idéias boas são sempre bem vindas! Abraço!!  


//Pinagem---------------------------
int IN1 =13;                // out  - motor1   - pino 11 e 12 que levam informação de controle do arduino para a placa  controla o motor (módulo driver ponte H -L298N )
int IN2 =12;                // out  - motor1
int pinEnableMotor=11;      // in   - sensor - informa ao sistema se há um ambu no aparelho ou se está fora de posição   
int Ambu=9;
int display7s[7]={2,3,4,5,6,7,8};            //pins de 2 a 8   display de 7 segmentos
                           // a b c d e f g  
byte  displayConfig[11][7] ={{0,1,1,0,0,0,0}, //Digito 1
                             {1,1,0,1,1,0,1}, //Digito 2
                             {1,1,1,1,0,0,1}, //Digito 3
                             {0,1,1,0,0,1,1}, //Digito 4
                             {1,0,1,1,0,1,1}, //Digito 5
                             {1,0,1,1,1,1,1}, //Digito 6
                             {1,1,1,0,0,0,0}, //Digito 7
                             {1,1,1,1,1,1,1}, //Digito 8
                             {1,1,1,1,0,1,1}, //Digito 9
                             {0,0,0,0,0,0,0}, //vazio
                             (1,0,0,1,1,1,1)};//letra E
                                      
const int potV = A0;        // potenciometro responsável pela definição da velocidade do movimento do braço  - Knob VERDE 
const int potF = A1;        //    "              "        "      "      do ciclo ou frequencia da respiração = tempo de espera pós subida - - Knob AZUL
const int potC = A2;        // potenciometro responsável pela definição do curso do braço (quantidade de ar injetada no paciente)  - Knob AMARELO
                            // pode ser entendido também como o CURSO ou tamanho do percurso do braço mecânico
const int potA = A3;        // pot responsável por enviar um valor de velocidade para a FASE B do movimento do motor (substituto do ALGORITMO- por isso o A
const int potM = A4;        // pot acoplado ao eixo do motor- funciona como um encoder- conforme o motor gira o arduino recebe 
                            // uma informação deste pot da posição do motor e consequentemente da posição do braço mecânico.

//int MicrosuitchUP = 10;     // in   - sensor - microsuich q informa ao sistema quando e se o braço atingiu o ponto superior máximo do percurso
//int MicrosuitchDOWN = 9;    // in   - sensor - microsuich q informa ao sistema quando e se o braço atingiu o ponto inferior mínimo do percurso  

//variáveis---------------------------
int leituraAmbu;            // variável q sinaliza se há um Ambu no equipamento.
int leituraC;               // variável - valor analógico lido no potenciometroC  
int leituraV;               //    "          "    "         "     potenciometroE  
int leituraF;               //    "          "    "         "     potenciometroF  
int leituraA;               //    "          "    "         "     potenciometroA
int leituraM;               //    "          "    "         "     potenciometroM
int valorC;                 // variável - valor parametrizado da leituraC -(de 1 a 9 - é o que vai ser visível no display de leds)
int valorV;                 // variável - valor parametrizado da leituraE                   "
int valorF;                 // variável - valor parametrizado da leituraF                   "
int valorA;                 // variável - valor parametrizado da leituraA                   "
int valorM;                 // variável - valor parametrizado da leituraM                   "
int VpotC ;
int VpotV ;
int VpotF ;
int VpotA ;
int VpotM ;
int marca=0;                // variável - indica se o programa acabou de iniciar ou não
int contador=0;             // variável auxiliar para contagem de tempo
int valorguardadoC=0;       // variavel que guarda o último valor escolhido anteriormente para o curso do braço   - C
int valorguardadoV=0;       //      "    "     "        "     "      "          "           "    velocidade de descida do braço mecanico - V
int valorguardadoF=0;       //      "    "     "        "     "      "          "           "    para o tamanho do período ou frequencia - F
int valorguardadoA=0;       //      "    "     "        "     "      "          "           "    velocidade da  FASE B  
int valorguardadoM=0;       //      "    "     "   o valor da posição do braço mecanico

int valor;                  // variavel que guarda valor que é lançado no display de leds
int xx;                     // variavel que guarda o resultado de calculos de tempo 
//int interruptUP;            // variável q guarda a informação do status do MicrosuitchUP (pino 10) sinaliza chegada do braço no sensor de cima
//int interruptDOWN;          //    "          "          "           "      MIcrosuitchDOWN(pino 9)  "          "         "         "   de baixo

void setup(){        

 //pinos do motor
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);  pinMode(pinEnableMotor, OUTPUT);    
 
 // pinos dos 5 potenciometros
  pinMode(potC, INPUT);pinMode(potF, INPUT);pinMode(potV, INPUT);pinMode(potA, INPUT);pinMode(potM, INPUT);    
  
  // pinos do display de 7 segmentos
  for (int nP=0;nP<=6;nP++){pinMode(display7s[nP],OUTPUT); }                  //  2=d;  3= c;  4=e;  5=g; 6=f;  7=a;  8= b
 }
 
void loop(){  
  
  if (marca==0) {                                                                     // se certifica que esse procedimento só acontece uma vez quando o equipamento é ligado                                                               
  checando(); marca=1;                                                                // chama a rotina que faz a checagem se o braço está na posição inicial(ponto zero)
  leituraC = analogRead(potC); valorguardadoC = map(leituraC, 0, 1023, 1, 9);         // e lá também é atualizado o display de leds    
  leituraV = analogRead(potv); valorguardadoV = map(leituraV, 0, 1023, 1, 9);
  leituraF = analogRead(potF); valorguardadoF = map(leituraF, 0, 1023, 1, 9); 
  leituraA = analogRead(potA); valorguardadoA = map(leituraA, 0, 1023, 1, 9);
  leituraM = analogRead(potM); valorguardadoM = map(leituraM, 0, 1023, 1, 9);
  delay(2000);                                                                        // delayzinho de espera p começar
                }
  
 // baseado nos valores dos potenciômetros indica o valor correspondente a ser usado como parametro do comportamento do motor
                                                       
 VpotC = 650 +(valorguardadoC*450);                   // valor de VpotC vai ser entre 1100(650+(1*450)) até 4700(650+(9*450))      
 VpotV = leiturav;                                    // velocidade 
 VpotF = 50  +(valorguardadoF*375);                                             
 VpotA = valorguardadoA;                              // essa informaçaõ da ao fisioterapeuta a possibilidade de alterar de acordo com a sua observação a desaceleração do braço mecanico
 VpotM = valorguardadoM;                              //essa informação substitui a versão anterior dos microsuitches

//.........................................INSUFLANDO

          int medida = 60 * VpotC;                     // medida- é o valor diretamente ligado ao potenciometro C multiplicado por um valor observado empiricamente  
          int medidinha=30 * VpotC;                    // medidinha  é 50% da medida. Na FASE B, vai corresponder a 50% da velocidade do motor na FASE A
          int espera =  VpotF;                         // potF controla o tempo de espera final depois de descer. Sua utilidade é construir os formatos padrões 1/1 ou 1/1,5 1/2 etc. na relação insp/exp    
          
 //faseA  
         //interruptDOWN= digitalRead(MicrosuitchDOWN);                                  // faz a leitura do microsuitchDOWN, braço chegou em baixo? sim ou não 
          leituraM = analogRead(potM); 
          contador=0;
   while (contador < medida && valorguardadoM<230)                                     // enquanto o braço não chegou no final e nem contador chegou a contagem esperada...
      {   analogWrite(pinEnableMotor, VpotV);                                                 
          digitalWrite(IN1,HIGH);  digitalWrite(IN2,LOW);                               //motor gira de acordo com a velocidade escolhida no potenciometro                
          ajuste();                                                                     // da uma passadinha p atualizar o display de leds
          contador++;                                                                   // incrementa o contador
      }
                                                            
 //fase B                                                                                //mesma coisa da FASE A - só que com metade da velocidade
          contador=0;       
   while (contador < medidinha && interruptDOWN==HIGH)
       {                                                   
          analogWrite(pinEnableMotor, VpotA);  //@@@@@@@@@@@@@@@@@@@ AVALIAR EPIRICAMENTE O COMPORTAMENTO DO MOTOR
          digitalWrite(IN1,HIGH);  digitalWrite(IN2,LOW);  
          digitalWrite(IN1,HIGH);  digitalWrite(IN2,HIGH);                               // parada do motor                         
          ajuste();
          contador++;
        }                                                               
//freadinha
          digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); delay(30);                     // dá uma micro girada em sentido contrário p evitar que o motor gire mais                   
          digitalWrite(IN1,HIGH); digitalWrite(IN2,HIGH); delay(20);                     // do que devia.  Como estamos trabalhando com um motor de baixo custo sem encoder
                                                                                         // essa foi a maneira encontrada p conseguir uma certa precisão no giro do motor
          
//........................ MOTOR RETORNANDO A POSIÇÃO SUPERIOR - PONTO ZERO
     
           
    // while (interruptUP==HIGH)  
     @@@@@@@@@@@@@@@@@@@@2
     while encoder naõ chegar a tal numero
       {
           analogWrite(pinEnableMotor, 150);                                            // habilita o PWM com velocidade mediana a ser verificada empiricamente
           digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);                              // giro invertido
           digitalWrite(IN1,HIGH); digitalWrite(IN2,HIGH);                              // desliga motor
           ajuste();
        }
//freadinha
           digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  delay(30); 
           digitalWrite(IN1,HIGH); digitalWrite(IN2,HIGH);          
           
// tempo de espera 
           xx=espera/20;                                         // lembrando que espera é o valor do potenciometro F responsavel pela frequencia  ou tamanho do ciclo total
           for (int i = 0; i <=20; i++) {delay(xx); ajuste();}}  // da mesma forma que a relação insp/exp
                                                                 // poderiamos ter colocado um delay com o tempo xx, mas preferimo dividi-lo arbitrariamente em 20 pedaços
                                                                 // para que a cada pedaço o procedimento desse um pulinho na rotina ajuste() que é quem atualiza alguma mudança
                                                                 // nos potenciometros. Se não fosse feito isso durante esse espaço de tempo as alteraçoes do usuário não seriam visualiadas
                                                                 
void ajuste(){                                                   // rotina resp. pela atualização da leitura dos 3 potenciometros e envio p a rotina display7                                      
          
           leituraC = analogRead(potC); valorC = map(leituraC, 0, 1023, 1, 9);       
           leituraV = analogRead(potE); valorV = map(leituraV, 0, 1023, 1, 9);
           leituraF = analogRead(potF); valorF = map(leituraF, 0, 1023, 1, 9); 
           leituraA = analogRead(potF); valorA = map(leituraA, 0, 1023, 1, 9);
           leituraM = analogRead(potM); valorM = map(leituraM, 0, 1023, 1, 9);                    
                       
           if ((valorC != valorguardadoC) )  {valorguardadoC=valorC; valor=valorC; display7();}
           if ((valorV != valorguardadoV) )  {valorguardadoV=valorV; valor=valorV; display7();}
           if ((valorF != valorguardadoF) )  {valorguardadoF=valorF; valor=valorF; display7();} 
           if ((valorA != valorguardadoA) )  {valorguardadoA=valorA; valor=valorA; display7();} 
           if ((valorM != valorguardadoM) )  {valorguardadoM=valorM; valor=valorM; display7();}
           }

void display7(){                                                 // rotina responsavel pela atualização do display de leds
         
          for (int nD = 0; nD <= 6; nD++) { digitalWrite(display7s[nD], displayConfig[valor][nD]);} 
}
                    
void alertaInicio(){                                               // pisca "E" no display
  
           digitalWrite(2,HIGH);digitalWrite(3,LOW);digitalWrite(4,LOW);digitalWrite(5,HIGH);digitalWrite(6,HIGH);digitalWrite(7,HIGH);digitalWrite(8,HIGH); delay(200);
           digitalWrite(2,LOW);digitalWrite(3,LOW); digitalWrite(4,LOW); digitalWrite(5,LOW); digitalWrite(6,LOW) ;digitalWrite(7,LOW); digitalWrite(8,LOW); delay(200);
           }

void checando(){                                                                                        // ROTINA DE INICIO DE TRABALHAO
   
           interruptUP= digitalRead(MicrosuitchUP); 
           contador=0;                                                              
     while (interruptUP==HIGH  && contador<800) 
        {                                                      
           digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);ajuste();                                       // subindo lentamente até atingir o micro switch   
           digitalWrite(IN1,HIGH);digitalWrite(IN2,HIGH);delay(2);ajuste();
           interruptUP=digitalRead(MicrosuitchUP);contador++;
        }
        
           if (contador< 800){contador=0;} 
           else {                                                                                          
           digitalWrite(IN1,HIGH);digitalWrite(IN2,HIGH);
                   while (1==1)                                                                        // rs, condição p ficar eternamente piscando, vamos melhorar esse codigo
                      {
                        alertaInicio() ;                                                                // pisca leta E
                            
                      }
                }     
}                                                                               
                        

       
               

     

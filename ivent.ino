
int pinoLed = 13; //PINO DIGITAL UTILIZADO PELO LED  
int pinoSensor = 2; //PINO DIGITAL UTILIZADO PELO SENSOR
int contador;
void setup(){ 
  pinMode(4,OUTPUT);digitalWrite(4, 1);
  pinMode(3,OUTPUT);digitalWrite(3, 0);
  pinMode(pinoSensor, INPUT); //DEFINE O PINO COMO ENTRADA
  pinMode(pinoLed, OUTPUT); //DEFINE O PINO COMO SAÍDA   
  digitalWrite(pinoLed, LOW); //LED INICIA DESLIGADO
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), interrompendo1, RISING);           // CONFIGURAÇÃO DA INTERRUPÇÃO NO PINO 2

}  
   
void loop(){
     control = 0
     if (digitalRead(pinoSensor) == 0){ 
         digitalWrite(pinoLed, 1); 
        if (control == 0){
         contador ++;
         Serial.println(contador);}
         }
        control = 1
   else{ digitalWrite(pinoLed, 0); control = 0    }}
        
 
 void interrompendo1() 
 
  

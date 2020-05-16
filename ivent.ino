
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

          lcd.clear();   
  

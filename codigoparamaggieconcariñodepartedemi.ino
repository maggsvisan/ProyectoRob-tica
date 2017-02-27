#include <TimerOne.h>
volatile int count=0;
volatile int buttonState;
volatile int  state1,state2;
void setup() 
{
  Serial.begin(9600);
  pinMode(2,INPUT);
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
  digitalWrite(9,LOW);
  // Initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards
  Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
}
 
void loop()
{
 unsigned long copia,copia2;
 noInterrupts();
 copia=!buttonState;
 copia2=count;
 interrupts();
 Serial.println(copia);
 Serial.println(copia2);
}
 
/// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
void timerIsr()
{
  state1=!state1;
  digitalWrite(8,state1);
  buttonState=digitalRead(2);
  if(!buttonState){
    count = count+1;
    if(count <=5){
      state2=HIGH;
      digitalWrite(9,state2);
    }
    else if(count <=7){
      state2=LOW;
      digitalWrite(9,state2);
    }
    else{
      count=0;
      state2=LOW;
      digitalWrite(9,state2);
    }
   }
   else{
     count=0;
     state2=LOW;
     digitalWrite(9,state2);
   }
     
}
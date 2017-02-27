#include <TimerOne.h>
volatile int count=0, count2=0;;
volatile int buttonState, buttonState2;;
volatile int  state1,state2;
void setup() 
{
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
  digitalWrite(9,LOW);
  digitalWrite(8,LOW);
  // Initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards
  Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
}
 
void loop()
{
}
 
/// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
void timerIsr()
{
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

    buttonState2=digitalRead(3);
  if(!buttonState2){
    count2 = count2+1;
    if(count2 <=5){
      state1=HIGH;
      digitalWrite(8,state1);
    }
    else if(count2 <=7){
      state1=LOW;
      digitalWrite(8,state1);
    }
    else{
      count2=0;
      state1=LOW;
      digitalWrite(8,state1);
    }
   }
   else{
     count2=0;
     state1=LOW;
     digitalWrite(8,state1);
   }
     
}
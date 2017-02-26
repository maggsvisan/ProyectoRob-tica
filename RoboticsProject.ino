/// constans for ON/OFF with interrrupts ///
const int buttonPin1 = 2;     // the number of the pushbutton pin
const int ledPin1 =  9;      // the number of the LED pin

const int buttonPin2 = 3;     // the number of the pushbutton pin
const int ledPin2 =  8;      // the number of the LED pin
// variables will change:
volatile int buttonState1 = 0;         // variable for reading the pushbutton status
volatile int buttonState2 = 0;         // variable for reading the pushbutton status
//////////////////////////////////////////////////

//// contants for PWM and DC ///
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
/////////////////////////

/// initial values for servo motor ///
int servoPin = 9;
int potPin = 0;
int angle;
int pwm;
int val;
/////////////////////////


void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);

  // Attach an interrupt to the ISR vector
  attachInterrupt(digitalPinToInterrupt(buttonPin1), pin_ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonPin2), pin_ISR2, CHANGE);
  
  // initialize pin servo 
  pinMode(servoPin, OUTPUT);
  pinMode(potPin,INPUT);
  
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}



void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  // change the analog out value:
  //analogWrite(analogOutPin, outputValue);

  moverDC(analogOutPin,outputValue);
  
  ///// move Servo ////
  val=analogRead(potPin);
  angle=map(val,0,1023,0,255);
  servoPulse(servoPin, angle);  
  //////////////////

  // print the results to the serial monitor:
  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("\t output = ");
  Serial.println(outputValue);

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);

}
 
void servoPulse (int servo, int angle)
{
 pwm = (angle*11) + 500;      // Convert angle to microseconds
 digitalWrite(servo, HIGH);
 delayMicroseconds(pwm);
 digitalWrite(servo, LOW);
 delay(50);                   // Refresh cycle of servo
}

void moverDC (int pin, int angulo)
{
   float pausa;
   pausa = angulo*2000.0/180.0 + 500;
   digitalWrite(pin, HIGH);
   delayMicroseconds(pausa);
   digitalWrite(pin, LOW);
   delayMicroseconds(25000-pausa);
}

void pin_ISR1() {
  buttonState1 = digitalRead(buttonPin1);
  digitalWrite(ledPin1, buttonState1);
}

void pin_ISR2() {
  buttonState2 = digitalRead(buttonPin2);
  digitalWrite(ledPin2, buttonState2);
}

const int buttonPin1 = 2;     // the number of the pushbutton pin
const int ledPin1 =  9;      // the number of the LED pin

const int buttonPin2 = 3;     // the number of the pushbutton pin
const int ledPin2 =  8;      // the number of the LED pin
// variables will change:
volatile int buttonState1 = 0;         // variable for reading the pushbutton status
volatile int buttonState2 = 0;         // variable for reading the pushbutton status

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
}

void loop() {
  // Nothing here!
}

void pin_ISR1() {
  buttonState1 = digitalRead(buttonPin1);
  digitalWrite(ledPin1, buttonState1);
}

void pin_ISR2() {
  buttonState2 = digitalRead(buttonPin2);
  digitalWrite(ledPin2, buttonState2);
}
//Control a Parallax Servo without using the Arduino library

int servoPin = 9;
int potPin = 0;
int angle;
int pwm;
int val;

void setup()
{
 pinMode(servoPin, OUTPUT);
 pinMode(potPin,INPUT);
}

void loop ()
{  val=analogRead(potPin);
   angle=map(val,0,1023,0,255);
   servoPulse(servoPin, angle);  
}
 
void servoPulse (int servo, int angle)
{
 pwm = (angle*11) + 500;      // Convert angle to microseconds
 digitalWrite(servo, HIGH);
 delayMicroseconds(pwm);
 digitalWrite(servo, LOW);
 delay(50);                   // Refresh cycle of servo
}

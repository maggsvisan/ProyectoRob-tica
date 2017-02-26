#include <avr/io.h> // header file file for input output pins
#include <util/delay.h> // header file for delay.

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define TCCR0A _SFR_IO8(0x24)
#define WGM00 0
#define WGM01 1
#define COM0B0 4
#define COM0B1 5
#define COM0A0 6
#define COM0A1 7

#define TCCR0B _SFR_IO8(0x25)
#define CS00 0
#define CS01 1
#define CS02 2
#define WGM02 3
#define FOC0B 6
#define FOC0A 7

#define TCCR1A _SFR_MEM8(0x80)
#define WGM10 0
#define WGM11 1
#define COM1B0 4
#define COM1B1 5
#define COM1A0 6
#define COM1A1 7

#define TCCR1B _SFR_MEM8(0x81)
#define CS10 0
#define CS11 1
#define CS12 2
#define WGM12 3
#define WGM13 4
#define ICES1 6
#define ICNC1 7

#define TCCR2A _SFR_MEM8(0xB0)
#define WGM20 0
#define WGM21 1
#define COM2B0 4
#define COM2B1 5
#define COM2A0 6
#define COM2A1 7

#define TCCR2B _SFR_MEM8(0xB1)
#define CS20 0
#define CS21 1
#define CS22 2
#define WGM22 3
#define FOC2B 6
#define FOC2A 7

uint8_t analog_reference = DEFAULT;

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
  _BV(0), /* 0, port D */
  _BV(1),
  _BV(2),
  _BV(3),
  _BV(4),
  _BV(5),
  _BV(6),
  _BV(7),
  _BV(0), /* 8, port B */
  _BV(1),
  _BV(2),
  _BV(3),
  _BV(4),
  _BV(5),
  _BV(0), /* 14, port C */
  _BV(1),
  _BV(2),
  _BV(3),
  _BV(4),
  _BV(5),
};
const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
  PD, /* 0 */
  PD,
  PD,
  PD,
  PD,
  PD,
  PD,
  PD,
  PB, /* 8 */
  PB,
  PB,
  PB,
  PB,
  PB,
  PC, /* 14 */
  PC,
  PC,
  PC,
  PC,
  PC,
};
static void turnOffPWM(uint8_t timer){
  switch (timer)
  {
    case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
    case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
    case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
    case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
    case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
    case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
  }
}


void pinMode(uint8_t pin, uint8_t mode)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *reg, *out;

  if (port == NOT_A_PIN) return;
  reg = portModeRegister(port);
  out = portOutputRegister(port);

  if (mode == INPUT) { 
    uint8_t oldSREG = SREG;
    cli();
    *reg &= ~bit;
    *out &= ~bit;
    SREG = oldSREG;
  } else if (mode == INPUT_PULLUP) {
    uint8_t oldSREG = SREG;
    cli();
    *reg &= ~bit;
    *out |= bit;
    SREG = oldSREG;
  } else {
    uint8_t oldSREG = SREG;
    cli();
    *reg |= bit;
    SREG = oldSREG;
  }
}

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = 
{
  NOT_ON_TIMER, /* 0 - port D */
  NOT_ON_TIMER,
  NOT_ON_TIMER,
  TIMER2B,
  NOT_ON_TIMER,
  TIMER0B,
  TIMER0A,
  NOT_ON_TIMER,
  NOT_ON_TIMER, /* 8 - port B */
  TIMER1A,
  TIMER1B,
  TIMER2A,
  NOT_ON_TIMER,
  NOT_ON_TIMER,
  NOT_ON_TIMER,
  NOT_ON_TIMER, /* 14 - port C */
  NOT_ON_TIMER,
  NOT_ON_TIMER,
  NOT_ON_TIMER,
  NOT_ON_TIMER,
};


long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void analogReference(uint8_t mode)
{
  analog_reference = mode;
}

int analogRead(uint8_t pin){
  uint8_t low, high;
  if (pin >= 14) 
  {
    pin -= 14; // allow for channel or pin numbers
  }
  ADMUX = (analog_reference << 6) | (pin & 0x07);
  sbi(ADCSRA, ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  low  = ADCL;
  high = ADCH;
  return (high << 8) | low;
}

void analogWrite(uint8_t pin, int val)
{
  pinMode(pin, OUTPUT);
  if (val == 0)
  {
    digitalWrite(pin, LOW);
  }
  else if (val == 255)
  {
    digitalWrite(pin, HIGH);
  }
  else
  {
    switch(digitalPinToTimer(pin))
    {
      case TIMER0A:                
      sbi(TCCR0A, COM0A1);
      OCR0A = val; // set pwm duty
      break;

      case TIMER0B:  
      sbi(TCCR0A, COM0B1);
      OCR0B = val; // set pwm duty
      break;

      case TIMER1A:
      sbi(TCCR1A, COM1A1);
      OCR1A = val; // set pwm duty
      break;

      case TIMER1B:
      sbi(TCCR1A, COM1B1);
      OCR1B = val; // set pwm duty
      break;

      case TIMER2A:
      sbi(TCCR2A, COM2A1);
      OCR2A = val; // set pwm duty
      break;

      case TIMER2B:
      sbi(TCCR2A, COM2B1);
      OCR2B = val; // set pwm duty
      break;

      case NOT_ON_TIMER:
      default:
      if (val < 128) {
        digitalWrite(pin, LOW);
      } else {
        digitalWrite(pin, HIGH);
      }
    }
  }
}



void digitalWrite(uint8_t pin, uint8_t val)
{
  uint8_t timer = digitalPinToTimer(pin);
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *out;

  if (port == NOT_A_PIN) 
    return;

  if (timer != NOT_ON_TIMER) 
    turnOffPWM(timer);
  
  out = portOutputRegister(port);

  uint8_t oldSREG = SREG;
  cli();

  if (val == LOW) 
  {
    *out &= ~bit;
    } 
  else 
  {
    *out |= bit;
  }
  SREG = oldSREG;
}



int main(void){
  const int analogInPin = A0;  
  const int analogOutPin = 9; 

  int sensorValue = 0;        
  int outputValue = 0; 

  while (1)
  {
    sensorValue = analogRead(analogInPin);
  
    outputValue = map(sensorValue, 0, 1023, 0, 255);
  
    analogWrite(analogOutPin, outputValue);

    _delay_ms(200);
  }    
}

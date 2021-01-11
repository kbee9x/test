#include <avr/interrupt.h> //thu vien ngat
#include <SoftwareSerial.h>
//sensor
#define trig A1
#define echo A0
//UART
#define Rx A2  
#define Tx A3

//DRIVER RIGHT WHEEL(1)
#define R_IS 4
#define L_IS 7

#define R_EN1 5
#define R_PWM1 6
#define L_EN1 8
#define L_PWM1 9

//DRIVER LEFT WHEEL(2)
#define R_EN2 12
#define R_PWM2 10
#define L_EN2 13
#define L_PWM2 11

// ENCODER RIGHT WHEEL(1)
#define phaseA1 2    
#define phaseB1 1

// ENCODER LEFT WHEEL(2)
#define phaseA2 3    
#define phaseB2 0

//read encoder pulse
int pos1 = 0;
int pos2 = 0;

//initializing serial gate
SoftwareSerial mySerial(Rx, Tx);
int distance = 0;

void count1()
{
  if (digitalRead(phaseB1) == LOW)
  {
    pos1++;
  }
  else
  {
    pos1--;
  }
}
void count2()
{
  if (digitalRead(phaseB2) == HIGH)
  {
    pos2++;
  }
  else
  {
    pos2--;
  }
}
void on()
{
  digitalWrite(R_EN1, HIGH);
  digitalWrite(L_EN1, HIGH); 
  digitalWrite(R_EN2, HIGH);
  digitalWrite(L_EN2, HIGH); 
}
void off()
{
  digitalWrite(R_EN1, LOW);
  digitalWrite(L_EN1, LOW); 
  digitalWrite(R_EN2, LOW);
  digitalWrite(L_EN2, LOW); 
}
void gobackward()
{
  on();
  //go back ward (75% PWM)
  analogWrite(R_PWM1,0);
  analogWrite(L_PWM1,190);
  analogWrite(R_PWM2,190);
  analogWrite(L_PWM2,0);
  delay(20);
  off();
}
void goforward()
{
  on();
  //go forward (75% PWM)
  analogWrite(R_PWM1,190);
  analogWrite(L_PWM1,0);
  analogWrite(R_PWM2,0);
  analogWrite(L_PWM2,190);
  delay(20);
  off();
}
void turnright()
{
    pos2 = 0;
    digitalWrite(R_EN1, LOW);
    digitalWrite(L_EN1, LOW); 
    digitalWrite(R_EN2, HIGH);
    digitalWrite(L_EN2, HIGH); 
    if (pos2 >= 900)
    {
      digitalWrite(R_EN2, LOW);
      digitalWrite(L_EN2, LOW);
      pos2 = 0;   
    }
}
void turnleft()
{
    pos1 = 0;
    digitalWrite(R_EN1, HIGH);
    digitalWrite(L_EN1, HIGH); 
    digitalWrite(R_EN2, LOW);
    digitalWrite(L_EN2, LOW);
    for (int i = 0; i < 10000; i++)
    {
      analogWrite(R_PWM1,190);
      analogWrite(L_PWM1,0);
      Serial.print(pos1);
      //Serial.print("\n");
      if (pos1 >= 900)
      {
        digitalWrite(R_EN1, LOW);
        digitalWrite(L_EN1, LOW);
        break; 
      }
    }  
}
void setup()
{
    //arduino f = 16Mhz prescale 64 -> f = 250kHz -> T = 0.4us -> count 0-65536 = 0.4x65536=0.26s
  Serial.begin(115200);
  mySerial.begin(115200);
  
  pinMode(R_IS, OUTPUT);
  pinMode(L_IS, OUTPUT);
  digitalWrite(R_IS, LOW);
  digitalWrite(L_IS, LOW);
  
  pinMode(R_EN1, OUTPUT);
  pinMode(L_EN1, OUTPUT);
  pinMode(R_PWM1, OUTPUT);
  pinMode(L_PWM1, OUTPUT);
  digitalWrite(R_EN1, HIGH);
  digitalWrite(L_EN1, HIGH);

  pinMode(R_EN2, OUTPUT);
  pinMode(L_EN2, OUTPUT);
  pinMode(R_PWM2, OUTPUT);
  pinMode(L_PWM2, OUTPUT);
  digitalWrite(R_EN2, HIGH);
  digitalWrite(L_EN2, HIGH);
  
  //interrupt 0 is pin 2
  attachInterrupt(0, count1 , FALLING);
  pinMode (phaseA1, INPUT_PULLUP);
  pinMode (phaseB1, INPUT_PULLUP);
  //interrupt 1 is pin 3
  attachInterrupt(1, count2 , FALLING);
  pinMode (phaseA2, INPUT_PULLUP);
  pinMode (phaseB2, INPUT_PULLUP); 

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  //turn off interrupt global
  cli();
  //reset timer/counter 1
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  //set up t/c 1
  TCCR1B |= (1 << CS11) | (1 << CS10); //set prescale 64 (register tccr have cs10,cs11,cs12 bit set prescale for t/c)
  TCNT1 = 0; // set value for t/c 1
  TIMSK1 = (1 << TOIE1);  // register timsk have toie bit (enable interrupt when overflow t/c 1)
  //enable interrupt global
  sei();
}
void distanceMeasure()
{
  unsigned long timeSensor = 0;
  // create 1 pulse 10us 
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW); 

  timeSensor = pulseIn(echo, HIGH);
  // calculate distance, velocity of sound 340m/s -> 29.4us/cm
  distance = int(timeSensor / 2 / 29.412);
  Serial.println(distance);
}
//function is t/c 1 overflow
int check = 0;
ISR(TIMER1_OVF_vect)
{
  TCNT1 = 0;
  distanceMeasure();
  
  if(distance < 100)
  {
    //mySerial.print('1');
    check = 1;
  }
  else if ((distance > 100) && (check == 1))
  {
    //mySerial.print('0');
    check = 0;
  }
  
}
void loop()
{
  for(int i = 0; i < 100; i++)
  {
    goforward();
  }
  turnleft();
  delay(500);
  
 
}

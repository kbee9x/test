
#include <SoftwareSerial.h>
//sensor
#define trig A1
#define echo A0

#define Rx 11  
#define Tx 12 


//initializing serial gate
SoftwareSerial mySerial(Rx, Tx);
int distance = 0;

void setup() {
  //arduino f = 16Mhz prescale 64 -> f = 250kHz -> T = 0.4us -> count 0-65536 = 0.4x65536=0.26s
  Serial.begin(115200);
  mySerial.begin(115200);

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
}
//function is t/c 1 overflow

ISR(TIMER1_OVF_vect)
{
  TCNT1 = 0;
  distanceMeasure();
  Serial.println(distance);
  //if(distance < 100)
  {
    //mySerial.print('1'); //send turn on alarm signal to node esp
  }
  //else
  {
    //mySerial.print('0');
  }
  
}
void loop() {
  
  
}

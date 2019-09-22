/** File Information **********************************************************
 * Name:        Simple_Trapezoidal.ino
 * Authors:     Will Floyd-Jones and Patrick Grady 
 * Date:        Nov 11, 2014
 * Version:     1.0
 * Comments:    Trapezoidal commuation based on hall effect state table
 *******************************************************************************/


#include "TimerOne.h"
#include "SPI.h"

#define INHA 4
#define INLA 3
#define INHB 20 
#define INLB 21
#define INHC 23
#define INLC 22

#define HALL1 9
#define HALL2 8
#define HALL3 7

#define THROTTLE 17

#define DRV_CS 10
#define DRV_EN_GATE 19

// hall state table
// the index is the decimal value of the hall state
// the value is the position (255 is an error)
uint8_t hallOrder[] = {255, 2, 0, 1, 4, 3, 5, 255};

uint16_t throttle = 0;
unsigned long prevTime = 0;

void setup(){
  pinMode(INHA, OUTPUT);
  pinMode(INLA, OUTPUT);
  pinMode(INHB, OUTPUT);
  pinMode(INLB, OUTPUT);
  pinMode(INHC, OUTPUT);
  pinMode(INLC, OUTPUT);

  pinMode(DRV_EN_GATE, OUTPUT);
  digitalWriteFast(DRV_EN_GATE, LOW);
  delay(100);
  digitalWriteFast(DRV_EN_GATE, HIGH);

  pinMode(HALL1, INPUT_PULLUP);
  pinMode(HALL2, INPUT_PULLUP);
  pinMode(HALL3, INPUT_PULLUP);

  pinMode(THROTTLE, INPUT);

  // change the analog write frequency to 8 kHz
  analogWriteFrequency(INHA, 8000);
  analogWriteFrequency(INHB, 8000);
  analogWriteFrequency(INHC, 8000);
  analogWriteResolution(12); // write from 0 to 2^12 = 4095

  // call the commutate function every 20 microseconds
  Timer1.initialize(20);
  Timer1.attachInterrupt(commutate);

  Serial.begin(9600);
}

// get the thottle value every 10 ms
void loop(){
  unsigned long currentTime = millis();
  if (prevTime - currentTime > 10){
    throttle = 4*analogRead(THROTTLE);
    prevTime = currentTime; 
  }
  int hallsensorval = analogRead(HALL1);
  int hallsensorval2 = analogRead(HALL2);
  int hallsensorval3 = analogRead(HALL3);

  float voltage = hallsensorval * (5.0/1023);
  float voltage2 = hallsensorval2 * (5.0/1023);
  float voltage3 = hallsensorval3 * (5.0/1023);

  Serial.print(voltage);
  Serial.print("\t");
  Serial.print(voltage2);
  Serial.print("\t");
  Serial.println(voltage3);
  
}

// ISR called every 20 microseconds
void commutate(){
  uint8_t hallState = (digitalReadFast(HALL3) << 2) | 
                      (digitalReadFast(HALL2) << 1) | 
                      (digitalReadFast(HALL1) << 0);
  
  uint8_t pos = hallOrder[hallState];
  if(pos <= 6){ // we have a valid hall state
    writeState(pos);
  }
}

// write the state according to the position
void writeState(uint8_t pos){
  switch(pos){
  case 0://LOW A, HIGH B
    writeLow(0b001);
    writeHigh(0b010);
    break;
  case 1://LOW A, HIGH C
    writeLow(0b001);
    writeHigh(0b100);
    break;
  case 2://LOW B, HIGH C
    writeLow(0b010);
    writeHigh(0b100);
    break;
  case 3://LOW B, HIGH A
    writeLow(0b010);
    writeHigh(0b001);
    break;
  case 4://LOW C, HIGH A
    writeLow(0b100);
    writeHigh(0b001);
    break;
  case 5://LOW C, HIGH B
    writeLow(0b100);
    writeHigh(0b010);
    break;
  }
}

// write the phase to the low side gates 
// 1 = A, 2 = B, 3 = C
void writeLow(uint8_t phase){
  digitalWriteFast(INLA, (phase&(1<<0)));
  digitalWriteFast(INLB, (phase&(1<<1)));
  digitalWriteFast(INLC, (phase&(1<<2)));
}

// write the phase to the high side gates
// 1 = A, 2 = B, 3 = C
void writeHigh(uint8_t phase){
  switch(phase){
  case 0b001: // Phase A
    analogWrite(INHA, throttle);
    analogWrite(INHB, 0);
    analogWrite(INHC, 0);
    digitalWriteFast(INHB, LOW);
    digitalWriteFast(INHC, LOW);
    break;
  case 0b010: // Phase B
    digitalWriteFast(INHA, LOW);
    analogWrite(INHB, throttle);
    analogWrite(INHA, 0);
    analogWrite(INHC, 0);
    digitalWriteFast(INHC, LOW);
    break;
  case 0b100:// Phase C
    digitalWriteFast(INHA, LOW);
    digitalWriteFast(INHB, LOW);
    analogWrite(INHC, throttle);
    analogWrite(INHB, 0);
    analogWrite(INHA, 0);
    break;
  }
}

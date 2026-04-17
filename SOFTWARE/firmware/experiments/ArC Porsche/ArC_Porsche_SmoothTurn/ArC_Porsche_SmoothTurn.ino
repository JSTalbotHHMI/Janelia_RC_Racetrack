#include "MCP4261.h"

#define wpPin 7

// uint8_t accFwd = 155;
// uint8_t accStop = 135;
// uint8_t accRev = 123;

uint8_t accFwd = 80;
uint8_t accStop = 135;
uint8_t accRev = 160;
uint8_t accCurrent = accFwd;

// uint8_t strR = 230;
// uint8_t strS = 150;
// uint8_t strL = 70;

uint8_t strR = 40;
uint8_t strS = 150;
uint8_t strL = 200;
uint8_t strCurrent = strS;


//  select, shutdown, dataIn, dataOut, clock == SOFTWARE SPI
MCP4261 pot(10, 6, 11, 12, 13);

//  select, shutdown, &SPI === HW SPI UNO clock = 13, dataOut = 11
// MCP4261 pot(10, 6, &SPI);


void setup()
{
  Serial.begin(115200);

  pinMode(wpPin,OUTPUT);
  delay(10);
  digitalWrite(wpPin,HIGH);

  SPI.begin();

  pot.begin();
}


void loop() {
  pot.setValue(1, accCurrent);

  for(; strCurrent <= strL; strCurrent++)
  {
    pot.setValue(0, strCurrent);
    Serial.println(strCurrent);
    delay(3);
  }

  delay(20);

  for(; strCurrent >= strR; strCurrent--)
  {
    pot.setValue(0, strCurrent);
    Serial.println(strCurrent);
    delay(3);
  }

  delay(20);
}
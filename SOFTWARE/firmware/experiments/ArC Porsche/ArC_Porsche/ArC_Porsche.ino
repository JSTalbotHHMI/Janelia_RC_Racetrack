#include "MCP4261.h"

#define wpPin 1

uint32_t start, stop;

uint8_t accMax = 155;
uint8_t accZero = 135;
uint8_t accMin = 123;

uint8_t strMax = 230;
uint8_t strZero = 150;
uint8_t strMin = 90;


//  select, shutdown, dataIn, dataOut, clock == SOFTWARE SPI
MCP4261 pot(10, 6, 11, 12, 13);

//  select, shutdown, &SPI === HW SPI UNO clock = 13, dataOut = 11
// MCP4261 pot(10, 6, &SPI);


void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("MCP4261_LIB_VERSION: ");
  Serial.println(MCP4261_LIB_VERSION);
  Serial.println();

  pinMode(wpPin,OUTPUT);
  delay(10);
  digitalWrite(wpPin,HIGH);

  SPI.begin();

  pot.begin();
  
  Serial.println("\nDone...");
}


void loop() {
  Serial.println("FL");
  pot.setValue(0, strMin);
  pot.setValue(1, accMax);
  delay(500);

  Serial.println("FS");
  pot.setValue(0, strZero);
  delay(500);

  Serial.println("FR");
  pot.setValue(0, strMax);
  delay(500);

  Serial.println("SL");
  pot.setValue(0, strMin);
  pot.setValue(1, accZero);
  delay(500);

  Serial.println("SS");
  pot.setValue(0, strZero);
  delay(500);

  Serial.println("SR");
  pot.setValue(0, strMax);
  delay(500);

  
  Serial.println("RL");
  pot.setValue(0, strMin);
  pot.setValue(1, accMin);
  delay(500);

  Serial.println("RS");
  pot.setValue(0, strZero);
  delay(500);

  Serial.println("RR");
  pot.setValue(0, strMax);
  delay(500);


}
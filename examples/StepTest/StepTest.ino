#include <SPI.h>
#include <AMIS30543.h>
#include <AStar32U4Prime.h>  // TODO: remove

const uint8_t amisSlaveSelect = 10;

AMIS30543 stepper;

AStar32U4PrimeButtonA buttonA;
AStar32U4PrimeButtonB buttonB;
AStar32U4PrimeButtonC buttonC;

SPISettings settings(500000, MSBFIRST, SPI_MODE0);

#define STEP 9  //STEP

//Commands
#define READ B000  //  CMD2=0
#define WRITE B100  //  CMD2=1

//Addresses of control registers
#define WR 0x0
#define CR0 0x1
#define CR1 0x2
#define CR2 0x3
#define CR3 0x9

//Addresses of status regusters
#define SR0 0x4
#define SR1 0x5
#define SR2 0x6
#define SR3 0x7
#define SR4 0xA

//Control register parameters
#define DIRCTRL 1
#define NXTP 2
#define EMC 3
#define SLAT 4
#define SLAG 5
#define PWMF 6
#define PWMJ 7
#define SM 8
#define ESM 9
#define SLP 10
#define MOTEN 11
#define CUR 12
#define WDEN 13
#define WDT 14


void setup()
{
  delay(1000);
  Serial.begin(9600);
  Serial.println("start");

  SPI.begin();  //initialize SPI
  stepper.init(amisSlaveSelect);

  pinMode(STEP, OUTPUT);

  Serial.println("AMIS SPI test");
  while (1)
  {
    if (buttonB.getSingleDebouncedRelease())
    {
      Serial.println("button pushed");
      break;
    }
  }
  //All registers should start out with 0 in them after reset
  //so to verify that SPI is working, we will write to one and
  //then read from it.
  Serial.print("DIRCTRL = ");
  Serial.println((SPItransmit(READ, CR1, 0) & B10000000)>>7);  //print value from DIRCTRL
  SPIwriteParam(DIRCTRL, 1);  //write to DIRCTRL
  Serial.print("DIRCTRL = ");
  Serial.println((SPItransmit(READ, CR1, 0) & B10000000)>>7);  //print value from DIRCTRL

  //Next lets test some of the status bits like charge pump
  //failure, micro-step position, and OPEN Coil
  //First we might have to enable the motor outputs
  Serial.print("MOTEN = ");
  Serial.println((SPItransmit(READ, CR2, 0) & B10000000)>>7);  //print value from DIRCTRL
  SPIwriteParam(MOTEN, 1);  //write to DIRCTRL
  Serial.print("MOTEN = ");
  Serial.println((SPItransmit(READ, CR2, 0) & B10000000)>>7);  //print value from DIRCTRL

  Serial.print("CPFail = ");
  Serial.println((SPItransmit(READ, SR0, 0) & B00100000)>>5);
  Serial.print("Micro-step position = ");
  Serial.println(SPItransmit(READ, SR3, 0) & B01111111);
  Serial.print("OPEN Coil X = ");
  Serial.println((SPItransmit(READ, SR0, 0) & B00001000)>>3);
  Serial.print("OPEN Coil Y = ");
  Serial.println((SPItransmit(READ, SR0, 0) & B00000100)>>2);

  // Next lets try setting the current limit higher
  Serial.print("CUR = ");
  Serial.println((SPItransmit(READ, CR0, 0) & B00011111)>>0);  //print value from DIRCTRL
  SPIwriteParam(CUR, 10);  //write to DIRCTRL
  Serial.print("CUR = ");
  Serial.println((SPItransmit(READ, CR0, 0) & B00011111)>>0);  //print value from DIRCTRL
}

void loop()
{
  // The NXT minimum high pulse width is 2 microseconds.
  digitalWrite(STEP, HIGH);
  delayMicroseconds(3);
  digitalWrite(STEP, LOW);
  delayMicroseconds(3);

  // If we press the button again display the microstep
  if(!buttonA.isPressed())
  {
    Serial.print("Micro-step position = ");
    Serial.println((SPItransmit(READ, SR0, 0) & B00000100)>>2);
  }
}

void SPIwriteParam(int param, byte newData)
{
  //Before writing a single param we must read what is already at
  //that address so we can preserve it.
  byte prevData;

  switch(param)
  {
    case DIRCTRL:
    prevData = SPItransmit(READ, CR1, 0) & B01111111;
    SPItransmit(WRITE, CR1, prevData | newData<<7);
    break;
    case NXTP:
    prevData = SPItransmit(READ, CR1, 0) & B10111111;
    SPItransmit(WRITE, CR1, prevData | newData<<6);
    break;
    case EMC:
    prevData = SPItransmit(READ, CR1, 0) & B11111100;
    SPItransmit(WRITE, CR1, prevData | newData<<0);
    break;
    case SLAT:
    prevData = SPItransmit(READ, CR2, 0) & B11101111;
    SPItransmit(WRITE, CR2, prevData | newData<<4);
    break;
    case SLAG:
    prevData = SPItransmit(READ, CR2, 0) & B11011111;
    SPItransmit(WRITE, CR2, prevData | newData<<5);
    break;
    case PWMF:
    prevData = SPItransmit(READ, CR1, 0) & B11110111;
    SPItransmit(WRITE, CR1, prevData | newData<<3);
    break;
    case PWMJ:
    prevData = SPItransmit(READ, CR1, 0) & B11111011;
    SPItransmit(WRITE, CR1, prevData | newData<<2);
    break;
    case SM:
    prevData = SPItransmit(READ, CR0, 0) & B00011111;
    SPItransmit(WRITE, CR0, prevData | newData<<5);
    break;
    case ESM:
    prevData = SPItransmit(READ, CR3, 0) & B11111000;
    SPItransmit(WRITE, CR3, prevData | newData<<0);
    break;
    case SLP:
    prevData = SPItransmit(READ, CR2, 0) & B10111111;
    SPItransmit(WRITE, CR2, prevData | newData<<6);
    break;
    case MOTEN:
    prevData = SPItransmit(READ, CR2, 0) & B01111111;
    SPItransmit(WRITE, CR2, prevData | newData<<7);
    break;
    case CUR:
    prevData = SPItransmit(READ, CR0, 0) & B11100000;
    SPItransmit(WRITE, CR0, prevData | newData<<0);
    break;
    case WDEN:
    prevData = SPItransmit(READ, WR, 0) & B01111111;
    SPItransmit(WRITE, WR, prevData | newData<<7);
    break;
    case WDT:
    prevData = SPItransmit(READ, WR, 0) & B10000111;
    SPItransmit(WRITE, WR, prevData | newData<<3);
    break;
  }
}

byte SPItransmit(int cmd, int adr, byte dataIn)
{
    digitalWrite(amisSlaveSelect, LOW);  //take the ss pin low to select the chip
    SPI.transfer(cmd<<5 | adr);  //send command and address byte
    byte dataOut = SPI.transfer(dataIn);  //send data byte for write and read data dyte for read
    digitalWrite(amisSlaveSelect, HIGH); //take the ss pin high to de-select the chip
    return dataOut;
}


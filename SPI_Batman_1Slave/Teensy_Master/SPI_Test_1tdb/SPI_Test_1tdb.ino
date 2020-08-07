// inslude the SPI library:
#include <SPI.h>
#define SS 10

uint16_t WakeUp[2] = {0x2ad4, 0x0000};
uint16_t Com[2] = {0x21f2, 0x4d00};
uint16_t send5 = 0x0008;
uint16_t sendX[2] = {0x0000, 0x0000}; //place holder array for random messages
uint16_t req47[2] = {0x4700, 0x7000};
uint16_t req48[2] = {0x4800, 0x3400} ;
uint16_t req49[2] = {0x4900, 0xdd00};
uint16_t req4a[2] = {0x4a00, 0xc900};
uint16_t req4b[2] = {0x4b00, 0x2000};
uint16_t req4c[2] = {0x4c00, 0xe100};
uint16_t req4d[2] = {0x0100, 0x2700};
uint16_t req4e[2] = {0x0100, 0x3300};
uint16_t req4f[2] = {0x0100, 0xda00};
uint16_t req50[2] = {0x0000, 0x9400};
uint16_t padding = 0x0000;
uint16_t Request_A = 0x0000;
uint16_t Request_B = 0x0000;
uint16_t receive1 = 0;
uint16_t receive2 = 0;
uint8_t Fluffer[72];
byte count1 = 0;
byte count2 = 0;

unsigned long LoopTimer1 = 0;

uint16_t Voltage[8][15] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

uint16_t Temps [8][2] = {{0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0}
};

bool debug = 0;


void setup() {
  // put your setup code here, to run once:
  SPI.begin();
  pinMode(SS, OUTPUT); //select pin
  SerialUSB.begin(115200);//normal port

}

void loop()
{

  if (Serial.available() > 0)
  {
    byte incomingByte = Serial.read(); // read the incoming byte:

    if (incomingByte == 'd')
    {
      debug = !debug;
    }
  }

  if (millis() - LoopTimer1 > 500)
  {
    LoopTimer1 = millis();

    WakeUP();
    Generic_Send_Once(Com);

    sendX[0] = 0x0800;
    sendX[1] = 0x0000;
    Generic_Send_Once(sendX);

    Generic_Send_Once(Com);

    Generic_Send_Once(sendX);
    WakeUP();
    SerialUSB.println("0x47 Request:");
    GetData(req47, 0x47);

    WakeUP();
    SerialUSB.println("0x48 Request:");
    GetData(req48, 0x48);

    WakeUP();
    SerialUSB.println("0x49 Request:");
    GetData(req49, 0x49);

    WakeUP();
    SerialUSB.println("0x4A Request:");
    GetData(req4a, 0x4A);

    WakeUP();
    SerialUSB.println("0x4B Request:");
    GetData(req4b, 0x4B);

    WakeUP();
    SerialUSB.println("0x4C Request:");
    GetData(req4c, 0x4C);

    WakeUP();
    SerialUSB.println("0x4D Request:");
    GetData(req4d, 0x4D);

    WakeUP();
    SerialUSB.println("0x4E Request:");
    GetData(req4e, 0x4E);

    WakeUP();
    SerialUSB.println("0x4F Request:");
    GetData(req4f, 0x4F);

    WakeUP();
    SerialUSB.println("0x50 Request:");
    GetData(req50, 0x50);
    SerialUSB.println();
    for (int h = 0; h <= 1; h++)
    {
      SerialUSB.print("IC ");
      SerialUSB.print(h + 1);
      SerialUSB.print(" : ");
      for (int g = 0; g < 14; g++)
      {
        //SerialUSB.print("Cell");
        //SerialUSB.print(g + 1);
        SerialUSB.print("| ");
        SerialUSB.print(Voltage[h][g]);
        SerialUSB.print("mV");
      }
      SerialUSB.println();
    }
    uint16_t tempvol = 0x9F6E / 12.5;
    SerialUSB.println(tempvol);
    tempvol = 0x9F6E * 0.08;
    SerialUSB.println(tempvol);
    SerialUSB.println();
  }
}

void GetData(uint16_t Request[2], uint8_t ReqID)
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));//1mhz clock,msb first, mode 3
  digitalWrite (SS, LOW);        // assert Slave Select
  receive1 = SPI.transfer16(Request[0]);  // do a transfer
  receive2 = SPI.transfer16(Request[1]);  // do a transfer

  for (count2 = 0; count2 <= 72; count2 = count2 + 2)
  {
    receive1 = SPI.transfer16(padding);  // do a transfer
    if (debug == 1)
    {
      if (receive1 != 0xffff) SerialUSB.println(receive1, HEX);
    }
    //SerialUSB.println(receive1,HEX);
    Fluffer[count2] = highByte(receive1);
    Fluffer[count2 + 1] = lowByte(receive1);
  }
  digitalWrite (SS, HIGH);       // de-assert Slave Select
  SPI.endTransaction ();         // transaction over

  uint16_t tempvol = 0;

  if (Fluffer != 0xffff)
  {
    switch (ReqID)
    {
      case 0x47:
        for (int h = 0; h <= 1; h++)
        {
          for (int g = 0; g <= 2; g++)
          {
            tempvol = Fluffer[1 + (h * 9) + (g * 2)] * 256 + Fluffer [0 + (h * 9) + (g * 2)];
            if (debug == 1)
            {
              SerialUSB.println(Fluffer [0 + (h * 9) + (g * 2)], HEX);
              SerialUSB.println(Fluffer[1 + (h * 9) + (g * 2)], HEX);
            }
            if (tempvol != 0xffff)
            {
              Voltage[h][g] = tempvol / 12.5;
            }
          }
        }
        break;

      case 0x48:
        for (int h = 0; h <= 1; h++)
        {
          for (int g = 3; g <= 5; g++)
          {
            tempvol = Fluffer[1 + (h * 9) + (g * 2)] * 256 + Fluffer [0 + (h * 9) + (g * 2)];
            if (debug == 1)
            {
              SerialUSB.println(Fluffer [0 + (h * 9) + (g * 2)], HEX);
              SerialUSB.println(Fluffer[1 + (h * 9) + (g * 2)], HEX);
            }
            if (tempvol != 0xffff)
            {
              Voltage[h][g] = tempvol / 12.5;
            }
          }
        }
        break;

      case 0x49:
        for (int h = 0; h <= 1; h++)
        {
          for (int g = 6; g <= 8; g++)
          {
            tempvol = Fluffer[1 + (h * 9) + (g * 2)] * 256 + Fluffer [0 + (h * 9) + (g * 2)];
            if (debug == 1)
            {
              SerialUSB.println(Fluffer [0 + (h * 9) + (g * 2)], HEX);
              SerialUSB.println(Fluffer[1 + (h * 9) + (g * 2)], HEX);
            }
            if (tempvol != 0xffff)
            {
              Voltage[h][g] = tempvol / 12.5 ;
            }
          }
        }
        break;


      case 0x4A:
        for (int h = 0; h <= 1; h++)
        {
          for (int g = 9; g <= 11; g++)
          {
            tempvol = Fluffer[1 + (h * 9) + (g * 2)] * 256 + Fluffer [0 + (h * 9) + (g * 2)];
            if (debug == 1)
            {
              SerialUSB.println(Fluffer [0 + (h * 9) + (g * 2)], HEX);
              SerialUSB.println(Fluffer[1 + (h * 9) + (g * 2)], HEX);
            }
            if (tempvol != 0xffff)
            {
              Voltage[h][g] = tempvol / 12.5;
            }
          }
        }
        break;

      case 0x4B:
        for (int h = 0; h <= 1; h++)
        {
          for (int g = 12; g <= 14; g++)
          {
            tempvol = Fluffer[1 + (h * 9) + (g * 2)] * 256 + Fluffer [0 + (h * 9) + (g * 2)];
            if (debug == 1)
            {
              SerialUSB.println(Fluffer [0 + (h * 9) + (g * 2)], HEX);
              SerialUSB.println(Fluffer[1 + (h * 9) + (g * 2)], HEX);
            }
            if (tempvol != 0xffff)
            {
              Voltage[h][g] = tempvol / 12.5;
            }
          }
        }
        break;

      default:
        // statements
        break;
    }

  }

  delay(75);
}

void WakeUP()
{
  for (count1 = 0; count1 <= 4; count1++)
  {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));//1mhz clock,msb first, mode 3
    digitalWrite (SS, LOW);        // assert Slave Select
    receive1 = SPI.transfer16(WakeUp[0]);  // do a transfer
    receive2 = SPI.transfer16(WakeUp[1]);  // do a transfer
    digitalWrite (SS, HIGH);       // de-assert Slave Select
    SPI.endTransaction ();         // transaction over
    delayMicroseconds(20);
  }
}

void Generic_Send_Once(uint16_t Command[2])
{

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));//1mhz clock,msb first, mode 3
  digitalWrite (SS, LOW);        // assert Slave Select
  receive1 = SPI.transfer16(Command[0]);  // do a transfer
  receive2 = SPI.transfer16(Command[1]);  // do a transfer
  digitalWrite (SS, HIGH);       // de-assert Slave Select
  SPI.endTransaction ();         // transaction over
  delayMicroseconds(20);
}

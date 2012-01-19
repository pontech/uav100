#include "pic32lib/core.h"
#include "TokenParser/TokenParser.h"
#include "GPS/GPS.h"
#include <EEPROM.h>
#include <HardwareSerial.h>
#include <SPI.h>
#define WantNewLine // todo: 2 comment out before finalized
#define s8 signed char
#define s16 signed short int
#define s32 signed long
#define us8 unsigned char
#define us16 unsigned short int
#define us32 long unsigned
#define CORETIMER_TICKS_PER_MICROSECOND		(F_CPU / 2 / 1000000UL)
typedef struct packed_int2 {
  us8 l;
  us8 u;
} hilow;

typedef union {
  hilow p;
  us16 val;
} hilow16;

typedef struct {
  us32 bitrate;
  us8 mapping[16];
  us16 servoPos[16];
  packed spe;
  us16 lowerlimit;
  us16 upperlimit;
  us8 board;
  us8 pivotstate;
  us8 structend;
} ram_struct;
// Uses one or two timers, output compare and interrupts
// to achieve better than 14-bit resolution with 8 servos
// (or better than 16-bit if EXTRA_RES is #defined
// ...though even the lower figure is already pretty
// absurd and more than typical servos can resolve).


// These can be changed
// to most anything EXCEPT pin 3, and they do not need
// to be consecutive pins nor in-order.
#ifdef EXTRA_RES
  #define SERVO_MIN  (F_CPU / 1000)                // 1.0 mS
  typedef int servo_t;
#else
  #define SERVO_MIN  (F_CPU / 4 / 1000)            // 1.0 mS
  typedef unsigned short servo_t;
#endif
#define SERVO_MAX    (SERVO_MIN * 2)               // 2.0 mS
#define SERVO_CENTER ((SERVO_MIN + SERVO_MAX) / 2) // 1.5 mS

us8 servoPin[16] = { 64, 65, 66, 67, 68, 69, 70, 71, 16, 17, 18, 19, 20, 21, 31, 54}; //servo in and out pins
us8 servoOnOff[16] = { 1, 1, 1, 1, 1, 1, 1, 1}; //sets the servo outs to be off or on
us32 risingtime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; //track risetimes
us32 intime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; //time value was high
us32 lastused[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; //track last time a value was captured
ram_struct ram;
GPS_INFO gps_data;
servo_t servoPos[16];//array that holds servo positions output and input
us8 buff[0x20];
us8 ctr;
us8 ch;
us8 buff1[0x7f];
us8 ctr1;
us8 ch1;
//volatile int state = LOW;
us8 servo = 0;
us8 mapping[16] ;//= {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
bool active = true;
us8 i; //index for loops
us8 pivot_last = 3;
us8 pivot_this;
hilow16 accel_x;
hilow16 accel_y;
hilow16 accel_z;
hilow16 temp;
hilow16 gyro_x;
hilow16 gyro_y;
hilow16 gyro_z;
us8 gyro_cs = 26;
us8 GyroScroll = 0;
us8 gpsraw = 0;
us32 gnow;

//HardwareSerial& MySerial=Serial0;
USBSerial& MySerial=Serial;

void setup()
{
  TurnOffSecondaryOscillator();
  eeprom_out(0,(us32*)&ram,sizeof(ram)); //get structure from memory
  if (ram.structend != 42)
    set_default_ram();
  // Enable output pins for servos
  for(i = 0; i < 8; i++) {
    pinMode(servoPin[i], OUTPUT);
    digitalWrite(servoPin[i], LOW);
  }
  us16 mask = 1;
  for(i=0;i<8;i++) {
    servoOnOff[i] = (ram.spe.i & mask) > 0 ? 1 : 0;
    mask = mask<<1;
  }
  memcpy(servoPos,ram.servoPos,sizeof(servoPos));
  memcpy(mapping,ram.mapping,sizeof(mapping));
  for(i=0;i<7;i++){ //todo: 2 fix memory
    ram.mapping[i]=i+8;
  }
  MySerial.begin(ram.bitrate);
  Serial1.begin(9600);

  pinMode(102, OUTPUT);
//  attachInterrupt(1, RisingInterrupt, RISING);

  // Initialize timer to 400 Hz (50 Hz * 8 servos)
#ifdef EXTRA_RES
  // Extra positional accuracy requires joining Timers 2 and 3.
  // Interrupt function is attached to Timer 3 -- this issues
  // the 'up' tick for each servo.
  ConfigIntTimer23(T23_INT_ON | T23_INT_PRIOR_3);
  OpenTimer23(T23_ON | T23_PS_1_1 | T23_32BIT_MODE_ON, F_CPU / 50 / 8);
  // Output Compare 1 w/interrupt handles the 'down' tick.
  // OC1 is the reason pin 3 can't be used; its output state
  // will be the combined PWM for all servos together.
  ConfigIntOC1(OC_INT_ON | OC_INT_PRIOR_3);
  OpenOC1(OC_ON | OC_IDLE_CON | OC_TIMER_MODE32 | OC_CONTINUE_PULSE, 0, 0);
#else
  // Standard resolution uses Timer 3 with 1:4 prescaler.
  // Same interrupts and output compare situation.
  ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_3);
  OpenTimer3(T3_ON | T3_PS_1_4, F_CPU / 4 / 50 / 8);
  ConfigIntOC1(OC_INT_ON | OC_INT_PRIOR_3);
  OpenOC1(OC_ON | OC_IDLE_CON | OC_TIMER3_SRC | OC_CONTINUE_PULSE, 0, 0);
#endif
  us16 value; //code for Change Notice interrupts
  TRISBSET = 0x803f;
  TRISDSET = 0x20;
  IEC0CLR = 0x10000000;   //turn intrupts off
  AD1PCFGSET = 0x0000803c;     //turn analog 2-5,15 off 0x0000403c
  AD1CON2CLR = 0x0000d000; //turn off external voltage compare pins?
  CNCON = 0x00008000;   //turn "interrupt on change" on
  CNEN = 0x000090fc;   //enable cn2-7,12,15
  CNPUE= 0x00000000; //weak pull up off
  value = PORTB;      //Read the Ports
  value = PORTD;
  IPC6SET = 0x00060000;//0x00060000; //0x001f0000;//set priority to 7 sub 4
  IFS1CLR = 0x0001; //clear the interupt flag bit
  IEC1SET= 0x0001; // Enable Change Notice interrupts
  IEC0SET = 0x10000000;   //turn intrupts on
  pinMode(17, INPUT);
  pinMode(16, INPUT);
  pinMode(29, INPUT);
  pinMode(54, INPUT);
  pinMode(80, OUTPUT);
  pinMode(102, OUTPUT);
  digitalWrite(102,LOW);

  TRISBCLR = 0x3000; //code for relay driver
  TRISCCLR = 0x2000;
  TRISDCLR = 0x01;
  PORTBCLR = 0x3000;
  PORTCCLR = 0x2000;
  PORTDCLR = 0x01;
  pinMode(48, OUTPUT);
  
  //gyro setup
    pinMode(25,OUTPUT);
  digitalWrite(25,HIGH);
  SPI.begin();
  SPI.setDataMode(0x40);
  pinMode(gyro_cs, OUTPUT);
  digitalWrite(gyro_cs,HIGH);

  SPI.setClockDivider(39);//0=40mhz 1=20 2=13.3
  while(millis()<100) {
    delay(10);
  }
  digitalWrite(gyro_cs,LOW);
  SPI.transfer(0x6a);//USER_CTRL
  SPI.transfer(0x50);
  digitalWrite(gyro_cs,HIGH);
  digitalWrite(gyro_cs,LOW);
  SPI.transfer(0x6b);//Power Managment 1
  SPI.transfer(0x01);//clock x gyro,temp enabled,cycle off,sleep off,reset off
  digitalWrite(gyro_cs,HIGH);

}

e16 num1;//temporary values to parse into
e16 num2;
e32 num3;

void loop()
{
  for(i=0;i<8;i++){
    //zero the values if no input for 100 ms
    us32 nowa=ReadCoreTimer();//micros();
    us32 last;
    last=(nowa>lastused[i]) ? nowa-lastused[i] : (0xffffffff - lastused[i]) + nowa;
    //last = nowa-lastused[i];
//    delay(100);
//    MySerial.print(last,DEC);
//    PrintCR();
    if (last>40000000)//40=1us
      servoPos[i+8]=0;
    //Copy input to output if mapped
/*    if (mapping[i]>7)
    {
      //IEC0CLR = 0x10000000;   //turn intrupts off
      servoPos[i] = servoPos[mapping[i]] > ram.upperlimit ? ram.upperlimit : servoPos[mapping[i]] < ram.lowerlimit ? ram.lowerlimit : servoPos[mapping[i]];
      //servoPos[i]=servoPos[mapping[i]]; // todo: 3 fix softlimits
      //MySerial.print(servoPos[mapping[i]],DEC);
      //PrintCR();
      //IEC0SET = 0x10000000;   //turn intrupts on
    }
*/
  }
  IEC0CLR = 0x10000000;   //turn intrupts off
  us32 servoPos15=servoPos[15];
  IEC0SET = 0x10000000;   //turn intrupts on
  
  if (ram.pivotstate == 0)
    pivot_this = digitalRead(servoPin[15]);
  else
    //pivot_this = servoPos15<25000 ? 1 :  0;//servoPos15>32000 ? 1 : servoPos15<28000 ? 0 : 2;
    pivot_this = (servoPos15>32000 || servoPos15==0) ? 1 : servoPos15<28000 ? 0 : 2;//servoPos15>32000 ? 1 : servoPos15<28000 ? 0 : 2;
  if (pivot_this != pivot_last ) {
    pivot_last = pivot_this;
    if (pivot_this == 1) {
      SSD();
      digitalWrite(80,HIGH);
//      MySerial.println("changing to SSD");
//      MySerial.println(servoPos[15]);
      
    }
    else if(pivot_this == 0) {
      SRS();
      digitalWrite(80,LOW);
//      MySerial.println("changing to SRS");
//      MySerial.println(servoPos[15]);
    }
  }
  if (GyroScroll != 0){
    GyroPrint();
    delay(200);
  }
//MySerial.println(servoPos15);
  if (MySerial.available() > 0)
  {
    ch = MySerial.read();
    if( ctr < sizeof(buff)) {
      buff[ctr++] = ch;
    }
 //MySerial.println(intime[0]); // todo: 2 buffer this value
 
    if (ch == '\r')
    {
      buff[ctr-1] = ' ';
      TokenParser tokpars(buff,ctr);
      tokpars.nextToken();
      ctr = 0;
      //ch = buff[0];
      if( tokpars.compare("BD?" ) ) {
        if(!tokpars.contains("?") && !tokpars.compare("BD "))
        {
          tokpars.advanceTail(2);
          num1 = tokpars.to_e16();
          if (num1.value==ram.board || num1.value==0)
          {
            active=true;
//            MySerial.print("OK");
//            PrintCR();
          }
          else
          {
            active=false;
          }
        }
      }
      if(active)
        {
        if( tokpars.compare("BD?",'|' ) ) {
//          MySerial.print("Board ");
          MySerial.print(ram.board,DEC);
          PrintCR();
        }
        else if( tokpars.compare("ID?" ) ) {
          tokpars.advanceTail(2);
          if(tokpars.contains("?"))
          {
//            MySerial.print("Board ");
            MySerial.print(ram.board,DEC);
            PrintCR();            
          }
          else
          {
            num1 = tokpars.to_e16();
            if (num1.value > 0 && num1.value < 256)
            {
              ram.board = num1.value;
//              MySerial.print("Board ");
//              MySerial.print(ram.board,DEC);
              active=false;
            }
            else
            {
              MySerial.print("NOK");
            }
            PrintCR();
          }
        }
        else if( tokpars.compare("BR?" ) ) {
          tokpars.advanceTail(2);
          num3 = tokpars.to_e32();
          ram.bitrate = num3.value;
//          MySerial.print("Bit Rate ");
//          MySerial.print(ram.bitrate);
//          PrintCR();
          MySerial.end();
          MySerial.begin(ram.bitrate);
        }
        else if( tokpars.compare("WE?" ) ) {
          tokpars.advanceTail(2);
          num1 = tokpars.to_e16();
          tokpars.nextToken();
          num3 = tokpars.to_e32();
          EEPROM.write(num1.value, num3.value);
//          MySerial.print("Write ");
//          MySerial.print(num2.value);
//          MySerial.print(" to ");
//          MySerial.print(num1.value);
//          PrintCR();
        }
        else if( tokpars.compare("RE?" ) ) {
          tokpars.advanceTail(2);
          num3 = tokpars.to_e32();
          MySerial.print(EEPROM.read(num3.value),DEC);
//          MySerial.print(num3.value,DEC);
          PrintCR();
        }
        else if( tokpars.compare("WSS") ) {
          eeprom_in((us32*)&ram,0,sizeof(ram));
//          MySerial.print("Write System Settings");
//          PrintCR();
        }
        else if( tokpars.compare("RSS") ) {
          eeprom_out(0,(us32*)&ram,sizeof(ram));
          MySerial.end();
          MySerial.begin(ram.bitrate);
          active=false;
          us16 mask = 1;
          for(i=0;i<8;i++) {
            servoOnOff[i] = (ram.spe.i & mask) > 0 ? 1 : 0;
            mask = mask<<1;
          }
          memcpy(servoPos,ram.servoPos,sizeof(servoPos));
          memcpy(mapping,ram.mapping,sizeof(mapping));
//          MySerial.print("Read System Settings");
//          PrintCR();
        }
        else if( tokpars.compare("DSS") ) {
          set_default_ram();
          MySerial.end();
          MySerial.begin(ram.bitrate);
          active=false;
          us16 mask = 1;
          for(i=0;i<8;i++) {
            servoOnOff[i] = (ram.spe.i & mask) > 0 ? 1 : 0;
            mask = mask<<1;
          }
          memcpy(servoPos,ram.servoPos,sizeof(servoPos));
          memcpy(mapping,ram.mapping,sizeof(mapping));
//          MySerial.print("Default System Settings");
//          PrintCR();
        }
        else if( tokpars.compare("V?",'|') ) {
          MySerial.print("UAV100 Version 0.3 2012.01.03.12.30");
/*
#if defined (_BOARD_MEGA_)
          MySerial.print(" pins_arduino_pic32_mega.cxx");
#elif defined (_BOARD_UNO_)
	          MySerial.print(" pins_arduino_pic32_uno.cxx");
#else
	          MySerial.print(" pins_arduino_pic32_default.cxx");
#endif
*/
          PrintCR();
        }
        else if( tokpars.compare("?",'|') ) {
          MySerial.print("Visit pontech.com for assistance.");
          PrintCR();
        }
        else if( tokpars.compare("MS?") ) {
          tokpars.advanceTail(2);
          if( tokpars.compare("?",'|') )
          {
            MySerial.print("Read Mode Removed");
            PrintCR();
          }
          else
          {
            MySerial.print("Write Mode Removed");
            PrintCR();
          }
        }
        else if( tokpars.compare("SV?" ) ) {
          tokpars.advanceTail(2);
          num1 = tokpars.to_e16();
          servo = (us8)num1.value;
//          MySerial.print("servo ");
//          MySerial.print(servo,DEC);
//          PrintCR();
        }
        else if( tokpars.compare("M?") ) {
          tokpars.advanceTail(1);
          if( tokpars.compare("?|",'|') )
          {
            tokpars.advanceTail(1);
            num1 = tokpars.to_e16();
//            MySerial.print("Servo ");
//            MySerial.print(num1.value,DEC);
//            MySerial.print(" in pos ");
            MySerial.print(servoPos[mapping[num1.value]]/2,DEC);
            PrintCR();
          }
          else
          {
            num1 = tokpars.to_e16();
            if(servo<8) // todo: 
              servoPos[servo] = num1.value*2 > ram.upperlimit ? ram.upperlimit : num1.value*2 < ram.lowerlimit ? ram.lowerlimit : num1.value*2;
//            MySerial.print("Move Servo ");
//            MySerial.print(mapping[servo],DEC);
//            PrintCR();
          }
        }
        else if( tokpars.compare("I?" ) ) {
          tokpars.advanceTail(1);
          num1 = tokpars.to_e16();
          servoPos[servo] = num1.value*2 + servoPos[servo] > ram.upperlimit ? ram.upperlimit : num1.value*2 + servoPos[servo] < ram.lowerlimit ? ram.lowerlimit : num1.value*2+servoPos[servo];
//          MySerial.print("Relative ");
//          MySerial.print(num1.value);
//          PrintCR();
        }
        else if( tokpars.compare("SS?" ) ) {
          tokpars.advanceTail(2);
          if( tokpars.compare("D" ) ) {
            SSD();
//            MySerial.print("Default Map");
//            PrintCR();
          }
          else if( tokpars.compare("R" ) ) {
            memcpy(ram.mapping,mapping,sizeof(mapping));
//            MySerial.print("Copy Map to Ram");
//            PrintCR();
          }
          else if( tokpars.compare("M" ) ) {
            for(i=0;i<16;i++) {
              MySerial.print(mapping[i],DEC);
              MySerial.print(" ");
            }
            PrintCR();
          }
          else
          {
            num1 = tokpars.to_e16();
            tokpars.nextToken();
            num2 = tokpars.to_e16();
            mapping[num2.value]=num1.value;
//            MySerial.print("Map ");
//            MySerial.print(num2.value);
//            MySerial.print(" to ");
//            MySerial.print(num1.value);
//            PrintCR();
          }
        }
        else if( tokpars.compare("SRS") ) {
          SRS();
//          MySerial.print("Copy Ram to Map");
//          PrintCR();
        }
        else if( tokpars.compare("CSR") ) {
          memcpy(ram.servoPos,servoPos,sizeof(servoPos));
//          MySerial.print("Copy Servo to Ram");
//          PrintCR();
        }
        else if( tokpars.compare("CRS") ) {
          memcpy(servoPos,ram.servoPos,sizeof(servoPos));
//          MySerial.print("Copy Ram to Servo");
//          PrintCR();
        }
        else if( tokpars.compare("SPE?") ) {
          tokpars.advanceTail(3);
          if( tokpars.compare("?",'|') ) {
            MySerial.print("0x");
            MySerial.print(ram.spe.i,HEX); 
            PrintCR();
          }
          else
          {
            num3 = tokpars.to_e32();
            ram.spe.i = num3.value;
            us16 mask = 1;
            for(i=0;i<8;i++) {
              servoOnOff[i] = (ram.spe.i & mask) > 0 ? 1 : 0;
              mask = mask<<1;
            }
//            MySerial.print("Servo PPM Enable ");
//            MySerial.print(ram.spe,DEC);
//            PrintCR();
          }
        }
        else if( tokpars.compare("PS?" ) ) {
          tokpars.advanceTail(2);
          num1 = tokpars.to_e16();
          if(num1.value >=0 && num1.value <8)
            digitalWrite(servoPin[num1.value], HIGH);
//          MySerial.print("Set Pin ");
//          MySerial.print(num1.value);
//          PrintCR();
        }
        else if( tokpars.compare("PC?" ) ) {
          tokpars.advanceTail(2);
          num1 = tokpars.to_e16();
          if(num1.value >=0 && num1.value <8)
            digitalWrite(servoPin[num1.value], LOW);
//          MySerial.print("Clear Pin ");
//          MySerial.print(num1.value);
//          PrintCR();
        }
        else if( tokpars.compare("PT?" ) ) {
          tokpars.advanceTail(2);
          num1 = tokpars.to_e16();
          if(num1.value >=0 && num1.value <8)
            digitalWrite(servoPin[num1.value], !digitalRead(servoPin[num1.value]));
//          MySerial.print("Toggle Pin ");
//          MySerial.print(num1.value);
//          PrintCR();
        }
        else if( tokpars.compare("RP?|",'|' ) ) {
          tokpars.advanceTail(3);
          num1 = tokpars.to_e16();
          if(num1.value >7 && num1.value <16) {
            MySerial.print(!digitalRead(servoPin[num1.value]),DEC);
//            MySerial.print("Read Pin ");
//            MySerial.print(num1.value);
            PrintCR();
          }
        }
        else if( tokpars.compare("AD?|",'|' ) ) {
          tokpars.advanceTail(3);
          num1 = tokpars.to_e16();
          
 //         MySerial.print("Analog ");
          MySerial.print(analogRead(A11));
          PrintCR();
        }
        else if( tokpars.compare("SLU?") ) {
          tokpars.advanceTail(3);
          num1 = tokpars.to_e16();
          ram.upperlimit = num1.value*2;
        }
        else if( tokpars.compare("SLL?") ) {
          tokpars.advanceTail(3);
          num1 = tokpars.to_e16();
          ram.lowerlimit = num1.value*2;
        }
        else if( tokpars.compare("RLY?") ) {
          tokpars.advanceTail(3);
          num1 = tokpars.to_e16();
          tokpars.nextToken();
          num2 = tokpars.to_e16();
//          MySerial.print(num1.value,DEC);
//          PrintCR();
          if(num1.value == 0)
          {
            if (num2.value == 0)
              PORTBCLR = 0x1000;
                else
              PORTBSET = 0x1000;
          }
          else if(num1.value == 1)
          {
            if (num2.value == 0)
              PORTBCLR = 0x2000;
                else
              PORTBSET = 0x2000;
          }
          else if(num1.value == 2)
          {
            if (num2.value == 0)
              PORTCCLR = 0x2000;
                else
              PORTCSET = 0x2000;
          }
          else if(num1.value == 3)
          {
            if (num2.value == 0)
              PORTDCLR = 0x01;
                else
              PORTDSET = 0x01;
          }
        }
        else if( tokpars.compare("PIVOT?") ) {
          tokpars.advanceTail(5);
          num1 = tokpars.to_e16();
//          MySerial.print(num1.value,DEC);
          ram.pivotstate = num1.value;
          if (ram.pivotstate == 0) {
            digitalWrite(102,HIGH);
            ram.spe.i &= ~0x8000;
          }
          else
          {
            digitalWrite(102,LOW);
            ram.spe.i |= 0x8000;
          }
        }
        else if( tokpars.compare("GYROSCROLL") ) {
          GyroScroll=!GyroScroll;
        }
        else if( tokpars.compare("GYROSINGLE") ) {
          GyroPrint();
        }
        else if( tokpars.compare("GPSRAW") ) {
          gpsraw=(gpsraw == 0 ? 1 : 0);
          MySerial.print(gpsraw,DEC);
          PrintCR();
        }
        else if( tokpars.compare("A") ) {
          us32 nowa=micros();
          us32 last;
          last=(nowa>lastused[7]) ? nowa-lastused[7] : (0xffffffff - lastused[7]) + nowa;

          MySerial.print(nowa,DEC);
          MySerial.print(", ");
          MySerial.print(servoPos15,DEC);
          MySerial.print(", ");
          MySerial.print(last,DEC);
          
          PrintCR();
        }
/*        else if( tokpars.compare("size") ) {
          MySerial.print(sizeof(ram.board),DEC);
          PrintCR();
          MySerial.print(sizeof(ram.bitrate),DEC);
          PrintCR();
          MySerial.print(sizeof(ram.mapping),DEC);
          PrintCR();
          MySerial.print(sizeof(ram.servoPos),DEC);
          PrintCR();
          MySerial.print(sizeof(packed_int),DEC);
          PrintCR();
          MySerial.print(sizeof(ram.lowerlimit),DEC);
          PrintCR();
          MySerial.print(sizeof(ram.upperlimit),DEC);
          PrintCR();
          MySerial.print(sizeof(ram.structend),DEC);
          PrintCR();
          MySerial.print(sizeof(ram_struct),DEC);
          PrintCR();
        }
*/  
  
        if( tokpars.compare("TIMES") ) {
          for(i=0;i<16;i++) {
          MySerial.print(servoPos[i],DEC);
          PrintCR();
          }
        }
      }
    }
  }
  /* //GPS
  if(gpsraw!=0)
  {
    if( Serial1.available() > 0) {
      ch1 = Serial1.read();
      MySerial.print(ch1);
    }
  }
  else
  {
    if( Serial1.available() > 0) {
      ch1 = Serial1.read();
      if( ctr1 < sizeof(buff1)) {
        buff1[ctr1++] = ch1;
      }
   
      if (ch1 == '\n')
      {
        buff1[ctr1-2] = ' ';
        TokenParser tokpars1(buff1,ctr1);
        tokpars1.replace(',',' ');
        tokpars1.nextToken();
        ctr1 = 0;
        //ch = buff[0];
        if( tokpars1.compare("$GPGGA" ) ) {
          tokpars1.nextToken();
          num3=tokpars1.to_e32();
          gps_data.time.hour=num3.value/10000000;
          gps_data.time.minute=(num3.value%10000000)/100000;//(num3.value-gps_data.time.hour*10000000)/100000;
          gps_data.time.second=(num3.value%100000)/1000;//(num3.value-gps_data.time.hour*10000000 - gps_data.time.minute*100000)/1000;
          gps_data.time.millisecond=(num3.value%1000);//(num3.value-gps_data.time.hour*10000000 - gps_data.time.minute*100000 - gps_data.time.second*1000);
          MySerial.print(gps_data.time.hour,DEC);
          MySerial.print(":");
          MySerial.print(gps_data.time.minute,DEC);
          MySerial.print(":");
          MySerial.print(gps_data.time.second,DEC);
          MySerial.print(":");
          MySerial.print(gps_data.time.millisecond,DEC);
          MySerial.print(",");
          MySerial.print(num3.value,DEC);//time
          MySerial.print(" ");
          tokpars1.nextToken();
          num3=tokpars1.to_e32();
          MySerial.print(num3.value,DEC);//latitude
          MySerial.print(" ");
          tokpars1.nextToken();
          if( tokpars1.compare("N" ) ) {
            MySerial.print("N");          
          }
          else
          {
            MySerial.print("S");
          }
          tokpars1.advanceTail(1);

          MySerial.print(" ");
          tokpars1.nextToken();
          num3=tokpars1.to_e32();
          MySerial.print(num3.value,DEC);//longitude
          MySerial.print(" ");
          tokpars1.nextToken();
          if( tokpars1.compare("W" ) ) {
            MySerial.print("W");          
          }
          else
          {
            MySerial.print("E");
          }
          tokpars1.advanceTail(1);
          MySerial.print(" ");
          tokpars1.nextToken();
          num3=tokpars1.to_e32();
          MySerial.print(num3.value,DEC);//quality
          PrintCR();
        }
      }
    }
  }
  */
}
void PrintCR() {
  #ifdef WantNewLine
  MySerial.print("\r\n");
  #else
  MySerial.print("\r");
  #endif
}

void eeprom_in(us32* Data,us16 eeprom_adress,us16 bytes) {
  int i;
  for(i=0;i<bytes/4;i++){
    EEPROM.write(eeprom_adress++, *Data++);
  }
}
void eeprom_out(us16 eeprom_adress,us32* Data,us16 bytes) {
  int i;
  for(i=0;i<bytes/4;i++){
    *(Data++) = EEPROM.read(eeprom_adress++);
  }
}
void SRS() {
  memcpy(mapping,ram.mapping,sizeof(mapping));
}
void SSD() {
  for(i=0;i<16;i++) {
    mapping[i] = i;
  }
}
void set_default_ram() {
  ram.board = 1;
  ram.bitrate = 115200;
  ram.spe.i = 0xffff;
  ram.lowerlimit = 20000;
  ram.upperlimit = 40000;
  ram.pivotstate = 1;
  ram.structend = 42;
  int i;
  for(i=0;i<16;i++){
  ram.mapping[i] = i;
  if(i<8)
    ram.servoPos[i] = 30000;
  else
    ram.servoPos[i] = 0;
  }
}
void TurnOffSecondaryOscillator() {
  unsigned int dma_status;
  unsigned int int_status;
  mSYSTEMUnlock(int_status, dma_status);
  OSCCONCLR = _OSCCON_SOSCEN_MASK;
  mSYSTEMLock(int_status, dma_status);
}
void GyroPrint() {
  digitalWrite(gyro_cs,LOW);
  SPI.transfer(0x3b | 0x80);//accel_xout_h
  accel_x.p.u=SPI.transfer(0x00);
  accel_x.p.l=SPI.transfer(0x00);
  accel_y.p.u=SPI.transfer(0x00);
  accel_y.p.l=SPI.transfer(0x00);
  accel_z.p.u=SPI.transfer(0x00);
  accel_z.p.l=SPI.transfer(0x00);
  temp.p.u=SPI.transfer(0x00);
  temp.p.l=SPI.transfer(0x00);
  gyro_x.p.u=SPI.transfer(0x00);
  gyro_x.p.l=SPI.transfer(0x00);
  gyro_y.p.u=SPI.transfer(0x00);
  gyro_y.p.l=SPI.transfer(0x00);
  gyro_z.p.u=SPI.transfer(0x00);
  gyro_z.p.l=SPI.transfer(0x00);
  digitalWrite(gyro_cs,HIGH);
  MySerial.print((s16)accel_x.val,DEC);
  MySerial.print(", ");
  MySerial.print((s16)accel_y.val,DEC);
  MySerial.print(", ");
  MySerial.print((s16)accel_z.val,DEC);
  MySerial.print(", ");
  MySerial.print((float)((s16)temp.val+521)/340+35);
  MySerial.print(", ");
  MySerial.print((s16)gyro_x.val,DEC);
  MySerial.print(", ");
  MySerial.print((s16)gyro_y.val,DEC);
  MySerial.print(", ");
  MySerial.print((s16)gyro_z.val,DEC);
  PrintCR();

}
/*void RisingInterrupt()
{
 attachInterrupt(1, FallingInterrupt, FALLING);
 risingtime[0]=micros();
}
void FallingInterrupt()
{
 attachInterrupt(1, RisingInterrupt, RISING);
 intime[0]=micros()-risingtime[0];
}
*/
extern "C"
{

static volatile byte servoNum = 0;  // Cycles through servos

// This is the timer interrupt, invoked at a uniform 400 Hz.
// Generates the 'up' tick for each of 8 servos (unless
// position is set to 0) and sets the OC1 PWM duration
// for the subsequent 'down' tick.
// These two functions would obviously benefit from some
// direct PORT love (instead of digitalWrite())...just done
// this way for quick and easy implementation.
void __ISR(_TIMER_3_VECTOR,ipl3) pwmOn(void)
{
  mT3ClearIntFlag();  // Clear interrupt flag
  if(servoPos[mapping[servoNum]] > 0 && servoOnOff[servoNum] == 1 ) {
    digitalWrite(servoPin[servoNum], HIGH);
    SetDCOC1PWM(servoPos[mapping[servoNum]]);
  }
}

// This is the output compare interrupt, also invoked 400
// times per second, but the spacing is not uniform; this
// establishes the position of each servo.  'Down' tick
// occurs here, then servo number is advanced by 1.
void __ISR(_OUTPUT_COMPARE_1_VECTOR,ipl3) pwmOff(void)
{
  mOC1ClearIntFlag();
  if(servoOnOff[servoNum] == 1 ) 
    digitalWrite(servoPin[servoNum], LOW);
  if(++servoNum > 7) servoNum = 0;  // Back to start
}

void __ISR(_CHANGE_NOTICE_VECTOR, ipl1) CN_Interrupt_ISR(void)//__ISR(_CHANGE_NOTICE_VECTOR, ipl7) CN_Interrupt_ISR(void)
{
//  bool temp = digitalRead(43);
//  digitalWrite(43, temp ^ 1);
  
  us16 static lastb;
  us16 static lastd;
  us16 thisb = PORTB;
  us16 thisd = PORTD;
  us32 now;
  now = ReadCoreTimer();
  if ((thisb ^ lastb) & 0x01)  //cn2
  {
    lastused[0]=now;
    if (!(thisb & 0x01))
      risingtime[0]=now;
    else if(ram.spe.b.b8)
      servoPos[8]=((now>risingtime[0]) ? now-risingtime[0] : (0xffffffff - risingtime[0]) + now) >> 1;
  }
  if ((thisb ^ lastb) & 0x02) //cn3
  {
    lastused[1]=now;
    if (!(thisb & 0x02))
      risingtime[1]=now;
    else if(ram.spe.b.b9)
      servoPos[9]=((now>risingtime[1]) ? now-risingtime[1] : (0xffffffff - risingtime[1]) + now) >> 1;
  }
  if ((thisb ^ lastb) & 0x04) //cn4
  {
    lastused[2]=now;
    if (!(thisb & 0x04)) //cn4
      risingtime[2]=now;
    else if(ram.spe.b.b10)
      servoPos[10]=((now>risingtime[2]) ? now-risingtime[2] : (0xffffffff - risingtime[2]) + now) >> 1;
  }
  if ((thisb ^ lastb) & 0x08) //cn5
  {
    lastused[3]=now;
    if (!(thisb & 0x08))
      risingtime[3]=now;
    else if(ram.spe.b.b11)
      servoPos[11]=((now>risingtime[3]) ? now-risingtime[3] : (0xffffffff - risingtime[3]) + now) >> 1;
  }
  if ((thisb ^ lastb) & 0x10) //cn6
  {
    lastused[4]=now;
    if (!(thisb & 0x10))
      risingtime[4]=now;
    else  if(ram.spe.b.b12)
      servoPos[12]=((now>risingtime[4]) ? now-risingtime[4] : (0xffffffff - risingtime[4]) + now) >> 1;
  }
  if ((thisb ^ lastb) & 0x20) //cn7
  {
    lastused[5]=now;
    if (!(thisb & 0x20))
      risingtime[5]=now;
    else if(ram.spe.b.b13)
      servoPos[13]=((now>risingtime[5]) ? now-risingtime[5] : (0xffffffff - risingtime[5]) + now) >> 1;
  }
  if ((thisb ^ lastb) & 0x8000) //cn12
  {
    lastused[6]=now;
    if (!(thisb & 0x8000))
      risingtime[6]=now;
    else if(ram.spe.b.b14)
      servoPos[14]=((now>risingtime[6]) ? now-risingtime[6] : (0xffffffff - risingtime[6]) + now) >> 1;
  }
 if ((thisd ^ lastd) & 0x40) //cn15
  {
    lastused[7]=now;
    if (!(thisd & 0x40))
      risingtime[7]=now;
    else if(ram.spe.b.b15)
      //servoPos[15]= (now-risingtime[7]) >> 1;
      servoPos[15]=((now>risingtime[7]) ? (now-risingtime[7]) : (0xffffffff - risingtime[7]) + now) >> 1;
  }
  lastb = thisb; // Read PORTB to clear mismatch condition
  lastd = thisd; // Read PORTD to clear mismatch condition
  IFS1CLR = 0x0001; // Be sure to clear the CN interrupt status
                  // flag before exiting the service routine.
}

} // end extern "C"


//i2c eeprom example code
/*
#include <Wire.h>
#include "pic32lib/core.h"
#include "TokenParser/TokenParser.h"
us8 buff[0x20];
us8 ctr;
us8 ch;
e16 num1;//temporary values to parse into
e16 num2;


void setup()
{
  Serial.begin(115200);
  Wire.begin();
  
}

void loop ()
{
  if (Serial.available() > 0)
  {
    ch = Serial.read();
    if( ctr < 0x20) {
      buff[ctr++] = ch;
    }
 
    if (ch == '\r')
    {
      buff[ctr-1] = ' ';
      TokenParser tokpars(buff,ctr);
      tokpars.nextToken();
      ctr = 0;
      if( tokpars.compare("WR?" ) ) {
        tokpars.advanceTail(2);
        num1 = tokpars.to_e16();
        tokpars.nextToken();
        num2 = tokpars.to_e16();
        Serial.print(WriteByte(num1.value,num2.value),DEC);
//        Serial.print(num1.value,DEC);
//        Serial.print(", ");
//        Serial.print(num2.value,DEC);
        Serial.print("\r\n");      
      }
      else if( tokpars.compare("RD?" ) ) {
        tokpars.advanceTail(2);
        num1 = tokpars.to_e16();
        Serial.print(ReadByte(num1.value),DEC);
//        Serial.print(num1.value,DEC);
        Serial.print("\r\n");        
      }
    }
  }
}

us8 WriteByte(us8 adress,us8 data){
  Wire.beginTransmission(0x50);
  Wire.send(adress);
  Wire.send(data);
  return Wire.endTransmission();
}
us8 ReadByte(us8 adress){
  us8 data;
  Wire.beginTransmission(0x50);
  Wire.send(adress);
  Wire.endTransmission();
  Wire.requestFrom(0x50, 1);
  data = Wire.receive();
  return data;
}
*/


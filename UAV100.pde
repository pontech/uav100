#include <core.h>
#include <TokenParser.h>
#include <EEPROM.h>
#define WantNewLine // todo: 2 comment out before finalized
#define s8 signed char
#define s16 signed short int
#define s32 signed long
#define us8 unsigned char
#define us16 unsigned short int
#define us32 long unsigned
typedef struct {
  us32 bitrate;
  us8 mapping[16];
  us16 servoPos[16];
  packed spe;
  us16 lowerlimit;
  us16 upperlimit;
  us8 board;
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

us8 servoPin[8] = { 4, 5, 6, 7, 8, 9, 11, 12 }; //servo pins
us8 servoOnOff[16] = { 1, 1, 1, 1, 1, 1, 1, 1}; //sets the servo outs to be off or on
us32 risingtime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; //track risetimes
us32 intime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; //time value was high
us32 lastused[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; //track last time a value was captured
ram_struct ram;
servo_t servoPos[16];//array that holds servo positions output and input
us8 buff[0x20];
us8 ctr;
us8 ch;
//volatile int state = LOW;
us8 servo = 0;
us8 mapping[16] ;//= {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
bool active = false;
us8 i; //index for loops

void setup()
{
  eeprom_out(0,(us32*)&ram,sizeof(ram)); //get structure from memory
  if (ram.structend != 42)
    set_default_ram();
  // Enable output pins for servos
  for(int i = 0; i < 8; i++) {
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
  Serial.begin(ram.bitrate);

  //ram.board=1;// todo: 1 get from memory

  pinMode(13, OUTPUT);
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
  IPC6SET = 0x001f0000; //set priority to 7 sub 4
  IFS1CLR = 0x0001; //clear the interupt flag bit
  IEC1SET= 0x0001; // Enable Change Notice interrupts
  IEC0SET = 0x10000000;   //turn intrupts on
  pinMode(41, INPUT);
  pinMode(42, INPUT);
  pinMode(36, INPUT);
}

e16 num1;//temporary values to parse into
e16 num2;
e32 num3;

void loop()
{
  for(i=0;i<8;i++){
    //zero the values if no input for 100 ms
    us32 nowa=micros();
    us32 last;
    last=(nowa>lastused[i]) ? nowa-lastused[i] : (0xffffffff - lastused[i]) + nowa;
    if (last>100000)
      servoPos[i+8]=0;
    //Copy input to output if mapped
    if (mapping[i]>7)
    {
      //IEC0CLR = 0x10000000;   //turn intrupts off
      //servoPos[i] = servoPos[mapping[i]] > ram.upperlimit ? ram.upperlimit : servoPos[mapping[i]] < ram.lowerlimit ? ram.lowerlimit : servoPos[mapping[i]];
      servoPos[i]=servoPos[mapping[i]]; // todo: 3 fix softlimits
      //Serial.print(servoPos[mapping[i]],DEC);
      //PrintCR();
      //IEC0SET = 0x10000000;   //turn intrupts on
    }
  }
  if (Serial.available() > 0)
  {
    ch = Serial.read();
    if( ctr < 0x20) {
      buff[ctr++] = ch;
    }
 //Serial.println(intime[0]); // todo: 2 buffer this value
 
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
//            Serial.print("OK");
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
          Serial.print("Board ");
          Serial.print(ram.board,DEC);
          PrintCR();
        }
        else if( tokpars.compare("ID?" ) ) {
          tokpars.advanceTail(2);
          if(tokpars.contains("?"))
          {
            Serial.print("Board ");
            Serial.print(ram.board,DEC);
            PrintCR();            
          }
          else
          {
            num1 = tokpars.to_e16();
            if (num1.value > 0 && num1.value < 256)
            {
              ram.board = num1.value;
//              Serial.print("Board ");
//              Serial.print(ram.board,DEC);
              active=false;
            }
            else
            {
              Serial.print("Board Number Invalid");
            }
            PrintCR();
          }
        }
        else if( tokpars.compare("BR?" ) ) {
          tokpars.advanceTail(2);
          num3 = tokpars.to_e32();
          ram.bitrate = num3.value;
          Serial.print("Bit Rate ");
          Serial.print(ram.bitrate);
          Serial.end();
          Serial.begin(ram.bitrate);
          PrintCR();
        }
        else if( tokpars.compare("WR?" ) ) {
          tokpars.advanceTail(2);
          num1 = tokpars.to_e16();
          tokpars.nextToken();
          num3 = tokpars.to_e32();
          EEPROM.write(num1.value, num3.value);
//          Serial.print("Write ");
//          Serial.print(num2.value);
//          Serial.print(" to ");
//          Serial.print(num1.value);
//          PrintCR();
        }
        else if( tokpars.compare("RR?" ) ) {
          tokpars.advanceTail(2);
          num3 = tokpars.to_e32();
          Serial.print(EEPROM.read(num3.value),DEC);
//          Serial.print(num3.value,DEC);
          PrintCR();
        }
        else if( tokpars.compare("WSS") ) {
          eeprom_in((us32*)&ram,0,sizeof(ram));
//          Serial.print("Write System Settings");
//          PrintCR();
        }
        else if( tokpars.compare("RSS") ) {
          eeprom_out(0,(us32*)&ram,sizeof(ram));
          Serial.end();
          Serial.begin(ram.bitrate);
          active=false;
          us16 mask = 1;
          for(i=0;i<8;i++) {
            servoOnOff[i] = (ram.spe.i & mask) > 0 ? 1 : 0;
            mask = mask<<1;
          }
          memcpy(servoPos,ram.servoPos,sizeof(servoPos));
          memcpy(mapping,ram.mapping,sizeof(mapping));
//          Serial.print("Read System Settings");
//          PrintCR();
        }
        else if( tokpars.compare("DSS") ) {
          set_default_ram();
          Serial.end();
          Serial.begin(ram.bitrate);
          active=false;
          us16 mask = 1;
          for(i=0;i<8;i++) {
            servoOnOff[i] = (ram.spe.i & mask) > 0 ? 1 : 0;
            mask = mask<<1;
          }
          memcpy(servoPos,ram.servoPos,sizeof(servoPos));
          memcpy(mapping,ram.mapping,sizeof(mapping));
//          Serial.print("Default System Settings");
//          PrintCR();
        }
        else if( tokpars.compare("V?",'|') ) {
          Serial.print("Version 1.0");
          PrintCR();
        }
        else if( tokpars.compare("?",'|') ) {
          Serial.print("Visit Pontech.com for assistance.");
          PrintCR();
        }
        else if( tokpars.compare("MS?") ) {
          tokpars.advanceTail(2);
          if( tokpars.compare("?",'|') )
          {
            Serial.print("Read Mode Removed");
            PrintCR();
          }
          else
          {
            Serial.print("Write Mode Removed");
            PrintCR();
          }
        }
        else if( tokpars.compare("SV?" ) ) {
          tokpars.advanceTail(2);
          num1 = tokpars.to_e16();
          servo = (us8)num1.value;
//          Serial.print("servo ");
//          Serial.print(servo,DEC);
//          PrintCR();
        }
        else if( tokpars.compare("M?") ) {
          tokpars.advanceTail(1);
          if( tokpars.compare("?|",'|') )
          {
            tokpars.advanceTail(1);
            num1 = tokpars.to_e16();
            Serial.print("Servo ");
            Serial.print(num1.value,DEC);
            Serial.print(" in pos ");
            if (num1.value>8)
              Serial.print(servoPos[mapping[num1.value]]/2,DEC);
            else
              Serial.print(servoPos[mapping[num1.value]]/2,DEC);
            PrintCR();
          }
          else
          {
            num1 = tokpars.to_e16();
            if(mapping[servo]<8)
              servoPos[mapping[servo]] = num1.value*2 > ram.upperlimit ? ram.upperlimit : num1.value*2 < ram.lowerlimit ? ram.lowerlimit : num1.value*2;
//            Serial.print("Move Servo ");
//            Serial.print(mapping[servo],DEC);
//            PrintCR();
          }
        }
        else if( tokpars.compare("I?" ) ) {
          tokpars.advanceTail(1);
          num1 = tokpars.to_e16();
          servoPos[mapping[servo]] = num1.value*2+servoPos[mapping[servo]] > ram.upperlimit ? ram.upperlimit : num1.value*2+servoPos[mapping[servo]] < ram.lowerlimit ? ram.lowerlimit : num1.value*2+servoPos[mapping[servo]];
//          Serial.print("Relative ");
//          Serial.print(num1.value);
//          PrintCR();
        }
        else if( tokpars.compare("SS?" ) ) {
          tokpars.advanceTail(2);
          if( tokpars.compare("D" ) ) {
            for(i=0;i<16;i++) {
              mapping[i] = i;
            }
//            Serial.print("Default Map");
//            PrintCR();
          }
          else if( tokpars.compare("R" ) ) {
            memcpy(ram.mapping,mapping,sizeof(mapping));
//            Serial.print("Copy Map to Ram");
//            PrintCR();
          }
          else if( tokpars.compare("M" ) ) {
            for(i=0;i<16;i++) {
              Serial.print(mapping[i],DEC);
              Serial.print(" ");
            }
            PrintCR();
          }
          else
          {
            num1 = tokpars.to_e16();
            tokpars.nextToken();
            num2 = tokpars.to_e16();
            mapping[num2.value]=num1.value;
//            Serial.print("Map ");
//            Serial.print(num2.value);
//            Serial.print(" to ");
//            Serial.print(num1.value);
//            PrintCR();
          }
        }
        else if( tokpars.compare("SRS") ) {
          memcpy(mapping,ram.mapping,sizeof(mapping));
//          Serial.print("Copy Ram to Map");
//          PrintCR();
        }
        else if( tokpars.compare("CSR") ) {
          memcpy(ram.servoPos,servoPos,sizeof(servoPos));
//          Serial.print("Copy Servo to Ram");
//          PrintCR();
        }
        else if( tokpars.compare("CRS") ) {
          memcpy(servoPos,ram.servoPos,sizeof(servoPos));
//          Serial.print("Copy Ram to Servo");
//          PrintCR();
        }
        else if( tokpars.compare("SPE?") ) {
          tokpars.advanceTail(3);
          if( tokpars.compare("?",'|') ) {
            Serial.print("SPE 0x");
            Serial.print(ram.spe.i,HEX); 
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
//            Serial.print("Servo PPM Enable ");
//            Serial.print(ram.spe,DEC);
//            PrintCR();
          }
        }
        else if( tokpars.compare("PS?" ) ) {
          tokpars.advanceTail(2);
          num1 = tokpars.to_e16();
          if(num1.value >=0 && num1.value <8)
            digitalWrite(servoPin[num1.value], HIGH);
//          Serial.print("Set Pin ");
//          Serial.print(num1.value);
//          PrintCR();
        }
        else if( tokpars.compare("PC?" ) ) {
          tokpars.advanceTail(2);
          num1 = tokpars.to_e16();
          if(num1.value >=0 && num1.value <8)
            digitalWrite(servoPin[num1.value], LOW);
//          Serial.print("Clear Pin ");
//          Serial.print(num1.value);
//          PrintCR();
        }
        else if( tokpars.compare("PT?" ) ) {
          tokpars.advanceTail(2);
          num1 = tokpars.to_e16();
          if(num1.value >=0 && num1.value <8)
            digitalWrite(servoPin[num1.value], !digitalRead(servoPin[num1.value]));
//          Serial.print("Toggle Pin ");
//          Serial.print(num1.value);
//          PrintCR();
        }
        else if( tokpars.compare("RP?|",'|' ) ) {
          tokpars.advanceTail(3);
          num1 = tokpars.to_e16();
          if(num1.value >7 && num1.value <16) {
            Serial.print(digitalRead(servoPin[num1.value]),DEC);
//            Serial.print("Read Pin ");
//            Serial.print(num1.value);
            PrintCR();
          }
        }
        else if( tokpars.compare("AD?|",'|' ) ) {
          tokpars.advanceTail(3);
          num1 = tokpars.to_e16();
          
          Serial.print("Analog ");
          Serial.print(analogRead(A9));
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
/*        else if( tokpars.compare("size") ) {
          Serial.print(sizeof(ram.board),DEC);
          PrintCR();
          Serial.print(sizeof(ram.bitrate),DEC);
          PrintCR();
          Serial.print(sizeof(ram.mapping),DEC);
          PrintCR();
          Serial.print(sizeof(ram.servoPos),DEC);
          PrintCR();
          Serial.print(sizeof(packed_int),DEC);
          PrintCR();
          Serial.print(sizeof(ram.lowerlimit),DEC);
          PrintCR();
          Serial.print(sizeof(ram.upperlimit),DEC);
          PrintCR();
          Serial.print(sizeof(ram.structend),DEC);
          PrintCR();
          Serial.print(sizeof(ram_struct),DEC);
          PrintCR();
        }
*/  
  
        if( tokpars.compare("times") ) {
          for(i=0;i<16;i++) {
          Serial.print(servoPos[i],DEC);
          PrintCR();
          }
        }
      }
    }
  }
}
void PrintCR() {
  #ifdef WantNewLine
  Serial.println("");
  #else
  Serial.print("\r");
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
void set_default_ram() {
  ram.board = 1;
  ram.bitrate = 9600;//115200
  ram.spe.i = 0xffff;
  ram.lowerlimit = 20000;
  ram.upperlimit = 40000;
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
  if(servoPos[servoNum] > 0 && servoOnOff[servoNum] == 1 ) {
    digitalWrite(servoPin[servoNum], HIGH);
    SetDCOC1PWM(servoPos[servoNum]);
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

void __ISR(_CHANGE_NOTICE_VECTOR, ipl7) CN_Interrupt_ISR(void)
{
//  bool temp = digitalRead(43);
//  digitalWrite(43, temp ^ 1);
  
  us16 static lastb;
  us16 static lastd;
  us32 now;
  now = micros();
  if ((PORTB & 0x01) != (lastb & 0x01)) //cn2
  {
    lastused[0]=now;
    if (PORTB & 0x01) 
      risingtime[0]=now;
    else if(ram.spe.b.b8)
      servoPos[8]=((now>risingtime[0]) ? now-risingtime[0] : (0xffffffff - risingtime[0]) + now)*20;
  }
  if ((PORTB & 0x02) != (lastb & 0x02)) //cn3
  {
    lastused[1]=now;
    if (PORTB & 0x02)
      risingtime[1]=now;
    else if(ram.spe.b.b9)
      servoPos[9]=((now>risingtime[1]) ? now-risingtime[1] : (0xffffffff - risingtime[1]) + now)*20;
  }
  if ((PORTB & 0x04) != (lastb & 0x04)) //cn4
  {
    lastused[2]=now;
    if (PORTB & 0x04) //cn4
      risingtime[2]=now;
    else if(ram.spe.b.b10)
      servoPos[10]=((now>risingtime[2]) ? now-risingtime[2] : (0xffffffff - risingtime[2]) + now)*20;
  }
  if ((PORTB & 0x08) != (lastb & 0x08)) //cn5
  {
    lastused[3]=now;
    if (PORTB & 0x08)
      risingtime[3]=now;
    else if(ram.spe.b.b11)
      servoPos[11]=((now>risingtime[3]) ? now-risingtime[3] : (0xffffffff - risingtime[3]) + now)*20;
  }
  if ((PORTB & 0x10) != (lastb & 0x10)) //cn6
  {
    lastused[4]=now;
    if (PORTB & 0x10)
      risingtime[4]=now;
    else  if(ram.spe.b.b12)
      servoPos[12]=((now>risingtime[4]) ? now-risingtime[4] : (0xffffffff - risingtime[4]) + now)*20;
  }
  if ((PORTB & 0x20) != (lastb & 0x20)) //cn7
  {
    lastused[5]=now;
    if (PORTB & 0x20) 
      risingtime[5]=now;
    else if(ram.spe.b.b13)
      servoPos[13]=((now>risingtime[5]) ? now-risingtime[5] : (0xffffffff - risingtime[5]) + now)*20;
  }
  if ((PORTB & 0x8000) != (lastb & 0x8000)) //cn12
  {
    lastused[6]=now;
    if (PORTB & 0x8000)
      risingtime[6]=now;
    else if(ram.spe.b.b14)
      servoPos[14]=((now>risingtime[6]) ? now-risingtime[6] : (0xffffffff - risingtime[6]) + now)*20;
  }
 if ((PORTD & 0x40) != (lastd & 0x40)) //cn15
  {
    lastused[7]=now;
    if (PORTD & 0x40)
      risingtime[7]=now;
    else if(ram.spe.b.b15)
      servoPos[15]=((now>risingtime[7]) ? now-risingtime[7] : (0xffffffff - risingtime[7]) + now)*20;
  }
  lastb = PORTB; // Read PORTB to clear mismatch condition
  lastd = PORTD; // Read PORTD to clear mismatch condition
  IFS1CLR = 0x0001; // Be sure to clear the CN interrupt status
                  // flag before exiting the service routine.
}

} // end extern "C"


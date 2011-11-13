#define s8 signed char
#define s16 signed int
#define s32 signed long
#define us8 unsigned char
#define us16 unsigned int
#define us32 long unsigned
// Uses one or two timers, output compare and interrupts
// to achieve better than 14-bit resolution with 8 servos
// (or better than 16-bit if EXTRA_RES is #defined
// ...though even the lower figure is already pretty
// absurd and more than typical servos can resolve).

// This uses the "communication" row on the Max32
// simply because my servo adapter doodad fit there,
// not for any important reason.  These can be changed
// to most anything EXCEPT pin 3, and they do not need
// to be consecutive pins nor in-order.
byte servoPin[8] = { 12, 11, 16, 17, 18, 19, 20, 21 };
us32 risingtime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
us32 intime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// This code assumes servo timings with the canonical
// 50 Hz (20 millisecond) frequency with a 1.0 to 2.0
// millisecond pulse for position.  Some servos are
// capable of operating beyond this range, but the
// timings here are set up for the conservative case.
// The longest pulse this can handle is 2.5 mS due to
// the 8-way split of the 20 mS interval.
#ifdef EXTRA_RES
  #define SERVO_MIN  (F_CPU / 1000)                // 1.0 mS
  typedef int servo_t;
#else
  #define SERVO_MIN  (F_CPU / 4 / 1000)            // 1.0 mS
  typedef unsigned short servo_t;
#endif
#define SERVO_MAX    (SERVO_MIN * 2)               // 2.0 mS
#define SERVO_CENTER ((SERVO_MIN + SERVO_MAX) / 2) // 1.5 mS
servo_t servoPos[8];//array that holds servo positions
us8 buff[0x20];
us8 ctr;
us8 ch;
volatile int state = LOW;
void setup()
{
  // Enable output pins for servos, set inital states to 'off'
  for(int i = 0; i < 8; i++) {
    pinMode(servoPin[i], OUTPUT);
    digitalWrite(servoPin[i], LOW);
    servoPos[i] = 0;
  }
  Serial.begin(9600);
  pinMode(13,OUTPUT);
  attachInterrupt(1, RisingInterrupt, RISING);

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
us16 value;
TRISBSET = 0x04;
IEC0CLR = 0x10000000;   //turn intrupts off
AD1PCFGSET = 0x04;     //turn analog 2 off
CNCON = 0x00008000;   //turn "interrupt on change" on
CNEN = 0x00000010;   //enable cn4
CNPUE= 0x00000010; //weak pull up
value = PORTB;      //Read the Port
IPC6SET = 0x001f0000; //set priority to 7 sub 4
IFS1CLR = 0x0001; //clear the interupt flag bit
IEC1SET= 0x0001; // Enable Change Notice interrupts
IEC0SET = 0x10000000;   //turn intrupts on
}

// Motion example generates a nice millipede-like sine wave
// across a row of eight servos.

float x = 0.0;  // Used only for motion demo below; not servo driver

void loop()
{
  if (Serial.available() > 0)
  {
    digitalWrite(13, state);
    ch = Serial.read();
    if( ctr < 0x20) {
      buff[ctr++] = ch;
    }
    //digitalWrite(13, state);   // set the LED on
 //Serial.println(intime[0]); // todo: 2 buffer this value
 
    if (ch == '\r')
    {
      buff[--ctr] = 0;
      ctr = 0;
      ch = buff[0];
      if( strcmp( (const char *)buff, "s0 1" ) == 0 ) {
        servoPos[0] = SERVO_MIN;
        Serial.println("OK");
      }
      if( strcmp( (const char *)buff, "s0 2" ) == 0 ) {
        servoPos[0] = SERVO_MAX;
        Serial.println("OK");
      }
      if( strcmp( (const char *)buff, "s1 1" ) == 0 ) {
        servoPos[1] = SERVO_MIN;
        Serial.println("OK");
      }
      if( strcmp( (const char *)buff, "s1 2" ) == 0 ) {
        servoPos[1] = SERVO_MAX;
        Serial.println("OK");
      }
      if( strcmp( (const char *)buff, "follow" ) == 0 ) {
        servoPos[0] = 20*intime[0];
        Serial.println("OK");
      }
      if( strcmp( (const char *)buff, "readb" ) == 0 ) {
        Serial.println(PORTB, HEX);
      }
     delay(20);
    }
  }

/*  float y = x;
  for(int i = 0; i < 8;i ++) {
    servoPos[i] = SERVO_CENTER +
      (int)((float)(SERVO_MAX - SERVO_CENTER) * sin(y));
    y += M_PI / 5.0;
  }
  x += M_PI / 100.0;

  delay(20);  // No point updating faster than servo pulses
*/}

void RisingInterrupt()
{
 attachInterrupt(1, FallingInterrupt, FALLING);
 risingtime[0]=micros();
}
void FallingInterrupt()
{
 attachInterrupt(1, RisingInterrupt, RISING);
 intime[0]=micros()-risingtime[0];
}

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
  if(servoPos[servoNum] > 0) {
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
  digitalWrite(servoPin[servoNum], LOW);
  if(++servoNum > 7) servoNum = 0;  // Back to start
}
void __ISR(_CHANGE_NOTICE_VECTOR, ipl5) CN_Interrupt_ISR(void)
{
unsigned int value;
value = PORTB; // Read PORTB to clear CN2 mismatch condition
state=!state;//... perform application specific operations in response to the interrupt
digitalWrite(13, state);
IFS1CLR = 0x0001; // Be sure to clear the CN interrupt status
                  // flag before exiting the service routine.
}

} // end extern "C"


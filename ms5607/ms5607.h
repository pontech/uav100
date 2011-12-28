#ifndef _MS5607_H_
#define _MS5607_H_

#include <core.h>
#include <SPI.h>

#define average_ready 10

#define MS5607_READ_C1  0xa2
#define MS5607_READ_C2  0xa4
#define MS5607_READ_C3  0xa6
#define MS5607_READ_C4  0xa8
#define MS5607_READ_C5  0xaa
#define MS5607_READ_C6  0xac
#define MS5607_READ_D1  0x48  // raw pressure
#define MS5607_READ_D2  0x58  // raw temprature

typedef struct {
  us16 sens; //Pressure sensitivity
  us16 off; //Pressure offset
  us16 tcs; //Temperature coefficient of pressure sensitivity
  us16 tco; //Temperature coefficient of pressure offset
  us16 tref; //Reference temperature
  us16 tempsens; //Temperature coefficient of the temperature
  s32 temperature; //tempreature out cx100
  s32 pressure; //pressure out mbx100
  s32 alt_offset; //offset the calculated altitude to known altitude
  us8 measurements; //variable to track the number of measurements made
  s32 alt_raw; //calculated altitude mm
  s32 alt_averaged; //averaged altitude mm
  s32 alt_average_with_offset;
} Alt_Data; // 36 bytes
Alt_Data Altitude_Data;
s32 Altitude_Display;
us8 ms5607_cs;

//Functions
void ms5607_init(us8 cs) {
  ms5607_cs = cs;
  SPI.begin();
  SPI.setClockDivider(1);//0=40mhz 1=20 2=13.3
  pinMode(ms5607_cs, OUTPUT);
  digitalWrite(ms5607_cs,HIGH);
  digitalWrite(ms5607_cs,LOW);
  SPI.transfer(0x1e); //Reset the device
  delay(6);//wait for reset
  digitalWrite(ms5607_cs,HIGH);
  digitalWrite(ms5607_cs,LOW);
  SPI.transfer(MS5607_READ_C1); //read prom 1 instruction
  Altitude_Data.sens = SPI.transfer(0x00)<<8 | SPI.transfer(0x00);
  digitalWrite(ms5607_cs,HIGH);
  digitalWrite(ms5607_cs,LOW);
  SPI.transfer(MS5607_READ_C2); //read prom 2 instruction
  Altitude_Data.off = SPI.transfer(0x00)<<8 | SPI.transfer(0x00);
  digitalWrite(ms5607_cs,HIGH);
  digitalWrite(ms5607_cs,LOW);
  SPI.transfer(MS5607_READ_C3); //read prom 3 instruction
  Altitude_Data.tcs = SPI.transfer(0x00)<<8 | SPI.transfer(0x00);
  digitalWrite(ms5607_cs,HIGH);
  digitalWrite(ms5607_cs,LOW);
  SPI.transfer(MS5607_READ_C4); //read prom 4 instruction
  Altitude_Data.tco = SPI.transfer(0x00)<<8 | SPI.transfer(0x00);
  digitalWrite(ms5607_cs,HIGH);
  digitalWrite(ms5607_cs,LOW);
  SPI.transfer(MS5607_READ_C5); //read prom 5 instruction
  Altitude_Data.tref = SPI.transfer(0x00)<<8 | SPI.transfer(0x00);
  digitalWrite(ms5607_cs,HIGH);
  digitalWrite(ms5607_cs,LOW);
  SPI.transfer(MS5607_READ_C6); //read prom 6 instruction
  Altitude_Data.tempsens = SPI.transfer(0x00)<<8 | SPI.transfer(0x00);
  digitalWrite(ms5607_cs,HIGH);
  Altitude_Data.alt_offset = 0; //set offset to known value
  Altitude_Data.measurements = 0; //preset measurements to zero
}


us32 ms5607_24bit_read(us8 ms5607_command_byte){
  SPI.begin();
  SPI.setClockDivider(1);//0=40mhz 1=20 2=13.3
  uus32 value;

  digitalWrite(ms5607_cs,LOW);
  SPI.transfer(ms5607_command_byte);
  delay(10);
  digitalWrite(ms5607_cs,HIGH);

  digitalWrite(ms5607_cs,LOW);
  SPI.transfer(0x00); //Read ADC
  value.parts.hi.parts.hi.value = 0x00;
  value.parts.hi.parts.lo.value = SPI.transfer(0x00);
  value.parts.lo.parts.hi.value = SPI.transfer(0x00);
  value.parts.lo.parts.lo.value = SPI.transfer(0x00);
  digitalWrite(ms5607_cs,HIGH);
  return value.value;
}
void ms5607_real_both(){
    s32 dt;
    s32 temp;
    us32 d2;
    us32 var;
    d2 = ms5607_24bit_read(MS5607_READ_D2);//alt_read_temperature();
    var = ((us32)Altitude_Data.tref*256);
    dt = d2-var; // =  D2 - C5 * 2^8
    temp = Altitude_Data.tempsens;
    temp = temp >> 4;
    temp = 2000 + (dt*temp)/0x80000; //TEMP = 20° + dT * TEMPSENS = 2000 + dT * C6 / 2^23
    // todo: 4 add SECOND ORDER TEMPERATURE COMPENSATION
    Altitude_Data.temperature = temp;
    us32 off;//should be s64
    us32 sens;//should be s64
    us32 d1;
    d1 = ms5607_24bit_read(MS5607_READ_D1);//alt_read_pressure();
//    dt >>= 4;// todo: 4 verify this works instead of divide by 16
    dt /= 16;
    if (dt>0) {
        off = (us32)(Altitude_Data.off)*0x2000+((us32)(Altitude_Data.tco)*(dt))/64; //OFF/16
        sens = (us32)(Altitude_Data.sens)*0x1000+((us32)(Altitude_Data.tcs)*(dt))/128; //SENS/16
        Altitude_Data.pressure = (((d1>>4)*(sens/0x8000))/0x40-off/16)/0x80; //P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15
    }
    else
    {
        off = (us32)(Altitude_Data.off)*0x2000+((us32)(Altitude_Data.tco)*(dt))/64; //OFF/16
        sens = (us32)(Altitude_Data.sens)*0x1000+((us32)(Altitude_Data.tcs)*(dt))/128; //SENS/16
        Altitude_Data.pressure = -(((d1>>4)*(sens/0x8000))/0x40+off/16)/0x80; //P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15
    }
}
// Plower Values from data sheet * 10 to put them in tenth meter's
const s32 Plower5607[] = {10000,11300,13000,15000,17300,20000,23000,26500,30000,33500,37000,41000,45000,50000,55000,60000,65000,71000,78000,85000,92000,97000,103000};
const int i[] = {12256,10758,9329,8085,7001,6069,5360,4816,4371,4020,3702,3420,3158,2908,2699,2523,2359,2188,2033,1905,1802,1720,1638};
// j Values from data sheet * 10 to put them in tenth meter's
const s32 j5607[] = {162120,154340,145410,136300,127220,117990,109100,99940,91710,84240,77370,70140,63460,55750,48650,42060,35900,28990,21510,14560,8050,3650,-1390};


void ms5607_calculation(us8 buffer_clean = 0) {
    ms5607_real_both();
    int MAX = 23;
    int n;
    if (buffer_clean == 1)
    {
        Altitude_Data.measurements = 0;
    }
    s32 Altitude_Temp;
    // Lookup i and j from table
    for (n = 0; n < MAX; n++) {
        if (Altitude_Data.pressure < Plower5607[n]) {
            break;
        }
    }
    n--;

    Altitude_Data.alt_raw = ((s32)j5607[n] - ((((s32)Altitude_Data.pressure - (s32)Plower5607[n]) * (s32)i[n]) / 2048L))*100;
    //Altitude += Altitude_Offset;


    if (Altitude_Data.measurements <= 1) {
        Altitude_Data.measurements++;
        Altitude_Temp = Altitude_Data.alt_raw;
        Altitude_Data.alt_averaged = Altitude_Data.alt_raw;
    }
    else if (Altitude_Data.measurements < average_ready) {
        Altitude_Data.measurements++;
        Altitude_Temp = Altitude_Data.alt_raw;
    }
    else {
        Altitude_Temp = Altitude_Data.alt_raw;
    }

    // Commented for Offset
    Altitude_Data.alt_averaged = ((Altitude_Temp * 7) + (Altitude_Data.alt_averaged * 93)) / 100;

    // Offset
    Altitude_Data.alt_average_with_offset = Altitude_Data.alt_averaged + Altitude_Data.alt_offset;

}
us32 barometer_adjust(us32 pressure_in, s32 altitude_in) {//returns pressure adjusted to sea level for use as a barometer
    us32 standardpressure;
    us8 n;
    us8 MAX = 23;
    // Lookup n for table
    for (n = 0; n < MAX; n++) {
        if (altitude_in > j5607[n]) {
            break;
        }
    }
    n--;
    standardpressure = (s32)j5607[n] - altitude_in; //((jn-h) * 2^11) / in + Plower = p
    standardpressure = standardpressure * 2048;
    standardpressure = standardpressure / (s32)i[n];
    standardpressure = standardpressure + (s32)Plower5607[n]; //standard pressure holds what the pressure should be at that altitude
    return 101325 - (standardpressure - pressure_in);//101325 standard sea level pressure
}


#endif

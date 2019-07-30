#include <Wire.h>
#include <Adafruit_Sensor.h>

#define OUT Serial1 // Use Serial instead to output via USB cable
#define BAUD_RATE (57600) // RPi must use 57600

enum wvsu_experiment {
  WVSU_SYSTEM = 0,
  WVSU_OPTICAL,
  WVSU_FXOS8700,
  WVSU_FXAS21002C,
  WVSU_BMP280,
  WVSU_GEIGER,
  WVSU_PARTICLE
};
int wvsu_state = WVSU_OPTICAL;

/**********************************\
 * OPTICAL ORIENTATION EXPERIMENT *
\**********************************/
// Optical global variables
const int optical_pin[] = {A4,A5,A6};
uint8_t optical_raw[] = {0,0,0,0,0,0};
enum optical_states {
  OPTICAL_1 = 0,
  OPTICAL_2,
  OPTICAL_3
};
int optical_state = OPTICAL_1;

static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void ADCsync()
{
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncGCLK()
{
  while (GCLK->STATUS.bit.SYNCBUSY == 1)
    ; //Just wait till the ADC is free
}

uint32_t anaRead(uint32_t ulPin)
{
  ADCsync();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ulPin].ulADCChannelNumber; // Selection for the positive ADC input
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
  ADC->INTFLAG.bit.RESRDY = 1;              // Data ready flag cleared
  ADCsync();
  ADC->SWTRIG.bit.START = 1;                // Start ADC conversion
  while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Wait till conversion done
  ADCsync();
  uint32_t valueRead = ADC->RESULT.reg;
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable the ADC 
  ADCsync();
  ADC->SWTRIG.reg = 0x01;                    //  and flush for good measure
  return valueRead;
}

void optical_setup() {
  ADCsync();
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;      // Gain select as 1X
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; //  2.2297 V Supply VDDANA

  // Set sample length and averaging
  ADCsync();
  ADC->AVGCTRL.reg = 0x00 ;       //Single conversion no averaging
  ADCsync();
  ADC->SAMPCTRL.reg = 0x0A;  ; //sample length in 1/2 CLK_ADC cycles Default is 3F
  
  //Control B register
  int16_t ctrlb = 0x400;       // Control register B hibyte = prescale, lobyte is resolution and mode 
  ADCsync();
  ADC->CTRLB.reg =  ctrlb     ; 
  anaRead(optical_pin[0]);  //Discard first conversion after setup as ref changed
}

void optical_iterate() {
  switch (optical_state) {
    case OPTICAL_1: case OPTICAL_2: case OPTICAL_3:
    {
//      optical_raw[optical_state] = analogRead(optical_pin[optical_state]);
      optical_raw[optical_state] = anaRead(optical_pin[optical_state]);

      optical_state++;
    }
  }
}

void optical_report() { //File FILE) {
  OUT.write(optical_raw,3);
//  FILE.write(optical_raw,3);
  optical_state = OPTICAL_1;
}


/**********************************\
 * GEIGER-MUELLER TUBE EXPERIMENT *
\**********************************/
// Geiger global variables
const int geiger_pin[] = {6,7,8,9,10};
uint8_t geiger_count[] = {0,0,0,0,0};
int geiger_raw[] = {LOW,LOW,LOW,LOW,LOW};
int GeigerState[] = {LOW,LOW,LOW,LOW,LOW};
int geiger_last[] = {LOW,LOW,LOW,LOW,LOW};
int GeigerOutput[] = {LOW,LOW,LOW,LOW,LOW};
unsigned long GeigerFlickerTime[] = {0,0,0,0,0};
unsigned long GeigerFlickerDelay = 0;

void geiger_setup() {
  for ( int x = 0; x < 5; x++ ) {
    pinMode(geiger_pin[x], INPUT);
  }
}

void geiger_iterate() {
  for ( int x = 0; x < 5; x++ ) {
    geiger_raw[x] = digitalRead(geiger_pin[x]);
  }
  for ( int x = 0; x < 5; x++ ) {
    if ( geiger_last[x] == HIGH &&  geiger_raw[x] == LOW ) {
      // falling edge of signal counts
      geiger_count[x]++;
    }

    // save the reading. Next time through the loop, it'll be the GeigerLast:
    geiger_last[x] = geiger_raw[x];
  }
}

void geiger_report() { //File FILE) {
  OUT.write(geiger_count,5);
//  FILE.write(geiger_count,5);
  geiger_count[0] = 0;
  geiger_count[1] = 0;
  geiger_count[2] = 0;
  geiger_count[3] = 0;
  geiger_count[4] = 0;
}


/***********************************\
 * SI PARTICLE DETECTOR EXPERIMENT *
\***********************************/
// Particle global variables
int ParticleSignalPin[] = {4,2,0};
int ParticleNoisePin[] = {5,3,1};
uint8_t ParticleSignalCount[] = {0,0,0};
uint8_t ParticleNoiseCount[] = {0,0,0};
int ParticleSignalRaw[] = {LOW,LOW,LOW};
int ParticleNoiseRaw[] = {HIGH,HIGH,HIGH};
int ParticleSignalState[] = {LOW,LOW,LOW};
int ParticleNoiseState[] = {HIGH,HIGH,HIGH};
int ParticleSignalLast[] = {LOW,LOW,LOW};
int ParticleNoiseLast[] = {HIGH,HIGH,HIGH};
int ParticleSignalOutput[] = {LOW,LOW,LOW};
int ParticleNoiseOutput[] = {HIGH,HIGH,HIGH};
unsigned long ParticleSignalFlickerTime[] = {0,0,0};
unsigned long ParticleNoiseFlickerTime[] = {0,0,0};
unsigned long ParticleFlickerDelay = 0;

void particle_setup() {
  for ( int x = 0; x < 3; x++ ) {
    pinMode(ParticleSignalPin[x], INPUT_PULLUP);
    pinMode(ParticleNoisePin[x], INPUT_PULLUP);
  }
}

void particle_iterate() {
  for ( int x = 0; x < 3; x++ ) {
    ParticleSignalRaw[x] = digitalRead(ParticleSignalPin[x]);
    ParticleNoiseRaw[x] = digitalRead(ParticleNoisePin[x]);
  }
  for ( int x = 0; x < 3; x++ ) {
    if ( ParticleSignalLast[x] == HIGH &&  ParticleSignalRaw[x] == LOW ) {
      // falling edge of signal counts
      ParticleSignalCount[x]++;
    }
    if ( ParticleNoiseLast[x] == LOW && ParticleNoiseRaw[x] == HIGH ) {
      // rising edge of signal counts
      ParticleNoiseCount[x]++;
    }
    // save the reading.
    ParticleSignalLast[x] = ParticleSignalRaw[x];
    ParticleNoiseLast[x] = ParticleNoiseRaw[x];
  }
}

void particle_report() { //File FILE) {
  OUT.write(ParticleSignalCount,3);
  OUT.write(ParticleNoiseCount,3);
//  FILE.write(ParticleSignalCount,3);
//  FILE.write(ParticleNoiseCount,3);
  ParticleSignalCount[0] = 0;
  ParticleSignalCount[1] = 0;
  ParticleSignalCount[2] = 0;
  ParticleNoiseCount[0] = 0;
  ParticleNoiseCount[1] = 0;
  ParticleNoiseCount[2] = 0;
}


/*********************************\
 * STANDALONE I2C SENSOR MODULES *
\*********************************/
#define FXOS8700_ADDRESS                (0x1F)
#define FXOS8700_ID                (0xC7)     // 1100 0111
#define FXOS8700_REGISTER_WHO_AM_I      (0x0D)
#define FXOS8700_REGISTER_CTRL_REG1     (0x2A)
#define FXOS8700_REGISTER_XYZ_DATA_CFG  (0x0E)
#define FXOS8700_REGISTER_CTRL_REG2     (0x2B)
#define FXOS8700_REGISTER_MCTRL_REG1    (0x5B)
#define FXOS8700_REGISTER_MCTRL_REG2    (0x5C)
#define FXOS8700_REGISTER_STATUS        (0x00)

#define FXAS21002C_ADDRESS              (0x21)       // 0100001
#define FXAS21002C_ID                   (0xD7)       // 1101 0111
#define GYRO_REGISTER_WHO_AM_I          (0x0C)       // 0x0C (default value = 0b11010111, read only)
#define GYRO_REGISTER_CTRL_REG0         (0x0D)       // 0x0D (default value = 0b00000000, read/write)
#define GYRO_REGISTER_CTRL_REG1         (0x13)       // 0x13 (default value = 0b00000000, read/write)
#define GYRO_REGISTER_STATUS            (0x00)       // 0x00

#define BMP280_ADDRESS (0x77) /**< The default I2C address for the sensor. */
#define BMP280_CHIPID (0x58) /**< Default chip ID. */
#define BMP280_REGISTER_CHIPID          (0xD0)
#define BMP280_REGISTER_DIG_T1          (0x88)
#define BMP280_REGISTER_DIG_T2          (0x8A)
#define BMP280_REGISTER_DIG_T3          (0x8C)
#define BMP280_REGISTER_DIG_P1          (0x8E)
#define BMP280_REGISTER_DIG_P2          (0x90)
#define BMP280_REGISTER_DIG_P3          (0x92)
#define BMP280_REGISTER_DIG_P4          (0x94)
#define BMP280_REGISTER_DIG_P5          (0x96)
#define BMP280_REGISTER_DIG_P6          (0x98)
#define BMP280_REGISTER_DIG_P7          (0x9A)
#define BMP280_REGISTER_DIG_P8          (0x9C)
#define BMP280_REGISTER_DIG_P9          (0x9E)
#define BMP280_REGISTER_CONTROL         (0xF4)
#define BMP280_REGISTER_CONFIG          (0xF5)
#define BMP280_REGISTER_PRESSUREDATA    (0xF7)
#define BMP280_REGISTER_TEMPDATA        (0xFA)

enum i2c_modules {
  I2C_FXOS8700 = 4,
  I2C_FXAS21002C,
  I2C_BMP280_TEMP,
  I2C_BMP280_PRESS
};
int i2c_lock = I2C_FXOS8700;
uint8_t i2c_fxos8700_raw[13];
uint8_t i2c_fxas21002c_raw[7];
uint8_t i2c_bmp280_t1_raw[2];
uint8_t i2c_bmp280_t2_raw[2];
uint8_t i2c_bmp280_t3_raw[2];
uint8_t i2c_bmp280_p1_raw[2];
uint8_t i2c_bmp280_p2_raw[2];
uint8_t i2c_bmp280_p3_raw[2];
uint8_t i2c_bmp280_p4_raw[2];
uint8_t i2c_bmp280_p5_raw[2];
uint8_t i2c_bmp280_p6_raw[2];
uint8_t i2c_bmp280_p7_raw[2];
uint8_t i2c_bmp280_p8_raw[2];
uint8_t i2c_bmp280_p9_raw[2];
uint8_t i2c_bmp280_temperature_raw[3];
uint8_t i2c_bmp280_pressure_raw[3];
bool i2c_fxos8700_stale = true;
bool i2c_fxas21002c_stale = true;
bool i2c_bmp280_temp_stale = true;
bool i2c_bmp280_press_stale = true;

void i2c_write8(byte addr, byte reg, byte value)
{
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

byte i2c_read(byte addr, byte reg, uint8_t* value, byte len) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)reg);
  if ( Wire.endTransmission() != 0) return 0;
  Wire.requestFrom(addr, len);
  byte x = 0;
  while(x < len) {
    value[x++] = Wire.read();
  }
  return x;
}

byte i2c_read8(byte addr, byte reg)
{
  byte value;

  Wire.beginTransmission(addr);
  Wire.write((uint8_t)reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(addr, (byte)1);
  value = Wire.read();

  return value;
}

void i2c_setup() {
  /* Enable I2C */
  Wire.begin();
}

void fxos8700_setup() {
  /* Confirm presence of FXOS8700 sensor */
  uint8_t i2c_fxos8700_id = i2c_read8(FXOS8700_ADDRESS,FXOS8700_REGISTER_WHO_AM_I);
  if (i2c_fxos8700_id == FXOS8700_ID) {
    /* Set to standby mode (required to make changes to this register) */
    i2c_write8(FXOS8700_ADDRESS,FXOS8700_REGISTER_CTRL_REG1, 0);
//    delay(100);
    /* 2G range */
    i2c_write8(FXOS8700_ADDRESS,FXOS8700_REGISTER_XYZ_DATA_CFG, 0x00);
    /* High resolution */
    i2c_write8(FXOS8700_ADDRESS,FXOS8700_REGISTER_CTRL_REG2, 0x02);
    /* Active, Normal Mode, Low Noise, 400Hz in Hybrid Mode */
    i2c_write8(FXOS8700_ADDRESS,FXOS8700_REGISTER_CTRL_REG1, 0x05);
    /* Configure the magnetometer */
    /* Hybrid Mode, Over Sampling Rate = 16 */
    i2c_write8(FXOS8700_ADDRESS,FXOS8700_REGISTER_MCTRL_REG1, 0x1F);
    /* Jump to reg 0x33 after reading 0x06 */
    i2c_write8(FXOS8700_ADDRESS,FXOS8700_REGISTER_MCTRL_REG2, 0x20);
    delay(100);
  }
}

void fxas21002c_setup() {
  /* Confirm presence of FXAS21002C sensor */
  uint8_t i2c_fxas21002c_id = i2c_read8(FXAS21002C_ADDRESS,GYRO_REGISTER_WHO_AM_I);
  if ( i2c_fxas21002c_id == FXAS21002C_ID ) {
    i2c_write8(FXAS21002C_ADDRESS,GYRO_REGISTER_CTRL_REG1, 0x00);     // Standby
    i2c_write8(FXAS21002C_ADDRESS,GYRO_REGISTER_CTRL_REG1, (1<<6));   // Reset
    i2c_write8(FXAS21002C_ADDRESS,GYRO_REGISTER_CTRL_REG0, 0x03); // Set sensitivity
    i2c_write8(FXAS21002C_ADDRESS,GYRO_REGISTER_CTRL_REG1, 0x0E);     // Active
    delay(100); // 60 ms + 1/ODR
  }
}

void bmp280_setup() {
  /* Confirm presence of BMP280 sensor */
  uint8_t i2c_bmp280_id = i2c_read8(BMP280_ADDRESS,BMP280_REGISTER_CHIPID);
  if ( i2c_bmp280_id == BMP280_CHIPID ) {
/* These calibration values do not change unless module changes.
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_T1, i2c_bmp280_t1_raw, 2);
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_T2, i2c_bmp280_t2_raw, 2);
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_T3, i2c_bmp280_t3_raw, 2);
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_P1, i2c_bmp280_p1_raw, 2);
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_P2, i2c_bmp280_p2_raw, 2);
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_P3, i2c_bmp280_p3_raw, 2);
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_P4, i2c_bmp280_p4_raw, 2);
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_P5, i2c_bmp280_p5_raw, 2);
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_P6, i2c_bmp280_p6_raw, 2);
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_P7, i2c_bmp280_p7_raw, 2);
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_P8, i2c_bmp280_p8_raw, 2);
    i2c_read(BMP280_ADDRESS,BMP280_REGISTER_DIG_P9, i2c_bmp280_p9_raw, 2);
*/
    i2c_write8(BMP280_ADDRESS,BMP280_REGISTER_CONFIG, 0x11);
    i2c_write8(BMP280_ADDRESS,BMP280_REGISTER_CONTROL, 0x57);
    delay(100); // 60 ms + 1/ODR
  }
}

void fxos8700_iterate() {
  if ( i2c_fxos8700_stale ) {
    /* Request 13 bytes from the sensor */
    if ( i2c_read(FXOS8700_ADDRESS, (FXOS8700_REGISTER_STATUS | 0x80), i2c_fxos8700_raw, 13) != 13 ) {
      // report an error.
    }
    i2c_fxos8700_stale = false;
  }
}

void fxos8700_report() { //File FILE) {
  OUT.write(i2c_fxos8700_raw,13);
  i2c_fxos8700_stale = true;
}

void fxas21002c_iterate() {
  if ( i2c_fxas21002c_stale ) {
    /* Read 7 bytes from the sensor */
    if ( i2c_read(FXAS21002C_ADDRESS, (GYRO_REGISTER_STATUS | 0x80), i2c_fxas21002c_raw, 7) != 7 ) {
      // report an error.
    }
    i2c_fxas21002c_stale = false;
  }
}

void fxas21002c_report() { //File FILE) {
  OUT.write(i2c_fxas21002c_raw,7);
  i2c_fxas21002c_stale = true;
}

enum bmp280_states {
  BMP280_READ_TEMP = 0,
  BMP280_READ_PRESS,
  BMP280_DONE
};
int bmp280_state = BMP280_READ_TEMP;

void bmp280_iterate() {
  switch (bmp280_state) {
    case BMP280_READ_TEMP:
    {
      /* Read 3 bytes from the sensor */
      if ( i2c_read(BMP280_ADDRESS, BMP280_REGISTER_TEMPDATA, i2c_bmp280_temperature_raw, 3) != 3 ) {
        // report an error.
      }
      break;
    }
    case (I2C_BMP280_PRESS): {
      /* Read 3 bytes from the sensor */
      if ( i2c_read(BMP280_ADDRESS, BMP280_REGISTER_PRESSUREDATA, i2c_bmp280_pressure_raw, 3) != 3 ) {
        // report an error.
      }
      break;
    }
  }
  bmp280_state++;
}

void bmp280_report() { //File FILE) {
/* enable this section to output calibration values which are unique to the module
  OUT.write(i2c_bmp280_t1_raw,2);
  OUT.write(i2c_bmp280_t2_raw,2);
  OUT.write(i2c_bmp280_t3_raw,2);
  OUT.write(i2c_bmp280_p1_raw,2);
  OUT.write(i2c_bmp280_p2_raw,2);
  OUT.write(i2c_bmp280_p3_raw,2);
  OUT.write(i2c_bmp280_p4_raw,2);
  OUT.write(i2c_bmp280_p5_raw,2);
  OUT.write(i2c_bmp280_p6_raw,2);
  OUT.write(i2c_bmp280_p7_raw,2);
  OUT.write(i2c_bmp280_p8_raw,2);
  OUT.write(i2c_bmp280_p9_raw,2);
//*/
  OUT.write(i2c_bmp280_temperature_raw,3);
  OUT.write(i2c_bmp280_pressure_raw,3);
  bmp280_state = BMP280_READ_TEMP;
}

/*************************\
 * COMPUTE MODULE CLOCK  *
 *************************/
enum rpi_states
{
  RPI_W = 0,
  RPI_V,
  RPI_S,
  RPI_U,
  RPI_TIME_1,
  RPI_TIME_2,
  RPI_TIME_3,
  RPI_TIME_4,
  RPI_TIME_5,
  RPI_TIME_6,
  RPI_TIME_7,
  RPI_TIME_8,
  RPI_COMPLETE
};
int rpi_state = 0;
uint8_t rpi_clock[] = {0,0,0,0,0,0,0,0};
uint8_t rpi_buffer[] = {0,0,0,0,0,0,0,0};
bool rpi_available = false;

void rpi_iterate()
{
  if ( OUT.available() > 0 )
  {
    uint8_t b = OUT.read();
    switch ( rpi_state )
    {
      case RPI_W:
      {
        if ( b != 'W' ) return;
        break;
      }
      case RPI_V:
      {
        if ( b != 'V' )
        {
          rpi_state = RPI_W;
          return;
        }
        break;
      }
      case RPI_S:
      {
        if ( b != 'S' )
        {
          rpi_state = RPI_W;
          return;
        }
        break;
      }
      case RPI_U:
      {
        if ( b != 'U' )
        {
          rpi_state = RPI_W;
          return;
        }
        break;
      }
      case RPI_TIME_1:
      {
        rpi_buffer[0] = b;
        break;
      }
      case RPI_TIME_2:
      {
        rpi_buffer[1] = b;
        break;
      }
      case RPI_TIME_3:
      {
        rpi_buffer[2] = b;
        break;
      }
      case RPI_TIME_4:
      {
        rpi_buffer[3] = b;
        break;
      }
      case RPI_TIME_5:
      {
        rpi_buffer[4] = b;
        break;
      }
      case RPI_TIME_6:
      {
        rpi_buffer[5] = b;
        break;
      }
      case RPI_TIME_7:
      {
        rpi_buffer[6] = b;
        break;
      }
      case RPI_TIME_8:
      {
        rpi_buffer[7] = b;
        break;
      }
      case RPI_COMPLETE:
      {
        if ( b == 0xFF ) {
          // reverse byte order
          rpi_clock[7] = rpi_buffer[0];
          rpi_clock[6] = rpi_buffer[1];
          rpi_clock[5] = rpi_buffer[2];
          rpi_clock[4] = rpi_buffer[3];
          rpi_clock[3] = rpi_buffer[4];
          rpi_clock[2] = rpi_buffer[5];
          rpi_clock[1] = rpi_buffer[6];
          rpi_clock[0] = rpi_buffer[7];
          rpi_available = true;
          rpi_state = RPI_W;
          return;
        }
      }
    }
    rpi_state++;
  }
}

void rpi_report() {
  if ( rpi_available )
  {
    OUT.write(rpi_clock,8);
    rpi_available = false;
  } else
  {
    uint8_t zero[] = {0,0,0,0,0,0,0,0};
    OUT.write(zero,8);
  }
}

/*************************\
 * SYSTEM-LEVEL ACTIVITY *
\*************************/

int lastReport = 0;
uint16_t cycleCount = 0;
int incomingByte;

void setup() {
  OUT.begin(BAUD_RATE);
  optical_setup();
  geiger_setup();
  particle_setup();
  i2c_setup();
  fxos8700_setup();
  fxas21002c_setup();
  bmp280_setup();
}

uint16_t dwell_time[] = {0,0,0,0};

void loop() {
  cycleCount++;
  unsigned long now = micros();
  unsigned long timeElapsed = now - lastReport;
  bool report = ( ( timeElapsed >= 20000 ) || ( ( timeElapsed > 15000 ) && ( (now/1000) % 20 == 0 ) ) );
  if ( report ) 
  {
    lastReport = now;

    OUT.write("WVSU"); // header
    OUT.write(now>>24); // msb of micros
    OUT.write(now>>16); // micros
    OUT.write(now>>8);  // micros
    OUT.write(now);       // lsb of micros
    rpi_report();         // report rpi time
    OUT.write(cycleCount>>8);
    OUT.write(cycleCount);
    OUT.write(dwell_time[0]>>8);
    OUT.write(dwell_time[0]);
    OUT.write(dwell_time[1]>>8);
    OUT.write(dwell_time[1]);
    OUT.write(dwell_time[2]>>8);
    OUT.write(dwell_time[2]);
    OUT.write(dwell_time[3]>>8);
    OUT.write(dwell_time[3]);
    dwell_time[0] = 0;
    dwell_time[1] = 0;
    dwell_time[2] = 0;
    dwell_time[3] = 0;

    cycleCount = 0;
    optical_report(); //FILE);     // 6 bytes
    geiger_report(); //FILE);      // 5 bytes
    particle_report(); //FILE);    // 6 bytes
    fxos8700_report(); //FILE);    // 13 bytes
    fxas21002c_report(); //FILE);  // 7 bytes
    bmp280_report(); //FILE);      // 6 bytes

    OUT.write((uint8_t)0xAA);
  }

  geiger_iterate();
  particle_iterate();
  rpi_iterate();
  
  /* Read values */
  switch( wvsu_state ) {
    case WVSU_OPTICAL:
    {
      optical_iterate();
      wvsu_state = WVSU_FXOS8700;
      break;
    }
    case WVSU_FXOS8700:
    {
      fxos8700_iterate();
      wvsu_state = WVSU_FXAS21002C;
      break;
    }
    case WVSU_FXAS21002C:
    {
      fxas21002c_iterate();
      wvsu_state = WVSU_BMP280;
      break;
    }
    case WVSU_BMP280:
    {
      bmp280_iterate();
      wvsu_state = WVSU_OPTICAL;
      break;
    }
    default:
    {
      wvsu_state = WVSU_OPTICAL;
    }
  }

  // track dwell time
  dwell_time[wvsu_state-1] += (uint8_t)(micros() - now);
}

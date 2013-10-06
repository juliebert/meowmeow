#include <Wire.h>
#include <MPU6050.h>

//**************************************
// Global Variables for read_accelerometer
//*************************************
// Declaring an union for the registers and the axis values.
// The byte order does not match the byte order of
// the compiler and AVR chip.
// The AVR chip (on the Arduino board) has the Low Byte
// at the lower address.
// But the MPU-6050 has a different order: High Byte at
// lower address, so that has to be corrected.
// The register part "reg" is only used internally,
// and are swapped in code.
typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};
//****************************************************************
//Enum to keep track of the Turn_Signal State and the Braking State
//****************************************************************
enum Turn_Signal{
     right_on,
     left_on,
     toff
}TSignal;
   
enum Brake_Light{
     on,
     boff
}BLight;

//*******************************************************************
//Global Variables accessed by both set-up and loop functions
//*******************************************************************
//These read out the gyro values
int xin,yin,zin,xout,yout,zout;
//Turn on and off the print statements to the serial port
int print1 = 0;
//Threshold may need some tuning
int threshold = 2000;
//accel union
accel_t_gyro_union accel_t_gyro;
//Output pins
int BRAKE_PIN = 8;
int TURN_PIN = 7;
int error;
//******************************************************************************
//***************************************************************************

//debug for button state
int print2 = 1;

int clock = 0;
// Left and Right Turn Signal Buttons
const int rightButton = 2;
const int leftButton = 3;

// Speed of turn signal blinks
int blinkTime = 100;

// Turn Signal LEDS
int pinCount = 10;
int rightSignal = 12;
int leftSignal = 13;

// Setting Up Compontents
void setup() {

  //Setting up Accellerometer Variables
  int error;
  uint8_t c;
  
  // Hello Right and Left buttons
  pinMode(rightButton, INPUT);
  pinMode(leftButton, INPUT);
  
  // Hello LED Turn Signals
  pinMode(rightSignal, OUTPUT);  
  pinMode(leftSignal, OUTPUT); 

  digitalWrite(rightSignal, LOW);
  digitalWrite(rightSignal, LOW);
  
  //Setting up Accellerometer
  BLight = boff;
  TSignal = toff;
  
  if (print1 || print2){
    Serial.begin(9600);
  }
  
  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();


  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //
  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  

  // According to the datasheet, the 'sleep' bit
  // should read a '1'. But I read a '0'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. Even if the
  // bit reads '0'.
  error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);


  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

  // Read
   readRawAccelValues('S');
  
  //initialize the raw read values for the accelerometer
  xin = accel_t_gyro.value.x_accel;
  yin = accel_t_gyro.value.y_accel;
  zin = accel_t_gyro.value.z_accel;
  
  //initialize the filtered read values for the accelerometer
  xout = xin;
  yout = yin;
  zout = zin;
  
  //These are the outputs for the brake light and turnsignal test
  pinMode(BRAKE_PIN, OUTPUT); //Brakelight
  pinMode(TURN_PIN, OUTPUT); //Turnsignal
  
}

void loop() {
  // Master Clock to time events
  clock = clock+1;
  if(print2){
    Serial.print("Clock: ");
    Serial.println(clock,DEC);
  }
  // Save Button States
  int rightButtonState;
  int leftButtonState;
  
  // Read Button States
  rightButtonState = digitalRead(rightButton);
  if(print2){
    Serial.print("R button: ");
    Serial.println(rightButtonState,DEC);
  }
  leftButtonState = digitalRead(leftButton);
  if(print2){
    Serial.print("L button: ");
    Serial.println(leftButtonState,DEC);
  }
  if ( rightButtonState == LOW ){
    if (TSignal == right_on){
      TSignal = toff;
    }else{
      TSignal = right_on;
    }
  }
  if ( leftButtonState == LOW){
    if (TSignal == left_on){
      TSignal = toff;
    }else{
      TSignal = left_on;
    }
  }
  
  if ( (TSignal == right_on) & (clock % 3 == 0) ) {
    digitalWrite(rightSignal, HIGH);
  }
  if ( TSignal == left_on  & (clock % 3 == 0) ) {
    digitalWrite(leftSignal, HIGH);
  }
  if ((clock+1) % 3 == 0){
    digitalWrite(rightSignal, LOW);
    digitalWrite(leftSignal, LOW);
  }
  
  // Read
  readRawAccelValues('L');
  
  //updat raw acceleration values
  xin = accel_t_gyro.value.x_accel;
  yin = accel_t_gyro.value.y_accel;
  zin = accel_t_gyro.value.z_accel;
  
  //update filtered acceleration data
  xout = xout + 0.1 * (xin-xout);
  yout = yout + 0.1 * (yin-yout);
  zout = zout + 0.1 * (zin-zout);

  //Determine if bike is decelerating
  //If it is turn on the break light 
  if (zout > threshold){
    BLight = on;
  }
  else{
    BLight = boff;
  }
  //Turn on/off pin 12 as output for break light
  if (BLight == on){
    digitalWrite(BRAKE_PIN, HIGH);
  }
  if (BLight == boff){
    digitalWrite(BRAKE_PIN,LOW);
  }
  
  //If the Turn signal is not off 
  // check if the y is past the threshold and turn off the turn signal
  if (TSignal != toff){
    if (yout > 2*threshold || yout < -2*threshold){
      TSignal = toff;
    }
  }
  
  //Debugging for turn signal
  if (yout > 2*threshold || yout < -2*threshold){
    digitalWrite(TURN_PIN, HIGH);
  }else{
    digitalWrite(TURN_PIN, LOW);
  }
  
  delay(100);

}

// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus.
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read.
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission();    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

//----------------------------------------------------------
// Read Accelerometer 
//----------------------------------------------------------
void readRawAccelValues(char setupOrLoop){
	 // Read the raw values.
	  // Read 14 bytes at once,
	  // containing acceleration, temperature and gyro.
	  // With the default settings of the MPU-6050,
	  // there is no filter enabled, and the values
	  // are not very stable.
	  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));


	  // Swap all high and low bytes.
	  // After this, the registers values are swapped,
	  // so the structure name like x_accel_l does no
	  // longer contain the lower byte.
	  uint8_t swap;
	  #define SWAP(x,y) swap = x; x = y; y = swap

	  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
	  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
	  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
	  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
	  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
	  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
	  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);


	  // Print the raw acceleration values to serial port for monitoring
	  // Only prints in debug mode
	  if (print1) {
                Serial.print("Setup or Loop Function: ");
                Serial.println(setupOrLoop);
		Serial.print(F("x: "));
		Serial.println(accel_t_gyro.value.x_accel, DEC);
		Serial.print(F("y: "));
		Serial.println(accel_t_gyro.value.y_accel, DEC);
		Serial.print(F("z: "));
		Serial.println(accel_t_gyro.value.z_accel, DEC);
		//Serial.println(F(""));
	  }
  }

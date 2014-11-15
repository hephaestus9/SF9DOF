/***************************************************************************************************************
* Razor AHRS Firmware v1.4.2
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
* and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
*
*Edited by J.Brian 10-29-14
*/
#include <Wire.h>

#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)
#define OUTPUT__BAUD_RATE 115200
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds
#define STATUS_LED_PIN PF_2  // Pin number of status LED (Blue LED)


// Sensor I2C addresses
#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int) 0x1E) // 0x1E = 0x3C / 2


#define GYRO_ADDRESS  ((int) 0x68) // 0x68 = 0xD0 / 2
#define WHO_AM_I      ((int) 0x00)
#define SMPLRT_DIV    ((int) 0x15)
#define DLPF_FS       ((int) 0x16)
#define GYRO_XOUT_H   ((int) 0x1D) //Internal x 
#define GYRO_XOUT_L   ((int) 0x1E) 
#define GYRO_YOUT_H   ((int) 0x1F) //Internal y
#define GYRO_YOUT_L   ((int) 0x20)
#define GYRO_ZOUT_H   ((int) 0x21)
#define GYRO_ZOUT_L   ((int) 0x22)


#define WIRE_SEND(b) Wire.write((byte) b) 
#define WIRE_RECEIVE() Wire.read() 

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float magnetom[3];
float gyro[3];

int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;


void setup()
{
  // Init serial output
  //Serial.begin(OUTPUT__BAUD_RATE);
  Serial1.begin(OUTPUT__BAUD_RATE);
  
  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Init sensors
  delay(50);  // Give sensors enough time to start
  init_sensors();
  delay(20);  
}

void loop()
{
  // Time to read the sensors again?
  if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;
  }
    // Update sensor readings
    read_sensors();
    digitalWrite(STATUS_LED_PIN, HIGH);

    Serial1.print(G_Dt);
    Serial1.print(":");
    
    Serial1.print(accel[0]);
    Serial1.print(":");
    Serial1.print(accel[1]);
    Serial1.print(":");
    Serial1.print(accel[2]);
    Serial1.print(":");
    
    Serial1.print(magnetom[0]);
    Serial1.print(":");
    Serial1.print(magnetom[1]);
    Serial1.print(":");
    Serial1.print(magnetom[2]);
    Serial1.print(":");
    
    Serial1.print(gyro[0]);
    Serial1.print(":");
    Serial1.print(gyro[1]);
    Serial1.print(":");
    Serial1.println(gyro[2]);
    digitalWrite(STATUS_LED_PIN, LOW);

    /*Serial.print(G_Dt);
    Serial.print(":");
    
    Serial.print(accel[0]);
    Serial.print(":");
    Serial.print(accel[1]);
    Serial.print(":");
    Serial.print(accel[2]);
    Serial.print(":");
    
    Serial.print(magnetom[0]);
    Serial.print(":");
    Serial.print(magnetom[1]);
    Serial.print(":");
    Serial.print(magnetom[2]);
    Serial.print(":");
    
    Serial.print(gyro[0]);
    Serial.print(":");
    Serial.print(gyro[1]);
    Serial.print(":");
    Serial.println(gyro[2]);*/
}

void init_sensors(){
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
}

void I2C_Init()
{
  Wire.setModule(3);
  Wire.begin();
}

void Accel_Init()
{
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2D);  // Power register
  WIRE_SEND(0x08);  // Measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x31);  // Data format register
  WIRE_SEND(0x08);  // Set to full resolution
  Wire.endTransmission();
  delay(5);
  
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2C);  // Rate
  WIRE_SEND(0x09);  // Set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(ACCEL_ADDRESS); 
  WIRE_SEND(0x32);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    // No multiply by -1 for coordinate system transformation here, because of double negation:
    // We want the gravity vector, which is negated acceleration vector.
    accel[0] = (((int) buff[3]) << 8) | buff[2];  // X axis (internal sensor y axis)
    accel[1] = (((int) buff[1]) << 8) | buff[0];  // Y axis (internal sensor x axis)
    accel[2] = (((int) buff[5]) << 8) | buff[4];  // Z axis (internal sensor z axis)
    
    if (accel[0] > 50000){
      accel[0] = -1 * (65536 - accel[0]);
    }
  
    if (accel[1] > 50000){
      accel[1] = -1 * (65536 - accel[1]);
    }
  
    if (accel[2] > 50000){
      accel[2] = -1 * (65536 - accel[2]);
    }
  }
  else
  {
    num_accel_errors++;
    if (output_errors) Serial.println("!ERR: reading accelerometer");
  }
}

void Magn_Init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02); 
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void Read_Magn()
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
// 9DOF Razor IMU SEN-10125 using HMC5843 magnetometer
#if HW__VERSION_CODE == 10125
    // MSB byte first, then LSB; X, Y, Z
    magnetom[0] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // X axis (internal sensor -y axis)
    magnetom[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
    magnetom[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
// 9DOF Razor IMU SEN-10736 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10736
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // X axis (internal sensor -y axis)
    magnetom[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
    magnetom[2] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
// 9DOF Sensor Stick SEN-10183 and SEN-10321 using HMC5843 magnetometer
#elif (HW__VERSION_CODE == 10183) || (HW__VERSION_CODE == 10321)
    // MSB byte first, then LSB; X, Y, Z
    magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
    magnetom[1] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Y axis (internal sensor -y axis)
    magnetom[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
// 9DOF Sensor Stick SEN-10724 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10724
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
    magnetom[1] = ((((int) buff[4]) << 8) | buff[5]);  // Y axis (internal sensor -y axis)
    magnetom[2] = ((((int) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
    
    if (magnetom[0] > 50000){
      magnetom[0] = -1 * (65536 - magnetom[0]);
    }
  
    if (magnetom[1] > 50000){
      magnetom[1] = -1 * (65536 - magnetom[1]);
    }
  
    if (magnetom[2] > 50000){
      magnetom[2] = -1 * (65536 - magnetom[2]);
    }
    
    magnetom[1] = -1 * magnetom[1];
    magnetom[2] = -1 * magnetom[2];
#endif
  }
  else
  {
    num_magn_errors++;
    if (output_errors) Serial.println("!ERR: reading magnetometer");
  }
}

void Gyro_Init()
{
  // Power up reset defaults
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);
  
  // Select full-scale range of the gyro sensors
  // Set LP filter bandwidth to 42Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x16);
  WIRE_SEND(0x1B);  // DLPF_CFG = 3, FS_SEL = 3
  Wire.endTransmission();
  delay(5);
  
  // Set sample rato to 50Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x15);
  WIRE_SEND(0x0A);  //  SMPLRT_DIV = 10 (50Hz)
  Wire.endTransmission();
  delay(5);

  // Set clock to PLL with z gyro reference
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);
}

// Reads x, y and z gyroscope registers
unsigned char gyroRead(int address, int registerAddress)
{
  //This variable will hold the contents read from the i2c device.
  unsigned char data=0;
  
  //Send the register address to be read.
  Wire.beginTransmission(address);
  //Send the Register Address
  Wire.write(registerAddress);
  //End the communication sequence.
  Wire.endTransmission();
  
  //Ask the I2C device for data
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 1);
  
  //Wait for a response from the I2C device
  if(Wire.available()){
    //Save the data sent from the I2C device
    data = Wire.read();
  }
  
  //End the communication sequence.
  Wire.endTransmission();
  
  //Return the data read during the operation
  return data;
}


void Read_Gyro()
{
    int data = 0;
    data = (gyroRead(GYRO_ADDRESS, GYRO_YOUT_H) << 8);    // X axis (internal sensor -y axis)
    data |= (gyroRead(GYRO_ADDRESS, GYRO_YOUT_L));
    gyro[0] = data;
    
    data = 0;
    data = (gyroRead(GYRO_ADDRESS, GYRO_XOUT_H) << 8);    // Y axis (internal sensor -x axis)
    data |= (gyroRead(GYRO_ADDRESS, GYRO_XOUT_L));
    gyro[1] = data;
    
    data = 0;
    data = (gyroRead(GYRO_ADDRESS, GYRO_ZOUT_H) << 8);    // Z axis (internal sensor -z axis)
    data |= (gyroRead(GYRO_ADDRESS, GYRO_ZOUT_L));
    gyro[2] = data;
    
    if (gyro[0] > 60000){
      gyro[0] = -1 * (65536 - gyro[0]);
    }
  
    if (gyro[1] > 60000){
      gyro[1] = -1 * (65536 - gyro[1]);
    }
  
    if (gyro[2] > 60000){
      gyro[2] = -1 * (65536 - gyro[2]);
    }
    
    gyro[0] = -1 * gyro[0];
    gyro[1] = -1 * gyro[1];
    gyro[2] = -1 * gyro[2];
}

void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}


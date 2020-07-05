#include <math.h>
#include <Wire.h>

#define address 0x1E //0011110b, I2C 7bit address of HMC5883
#define PI 3.14159265358979323846

void setup() {
  //Initialize Serial and I2C communications
  Serial.begin(9600);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

//  Wire.beginTransmission(address); //open communication with HMC5883
//  Wire.write(0x00); //select config a register
//  Wire.write(B00010000); //default
//  Wire.endTransmission();
//
//  Wire.beginTransmission(address); //open communication with HMC5883
//  Wire.write(0x01); //select config b register
//  Wire.write(B00100000); //continuous measurement mode
//  Wire.endTransmission();
}

void loop() {

  int16_t x,y,z; //triple axis data

  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
   z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  double heading = atan2((double)y, (double)x);

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Convert radians to degrees for readability.
  double headingDegrees = heading * 180 / PI;

  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.print(z);
  Serial.print(" Heading: ");
  Serial.print(headingDegrees);
  Serial.print(" / ");
  Serial.println(heading);
  
  //delay(1000);
}

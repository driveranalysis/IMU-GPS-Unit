#define mag_adress 0x1E //0011110b
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_DO 0x63
#define mpuAddress 0x68
#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(mpuAddress);
  Wire.write(I2C_SLV0_ADDR);
  Wire.write(B00011110); // addres of magnetometer, zero at bit 7 means write
  Wire.endTransmission();
  
  Wire.beginTransmission(mpuAddress);
  Wire.write(I2C_SLV0_REG);
  Wire.write(B10000010); //selecting mode register on magnetometer
  Wire.endTransmission();

  Wire.beginTransmission(mpuAddress);
  Wire.write(I2C_SLV0_CTRL);
  Wire.write(B10000110); //continious measurement mode
  Wire.endTransmission();

  Wire.beginTransmission(mpuAddress);
  Wire.write(I2C_SLV0_DO);
  Wire.write(0x00); //continious measurement mode
  Wire.endTransmission();

}

void loop() {
  // put your main code here, to run repeatedly:
  int x,y,z; //triple axis data
  int xmin,xmax,ymin,ymax,zmin,zmax;
  xmin=0; xmax=0; ymax=0; ymin = 0; zmin=0;zmax=0;

 
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x49); //selecting external sensor data register
  Wire.endTransmission();
  Wire.requestFrom(mpuAddress, 6);
  
  if(6<=Wire.available()){
//    x1 = Wire.read()<<8; //X msb
//    x2 = Wire.read(); //X lsb
//    z1 = Wire.read()<<8; //Z msb
//    z2 = Wire.read(); //Z lsb
//    y1 = Wire.read()<<8; //Y msb
//    y2 = Wire.read(); //Y lsb
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }

  //Print out values of each axis
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.println(z); 
//   *mx = (((int16_t)x1) << 8) | x2;
//   *my = (((int16_t)y1) << 8) | y2;
//   *mz = (((int16_t)z1) << 8) | z2;

}

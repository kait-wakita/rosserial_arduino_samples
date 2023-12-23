////////////////////////////////////////////////////////////////////////
// gyro test
////////////////////////////////////////////////////////////////////////
#include <Wire.h>

////////////////////////////////////////////////////////////////////////
////  init
////////////////////////////////////////////////////////////////////////
void setup() {
  gyro_init();
  
  Serial.begin(9600);
}


////////////////////////////////////////////////////////////////////////
////  main loop
////////////////////////////////////////////////////////////////////////

void loop() {
  static unsigned long tm=millis();
  static float delta_t = 0.2;
  static float angle=0;
  
  float yaw_rate = yaw_measure();
  angle += yaw_rate * delta_t;
  
  Serial.print(millis()); Serial.print(" (ms)   ");
  Serial.print(yaw_rate); Serial.print(" (deg/s)   ");
  Serial.print(angle); Serial.println(" (deg)");
  
  while(tm+delta_t*1000>millis());
  tm += delta_t*1000;
}



////////////////////////////////////////////////////////////////////////
//// FUNCTIONS
////////////////////////////////////////////////////////////////////////

////////////////////////
// gyro MPU6050
////////////////////////

// MPU6050 address/register
#define MPU6050_WHO_AM_I     0x75  // Read Only
#define MPU6050_PWR_MGMT_1   0x6B  // Read and Write
#define MPU_ADDRESS  0x68

void gyro_init() {
  // I2C read
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);  //MPU6050_PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission();

  // read mode 
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);  //MPU6050_PWR_MGMT_1レジスタの設定
  Wire.write(0x00);
  Wire.endTransmission();

  // change scale for rotaion
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x10);  //  0x00:250, 0x08:500, 0x10:1000, 0x18:2000deg/s
  Wire.endTransmission();
}


float yaw_measure() {
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, Temperature;

  axRaw = Wire.read() << 8 | Wire.read();
  ayRaw = Wire.read() << 8 | Wire.read();
  azRaw = Wire.read() << 8 | Wire.read();
  Temperature = Wire.read() << 8 | Wire.read();
  gxRaw = Wire.read() << 8 | Wire.read();
  gyRaw = Wire.read() << 8 | Wire.read();
  gzRaw = Wire.read() << 8 | Wire.read();

  acc_x = axRaw / 16384.0;  //FS_SEL_0 16,384 LSB / g
  acc_y = ayRaw / 16384.0;
  acc_z = azRaw / 16384.0;

  gyro_x = gxRaw / 32.8 ;//FS_SEL_0 32.8 LSB / (°/s)
  gyro_y = gyRaw / 32.8 ;
  gyro_z = -gzRaw / 32.8 ;


  gyro_z -= 3.00; // ad hoc bias compensation

  return(gyro_z);
}

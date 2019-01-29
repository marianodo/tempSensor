#include "Wire.h"
#include "math.h"
#include "constants.h"

const int MPU_addr=0x68; // I2C address of the MPU-6050
int cal_int; 
int16_t ax, ay, az, Tmp, gx, gy, gz;
float gx_cal, gy_cal, gz_cal, gyro_roll, gyro_pitch, gyro_yaw, angle_roll, angle_pitch, Temp, angle;
double acc_total_vector, angle_pitch_acc, angle_roll_acc;
unsigned long int d, thisTime, previousTime;
bool set_gyro_angles=false;

void initMPU(){
  Wire.begin(4,5);
 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);                               // PWR_MGMT_1 register
  Wire.write(0x00);                               // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);                               // Registro de configuración del giroscopio
  Wire.write(0x08);                               // FS_SEL=1 ==> +/- 500 º/s
  Wire.endTransmission(true);
  delay(10);
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);                               // Registro de configuración del acelerómetro
  Wire.write(0x10);                               // AFS_SEL=2 ==> +/- 8g
  Wire.endTransmission(true);
  delay(10);
}

void readMPU(){
  
  previousTime=thisTime;
  thisTime=micros();
  d=thisTime - previousTime; 
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);                        // request a total of 14 registers
  while(Wire.available() < 14);
  ax = Wire.read()<<8|Wire.read();                           // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = Wire.read()<<8|Wire.read();                           // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = (Wire.read()<<8|Wire.read()) + Z_ACCEL_OFFSET;        // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = (Wire.read()<<8|Wire.read()) + TEMP_OFFSET;          // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx = Wire.read()<<8|Wire.read();                           // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy = Wire.read()<<8|Wire.read();                           // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz = Wire.read()<<8|Wire.read();                           // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Temp = (float)Tmp/340 + 36.53;
/*
  Serial.print("ax = "); Serial.print(ax);
  Serial.print(" | ay = "); Serial.print(ay);
  Serial.print(" | az = "); Serial.print(az);
  Serial.print(" | Tmp = "); Serial.print(Tmp);
  Serial.print(" | gx = "); Serial.print(gx);
  Serial.print(" | gy = "); Serial.print(gy);
  Serial.print(" | gz = "); Serial.println(gz);
*/

  gyro_pitch = gx-GX_CAL;                                                                 // Valor de la velocidad angular ya calibrada (todavía no está en GRAD/SEG)
  gyro_roll = gy-GY_CAL;                                                                  // Para una velocidad angular máxima de +-500 º/s arrojará un valor máximo de +-32750 (16 bits)
  gyro_yaw = gz-GZ_CAL;                                                                   // Es decir, 32750/500 = 65,5
  
  angle_roll += gyro_roll * (float)d/65500000;                                            // Valor del ángulo calculado por el giroscopio en grados
  angle_pitch += gyro_pitch * (float)d/65500000;                                          // ángulo = velocidad angular (valor crudo)/65,5 * (d en microsegundos/1000000)

  angle_roll -= angle_pitch * sin(gyro_yaw * (float)d/3752873558.106);                  //Valor del ángulo en grados (gyro_yaw en radianes (d/65500000) * (3,142/180))
 // angle_pitch += angle_roll * sin(gyro_yaw * (float)d/3752873558.106);                  //Si de produce un giro alrededor de YAW, el ángulo pitch se transfiere al roll y viceversa

  acc_total_vector=sqrt(((double)ax * (double)ax) + ((double)ay * (double)ay) + ((double)az * (double)az));

 // if(abs(ay) < acc_total_vector){                                                       // Previene que la función asin genere un NaN
 //   angle_pitch_acc= asin((double)(ay)/acc_total_vector) * 57.2958;                      // La función asin devuelve radianes, se multiplica por 57.2958 para pasar a grados  
 //  }

  if(abs(ax) < acc_total_vector){                                                       // Previene que la función asin genere un NaN
    angle_roll_acc= asin((double)(ax)/acc_total_vector) * -57.2958;                      // La función asin devuelve radianes, se multiplica por 57.2958 para pasar a grados 
   }
    
 // angle_pitch_acc -= ERROR_PITCH_ACC;                                                          // Accelerometer calibration value for pitch.
  angle_roll_acc -= ERROR_ROLL_ACC;                                                           // Accelerometer calibration value for roll.
    
  if(set_gyro_angles){
 //   angle_pitch = angle_pitch * 0.93 + angle_pitch_acc * 0.07;
    angle_roll = angle_roll * 0.93 + angle_roll_acc * 0.07; 
   }
  else{
  //  angle_pitch=(float)angle_pitch_acc;
    angle_roll=(float)angle_roll_acc;
      
    set_gyro_angles=true;
   }  
///  Serial.print(angle_pitch_acc); Serial.print(" ");
//  Serial.print(angle_roll_acc); Serial.print(" ");
//  Serial.print("Acc_total_vector= "); Serial.print(acc_total_vector);
//  Serial.print(angle_pitch); Serial.print(" ");
//  Serial.println(angle_roll);
   
}

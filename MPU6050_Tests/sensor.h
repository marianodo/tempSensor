#include "Wire.h"
#include "math.h"

#define TEMP_OFFSET -1600
#define Z_ACCEL_OFFSET 0

const int MPU_addr=0x68; // I2C address of the MPU-6050
int cal_int; 
int16_t ax, ay, az, Tmp, gx, gy, gz;
float gx_cal, gy_cal, gz_cal, gyro_roll, gyro_pitch, gyro_yaw, angle_roll, angle_pitch, Temp;
double acc_total_vector, angle_pitch_acc, angle_roll_acc, error_pitch_acc, error_roll_acc;
unsigned long int d, thisTime, previousTime;
bool set_gyro_angles=false;

byte check_I2c(byte addr);

void initMPU(){
  Wire.begin(4,5);
  check_I2c(MPU_addr); // Check that there is an MPU //TODO This is an usefull method
 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B); // Registro de configuración del giroscopio
  Wire.write(0x08); // FS_SEL=1 ==> +/- 500 º/s
  Wire.endTransmission(true);
  delay(10);
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C); // Registro de configuración del acelerómetro
  Wire.write(0x10); // AFS_SEL=2 ==> +/- 8g
  Wire.endTransmission(true);
  delay(10);
}

void calibrate_gyro(){
  Serial.print("Calibrando Gyro");
  for (cal_int = 0; cal_int < 6000 ; cal_int ++){            //Toma 6000 lecturas para calibración.
        if(cal_int % 100 == 0)Serial.print(F("."));                //Imprime un punto indicando calibración.
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x43); // starting with register 0x43 (GYRO_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr,6,true); // request a total of 14 registers
        while(Wire.available() < 6);
        gx = Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        gy = Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        gz = Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      
        gx_cal += gx;                                              //Ad roll value to gyro_roll_cal.
        gy_cal += gy;                                              //Ad pitch value to gyro_pitch_cal.
        gz_cal += gz;                                              //Ad yaw value to gyro_yaw_cal.

        delay(3);                                                  //Wait 3 milliseconds before the next loop.
      }

    gx_cal /= 6000;                                         //Divide the roll total by 6000.
    gy_cal /= 6000;                                         //Divide the pitch total by 6000.
    gz_cal /= 6000;                                        //Divide the yaw total by 6000.  

    Serial.println("Valores de calibración:");
    Serial.print("gx: ");
    Serial.println(gx_cal);
    Serial.print("gy: ");
    Serial.println(gy_cal);
    Serial.print("gz: ");
    Serial.println(gz_cal);
  
}
void calibrate_accel(){
  Serial.print("Calibrando Accel");
  for (cal_int = 0; cal_int < 6000 ; cal_int ++){                   //Toma 6000 lecturas para calibración.
        if(cal_int % 100 == 0)Serial.print(F("."));                 //Imprime un punto indicando calibración.
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B);                                           // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr,6,true);                          // request a total of 14 registers
        ax = Wire.read()<<8|Wire.read();                            // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        ay = Wire.read()<<8|Wire.read();                            // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        az = (Wire.read()<<8|Wire.read()) + Z_ACCEL_OFFSET;         // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

        acc_total_vector=sqrt(((double)ax * (double)ax) + ((double)ay * (double)ay) + ((double)az * (double)az));

        if(abs(ay) < acc_total_vector){                                                       // Previene que la función asin genere un NaN
          error_pitch_acc += asin((double)(ay)/acc_total_vector) * 57.2958;                   // La función asin devuelve radianes, se multiplica por 57.2958 para pasar a grados  
         }

        if(abs(ax) < acc_total_vector){                                                       // Previene que la función asin genere un NaN
          error_roll_acc += asin((double)(ax)/acc_total_vector) * -57.2958;                   // La función asin devuelve radianes, se multiplica por 57.2958 para pasar a grados 
         }

        delay(3);                                                                             //Wait 3 milliseconds before the next loop.
      }
  
    error_pitch_acc /= 6000;                                        //Divide the roll total by 6000.
    error_roll_acc /= 6000;                                         //Divide the pitch total by 6000.

    Serial.println("Valores de calibración:");
    Serial.print("error_pitch_acc: ");
    Serial.println(error_pitch_acc);
    Serial.print("error_roll_acc: ");
    Serial.println(error_roll_acc);
}

void readMPU(){
  previousTime=thisTime;
  thisTime=micros();
  d=thisTime - previousTime; 

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers

  while(Wire.available() < 14);
  ax = Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = (Wire.read()<<8|Wire.read()) + Z_ACCEL_OFFSET; // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = (Wire.read()<<8|Wire.read()) + TEMP_OFFSET; //-1600 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx = Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy = Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz = Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Temp = (float)Tmp/340 + 36.53;
/*
  Serial.print("ax = "); Serial.print(ax);
  Serial.print("ay = "); Serial.print(ay);
  Serial.print("az = "); Serial.print(az);
  Serial.print("Tmp = "); Serial.println(Tmp);
  Serial.print("Temp = "); Serial.println(Temp);
  Serial.print("gx = "); Serial.print(gx);
  Serial.print("gy = "); Serial.print(gy);
  Serial.print("gz = "); Serial.println(gz);
*/
  gyro_pitch = gx-gx_cal;                                                                 // Valor de la velocidad angular ya calibrada (todavía no está en GRAD/SEG)
  gyro_roll = gy-gy_cal;                                                                  // Para una velocidad angular máxima de +-500 º/s arrojará un valor máximo de +-32750 (16 bits)
  gyro_yaw = gz-gz_cal;                                                                   // Es decir, 32750/500 = 65,5
  
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
    
  //  angle_pitch_acc -= error_pitch_acc;                                                          // Accelerometer calibration value for pitch.
    angle_roll_acc -= error_roll_acc;                                                           // Accelerometer calibration value for roll.
    
  if(set_gyro_angles){
   // angle_pitch = angle_pitch * 0.93 + angle_pitch_acc * 0.07;
    angle_roll = angle_roll * 0.93 + angle_roll_acc * 0.07; 
   }
  else{
  //  angle_pitch=(float)angle_pitch_acc;
    angle_roll=(float)angle_roll_acc;
      
    set_gyro_angles=true;
   } 
   
//  Serial.print(ax); Serial.print(" ");
//  Serial.print(ay); Serial.print(" ");
//  Serial.print(az); Serial.print(" ");
//  Serial.print(angle_pitch_acc); Serial.print(" ");
//  Serial.print(angle_roll_acc); Serial.print(" ");
//  Serial.print("Acc_total_vector= "); Serial.print(acc_total_vector);
//  Serial.print(angle_pitch); Serial.print(" ");
  Serial.println(angle_roll); Serial.print(" ");
}

byte check_I2c(byte addr){
  // We are using the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  
  Wire.beginTransmission(addr);
  const byte error = Wire.endTransmission();
   
  if (error == 0){
    Serial.print(" Device Found at 0x");
    Serial.println(addr,HEX);
  }
  else{
    Serial.print(" No Device Found at 0x");
    Serial.println(addr,HEX);
  }
  return Wire.endTransmission();
}

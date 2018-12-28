#include "Wire.h"
#include "math.h"

const int MPU_addr=0x68; // I2C address of the MPU-6050
int cal_int; 
int16_t ax, ay, az, Tmp, gx, gy, gz;
float gx_cal, gy_cal, gz_cal, ax_cal, ay_cal, az_cal, gyro_roll, gyro_pitch, gyro_yaw, angle_roll, angle_pitch, angle_yaw, Temp, angle;
double acc_total_vector, angle_pitch_acc, angle_roll_acc;
unsigned long int ti, tf,d;
bool set_gyro_angles;

//byte check_I2c(byte addr);

void initMPU(){
  Wire.begin(4,5);
  //check_I2c(MPU_addr); // Check that there is an MPU //TODO This is an usefull method
 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B); // Registro de configuración del giroscopio
  Wire.write(0x08); // FS_SEL=1 ==> +/- 500 º/s
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C); // Registro de configuración del acelerómetro
  Wire.write(0x10); // AFS_SEL=2 ==> +/- 8g
  Wire.endTransmission(true);
}

void calibrateMPU(){
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){            //Toma 2000 lecturas para calibración.
        if(cal_int % 100 == 0)Serial.print(F("."));                //Imprime un punto indicando calibración.
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
        ax = Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        ay = Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        az = Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        Tmp = Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
        gx = Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        gy = Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        gz = Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      
        gx_cal += gx;                                              //Ad roll value to gyro_roll_cal.
        gy_cal += gy;                                              //Ad pitch value to gyro_pitch_cal.
        gz_cal += gz;                                              //Ad yaw value to gyro_yaw_cal.

        ax_cal += ax;
        ay_cal += ay;
        az_cal += az;

        delay(3);                                                  //Wait 3 milliseconds before the next loop.
      }
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gx_cal /= 2000;                                       //Divide the roll total by 2000.
    gy_cal /= 2000;                                      //Divide the pitch total by 2000.
    gz_cal /= 2000;                                        //Divide the yaw total by 2000.  

    ax_cal /= 2000;                                       //Divide the roll total by 2000.
    ay_cal /= 2000;                                      //Divide the pitch total by 2000.
    az_cal /= 2000;                                        //Divide the yaw total by 2000.  
  
}

void readMPU(){
  int i;
  for(i=0; i<3; i++){ //What is 3??
  ti=micros();
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  ax = Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx = Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy = Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz = Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

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

  gyro_pitch = gx-gx_cal;                                                                 // Valor de la velocidad angular ya calibrada (todavía no está en GRAD/SEG)
  gyro_roll = gy-gy_cal;                                                                  // Para una velocidad angular máxima de +-500 º/s arrojará un valor máximo de +-32750 (16 bits)
  gyro_yaw = gz-gz_cal;                                                                   // Es decir, 32750/500 = 65,5
  
  angle_roll += gyro_roll * (float)d/65500000;                                            // Valor del ángulo calculado por el giroscopio en grados
  angle_pitch += gyro_pitch * (float)d/65500000;                                          // ángulo = velocidad angular (valor crudo)/65,5 * (d en microsegundos/1000000)
  angle_yaw += gyro_yaw * (float)d/65500000;

  angle_roll -= angle_pitch * sin(gyro_yaw * (float)d/3752873558.106);                  //Valor del ángulo en grados (gyro_yaw en radianes (d/65500000) * (3,142/180))
  angle_pitch += angle_roll * sin(gyro_yaw * (float)d/3752873558.106);                  //Si de produce un giro alrededor de YAW, el ángulo pitch se transfiere al roll y viceversa

  acc_total_vector=sqrt(((float)ax * (float)ax) + ((float)ay * (float)ay) + ((float)az * (float)az));

  if(abs(ay) < acc_total_vector){                                                       // Previene que la función asin genere un NaN
    angle_pitch_acc= asin((float)(ay)/acc_total_vector) * 57.2958;                      // La función asin devuelve radianes, se multiplica por 57.2958 para pasar a grados  
   }

  if(abs(ax) < acc_total_vector){                                                       // Previene que la función asin genere un NaN
    angle_roll_acc= asin((float)(ax)/acc_total_vector) * -57.2958;                      // La función asin devuelve radianes, se multiplica por 57.2958 para pasar a grados 
   }
    
//    angle_pitch_acc -= ay_cal;                                                          // Accelerometer calibration value for pitch.
//    angle_roll_acc -= ax_cal;                                                           // Accelerometer calibration value for roll.
    
  if(set_gyro_angles){
    angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;
    angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02; 
   }
  else{
    angle_pitch=angle_pitch_acc;
    angle_roll=angle_roll_acc;
      
    set_gyro_angles=true;
   }  
//  Serial.print(gx); Serial.print(" ");
//  Serial.print("angle pitch = "); Serial.print(angle_pitch);
//  Serial.print(" ");
//  Serial.print("angle roll = "); Serial.println(angle_roll);
//  Serial.print(angle_pitch_acc); Serial.print(" ");
//  Serial.print(angle_roll_acc); Serial.print(" ");
//  Serial.print(acc_total_vector); Serial.print(" ");
//  Serial.print("Temp: "); Serial.print(Temp); Serial.print(" ");
  //Serial.print("Pitch: "); Serial.print(angle_pitch); Serial.print(" ");
//  Serial.print("Roll: "); Serial.println(angle_roll);
/*  Serial.print("Yaw: "); Serial.println(angle_yaw); 
    

  Serial.print("ax = "); Serial.print(ax);
  Serial.print(" | ay = "); Serial.print(ay);
  Serial.print(" | az = "); Serial.print(az);
  Serial.print(" | Tmp = "); Serial.print(Tmp);
  Serial.print(" | gx = "); Serial.print(gx);
  Serial.print(" | gy = "); Serial.print(gy);
  Serial.print(" | gz = "); Serial.println(gz);
*/  
 // delay(300); // Wait 0.5 seconds and scan again
  tf=micros(); 
  d=tf-ti;
  }
}
/*byte check_I2c(byte addr){
  // We are using the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  
  Wire.beginTransmission(addr);
  //const byte error = Wire.endTransmission();
   
  if (error == 0){
    Serial.print(" Device Found at 0x");
    Serial.println(addr,HEX);
  }
  else{
    Serial.print(" No Device Found at 0x");
    Serial.println(addr,HEX);
  }
  return Wire.endTransmission();
}*/

#define PARAM_FOCAL_LENGTH_MM 16
#define REG_GYRO_PROC_XY   0x5C
#define REG_GYRO_PROC_Z    0x5D
#define REG_ACCEL_PROC_XY  0x5E
#define REG_ACCEL_PROC_Z   0x5F
#define REG_ACCEL_PROC_Z   0x5F
#define REG_EULER_PHI_THETA 0x62
#define REG_EULER_PSI 0x63
#define GYRO_SCALE_FACTOR  0.0610352
#define ACCEL_SCALE_FACTOR  0.000183105
#define EULER_SCALE_FACTOR  0.0109863
#define PT_HAS_DATA        0x80  //10000000
#define PT_IS_BATCH        0x40  //01000000
#define PT_BATCH_LEN_XY       0x08  //1000 Batch length = 2
#define PT_BATCH_LEN_Z       0x04  //0100 Batch length = 1

struct G{
  int tams[9];
  char datos[9][20];
}GPS[6],GPS_envio[6];//0RMC/8datos-1GSV/4datos-2VTG/4datos-3GGA/5datos-4GSA/3datos

struct IMU{ 
    int16_t x;
    int16_t y;
    int16_t z;
    double x_imp;
    double y_imp;
    double z_imp;
}gyro,accel;

struct IMU2{ 
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    double roll_imp;
    double pitch_imp;
    double yaw_imp;
}euler;

char dato=' ',ref[5];
uint8_t batch_length;
uint8_t reg;
int actual,ant,c_data_xy,c_data_z;
double acum_x,acum_y,acum_z,gan_x,gan_y,gan_z;

int cont;


void setup()
{
 Serial.begin(115200);  
 Serial2.begin(115200);
 ant=0;
 gan_x=0;
 gan_y=0;
 gan_z=0;
 acum_x=0;
 acum_y=0;
 acum_z=0;
 c_data_xy=0;
 c_data_z=0;
 actual=0;
 
 cont=0;
}

void loop()
{
  while(1){
    do{
      poll_UM6_gyro_XY();
      read_UM6();
    }while(read_UM6==false);
    do{
      poll_UM6_gyro_Z();
      read_UM6();
    }while(read_UM6==false);
    do{
      poll_UM6_accel_XY();
      read_UM6();
    }while(read_UM6==false);
    do{
      poll_UM6_accel_Z();
      read_UM6();
    }while(read_UM6==false);
    do{
      poll_UM6_euler_phi_theta();
      read_UM6();
    }while(read_UM6==false);
    do{
      poll_UM6_euler_psi();
      read_UM6();
    }while(read_UM6==false);
    actual=millis();
    if((actual%10)==0 && actual!=0 && actual!=ant)
    {
      Serial.print(gyro.x_imp);
      Serial.print(",");
      Serial.print(gyro.y_imp);
      Serial.print(",");
      Serial.print(gyro.z_imp);
      Serial.print(",");
      Serial.print(accel.x_imp,8);
      Serial.print(",");
      Serial.print(accel.y_imp,8);
      Serial.print(",");
      Serial.print(accel.z_imp,8);
      Serial.print(",");
      Serial.print(euler.roll_imp,3);
      Serial.print(",");
      Serial.print(euler.pitch_imp,3);
      Serial.print(",");
      Serial.println(euler.yaw_imp,3);
      ant=actual;
    }
  }
}

void poll_UM6_gyro_XY(){
  byte chksum0 = 0, chksum1 = 0;
  unsigned int chksum = 0; 
  chksum = 's' + 'n' + 'p' + (PT_IS_BATCH | PT_BATCH_LEN_XY) + REG_GYRO_PROC_XY; 
  chksum1 = chksum >> 8;
  chksum0 = chksum & 0xFF;
  Serial2.write('s');
  Serial2.write('n');
  Serial2.write('p');
  Serial2.write(PT_IS_BATCH | PT_BATCH_LEN_XY);
  Serial2.write(REG_GYRO_PROC_XY);
  Serial2.write(chksum1);
  Serial2.write(chksum0);
}

void poll_UM6_gyro_Z(){
  byte chksum0 = 0, chksum1 = 0;
  unsigned int chksum = 0; 
  chksum = 's' + 'n' + 'p' + (PT_IS_BATCH | PT_BATCH_LEN_Z) + REG_GYRO_PROC_Z; 
  chksum1 = chksum >> 8;
  chksum0 = chksum & 0xFF;
  Serial2.write('s');
  Serial2.write('n');
  Serial2.write('p');
  Serial2.write(PT_IS_BATCH | PT_BATCH_LEN_Z);
  Serial2.write(REG_GYRO_PROC_Z);
  Serial2.write(chksum1);
  Serial2.write(chksum0);
} 

void poll_UM6_accel_XY(){
  byte chksum0 = 0, chksum1 = 0;
  unsigned int chksum = 0; 
  chksum = 's' + 'n' + 'p' + (PT_IS_BATCH | PT_BATCH_LEN_XY) + REG_ACCEL_PROC_XY; 
  chksum1 = chksum >> 8;
  chksum0 = chksum & 0xFF;
  Serial2.write('s');
  Serial2.write('n');
  Serial2.write('p');
  Serial2.write(PT_IS_BATCH | PT_BATCH_LEN_XY);
  Serial2.write(REG_ACCEL_PROC_XY);
  Serial2.write(chksum1);
  Serial2.write(chksum0);
}

void poll_UM6_accel_Z(){
  byte chksum0 = 0, chksum1 = 0;
  unsigned int chksum = 0; 
  chksum = 's' + 'n' + 'p' + (PT_IS_BATCH | PT_BATCH_LEN_Z) + REG_ACCEL_PROC_Z; 
  chksum1 = chksum >> 8;
  chksum0 = chksum & 0xFF;
  Serial2.write('s');
  Serial2.write('n');
  Serial2.write('p');
  Serial2.write(PT_IS_BATCH | PT_BATCH_LEN_Z);
  Serial2.write(REG_ACCEL_PROC_Z);
  Serial2.write(chksum1);
  Serial2.write(chksum0);
}

void poll_UM6_euler_phi_theta(){
  byte chksum0 = 0, chksum1 = 0;
  unsigned int chksum = 0; 
  chksum = 's' + 'n' + 'p' + (PT_IS_BATCH | PT_BATCH_LEN_XY) + REG_EULER_PHI_THETA; 
  chksum1 = chksum >> 8;
  chksum0 = chksum & 0xFF;
  Serial2.write('s');
  Serial2.write('n');
  Serial2.write('p');
  Serial2.write(PT_IS_BATCH | PT_BATCH_LEN_XY);
  Serial2.write(REG_EULER_PHI_THETA);
  Serial2.write(chksum1);
  Serial2.write(chksum0);
}

void poll_UM6_euler_psi(){
  byte chksum0 = 0, chksum1 = 0;
  unsigned int chksum = 0; 
  chksum = 's' + 'n' + 'p' + (PT_IS_BATCH | PT_BATCH_LEN_Z) + REG_EULER_PSI; 
  chksum1 = chksum >> 8;
  chksum0 = chksum & 0xFF;
  Serial2.write('s');
  Serial2.write('n');
  Serial2.write('p');
  Serial2.write(PT_IS_BATCH | PT_BATCH_LEN_Z);
  Serial2.write(REG_EULER_PSI);
  Serial2.write(chksum1);
  Serial2.write(chksum0);
}

boolean read_UM6(){ 
  unsigned int c = 0;
  uint8_t data[16] = {0};
  unsigned long data_sum = 0; 
  byte blank = 0, temp = 0;
  byte chksum1 = 0, chksum0 = 0;
  unsigned int chksum = 0;
  
  if ((Serial2.available() > 0)){
    c = Serial2.read();
    if ((c == 's')){
      c = Serial2.read();
      if ((c == 'n')){
        c = Serial2.read();
        if ((c == 'p')) {
          c = Serial2.read();
          batch_length=(c>>2)& 0x0F;
          if ((c == (PT_HAS_DATA | PT_IS_BATCH | PT_BATCH_LEN_XY)) || (c == (PT_HAS_DATA | PT_IS_BATCH | PT_BATCH_LEN_Z))) {
            c = Serial2.read();
            reg=c;
            if ((c == REG_GYRO_PROC_XY) || (c == REG_GYRO_PROC_Z) || (c == REG_ACCEL_PROC_XY) || (c == REG_ACCEL_PROC_Z) || (c == REG_EULER_PHI_THETA) || (c == REG_EULER_PSI))  {
              for (byte i = 0; i < batch_length*4; i++)  {
                data[i] = Serial2.read();
                data_sum += data[i];
              }
              chksum1 = Serial2.read();
              chksum0 = Serial2.read(); 
              chksum = (chksum1 << 8) | chksum0;
             if(reg==REG_GYRO_PROC_XY){  //para saber si es el registro XY
                if (chksum == ('s' + 'n' + 'p' + (PT_HAS_DATA | PT_IS_BATCH | PT_BATCH_LEN_XY) + REG_GYRO_PROC_XY + data_sum)){
                  gyro.x = (data[1] | (data[0] << 8));
                  gyro.y = (data[3] | (data[2] << 8));
                  gyro.x_imp = gyro.x * GYRO_SCALE_FACTOR;
                  gyro.y_imp = gyro.y * GYRO_SCALE_FACTOR;
                  if(actual>1000 && actual<10000)
                  {
                    c_data_xy++;
                    acum_x+=gyro.x_imp;
                    acum_y+=gyro.y_imp;
                    gan_x=acum_x/c_data_xy;
                    gan_y=acum_y/c_data_xy;
                  }
                  return true;
                }
                else {
                  
                  return false;
                }
              }
              else if(reg==REG_GYRO_PROC_Z){
                   if (chksum == ('s' + 'n' + 'p' + (PT_HAS_DATA | PT_IS_BATCH | PT_BATCH_LEN_Z) + REG_GYRO_PROC_Z + data_sum)){
                    gyro.z = (data[1] | (data[0] << 8));
                    gyro.z_imp = gyro.z * GYRO_SCALE_FACTOR;
                    return true;
                   }
                   else {
                    
                    return false;
                   }
                  }
                  else if(reg==REG_ACCEL_PROC_XY){  //para saber si es el registro XY
                       if (chksum == ('s' + 'n' + 'p' + (PT_HAS_DATA | PT_IS_BATCH | PT_BATCH_LEN_XY) + REG_ACCEL_PROC_XY + data_sum)){
                        accel.x = (data[1] | (data[0] << 8));
                        accel.y = (data[3] | (data[2] << 8));
                        accel.x_imp = accel.x * ACCEL_SCALE_FACTOR;
                        accel.y_imp = accel.y * ACCEL_SCALE_FACTOR;
                        return true;
                       }
                       else {
                        
                        return false;
                       }
                      }
                      else if(reg==REG_ACCEL_PROC_Z){
                           if (chksum == ('s' + 'n' + 'p' + (PT_HAS_DATA | PT_IS_BATCH | PT_BATCH_LEN_Z) + REG_ACCEL_PROC_Z + data_sum)){
                            accel.z = (data[1] | (data[0] << 8));
                            accel.z_imp = accel.z * ACCEL_SCALE_FACTOR;
                            return true;
                           }
                           else {
                            
                            return false;
                           }
                          }
                          else if(reg==REG_EULER_PHI_THETA){  
                               if (chksum == ('s' + 'n' + 'p' + (PT_HAS_DATA | PT_IS_BATCH | PT_BATCH_LEN_XY) + REG_EULER_PHI_THETA + data_sum)){
                                euler.roll = (data[1] | (data[0] << 8));
                                euler.pitch = (data[3] | (data[2] << 8));
                                euler.roll_imp = euler.roll * EULER_SCALE_FACTOR;
                                euler.pitch_imp = euler.pitch * EULER_SCALE_FACTOR;
                                return true;
                               }
                               else {
                                
                                return false;
                               }
                              }
                              else if(reg==REG_EULER_PSI){
                                   if (chksum == ('s' + 'n' + 'p' + (PT_HAS_DATA | PT_IS_BATCH | PT_BATCH_LEN_Z) + REG_EULER_PSI + data_sum)){
                                    euler.yaw = (data[1] | (data[0] << 8));
                                    euler.yaw_imp = euler.yaw * EULER_SCALE_FACTOR;
                                    return true;
                                   }
                                   else {
                                    
                                    return false;
                                   }
                                  }
            }
            else  {
             return false;
            }
          }
          else  {
           return false; 
          }
        }
        else  {
         return false;
        }
      }
      else  {
       return false; 
      }
    }
    else {
     return false;
    }
  }
  else if (Serial2.available() == 0)  {
    return false;
  }
  else  {
    return false;
  }
}

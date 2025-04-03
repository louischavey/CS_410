#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>

// pitch (y axis or axis perpendicular to nose): direction of ports is + and opposite is -
// roll (x axis)
//quad02
//gcc -o week1 week_1_student.cpp -lwiringPi  -lm


int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();

//global variables
int accel_address,gyro_address;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //accel xyz,  gyro xyz, 
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
#define RAW_TO_GS 10922.667
#define RAW_TO_DEG 32.768

 
int main (int argc, char *argv[])
{

    setup_imu();
    calibrate_imu();    

    float sum[3] = {0.0, 0.0, 0.0};
    
    while(1)
    {

      
    // for (size_t i = 0; i < 1000; i++) {
    //   read_imu();
    //   sum[0] += imu_data[3];
    //   sum[1] += imu_data[4];
    // }
    // sum[0] /= 1000.0f;
    // sum[1] /= 1000.0f;
    // printf("%f %f %f\n", sum[0], sum[1]);
      read_imu();    
      
    }
  
}

// void find_avg() {  // 1: x, 2:, y, 3: z
//   printf("in find avg");
//   float sum[3] = {0.0, 0.0, 0.0};
//   for (size_t i = 0; i < 1000; i++) {
//     read_imu();
//     sum[0] += imu_data[3];
//     sum[1] += imu_data[4];
//     sum[2] += imu_data[5];
//   }
//   sum[0] /= 1000.0f;
//   sum[1] /= 1000.0f;
//   sum[2] /= 1000.0f;
//   printf("%f %f %f\n", sum[0], sum[1], sum[2]);
// }

void calibrate_imu()  // note that we calibrate the angles not, the accelerometer readings
{
 
  
  x_gyro_calibration=-0.197174;
  y_gyro_calibration=0.236969;
  z_gyro_calibration=0.138458;
  roll_calibration= -0.122711;
  pitch_calibration=0.668482;
  // accel_z_calibration=??
  
  printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);

}

void read_imu()
{
  uint8_t address=0;//todo: set address value for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh=0;
  int vl=0;
  int vw=0;


  //accel reads: x, y, z

  address=0x12;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[0]=vw/RAW_TO_GS;//convert to g's 
  
  address=0x14;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[1]=vw/RAW_TO_GS;//convert to g's  
  
  address=0x16;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement     
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]=vw/RAW_TO_GS;//convert to g's  
  
  
     

  //gyro reads

  address=0x02;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement          
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[3]=(vw/RAW_TO_DEG) - x_gyro_calibration;//convert to degrees/sec
  
  address=0x04;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);    
  //convert from 2's complement              
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4]=(vw/RAW_TO_DEG) - y_gyro_calibration;//convert to degrees/sec
  
  address=0x06;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement               
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]=(vw/RAW_TO_DEG) - z_gyro_calibration;//convert to degrees/sec  

  // Calculate pitch and roll
  pitch_angle = atan2(imu_data[1], imu_data[0]) * 180.0/3.14159;
  pitch_angle -= pitch_calibration;
  roll_angle = atan2(imu_data[2], imu_data[0]) * 180.0/3.14159;
  roll_angle -= roll_calibration;

  // printf("%10.5f %10.5f %10.5f %10.5f %10.5f %10.5f\n", imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5]);
  printf("%10.5f %10.5f %10.5f %10.5f %10.5f\n", imu_data[3], imu_data[4], imu_data[5], pitch_angle, roll_angle);
}


int setup_imu()
{
  wiringPiSetup ();
  
  //setup imu on I2C
  accel_address=wiringPiI2CSetup (0x19) ; 
  
  
  gyro_address=wiringPiI2CSetup (0x69) ; 
  
  if(accel_address==-1)
  {
    printf("-----cant connect to accel I2C device %d --------\n",accel_address);
    return -1;
  }
  else if(gyro_address==-1)
  {
    printf("-----cant connect to gyro I2C device %d --------\n",gyro_address);
    return -1;
  }
  else
  {
    printf("all i2c devices detected\n");
    sleep(1);
    wiringPiI2CWriteReg8(accel_address, 0x7d, 0x04); //power on accel    
    wiringPiI2CWriteReg8(accel_address, 0x41, 0x00); //accel range to +_3g    
    wiringPiI2CWriteReg8(accel_address, 0x40, 0x89); //high speed filtered accel
    printf("finished accel");
    wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00);//power on gyro
    wiringPiI2CWriteReg8(gyro_address, 0x0F, 0x01);//set gyro to +-1000dps
    wiringPiI2CWriteReg8(gyro_address, 0x10, 0x03);//set data rate and bandwith
    
    
    sleep(1);
  }
  printf("finished setup");
  return 0;
}



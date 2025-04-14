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

#define RAW_TO_GS 10922.667
#define RAW_TO_DEG 32.768
#define ALPHA 0.02
#define ROLL_BOUND 45
#define PITCH_BOUND 45
#define GYRO_BOUND 300
#define TIMEOUT 0.35
#define MAX_ITERS 10000  // for data collection

// pitch (y axis or axis perpendicular to nose): direction of ports is + and opposite is -
// roll (x axis)
//quad02
// gcc -o week1_student week1_student.cpp -lwiringPi -lm
// gcc -o udp_rx udp_rx.cpp -lwiringPi -lm
// scp C:\Users\jarmi\CS_410\week1_student.cpp pi@10.42.0.1:/home/pi/flight_controller/week1_student.cpp

struct Joystick   // struct to hold joystick data
{
  int key0;       // A
  int key1;       // B
  int key2;       // X
  int key3;       // Y
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num;
};


int check_end_conditions(Joystick joystick_data, long tstart);
void calibrate_imu();      
void read_imu();    
void update_filter();
void to_csv(float *arr, int rows, int cols);
int setup_imu();
void setup_joystick();
void trap(int signal);

//global variables
int accel_address,gyro_address;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //accel xyz,  gyro xyz,
float roll_gyro_delta=0;
float pitch_gyro_delta=0;
float roll_filter=0;
float pitch_filter=0;
float filter_plot[MAX_ITERS][6];  // first 3 cols roll, second 3 are pitch
float intl_roll=0;
float intl_pitch=0;
int acc_data_raw[3];// Raw acc data for debug
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_accel=0;
float roll_accel=0;
int iteration=0;
Joystick* shared_memory;
int run_program=1;

 
int main (int argc, char *argv[])
{
    // Setup peripherals
    setup_imu();
    calibrate_imu();    
    setup_joystick();
    signal(SIGINT, &trap);

    // Variables for controller timeout
    int prev_seq_num = shared_memory->sequence_num;
    long timeout_start;

    while(run_program)
    {
      // Get most recent joystick data
      Joystick joystick_data=*shared_memory;

      // If sequence number has changed, restart timeout
      if (joystick_data.sequence_num != prev_seq_num) {
        // get current time in seconds
        timespec_get(&te,TIME_UTC);
        timeout_start=te.tv_sec;
      }

      // Read and filter IMU data
      read_imu(); 
      update_filter();

      printf("0: %10d 1: %10d 2: %10d 3: %10d pitch: %10d roll: %10d yaw: %10d thrust: %10d seq_num: %10d\n\r", joystick_data.key0, joystick_data.key1, joystick_data.key2, joystick_data.key3, joystick_data.pitch, joystick_data.roll, joystick_data.yaw, joystick_data.thrust, joystick_data.sequence_num);
      // printf("gyro_x: %10.5f gyro_y: %10.5f gyro_z: %10.5f roll: %10.5f pitch: %10.5f\n\r", imu_data[3], imu_data[4], imu_data[5], roll_accel, pitch_accel);
      // printf("roll_filter: %10.5f pitch_filter: %10.5f\n\r", roll_filter, pitch_filter);

      run_program = check_end_conditions(joystick_data, timeout_start);
    }

    return 0;
}

//
// check_end_conditions
//
// Check all stopping conditions, and exit if any are true. Report
// reason why before exiting.
//
int check_end_conditions(Joystick joystick_data, long tstart) {
  if(fabs(imu_data[3] > GYRO_BOUND) || fabs(imu_data[4] > GYRO_BOUND) || fabs(imu_data[5] > GYRO_BOUND)) {     // gyro speed out of bounds
    printf("EXIT: gyro speed out of safe bounds (+/- %d deg/sec)\n\r", GYRO_BOUND);
    return 0;
  }

  if (fabs(roll_filter) > ROLL_BOUND) {                     // roll angle out of bounds
    printf("EXIT: roll angle out of safe bounds (+/- %d deg)\n\r", ROLL_BOUND);
    return 0;
  }

  if (fabs(pitch_filter) > PITCH_BOUND) {                    // pitch angle out of bounds
    printf("EXIT: pitch angle out of bounds (+/- %d deg)\n\r", PITCH_BOUND);
    return 0;
  }

  if(joystick_data.key1) {                          // manual exit (B button)
    printf("EXIT: manual exit (B button)\n\r");
    return 0;
  }

  // get current time in seconds
  timespec_get(&te,TIME_UTC);
  long tcurr = te.tv_sec;
  if (tcurr - tstart >= TIMEOUT) {                  // controller timeout (if sequence number hasn't change for TIMEOUT seconds)
    printf("EXIT: controller timeout (%f seconds)\n\r", TIMEOUT);
    return 0;
  }

  return run_program;   // return 'run_program' instead of '1' to detect ctrl-c force quit
}

//////////////////////////////////////////////
// IMU Functions
//////////////////////////////////////////////

//
// update_filter
//
// Update pitch and roll filters for an accurate and noise free measurement
//
void update_filter()
{
  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;
  // calculate roll_gyro_delta
  // integrate y-axis for roll
  roll_gyro_delta = (imu_data[4] * imu_diff);
  // integrate z-axis for pitch
  intl_roll += roll_gyro_delta;
  pitch_gyro_delta = (imu_data[5] * imu_diff);
  intl_pitch += pitch_gyro_delta;
  // roll = roll_accel * A + (1-A) * (roll_gyro_delta + Rollt-1)
  roll_filter = roll_accel * ALPHA + ((1-ALPHA) * (roll_gyro_delta + roll_filter));
  pitch_filter = pitch_accel * ALPHA + ((1-ALPHA) * (pitch_gyro_delta + pitch_filter));

  // write to data array
  filter_plot[iteration][0] = roll_accel;
  filter_plot[iteration][1] = intl_roll;
  filter_plot[iteration][2] = roll_filter;
  filter_plot[iteration][3] = pitch_accel;
  filter_plot[iteration][4] = intl_pitch;
  filter_plot[iteration][5] = pitch_filter;
}

//
// calibrate_imu
//
// Ensure gyro, roll, and pitch values are initialized to zero at
// starting configuration of IMU
//
void calibrate_imu()  // note that we calibrate the angles not, the accelerometer readings
{
  // get average stationary value over 1000 samples
  float sum[5] = {0.0, 0.0, 0.0, 0.0, 0.0};   // gyro_x, gyro_y, gyro_z, roll, pitch
  for (size_t i = 0; i < 1000; i++) { 
    read_imu();
    sum[0] += imu_data[3];
    sum[1] += imu_data[4];
    sum[2] += imu_data[5];
    sum[3] += roll_accel;
    sum[4] += pitch_accel;
  }
  
  x_gyro_calibration = sum[0] / 1000.0f;
  y_gyro_calibration = sum[1] / 1000.0f;
  z_gyro_calibration = sum[2] / 1000.0f;
  roll_calibration = sum[3] / 1000.0f;
  pitch_calibration = sum[4] / 1000.0f;
  // accel_z_calibration=??
  
  printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);

}

//
// read_imu
//
// Read raw accel/gyro values, convert to units, and calculate roll, pitch angles
//
void read_imu()
{
  uint8_t address=0;//todo: set address value for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh=0;
  int vl=0;
  int vw=0;

  //
  // accel reads: x, y, z
  //

  address=0x12;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);     // accel_x
  acc_data_raw[0] = vw; // store raw value for debug
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[0]=vw/RAW_TO_GS;//convert to g's 
  
  address=0x14;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);     // accel_y
  acc_data_raw[1] = vw;
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[1]=vw/RAW_TO_GS;//convert to g's  
  
  address=0x16;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);     // accel_z
  acc_data_raw[2] = vw;
  //convert from 2's complement     
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]=vw/RAW_TO_GS;//convert to g's  
     
  //
  // gyro reads: x, y, z
  //

  address=0x02;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);      // gyro_x
  //convert from 2's complement          
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[3]=(vw/RAW_TO_DEG) - x_gyro_calibration; // convert to degrees/sec
  
  address=0x04;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);      // gyro_y
  //convert from 2's complement              
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4]=(vw/RAW_TO_DEG) - y_gyro_calibration; 
  
  address=0x06;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);      // gyro_z
  //convert from 2's complement               
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]=-((vw/RAW_TO_DEG) - z_gyro_calibration); 

  // calculate pitch and roll and convert from rad to deg
  pitch_accel = atan2(imu_data[1], imu_data[0]) * 180.0/3.14159;
  pitch_accel -= pitch_calibration;
  roll_accel = atan2(imu_data[2], imu_data[0]) * 180.0/3.14159;
  roll_accel -= roll_calibration;

  // printf("accx: %10.5f accy: %10.5f accz: %10.5f gyrx: %10.5f gyry: %10.5f gyrz: %10.5f roll: %10.5f pitch: %10.5f\n\r", imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5], roll_accel, pitch_accel);
  // printf("accx_raw: %10d accy_raw: %10d accz_raw: %10d\n\r", (int)acc_data_raw[0], (int)acc_data_raw[1], (int)acc_data_raw[2]);
  
}

//////////////////////////////////////////////
// Setup Functions
//////////////////////////////////////////////

//
// setup_imu
//
// Initialize registers and I2C to prepare IMU
//
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
    wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00);//power on gyro
    wiringPiI2CWriteReg8(gyro_address, 0x0F, 0x01);//set gyro to +-1000dps
    wiringPiI2CWriteReg8(gyro_address, 0x10, 0x03);//set data rate and bandwith
    
    
    sleep(1);
  }
  return 0;
}

//
// setup_joystick
//
// Set up joystick shared memory
//
void setup_joystick()
{
  int segment_id;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size = 0x6400;
  int smhkey=33222;

  /* Allocate a shared memory segment. */
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
  /* Attach the shared memory segment. */
  shared_memory = (Joystick*) shmat (segment_id, 0, 0);
  printf ("shared memory attached at address %p\n", shared_memory);
  /* Determine the segment's size. */
  shmctl (segment_id, IPC_STAT, &shmbuffer);
  segment_size = shmbuffer.shm_segsz;
  printf ("segment size: %d\n", segment_size);
  /* Write a string to the shared memory segment. */
  //sprintf (shared_memory, "test!!!!.");
}

//
// trap
//
// When cntrl+c pressed, kill motors
//
void trap(int signal)
{
  printf("EXIT: force quit\n\r");
  run_program=0;
}

//////////////////////////////////////////////
// Helper Functions
//////////////////////////////////////////////

//
// to_csv
//
// Save 2D array to CSV file
// Call like: 
//
// int data[3][4];
// printArray(&data[0][0], 3, 4);
//
void to_csv(float *arr, int rows, int cols)
{
  // Create file
  FILE *file;
  file = fopen("data.csv", "w");

  // Iterate over values
  for (int i = 0; i < rows; i++){
    for (int j = 0; j < cols; j++){
        fprintf(file, "%lf%s",arr[i * cols + j], (j < cols-1?",":""));    // Save data, append comma if not last value in row
    }
    fprintf(file,"\n");
  }
  fclose(file);
}

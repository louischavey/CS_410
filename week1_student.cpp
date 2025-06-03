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
// gcc -o week1_student week1_student.cpp -lwiringPi -lm
// gcc -o udp_rx udp_rx.cpp -lwiringPi -lm
// scp C:\Users\jarmi\CS_410\week1_student.cpp pi@10.42.0.1:/home/pi/flight_controller/week1_student.cpp
//

//////////////////////////////////////////////
// Function Prototypes and Structs
//////////////////////////////////////////////

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

void calc_pid();
double timespec_diff_sec(struct timespec start, struct timespec end);
int check_end_conditions(Joystick joystick_data, struct timespec tstart);
void calibrate_imu();      
void read_imu();    
void update_filter();
void to_csv(float *arr, int rows, int cols);
int setup_imu();
void setup_joystick();
void trap(int signal);
void kill_motors();
void motor_enable();
void set_motors(int motor0, int motor1, int motor2, int motor3);
void setup_camera();


//////////////////////////////////////////////
// Global Variables and Constants
//////////////////////////////////////////////

// IMU Reading
int accel_address,gyro_address;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //accel xyz,  gyro xyz,
int acc_data_raw[3];// Raw acc data for debug
#define RAW_TO_GS 10922.667
#define RAW_TO_DEG 32.768

// IMU Filtering
#define ALPHA 0.02
float roll_gyro_delta=0;
float pitch_gyro_delta=0;
float roll_filter=0;
float pitch_filter=0;
float intl_roll=0;
float intl_pitch=0;

// Data Plotting
int plot = 0;
#define MAX_ITERS 700  // for data collection
#define PLOT_COLS 6
float plot_data[MAX_ITERS][PLOT_COLS];  // first 3 cols roll, second 3 are pitch
int iteration=0;

// Joystick and Safety Bounds
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_accel=0;
float roll_accel=0;
Joystick* shared_memory;
Joystick joystick_data;
int run_program=1;
int paused=0;
int autonomous=0;
#define ROLL_BOUND 45
#define PITCH_BOUND 45
#define GYRO_BOUND 450
#define TIMEOUT 0.5

// PID Control
int motor_commands[4];  // hold commanded motor speeds based on PID control
#define THRUST_NEUTRAL 1550  // flying off ground 2-4 feet: 1650
#define THRUST_AMP 300  // flying off ground 2-4 feet: 300
int thrust=THRUST_NEUTRAL;
// Pitch
#define PITCH_AMP 5  // default: 5
#define PPGAIN 13.5    // PPGAIN = 13.5
#define PDGAIN 2.7  // PDGAIN = 2.7
#define PIGAIN 0.2  // PIGAIN = 0.2
float Pintegral = 0.0;
#define PISATURATE 100
// Roll
#define ROLL_AMP 5  // default: 5
#define RPGAIN 13.5    // RPGAIN = 13.5
#define RDGAIN 2.2  // RDGAIN = 2.2
#define RIGAIN 0.17  // RIGAIN = 0.17
float Rintegral = 0.0;
#define RISATURATE 100
// Yaw
#define YAW_AMP  200 // YAMP = 200
#define YPGAIN 6.9    // YPGAIN = 3.0

// Motor Interfacing
int motor_address;
#define MOTOR_LIM 2000

// Camera + Autonomous Flight
// Camera
struct Camera
{
  int x;
  int y;
  int z;
  int yaw;
  int sequence_num;
};
Camera* camera_memory;
Camera camera_data;
struct timespec camera_time;
int prev_cam_seq_num = -1;

#define DESIRED_YAW 0.0
#define CAM_YAW_GAIN 3.0

#define DESIRED_Z 609.6   // 2 feet in mm
int Z_PREV = 0;
#define CAM_Z_PGAIN 1.0
#define CAM_Z_DGAIN 0.0
#define CAM_Z_IGAIN 0.0
float auto_thrust_i = 0.0;
#define AUTO_I_SATURATE 100

#define AUTO_ANGLE_LIM 20.0

int init_desired_x = 0;
float DESIRED_X;
float X_PREV;
float camera_x_estimated;
#define CAM_X_PGAIN 0.02
#define CAM_X_DGAIN 0.005

int init_desired_y = 0;
float DESIRED_Y;
float Y_PREV;
float camera_y_estimated;
#define CAM_Y_PGAIN 0.04
#define CAM_Y_DGAIN 0.001


//////////////////////////////////////////////
// Main
//////////////////////////////////////////////
 
int main (int argc, char *argv[])
{
    // Setup peripherals
    setup_imu();
    motor_address = wiringPiI2CSetup(0x56);
    calibrate_imu();
    motor_enable();    
    setup_joystick();
    signal(SIGINT, &trap);
    setup_camera();

    // Variables for controller timeout
    int prev_seq_num = shared_memory->sequence_num;
    struct timespec timeout_start;
    timespec_get(&timeout_start, TIME_UTC);
    struct timespec auto_timeout_start;
    timespec_get(&auto_timeout_start, TIME_UTC);
    timespec_get(&camera_time, TIME_UTC);


    while(run_program)
    {
      if (iteration == MAX_ITERS) {break;}
      if(plot) {printf("%d\n\r", iteration);}
      // Get most recent joystick data
      joystick_data = *shared_memory;

      // Autonomous flight activation key
      if (joystick_data.key3) {
        paused=0;
      }
      if (joystick_data.key0 || paused) {
        paused=1;
        set_motors(1, 1, 1, 1);
        continue;
      }
      if (joystick_data.key2 && !autonomous) {  // turn on
        // start timeout
        timespec_get(&auto_timeout_start,TIME_UTC);
        autonomous=1;
        printf("autonomous on\n");
      }
      struct timespec auto_tcurr;
      timespec_get(&auto_tcurr,TIME_UTC);
      if (timespec_diff_sec(auto_tcurr, auto_timeout_start) > 1 && joystick_data.key2 && autonomous) {  // turn off
        autonomous=0;
        printf("autonomous off\n");
      }


      // If sequence number has changed, restart timeout
      if (joystick_data.sequence_num != prev_seq_num) {
        // get current time in seconds
        timespec_get(&timeout_start,TIME_UTC);
        prev_seq_num = joystick_data.sequence_num;
      }

      // Read, filter IMU data and check safety conditions
      read_imu(); 
      update_filter();
      run_program = check_end_conditions(joystick_data, timeout_start);

      // Access camera data from shared memory
      camera_data=*camera_memory;
      if (camera_data.sequence_num != prev_cam_seq_num) {
        prev_cam_seq_num = camera_data.sequence_num;
        timespec_get(&camera_time, TIME_UTC);
      }
      // printf("x: %d y: %d z: %d yaw: %d %d\n\r",camera_data.x,camera_data.y,camera_data.z,camera_data.yaw,camera_data.sequence_num);

      calc_pid();  // set motor speeds based on PID control
      set_motors(motor_commands[3], motor_commands[2], motor_commands[1], motor_commands[0]);
      if(plot){iteration++;}
    }

    to_csv(&plot_data[0][0], MAX_ITERS, PLOT_COLS);

    return 0;
}

//////////////////////////////////////////////
// Control Functions
//////////////////////////////////////////////

//
// set_motors
//
// Compute thrust and set motor values with PID control
//
void calc_pid() {
  // Calculate joystick thrust
  float thrust_mult = (-1 * (joystick_data.thrust - 128)) / 128.0f;
  float joystick_thrust = (int)((float)THRUST_NEUTRAL + (thrust_mult * THRUST_AMP));

  
  // float auto_thrust = 0.0;
  // if (!autonomous) {
  thrust = joystick_thrust;
  // }
  // else {
  //   // Calculate autonomous thrust
  //   float auto_thrust_p = CAM_Z_PGAIN*(-((float)camera_data.z) - DESIRED_Z);
    
  //   float elapsed_cam_time = (timespec_diff_sec(prev_camera_time, camera_time) * 1000); // Convert elapsed time to ms
  //   float auto_thrust_d = CAM_Z_DGAIN*((-((float)camera_data.z) - (float)Z_PREV) / elapsed_cam_time); 
  //   Z_PREV = -1 * camera_data.z;

  //   auto_thrust_i += CAM_Z_IGAIN*(-1*((float)camera_data.z) - DESIRED_Z);
  //   if (auto_thrust_i > AUTO_I_SATURATE) {auto_thrust_i = AUTO_I_SATURATE;}
  //   else if (auto_thrust_i < -AUTO_I_SATURATE) {auto_thrust_i = -AUTO_I_SATURATE;}

  //   auto_thrust = auto_thrust_p + auto_thrust_d + auto_thrust_i;

  //   thrust = 0.5*joystick_thrust + 0.5*auto_thrust;

  //   printf("autonomous: %d camera_z: %d, auto_thrust: %f auto_p: %f auto_d: %f auto_i: %f\n\r", autonomous, camera_data.z, auto_thrust, auto_thrust_p, auto_thrust_d, auto_thrust_i);
  // }

  //
  // Pitch
  //

  // Pitch error = joystick_pitch - filter_pitch
  float pitch_mult = ((float)joystick_data.pitch - 128.0f) / 128.0f;
  float joystick_desired_pitch = ((float)PITCH_AMP) * pitch_mult;

  float pitch_desired = 0.0;
  if (!autonomous) {
    pitch_desired = joystick_desired_pitch;
  }
  else {
    // Initialize desired y value and previous y_value to first camera reading
    if (!init_desired_x) {
      DESIRED_X = camera_data.x;
      X_PREV = camera_data.x;
      camera_x_estimated = camera_data.x;
      init_desired_x = 1;
    }
    // Filtered estimate for current y position
    // Prevent large reading spikes in filtered y position
    if (!(fabs((float)camera_data.x) >= fabs(100.0*camera_x_estimated))) {
      camera_x_estimated = 0.6*camera_x_estimated + 0.4*((float)camera_data.x);
    }

    // PD controller for y
    struct timespec tcurr;      // get current time in seconds
    timespec_get(&tcurr, TIME_UTC);
    float elapsed_cam_time = (timespec_diff_sec(camera_time, tcurr) * 1000); // Convert elapsed time to ms
    float auto_desired_pitch_p = CAM_X_PGAIN*(camera_x_estimated - DESIRED_X);
    float auto_desired_pitch_d = CAM_X_DGAIN*(camera_x_estimated - X_PREV) / elapsed_cam_time;
    X_PREV = camera_x_estimated;    // Update previous y position

    float auto_desired_pitch = auto_desired_pitch_p - auto_desired_pitch_d;

    // Put cap on auto desired roll
    if (auto_desired_pitch >= AUTO_ANGLE_LIM) {
      auto_desired_pitch = AUTO_ANGLE_LIM;
    }
    else if (auto_desired_pitch <= -AUTO_ANGLE_LIM) {
      auto_desired_pitch = -AUTO_ANGLE_LIM;
    }
    
    pitch_desired = 0.5*joystick_desired_pitch + 0.5*auto_desired_pitch;
    printf("x_desired: %f pitch_actual: %f pitch_desired: %f auto_desired: %f filtered_x: %f camera_x: %d pitch_p: %f pitch_d: %f\n\r", DESIRED_X, pitch_filter, pitch_desired, auto_desired_pitch, camera_x_estimated, camera_data.x, auto_desired_pitch_p, auto_desired_pitch_d);

    // write to data array
    if(plot) {
      plot_data[iteration][0] = pitch_filter;
      plot_data[iteration][1] = auto_desired_pitch;
      plot_data[iteration][2] = camera_y_estimated;
      plot_data[iteration][3] = auto_desired_pitch_p;
      plot_data[iteration][4] = auto_desired_pitch_d;
      plot_data[iteration][5] = DESIRED_X;

    }
  }
  float pitch_error = (float)pitch_desired - pitch_filter;

  // Calculate integral
  Pintegral += PIGAIN * pitch_error;
  if (Pintegral > PISATURATE) {Pintegral = PISATURATE;}
  else if (Pintegral < -PISATURATE) {Pintegral = -PISATURATE;}

  //
  // Roll
  //

  // Roll error = joystick_roll - filter_roll
  float roll_mult = ((float)joystick_data.roll - 128.0f) / 128.0f;
  float joystick_desired_roll = ((float)ROLL_AMP) * roll_mult;
  float roll_desired = 0.0;
  if (!autonomous) {
    roll_desired = joystick_desired_roll;
  }
  else {
    // Initialize desired y value and previous y_value to first camera reading
    if (!init_desired_y) {
      DESIRED_Y = camera_data.y;
      Y_PREV = camera_data.y;
      camera_y_estimated = camera_data.y;
      init_desired_y = 1;
    }
    // Filtered estimate for current y position
    // Prevent large reading spikes in filtered y position
    if (!(fabs((float)camera_data.y) >= fabs(100.0*camera_y_estimated))) {
      camera_y_estimated = 0.6*camera_y_estimated + 0.4*((float)camera_data.y);
    }

    // PD controller for y
    struct timespec tcurr;      // get current time in seconds
    timespec_get(&tcurr, TIME_UTC);
    float elapsed_cam_time = (timespec_diff_sec(camera_time, tcurr) * 1000); // Convert elapsed time to ms
    float auto_desired_roll_p = CAM_Y_PGAIN*(camera_y_estimated - DESIRED_Y);
    float auto_desired_roll_d = CAM_Y_DGAIN*(camera_y_estimated - Y_PREV) / elapsed_cam_time;
    Y_PREV = camera_y_estimated;    // Update previous y position

    float auto_desired_roll = auto_desired_roll_p - auto_desired_roll_d;

    // Put cap on auto desired roll
    if (auto_desired_roll >= AUTO_ANGLE_LIM) {
      auto_desired_roll = AUTO_ANGLE_LIM;
    }
    else if (auto_desired_roll <= -AUTO_ANGLE_LIM) {
      auto_desired_roll = -AUTO_ANGLE_LIM;
    }
    
    roll_desired = 0.5*joystick_desired_roll + 0.5*auto_desired_roll;
    printf("y_desired: %f roll_actual: %f roll_desired: %f auto_desired: %f filtered_y: %f camera_y: %d roll_p: %f roll_d: %f\n\r", DESIRED_Y, roll_filter, roll_desired, auto_desired_roll, camera_y_estimated, camera_data.y, auto_desired_roll_p, auto_desired_roll_d);

    // write to data array
    if(plot) {
      plot_data[iteration][0] = roll_filter;
      plot_data[iteration][1] = auto_desired_roll;
      plot_data[iteration][2] = camera_y_estimated;
      plot_data[iteration][3] = auto_desired_roll_p;
      plot_data[iteration][4] = auto_desired_roll_d;
      plot_data[iteration][5] = DESIRED_Y;

    }
  }
  float roll_error = (float)roll_desired - roll_filter;

  // Calculate integral
  Rintegral += RIGAIN * roll_error;
  if (Rintegral > RISATURATE) {Rintegral = RISATURATE;}
  else if (Rintegral < -RISATURATE) {Rintegral = -RISATURATE;}

  //
  // Yaw
  //

  // Yaw velocity error = joystick_yaw_velocity - gyroscope[x]
  float yaw_mult = ((float)joystick_data.yaw - 128.0f) / 128.0f;
  float yaw_vel_desired;
  if (!autonomous) {  // from joystick
    yaw_vel_desired = -((float)YAW_AMP) * yaw_mult;
  }
  else {  
    // printf("Camera data in calc_pid: %d\n", camera_data.yaw);
    float diff = ((float)camera_data.yaw) - DESIRED_YAW;
    yaw_vel_desired = -diff * CAM_YAW_GAIN;
  }
  float yaw_vel_error = -(float)yaw_vel_desired + imu_data[3]; 

  //
  // Assign commands
  //

  int pitch_front_command = (int)(PPGAIN * pitch_error) - (int)(PDGAIN * imu_data[5]) + (int)(Pintegral);
  int pitch_back_command = -(int)(PPGAIN * pitch_error) + (int)(PDGAIN * imu_data[5]) - (int)(Pintegral);
  int roll_right_command = -(int)(RPGAIN * roll_error) + (int)(RDGAIN * imu_data[4]) - (int)(Rintegral);
  int roll_left_command = (int)(RPGAIN * roll_error) - (int)(RDGAIN * imu_data[4]) + (int)(Rintegral);
  int yaw_cw_command = (int)(YPGAIN * yaw_vel_error);
  int yaw_ccw_command = -(int)(YPGAIN * yaw_vel_error);

  // Update motors
  motor_commands[0] = thrust + pitch_front_command + roll_left_command + yaw_cw_command;    // front left
  motor_commands[1] = thrust + pitch_back_command + roll_left_command + yaw_ccw_command;    // back left
  motor_commands[2] = thrust + pitch_front_command + roll_right_command + yaw_ccw_command;  // front right
  motor_commands[3] = thrust + pitch_back_command + roll_right_command + yaw_cw_command;    // back right

  for(size_t i = 0; i < 4; i++) {
    if(motor_commands[i] > MOTOR_LIM) {
      motor_commands[i] = MOTOR_LIM;
    }
    else if(motor_commands[i] < -MOTOR_LIM) {
      motor_commands[i] = -MOTOR_LIM;
    }
  }

  if(plot) {
      plot_data[iteration][0] = pitch_filter;
      plot_data[iteration][1] = pitch_desired;
  }
  

  

}

void set_motors(int motor0, int motor1, int motor2, int motor3)
{
  if(motor0<0)
    motor0=0;
  if(motor0>2000)
    motor0=2000;
  if(motor1<0)
    motor1=0;
  if(motor1>2000)
    motor1=2000;
  if(motor2<0)
    motor2=0;
  if(motor2>2000)
    motor2=2000;
  if(motor3<0)
    motor3=0;
  if(motor3>2000)
    motor3=2000;
  uint8_t motor_id=0;
  uint8_t special_command=0;
  uint16_t commanded_speed_0=1000;
  uint16_t commanded_speed_1=0;
  uint16_t commanded_speed=0;
  uint8_t data[2];
  // wiringPiI2CWriteReg8(motor_address, 0x00,data[0] );
  //wiringPiI2CWrite (motor_address,data[0]) ;
  int com_delay=500;
  motor_id=0;
  commanded_speed=motor0;
  data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
  data[1]=commanded_speed&0x7f;
  wiringPiI2CWrite(motor_address,data[0]);
  usleep(com_delay);
  wiringPiI2CWrite(motor_address,data[1]);
  usleep(com_delay);
  motor_id=1;
  commanded_speed=motor1;
  data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
  data[1]=commanded_speed&0x7f;
  usleep(com_delay);
  wiringPiI2CWrite(motor_address,data[0]);
  wiringPiI2CWrite(motor_address,data[1]);
  usleep(com_delay);
  motor_id=2;
  commanded_speed=motor2;
  data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
  data[1]=commanded_speed&0x7f;
  wiringPiI2CWrite(motor_address,data[0]);
  usleep(com_delay);
  wiringPiI2CWrite(motor_address,data[1]);
  usleep(com_delay);
  motor_id=3;
  commanded_speed=motor3;
  data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
  data[1]=commanded_speed&0x7f;
  wiringPiI2CWrite(motor_address,data[0]);
  usleep(com_delay);
  wiringPiI2CWrite(motor_address,data[1]);
  usleep(com_delay);
}




//////////////////////////////////////////////
// IMU Functions
//////////////////////////////////////////////


void kill_motors() {
  for(size_t i=0; i < 4; i++) {
    motor_commands[i] = 0;
  }
}

//
// check_end_conditions
//
// Check all stopping conditions, and exit if any are true. Report
// reason why before exiting.
//
int check_end_conditions(Joystick joystick_data, struct timespec tstart) {
  if(fabs(imu_data[3]) > GYRO_BOUND || fabs(imu_data[4]) > GYRO_BOUND || fabs(imu_data[5]) > GYRO_BOUND) {     // gyro speed out of bounds
    printf("EXIT: gyro speed out of safe bounds (+/- %d deg/sec)\n\r", GYRO_BOUND);
    kill_motors();
    return 0;
  }

  if (fabs(roll_filter) > ROLL_BOUND) {                     // roll angle out of bounds
    kill_motors();
    printf("EXIT: roll angle out of safe bounds (+/- %d deg)\n\r", ROLL_BOUND);
    return 0;
  }

  if (fabs(pitch_filter) > PITCH_BOUND) {                    // pitch angle out of bounds
    kill_motors();
    printf("EXIT: pitch angle out of bounds (+/- %d deg)\n\r", PITCH_BOUND);
    return 0;
  }

  if(joystick_data.key1) {                          // manual exit (B button)
    kill_motors();
    printf("EXIT: manual exit (B button)\n\r");
    return 0;
  }

  // get current time in seconds
  struct timespec tcurr;
  timespec_get(&tcurr, TIME_UTC);
  double elapsed = timespec_diff_sec(tstart, tcurr);
  // printf("Elapsed: %.3f sec Seq_num: %10d\n\r", elapsed, joystick_data.sequence_num);

  if (elapsed >= TIMEOUT) {                  // controller timeout (if sequence number hasn't change for TIMEOUT seconds)
    printf("EXIT: controller timeout (%f seconds)\n\r", TIMEOUT);
    kill_motors();
    return 0;
  }

  return run_program;   // return 'run_program' instead of '1' to detect ctrl-c force quit
}

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
    wiringPiI2CWriteReg8(accel_address, 0x40, 0x99); //high speed filtered accel
    wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00);  //power on gyro
    wiringPiI2CWriteReg8(gyro_address, 0x0F, 0x01);  //set gyro to +-1000dps
    wiringPiI2CWriteReg8(gyro_address, 0x10, 0x03);  //set data rate and bandwith
    
    
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

void setup_camera()
{
  int segment_id;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size = sizeof(struct Camera);
  int smhkey=123456;
  /* Allocate a shared memory segment. */
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
  /* Attach the shared memory segment. */
  camera_memory = (Camera*) shmat (segment_id, 0, 0);
  printf ("shared memory attached at address %p\n", camera_memory);
  /* Determine the segment's size. */
  shmctl (segment_id, IPC_STAT, &shmbuffer);
  segment_size = shmbuffer.shm_segsz;
  printf ("segment size: %d\n", segment_size);
  /* Write a string to the shared memory segment. */
  // sprintf (shared_memory, "test!!!!.");
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

//
// motor_enable
//
// Enable motors by sending command to motor controller
//
void motor_enable()
{
  uint8_t motor_id=0;
  uint8_t special_command=0;
  uint16_t commanded_speed_0=1000;
  uint16_t commanded_speed_1=0;
  uint16_t commanded_speed=0;
  uint8_t data[2];
  int cal_delay=50;
  for(int i=0;i<1000;i++)
  {
    motor_id=0;
    commanded_speed=0;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(cal_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(cal_delay);
    motor_id=1;
    commanded_speed=0;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(cal_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(cal_delay);
    motor_id=2;
    commanded_speed=0;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(cal_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(cal_delay);
    motor_id=3;
    commanded_speed=0;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(cal_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(cal_delay);
  }
  for(int i=0;i<2000;i++)
  {
    motor_id=0;
    commanded_speed=50;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(cal_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(cal_delay);
    motor_id=1;
    commanded_speed=50;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(cal_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(cal_delay);
    motor_id=2;
    commanded_speed=50;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(cal_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(cal_delay);
    motor_id=3;
    commanded_speed=50;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;
    wiringPiI2CWrite(motor_address,data[0]);
    usleep(cal_delay);
    wiringPiI2CWrite(motor_address,data[1]);
    usleep(cal_delay);
  }
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
  if (!file) {
    perror("fopen");
    return;
  }

  // Iterate over values
  for (int i = 0; i < rows; i++){
    for (int j = 0; j < cols; j++){
        fprintf(file, "%f%s",arr[i * cols + j], (j < cols-1?",":""));    // Save data, append comma if not last value in row
    }
    fprintf(file,"\n");
  }
  fclose(file);
}

// Compute difference in seconds between two timespecs
double timespec_diff_sec(struct timespec start, struct timespec end) {
  return (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
}

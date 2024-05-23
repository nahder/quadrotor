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
#include <curses.h>
#include "vive.h"

//declare global struct
Position local_p;

// gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG 0x1A
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define USER_CTRL 0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1 0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2 0x6C


// define safety-related constants
#define SAFTEY_GYRO_RATE 400
#define SAFETY_ROLL_LIM 45
#define SAFETY_PITCH_LIM 45
#define SAFETY_HEARTBEAT_TO 500
#define SAFETY_VIVE_TO 500

// define filter-related constants
#define A_FILTER 0.01 // 0.02

// #define KP_PITCH 0.0
// #define KD_PITCH 0.0
// #define KI_PITCH 0.0

#define KP_PITCH 11.05 // 11.03
#define KD_PITCH 1.45 //1.8
#define KI_PITCH 0.03 //0.045

// #define KP_ROLL 0.0
// #define KD_ROLL 0.0
// #define KI_ROLL 0.0

#define KP_ROLL 10.5 //10.5
#define KD_ROLL 1.25 // 1.3
#define KI_ROLL 0.03 // 0.06

// #define KD_YAW 0.0

#define KD_YAW 2.8

#define KP_YAW_POSITION 201.0

//add global variable
int pwm;

float motor_0_pwm = 0.0, motor_1_pwm = 0.0, motor_2_pwm = 0.0, motor_3_pwm = 0.0;
float neutral_power = 1351;

//add constants
#define PWM_MAX 1840
#define LED0 0x6
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define LED_MULTIPLYER 4


enum Ascale
{
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale
{
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

// function declarations
int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();
void get_rpy();

void print_data();

void print_filter();
void safety_check();

void init_pwm();
void init_motor(uint8_t channel);
void set_PWM(uint8_t channel, float time_on_us);
void yaw_control();

void pid_update();

int map();
void joystick_interface();

// global variables
int imu;
float x_gyro_calibration = 0; // gyro calibration offsets
float y_gyro_calibration = 0;
float z_gyro_calibration = 0;
float roll_calibration = 0; // accel calibration offsets
float pitch_calibration = 0;
float accel_z_calibration = 0;
float imu_data[6]; // gyro xyz, accel xyz
long time_curr;     // time-tracKI_PITCHng variables
long time_prev;
struct timespec te;
struct timeval tv;
float yaw = 0;
float pitch_accel = 0;  // pitch according to accel
float roll_accel = 0;   // roll according to accel
float roll_gyro_delta;
float pitch_gyro_delta;
float pitch_t = 0.0;    // pitch according to filter
float roll_t = 0.0;     // roll according to filter
static float pitch_integral = 0.0;
static float roll_integral = 0.0;

float thrust = 0.0;

float pitch_setpoint = 0.0;

float roll_setpoint = 0.0;

float yaw_setpoint = 0.0;

float imu_diff_copy = 0.0;

float vive_x_offset = 0.0;
float vive_y_offset = 0.0;

struct Joystick
{
  int key0; //byte 4, X = unpause
  int key1; //byte 5, A = kill all
  int key2; //byte 6, B = pause
  int key3; //byte 7, Y = calibrate
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num;
};


Joystick * joystick_memory;

int run_program = 1;

bool paused = true;
bool calibrate = false;
bool firstrun = true;
// function to add
void setup_joystick()
{

  int segment_id;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size = 0x6400;
  int smhkey = 33222;

  /* Allocate a shared memory segment.  */
  segment_id = shmget(smhkey, shared_segment_size, IPC_CREAT | 0666);
  /* Attach the shared memory segment.  */
  joystick_memory = (Joystick *)shmat(segment_id, 0, 0);
  printf("shared memory attached at address %p\n", joystick_memory);
  /* Determine the segment's size. */
  shmctl(segment_id, IPC_STAT, &shmbuffer);
  segment_size = shmbuffer.shm_segsz;
  printf("segment size: %d\n", segment_size);
  /* Write a string to the shared memory segment.  */
  // sprintf (shared_memory, "test!!!!.");
}


// when cntrl+c pressed, KI_PITCHll motors
void trap(int signal)
{
  printf("ending program due to CRTL+C called\n\r");
  set_PWM(0, 1000);
  set_PWM(1, 1000);
  set_PWM(2, 1000);
  set_PWM(3, 1000);
  run_program = 0;
}

int main(int argc, char * argv[])
{
  //call at start of main loop
    init_shared_memory();

  //in main function before calibrate imu add
  init_pwm();
  init_motor(0);
  init_motor(1);
  init_motor(2);
  init_motor(3);
  delay(1000);

  setup_imu();
  calibrate_imu();
  setup_joystick();
  signal(SIGINT, &trap);

  while (run_program == 1) {
    //run this command at the start of the while(1) loop to refresh vive data
    local_p=*position;  

    // keyboard_interface();
    joystick_interface();

    if (paused) {
      set_PWM(0, 1000);
      set_PWM(1, 1000);
      set_PWM(2, 1000);
      set_PWM(3, 1000);

      if (calibrate) {
        calibrate_imu();
        //now you can use the vive sensor values: 
        vive_x_offset = local_p.x;
        vive_y_offset = local_p.y;
        firstrun = true;
        calibrate = false;
      }

    } else {

      read_imu();   // read data from IMU, caluclate accelerometer pitch,roll

      update_filter();   // update complementary filter
      yaw_control();
      pid_update();

      set_PWM(0, motor_0_pwm);
      set_PWM(1, motor_1_pwm);
      set_PWM(2, motor_2_pwm);
      set_PWM(3, motor_3_pwm);

    }

    // time
    gettimeofday(&tv, NULL);
    // safety checks
    safety_check();

    print_data();

  }
  return 0;
}

void safety_check()
{
  static long last_heartbeat_time = 0;
  static long last_vive_time = 0;
  static long last_vive_value = 0;
  static int last_heartbeat = 0;

  if (firstrun) {
    long curr_time = tv.tv_sec * 1000LL + tv.tv_usec / 1000;
    last_heartbeat_time = curr_time;
    last_vive_time = curr_time;
    firstrun = false;
  }

  Joystick joystick = *joystick_memory;

  // gyros rate is too high
  if (imu_data[0] > SAFTEY_GYRO_RATE || imu_data[0] < -SAFTEY_GYRO_RATE ||
    imu_data[1] > SAFTEY_GYRO_RATE || imu_data[1] < -SAFTEY_GYRO_RATE ||
    imu_data[2] > SAFTEY_GYRO_RATE || imu_data[2] < -SAFTEY_GYRO_RATE)
  {
    printf("ending program due to gyro rate\n\r");
    run_program = 0;
  }
  // roll angle
  if (roll_t > SAFETY_ROLL_LIM || roll_t < -SAFETY_ROLL_LIM) {
    printf("ending program due to roll greater than 45 degrees\n\r");
    run_program = 0;
  }
  // pitch angle
  if (pitch_t > SAFETY_PITCH_LIM || pitch_t < -SAFETY_PITCH_LIM) {
    printf("ending program due to pitch greater than 45 degrees\n\r");
    run_program = 0;
  }
  // heartbeat check
  long curr_time = tv.tv_sec * 1000LL + tv.tv_usec / 1000;
  if (curr_time > last_heartbeat_time + SAFETY_HEARTBEAT_TO) {
    printf("ending program due to heartbeat timeout\n\r");
    run_program = 0;
  }

  if (curr_time > last_vive_time + SAFETY_VIVE_TO) {
    printf("ending program due to vive timeout\n\r");
    run_program = 0;
  }

  if (joystick.sequence_num > last_heartbeat) {
    last_heartbeat_time = curr_time;
    last_heartbeat = joystick.sequence_num;
  }

  if (local_p.version > last_vive_value) {
    last_vive_time = curr_time;
    last_vive_value = local_p.version;
  }

  if (joystick.sequence_num == 255) {
    last_heartbeat = -1;
  }

  if (local_p.x - vive_x_offset > 1000.0 || local_p.x  - vive_x_offset < -1000.0 || local_p.y - vive_y_offset > 1000.0 || local_p.y - vive_y_offset < -1000.0){
    run_program = 0;
  }

  if (run_program == 0.0) {
    set_PWM(0, 1000);
    set_PWM(1, 1000);
    set_PWM(2, 1000);
    set_PWM(3, 1000);
  }
}


int map(int original, int in_min, int in_max, int out_min, int out_max)
{
  return ((original - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;

}

void joystick_interface()
{

  // int key0;   //byte 4, X = unpause
  // int key1; //byte 5, A = kill all
  // int key2; //byte 6, B = pause
  // int key3; //byte 7, Y = calibrate
  Joystick joystick = *joystick_memory;     // to refresh values from shared memory first
  int thrust_mapped = 0, pitch_mapped = 0, roll_mapped = 0, yaw_mapped = 0;

  //unpause
  if (joystick.key0 == 1) {
    paused = false;
  }

  if (joystick.key2 == 1) {
    paused = true;
  }

  if (joystick.key3 == 1) {
    calibrate = true;
  }

  if (joystick.key1 == 1) {
    run_program = 0;
  }

  thrust_mapped = map(joystick.thrust, 255, 0, -100, 100);
  pitch_mapped = map(joystick.pitch, 255, 0, -8, 8);
  roll_mapped = map(joystick.roll, 255, 0, -8, 8);

  yaw_mapped = map(joystick.yaw, 0, 255, -40, 40);

  thrust = thrust_mapped;
  pitch_setpoint = pitch_mapped;
  roll_setpoint = roll_mapped;
  // yaw_setpoint = yaw_mapped;

  //thrust ++ byte 1 (down)
  //thrust -- byte 1 (up)

  //yaw ++ byte 0 (right)
  //yaw -- byte 0 (left)

  //pitch ++ byte 3 (down)
  //pitch -- byte 3 (up)

  //roll ++ byte 2 (right)
  //roll -- byte 2 (left)
}


void calibrate_imu()
{
  /*
  accel_z_calibration=??
  */
  float x_gyro_offset = 0.0, y_gyro_offset = 0.0, z_gyro_offset = 0.0;
  float roll_offset = 0.0, pitch_offset = 0.0;
  pitch_calibration = 0.0;
  roll_calibration = 0.0;


  float num_samples = 1000;

  for (int i = 0; i < (int)num_samples; i++) {
    read_imu();
    get_rpy();
    x_gyro_offset += imu_data[0];
    y_gyro_offset -= imu_data[1];
    z_gyro_offset -= imu_data[2];

    roll_offset += roll_accel;
    pitch_offset += pitch_accel;
  }

  x_gyro_calibration = x_gyro_offset / num_samples;
  y_gyro_calibration = y_gyro_offset / num_samples;
  z_gyro_calibration = z_gyro_offset / num_samples;

  roll_calibration = roll_offset / num_samples;
  pitch_calibration = pitch_offset / num_samples;

  roll_calibration *= M_PI / 180.0;
  pitch_calibration *= M_PI / 180.0;

  printf(
    "calibration complete, %f %f %f %f %f %f\n\r", x_gyro_calibration, y_gyro_calibration,
    z_gyro_calibration, roll_calibration, pitch_calibration, accel_z_calibration);
}

void get_rpy()
{
  pitch_accel = -(atan2(imu_data[4], imu_data[5]) + pitch_calibration);
  roll_accel = -(atan2(imu_data[3], imu_data[5]) + roll_calibration);

  if (pitch_accel > M_PI) {
    pitch_accel -= 2 * M_PI;
  } else if (pitch_accel < -M_PI) {
    pitch_accel += 2 * M_PI;
  }

  if (roll_accel > M_PI) {
    roll_accel -= 2 * M_PI;
  } else if (roll_accel < -M_PI) {
    roll_accel += 2 * M_PI;
  }

  pitch_accel *= 180.0 / M_PI;
  roll_accel *= 180.0 / M_PI;
}

void read_imu()
{
  int address = 59;   // set address value for accel x value
  float ax = 0;
  float az = 0;
  float ay = 0;
  int vh, vl;

  // read in data
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  // convert 2 complement
  int vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }
  // imu_data from -32k to 32k

  imu_data[3] = 2.0 * vw / 32767.0;   // convert vw from raw values to "g's"

  address = 61;   // set address value for accel y value
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }
  imu_data[4] = 2.0 * vw / 32767.0;   // convert vw from raw values to "g's"

  address = 63;   // set addres value for accel z value;
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }
  imu_data[5] = 2.0 * vw / 32767.0;   // convert vw from raw values to g's

  address = 67;   // set addres value for gyro x value;
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }

  imu_data[0] = -(x_gyro_calibration + (vw * 500.0 / 32767.0));   // convert vw from raw values to degrees/second

  address = 69;   // set addres value for gyro y value;
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }
  imu_data[1] = y_gyro_calibration + (vw * 500.0 / 32767.0);   // convert vw from raw values to degrees/second

  address = 71;   // set addres value for gyro z value;
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }
  imu_data[2] = z_gyro_calibration + (vw * 500.0 / 32767.0);   // convert vw from raw values to degrees/second

  get_rpy();
}

void update_filter()
{

  // get current time in nanoseconds
  timespec_get(&te, TIME_UTC);
  time_curr = te.tv_nsec;
  // compute time since last execution
  float imu_diff = time_curr - time_prev;

  // check for rollover
  if (imu_diff <= 0) {
    imu_diff += 1000000000;
  }
  // convert to seconds
  imu_diff = imu_diff / 1000000000;
  time_prev = time_curr;

  // multiply gyro reading by delta t (integrate)
  roll_gyro_delta = imu_data[1] * imu_diff;
  pitch_gyro_delta = imu_data[0] * imu_diff;

  // comp. filter for roll, pitch here:
  roll_t = roll_accel * A_FILTER + (1.0 - A_FILTER) * (roll_gyro_delta + roll_t);
  pitch_t = pitch_accel * A_FILTER + (1.0 - A_FILTER) * (pitch_gyro_delta + pitch_t);

  // copy time diff to global variable
  imu_diff_copy = imu_diff;
}

int setup_imu()
{
  wiringPiSetup();
  // setup imu on I2C
  imu = wiringPiI2CSetup(0x68);   // accel/gyro address

  if (imu == -1) {
    printf("-----cant connect to I2C device %d --------\n", imu);
    return -1;
  } else {

    printf("connected to i2c device %d\n", imu);
    printf("imu who am i is %d \n", wiringPiI2CReadReg8(imu, 0x75));

    uint8_t Ascale = AFS_2G;         // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS;     // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

    // init imu
    wiringPiI2CWriteReg8(imu, PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu, PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00);     // 0x04
    int c = wiringPiI2CReadReg8(imu, GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);
    c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0xE0);     // Clear self-test bits [7:5]
    wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0x18);     // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c | Ascale << 3);
    c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
    wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c & ~0x0F);     //
    wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c | 0x00);


    // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x06 | 0x01 << 3);
    // c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
    // printf("accel config 2 %d \n", c);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x02);
    wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x03);
    // c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
    // printf("accel config 2 %d \n", c);

    // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x03 | 0x01 << 3);
    // c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
    // printf("accel config 2 %d \n", c);


    // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x02 | 0x01 << 3);
    // c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
    // printf("accel config 2 %d \n", c);


    // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x00 | 0x01 << 3);
    // c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
    // printf("accel config 2 %d \n", c);
  }
  return 0;
}

void init_pwm()
{

  pwm = wiringPiI2CSetup(0x40);
  if (pwm == -1) {
    printf("-----cant connect to I2C device %d --------\n", pwm);

  } else {

    float freq = 400.0 * .95;
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = floor(prescaleval + 0.5);
    int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
    int sleep = settings | 0x10;
    int wake = settings & 0xef;
    int restart = wake | 0x80;
    wiringPiI2CWriteReg8(pwm, 0x00, sleep);
    wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
    wiringPiI2CWriteReg8(pwm, 0x00, wake);
    delay(10);
    wiringPiI2CWriteReg8(pwm, 0x00, restart | 0x20);
  }
}


void init_motor(uint8_t channel)
{
  int on_value = 0;

  int time_on_us = 900;
  uint16_t off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));

  wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
  delay(100);

  time_on_us = 1200;
  off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));

  wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
  delay(100);

  time_on_us = 1000;
  off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));

  wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
  delay(100);

}

void set_PWM(uint8_t channel, float time_on_us)
{
  if (run_program == 1) {
    if (time_on_us > PWM_MAX) {
      time_on_us = PWM_MAX;
    } else if (time_on_us < 1000) {
      time_on_us = 1000;
    }
    uint16_t off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));
    wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value);
  } else {
    time_on_us = 1000;
    uint16_t off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));
    wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value);
  }
}

void pid_update()
{
  // static float pitch_integral = 0.0;
  auto pitch_vel = imu_data[0]; // copy the gyro value
  auto roll_vel = imu_data[1]; // copy the gyro value
  auto yaw_vel = imu_data[2]; // copy the gyro value

  //pitch_t, roll_t filtered
  auto pitch_error = pitch_t - pitch_setpoint;
  auto roll_error = roll_t - roll_setpoint;
  auto yaw_error = yaw_vel - yaw_setpoint;

  // integrate error
  pitch_integral += pitch_error * KI_PITCH;
  roll_integral += roll_error * KI_ROLL;

  auto KI_SAT = 80.0;
  auto KI_ROLL_SAT = 80.0;
  // limit integral term
  if (pitch_integral > KI_SAT) {
    pitch_integral = KI_SAT;
  }
  if (pitch_integral < -KI_SAT) {
    pitch_integral = -KI_SAT;
  }

  // limit integral term
  if (roll_integral > KI_ROLL_SAT) {
    roll_integral = KI_ROLL_SAT;
  }
  if (roll_integral < -KI_ROLL_SAT) {
    roll_integral = -KI_ROLL_SAT;
  }

  // FR
  motor_0_pwm = neutral_power + thrust +
    KP_PITCH * pitch_error + KD_PITCH * pitch_vel + pitch_integral -
    KP_ROLL * roll_error - KD_ROLL * roll_vel - roll_integral - KD_YAW * yaw_error;
  // FL
  motor_3_pwm = neutral_power + thrust +
    KP_PITCH * pitch_error + KD_PITCH * pitch_vel + pitch_integral +
    KP_ROLL * roll_error + KD_ROLL * roll_vel + roll_integral + KD_YAW * yaw_error;
  // RR
  motor_1_pwm = neutral_power + thrust -
    KP_PITCH * pitch_error - KD_PITCH * pitch_vel - pitch_integral -
    KP_ROLL * roll_error - KD_ROLL * roll_vel - roll_integral + KD_YAW * yaw_error;
  // RL
  motor_2_pwm = neutral_power + thrust -
    KP_PITCH * pitch_error - KD_PITCH * pitch_vel - pitch_integral +
    KP_ROLL * roll_error + KD_ROLL * roll_vel + roll_integral - KD_YAW * yaw_error;

}

void yaw_control(){
    yaw_setpoint = KP_YAW_POSITION * local_p.yaw;
}

void print_data()
{
  printf(
    "%10.5f,%10.5f,%10.5f\n\r",
    local_p.x - vive_x_offset,
    local_p.y - vive_y_offset,
    local_p.yaw
  );

}

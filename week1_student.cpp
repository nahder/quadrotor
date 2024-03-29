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

//gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C


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

int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();
void print_imu();
void get_rpy();
void print_milestone1();

//global variables
int imu;
float x_gyro_calibration = 0;
float y_gyro_calibration = 0;
float z_gyro_calibration = 0;
float roll_calibration = 0;
float pitch_calibration = 0;
float accel_z_calibration = 0;
float imu_data[6]; //gyro xyz, accel xyz
long time_curr;
long time_prev;
struct timespec te;
float yaw = 0;
float pitch_angle = 0;
float roll_angle = 0;


int main(int argc, char * argv[])
{

  setup_imu();
  calibrate_imu();


  while (1) {
    read_imu();
    get_rpy();
    // print_imu();
    print_milestone1();
    update_filter();
  }


}

void print_imu()
{
  printf(
    "ax %10.5f ay %10.5f az %10.5f gx %10.5f gy %10.5f gz %10.5f\n\r",
    imu_data[3],
    imu_data[4],
    imu_data[5],
    imu_data[0],
    imu_data[1],
    imu_data[2]);
}

// print gx gy gz roll pitch (angles)
void print_milestone1()
{
  printf(
    "gx %10.5f gy %10.5f gz %10.5f roll %10.5f pitch %10.5f\n\r",
    imu_data[0],
    imu_data[1],
    imu_data[2],
    roll_angle,
    pitch_angle);
}


void calibrate_imu()
{
  /*
  accel_z_calibration=??
  */
  float x_gyro_offset, y_gyro_offset, z_gyro_offset;
  float roll_offset, pitch_offset;

  float num_samples = 1000;

  for (int i = 0; i < (int)num_samples; i++) {
    read_imu();
    get_rpy();
    x_gyro_offset += imu_data[0];
    y_gyro_offset -= imu_data[1];
    z_gyro_offset -= imu_data[2];

    roll_offset += roll_angle;
    pitch_offset += pitch_angle;
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
  pitch_angle = -(atan2(imu_data[4], imu_data[5]) + pitch_calibration);
  roll_angle = -(atan2(imu_data[3], imu_data[5]) + roll_calibration);

  if (pitch_angle > M_PI) {
    pitch_angle -= 2 * M_PI;
  } else if (pitch_angle < -M_PI) {
    pitch_angle += 2 * M_PI;
  }

  if (roll_angle > M_PI) {
    roll_angle -= 2 * M_PI;
  } else if (roll_angle < -M_PI) {
    roll_angle += 2 * M_PI;
  }

  pitch_angle *= 180.0 / M_PI;
  roll_angle *= 180.0 / M_PI;

}

void read_imu()
{
  int address = 59; //set address value for accel x value
  float ax = 0;
  float az = 0;
  float ay = 0;
  int vh, vl;

  //read in data
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  //convert 2 complement
  int vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }
  //imu_data from -32k to 32k

  imu_data[3] = 2.0 * vw / 32767.0;  // convert vw from raw values to "g's"


  address = 61;// set address value for accel y value
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }
  imu_data[4] = 2.0 * vw / 32767.0; // convert vw from raw values to "g's"


  address = 63; // set addres value for accel z value;
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }
  imu_data[5] = 2.0 * vw / 32767.0;  // convert vw from raw values to g's


  address = 67;// set addres value for gyro x value;
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }

  imu_data[0] = -(x_gyro_calibration + (vw * 500.0 / 32767.0)); // convert vw from raw values to degrees/second

  address = 69; // set addres value for gyro y value;
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }
  imu_data[1] = y_gyro_calibration + (vw * 500.0 / 32767.0); // convert vw from raw values to degrees/second

  address = 71; // set addres value for gyro z value;
  vh = wiringPiI2CReadReg8(imu, address);
  vl = wiringPiI2CReadReg8(imu, address + 1);
  vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
  if (vw > 0x8000) {
    vw = vw ^ 0xffff;
    vw = -vw - 1;
  }
  imu_data[2] = z_gyro_calibration + (vw * 500.0 / 32767.0); // convert vw from raw values to degrees/second


}

void update_filter()
{

  //get current time in nanoseconds
  timespec_get(&te, TIME_UTC);
  time_curr = te.tv_nsec;
  //compute time since last execution
  float imu_diff = time_curr - time_prev;

  //check for rollover
  if (imu_diff <= 0) {
    imu_diff += 1000000000;
  }
  //convert to seconds
  imu_diff = imu_diff / 1000000000;
  time_prev = time_curr;

  //comp. filter for roll, pitch here:
}


int setup_imu()
{
  wiringPiSetup();
  //setup imu on I2C
  imu = wiringPiI2CSetup(0x68); //accel/gyro address

  if (imu == -1) {
    printf("-----cant connect to I2C device %d --------\n", imu);
    return -1;
  } else {

    printf("connected to i2c device %d\n", imu);
    printf("imu who am i is %d \n", wiringPiI2CReadReg8(imu, 0x75));

    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS


    //init imu
    wiringPiI2CWriteReg8(imu, PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu, PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04
    int c = wiringPiI2CReadReg8(imu, GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);
    c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0xE0);  // Clear self-test bits [7:5]
    wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0x18);  // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c | Ascale << 3);
    c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
    wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c & ~0x0F);  //
    wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c | 0x00);
  }
  return 0;
}

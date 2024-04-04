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
#define SAFTEY_GYRO_RATE 300
#define SAFETY_ROLL_LIM 45
#define SAFETY_PITCH_LIM 45
#define SAFETY_HEARTBEAT_TO 250

// define filter-related constants
#define A_FILTER 0.02

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
void print_milestone1();
void print_filter();
void safety_check();

// global variables
int imu;
float x_gyro_calibration = 0; // gyro calibration offsets
float y_gyro_calibration = 0;
float z_gyro_calibration = 0;
float roll_calibration = 0; // accel calibration offsets
float pitch_calibration = 0;
float accel_z_calibration = 0;
float imu_data[6]; // gyro xyz, accel xyz
long time_curr;     // time-tracking variables
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

float imu_diff_copy = 0.0;

// global variables to add
struct Keyboard
{
    char key_press;
    int heartbeat;
    int version;
};
Keyboard *shared_memory;
int run_program = 1;

// function to add
void setup_keyboard()
{

    int segment_id;
    struct shmid_ds shmbuffer;
    int segment_size;
    const int shared_segment_size = 0x6400;
    int smhkey = 33222;

    /* Allocate a shared memory segment.  */
    segment_id = shmget(smhkey, shared_segment_size, IPC_CREAT | 0666);
    /* Attach the shared memory segment.  */
    shared_memory = (Keyboard *)shmat(segment_id, 0, 0);
    printf("shared memory attached at address %p\n", shared_memory);
    /* Determine the segment's size. */
    shmctl(segment_id, IPC_STAT, &shmbuffer);
    segment_size = shmbuffer.shm_segsz;
    printf("segment size: %d\n", segment_size);
    /* Write a string to the shared memory segment.  */
    // sprintf (shared_memory, "test!!!!.");
}

// when cntrl+c pressed, kill motors
void trap(int signal)
{
    printf("ending program due to CRTL+C called\n\r");
    run_program = 0;
}

int main(int argc, char *argv[])
{

    setup_imu();
    calibrate_imu();
    setup_keyboard();
    signal(SIGINT, &trap);

    while (run_program == 1)
    {
        read_imu(); // read data from IMU, caluclate accelerometer pitch,roll

        // print_milestone1();

        update_filter(); // update complementary filter
        print_filter();

        // time
        gettimeofday(&tv, NULL);

        // safety checks
        safety_check();
    }
    return 0;
}

void safety_check()
{
    static long last_heartbeat_time = 0;
    static int last_heartbeat = 0;
    static bool firstrun = true;

    if (firstrun)
    {
        long curr_time = tv.tv_sec * 1000LL + tv.tv_usec / 1000;
        last_heartbeat_time = curr_time;
        firstrun = false;
    }

    Keyboard keyboard = *shared_memory; // to refresh values from shared memory first

    // gyros rate is too high
    if (imu_data[0] > SAFTEY_GYRO_RATE || imu_data[0] < -SAFTEY_GYRO_RATE || imu_data[1] > SAFTEY_GYRO_RATE || imu_data[1] < -SAFTEY_GYRO_RATE || imu_data[2] > SAFTEY_GYRO_RATE || imu_data[2] < -SAFTEY_GYRO_RATE)
    {
        printf("ending program due to gyro rate\n\r");
        run_program = 0;
    }
    // roll angle
    if (roll_t > SAFETY_ROLL_LIM || roll_t < -SAFETY_ROLL_LIM)
    {
        printf("ending program due to roll greater than 45 degrees\n\r");
        run_program = 0;
    }
    // pitch angle
    if (pitch_t > SAFETY_PITCH_LIM || pitch_t < -SAFETY_PITCH_LIM)
    {
        printf("ending program due to pitch greater than 45 degrees\n\r");
        run_program = 0;
    }
    // heartbeat check
    long curr_time = tv.tv_sec * 1000LL + tv.tv_usec / 1000;
    if (curr_time > last_heartbeat_time + SAFETY_HEARTBEAT_TO)
    {
        printf("ending program due to heartbeat timeout\n\r");
        run_program = 0;
    }
    // update heartbeat time if theres a new heartbeat value
    if (keyboard.heartbeat > last_heartbeat)
    {
        last_heartbeat_time = curr_time;
        last_heartbeat = keyboard.heartbeat;
    }
    // whitespace
    if (keyboard.key_press == 32)
    {
        printf("ending program due to space pressed\n\r");
        run_program = 0;
    }
}

void print_filter()
{

    printf(
        "%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f\n\r",
        roll_accel,
        pitch_accel,
        roll_t,
        pitch_t,
        roll_gyro_delta,
        pitch_gyro_delta,
        imu_diff_copy);
}

// print gx gy gz roll pitch (angles)
void print_milestone1()
{
    printf(
        "gx %10.5f gy %10.5f gz %10.5f roll %10.5f pitch %10.5f\n\r",
        imu_data[0],
        imu_data[1],
        imu_data[2],
        roll_accel,
        pitch_accel);
}

void calibrate_imu()
{
    /*
    accel_z_calibration=??
    */
    float x_gyro_offset = 0.0, y_gyro_offset = 0.0, z_gyro_offset = 0.0;
    float roll_offset = 0.0, pitch_offset = 0.0;

    float num_samples = 1000;

    for (int i = 0; i < (int)num_samples; i++)
    {
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

    if (pitch_accel > M_PI)
    {
        pitch_accel -= 2 * M_PI;
    }
    else if (pitch_accel < -M_PI)
    {
        pitch_accel += 2 * M_PI;
    }

    if (roll_accel > M_PI)
    {
        roll_accel -= 2 * M_PI;
    }
    else if (roll_accel < -M_PI)
    {
        roll_accel += 2 * M_PI;
    }

    pitch_accel *= 180.0 / M_PI;
    roll_accel *= 180.0 / M_PI;
}

void read_imu()
{
    int address = 59; // set address value for accel x value
    float ax = 0;
    float az = 0;
    float ay = 0;
    int vh, vl;

    // read in data
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    // convert 2 complement
    int vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    // imu_data from -32k to 32k

    imu_data[3] = 2.0 * vw / 32767.0; // convert vw from raw values to "g's"

    address = 61; // set address value for accel y value
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[4] = 2.0 * vw / 32767.0; // convert vw from raw values to "g's"

    address = 63; // set addres value for accel z value;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[5] = 2.0 * vw / 32767.0; // convert vw from raw values to g's

    address = 67; // set addres value for gyro x value;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }

    imu_data[0] = -(x_gyro_calibration + (vw * 500.0 / 32767.0)); // convert vw from raw values to degrees/second

    address = 69; // set addres value for gyro y value;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[1] = y_gyro_calibration + (vw * 500.0 / 32767.0); // convert vw from raw values to degrees/second

    address = 71; // set addres value for gyro z value;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[2] = z_gyro_calibration + (vw * 500.0 / 32767.0); // convert vw from raw values to degrees/second

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
    if (imu_diff <= 0)
    {
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
    imu = wiringPiI2CSetup(0x68); // accel/gyro address

    if (imu == -1)
    {
        printf("-----cant connect to I2C device %d --------\n", imu);
        return -1;
    }
    else
    {

        printf("connected to i2c device %d\n", imu);
        printf("imu who am i is %d \n", wiringPiI2CReadReg8(imu, 0x75));

        uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
        uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

        // init imu
        wiringPiI2CWriteReg8(imu, PWR_MGMT_1, 0x00);
        printf("                    \n\r");
        wiringPiI2CWriteReg8(imu, PWR_MGMT_1, 0x01);
        wiringPiI2CWriteReg8(imu, CONFIG, 0x00);
        wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); // 0x04
        int c = wiringPiI2CReadReg8(imu, GYRO_CONFIG);
        wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0xE0);
        wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
        wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);
        c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c | Ascale << 3);
        c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c & ~0x0F); //
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c | 0x00);
    }
    return 0;
}

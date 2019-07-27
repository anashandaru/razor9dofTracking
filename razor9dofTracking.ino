#include <SparkFunMPU9250-DMP.h>
#include <Filters.h>
#include "config.h"

struct vec{
  float x;
  float y;
  float z;
};

struct quat{
  float w;
  float x;
  float y;
  float z;
};

// IMU Digital Motion Processing Object Declaration
MPU9250_DMP imu;

// Stationary Detection Object Declaration
FilterOnePole highpassFilter(HIGHPASS, StadeHighpassFreq);
FilterOnePole lowpassFilter(LOWPASS, StadeLowpassFreq);

void setup(){
	SerialPort.begin(115200);

  // Call imu.begin() to verify communication with and
  // initialize the MPU-9250 to it's default values.
  // Most functions return an error code - INV_SUCCESS (0)
  // indicates the IMU was present and successfully set up
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

// Use setSensors to turn on or off MPU-9250 sensors.
  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
  //imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(GyroFSR); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(AccelFSR); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(Lpf); // Set LPF corner frequency to 5Hz

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  //imu.setSampleRate(Sps); // Set sample rate to 10Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(Sps); // Set mag rate to 10Hz
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_SEND_RAW_ACCEL, // Use gyro calibration
              100); // Set DMP FIFO rate to 10 Hz
  
}

void loop() 
{
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      printIMUData();
    }
  }
}

void printIMUData(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);

//  SerialPort.println(isStationary(accelX,accelY,accelZ));
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

  vec accel,quat,trueAccel;
  accel.x = accelX;
  accel.y = accelY;
  accel.z = accelZ;

  quat.x = q1;
  quat.y = q2;
  quat.z = q3;

  trueAccel = rotateVec2(accel, quat, q0);

  SerialPort.print(trueAccel.x); SerialPort.print(", ");
  SerialPort.print(trueAccel.y); SerialPort.print(", ");
  SerialPort.println(trueAccel.z);
    
}

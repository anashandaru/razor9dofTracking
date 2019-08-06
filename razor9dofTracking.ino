#include <SparkFunMPU9250-DMP.h>
#include <Filters.h>
#include "config.h"

struct vec{
  float x=0;
  float y=0;
  float z=0;
};

struct quat{
  float w=0;
  float x=0;
  float y=0;
  float z=0;
};

vec vel,pos;

// IMU Digital Motion Processing Object Declaration
MPU9250_DMP imu;

// Stationary Detection Object Declaration
FilterOnePole highpassFilter(HIGHPASS, StadeHighpassFreq);
FilterOnePole lowpassFilter(LOWPASS, StadeLowpassFreq);

// Filter Accelerometer data
FilterOnePole lpfX(LOWPASS, AccellpfFreq);
FilterOnePole lpfY(LOWPASS, AccellpfFreq);
FilterOnePole lpfZ(LOWPASS, AccellpfFreq);


void setup(){
  pinMode(ledPin, OUTPUT);
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
              Sps); // Set DMP FIFO rate to 10 Hz
  
}

void loop() 
{
  if ( !imu.fifoAvailable() ) return;
  if ( imu.dmpUpdateFifo() != INV_SUCCESS) return;
  
  vec accel, q, trueAccel, trueAccelF;
  accel.x = imu.calcAccel(imu.ax);
  accel.y = imu.calcAccel(imu.ay);
  accel.z = imu.calcAccel(imu.az);
  if(isStationary2(accel.x,accel.y,accel.z)){
    vel.x=0; vel.y=0; vel.z=0;
    turnLed(true);
  }
  else{
    turnLed(false);
    float w;
    w = imu.calcQuat(imu.qw);
    q.x = imu.calcQuat(imu.qx);
    q.y = imu.calcQuat(imu.qy);
    q.z = imu.calcQuat(imu.qz);

    trueAccel = rotateVec2(accel, q, w);
    trueAccelF.x = lpfX.input(trueAccel.x);
    trueAccelF.y = lpfY.input(trueAccel.y);
    trueAccelF.z = lpfZ.input(trueAccel.z);

    g2mss(trueAccelF);
    removeEarthGravity(trueAccelF);
    integrate(vel,trueAccelF,1.0/Sps);
    if(millis()>8000)integrate(pos,vel,1.0/Sps);
  }
  printVec(pos);
  //printCompare(trueAccel.z,trueAccelF.z);
}

bool feedIMU(){
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() ){
      // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
      if ( imu.dmpUpdateFifo() == INV_SUCCESS) return true;    
  }
  else return false;
  return false;
}

void printVec(vec a){
  SerialPort.print(a.x); SerialPort.print(", ");
  SerialPort.print(a.y); SerialPort.print(", ");
  SerialPort.println(a.z);
}

void printCompare(float a, float b){
  SerialPort.print(a); SerialPort.print(", ");
  SerialPort.println(b);
}

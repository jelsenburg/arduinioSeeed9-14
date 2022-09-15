#include <LSM6DS3.h>
#include <Wire.h>

// Variable definitions

#define Kp 100.0f     //Proportional gain dominance rate converges to accelerometer/magnetometer
#define Ki 0.002f     //Connection of gyroscope bias of integral gain dominance rate
#define halfT 0.001f  //Half of the sampling period

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   //The elements of the quaternion represent the estimated direction
float exInt = 0, eyInt = 0, ezInt = 0;  //Reduce the integral error proportionally
float Yaw, Pitch, Roll;                 //Yaw angle, pitch angle, roll angle


//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A
float aX, aY, aZ, gX, gY, gZ;
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;
const float accelerationThreshold = 2.5;  // threshold of significant in G's
const int numSamples = 119;
int samplesRead = numSamples;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial)
    ;
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("aX,aY,aZ,gX,gY,gZ");
  }
}
//Acceleration unit g, gyroscope rad/s
  void loop() {

  
  //Measurement normalization
  norm = sqrt(aX * aX + aY * aY + aZ * aZ);
  aX = aX / norm;  //Unitization
  aY = aY / norm;
  aZ = aZ / norm;

  //Estimate the direction of gravity
  vx = 2 * (q1 * q3 - q0 * q2);
  vy = 2 * (q0 * q1 + q2 * q3);
  vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  //The sum of the cross product between the wrong field and the direction sensor measuring the reference direction
  ex = (aY * vz - aZ * vy);
  ey = (aZ * vx - aX * vz);
  ez = (aX * vy - aY * vx);

  //integral error proportional integral gain
  exInt = exInt + ex * Ki;
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  //Adjusted gyroscope measurement
  gX = gX + Kp * ex + exInt;
  gY = gY + Kp * ey + eyInt;
  gZ = gZ + Kp * ez + ezInt;

  //Integrate quaternion rate and normalization
  q0 = q0 + (-q1 * gX - q2 * gY - q3 * gZ) * halfT;
  q1 = q1 + (q0 * gX + q2 * gZ - q3 * gY) * halfT;
  q2 = q2 + (q0 * gY - q1 * gZ + q3 * gX) * halfT;
  q3 = q3 + (q0 * gZ + q1 * gY - q2 * gX) * halfT;

  //normalize quaternion
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                 //pitch, converted to degrees
  Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;  //rollv
  //Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;//The offset is too large, wait for me to find a useful one of

  {
    // wait for significant motion
    while (samplesRead == numSamples) {
      // read the acceleration data
      aX = myIMU.readFloatAccelX();
      aY = myIMU.readFloatAccelY();
      aZ = myIMU.readFloatAccelZ();

      // sum up the absolutes
      float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

      // check if it's above the threshold
      if (aSum >= accelerationThreshold) {
        // reset the sample read count
        samplesRead = 0;
        break;
      }
    }

    // check if the all the required samples have been read since
    // the last time the significant motion was detected
    while (samplesRead < numSamples) {
      // check if both new acceleration and gyroscope data is
      // available
      // read the acceleration and gyroscope data

      samplesRead++;

      // print the data in CSV format
      Serial.print(myIMU.readFloatAccelX(), 3);
      Serial.print(',');
      Serial.print(myIMU.readFloatAccelY(), 3);
      Serial.print(',');
      Serial.print(myIMU.readFloatAccelZ(), 3);
      Serial.print(',');
      Serial.print(myIMU.readFloatGyroX(), 3);
      Serial.print(',');
      Serial.print(myIMU.readFloatGyroY(), 3);
      Serial.print(',');
      Serial.print(myIMU.readFloatGyroZ(), 3);
      Serial.print(',');
      Serial.print(" Pitch = ");
      Serial.print(asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3, 3);
      Serial.print(',');
      Serial.print(" Roll = ");
      Serial.print(atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3, 3);


      if (samplesRead == numSamples) {
        // add an empty line if it's the last sample
        Serial.println();
      }
    }
  }
}

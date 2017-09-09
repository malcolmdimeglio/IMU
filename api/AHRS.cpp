//
//  AHRS.cpp
//  
//
//  Created by Malcolm DI MEGLIO on 08/30/2017.
//
// Source https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU.git
//

#include "WProgram.h"
#include "AHRS.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include <math.h>

uint8_t AHRS::initSensors()
{
    bool init_success;

    Serial.println("+------------------------------------+");
    Serial.println("|        MPU6050 Configuration       |");
    Serial.println("+------------------------------------+\n");
    delay(1000);

    init_success = AcceleroGyro.init();

    if (!init_success) 
    {
        digitalWriteFast(9, HIGH); // Red
        Serial.println("AcceleroGyro Configuration FAIL");
        return 0;
    }

    Serial.println("+------------------------------------+");
    Serial.println("|       HMC5883L Configuration       |");
    Serial.println("+------------------------------------+\n");
    delay(1000);
    init_success = Compass.init();
    
    if (!init_success) 
    {
        digitalWriteFast(9, HIGH); // Red
        Serial.println("Compass Configuration FAIL");
        return 0;
    }

    #if CALIBRATION_NEEDED
        digitalWriteFast(10, HIGH); // Orange
        Compass.calibrate();
        digitalWriteFast(10, LOW); // Orange
    #endif

    return 1;

}

void AHRS::readSensors(void)
{
    AcceleroGyro.updateAxyzGxyz();
    Compass.updateMxyz();
}

void AHRS::getAngles(void)
{
    computeQuaternion();
    getYaw();
    getPitch();
    getRoll();
}

void AHRS::getYaw(void)
{
    Yaw = Compass.declination + (atan2(2.0 * (q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3])) * RAD_TO_DEG;

    // Due to positive or negative declination 
    if (Yaw > 360)
        Yaw -= 360
    if (Yaw < 0)
        Yaw += 360
}

void AHRS::getPitch(void)
{
    Pitch = (-asin(2.0 * (q[1]*q[3] - q[0]*[2]))) * RAD_TO_DEG;
}

void AHRS::getRoll(void)
{
    Roll = (atan2(2.0 * (q[0]*q[1] + q[2]*q[3]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])) * RAD_TO_DEG;
}
/*
// This uses the Madgwick algorithm
void AHRS::computeQuaternion(void)
{
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    float _2q1mx, _2q1my, _2q1mz, hx, hy, _2bx, _2bz, _4bx, _4bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4; // scalar
     // short name local variable for readability
    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];
    float q4 = q[3];

    // if self.start_time is None:
    //     self.start_time = micros()  # First run

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1 = 2 * q1;
    float _2q2 = 2 * q2;
    float _2q3 = 2 * q3;
    float _2q4 = 2 * q4;
    float _2q1q3 = 2 * q1 * q3;
    float _2q3q4 = 2 * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    float norm;

    ax = AcceleroGyro.Axyz[0];
    ay = AcceleroGyro.Axyz[1];
    az = AcceleroGyro.Axyz[2];

    gx = AcceleroGyro.Gxyz[0];
    gy = AcceleroGyro.Gxyz[1];
    gz = AcceleroGyro.Gxyz[2];

    mx = Compass.Mxyz[0];
    my = Compass.Mxyz[1];
    mz = Compass.Mxyz[2];

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0)
        return  // handle NaN
    norm = 1 / norm;                     // use reciprocal for division
    
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0)
        return                          // handle NaN
    norm = 1 / norm;                     // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2 * q1 * mx;
    _2q1my = 2 * q1 * my;
    _2q1mz = 2 * q1 * mz;
    _2q2mx = 2 * q2 * mx;

    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;

    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2 * _2bx;
    _4bz = 2 * _2bz;

    // Gradient descent algorithm corrective step
    s1 = (-_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) \
         + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) \
         + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz));

    s2 = (_2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az) \
         + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) \
         + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz));

    s3 = (-_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az) \
         + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) \
         + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) \
         + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz));

    s4 = (_2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) \
          + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) \
          + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz));

    norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    // normalise step magnitude
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
    qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
    qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
    qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

    // Integrate to yield quaternion
    deltat = elapsed_micros(self.start_time) / 1e6
    self.start_time = micros()
    q1 += qDot1 * deltat
    q2 += qDot2 * deltat
    q3 += qDot3 * deltat
    q4 += qDot4 * deltat
    norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    // normalise quaternion
    self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm
}
*/
AHRS Ahrs;




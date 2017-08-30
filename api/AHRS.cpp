//
//  AHRS.cpp
//  
//
//  Created by Malcolm DI MEGLIO on 08/30/2017.
//
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
    float Ax, Ay, Az, Mx, My;
    float Zrot;

    Ax = AcceleroGyro.Axyz[0];
    Ay = AcceleroGyro.Axyz[1];
    Az = AcceleroGyro.Axyz[2];
    
    Mx = Compass.Mxyz[0];
    My = Compass.Mxyz[1];

    Zrot = atan2(My,Mx) - Compass.declination;
    
    // due to addition/substraction of the declination 
    if (Zrot < 0)
        Zrot += 2*PI;
    if (Zrot > 2*PI)
        Zrot -= 2*PI;

// TODO : the ranges are wrong here. this needs to be double check and corrected
// should be 
// // YAW : [0° : 360°]
    // PITCH : [-90° : 90°]
    // ROLL : [-90° : 90°]
    // YAW : [-180° : 180°]
     
     
     // but it is
    // PITCH : [-90° : 90°]
    // ROLL : [-180° : 180°]
    Yaw = Zrot * RAD_TO_DEG;
    Pitch = atan2( Ax , sqrt (pow(Ay,2) + pow (Az,2)) ) * RAD_TO_DEG;
    Roll = atan2( Ay , Az ) * RAD_TO_DEG;
}

AHRS ahrs;




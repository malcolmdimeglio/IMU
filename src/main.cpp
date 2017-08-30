//
//  main.cpp
//  Projet 3D
//
//  Created by Malcolm DI MEGLIO on 06/26/2015.
//  Copyright (c) 2015 Malcolm DI MEGLIO. All rights reserved.
//

#define F_CPU 48000000

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>


#include "WProgram.h"
#include "TwoWire.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "AHRS.h"

using namespace std;



extern "C" int main(int argc, char *argv[]) 
{
#ifdef USING_MAKEFILE
    
    float ThetaX, ThetaY, ThetaZ;

    delay(3500);
    pinMode(12, OUTPUT); // Bleue
    pinMode(11, OUTPUT); // Green
    pinMode(10, OUTPUT); // Orange
    pinMode(9, OUTPUT); // Red
    digitalWriteFast(12, HIGH); // Bleue

    Serial.begin(115200);
    Twi.begin(); // (Here 400kHz speed) for further information check F_CPU/F_BUS in kinestis.h line 226

    ahrs.initSensors();

    digitalWriteFast(12, LOW); // Bleue
    digitalWriteFast(11, HIGH); // Green
    
    while (1)
    {
        ahrs.readSensors();
        ahrs.getAngles();

        ThetaX = ahrs.Roll;
        ThetaY = ahrs.Pitch;
        ThetaZ = ahrs.Yaw;

        Serial.printf("%.4f %.4f %.4f\n",ThetaX, ThetaY, ThetaZ);
        
        while (!Serial){}
            // TODO : /!\ Dangerous, implement timeout to exit the loop
        
    }
    
    
#endif
return 0;
}
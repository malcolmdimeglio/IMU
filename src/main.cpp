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
#include "Wire.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#define CALIBRATION_NEEDED 1



using namespace std;



extern "C" int main(void) 
{
#ifdef USING_MAKEFILE
    
    bool init_success;
    float ThetaX, ThetaY, ThetaZ;

    // string py_script_2D_path = "/Users/Malk/Desktop/Projet_Perso/Projet_3D/Python_Script/2D_plot_RealTime.py";
    // string py_script_3D_path = "/Users/Malk/Desktop/Projet_Perso/Projet_3D/Python_Script/3D_plot_RealTime.py";
    // string py_command = "python3";
    // string py_command_2D = py_command + py_script_2D_path;
    // string py_command_3D = py_command + py_script_3D_path;

    //  // Open file and write data in it and launch python scripts
    // ofstream myFile("/Users/Malk/Desktop/Projet_Perso/Projet_3D/Python_Script/IMU_data.txt", ios::out | ios::trunc);
    // system(py_command_2D.c_str()); // Python script execution
    // system(py_command_3D.c_str()); // Python script execution

    delay(3500);
    pinMode(12, OUTPUT); // Bleue
    pinMode(11, OUTPUT); // Green
    pinMode(10, OUTPUT); // Orange
    pinMode(9, OUTPUT); // Red
    digitalWriteFast(12, HIGH); // Bleue

    Serial.begin(115200);
    Twi.begin(); // (Here 400kHz speed) for further information check F_CPU/F_BUS in kinestis.h line 226
    
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

    digitalWriteFast(12, LOW); // Bleue
    digitalWriteFast(11, HIGH); // Green
    
    while (1)
    {
        AcceleroGyro.getThetaX();
        AcceleroGyro.getThetaY();
        Compass.getThetaZ();

        ThetaX = AcceleroGyro.ThetaX; // PITCH
        ThetaY = AcceleroGyro.ThetaY; // ROLL
        ThetaZ = Compass.ThetaZ;  // YAW

        Serial.printf("%.4f %.4f %.4f\n",ThetaX, ThetaY, ThetaZ);
        // if (myFile.is_open()) {
        //     myFile << ThetaX << " " << ThetaY << " " << ThetaZ << endl;
        // }
        // else
        //     Serial.println("Can't open the file");
        
        //fclose(myFile);
        while (!Serial){}
            // TODO : /!\ Dangerous, implement timeout to exit the loop
        
    }
    
    
#endif
return 0;
}
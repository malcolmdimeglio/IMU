//
//  AHRS.h
//  
//
//  Created by Malcolm DI MEGLIO on 08/30/2017.
//
//

#ifndef ____AHRS__
#define ____AHRS__

#define CALIBRATION_NEEDED 1

class AHRS
{

public:
    double Yaw, Pitch, Roll;

    uint8_t initSensors(void);
    void readSensors(void);
    void getAngles(void);

};

extern AHRS Ahrs;

#endif /* defined(____AHRS__) */
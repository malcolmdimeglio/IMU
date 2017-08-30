//
//  HMC5883L.h
//  
//
//  Created by Malcolm DI MEGLIO on 07/01/2015.
//
//

#ifndef ____HMC5883L__
#define ____HMC5883L__

#include <math.h>

#define ADRESS_HMC5883L             0x1E

#define HMC5883L_RA_CONF_REG_A      0x00
#define HMC5883L_RA_CONF_REG_B      0x01
#define HMC5883L_RA_MODE_REGISTER   0x02
#define HMC5883L_RA_XOUT_H          0x03
#define HMC5883L_RA_XOUT_L          0x04
#define HMC5883L_RA_ZOUT_H          0x05
#define HMC5883L_RA_ZOUT_L          0x06
#define HMC5883L_RA_YOUT_H          0x07
#define HMC5883L_RA_YOUT_L          0x08
#define HMC5883L_RA_STATUS_REG      0x09
#define HMC5883L_RA_ID_REG_A        0x0A
#define HMC5883L_RA_ID_REG_B        0x0B
#define HMC5883L_RA_ID_REG_C        0x0C

/**
 * Local declination http://www.magnetic-declination.com/ 
 * Declination is  the 'Error' of the magnetic field in current location.
 * EAST/WEST declination is not relative to GPS EAST/WEST position
 * 
 * Toulon : 1°26'EAST = 1.4333°
 * Paris : 0°33'EAST =  1,55°
 * Vancouver : 16° 19'EAST = 16,2666°
 */ 
#define DECLINATION_TOULON_FR       1.4333
#define DECLINATION_PARIS_FR        1.55
#define DECLINATION_VANCOUVER_CA    16.2666

#define DECLINATION DECLINATION_PARIS_FR

class HMC5883L
{
    
private:
    float Xoff, Yoff, Zoff, magneto_sensitivity;
    int16_t Mxyz_raw[3];

    void config_status(int);
    void getMxyz_raw (void);
    void getMagnetoScale(void);
    void getMxyz(void);
    int16_t getOutputRateDelay(void);
    
    
    
public:
    float Mxyz[3];
    const float declination = DECLINATION * DEG_TO_RAD;
    
    bool init(void);
    void calibrate(void);
    void updateMxyz(void);
    

};
extern HMC5883L Compass;



#endif /* defined(____HMC5883L__) */


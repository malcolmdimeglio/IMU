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

#define HMC5883L_CONF_REG_A      0x00
#define HMC5883L_CONF_REG_B      0x01
#define HMC5883L_MODE_REGISTER   0x02
#define HMC5883L_XOUT_H          0x03
#define HMC5883L_XOUT_L          0x04
#define HMC5883L_ZOUT_H          0x05
#define HMC5883L_ZOUT_L          0x06
#define HMC5883L_YOUT_H          0x07
#define HMC5883L_YOUT_L          0x08
#define HMC5883L_STATUS_REG      0x09
#define HMC5883L_ID_REG_A        0x0A
#define HMC5883L_ID_REG_B        0x0B
#define HMC5883L_ID_REG_C        0x0C

#define READ    1
#define WRITE   2
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



#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#define MIN(x,y) (((x) < (y)) ? (x) : (y))

typedef union
{
    uint8_t all_bits;
    struct
    {  
        uint8_t MS          : 2;        // Mesurement mode
        uint8_t DO          : 3;        // Output_rate
        uint8_t MA          : 2;        // Number of samples averaged (1 to 8)
        uint8_t RESERVED    : 1;        // TO CLEAR
    }Sub_Reg_A;

}HMC5883L_reg_conf_REG_A;

typedef union
{
    uint8_t all_bits;
    struct
    {  
        uint8_t RESERVED    : 5;        // TO CLEAR
        uint8_t GN          : 3;        // Gain
    }Sub_Reg_B;

}HMC5883L_reg_conf_REG_B;

typedef union
{
    uint8_t all_bits;
    struct 
    {  
        uint8_t MR          : 2;        // Operating mode
        uint8_t RESERVED    : 5;        // RESERVED
        uint8_t HS          : 1;        // I2C High speed
    }Sub_Reg_MODE;

}HMC5883L_reg_conf_REG_MODE;


typedef struct
{
    HMC5883L_reg_conf_REG_MODE Reg_MODE;
    HMC5883L_reg_conf_REG_A Reg_A;
    HMC5883L_reg_conf_REG_B Reg_B;
}HMC5883L_reg_conf;

class HMC5883L
{
    
private:
    float Xoff, Yoff, Zoff, magneto_sensitivity;
    uint16_t outputRateDelay;
    int16_t Mxyz_raw[3];

    HMC5883L_reg_conf HMC5883L_Register;

    int8_t ioctl(uint8_t, uint8_t, uint8_t);
    int8_t ioctl(uint8_t, uint8_t, uint8_t, uint8_t*);
    int8_t com_status(int8_t);
    void getMxyz_raw (void);
    void getMagnetoScale(void);
    void getMxyz(void);
    void getOutputRateDelay(void);
    
    
    
public:
    float Mxyz[3];
    const float declination = DECLINATION * DEG_TO_RAD;
    
    bool init(void);
    void calibrate(void);
    void updateMxyz(void);
    

};

extern HMC5883L Compass;



#endif /* defined(____HMC5883L__) */


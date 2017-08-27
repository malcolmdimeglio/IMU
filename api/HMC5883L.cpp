//
//  HMC5883L.cpp
//  
//
//  Created by Malcolm DI MEGLIO on 07/01/2015.
//
//

#include "WProgram.h"
#include "TwoWire.h"
#include "HMC5883L.h"
#include <math.h>


/** 
 * config_status prints out the outcome of each configuration command sent to the sensor
 *
 * @param err :"err" is the inidcator of each register configuration status
 *            0 = SUCCES
 *            1 = The received data is too long
 *            2 = Adress not received
 *            3 = Data not received
 */
void HMC5883L::config_status(int err)
{
    if (err == 0)
        Serial.println("SUCCESS");
    else if (err == 1)
        Serial.println("Data too long");
    else if (err == 2)
        Serial.println("recv addr NACK");
    else if (err == 3)
        Serial.println("recv data NACK");
    else
        Serial.println("other error");
}

/**
 * initialize all the sensor's register
 *
 * @return  [bool] Success/fail
 */
bool HMC5883L::init(void)
{
    uint8_t err;
    uint8_t t_err;

    Serial.print("Register A configuration : ");
    err = Twi.config(ADRESS_HMC5883L,HMC5883L_RA_CONF_REG_A,0x78);
    t_err = err;
    config_status(err);
    delay(200);
    
    /*
     CONF_REG_A :
     
       Hex Rg | Dec Reg |  Bit 7  |  Bit 6-5  |  Bit 4-3-2  |  Bit 1-0
       -------|---------|---------|-----------|-------------|-----------
         1B   |   27    |    0    |  MA[1:0]  |    DO[2:0]  |  MS[1:0]
     
     MA[1:0] --> Number of samples averaged (1 to 8)
        00 = 1(Default)
        01 = 2
        10 = 4
        11 = 8
     
     DO[2:0] --> Output Rate    with 8 samples and normal mesurement
        000 = 0.75Hz
        001 = 1.5Hz
        010 = 3Hz
        011 = 7.5Hz
        100 = 15Hz (Default)                0x70 (67ms delay)
        101 = 30Hz                          0x74 (34ms delay)
        110 = 75Hz                          0x78 (14ms delay)
        111 = Reserved
     
     MS[1:0] --> Mesurement mode
        00 = Normal (Default)
        01 = Positive Bias
        10 = Negative Bias
        11 =Reserved
     */
    
    Serial.print("Register B configuration : ");
    err = Twi.config(ADRESS_HMC5883L,HMC5883L_RA_CONF_REG_B,0x40);
    t_err += err;
    config_status(err);
    delay(200);
    
    /*
    
    CONF_REG_B :
     
       Hex Rg | Dec Reg |  Bit 7-6-5  |  Bit 4-3-2-1-0
       -------|---------|-------------|-----------------
         1B   |   27    |    GN[2:0]  |        0
    
    GN[2:0] --> Gain Configuration
     
                Recomanded sensor field range   Gain (LSb/Gauss)   Digital Resolution (mG/LSb)  Hex
        000 =           ± 0.88 Ga                   1370                    0.73                OxOO
        001 =           ± 1.3 Ga                    1090 (Default)          0.92                0x20
        010 =           ± 1.9 Ga                    820                     1.22                0x40
        011 =           ± 2.5 Ga                    660                     1.52                0x60
        100 =           ± 4.0 Ga                    440                     2.27                0x80
        101 =           ± 4.7 Ga                    390                     2.56                0xA0
        110 =           ± 5.6 Ga                    330                     3.03                0xC0
        111 =           ± 8.1 Ga                    230                     4.35                0xE0
    */
   
    Serial.print("Mode register configuration : ");
    err = Twi.config(ADRESS_HMC5883L,HMC5883L_RA_MODE_REGISTER,0x00);
    t_err += err;
    config_status(err);
    delay(200); // delay to help reading the configuration outcome on the terminal. Not absolutly needed
   
    /* 
     MODE_REGISTER :
     
       Hex Rg | Dec Reg |  Bit 7  |  Bit 6-5-4-3-2  |   Bit 1-0
       -------|---------|---------|-----------------|-------------
         1B   |   27    |    HS   |         0       |    MR[1:0]
     
     HS --> High Speed Register
     1 = Enable High Speed I2C, 3400kHz
     
     MR[1:0] --> Operating Mode
     00 = Continuous mode
     01 = Single mesurement
     10 = idle
     11 = idle
     */
    
    delay(10); // minimum 6ms delay needed after configuration and before reading
    
    if (t_err != 0) 
    {
        Serial.println("\n   !!!!!!   HMC5883L Configuration : ERROR   !!!!!!\n");
        delay(2000);
        return false;
    }
    else 
    {
        delay(700);
        Serial.println("\n   >>>>>   HMC5883L Configuration : SUCCESS   <<<<<\n");
        getMagnetoScale();
        delay(1500);
        return true;
    }
}

/**
 * According to the gain set up in the init process, "getMagnetoScale" gives the appropriate scale for further calculation
 * 
 * @return  [float]   The apropriate scale
 */
void HMC5883L::getMagnetoScale(void)
{
    uint8_t gain;
    uint8_t mask = 0b11100000;
    uint8_t* twi_read_value = NULL;

    twi_read_value = Twi.readFrom(ADRESS_HMC5883L, HMC5883L_RA_CONF_REG_B, 1, twi_read_value);
    if (twi_read_value == NULL)
    {
        printf("Couldn't allow memory, shuting down...");
        exit(EXIT_FAILURE);
    }

    gain = *twi_read_value;
    gain = (gain & mask) >> 5;
    
    free(twi_read_value);

    if (gain == 0) magneto_sensitivity = 0.73;
    if (gain == 1) magneto_sensitivity = 0.92;
    if (gain == 2) magneto_sensitivity = 1.22;
    if (gain == 3) magneto_sensitivity = 1.52;
    if (gain == 4) magneto_sensitivity = 2.27;
    if (gain == 5) magneto_sensitivity = 2.56;
    if (gain == 6) magneto_sensitivity = 3.03;
    if (gain == 7) magneto_sensitivity = 4.35;
    
}

/**
 * According to the frequency set up in the init process, getOutputRateDelay gives the delay to wait between each sample
 * 
 * @return  [int16_t]   Delay in ms to wait between each sample
 */
int16_t HMC5883L::getOutputRateDelay(void)
{
    uint8_t rate;
    uint8_t mask = 0x1C; // 0b00011100
    uint8_t* twi_read_value = NULL;

    twi_read_value = Twi.readFrom(ADRESS_HMC5883L, HMC5883L_RA_CONF_REG_A, 1, twi_read_value);
    if (twi_read_value == NULL)
    {
        printf("Couldn't allow memory, shuting down...");
        exit(EXIT_FAILURE);
    }

    rate = *twi_read_value;
    rate = (rate & mask) >> 2;

    free(twi_read_value);
    
    if (rate == 0) return 1334;     // 000 = 0.75Hz // delay in ms to wait between each sample
    if (rate == 1) return 667;      // 001 = 1.5Hz
    if (rate == 2) return 334;      // 010 = 3Hz
    if (rate == 3) return 134;      // 011 = 7.5Hz
    if (rate == 4) return 67;       // 100 = 15Hz (default)
    if (rate == 5) return 34;       // 101 = 30Hz
    if (rate == 6) return 14;       // 110 = 75Hz
    
    else return -1;
}

/**
 *  Initialte a sensor calibration to get x,y,y offsets
 */
void HMC5883L::calibrate(void)
{
    int16_t Xmin = 0;
    int16_t Xmax = 0;
    int16_t Ymin = 0;
    int16_t Ymax = 0;
    int16_t Zmin = 0;
    int16_t Zmax = 0;
    uint8_t cal_time = 20; // in seconds
    
    Serial.printf("Calibration : move the sensor on all three axes for %u \n", cal_time);
    delay(2000);
    Serial.println("!!! GO !!!");
    delay(500);
    for (uint16_t i=0; i<500; i++)
    {
        getMxyz_raw();
        
        if (i<100) {
            // DO NOTHING
            // get rid of the first values that might be really off
        }
        if (Mxyz_raw[0] > Xmax) Xmax = Mxyz_raw[0];
        if (Mxyz_raw[0] < Xmin) Xmin = Mxyz_raw[0];

        if (Mxyz_raw[1] > Ymax) Ymax = Mxyz_raw[1];
        if (Mxyz_raw[1] < Ymin) Ymin = Mxyz_raw[1];

        if (Mxyz_raw[2] > Zmax) Zmax = Mxyz_raw[2];
        if (Mxyz_raw[2] < Zmin) Zmin = Mxyz_raw[2];
        
    }
    
    Xoff = (Xmin + Xmax)/2;
    Yoff = (Ymin + Ymax)/2;
    Zoff = (Zmin + Zmax)/2;
    
    Serial.printf("Xoff = %d \t Yoff = %d \t Zoff = %d\n", Xoff, Yoff, Zoff);
    Serial.println("Calibration DONE\n");
    delay(1000);
}

/**
 *  Reads the raw magnetic field value sent by the sensor
 */
void HMC5883L::getMxyz_raw (void)
{
    
    uint8_t MxH, MxL, MyH, MyL, MzH, MzL;
    int16_t Mx_raw, My_raw, Mz_raw;
    uint8_t* twi_read_value = NULL;

    twi_read_value = Twi.readFrom(ADRESS_HMC5883L, HMC5883L_RA_XOUT_H, 6, twi_read_value);
    if (twi_read_value == NULL)
    {
        printf("Couldn't allow memory, shuting down...");
        exit(EXIT_FAILURE);
    }
 
    MxH = twi_read_value[0];
    MxL = twi_read_value[1];
    MzH = twi_read_value[2];
    MzL = twi_read_value[3];
    MyH = twi_read_value[4];
    MyL = twi_read_value[5];
    
    
    Mx_raw = (MxH << 8) + MxL;
    My_raw = (MyH << 8) + MyL;
    Mz_raw = (MzH << 8) + MzL;

    
    if (Mx_raw >= 0x8000) 
        Mx_raw = -((65535 - Mx_raw) + 1);
    if (My_raw >= 0x8000) 
        My_raw = -((65535 - My_raw) + 1);
    if (Mz_raw >= 0x8000) 
        Mz_raw = -((65535 - Mz_raw) + 1);
    
   
    Mxyz_raw[0] = Mx_raw;
    Mxyz_raw[1] = My_raw;
    Mxyz_raw[2] = Mz_raw;
    
    free(twi_read_value);
    delay(getOutputRateDelay()); // see init:Register A:Output rate + getOutputRateDelay()  
}

/**
 *  Converts the raw values into usable data
 */
void HMC5883L::getMxyz(void)
{
    float Mx,My,Mz;
    
    Mx = (Mxyz_raw[0] - Xoff) * magneto_sensitivity;
    My = (Mxyz_raw[1] - Yoff) * magneto_sensitivity;
    Mz = (Mxyz_raw[2] - Zoff) * magneto_sensitivity;
    
    Mxyz[0] = Mx;
    Mxyz[1] = My;
    Mxyz[2] = Mz;   
}

/**
 *  Converts the raw values into usable data
 */
void HMC5883L::updateMxyz(void)
{
    getMxyz_raw();
    getMxyz();
}

/**
 *  Uses the magnetic field data to calculate the Yaw value, takes declination into consideration
 * 
 * @return  [double]   Yaw in degrees
 */
void HMC5883L::getYaw(void)
{
    float Zrot, x, y;
    
    x = Mxyz[0]; // x is scaled in function getMxyz
    y = Mxyz[1]; // y is scaled in function getMxyz

    Zrot = atan2(y,x) - declination;
    
    // due to addition/substraction of the declination 
    if (Zrot < 0)
        Zrot += 2*PI;
    if (Zrot > 2*PI)
        Zrot -= 2*PI;
    
    Yaw = Zrot * RAD_TO_DEG;
}


HMC5883L Compass;

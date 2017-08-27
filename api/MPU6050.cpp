//
//  MPU6050.cpp
//  
//
//  Created by Malcolm DI MEGLIO on 06/29/2015.
//
//
#include "WProgram.h"
#include "TwoWire.h"
#include "MPU6050.h"
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
void MPU6050::config_status(int err)
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
 * Initialize all the sensor's register
 * 
 * @return  [bool]  Init Success/Fail
 */
bool MPU6050::init(void)
{
    uint8_t err;
    uint8_t t_err;
    
    Serial.print("Reset device : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_PWR_MGMT_1, 0x80);
    t_err = err;
    config_status(err);
    delay(300);
    
    Serial.print("Select Clock Source : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_PWR_MGMT_1, 0x03);
    t_err += err;
    // CLKSEL = 3 : Selects clock source PLL with Z axis gyroscope
    // TEMP_DIS = 0 : Temperature sensor enable
    config_status(err);
    delay(300);
    
    Serial.print("Config Low Pass Filter : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_CONFIG, 0x01);
    t_err += err;
    // DLPF_CFG = 0x00 : Fs = 8kHz(Gyro Output Rate), bandwidth = 256Hz
    // DLPF_CFG = 0x01 : Fs = 1kHz(Gyro Output Rate), bandwidth = 42Hz
    config_status(err);
    delay(300);
    
    Serial.print("Sample rate : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_SMPLRT_DIV, 0x00);
    t_err += err;

    /*
        SMPLRT_DIV = 0 : Sample Rate = Gyroscope Output Rate(DLPF_CFG = 0x00 : 8kHz) / (1 + SMPLRT_DIV) >> sample = 8kHz / 125µs
        SMPLRT_DIV = 0 : Sample Rate = Gyroscope Output Rate(DLPF_CFG = 0x01 : 1kHz) / (1 + SMPLRT_DIV) >> sample = 1kHz / 1ms
        SMPLRT_DIV = 1 : Sample Rate = Gyroscope Output Rate(DLPF_CFG = 0x01 : 1kHz) / (1 + SMPLRT_DIV) >> sample = 500Hz / 2m
    */
   
    config_status(err);
    delay(300);
    
    Serial.print("Gyro Setting : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_GYRO_CONFIG, 0x08);
    t_err += err;
    
    /*
         GYRO_CONFIG :

         Hex Rg | Dec Reg |  Bit 7  |  Bit 6  |  Bit 5  |   Bit 4-3     | Bit 2-1-0
        --------|---------|---------|---------|---------|---------------|-------
           1B   |   27    |  XG_ST  |  YG_ST  |  ZG_ST  |  FS_SEL[1:0]  | RESERVED
    
         XG_ST = YG_ST = ZG_ST = 0, GFS_SEL = 3 : Gyro Full scale range : ± 2000 °/s > 0x18
         XG_ST = YG_ST = ZG_ST = 0, GFS_SEL = 2 : Gyro Full scale range : ± 1000 °/s > 0x10
         XG_ST = YG_ST = ZG_ST = 0, GFS_SEL = 1 : Gyro Full scale range : ± 500 °/s > 0x08
         XG_ST = YG_ST = ZG_ST = 0, GFS_SEL = 0 : Gyro Full scale range : ± 250 °/s > 0x00

    */
   
    config_status(err);
    delay(300);
    
    Serial.print("Accelero Setting : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_ACCEL_CONFIG, 0x08);
    t_err += err;
    
    /*
         ACCEL_CONFIG :

         Hex Rg | Dec Reg |  Bit 7  |  Bit 6  |  Bit 5  |   Bit 4-3     | Bit 2-1-0
        --------|---------|---------|---------|---------|---------------|-------
           1B   |   27    |  XA_ST  |  YA_ST  |  ZA_ST  |  AFS_SEL[1:0] | RESERVED
        
         XA_ST = YA_ST = ZA_ST = 0, AFS_SEL = 3 : Accelero Full scale range : ± 16g > 0x18
         XA_ST = YA_ST = ZA_ST = 0, AFS_SEL = 2 : Accelero Full scale range : ± 8g > 0x10
         XA_ST = YA_ST = ZA_ST = 0, AFS_SEL = 1 : Accelero Full scale range : ± 4g > 0x08
         XA_ST = YA_ST = ZA_ST = 0, AFS_SEL = 0 : Accelero Full scale range : ± 2g > 0x00
    */
   
    config_status(err);
    delay(300);
    
    Serial.print("Disable interrupt : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_INT_ENABLE, 0X00);
    t_err += err;
    config_status(err);
    delay(300);
    
    Serial.print("Disable FIFO : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_FIFO_EN, 0X00);
    t_err += err;
    config_status(err);
    delay(300);
    
    Serial.print("Disable I2C master modes : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_I2C_MST_CTRL, 0X00);
    t_err += err;
    config_status(err);
    delay(300);
    
    Serial.print("Disable FIFO & I2C master modes : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_USER_CTRL, 0X00);
    t_err += err;
    config_status(err);
    delay(300);
    
    Serial.print("Reset FIFO & DMP : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_INT_ENABLE, 0X0C);
    t_err += err;
    config_status(err);
    delay(300);
    

    // Not sure this is a usefull configuration to do.
    Serial.print("Reset Gyro & Accelero : ");
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_SIGNAL_PATH_RESET, 0x07);
    t_err += err;
    // GYRO_RESET = ACCEL_RESET = TEMP_RESET = 1 : Resets all three sensors
    config_status(err);
    delay(300);

    if (t_err != 0)
    {
        Serial.println("\n   !!!!!!   MPU6050 Configuration : ERROR   !!!!!!\n");
        delay(2000);
        return false;
    }
    else 
    {
        delay(700);
        Serial.println("\n   >>>>>   MPU6050 Configuration : SUCCESS   <<<<<\n");

        // We want to initialize the scales once and for all
        getGyroScale();
        getAcceleroScale();

        delay(1500);
        return true;
    }       
}

/**
 * Read the raw acceleration value on X axis
 * 
 */
void MPU6050::getAxyz_raw(void)
{
    uint8_t AxH, AxL, AyH, AyL, AzH, AzL; 
    int16_t Ax_raw, Ay_raw, Az_raw;
    uint8_t* twi_read_value = NULL;

    twi_read_value = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_ACCEL_XOUT_H, 6, twi_read_value);
    if (twi_read_value == NULL)
    {
        printf("Couldn't allow memory, shuting down...");
        exit(EXIT_FAILURE);
    }

    AxH = twi_read_value[0];
    AxL = twi_read_value[1];
    AyH = twi_read_value[2];
    AyL = twi_read_value[3];
    AzH = twi_read_value[4];
    AzL = twi_read_value[5];

    free (twi_read_value);
    
    Ax_raw = (AxH << 8) + AxL;
    Ay_raw = (AyH << 8) + AyL;
    Az_raw = (AzH << 8) + AzL;

    // conversion for 2's complement
    if (Ax_raw >= 32768)
        Ax_raw = -((65535 - Ax_raw) + 1);
    if (Ay_raw >= 32768)
        Ay_raw = -((65535 - Ay_raw) + 1 );
    if (Az_raw >= 32768)
        Az_raw = -((65535 - Az_raw) + 1);
    
    Axyz_raw[0] = Ax_raw;
    Axyz_raw[1] = Ay_raw;
    Axyz_raw[2] = Az_raw;
}

/**
 * Read the raw Gyro value on X axis
 * 
 */
void MPU6050::getGxyz_raw(void)
{
    uint8_t GxH, GxL, GyH, GyL, GzH, GzL;
    int16_t Gx_raw, Gy_raw, Gz_raw;
    uint8_t* twi_read_value = NULL;

    twi_read_value = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_GYRO_XOUT_H, 6, twi_read_value);
    if (twi_read_value == NULL)
    {
        printf("Couldn't allow memory, shuting down...");
        exit(EXIT_FAILURE);
    }

    GxH = twi_read_value[0];
    GxL = twi_read_value[1];
    GyH = twi_read_value[2];
    GyL = twi_read_value[3];
    GzH = twi_read_value[4];
    GzL = twi_read_value[5];

    free (twi_read_value);

    Gx_raw = (GxH << 8) + GxL;
    Gy_raw = (GyH << 8) + GyL;
    Gz_raw = (GzH << 8) + GzL;

    // conversion for 2's complement
    if (Gx_raw >= 32768)
        Gx_raw = -((65535 - Gx_raw) + 1);
    if (Gy_raw >= 32768)
        Gy_raw = -((65535 - Gy_raw) + 1);
    if (Gz_raw >= 32768)
        Gz_raw = -((65535 - Gz_raw) + 1);

    Gxyz_raw[0] = Gx_raw;
    Gxyz_raw[1] = Gy_raw;
    Gxyz_raw[2] = Gz_raw;
}

/**
 * According to the range set up in the init process, "getGyroScale" gives the appropriate scale for further calculation
 * 
 * @return  [float]   The apropriate scale
 */
void MPU6050::getGyroScale(void)
{
    uint8_t gyro_scale;
    uint8_t mask_fs_sel = 0b00011000;
    uint8_t* twi_read_value = NULL;

    twi_read_value = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_GYRO_CONFIG,1 , twi_read_value);
    if (twi_read_value == NULL)
    {
        printf("Couldn't allow memory, shuting down...");
        exit(EXIT_FAILURE);
    }

    gyro_scale = *twi_read_value;
    gyro_scale = (gyro_scale & mask_fs_sel) >> 3;

    if (gyro_scale == 0) gyro_sensitivity = 131.0000;
    if (gyro_scale == 1) gyro_sensitivity = 65.5000;
    if (gyro_scale == 2) gyro_sensitivity = 32.7500;
    if (gyro_scale == 3) gyro_sensitivity = 16.3750;

    free (twi_read_value);
}

/**
 * According to the range set up in the init process, "getAcceleroScale" gives the appropriate scale for further calculation
 * 
 * @return  [float]   The apropriate scale
 */
void MPU6050::getAcceleroScale(void)
{
    uint8_t accelero_scale;
    uint8_t mask_fs_sel = 0b00011000;
    uint8_t* twi_read_value = NULL;

    twi_read_value = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_ACCEL_CONFIG, 1, twi_read_value);
    if (twi_read_value == NULL)
    {
        printf("Couldn't allow memory, shuting down...");
        exit(EXIT_FAILURE);
    }

    accelero_scale = *twi_read_value;
    accelero_scale = (accelero_scale & mask_fs_sel) >> 3;

    if (accelero_scale == 0) accelero_sensitivity = 16384.0000;
    if (accelero_scale == 1) accelero_sensitivity = 8192.0000;
    if (accelero_scale == 2) accelero_sensitivity = 4096.0000;
    if (accelero_scale == 3) accelero_sensitivity = 2048.0000;

    free (twi_read_value);
}

/**
 * Converts the X, Y, Z axis Accelormeter raw value into usable data
 * 
 */
void MPU6050::getAxyz(void)
{
    Axyz[0] = Axyz_raw[0] / accelero_sensitivity;
    Axyz[1] = Axyz_raw[1] / accelero_sensitivity;
    Axyz[2] = Axyz_raw[2] / accelero_sensitivity;
}

/**
 * Converts the X, Y, Z axis Gyroscope raw value into usable data
 * 
 */
void MPU6050::getGxyz(void)
{
    Gxyz[0] = Gxyz_raw[0] / gyro_sensitivity;
    Gxyz[1] = Gxyz_raw[1] / gyro_sensitivity;
    Gxyz[2] = Gxyz_raw[2] / gyro_sensitivity;
}

void MPU6050::updateAxyzGxyz(void)
{
    getAxyz_raw();
    getGxyz_raw();

    getAxyz();
    getGxyz();
}

/**
 *  Uses the Acceleration data to calculate the Roll value
 *  ROLL : [-180° : 180°]
 * 
 */
void MPU6050::getRoll(void)
{
    float Ay, Az;

    Ay = Axyz[1];
    Az = Axyz[2];
    
    Roll = atan2( Ay , Az ) * RAD_TO_DEG;

}

/**
 *  Uses the Gyro data to calculate the Pitch value
 *  PITCH : [-90° : 90°]
 * 
 */
void MPU6050::getPitch(void)
{
    float Ax, Ay, Az;

    Ax = Axyz[0];
    Ay = Axyz[1];
    Az = Axyz[2];

    Pitch = atan2( Ax , sqrt (pow(Ay,2) + pow (Az,2)) ) * RAD_TO_DEG;
    
}

MPU6050 AcceleroGyro;



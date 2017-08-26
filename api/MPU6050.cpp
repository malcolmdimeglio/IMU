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
    err = Twi.config(ADRESS_MPU6050, MPU6050_RA_SMPRT_DIV, 0x00);
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
        delay(1500);
        return true;
    }       
}

/**
 * Read the raw acceleration value on X axis
 * 
 * @return  [int16_t]   RAW X axis Acceleration
 */
int16_t MPU6050::getAx_raw(void)
{
    uint8_t AxH = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_ACCEL_XOUT_H,1);
    uint8_t AxL = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_ACCEL_XOUT_L,1);
    int16_t Ax_raw = (AxH << 8) + AxL;
    if (Ax_raw >= 0x8000){
        Ax_raw = -((65535 - Ax_raw) + 1);
    }
    return Ax_raw;
}

/**
 * Read the raw acceleration value on Y axis
 * 
 * @return  [int16_t]   RAW Y axis Acceleration
 */
int16_t MPU6050::getAy_raw(void)
{
    uint8_t AyH = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_ACCEL_YOUT_H,1);
    uint8_t AyL = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_ACCEL_YOUT_L,1);
    int16_t Ay_raw = (AyH << 8) + AyL;
    if (Ay_raw >= 0x8000)
    {
        Ay_raw = -((65535 - Ay_raw) + 1);
    }
    return Ay_raw;
}

/**
 * Read the raw acceleration value on Z axis
 * 
 * @return  [int16_t]   RAW Z axis Acceleration
 */
int16_t MPU6050::getAz_raw(void)
{
    uint8_t AzH = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_ACCEL_ZOUT_H,1);
    uint8_t AzL = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_ACCEL_ZOUT_L,1);
    int16_t Az_raw = (AzH << 8) + AzL;
    if (Az_raw >= 0x8000)
    {
        Az_raw = -((65535 - Az_raw) + 1);
    }
    return Az_raw;
}

/**
 * Read the raw Gyro value on X axis
 * 
 * @return  [int16_t]   RAW X axis Gyro
 */
int16_t MPU6050::getGx_raw(void)
{
    uint8_t GxH = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_GYRO_XOUT_H,1);
    uint8_t GxL = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_GYRO_XOUT_L,1);
    int16_t Gx_raw = (GxH << 8) + GxL;
    if (Gx_raw >= 0x8000)
    {
        Gx_raw = -((65535 - Gx_raw) + 1);
    }
    return Gx_raw;
}

/**
 * Read the raw Gyro value on Y axis
 * 
 * @return  [int16_t]   RAW Y axis Gyro
 */
int16_t MPU6050::getGy_raw(void)
{
    uint8_t GyH = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_GYRO_YOUT_H,1);
    uint8_t GyL = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_GYRO_YOUT_L,1);
    int16_t Gy_raw = (GyH << 8) + GyL;
    if (Gy_raw >= 0x8000)
    {
        Gy_raw = -((65535 - Gy_raw) + 1);
    }
    return Gy_raw;
}

/**
 * Read the raw Gyro value on Z axis
 * 
 * @return  [int16_t]   RAW Z axis Gyro
 */
int16_t MPU6050::getGz_raw(void)
{
    uint8_t GzH = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_GYRO_ZOUT_H,1);
    uint8_t GzL = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_GYRO_ZOUT_L,1);
    int16_t Gz_raw = (GzH << 8) + GzL;
    if (Gz_raw >= 0x8000)
    {
        Gz_raw = -((65535 - Gz_raw) + 1);
    }
    return Gz_raw;
}

/**
 * According to the range set up in the init process, "getGyroScale" gives the appropriate scale for further calculation
 * 
 * @return  [float]   The apropriate scale
 */
float MPU6050::getGyroScale(void)
{
    uint8_t gyro_scale;
    uint8_t mask_fs_sel = 0x18;  // 0b00011000;
    
    gyro_scale = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_GYRO_CONFIG,1);
    gyro_scale = (gyro_scale & mask_fs_sel) >> 3;

    if (gyro_scale == 0) return 131.0000;
    if (gyro_scale == 1) return 65.5000;
    if (gyro_scale == 2) return 32.7500;
    if (gyro_scale == 3) return 16.3750;
    
    else return -1;
}

/**
 * According to the range set up in the init process, "getAcceleroScale" gives the appropriate scale for further calculation
 * 
 * @return  [float]   The apropriate scale
 */
float MPU6050::getAcceleroScale(void)
{
    uint8_t acelero_scale;
    uint8_t mask_fs_sel = 0x18;  // 0b00011000;
    
    acelero_scale = Twi.readFrom(ADRESS_MPU6050, MPU6050_RA_ACCEL_CONFIG,1);
    acelero_scale = (acelero_scale & mask_fs_sel) >> 3;

    if (acelero_scale == 0) return 16384.0000;
    if (acelero_scale == 1) return 8192.0000;
    if (acelero_scale == 2) return 4096.0000;
    if (acelero_scale == 3) return 2048.0000;
    
    else return -1;
}

/**
 * Converts the X axis Accelormeter raw value into usable data
 * 
 * @return  [float]   X axis Acceleration in G
 */
float MPU6050::getAx(void)
{
    int16_t Ax_raw;
    float accelero_sensitivity, Ax;
    
    Ax_raw = getAx_raw();
    accelero_sensitivity = getAcceleroScale();
    Ax = Ax_raw / accelero_sensitivity;
    return Ax;
}

/**
 * Converts the Y axis Accelormeter raw value into usable data
 * 
 * @return  [float]   Y axis Acceleration in G
 */
float MPU6050::getAy(void)
{
    int16_t Ay_raw;
    float accelero_sensitivity, Ay;
    
    Ay_raw = getAy_raw();
    accelero_sensitivity = getAcceleroScale();
    Ay = Ay_raw / accelero_sensitivity;
    return Ay;
}

/**
 * Converts the Z axis Accelormeter raw value into usable data
 * 
 * @return  [float]   Z axis Acceleration in G
 */
float MPU6050::getAz(void)
{
    int16_t Az_raw;
    float accelero_sensitivity, Az;
    
    Az_raw = getAz_raw();
    accelero_sensitivity = getAcceleroScale();
    Az = Az_raw / accelero_sensitivity;
    return Az;
}

/**
 * Converts the X axis Gyroscope raw value into usable data
 * 
 * @return  [float]   X axis Rotation in °/s
 */
float MPU6050::getGx(void)
{
    int16_t Gx_raw;
    float gyro_sensitivity, Gx;
    
    Gx_raw = getGx_raw();
    gyro_sensitivity = getGyroScale();
    Gx = Gx_raw / gyro_sensitivity;
    return Gx;
}

/**
 * Converts the X axis Gyroscope raw value into usable data
 * 
 * @return  [float]   Y axis Rotation in °/s
 */
float MPU6050::getGy(void)
{
    int16_t Gy_raw;
    float gyro_sensitivity, Gy;
    
    Gy_raw = getGy_raw();
    gyro_sensitivity = getGyroScale();
    Gy = Gy_raw / gyro_sensitivity;
    return Gy;
}

/**
 * Converts the X axis Gyroscope raw value into usable data
 * 
 * @return  [float]   Z axis Rotation in °/s
 */
float MPU6050::getGz(void)
{
    int16_t Gz_raw;
    float gyro_sensitivity, Gz;
    
    Gz_raw = getGz_raw();
    gyro_sensitivity = getGyroScale();
    Gz = Gz_raw / gyro_sensitivity;
    return Gz;
}

/**
 *  Uses the Acceleration data to calculate the Roll value
 *  ROLL : [-180° : 180°]
 * 
 * @return  [double]   Roll in degrees
 */
void MPU6050::getThetaY(void)
{
    float Ax, Az;

    Ax = getAx();
    Az = getAz();
    
    ThetaY = (atan2(-Ax , Az)) * RAD_TO_DEG; // DEG_TO_RAD defined in wiring.h
    
}

/**
 *  Uses the Gyro data to calculate the Pitch value
 *  PITCH : [-90° : 90°]
 * 
 * @return  [double]   Pitch in degrees
 */
void MPU6050::getThetaX(void)
{
    float Ax, Ay, Az, r;
    
    Ax = getAx();
    Ay = getAy();
    Az = getAz();
    r = sqrt(Ax*Ax + Az*Az);

    ThetaX = (atan2(Ay , r)) * RAD_TO_DEG;
    
}


MPU6050 AcceleroGyro;



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
 *  Ioctl fucntion is overloaded.
 *  Reads or Writes depending on the given type and the amount of parameters.
 *  size in byte
 * 
 */
int8_t MPU6050::ioctl(uint8_t type, uint8_t address_reg, uint8_t size, uint8_t* data)
{
    int8_t err;
    uint8_t size_data;
    switch(type)
    {
        case READ:
            size_data = sizeof(data) / sizeof (uint8_t);
            if (size != size_data)
            {
                Serial.println("array size is wrong");
                return -1;
            }

            err = Twi.readFrom(ADRESS_MPU6050, address_reg, size, data);
            break;
        default:
            Serial.printf("Wrong ioctl type\n");
            return -1;
            break;
    }
}

int8_t MPU6050::ioctl(uint8_t type, uint8_t address_reg, uint8_t full_reg)
{
    switch(type)
    {
        case WRITE:
            return com_status( Twi.config(ADRESS_MPU6050, address_reg, full_reg) );
            break;
        default:
            Serial.printf("Wrong ioctl type\n");
            return -1;
            break;
    }
}

/** 
 * config_status prints out the outcome of each configuration command sent to the sensor
 *
 * @param err :"err" is the inidcator of each register configuration status
 *            0 = SUCCES
 *            1 = The received data is too long
 *            2 = Adress not received
 *            3 = Data not received
 */
int8_t MPU6050::com_status(uint8_t err)
{
    if (err == 0)
    {
        Serial.println("SUCCESS");
        return 0;
    }
    else if (err == 1)
        Serial.println("Data too long");
    else if (err == 2)
        Serial.println("recv addr NACK");
    else if (err == 3)
        Serial.println("recv data NACK");
    else
        Serial.println("other error");

    return -1;
}

/**
 * Initialize all the sensor's register
 * 
 * @return  [bool]  Init Success/Fail
 */
bool MPU6050::init(void)
{
    int8_t err = 0;
    
    Serial.print("Reset device : ");
    /*
     PWR_MGMT_1 :
     
       Hex Rg | Dec Reg |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2-1-0  |
       -------|---------|---------|---------|---------|---------|---------|-------------|
         6B   |   107   | DEVICE_ |  SLEEP  |  CYCLE  |    -    |  TEMP_  | CLKSEL[2:0] |
              |         |   RESET |         |         |         |    DIS  |             |

        DEVICE_RESET      
        When set to 1, this bit resets all internal registers to their default values
        The bit automatically clears to 0 once the reset is done

        SLEEP
        When set to 1, this bit puts the MPU-6050 into sleep mode
        When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle 
        between sleep mode and waking up to take a single sample of data from 
        active sensors at a rate determined by LP_WAKE_CTRL (register 108).

        TEMP_DIS
        When set to 1, this bit disables the temperature sensor

        CLKSEL -> Clock Source
        0 = Internal 8MHz oscillator
        1 = PLL with X axis gyroscope reference
        2 = PLL with Y axis gyroscope reference
        3 = PLL with Z axis gyroscope reference
        4 = PLL with external 32.768kHz reference
        5 = PLL with external 19.2MHz reference
        6 = Reserved
        7 = Stops the clock and keeps the timing generator in reset

        Upon power up, the MPU-60X0 clock source defaults to the internal oscillator. 
        However, it is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source) 
        as the clock reference for improved stability.

    */
   
    MPU6050_Register.Reg_PWR_MGMT_1.Sub_Reg_PWR_MGMT_1.DEVICE_RESET = 1;
    MPU6050_Register.Reg_PWR_MGMT_1.Sub_Reg_PWR_MGMT_1.SLEEP = 0;
    MPU6050_Register.Reg_PWR_MGMT_1.Sub_Reg_PWR_MGMT_1.CYCLE = 0;
    MPU6050_Register.Reg_PWR_MGMT_1.Sub_Reg_PWR_MGMT_1.TEMP_DIS = 0;
    MPU6050_Register.Reg_PWR_MGMT_1.Sub_Reg_PWR_MGMT_1.CLKSEL = 0;
   
    err += ioctl(WRITE, MPU6050_PWR_MGMT_1, MPU6050_Register.Reg_PWR_MGMT_1.all_bits);
    
    MPU6050_Register.Reg_PWR_MGMT_1.Sub_Reg_PWR_MGMT_1.DEVICE_RESET = 0; // After a reset, the Reset bit clears itself. Here we update the class acordingly
    delay(300);
    
    Serial.print("Disable Temperature Sensor : ");
    MPU6050_Register.Reg_PWR_MGMT_1.Sub_Reg_PWR_MGMT_1.TEMP_DIS = 1; //Temperature sensor disable

    err += ioctl(WRITE, MPU6050_PWR_MGMT_1, MPU6050_Register.Reg_PWR_MGMT_1.all_bits);
    delay(300);
    
    Serial.print("Config Low Pass Filter : ");
    /*
    CONFIG :
        This register configures the external Frame Synchronization (FSYNC) pin sampling 
        and the Digital Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers.

        Hex Rg | Dec Reg |  Bit 7  |  Bit 6  |     Bit 5-4-3     |   Bit 2-1-0   |
        -------|---------|---------|---------|-------------------|---------------|
          1A   |   26    |    -    |    -    | EXT_SYNC_SET[2:0] | DLPF_CFG[2:0] |


        EXT_SYNC_SET [2:0] -> Configures the FSYNC pin sampling.

         EXT_SYNC_SET |  FSYNC Bit Location
        --------------|---------------------
              0       |    Input disabled
              1       |    TEMP_OUT_L[0]
              2       |    GYRO_XOUT_L[0]
              3       |    GYRO_YOUT_L[0]
              4       |    GYRO_ZOUT_L[0]
              5       |    ACCEL_XOUT_L[0]
              6       |    ACCEL_YOUT_L[0]
              7       |    ACCEL_ZOUT_L[0]

        DLPF_CFG [2:0] -> Digital Low Pass Filter Configuration

        Accelerometer : Fs = 1kHz                           Gyroscope :
         DLPF_CFG | Bandwidth (Hz) | Delay (ms)             DLPF_CFG | Bandwidth (Hz) | Delay (ms) | Fs (kHz)
        ----------|----------------|-----------            ----------|----------------|------------|----------
            0     |      260       |     0                      0    |       256      |    0.98    |    8
            1     |      184       |    2.0                     1    |       188      |    1.9     |    1
            2     |       94       |    3.0                     2    |        98      |    2.8     |    1
            3     |       44       |    4.9                     3    |        42      |    4.8     |    1
            4     |       21       |    8.5                     4    |        20      |    8.3     |    1
            5     |       10       |    13.8                    5    |        10      |    13.4    |    1
            6     |        5       |    19.0                    6    |         5      |    18.6    |    1
            7     |    RESERVED    |  RESERVED                  7    |     RESERVED   |  RESERVED  |    8

        The DLPF is configured by DLPF_CFG. The accelerometer and gyroscope are filtered according to the value of DLPF_CFG
     */

    MPU6050_Register.Reg_CONFIG.Sub_Reg_CONFIG.EXT_SYNC_SET = 0;
    MPU6050_Register.Reg_CONFIG.Sub_Reg_CONFIG.DLPF_CFG = 1;

    err += ioctl(WRITE, MPU6050_CONFIG, MPU6050_Register.Reg_CONFIG.all_bits);
    delay(300);
    
    Serial.print("Init Sample rate : ");
    /*
    Reg_SMPLRT_DIV : (8 bits)
        This register specifies the divider from the gyroscope output rate used to generate the Sample Rate

        Sample Rate (Hz) = Gyroscope Output Rate / (1 + SMPLRT_DIV)
        Gyroscope Output Rate -> DLPF_CFG
    */
   
    MPU6050_Register.Reg_SMPLRT_DIV.Sub_Reg_SMPLRT_DIV.SMPLRT_DIV = 0;
    
    err += ioctl(WRITE, MPU6050_SMPLRT_DIV, MPU6050_Register.Reg_SMPLRT_DIV.all_bits);
    delay(300);
    
    Serial.print("Gyro Setting : ");
    /*
    GYRO_CONFIG :

         Hex Rg | Dec Reg |  Bit 7  |  Bit 6  |  Bit 5  |   Bit 4-3     | Bit 2-1-0 |
        --------|---------|---------|---------|---------|---------------|-----------|
           1B   |   27    |  XG_ST  |  YG_ST  |  ZG_ST  |  FS_SEL[1:0]  | RESERVED  |

         GFS_SEL |  Gyro Full scale range
        ---------|------------------------
            0    |      ± 250 °/s
            1    |      ± 500 °/s
            2    |      ± 1000 °/s
            3    |      ± 2000 °/s

        XG_ST, YG_ST, ZG_ST Setting this bit causes the X, Y, Z axis accelerometer to perform self test.
        Self-test response = Sensor output with self-test enabled – Sensor output without self-test enabled
    */
   
    MPU6050_Register.Reg_GYRO_CONFIG.Sub_Reg_GYRO_CONFIG.FS_SEL = 1;
    MPU6050_Register.Reg_GYRO_CONFIG.Sub_Reg_GYRO_CONFIG.ZG_ST = 0;
    MPU6050_Register.Reg_GYRO_CONFIG.Sub_Reg_GYRO_CONFIG.ZG_ST = 0;
    MPU6050_Register.Reg_GYRO_CONFIG.Sub_Reg_GYRO_CONFIG.ZG_ST = 0;

    err += ioctl(WRITE, MPU6050_GYRO_CONFIG, MPU6050_Register.Reg_GYRO_CONFIG.all_bits);
    delay(300);
    
    Serial.print("Accelero Setting : ");
    /*
    ACCEL_CONFIG :

         Hex Rg | Dec Reg |  Bit 7  |  Bit 6  |  Bit 5  |   Bit 4-3     | Bit 2-1-0 |
        --------|---------|---------|---------|---------|---------------|-----------|
           1C   |   28    |  XA_ST  |  YA_ST  |  ZA_ST  |  AFS_SEL[1:0] | RESERVED  |


         AFS_SEL |  Accelero Full scale range
        ---------|----------------------------
            0    |           ± 2g
            1    |           ± 4g
            2    |           ± 8g
            3    |           ± 16g

        XA_ST, YA_ST, ZA_ST Setting this bit causes the X, Y, Z axis accelerometer to perform self test.
        Self-test response = Sensor output with self-test enabled – Sensor output without self-test enabled
    */
    MPU6050_Register.Reg_ACCEL_CONFIG.Sub_Reg_ACCEL_CONFIG.AFS_SEL = 1;
    MPU6050_Register.Reg_ACCEL_CONFIG.Sub_Reg_ACCEL_CONFIG.ZA_ST = 0;
    MPU6050_Register.Reg_ACCEL_CONFIG.Sub_Reg_ACCEL_CONFIG.YA_ST = 0;
    MPU6050_Register.Reg_ACCEL_CONFIG.Sub_Reg_ACCEL_CONFIG.XA_ST = 0;

    err += ioctl(WRITE, MPU6050_ACCEL_CONFIG, MPU6050_Register.Reg_ACCEL_CONFIG.all_bits);
    delay(300);
    
    Serial.print("Disable interrupt : ");
    /*
    INT_ENABLE :

         Hex Rg | Dec Reg |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |   Bit 3   |  Bit 2-1  |  Bit 0  |
        --------|---------|---------|---------|---------|---------|-----------|-----------|---------|
           38   |   56    |    -    |  MOT_EN |    -    |  FIFO_  |  I2C_MST_ |     -     |  DATA_  |
                |         |         |         |         | OFLOW_EN|   INT_EN  |           |  RDY_EN |

        MOT_EN : Enables Motion detection to generate an in
        FIFO_OFLOW_EN : Enables a FIFO buffer overflow to generate an interrupt
        I2C_MST_INT_EN : Enables any of the I2C Master interrupt sources to generate an interrupt.
        DATA_RDY_EN : Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed
     */
    MPU6050_Register.Reg_INT_ENABLE.Sub_Reg_INT_ENABLE.DATA_RDY_EN = 0;
    MPU6050_Register.Reg_INT_ENABLE.Sub_Reg_INT_ENABLE.I2C_MST_INT_EN = 0;
    MPU6050_Register.Reg_INT_ENABLE.Sub_Reg_INT_ENABLE.FIFO_OFLOW_EN = 0;
    MPU6050_Register.Reg_INT_ENABLE.Sub_Reg_INT_ENABLE.MOT_EN = 0;

    err += ioctl(WRITE, MPU6050_INT_ENABLE, MPU6050_Register.Reg_INT_ENABLE.all_bits);
    delay(300);
    
    Serial.print("Disable FIFO : ");
    /*
    FIFO_EN :
    This register determines which sensor measurements are loaded into the FIFO buffer.
     
         Hex Rg | Dec Reg |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
        --------|---------|---------|---------|---------|---------|---------|---------|---------|---------|
           23   |   35    |  TEMP_  |   XG_   |   YG_   |   ZG_   |  ACCEL_ |  SLV2_  |  SLV1_  |  SLV0_  |
                |         | FIFO_EN | FIFO_EN | FIFO_EN | FIFO_EN | FIFO_EN | FIFO_EN | FIFO_EN | FIFO_EN |

     */
    MPU6050_Register.Reg_FIFO_EN.Sub_Reg_FIFO_EN.SLV0_FIFO_EN = 0;
    MPU6050_Register.Reg_FIFO_EN.Sub_Reg_FIFO_EN.SLV1_FIFO_EN = 0;
    MPU6050_Register.Reg_FIFO_EN.Sub_Reg_FIFO_EN.SLV2_FIFO_EN = 0;
    MPU6050_Register.Reg_FIFO_EN.Sub_Reg_FIFO_EN.ACCEL_FIFO_EN = 0;
    MPU6050_Register.Reg_FIFO_EN.Sub_Reg_FIFO_EN.ZG_FIFO_EN = 0;
    MPU6050_Register.Reg_FIFO_EN.Sub_Reg_FIFO_EN.YG_FIFO_EN = 0;
    MPU6050_Register.Reg_FIFO_EN.Sub_Reg_FIFO_EN.XG_FIFO_EN = 0;

    err += ioctl(WRITE, MPU6050_FIFO_EN, MPU6050_Register.Reg_FIFO_EN.all_bits);
    delay(300);
    
    Serial.print("Disable I2C master modes : ");
    /*
    I2C_MST_CTRL :
    This register configures the auxiliary I2C bus for single-master or multi-master control
    Also enables the writing of Slave 3 data into the FIFO buffer.

         Hex Rg | Dec Reg |  Bit 7   |  Bit 6  |  Bit 5  |  Bit 4   |  Bit 3-2-1-0  |
        --------|---------|----------|---------|---------|----------|---------------|
           24   |   36    |   MULT_  |  WAIT_  |  SLV3_  | I2C_MST_ |  I2C_MST_CLK  |
                |         | MST_CTRL |  FOR_ES | FIFO_EN |   P_NSR  |               |

         I2C_MST_CLK |  I2C Master Clock Speed  |  8MHz Clock Divider  |
        -------------|--------------------------|----------------------|
              0      |           348 kHz        |           23         |
              1      |           333 kHz        |           24         |
              2      |           320 kHz        |           25         |
              3      |           308 kHz        |           26         |
              4      |           296 kHz        |           27         |
              5      |           286 kHz        |           28         |
              6      |           276 kHz        |           29         |
              7      |           267 kHz        |           30         |
              8      |           258 kHz        |           31         |
              9      |           500 kHz        |           16         |
              10     |           471 kHz        |           17         |
              11     |           444 kHz        |           18         |
              12     |           421 kHz        |           19         |
              13     |           400 kHz        |           20         |
              14     |           381 kHz        |           21         |
              15     |           364 kHz        |           22         |

        MUL_MST_EN : Enables multi-master capability
        WAIT_FOR_ES : When set to 1, delays the Data Ready interrupt until 
                        External Sensor data from the Slave devices have been loaded into the EXT_SENS_DATA registers
        SLV3_FIFO_EN : Enables EXT_SENS_DATA registers associated with Slave 3 to be written into the FIFO
        I2C_MST_P_NSR : Controls the I2C Master’s transition from one slave read to the next slave read

    */
    
    MPU6050_Register.Reg_I2C_MST_CTRL.Sub_Reg_I2C_MST_CTRL.I2C_MST_CLK = 0;
    MPU6050_Register.Reg_I2C_MST_CTRL.Sub_Reg_I2C_MST_CTRL.I2C_MST_P_NSR = 0;
    MPU6050_Register.Reg_I2C_MST_CTRL.Sub_Reg_I2C_MST_CTRL.SLV3_FIFO_EN = 0;
    MPU6050_Register.Reg_I2C_MST_CTRL.Sub_Reg_I2C_MST_CTRL.WAIT_FOR_ES = 0;
    MPU6050_Register.Reg_I2C_MST_CTRL.Sub_Reg_I2C_MST_CTRL.MUL_MST_EN = 0;

    err += ioctl(WRITE, MPU6050_I2C_MST_CTRL, MPU6050_Register.Reg_I2C_MST_CTRL.all_bits);
    delay(300);
    
    Serial.print("Disable FIFO & I2C master modes : ");
    /*
    USER_CTRL :
    This register allows the user to enable and disable the FIFO buffer, I2C Master Mode, and primary I2C interface.

         Hex Rg | Dec Reg |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1   |   Bit 0   |
        --------|---------|---------|---------|---------|---------|---------|---------|----------|-----------|
           6A   |   106   |    -    | FIFO_EN |   I2C_  |   I2C_  |    -    |  FIFO_  | I2C_MST_ | SIG_COND_ |
                |         |         |         |  MST_EN |  IF_DIS |         |   RESET |    RESET |   RESET   |

        FIFO_EN : Enables FIFO operations
        I2C_MST_EN : Enables I2C Master Mode.
        I2C_IF_DIS : Always write 0 to I2C_IF_DIS.
        FIFO_RESET : This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0 (auto clear)
        I2C_MST_RESET : Resets the I2C Master when set to 1 while I2C_MST_EN equals 0 (auto clear)
        SIG_COND_RESET : When set to 1, this bit resets the signal paths for all sensors (auto clear)
     */
    
    MPU6050_Register.Reg_USER_CTRL.Sub_Reg_USER_CTRL.FIFO_EN = 0;
    MPU6050_Register.Reg_USER_CTRL.Sub_Reg_USER_CTRL.I2C_MST_EN = 0;
    MPU6050_Register.Reg_USER_CTRL.Sub_Reg_USER_CTRL.I2C_IF_DIS = 0;
    MPU6050_Register.Reg_USER_CTRL.Sub_Reg_USER_CTRL.FIFO_RESET = 0;
    MPU6050_Register.Reg_USER_CTRL.Sub_Reg_USER_CTRL.I2C_MST_RESET = 0;
    MPU6050_Register.Reg_USER_CTRL.Sub_Reg_USER_CTRL.SIG_COND_RESET = 0;

    err += ioctl(WRITE, MPU6050_USER_CTRL, MPU6050_Register.Reg_USER_CTRL.all_bits);
    delay(300);
    
    Serial.print("Reset Gyro & Accelero signals: ");
    /*
    SIGNAL_PATH_RESET : 
    This register is used to reset the analog and digital signal paths of the gyroscope, accelerometer, and temperature sensors
         Hex Rg | Dec Reg |  Bit 7-6-5-4-3  |  Bit 2  |  Bit 1   |   Bit 0   |
        --------|---------|-----------------|---------|----------|-----------|
           68   |   104   |        -        |  GYRO_  |  ACCEL_  |   TEMP_   |
                |         |                 |   RESET |    RESET |    RESET  |

        This register does not clear the sensor registers

     */
    MPU6050_Register.Reg_SIGNAL_PATH_RESET.Sub_Reg_SIGNAL_PATH_RESET.TEMP_RESET = 1;
    MPU6050_Register.Reg_SIGNAL_PATH_RESET.Sub_Reg_SIGNAL_PATH_RESET.ACCEL_RESET = 1;
    MPU6050_Register.Reg_SIGNAL_PATH_RESET.Sub_Reg_SIGNAL_PATH_RESET.GYRO_RESET = 1;

    err += ioctl(WRITE, MPU6050_SIGNAL_PATH_RESET, MPU6050_Register.Reg_SIGNAL_PATH_RESET.all_bits);
    delay(300);

    if (err != 0)
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
    uint8_t raw_data[6];
    int8_t err;

    err = ioctl(READ, MPU6050_ACCEL_XOUT_H, 6, raw_data);
    if (err != 0)
    {
        Serial.println("Data read error");
        return;
    }

    AxH = raw_data[0];
    AxL = raw_data[1];
    AyH = raw_data[2];
    AyL = raw_data[3];
    AzH = raw_data[4];
    AzL = raw_data[5];
    
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
    uint8_t raw_data[6];
    int8_t err;

    err = ioctl(READ, MPU6050_GYRO_XOUT_H, 6, raw_data);
    if (err != 0)
    {
        Serial.println("Data read error");
        return;
    }

    GxH = raw_data[0];
    GxL = raw_data[1];
    GyH = raw_data[2];
    GyL = raw_data[3];
    GzH = raw_data[4];
    GzL = raw_data[5];

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
    uint8_t raw_data[1];
    int8_t err;

    err = ioctl(READ, MPU6050_GYRO_CONFIG, 1, raw_data);
    if (err != 0)
    {
        Serial.println("Data read error");
        return;
    }

    gyro_scale = *raw_data;
    gyro_scale = (gyro_scale & mask_fs_sel) >> 3;

    if (gyro_scale == 0) gyro_sensitivity = 131.0000;
    if (gyro_scale == 1) gyro_sensitivity = 65.5000;
    if (gyro_scale == 2) gyro_sensitivity = 32.7500;
    if (gyro_scale == 3) gyro_sensitivity = 16.3750;

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
    uint8_t raw_data[1];
    int8_t err;

    err = ioctl(READ, MPU6050_ACCEL_CONFIG, 1, raw_data);
    if (err != 0)
    {
        Serial.println("Data read error");
        return;
    }

    accelero_scale = *raw_data;
    accelero_scale = (accelero_scale & mask_fs_sel) >> 3;

    if (accelero_scale == 0) accelero_sensitivity = 16384.0000;
    if (accelero_scale == 1) accelero_sensitivity = 8192.0000;
    if (accelero_scale == 2) accelero_sensitivity = 4096.0000;
    if (accelero_scale == 3) accelero_sensitivity = 2048.0000;
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
    // To be converted into radiants since value in °/s
    Gxyz[0] = (Gxyz_raw[0] / gyro_sensitivity) * DEG_TO_RAD;
    Gxyz[1] = (Gxyz_raw[1] / gyro_sensitivity) * DEG_TO_RAD;
    Gxyz[2] = (Gxyz_raw[2] / gyro_sensitivity) * DEG_TO_RAD;
}

void MPU6050::updateAxyzGxyz(void)
{
    getAxyz_raw();
    getGxyz_raw();

    getAxyz();
    getGxyz();

}

MPU6050 AcceleroGyro;



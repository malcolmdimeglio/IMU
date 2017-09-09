//
//  MPU6050.h
//  
//
//  Created by Malcolm DI MEGLIO on 06/29/2015.
//
//

#ifndef ____MPU6050__
#define ____MPU6050__

#define ADRESS_MPU6050 0x68  // 0x68 with pin AD0=Low, 0x69 with pin AD0=High



// REGISTER ADRESSES
#define MPU6050_XG_OFFS_TC           0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_YG_OFFS_TC           0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_ZG_OFFS_TC           0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_X_FINE_GAIN          0x03 //[7:0] X_FINE_GAIN
#define MPU6050_Y_FINE_GAIN          0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_Z_FINE_GAIN          0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_XA_OFFS_H            0x06 //[15:0] XA_OFFS
#define MPU6050_XA_OFFS_L_TC         0x07
#define MPU6050_YA_OFFS_H            0x08 //[15:0] YA_OFFS
#define MPU6050_YA_OFFS_L_TC         0x09
#define MPU6050_SELF_TEST_A          0x10
#define MPU6050_ZA_OFFS_H            0x0A //[15:0] ZA_OFFS
#define MPU6050_ZA_OFFS_L_TC         0x0B
#define MPU6050_SELF_TEST_X          0X0D
#define MPU6050_SELF_TEST_Y          0X0E
#define MPU6050_SELF_TEST_Z          0X0F
#define MPU6050_XG_OFFS_USR_H        0x13 //[15:0] XG_OFFS_USR
#define MPU6050_XG_OFFS_USR_L        0x14
#define MPU6050_YG_OFFS_USR_H        0x15 //[15:0] YG_OFFS_USR
#define MPU6050_YG_OFFS_USR_L        0x16
#define MPU6050_ZG_OFFS_USR_H        0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_ZG_OFFS_USR_L        0x18
#define MPU6050_SMPLRT_DIV           0x19
#define MPU6050_CONFIG               0x1A
#define MPU6050_GYRO_CONFIG          0x1B
#define MPU6050_ACCEL_CONFIG         0x1C
#define MPU6050_FF_THR               0x1D
#define MPU6050_FF_DUR               0x1E
#define MPU6050_MOT_THR              0x1F
#define MPU6050_MOT_DUR              0x20
#define MPU6050_ZRMOT_THR            0x21
#define MPU6050_ZRMOT_DUR            0x22
#define MPU6050_FIFO_EN              0x23
#define MPU6050_I2C_MST_CTRL         0x24
#define MPU6050_I2C_SLV0_ADDR        0x25
#define MPU6050_I2C_SLV0_REG         0x26
#define MPU6050_I2C_SLV0_CTRL        0x27
#define MPU6050_I2C_SLV1_ADDR        0x28
#define MPU6050_I2C_SLV1_REG         0x29
#define MPU6050_I2C_SLV1_CTRL        0x2A
#define MPU6050_I2C_SLV2_ADDR        0x2B
#define MPU6050_I2C_SLV2_REG         0x2C
#define MPU6050_I2C_SLV2_CTRL        0x2D
#define MPU6050_I2C_SLV3_ADDR        0x2E
#define MPU6050_I2C_SLV3_REG         0x2F
#define MPU6050_I2C_SLV3_CTRL        0x30
#define MPU6050_I2C_SLV4_ADDR        0x31
#define MPU6050_I2C_SLV4_REG         0x32
#define MPU6050_I2C_SLV4_DO          0x33
#define MPU6050_I2C_SLV4_CTRL        0x34
#define MPU6050_I2C_SLV4_DI          0x35 //Read Only
#define MPU6050_I2C_MST_STATUS       0x36 //Read Only
#define MPU6050_INT_PIN_CFG          0x37
#define MPU6050_INT_ENABLE           0x38
#define MPU6050_DMP_INT_STATUS       0x39 //Read Only
#define MPU6050_INT_STATUS           0x3A //Read Only
#define MPU6050_ACCEL_XOUT_H         0x3B //Read Only
#define MPU6050_ACCEL_XOUT_L         0x3C //Read Only
#define MPU6050_ACCEL_YOUT_H         0x3D //Read Only
#define MPU6050_ACCEL_YOUT_L         0x3E //Read Only
#define MPU6050_ACCEL_ZOUT_H         0x3F //Read Only
#define MPU6050_ACCEL_ZOUT_L         0x40 //Read Only
#define MPU6050_TEMP_OUT_H           0x41 //Read Only
#define MPU6050_TEMP_OUT_L           0x42 //Read Only
#define MPU6050_GYRO_XOUT_H          0x43 //Read Only
#define MPU6050_GYRO_XOUT_L          0x44 //Read Only
#define MPU6050_GYRO_YOUT_H          0x45 //Read Only
#define MPU6050_GYRO_YOUT_L          0x46 //Read Only
#define MPU6050_GYRO_ZOUT_H          0x47 //Read Only
#define MPU6050_GYRO_ZOUT_L          0x48 //Read Only
#define MPU6050_EXT_SENS_DATA_00     0x49 //Read Only
#define MPU6050_EXT_SENS_DATA_01     0x4A //Read Only
#define MPU6050_EXT_SENS_DATA_02     0x4B //Read Only
#define MPU6050_EXT_SENS_DATA_03     0x4C //Read Only
#define MPU6050_EXT_SENS_DATA_04     0x4D //Read Only
#define MPU6050_EXT_SENS_DATA_05     0x4E //Read Only
#define MPU6050_EXT_SENS_DATA_06     0x4F //Read Only
#define MPU6050_EXT_SENS_DATA_07     0x50 //Read Only
#define MPU6050_EXT_SENS_DATA_08     0x51 //Read Only
#define MPU6050_EXT_SENS_DATA_09     0x52 //Read Only
#define MPU6050_EXT_SENS_DATA_10     0x53 //Read Only
#define MPU6050_EXT_SENS_DATA_11     0x54 //Read Only
#define MPU6050_EXT_SENS_DATA_12     0x55 //Read Only
#define MPU6050_EXT_SENS_DATA_13     0x56 //Read Only
#define MPU6050_EXT_SENS_DATA_14     0x57 //Read Only
#define MPU6050_EXT_SENS_DATA_15     0x58 //Read Only
#define MPU6050_EXT_SENS_DATA_16     0x59 //Read Only
#define MPU6050_EXT_SENS_DATA_17     0x5A //Read Only
#define MPU6050_EXT_SENS_DATA_18     0x5B //Read Only
#define MPU6050_EXT_SENS_DATA_19     0x5C //Read Only
#define MPU6050_EXT_SENS_DATA_20     0x5D //Read Only
#define MPU6050_EXT_SENS_DATA_21     0x5E //Read Only
#define MPU6050_EXT_SENS_DATA_22     0x5F //Read Only
#define MPU6050_EXT_SENS_DATA_23     0x60 //Read Only
#define MPU6050_MOT_DETECT_STATUS    0x61 //Read Only
#define MPU6050_I2C_SLV0_DO          0x63
#define MPU6050_I2C_SLV1_DO          0x64
#define MPU6050_I2C_SLV2_DO          0x65
#define MPU6050_I2C_SLV3_DO          0x66
#define MPU6050_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_SIGNAL_PATH_RESET    0x68
#define MPU6050_MOT_DETECT_CTRL      0x69
#define MPU6050_USER_CTRL            0x6A
#define MPU6050_PWR_MGMT_1           0x6B
#define MPU6050_PWR_MGMT_2           0x6C
#define MPU6050_BANK_SEL             0x6D
#define MPU6050_MEM_START_ADDR       0x6E
#define MPU6050_MEM_R_W              0x6F
#define MPU6050_DMP_CFG_1            0x70
#define MPU6050_DMP_CFG_2            0x71
#define MPU6050_FIFO_COUNT_H         0x72
#define MPU6050_FIFO_COUNT_L         0x73
#define MPU6050_FIFO_R_W             0x74
#define MPU6050_WHO_AM_I             0x75 //Read Only


typedef union
{
    uint8_t all_bits;
    struct
    {
        uint8_t SMPLRT_DIV  :8;         // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) (8bits)
    }Sub_Reg_SMPLRT_DIV;

}MPU6050_reg_conf_SMPLRT_DIV;



// CONFIG : Configures the external Frame Synchronization (FSYNC) pin sampling and the Digital Low Pass Filter (DLPF)
typedef union
{
    uint8_t all_bits;
    struct
    {
        uint8_t DLPF_CFG     : 3;    // External Frame Synchronization (FSYNC) pin sampling
        uint8_t EXT_SYNC_SET : 3;    // Digital Low Pass Filter
        uint8_t RESERVED     : 2;
    }Sub_Reg_CONFIG; 

}MPU6050_reg_conf_CONFIG;

// GYRO_CONFIG : Trigger gyroscope self-test & configure the gyroscope full scale range
typedef union
{
    uint8_t all_bits;
    struct
    {
        uint8_t RESERVED    : 3;
        uint8_t FS_SEL      : 2;    // Selects the full scale range of gyroscope
        uint8_t ZG_ST       : 1;    // Self Test Z axis output
        uint8_t YG_ST       : 1;    // Self Test Y axis output
        uint8_t XG_ST       : 1;    // Self Test X axis output
    }Sub_Reg_GYRO_CONFIG; 

}MPU6050_reg_conf_GYRO_CONFIG;

// ACCEL_CONFIG : Trigger accelerometer self-test & configure the accelerometer full scale range
typedef union
{
    uint8_t all_bits;
    struct
    {
        uint8_t RESERVED    : 3;
        uint8_t AFS_SEL     : 2;    // Selects the full scale range of accelerometer
        uint8_t ZA_ST       : 1;    // Self Test Z axis output
        uint8_t YA_ST       : 1;    // Self Test Y axis output
        uint8_t XA_ST       : 1;    // Self Test X axis output
    }Sub_Reg_ACCEL_CONFIG; 

}MPU6050_reg_conf_ACCEL_CONFIG;

// FIFO_EN : Determines which sensor measurements are loaded into the FIFO buffer
typedef union
{
    uint8_t all_bits;
    struct
    {
        uint8_t SLV0_FIFO_EN    : 1;    // EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 0 will be written into the FIFO buffer
        uint8_t SLV1_FIFO_EN    : 1;    // EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 1 will be written into the FIFO buffer
        uint8_t SLV2_FIFO_EN    : 1;    // EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 2 will be written into the FIFO buffer
        uint8_t ACCEL_FIFO_EN   : 1;    // ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) will be written into the FIFO buffer
        uint8_t ZG_FIFO_EN      : 1;    // GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and 72) will be written into the FIFO buffer
        uint8_t YG_FIFO_EN      : 1;    // GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and 70) will be written into the FIFO buffer
        uint8_t XG_FIFO_EN      : 1;    // GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and 68) will be written into the FIFO buffer
        uint8_t TEMP_FIFO_EN    : 1;    // TEMP_OUT_H and TEMP_OUT_L (Registers 65 and 66) will be written into the FIFO buffer
    }Sub_Reg_FIFO_EN;

}MPU6050_reg_conf_FIFO_EN;

// MST_CTRL : Configures the auxiliary I2C bus for single-master or multi-master control
typedef union
{
    uint8_t all_bits;
    struct
    {
        uint8_t I2C_MST_CLK     : 4;    // Configures the I2C master clock speed divider
        uint8_t I2C_MST_P_NSR   : 1;    // Controls the I2C Master’s transition from one slave read to the next slave read
        uint8_t SLV3_FIFO_EN    : 1;    // Enables EXT_SENS_DATA registers associated with Slave 3 to be written into the FIFO
        uint8_t WAIT_FOR_ES     : 1;    // Delays the Data Ready interrupt until External Sensor data from the Slave devices have been loaded into the EXT_SENS_DATA
        uint8_t MUL_MST_EN      : 1;    // Enables multi-master capability
    }Sub_Reg_I2C_MST_CTRL;

}MPU6050_reg_conf_I2C_MST_CTRL;

// INT_ENABLE : Enables interrupt generation by interrupt sources (56)
typedef union
{
    uint8_t all_bits;
    struct
    {
        uint8_t DATA_RDY_EN     : 1;    // Enables the Data Ready interrupt
        uint8_t RESERVED_1      : 2;
        uint8_t I2C_MST_INT_EN  : 1;    // Enables any of the I2C Master interrupt sources to generate an interrupt
        uint8_t FIFO_OFLOW_EN   : 1;    // Enables a FIFO buffer overflow to generate an interrupt
        uint8_t RESERVED_2      : 1;
        uint8_t MOT_EN          : 1;    // Enables Motion detection to generate an interrupt

    }Sub_Reg_INT_ENABLE;
}MPU6050_reg_conf_INT_ENABLE;

// SIGNAL_PATH_RESET : Resets the analog and digital signal paths of the gyroscope, accelerometer, and temperature sensors
typedef union
{
    uint8_t all_bits;
    struct
    {
        uint8_t TEMP_RESET      : 1;    // Resets the temperature sensor analog and digital signal paths
        uint8_t ACCEL_RESET     : 1;    // Resets the accelerometer analog and digital signal paths
        uint8_t GYRO_RESET      : 1;    // Resets the gyroscope analog and digital signal paths
        uint8_t RESERVED        : 5;
    }Sub_Reg_SIGNAL_PATH_RESET;

}MPU6050_reg_conf_SIGNAL_PATH_RESET;

// USER_CTRL : Allows the user to enable and disable the FIFO buffer
typedef union
{
    uint8_t all_bits;
    struct
    {
        uint8_t SIG_COND_RESET  : 1;    // Resets the signal paths for all sensors
        uint8_t I2C_MST_RESET   : 1;    // Resets the I2C Master when set to 1 while I2C_MST_EN equals 0
        uint8_t FIFO_RESET      : 1;    // Resets the FIFO buffer when set to 1 while FIFO_EN equals 0
        uint8_t RESERVED_1      : 1;
        uint8_t I2C_IF_DIS      : 1;    // Always write this bit as zero
        uint8_t I2C_MST_EN      : 1;    // Enables I2C Master Mode
        uint8_t FIFO_EN         : 1;    // Enables FIFO operations
        uint8_t RESERVED_2      : 1;
    }Sub_Reg_USER_CTRL;

}MPU6050_reg_conf_USER_CTRL;

// PWR_MGMT_1 : configure the power mode and clock source. 
// It also provides a bit for resetting the entire device, and a bit for disabling the temperature sensor.
typedef union
{
    uint8_t all_bits;
    struct
    {
        uint8_t CLKSEL          : 3;        // Specifies the clock source of the device
        uint8_t TEMP_DIS        : 1;        // When set to 1, this bit disables the temperature sensor
        uint8_t RESERVED        : 1;
        uint8_t CYCLE           : 1;        // When this bit is set to 1 and SLEEP is disabled the MPU6050 will cycle between sleep mode and waking up
        uint8_t SLEEP           : 1;        // When set to 1, this bit puts the MPU6050 into sleep mode
        uint8_t DEVICE_RESET    : 1;        // When set to 1, this bit resets all internal registers to their default values
    }Sub_Reg_PWR_MGMT_1;

}MPU6050_reg_conf_PWR_MGMT_1;


typedef struct
{
    MPU6050_reg_conf_SMPLRT_DIV Reg_SMPLRT_DIV;
    MPU6050_reg_conf_CONFIG Reg_CONFIG;
    MPU6050_reg_conf_GYRO_CONFIG Reg_GYRO_CONFIG;
    MPU6050_reg_conf_ACCEL_CONFIG Reg_ACCEL_CONFIG;
    MPU6050_reg_conf_FIFO_EN Reg_FIFO_EN;
    MPU6050_reg_conf_I2C_MST_CTRL Reg_I2C_MST_CTRL;
    MPU6050_reg_conf_INT_ENABLE Reg_INT_ENABLE;
    MPU6050_reg_conf_SIGNAL_PATH_RESET Reg_SIGNAL_PATH_RESET;
    MPU6050_reg_conf_USER_CTRL Reg_USER_CTRL;
    MPU6050_reg_conf_PWR_MGMT_1 Reg_PWR_MGMT_1;
}MPU6050_reg_conf;

class MPU6050
{
private:
    float Axyz_raw[3], Gxyz_raw[3];
    float accelero_sensitivity, gyro_sensitivity;

    MPU6050_reg_conf MPU6050_Register;

    int8_t ioctl(uint8_t, uint8_t, uint8_t);
    int8_t ioctl(uint8_t, uint8_t, uint8_t, uint8_t);
    int8_t com_status(uint8_t);
    void getAxyz_raw(void);
    void getGxyz_raw(void);
    void getAcceleroScale(void);
    void getGyroScale(void);
    void getAxyz(void);
    void getGxyz(void);
    
    
public:
    float Axyz[3], Gxyz[3];

    bool init(void);
    void updateAxyzGxyz(void);

    
    
};
extern MPU6050 AcceleroGyro;



#endif /* defined(____MPU6050__) */




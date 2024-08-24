/**
 * @file    IMU.h
 * @brief   IMU驱动
 *
 * @author  Tang-yucheng
 * @date    2023-12-14 （创建）
 *
 */

#ifndef __HDL_IMU_H
#define __HDL_IMU_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "User_Usart.h"
#include "usbd_cdc_if.h"
#include "User_Math.h"

#include "cstring"

/* 宏定义 -------------------------------------------------------------------------------------------------------------*/
#define FDI_PACKET_CMD_OFFSET               1
#define FDI_PACKET_DATA_OFFSET              7

#define FDILINK_IMUDATA_PACKET_ID           0x40
#define FDILINK_AHRSDATA_PACKET_ID          0x41

/* 结构体定义 ----------------------------------------------------------------------------------------------------------*/
/**
 * @brief FDI-IMU数据结构体
 */
struct Struct_FDI_Data_IMU
{
    float Gyroscope_X;
    float Gyroscope_Y;
    float Gyroscope_Z;
    float Accelerometer_X;
    float Accelerometer_Y;
    float Accelerometer_Z;
    float Magnetometer_X;
    float Magnetometer_Y;
    float Magnetometer_Z;
    float IMU_Temperature;
    float Pressure;
    float Pressure_Temperature;
    uint64_t Timestamp;
};

/**
 * @brief FDI-AHRS数据结构体
 */
struct Struct_FDI_Data_AHRS
{
    float RollSpeed;
    float PitchSpeed;
    float HeadingSpeed;
    float Roll;
    float Pitch;
    float Heading;
    float Q1;
    float Q2;
    float Q3;
    float Q4;
    uint64_t Timestamp;
};

/**
 * @brief FDI数据结构体
 */
struct Struct_FDI_Data
{
    float Pitch;
    float Heading;
    float Roll;
    float PitchSpeed;
    float HeadingSpeed;
    float RollSpeed;
    float Gyroscope_Y;
    float Gyroscope_Z;
    float Gyroscope_X;
};

/* 类定义 -------------------------------------------------------------------------------------------------------------*/
/**
 * @brief FDI惯导模块类
 */
class Class_IMU_FDI
{
public:
    Struct_FDI_Data Data_Stand;                                       /*!< IMU变换后数据 */

    /* 函数 */
    void Init(Struct_UART_Manage_Object * UART_Manage_Obj);
    void DataGet(uint16_t Data_Size);
    void FreqCount(float _time);

    inline float GetFreq();
private:
    /* 常量 */
    Struct_UART_Manage_Object * UART_Manage_Object;                   /*!< 绑定的UART */
    float Rotation_Matrix[3][3] = {{-1.0f, 0, 0},     /*!< 陀螺仪坐标系变换旋转矩阵 */
                                   {0, -1.0f, 0},
                                   {0, 0, 1.0f}};

    /* 读写变量 */
    Struct_FDI_Data_IMU Data_IMU;                                     /*!< IMU原始数据 */
    Struct_FDI_Data_AHRS Data_AHRS;

    uint32_t Freq_CNT = 0;
    float Freq = 0;
};

/* 变量声明 ------------------------------------------------------------------------------------------------------------*/
extern Class_IMU_FDI IMU_Gimbal;

/* 接口函数定义 ---------------------------------------------------------------------------------------------------------*/
float Class_IMU_FDI::GetFreq()
{
    return this->Freq;
}

#endif  /* HDL_IMU.h */

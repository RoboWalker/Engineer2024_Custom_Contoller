/**
 * @file    IMU.cpp
 * @brief   IMU驱动
 *
 * @author  Tang-yucheng
 * @date    2023-12-14 （创建）
 *
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "IMU.h"

/* 全局变量创建 --------------------------------------------------------------------------------------------------------*/
Class_IMU_FDI IMU_Gimbal;

/* 函数定义 -----------------------------------------------------------------------------------------------------------*/
void Class_IMU_FDI::Init(Struct_UART_Manage_Object * UART_Manage_Obj)
{
    this->UART_Manage_Object = UART_Manage_Obj;

    /* UART初始化，使能DMA-IDLE接收 */
    UART_Init(UART_Manage_Obj, 256);
}

void Class_IMU_FDI::DataGet(uint16_t Data_Size)
{
    uint8_t * Data_Buffer = this->UART_Manage_Object->Rx_Buffer;
    float Data[3];

    if (Data_Size != 64 && Data_Size != 56)
    {
        return;
    }

    switch(Data_Buffer[FDI_PACKET_CMD_OFFSET])
    {
        case FDILINK_IMUDATA_PACKET_ID:
            this->Freq_CNT++;
            memcpy(&(this->Data_IMU), &Data_Buffer[FDI_PACKET_DATA_OFFSET], sizeof(Struct_FDI_Data_IMU));
            /* 坐标系变换 */
            Data[0] = this->Data_IMU.Gyroscope_Y;
            Data[1] = this->Data_IMU.Gyroscope_Z;
            Data[2] = this->Data_IMU.Gyroscope_X;
            Math_Matrix_Multiply_3_1(this->Rotation_Matrix, Data, &this->Data_Stand.Gyroscope_Y);
            break;
        case FDILINK_AHRSDATA_PACKET_ID:
            memcpy(&(this->Data_AHRS), &Data_Buffer[FDI_PACKET_DATA_OFFSET], sizeof(Struct_FDI_Data_AHRS));
            /* 坐标系变换 */
            Data[0] = this->Data_AHRS.Pitch;
            Data[1] = this->Data_AHRS.Heading;
            Data[2] = this->Data_AHRS.Roll;
            Math_Matrix_Multiply_3_1(this->Rotation_Matrix, Data, &this->Data_Stand.Pitch);
            Data[0] = this->Data_AHRS.PitchSpeed;
            Data[1] = this->Data_AHRS.HeadingSpeed;
            Data[2] = this->Data_AHRS.RollSpeed;
            Math_Matrix_Multiply_3_1(this->Rotation_Matrix, Data, &this->Data_Stand.PitchSpeed);
            break;
        default:
            break;
    }
}

void Class_IMU_FDI::FreqCount(float _time)
{
    this->Freq = (float)this->Freq_CNT / _time * 1.0f;
    this->Freq_CNT = 0;
}
/**
 * @file    Motor_DJI.cpp
 * @brief   大疆电机驱动
 *
 * @author  Tang-yucheng
 * @date    2023-11-17 （创建）
 *
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Motor_DJI.h"

/* 全局变量创建 --------------------------------------------------------------------------------------------------------*/

/* 发送缓冲区，便于电机数据打包发送 */
uint8_t DJI_C620_L_CAN1_Tx_Data[8];
uint8_t DJI_C620_L_CAN2_Tx_Data[8];
uint8_t DJI_GM6020_H_CAN1_Tx_Data[8];
uint8_t DJI_GM6020_H_CAN2_Tx_Data[8];
uint8_t DJI_GM6020_L_C620_H_CAN1_Tx_Data[8];
uint8_t DJI_GM6020_L_C620_H_CAN2_Tx_Data[8];

/* 函数定义 -----------------------------------------------------------------------------------------------------------*/
/***********************************************************************************************************************
 * @brief 大疆CAN发送函数（将缓冲区数据统一发送）（需定时发送）
 **********************************************************************************************************************/
 void DJI_CAN_SendData()
{
    /* DJI-CAN1电机 */
    CAN_Send_Data(&CAN1_Manage_Object, DJI_C620_L_ID_TX, DJI_C620_L_CAN1_Tx_Data, 8);
    CAN_Send_Data(&CAN1_Manage_Object, DJI_GM6020_H_ID_TX, DJI_GM6020_H_CAN1_Tx_Data, 8);
    CAN_Send_Data(&CAN1_Manage_Object, DJI_GM6020_L_C620_H_ID_TX, DJI_GM6020_L_C620_H_CAN1_Tx_Data, 8);

    /* DJI-CAN2电机 */
    CAN_Send_Data(&CAN2_Manage_Object, DJI_C620_L_ID_TX, DJI_C620_L_CAN2_Tx_Data, 8);
    CAN_Send_Data(&CAN2_Manage_Object, DJI_GM6020_H_ID_TX, DJI_GM6020_H_CAN2_Tx_Data, 8);
    CAN_Send_Data(&CAN2_Manage_Object, DJI_GM6020_L_C620_H_ID_TX, DJI_GM6020_L_C620_H_CAN2_Tx_Data, 8);
}

/***********************************************************************************************************************
 * @brief 大疆分配CAN发送缓冲区
 *
 * @param CAN_Manage_Obj        CAN处理结构体指针
 * @param __CAN_ID              CAN-ID
 * @return uint8_t*             缓冲区指针
 **********************************************************************************************************************/
uint8_t *allocate_tx_data(Struct_CAN_Manage_Object * CAN_Manage_Obj, Enum_DJI_Motor_ID __CAN_ID)
{
    CAN_HandleTypeDef * hcan = CAN_Manage_Obj->hcan;
    uint8_t * tmp_tx_data_ptr;

    /* 判断CAN外设 */
    if(hcan->Instance == hcan1.Instance)
    {
        switch(__CAN_ID)
        {
            case (DJI_Motor_ID_0x201):
                tmp_tx_data_ptr = &(DJI_C620_L_CAN1_Tx_Data[0]);
                break;
            case (DJI_Motor_ID_0x202):
                tmp_tx_data_ptr = &(DJI_C620_L_CAN1_Tx_Data[2]);
                break;
            case (DJI_Motor_ID_0x203):
                tmp_tx_data_ptr = &(DJI_C620_L_CAN1_Tx_Data[4]);
                break;
            case (DJI_Motor_ID_0x204):
                tmp_tx_data_ptr = &(DJI_C620_L_CAN1_Tx_Data[6]);
                break;
            case (DJI_Motor_ID_0x205):
                tmp_tx_data_ptr = &(DJI_GM6020_L_C620_H_CAN1_Tx_Data[0]);
                break;
            case (DJI_Motor_ID_0x206):
                tmp_tx_data_ptr = &(DJI_GM6020_L_C620_H_CAN1_Tx_Data[2]);
                break;
            case (DJI_Motor_ID_0x207):
                tmp_tx_data_ptr = &(DJI_GM6020_L_C620_H_CAN1_Tx_Data[4]);
                break;
            case (DJI_Motor_ID_0x208):
                tmp_tx_data_ptr = &(DJI_GM6020_L_C620_H_CAN1_Tx_Data[6]);
                break;
            case (DJI_Motor_ID_0x209):
                tmp_tx_data_ptr = &(DJI_GM6020_H_CAN1_Tx_Data[0]);
                break;
            case (DJI_Motor_ID_0x20A):
                tmp_tx_data_ptr = &(DJI_GM6020_H_CAN1_Tx_Data[2]);
                break;
            case (DJI_Motor_ID_0x20B):
                tmp_tx_data_ptr = &(DJI_GM6020_H_CAN1_Tx_Data[4]);
                break;
            case (DJI_Motor_ID_UNDEFINED):
                tmp_tx_data_ptr = NULL;
                break;
        }
    }
    else
    {
        switch(__CAN_ID)
        {
            case (DJI_Motor_ID_0x201):
                tmp_tx_data_ptr = &(DJI_C620_L_CAN2_Tx_Data[0]);
                break;
            case (DJI_Motor_ID_0x202):
                tmp_tx_data_ptr = &(DJI_C620_L_CAN2_Tx_Data[2]);
                break;
            case (DJI_Motor_ID_0x203):
                tmp_tx_data_ptr = &(DJI_C620_L_CAN2_Tx_Data[4]);
                break;
            case (DJI_Motor_ID_0x204):
                tmp_tx_data_ptr = &(DJI_C620_L_CAN2_Tx_Data[6]);
                break;
            case (DJI_Motor_ID_0x205):
                tmp_tx_data_ptr = &(DJI_GM6020_L_C620_H_CAN2_Tx_Data[0]);
                break;
            case (DJI_Motor_ID_0x206):
                tmp_tx_data_ptr = &(DJI_GM6020_L_C620_H_CAN2_Tx_Data[2]);
                break;
            case (DJI_Motor_ID_0x207):
                tmp_tx_data_ptr = &(DJI_GM6020_L_C620_H_CAN2_Tx_Data[4]);
                break;
            case (DJI_Motor_ID_0x208):
                tmp_tx_data_ptr = &(DJI_GM6020_L_C620_H_CAN2_Tx_Data[6]);
                break;
            case (DJI_Motor_ID_0x209):
                tmp_tx_data_ptr = &(DJI_GM6020_H_CAN2_Tx_Data[0]);
                break;
            case (DJI_Motor_ID_0x20A):
                tmp_tx_data_ptr = &(DJI_GM6020_H_CAN2_Tx_Data[2]);
                break;
            case (DJI_Motor_ID_0x20B):
                tmp_tx_data_ptr = &(DJI_GM6020_H_CAN2_Tx_Data[4]);
                break;
            case (DJI_Motor_ID_UNDEFINED):
                tmp_tx_data_ptr = NULL;
                break;
        }
    }

    return (tmp_tx_data_ptr);
}

/***********************************************************************************************************************
 * @brief GM6020初始化函数
 *
 * @param CAN_Manage_Obj                CAN处理结构体指针
 * @param __CAN_ID                      绑定的CAN-ID
 * @param __DJI_Motor_Control_Method    电机控制方式, 默认角度
 * @param __Encoder_Offset              编码器偏移, 默认0
 * @param __Omega_Max                   最大速度, 需根据不同负载测量后赋值, 也就开环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
 **********************************************************************************************************************/
void Class_DJI_Motor_GM6020::Init(Struct_CAN_Manage_Object * CAN_Manage_Obj, Enum_DJI_Motor_ID __CAN_ID,
                                  Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method,
                                  int32_t __Encoder_Offset, float __Omega_Max)
{
    CAN_Manage_Object = CAN_Manage_Obj;
    CAN_ID = __CAN_ID;
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
    Encoder_Offset = __Encoder_Offset;
    Omega_Max = __Omega_Max;
    CAN_Tx_Data = allocate_tx_data(CAN_Manage_Obj, __CAN_ID);
}

/***********************************************************************************************************************
 * @brief GM6020实际数据接收函数（CAN接收回调函数中）
 **********************************************************************************************************************/
void Class_DJI_Motor_GM6020::DataGet()
{
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_torque;
    int8_t tmp_temperature;
    auto tmp_buffer = (Struct_DJI_Motor_CAN_Data *) CAN_Manage_Object->Rx_Buffer.Data;

    /* 滑动窗口, 判断电机是否在线 */
    Flag += 1;

    /* 大小端处理 */
    Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Torque_Reverse, (void *) &tmp_torque);
    tmp_temperature = tmp_buffer->Temperature;

    /* 计算圈数与总编码器值 */
    delta_encoder = tmp_encoder - Data.Pre_Encoder;
    if(delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        /* 正方向转过了一圈 */
        Data.Total_Round++;
    }
    else if(delta_encoder > Encoder_Num_Per_Round / 2)
    {
        /* 反方向转过了一圈 */
        Data.Total_Round--;
    }
    Data.Total_Encoder = Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder + Encoder_Offset;

    /* 标准化接收数据 */
    Data.Now_Angle = (float) Data.Total_Encoder / (float) Encoder_Num_Per_Round * 2.0f * PI;
    Data.Now_Omega = Filter_lambda * ((float) tmp_omega * RPM_TO_RADPS) + (1 - Filter_lambda) * Data.Pre_Omega;
    Data.Now_Torque = (float) tmp_torque * Current_Conversion;
    Data.Now_Temperature = (float)tmp_temperature + CELSIUS_TO_KELVIN;

    /* 更新数据 */
    Data.Pre_Omega = Data.Now_Omega;
    Data.Pre_Encoder = tmp_encoder;
}

/***********************************************************************************************************************
 * @brief GM6020存活检测函数（使用此函数才能使能设备）
 **********************************************************************************************************************/
void Class_DJI_Motor_GM6020::AliveCheck()
{
    //判断该时间段内是否接收过电机数据
    if(Flag == Pre_Flag)
    {
        //电机断开连接
        DJI_Motor_Status = DJI_Motor_Status_DISABLE;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        PID_Torque.Set_Integral_Error(0.0f);
    }
    else
    {
        //电机保持连接
        DJI_Motor_Status = DJI_Motor_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

/***********************************************************************************************************************
 * @brief GM6020闭环控制函数（需定时控制，置于定时器中断回调函数中）
 **********************************************************************************************************************/
void Class_DJI_Motor_GM6020::Control()
{
    switch(DJI_Motor_Control_Method)
    {
        case (DJI_Motor_Control_Method_OPENLOOP):
        {
            //默认开环速度控制
            U_Out = Target_Omega / Omega_Max * (float)Output_Max;
            break;
        }
        case (DJI_Motor_Control_Method_TORQUE):
        {
            PID_Torque.Set_Target(Target_Torque);
            PID_Torque.Set_Now(Data.Now_Torque);
            PID_Torque.Calculate();

            Iq_Set = PID_Torque.Get_Out();
            break;
        }
        case (DJI_Motor_Control_Method_OMEGA):
        {
            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Omega.Calculate();

            Target_Torque = PID_Omega.Get_Out();

            PID_Torque.Set_Target(Target_Torque);
            PID_Torque.Set_Now(Data.Now_Torque);
            PID_Torque.Calculate();

            Iq_Set = PID_Torque.Get_Out();
            break;
        }
        case (DJI_Motor_Control_Method_ANGLE):
        {
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.Calculate();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Omega.Calculate();

            Target_Torque = PID_Omega.Get_Out();

            PID_Torque.Set_Target(Target_Torque);
            PID_Torque.Set_Now(Data.Now_Torque);
            PID_Torque.Calculate();

            Iq_Set = PID_Torque.Get_Out();
            break;
        }
        default:
        {
            Iq_Set = 0.0f;
            break;
        }
    }

    /* 前馈，计算输出电压 */
    U_Out = 7.5f * Target_Torque + 8.67f * Lamda * (Target_Torque - Data.Now_Torque) + 1 / 12.17f * Data.Now_Omega * Lamda_Omega;
    U_Set = U_Out / U_In * 25000.0f * 0.866 + Iq_Set;
    /* 输出限幅 */
    if(U_Set > Output_Max)
    {
        U_Set = Output_Max;
    }
    else if(U_Set < -Output_Max)
    {
        U_Set = -Output_Max;
    }

    /* CAN-TX缓冲区填充 */
    CAN_Tx_Data[0] = (int16_t) U_Set >> 8;
    CAN_Tx_Data[1] = (int16_t) U_Set;
}

/***********************************************************************************************************************
 * @brief C610初始化
 *
 * @param CAN_Manage_Obj                CAN处理结构体指针
 * @param __CAN_ID                      CAN-ID
 * @param __DJI_Motor_Control_Method    电机控制方式, 默认角度
 * @param __Gearbox_Rate                减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __Torque_Max                  最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
 **********************************************************************************************************************/
void Class_DJI_Motor_C610::Init(Struct_CAN_Manage_Object * CAN_Manage_Obj, Enum_DJI_Motor_ID __CAN_ID,
                                Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method,
                                float __Gearbox_Rate, float __Torque_Max)
{
    CAN_Manage_Object = CAN_Manage_Obj;
    CAN_ID = __CAN_ID;
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Torque_Max = __Torque_Max;
    CAN_Tx_Data = allocate_tx_data(CAN_Manage_Obj, __CAN_ID);
}

/***********************************************************************************************************************
 * @brief C610实际数据接收函数（CAN接收回调函数中）
 **********************************************************************************************************************/
void Class_DJI_Motor_C610::DataGet()
{
    //滑动窗口, 判断电机是否在线
    Flag += 1;

    //数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_torque, tmp_temperature;
    auto tmp_buffer = (Struct_DJI_Motor_CAN_Data *) CAN_Manage_Object->Rx_Buffer.Data;

    //处理大小端
    Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Torque_Reverse, (void *) &tmp_torque);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Temperature, (void *) &tmp_temperature);

    //计算圈数与总编码器值
    delta_encoder = tmp_encoder - Data.Pre_Encoder;
    if(delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        //正方向转过了一圈
        Data.Total_Round++;
    }
    else if(delta_encoder > Encoder_Num_Per_Round / 2)
    {
        //反方向转过了一圈
        Data.Total_Round--;
    }
    Data.Total_Encoder = Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder;

    //计算电机本身信息
    Data.Now_Angle = (float) Data.Total_Encoder / (float) Encoder_Num_Per_Round * 2.0f * PI / Gearbox_Rate;
    Data.Now_Omega = (float) tmp_omega * RPM_TO_RADPS / Gearbox_Rate;
    Data.Now_Torque = tmp_torque;
    Data.Now_Temperature = (float)tmp_temperature + CELSIUS_TO_KELVIN;

    //存储预备信息
    Data.Pre_Encoder = tmp_encoder;
}

/***********************************************************************************************************************
 * @brief C610存活检测函数
 **********************************************************************************************************************/
void Class_DJI_Motor_C610::AliveCheck()
{
    //判断该时间段内是否接收过电机数据
    if(Flag == Pre_Flag)
    {
        //电机断开连接
        DJI_Motor_Status = DJI_Motor_Status_DISABLE;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
    }
    else
    {
        //电机保持连接
        DJI_Motor_Status = DJI_Motor_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/***********************************************************************************************************************
 * @brief C610闭环控制函数（需定时控制）
 **********************************************************************************************************************/
void Class_DJI_Motor_C610::Control()
{
    switch(DJI_Motor_Control_Method)
    {
        case (DJI_Motor_Control_Method_OPENLOOP):
        {
            //默认开环扭矩控制
            Out = Target_Torque / Torque_Max * (float)Output_Max;

            break;
        }
        case (DJI_Motor_Control_Method_TORQUE):
        {
            //默认闭环扭矩控制
            Out = Target_Torque / Torque_Max * (float)Output_Max;
        }
            break;
        case (DJI_Motor_Control_Method_OMEGA):
        {
            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Omega.Calculate();

            Out = PID_Omega.Get_Out();

            break;
        }
        case (DJI_Motor_Control_Method_ANGLE):
        {
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.Calculate();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Omega.Calculate();

            Out = PID_Omega.Get_Out();

            break;
        }
        default:
        {
            Out = 0.0f;

            break;
        }
    }

    CAN_Tx_Data[0] = (int16_t) Out >> 8;
    CAN_Tx_Data[1] = (int16_t) Out;
}

/***********************************************************************************************************************
 * @brief C620初始化
 *
 * @param CAN_Manage_Obj                CAN处理结构体指针
 * @param __CAN_ID                      CAN-ID
 * @param __DJI_Motor_Control_Method    电机控制方式, 默认速度
 * @param __Gearbox_Rate                减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __Torque_Max                  最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
 **********************************************************************************************************************/
void Class_DJI_Motor_C620::Init(Struct_CAN_Manage_Object * CAN_Manage_Obj, Enum_DJI_Motor_ID __CAN_ID,
                                Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method,
                                float __Gearbox_Rate, float __Torque_Max)
{
    CAN_Manage_Object = CAN_Manage_Obj;
    CAN_ID = __CAN_ID;
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Torque_Max = __Torque_Max;
    CAN_Tx_Data = allocate_tx_data(CAN_Manage_Obj, __CAN_ID);
}

/***********************************************************************************************************************
 * @brief C620实际数据接收函数（CAN接收回调函数中）
 **********************************************************************************************************************/
void Class_DJI_Motor_C620::DataGet()
{
    //滑动窗口, 判断电机是否在线
    Flag += 1;

    //数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_torque, tmp_temperature;
    auto tmp_buffer = (Struct_DJI_Motor_CAN_Data *) CAN_Manage_Object->Rx_Buffer.Data;

    //处理大小端
    Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Torque_Reverse, (void *) &tmp_torque);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Temperature, (void *) &tmp_temperature);

    //计算圈数与总编码器值
    delta_encoder = tmp_encoder - Data.Pre_Encoder;
    if(delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        //正方向转过了一圈
        Data.Total_Round++;
    }
    else if(delta_encoder > Encoder_Num_Per_Round / 2)
    {
        //反方向转过了一圈
        Data.Total_Round--;
    }
    Data.Total_Encoder = Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder;

    //计算电机本身信息
    Data.Now_Angle = (float) Data.Total_Encoder / (float) Encoder_Num_Per_Round * 2.0f * PI / Gearbox_Rate;
    Data.Now_Omega = (float) tmp_omega * RPM_TO_RADPS / Gearbox_Rate;
    Data.Now_Torque = tmp_torque;
    Data.Now_Temperature = (float)tmp_temperature + CELSIUS_TO_KELVIN;

    //存储预备信息
    Data.Pre_Encoder = tmp_encoder;
}

/***********************************************************************************************************************
 * @brief C620存活检测函数
 **********************************************************************************************************************/
void Class_DJI_Motor_C620::AliveCheck()
{
    //判断该时间段内是否接收过电机数据
    if(Flag == Pre_Flag)
    {
        //电机断开连接
        DJI_Motor_Status = DJI_Motor_Status_DISABLE;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
    }
    else
    {
        //电机保持连接
        DJI_Motor_Status = DJI_Motor_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/***********************************************************************************************************************
 * @brief C620闭环控制函数（需定时控制）
 **********************************************************************************************************************/
void Class_DJI_Motor_C620::Control()
{
    switch(DJI_Motor_Control_Method)
    {
        case (DJI_Motor_Control_Method_OPENLOOP):
        {
            //默认开环扭矩控制
            Out = Target_Torque / Torque_Max * (float)Output_Max;
            break;
        }
        case (DJI_Motor_Control_Method_TORQUE):
        {
            //默认闭环扭矩控制
            Out = Target_Torque / Torque_Max * (float)Output_Max;
            break;
        }
        case (DJI_Motor_Control_Method_OMEGA):
        {
            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Omega.Calculate();

            Out = PID_Omega.Get_Out();
            break;
        }
        case (DJI_Motor_Control_Method_ANGLE):
        {
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.Calculate();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Omega.Calculate();

            Out = PID_Omega.Get_Out();
            break;
        }
        default:
        {
            Out = 0.0f;
            break;
        }
    }

    CAN_Tx_Data[0] = (int16_t) Out >> 8;
    CAN_Tx_Data[1] = (int16_t) Out;
}
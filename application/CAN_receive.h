/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H

#include "main.h"
#include "object_dictionary.h"

#define CHASSIS_CAN hcan1

// CAN发送相关定义
#define CAN_SEND_1_BYTE    0x2F
#define CAN_SEND_2_BYTE    0x2B
#define CAN_SEND_4_BYTE    0x23

// 控制字定义
#define CONTROL_WORD_ENABLE    0x0006
#define CONTROL_WORD_DISABLE   0x0000
#define CONTROL_WORD_ABSOLUTE_POSITION_ENABLE 0x001F

// 运行模式定义
typedef enum
{
    POSITION_CONTROL = 0x01,
    SPEED_CONTROL = 0x03,
    TORQUE_CONTROL = 0x04,
    HOMING_MODE = 0x06,
    INTERPOLATION_MODE = 0x07,
    CSP = 0x08,
    CSV = 0x09,
    CST = 0x0A,
    PULSE_TRAIN_CONTROL = -4
}MODE_ENUM;

void CAN_motor_enable(uint8_t motor_id);
void CAN_motor_mode(int8_t mode, uint8_t motor_id);
void CAN_motor_setSpeed(int32_t speed_ref, uint8_t motor_id);
void CAN_motor_setProfileSpeed(uint32_t profile_speed, uint8_t motor_id);
void CAN_motor_setPos(int32_t position_ref, uint8_t motor_id);

// 新增的统一CAN打包和发送接口
bool CAN_SDO_Send(uint8_t motor_id, uint16_t index, uint8_t subindex, void* value, OD_DataType dataType);

#endif

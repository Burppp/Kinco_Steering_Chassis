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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1

void CAN_motor_enable(uint8_t motor_id);
void CAN_motor_mode(int8_t mode, uint8_t motor_id);
void CAN_motor_setSpeed(int32_t speed_ref, uint8_t motor_id);
void CAN_motor_setProfileSpeed(uint32_t profile_speed, uint8_t motor_id);
void CAN_motor_setPos(int32_t position_ref, uint8_t motor_id);

#endif

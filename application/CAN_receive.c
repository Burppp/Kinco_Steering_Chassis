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

#include "CAN_receive.h"
#include "main.h"
#include "string.h"

#include "packet_param.h"
#include "object_dictionary.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern ObjectDictionary* od;

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        // case CAN_3508_M1_ID:
        // case CAN_3508_M2_ID:
        // case CAN_3508_M3_ID:
        // case CAN_3508_M4_ID:
        // case CAN_YAW_MOTOR_ID:
        // case CAN_PIT_MOTOR_ID:
        // case CAN_TRIGGER_MOTOR_ID:
        // {
        //     static uint8_t i = 0;
        //     //get motor id
        //     i = rx_header.StdId - CAN_3508_M1_ID;
        //     get_motor_measure(&motor_chassis[i], rx_data);
        //     break;
        // }

        case SDO_SELF_ID:
            SDO_FRAME_t frame;
            // memcpy(&frame, rx_data, sizeof(SDO_FRAME_t));
            frame.ctrl = rx_data[0];
            frame.index = rx_data[1] | (rx_data[2] << 8);
            frame.subindex = rx_data[3];
            if(frame.ctrl == SEND_SUCCESS_FEEDBACK)
            {
                OD_Entry* entry = OD_GetEntry(od, frame.index, frame.subindex);
                uint8_t data_length = 0;
                switch (entry->dataType) {
                    case OD_TYPE_INT8:
                    case OD_TYPE_UINT8:
                        data_length = 1;
                        break;
                    case OD_TYPE_INT16:
                    case OD_TYPE_UINT16:
                        data_length = 2;
                        break;
                    case OD_TYPE_INT32:
                    case OD_TYPE_UINT32:
                        data_length = 4;
                        break;
                    default:
                        break;
                }
                uint8_t *recv_data = NULL;
                // memcpy(recv_data, rx_data + sizeof(recv_data), data_length);
                recv_data = rx_data + sizeof(SDO_FRAME_t);
                OD_SetValue(od, frame.index, frame.subindex, (uint8_t *)&rx_data[4]);
            }
            else if(frame.ctrl == SEND_FAILED_FEEDBACK)
            {
                //error code
            }
            break;
        

        default:
        {
            break;
        }
    }
}

// 统一的CAN SDO发送接口
bool CAN_SDO_Send(uint8_t motor_id, uint16_t index, uint8_t subindex, void* value, OD_DataType dataType) {
    uint32_t send_mail_box;
    
    // 设置CAN消息头
    chassis_tx_message.StdId = 0x600 + motor_id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    
    // 获取对象字典条目
    OD_Entry* entry = OD_GetEntry(od, index, subindex);
    if (entry == NULL || !entry->accessWrite) {
        return false;
    }
    
    // 确定数据长度
    uint8_t data_length_ctrl = 0;
    uint8_t data_length = 0;
    switch (dataType) {
        case OD_TYPE_INT8:
        case OD_TYPE_UINT8:
            data_length_ctrl = CAN_SEND_1_BYTE;
            data_length = 1;
            break;
        case OD_TYPE_INT16:
        case OD_TYPE_UINT16:
            data_length_ctrl = CAN_SEND_2_BYTE;
            data_length = 2;
            break;
        case OD_TYPE_INT32:
        case OD_TYPE_UINT32:
            data_length_ctrl = CAN_SEND_4_BYTE;
            data_length = 4;
            break;
        default:
            return false;
    }
    
    // 打包CAN数据
    chassis_can_send_data[0] = data_length_ctrl;
    chassis_can_send_data[1] = index & 0xFF;
    chassis_can_send_data[2] = (index >> 8) & 0xFF;
    chassis_can_send_data[3] = subindex;
    memcpy(&chassis_can_send_data[4], value, data_length);
    
    // 填充剩余字节为0
    for(int i = 4 + data_length; i < 8; i++) {
        chassis_can_send_data[i] = 0x00;
    }
    
    // 发送CAN消息
    return HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box) == HAL_OK;
}

void CAN_motor_enable(uint8_t motor_id) {
    uint16_t value = CONTROL_WORD_ABSOLUTE_POSITION_ENABLE;
    CAN_SDO_Send(motor_id, OD_INDEX_CONTROL_WORD, OD_SUBINDEX_DEFAULT, &value, OD_TYPE_UINT16);
}

void CAN_motor_mode(int8_t mode, uint8_t motor_id) {
    CAN_SDO_Send(motor_id, OD_INDEX_MODE_OF_OPERATION, OD_SUBINDEX_DEFAULT, &mode, OD_TYPE_INT8);
}

void CAN_motor_setProfileSpeed(uint32_t profile_speed, uint8_t motor_id) {
    CAN_SDO_Send(motor_id, OD_INDEX_PROFILE_SPEED, OD_SUBINDEX_DEFAULT, &profile_speed, OD_TYPE_UINT32);
}

void CAN_motor_setPos(int32_t position_ref, uint8_t motor_id) {
    CAN_SDO_Send(motor_id, OD_INDEX_TARGET_POS, OD_SUBINDEX_DEFAULT, &position_ref, OD_TYPE_INT32);
}

void CAN_motor_setSpeed(int32_t speed_ref, uint8_t motor_id) {
    CAN_SDO_Send(motor_id, OD_INDEX_TARGET_SPEED, OD_SUBINDEX_DEFAULT, &speed_ref, OD_TYPE_INT32);
}

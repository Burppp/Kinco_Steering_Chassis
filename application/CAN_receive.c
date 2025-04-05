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
            memcpy(&frame, rx_data, sizeof(SDO_FRAME_t));
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
                memcpy(recv_data, rx_data + sizeof(recv_data), data_length);
                OD_SetValue(od, frame.index, frame.subindex, (uint8_t *)recv_data);
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

void CAN_motor_enable(uint8_t motor_id)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x600 + motor_id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
//    chassis_can_send_data = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
    // chassis_can_send_data[0] = 0x2B;
    // chassis_can_send_data[1] = 0x40;
    // chassis_can_send_data[2] = 0x60;
    // chassis_can_send_data[3] = 0x00;
    // chassis_can_send_data[4] = 0x0F;
    // chassis_can_send_data[5] = 0x00;
    // chassis_can_send_data[6] = 0x00;
    // chassis_can_send_data[7] = 0x00;

    OD_Entry* entry = OD_GetEntry(od, OD_INDEX_CONTROL_WORD, OD_SUBINDEX_DEFAULT);
    uint16_t value = CONTROL_WORD_ENABLE;
    if (entry != NULL) {
        
    }
    if(entry->accessWrite)
    {
        uint8_t data_length_ctrl = 0;
        uint8_t data_length = 0;
        switch (entry->dataType)
        {
        case OD_TYPE_INT8:
        case OD_TYPE_UINT8:
            data_length_ctrl = SDO_SEND_1_BYTE;
            data_length = 1;
            break;
        case OD_TYPE_INT16:
        case OD_TYPE_UINT16:
            data_length_ctrl = SDO_SEND_2_BYTE;
            data_length = 2;
            break;
        case OD_TYPE_INT32:
        case OD_TYPE_UINT32:
            data_length_ctrl = SDO_SEND_4_BYTE;
            data_length = 4;
            break;
        default:
            break;
        }

        chassis_can_send_data[0] = data_length_ctrl;
        chassis_can_send_data[1] = entry->index & 0xFF;
        chassis_can_send_data[2] = (entry->index >> 8) & 0xFF;
        chassis_can_send_data[3] = entry->subindex;
        memcpy(&chassis_can_send_data[4], (uint8_t *)&value, data_length);
        for(int i = 4 + data_length;i < 8;i++)
            chassis_can_send_data[i] = 0x00;
        
        // chassis_can_send_data[4] = CONTROL_WORD_ENABLE & 0xFF;
        // chassis_can_send_data[5] = (CONTROL_WORD_ENABLE >> 8) & 0xFF;
        // chassis_can_send_data[6] = 0x00;
        // chassis_can_send_data[7] = 0x00;
    }
    else
    {
        return;
    }

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_motor_mode(int8_t mode, uint8_t motor_id)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x600 + motor_id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
//    chassis_can_send_data = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
    // chassis_can_send_data[0] = 0x2F;
    // chassis_can_send_data[1] = 0x60;
    // chassis_can_send_data[2] = 0x60;
    // chassis_can_send_data[3] = 0x00;
    // chassis_can_send_data[4] = mode;
    // chassis_can_send_data[5] = 0x00;
    // chassis_can_send_data[6] = 0x00;
    // chassis_can_send_data[7] = 0x00;

    OD_Entry* entry = OD_GetEntry(od, OD_INDEX_MODE_OF_OPERATION, OD_SUBINDEX_DEFAULT);
    uint16_t value = mode;
    if (entry != NULL) {

    }
    if(entry->accessWrite)
    {
        uint8_t data_length_ctrl = 0;
        uint8_t data_length = 0;
        switch (entry->dataType)
        {
        case OD_TYPE_INT8:
        case OD_TYPE_UINT8:
            data_length_ctrl = SDO_SEND_1_BYTE;
            data_length = 1;
            break;
        case OD_TYPE_INT16:
        case OD_TYPE_UINT16:
            data_length_ctrl = SDO_SEND_2_BYTE;
            data_length = 2;
            break;
        case OD_TYPE_INT32:
        case OD_TYPE_UINT32:
            data_length_ctrl = SDO_SEND_4_BYTE;
            data_length = 4;
            break;
        default:
            break;
        }

        chassis_can_send_data[0] = data_length_ctrl;
        chassis_can_send_data[1] = entry->index & 0xFF;
        chassis_can_send_data[2] = (entry->index >> 8) & 0xFF;
        chassis_can_send_data[3] = entry->subindex;
        memcpy(&chassis_can_send_data[4], (uint8_t *)&value, data_length);
        for(int i = 4 + data_length;i < 8;i++)
            chassis_can_send_data[i] = 0x00;
        
        // chassis_can_send_data[3] = SPEED_CONTROL_WITH_ACCELERATION & 0xFF;
        // chassis_can_send_data[4] = (SPEED_CONTROL_WITH_ACCELERATION >> 8) & 0xFF;
        // chassis_can_send_data[5] = 0x00;
        // chassis_can_send_data[6] = 0x00;
        // chassis_can_send_data[7] = 0x00;
    }
    else
    {
        return;
    }

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_motor_setSpeed(int32_t speed_ref, uint8_t motor_id)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x600 + motor_id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    // chassis_can_send_data[0] = 0x23;
    // chassis_can_send_data[1] = 0xFF;
    // chassis_can_send_data[2] = 0x60;
    // chassis_can_send_data[3] = 0x00;
    // chassis_can_send_data[4] = 0x00;
    // chassis_can_send_data[5] = 0x40;
    // chassis_can_send_data[6] = 0x06;
    // chassis_can_send_data[7] = 0x00;

    OD_Entry* entry = OD_GetEntry(od, OD_INDEX_TARGET_SPEED, OD_SUBINDEX_DEFAULT);
    int32_t real_speed_ref = speed_ref * RPM_TO_DEC;
    if (entry != NULL) {
        
    }
    if(entry->accessWrite)
    {
        uint8_t data_length_ctrl = 0;
        uint8_t data_length = 0;
        switch (entry->dataType)
        {
        case OD_TYPE_INT8:
        case OD_TYPE_UINT8:
            data_length_ctrl = SDO_SEND_1_BYTE;
            data_length = 1;
            break;
        case OD_TYPE_INT16:
        case OD_TYPE_UINT16:
            data_length_ctrl = SDO_SEND_2_BYTE;
            data_length = 2;
            break;
        case OD_TYPE_INT32:
        case OD_TYPE_UINT32:
            data_length_ctrl = SDO_SEND_4_BYTE;
            data_length = 4;
            break;
        default:
            break;
        }

        chassis_can_send_data[0] = data_length_ctrl;
        chassis_can_send_data[1] = entry->index & 0xFF;
        chassis_can_send_data[2] = (entry->index >> 8) & 0xFF;
        chassis_can_send_data[3] = entry->subindex;
        memcpy(&chassis_can_send_data[4], (uint8_t *)&real_speed_ref, data_length);
        for(int i = 4 + data_length;i < 8;i++)
            chassis_can_send_data[i] = 0x00;
    }
    else
    {
        return;
    }
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

#include "Lora.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"

#define LPUART1_REC_LEN 1024
uint8_t bRxBufferUart1[1]; //��������
uint8_t LPUART1_RX_BUF[LPUART1_REC_LEN];//��������
volatile uint16_t LPUART1_RX_LEN=0;
uint32_t DefaultTimeout=300;//��ʱ
/*LoRa ģ�� AT ָ���
* ����: uint_t *cmd����Ҫ���͵�����
* uint8_t *result��������õĽ��
* uint32_t timeOut���ȴ����������ʱ��
* uint8_t isPrintf���Ƿ��ӡ Log
*/
void LoRa_SendCmd(uint8_t *cmd, uint8_t *result, uint32_t timeOut, uint8_t isPrintf) {
    char *pos;
    HAL_UART_Transmit( &huart1, cmd,strlen((const char *)cmd), 0xff); //����AT ָ��
    HAL_UART_Receive_IT(&huart1,bRxBufferUart1,1);//�����͹��Ĵ��ڽ����ж�
    HAL_Delay(timeOut); //��ʱ�ȴ�
    while(1)
    {
        pos= strstr((const char *) LPUART1_RX_BUF, (const char *) result); //�����������"OK"
        //printf("receive:%s\r\n", LPUART1_RX_BUF);//��ӡ AT ָ��ص�����
        if (pos) {
            //printf("Success!\r\n");
            LPUART1_RX_LEN=0;
            memset(LPUART1_RX_BUF, 0, strlen((const char *)LPUART1_RX_BUF));//�������
            break; //��������ݣ�������
        }
        else{
            //printf("Fail!\r\n");
            LPUART1_RX_LEN=0;
            memset(LPUART1_RX_BUF, 0, strlen((const char *)LPUART1_RX_BUF));//�������
            HAL_UART_Transmit(&huart1, cmd, strlen((const char *) cmd), 0xff); //error ������Ӧ���ٴη���
            HAL_UART_Receive_IT(&huart1,bRxBufferUart1,1); // �����͹��Ĵ��ڽ����ж�
            HAL_Delay(timeOut);
        }
    }
}

/*LoRa ģ�飬͸�������Ե�ģʽ����
* uint8_t isPrintf:�Ƿ��ӡ Log
* uint8_t isReboot:�Ƿ�����
* ģ���ַ��10086 ͨ���ŵ���10 ���书�ʣ�11dbm
*/
void LoRa_T_P_Attach(uint8_t isPrintf,uint8_t isReboot){
    if(isReboot == 1)
    {
        //HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_Delay(1000);
        LoRa_SendCmd((uint8_t *)"AT+UART=7,0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+WLRATE=10,5\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+TPOWER=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+TMODE=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+WLTIME=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+CWMODE=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+ADDR=27,66\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        //HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        printf("Attach!\r\n");
    }
}

/** LoRa ģ�飬͸������㲥ģʽ����
* uint8_t isPrintf:�Ƿ��ӡ Log
* uint8_t isReboot:�Ƿ�����
* ģ���ַ��0xFFFF ͨ���ŵ���10 ���书�ʣ�11dbm
*/
void LoRa_T_V_Attach(uint8_t isPrintf,uint8_t isReboot)
{
    if(isReboot == 1)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_Delay(1000);
        LoRa_SendCmd((uint8_t *)"AT+UART=7,0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);//baudrate=115200����У��
        LoRa_SendCmd((uint8_t *)"AT+WLRATE=10,5\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);//�ŵ�10��19.2kbps
        LoRa_SendCmd((uint8_t *)"AT+TPOWER=3\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);//20dBm
        LoRa_SendCmd((uint8_t *)"AT+TMODE=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);//͸������
        LoRa_SendCmd((uint8_t *)"AT+WLTIME=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);//����ʱ��1s
        LoRa_SendCmd((uint8_t *)"AT+CWMODE=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);//һ��ģʽ
        LoRa_SendCmd((uint8_t *)"AT+ADDR=FF,FF\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);//��ַ65535
        LoRa_SendCmd((uint8_t *)"AT+FLASH=1\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);//����󱣴�
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        //printf("Attach!\r\n");
    }
}

/*
* LoRa ģ�飬�������Ե�ģʽ����
* uint8_t isPrintf:�Ƿ��ӡ Log
* uint8_t isReboot:�Ƿ�����
*/
void LoRa_D_P_Attach(uint8_t isPrintf,uint8_t isReboot)
{
    if(isReboot == 1)
    {
        //HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_Delay(1000);
        LoRa_SendCmd((uint8_t *)"AT+UART=7,0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+WLRATE=10,5\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+TPOWER=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+TMODE=1\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+WLTIME=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+CWMODE=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+ADDR=27,66\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        //HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        printf("Attach!\r\n");
    }
}

/*
* LoRa ģ�飬������㲥ģʽ����
* uint8_t isPrintf:�Ƿ��ӡ Log
* uint8_t isReboot:�Ƿ�����
*/
void LoRa_D_V_Attach(uint8_t isPrintf,uint8_t isReboot)
{
    if(isReboot == 1)
    {
        //HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_Delay(1000);
        LoRa_SendCmd((uint8_t *)"AT+UART=7,0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+WLRATE=10,5\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+TPOWER=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+TMODE=1\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+WLTIME=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+CWMODE=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
        LoRa_SendCmd((uint8_t *)"AT+ADDR=FF,FF\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
        //HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        printf("Attach!\r\n");
    }
}

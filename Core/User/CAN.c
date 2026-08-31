#include "main.h"
#include <string.h>
#include "CAN.h"
#include "CAN_MSG.h"
#include "SSPC.h"

extern CAN_HandleTypeDef hcan;
extern IWDG_HandleTypeDef hiwdg;
/*变量*/
volatile uint8_t CAN_RxFinish;
uint8_t CAN_SendBuff[8] = {0};
uint32_t CAN_ID = 0;

//设置Filter过滤，使能FIFO0，并不过滤任何信息
uint8_t bsp_can1_filter_config(void)
{
	CAN_FilterTypeDef filter = {0};
	filter.FilterActivation     = ENABLE;
	filter.FilterMode           = CAN_FILTERMODE_IDMASK;
	filter.FilterScale          = CAN_FILTERSCALE_32BIT;
	filter.FilterBank           = 0;
	filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	filter.FilterIdLow          = 0;
	filter.FilterIdHigh         = 0;
	filter.FilterMaskIdLow      = 0;
	filter.FilterMaskIdHigh     = 0;
	if (HAL_CAN_ConfigFilter (&hcan,&filter) != HAL_OK){return 1;} // 配置失败
  return 0;   // 配置成功
}

//CAN发送函数
uint8_t CAN_SendMsg(uint16_t msgID,uint8_t *Data)
{

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId  = msgID;
	TxHeader.RTR    = CAN_RTR_DATA;
	TxHeader.IDE    = CAN_ID_STD;
	TxHeader.DLC    = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
	uint8_t TxData[8];
	TxData[0] = *(Data+0);
	TxData[1] = *(Data+1);
	TxData[2] = *(Data+2);
	TxData[3] = *(Data+3);
	TxData[4] = *(Data+4);
	TxData[5] = *(Data+5);
	TxData[6] = *(Data+6);
	TxData[7] = *(Data+7);

    uint32_t can_tick = HAL_GetTick();
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
	{
		if(HAL_GetTick() - can_tick  >= can_wait_tick){return 1;}
		HAL_IWDG_Refresh(&hiwdg);
	}//等待有可用邮箱
	uint32_t TxMailbox;//用于返回邮箱编号
	if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox) != HAL_OK){return 1;}
	return 0;
}

/**
* @brief SSPC指令发送函数
* @param id  : 设备ID
* @param func: 功能码
* @param chn ：通道码
* @param val ：操作数
*/
void SSPC_SendCmd(uint16_t id,uint8_t func,uint8_t chn,uint32_t val)
{
	SSPC_CAN_Msg_t tx;
  tx.msg.id_high = (id >> 8) & 0xFF;
  tx.msg.id_low  = id & 0xFF;
  tx.msg.func    = func;
  tx.msg.channel = chn;
  tx.msg.data_3  = (val >> 24) & 0xFF;
  tx.msg.data_2  = (val >> 16) & 0xFF;
  tx.msg.data_1  = (val >> 8)  & 0xFF;
  tx.msg.data_0  = val & 0xFF;
  CAN_SendMsg (id, tx.buf);
}


/*CAN回调函数*/
static CAN_RxHeaderTypeDef RxMessage;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t data[8];
	HAL_StatusTypeDef status;
	status = HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMessage,data);
	if(status == HAL_OK)
	{
		CAN_RxFinish = 1;
		CAN_ID = RxMessage.StdId;
		memcpy(CAN_SendBuff,data,8);
		
	}
}

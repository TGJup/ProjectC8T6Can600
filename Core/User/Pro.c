#include "main.h"
#include <string.h>
#include "Serial.h"
#include "OLED.h"
#include "CAN.h"
#include "CAN_MSG.h"
#include "LED.h"
#include "SSPC.h"
#include "Pro.h"

extern IWDG_HandleTypeDef hiwdg;
KEY sky_gnd_key;
/*串口模块*/
extern UART_HandleTypeDef huart1;
extern uint8_t USART1_RxFrame[32];
extern uint8_t USART1_RxFinish;

/*SSPC模块*/
extern FC_SendData Read_data; 
extern uint8_t CAN_SendBuff[8];
extern uint8_t CAN_RxFinish;
static uint8_t SSPC_Open_Key;
static uint8_t SSPC_Close_Key;
static uint8_t Vbus28_Full_Flag;
static uint32_t tick_V28check;
static uint32_t Unlock_ENS_tick;
static uint8_t  SSPC_ACK_flag;
uint8_t SSPC_Data[8];

/*端口初始状态*/
CAN_CHN_Status can = {.chn1 = 0,.chn2 = 1,.chn3 = 1,.chn4 = 0,.chn5 = 0,.chn6 = 0,.chn7 = 1,.chn8 = 1};
OpCmd_t last_time[8];                //用于上一次记录
uint8_t lastOp_valid_flag[8];        //用于确认上一次操作
uint16_t SSPC_Lock_flag;             //锁定标志位

/*发动机*/
volatile uint32_t ENG_StartTick;
volatile uint32_t ENG_StopTick;
uint8_t ENG_Start_Lock1;			//接收到打火命令就置1,避免重复进入打火流程，0：可以进入点火流程，可以打开通道4
uint8_t ENG_Start_Lock2;			//发动机2启动控制
uint8_t ENG_Stop_Lock;
uint8_t Eng_Num_Flag;  				// 1：发动机1 0：发动机2

// 全局互斥标志：任何一台发动机正在启动计时中
uint8_t ENG_Start_In_Progress;

/**
*@brief 主进程
*@retval
*/
void Process(void)
{
	  uint32_t now = HAL_GetTick();
	  ENG_START_6S(now);
		if(sky_gnd_key.last_stable == GPIO_PIN_SET)/*statu:SKY*/
		{
			SSPC_Close_Key = 0;
			if(SSPC_Open_Key == 0){
				 SSPC_Open_Key = 1;
				 SSPC_Init(sky_gnd_key.last_stable);
				 LED_ON();
			}
		}
		else                            /*statu:GROUND*/
		{
			SSPC_Open_Key = 0;
			LED_TURN(now,Pro_LED);
			if(SSPC_Close_Key == 0)
			{
				SSPC_Close_Key = 1;
				Vbus28_Full_Flag = 0;
				SSPC_Init(sky_gnd_key.last_stable);
			}
		}
/*接收飞控指令*/
		if(USART1_RxFinish == 1)
			{
				USART1_RxFinish = 0;
				Fly_Control();
			}
		  
/*接收SSPC数据*/
		if(CAN_RxFinish == 1)
		{
	        memcpy(SSPC_Data,CAN_SendBuff,8);
			SSPC_Cmd(now,SSPC_Data);
			CAN_RxFinish = 0;
//			OLED_ShowCANWord();
		if(SSPC_Lock_flag)
		{
			SSPC_CHN_Unlock(now,SSPC_Data);
		}
        }
}

/**
*@brief  飞控控制信息处理
*@retval  电脑发送备用指令码：0xEB 0x92 0xFF 0x00 0x00 0x7C
*/
void Fly_Control(void)
{
	uint8_t IO_Cmd1 = 0;
	uint8_t IO_Cmd2 = 0;
	if(USART1_RxFrame[2] == 0xFF){SSPC_Set();SSPC_Open_Key = 0;}
	if(USART1_RxFrame[3] == 0xFF && USART1_RxFrame[4] ==0x00)
	{
		Eng_Num_Flag = 1;
		IO_Cmd1 = USART1_RxFrame[2];
		FC_IO_CMD(IO_Cmd1,1);
	}
	if(USART1_RxFrame[3] == 0x00 && USART1_RxFrame[4] ==0xFF)
	{
		Eng_Num_Flag = 0;
		IO_Cmd2 = USART1_RxFrame[2];
		FC_IO_CMD(IO_Cmd2,0);
	}
}

/**
*@brief  飞控通道控制
*@param  cmd:指令码
*@param  eng:发动机编号
*@retval
*/
void FC_IO_CMD(uint8_t cmd,uint8_t eng)
{
  switch(eng)
	{
		case 1:       /*发动机1*/
		    if(cmd&0x80)
			{
				if(ENG_Start_Lock1 != 1 && ENG_Start_In_Progress == 0)
				{
					ENG_Start_In_Progress = 1;
					ENG_Start_Lock1 = 1;				//发动机1点火打开通道4
				    ENG_StartTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_4,0);
				    LogChannelOp(CHN_4,SSPC_FUNC_CHN_OPEN,0);
				}
			}
			if(cmd&0x40)
			{
				if(ENG_Stop_Lock != 1)
				{
					ENG_Stop_Lock = 1;
				    ENG_StopTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_2,0);
				    LogChannelOp(CHN_2,SSPC_FUNC_CHN_CLOSE,0);
				}

			}
			if(cmd&0x08)
			{
                if(can.chn5 == 0)
                {
					can.chn5 = 1;
					SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_5,0);
					//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_6,0);
					//LogChannelOp(CHN_6,SSPC_FUNC_CHN_OPEN,0);
					LogChannelOp(CHN_5,SSPC_FUNC_CHN_OPEN,0);
				}
			}

			if(!(cmd&0x08)&&can.chn5 == 1)
			{
				can.chn5 = 0;
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5,0);
				//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0);
				LogChannelOp(CHN_5,SSPC_FUNC_CHN_CLOSE,0);
				//LogChannelOp(CHN_6,SSPC_FUNC_CHN_CLOSE,0);
			}
		break;
			
		case 0:       /*发动机2*/
			 if(cmd&0x80)
			{
				if(ENG_Start_Lock2 != 1 && ENG_Start_In_Progress == 0)
				{
					ENG_Start_In_Progress = 1;
					ENG_Start_Lock2 = 1;
				    ENG_StartTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_8,0);
				    LogChannelOp(CHN_8,SSPC_FUNC_CHN_OPEN,0);
				}
			}
			if(cmd&0x40)
			{
				if(ENG_Stop_Lock != 1)
				{
					ENG_Stop_Lock = 1;
				    ENG_StopTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_2,0);
				    LogChannelOp(CHN_2,SSPC_FUNC_CHN_CLOSE,0);
				}
			}
			if(cmd&0x08)
			{
                if(can.chn6 == 0)
                {
					can.chn6 = 1;
					//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_5,0);
					SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_6,0);
					LogChannelOp(CHN_6,SSPC_FUNC_CHN_OPEN,0);
					//LogChannelOp(CHN_5,SSPC_FUNC_CHN_OPEN,0);
				}
			if(!(cmd&0x08)&&can.chn6 == 1)
			{
				can.chn6 = 0;
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0);
				//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0);
				LogChannelOp(CHN_6,SSPC_FUNC_CHN_CLOSE,0);
				//LogChannelOp(CHN_6,SSPC_FUNC_CHN_CLOSE,0);
			}
			}break;
			
		default:
			switch(cmd)
			{
				case 0x04: can.chn7^=1;                                             
                   if(can.chn7 == 1)
                   {
						SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_7,0);
						LogChannelOp(CHN_7,SSPC_FUNC_CHN_OPEN,0);
					}
				    else 
					{
						SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_7,0);
						LogChannelOp(CHN_7,SSPC_FUNC_CHN_CLOSE,0);
					}
					break;
//		  case 0x02:                                                        break;
//	  	  case 0x01:                                                        break;
			}break;
	}
}

/**
 * @brief 收到SSPC指令解析
 * @retval 
 */
void SSPC_Cmd(uint32_t now,uint8_t *Data)
{
    switch(Data[2])
	{
		case SSPC_STAT_LOCK_ERR       : SSPC_Lock_flag ++;break;
		case SSPC_STAT_REPORT_VIN_TEMP :
		case SSPC_STAT_REPORT_VOUT_I  : SSPC_CHN_Read(now,&Read_data,Data);break;
		case SSPC_STAT_CMD_ACK        :	if(SSPC_Lock_flag!=0&&Data[6] == 0x4F &&Data[7] == 0x4B
			                               && SKY_GND_FLAG == GPIO_PIN_SET)
		                                 {
											SSPC_ACK_flag = 1;
										 };break;
	    default: break;
			}

}



/**
 * @brief 发动机点火非阻塞6s
 * @retval 
 */
void ENG_START_6S(uint32_t now)
{
	static uint8_t ENG_Start_Finsh_Lock1 = 1;		
	static uint8_t ENG_Stop_Finsh_Lock = 1;		
	static uint8_t ENG_Start_Finsh_Lock2 = 1;	
	if(ENG_Start_Lock1)							//是否接收到打火命令
	{
		if(now-ENG_StartTick <= ENG_START_DELAY)  // 少于6秒
		{
			ENG_Start_Lock1 = 1;					//点火时间少于6秒，	ENG_Start_Lock 保持为1		
			ENG_Start_Finsh_Lock1 = 0;			//点火时间少于6秒，ENG_Start_Finsh_Lock置0
		}
		else 	//超过六秒就会进入这个流程
		{
			if(ENG_Start_Finsh_Lock1 == 0)		//点火大于6秒
			{
				ENG_Start_Finsh_Lock1 = 1;		// 进入点火流程且大于6秒后，重新给ENG_Start_Finsh_Lock置1
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_4,0);	// 点火完毕，关闭通道4
				LogChannelOp(CHN_4,SSPC_FUNC_CHN_CLOSE,0);
			}
			ENG_Start_Lock1 = 0;					//可以重新进入点火流程
			ENG_Start_In_Progress = 0;  			//释放互斥，允许另一台发动机启动
		}
	}
	//发动机2点火六秒流程
	if(ENG_Start_Lock2)							//是否接收到打火命令
	{
		if(now-ENG_StartTick <= ENG_START_DELAY)  // 少于6秒
		{
			ENG_Start_Lock2 = 1;					//点火时间少于6秒，	ENG_Start_Lock 保持为1		
			ENG_Start_Finsh_Lock2 = 0;			//点火时间少于6秒，ENG_Start_Finsh_Lock置0
		}
		else 	//超过六秒就会进入这个流程
		{
			if(ENG_Start_Finsh_Lock2 == 0)		//点火大于6秒
			{
				ENG_Start_Finsh_Lock2 = 1;		// 进入点火流程且大于6秒后，重新给ENG_Start_Finsh_Lock置1
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_8,0);	// 点火完毕，关闭通道4
				LogChannelOp(CHN_8,SSPC_FUNC_CHN_CLOSE,0);
			}
			ENG_Start_Lock2 = 0;					//可以重新进入点火流程
			ENG_Start_In_Progress = 0;  			//释放互斥，允许另一台发动机启动
		}
	}	
  if(ENG_Stop_Lock && (now-ENG_StopTick <= ENG_STOP_DELAY))
  {
    ENG_Stop_Lock = 1;
	ENG_Stop_Finsh_Lock = 0;
  }
	else 
  {
	if(ENG_Stop_Finsh_Lock == 0)
	{
		ENG_Stop_Finsh_Lock = 1;
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0);
		LogChannelOp(CHN_2,SSPC_FUNC_CHN_OPEN,0);
	}
	ENG_Stop_Lock = 0;
  }
}

/**
 * @brief  等待SSPC启动回复
 * @retval 
 */
void Waiting_SSPC(void)
{
	uint32_t tick = HAL_GetTick();
	uint8_t temp = 1;
	 do
	{
		uint32_t now = HAL_GetTick();
		LED_TURN(now,Wait_SSPC_LED);
		if(now - tick >= SSPC_START_WAIT)
		{
		   tick = now;   
		   SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_ALL,0);
		 }   
		if(CAN_RxFinish == 1)
		{
			if(CAN_SendBuff[6] == 0x4F&&CAN_SendBuff[7] == 0x4B){temp = 0;}
		}
	 HAL_IWDG_Refresh(&hiwdg);
	 
	 
	}while(temp);
}


/**
 * @brief  OLED显示SSPC数据
 * @retval 
 */
void OLED_ShowCANWord(void)
{
	OLED_ShowString(1,1,"DevID:",OLED_8X16);
	OLED_ShowHexNum(64,1,CAN_SendBuff[0],2,OLED_8X16);
	OLED_ShowHexNum(80,1,CAN_SendBuff[1],2,OLED_8X16);
	OLED_ShowString(1,16,"FUN  :",OLED_8X16);
	OLED_ShowHexNum(64,16,CAN_SendBuff[2],2,OLED_8X16);
	OLED_ShowString(1,32,"CHN  :",OLED_8X16);
	OLED_ShowHexNum(64,32,CAN_SendBuff[3],2,OLED_8X16);
	OLED_ShowString(1,48,"STATUS:",OLED_8X16);
	OLED_ShowHexNum(64,48,CAN_SendBuff[4],2,OLED_8X16);
	OLED_ShowHexNum(80,48,CAN_SendBuff[5],2,OLED_8X16);
	OLED_ShowHexNum(96,48,CAN_SendBuff[6],2,OLED_8X16);
	OLED_ShowHexNum(112,48,CAN_SendBuff[7],2,OLED_8X16);
	OLED_Update();
}

/**
* @brief SSPC配置
* @param 
*/
void SSPC_Set(void)
{
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_ALL,0);      //关闭功率通道
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_REPORT_CYC,0,0x1E8480);     //上报周期50ms
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_UVP,SSPC_CHN_ALL,0x2710);   //欠压10V
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_OVP,SSPC_CHN_ALL,0x7530);   //过压50V
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_5_8,0xC350);  //通道5-8额定电流50A
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_1,0x7530);    //通道1  额定电流30A
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_2,0x2710);    //通道234额定电流10A
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_3,0x2710);
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_4,0x2710);
	//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_SAVE_FLASH,0,0);              //保存
	
}

/**
* @brief SSPC初始化
* @param flag:地空开关标志位
* @param 
*/
void SSPC_Init(uint8_t flag)
{
	if(flag == GPIO_PIN_SET)
	{
	  SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_1,0);HAL_Delay(SSPC_SendDelay);
	  SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0);HAL_Delay(SSPC_SendDelay);   /*打开熄火开关*/
	  //SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_3,0);HAL_Delay(SSPC_SendDelay);   /*打开ECU*/
	 // SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_8,0);HAL_Delay(SSPC_SendDelay);   /*打开28V*/
      //SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_7,0); HAL_Delay(SSPC_SendDelay);  /*打开舵机*/
	 
	  LogChannelOp(CHN_1,SSPC_FUNC_CHN_CLOSE,0); 
	  LogChannelOp(CHN_2,SSPC_FUNC_CHN_OPEN,0);
	 // LogChannelOp(CHN_3,SSPC_FUNC_CHN_OPEN,0);
	 // LogChannelOp(CHN_8,SSPC_FUNC_CHN_OPEN,0);
	 // LogChannelOp(CHN_7,SSPC_FUNC_CHN_OPEN,0);
  }
	
	else if(flag == GPIO_PIN_RESET)
	{
		//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5_8,0);HAL_Delay(SSPC_SendDelay);
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5,0);HAL_Delay(SSPC_SendDelay);
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0);HAL_Delay(SSPC_SendDelay);
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_1,0);HAL_Delay(SSPC_SendDelay);   /*打开载荷*/
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0);HAL_Delay(SSPC_SendDelay);   /*打开熄火*/
		//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_3,0);HAL_Delay(SSPC_SendDelay);   /*打开ECU*/
		LogChannelOp(CHN_1,SSPC_FUNC_CHN_OPEN,0);
		LogChannelOp(CHN_2,SSPC_FUNC_CHN_OPEN,0);
		//LogChannelOp(CHN_3,SSPC_FUNC_CHN_OPEN,0);
		LogChannelOp(CHN_5,SSPC_FUNC_CHN_CLOSE,0);
		LogChannelOp(CHN_6,SSPC_FUNC_CHN_CLOSE,0);
		//LogChannelOp(CHN_7,SSPC_FUNC_CHN_CLOSE,0);
		//LogChannelOp(CHN_8,SSPC_FUNC_CHN_CLOSE,0);
  }
}

/**
 * @brief  SSPC解锁操作
 * @retval 
 */
void SSPC_CHN_Unlock(uint32_t now,uint8_t *data)
{
    static uint16_t time = 0; 
	static uint16_t count = 0;
	if(count != SSPC_Lock_flag)
	{
       count = SSPC_Lock_flag;
	   time  = 1;
	   SSPC_ACK_flag = 0;
	}
	
	if(time == 1)
	{
	    if(data[3] <= SSPC_CHN_4)
	    {
		    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_UNLOCK,SSPC_CHN_1_4,0);/*解除通道锁定*/
	    }
	    else 
	    {
		    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_UNLOCK,SSPC_CHN_5_8,0);
	    }
		Unlock_ENS_tick = HAL_GetTick();
		time = 2;
	}
	if(time == 2)
	{
		if(HAL_GetTick()-Unlock_ENS_tick >SSPC_UNLOCK_ENS)
		{
			time = 3;
		}
	}
	if(time == 3)
	{
		
	    if(SSPC_ACK_flag ==1)
		{
			SSPC_ACK_flag = 0;
			SSPC_Recover();
			SSPC_Lock_flag = 0;
		    time = 0;
		}

		
	}
}

/**
 * @brief  SSPC读取操作
 * @retval 
 */
void SSPC_CHN_Read(uint32_t now,FC_SendData* readdata,uint8_t data[8])
{
	switch(data[3])
	{
		case SSPC_CHN_1:   readdata->Vbus12H = data[4];readdata->Vbus12L = data[5];break;
		case SSPC_CHN_5:   readdata->Ichn5H = data[6];readdata->Ichn5L = data[7];  break;
		case SSPC_CHN_6:   readdata->Ichn6H = data[6];readdata->Ichn6L = data[7];  break;
	//case SSPC_CHN_7:   readdata->Ichn7H = data[6];readdata->Ichn7L = data[7];  break;
		case SSPC_CHN_8:   readdata->Ichn8H = data[6];readdata->Ichn8L = data[7];break;
		case SSPC_CHN_5_8: readdata->Vchn8H = data[4];readdata->Vchn8L = data[5];
							Vcheck_28Vbus(now,data[4],data[5]);
							break;
		default :break;
	}
}

/**
 * @brief  解锁操作记录
 * @retval 
 */
void Log_UnlockOp(uint16_t id)
{
	if(id == SSPC_CHN_1_4 )
	{
		LogChannelOp(CHN_1,SSPC_FUNC_UNLOCK,0);
	  LogChannelOp(CHN_2,SSPC_FUNC_UNLOCK,0);
	  LogChannelOp(CHN_3,SSPC_FUNC_UNLOCK,0);
	  LogChannelOp(CHN_4,SSPC_FUNC_UNLOCK,0);
	}
	else
 {
	 LogChannelOp(CHN_5,SSPC_FUNC_UNLOCK,0);
	 LogChannelOp(CHN_6,SSPC_FUNC_UNLOCK,0);
	 LogChannelOp(CHN_7,SSPC_FUNC_UNLOCK,0);
	 LogChannelOp(CHN_8,SSPC_FUNC_UNLOCK,0);
 }
}

/**
 * @brief  通道功能恢复程序
 * @retval 
 */
void SSPC_Recover(void)
{
	GetAllChnLastOp(last_time,lastOp_valid_flag);
	for(uint8_t ch=0;ch<8;ch++)
	{
		if(lastOp_valid_flag[ch] == 1 )
		{
			switch(last_time[ch].op_code)
			{
				case SSPC_FUNC_CHN_OPEN :
					switch(ch){
						case 0:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_1,0); break;
						case 1:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0); break;
						case 2:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_3,0); break;
						case 3: break;
						case 4:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_5,0);  break;
						case 5:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_6,0); break;
						case 6:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_7,0); break;
						case 7:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_8,0); break;
						default :break;
					}   break;
				case SSPC_FUNC_CHN_CLOSE: 
					 switch(ch){
						case 0:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_1,0); break;
						case 1:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_2,0); break;
						case 2:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_3,0); break;
						case 3:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_4,0); break;
						case 4:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5,0); break;
						case 5:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0); break;
						case 6:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_7,0); break;
						case 7:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_8,0); break;
						default :break;
					}   break;
				default :break;
			}
		}
		HAL_Delay(SSPC_SendDelay);
	}
}

/**
 * @brief  28V电压检测
 * @param  now:当前时间
 * @param  VH:电压高位
 * @param  VL:电压低位
 * @retval 
 */
void Vcheck_28Vbus(uint32_t now,uint8_t VH,uint8_t VL)
{
	static uint8_t counter = 0;
  if(Byte2_TO_U16(VH,VL) > 0x6B6C&& SKY_GND_FLAG == GPIO_PIN_SET&&counter == 0&& Vbus28_Full_Flag == 0)
	{
  	 SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_1,0);
     LogChannelOp(CHN_1,SSPC_FUNC_CHN_OPEN,0); 
	 }
	if(now - tick_V28check > Wait_SSPC_28V )
	{
		tick_V28check = now;
		if(Byte2_TO_U16(VH,VL) > 0x6B6C&& SKY_GND_FLAG == GPIO_PIN_SET&& Vbus28_Full_Flag == 0)
	   {
			 counter++; 
			 if(counter == 3)
		  {
		    counter = 0;
	      Vbus28_Full_Flag = 1;
	      SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_3,0);
	      LogChannelOp(CHN_3,SSPC_FUNC_CHN_CLOSE,0);
        return;				
	    }
		}
		 else{ counter = 0;return;}
	}
	else {return;}
	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM4)
  {
    GPIO_PinState now_io = SKY_GND_FLAG;
    if(now_io == sky_gnd_key.curr_read)
    {
        sky_gnd_key.filter_cnt++;
        if(sky_gnd_key.filter_cnt >= 20)
        {
            sky_gnd_key.last_stable = now_io;
            sky_gnd_key.filter_cnt = 20;
        }
    }
    else
    {
        sky_gnd_key.curr_read = now_io;
        sky_gnd_key.filter_cnt = 0;
    }
  }
}


/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved. 模板V1.4
 *
 * File			: MdlViewTech.c
 * Author		: Wang Renfei
 * Description	: V5产品平台统一UI显示模块，支持：
 			1. 中显(ViewTech)显示屏(全部页面),基于UART(RS485)
 			2. MCGS屏幕(ItemPage页面部分),基于UART(RS485)
 			3. MQTT远程配置，基于网络

			页面主要分为两大类型: ItemPage页面(用于显示Conf, Stat, Msg, Acq)，以及其他页面(如主页面、辅助页面等主要显示数值信息的)
 			ItemPage又可以进一步分为两类: 配置统计页面，消息采集页面
 				配置统计页面的核心工作是对 CONF_n_STAT_PROP 结构体的解析
 				消息采集页面的核心工作是通过访问接口获取已经存储的数据

 			ViewTech我们的显示算法是:
 			1. 在屏幕和设备上构造一一对应的显示元件(绝大部分是文本显示控件)，并把这个显示元件通过通讯地址关联起来，这个关联是在设计层面
 			2. 在设备上把当前页面待显示变量进行解析(主要是转码成文本)
 			3. 通过通讯接口写入屏幕对应的显示元件地址上，从而完成变量的显示

 			MCGS的其他页面，由mcgs通过modbusRTU协议访问具体独立的变量，合并在modbusRTU协议实现上，就如上位机访问一样，不做特殊处理。

 * Date			: 2017-3-11
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * V1.0	| 2005-2-28   |	king | 1. Create
 **************************************************************************/

/***************************************************************************
 						macro definition
***************************************************************************/
#define _MDL_GUI_C_							/* exclude redefinition */

/***************************************************************************
 include files
 ***************************************************************************/
/* 相关库文件 */
//#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "string.h"

/* 项目公共头文件 */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"
#include "MdlDataAccess.h"

/* 本级以及下级模块头文件 */
#include "MdlNet.h"
#include "MdlUARTnModbus.h"
#include "MdlGUI.h"
#include "BoardSupport.h"

#if SUPPORT_IEC101 || SUPPORT_IEC104
#include "Mdl101n104.h"
#endif

/***************************************************************************
 						global variables definition
***************************************************************************/

/***************************************************************************
 						ViewTech
***************************************************************************/
#if SUPPORT_SCREEN
#define OFFSCREEN_TIMEOUT							10			/* 默认息屏超时 */

/* 屏保 */
#define VT_REG_MODE_CONFIG							0x12		/* VIEWTECH 控制息屏亮度寄存器，写0实现息屏 */
#define VT_REG_R0ToRC_REG_CONFIRM					0x1D		/* VIEWTECH 写R0-Rc寄存器生效寄存器，写入0xA5应用修改 */
#define VT_VAL_DISABLE_OFF_SCREEN					0x16		/* VIEWTECH 关闭屏保 */

/* 配置页 */
#define VT_VAL_CONFIG_INPT_ITM_FIRST 				0			/* 数字键盘，Item点击操作对象-第一个值*/
#define VT_VAL_CONFIG_INPT_ITM_SECOND 				1			/* 数字键盘，Item点击操作对象-第二个值*/
#define VT_VAL_CONFIG_INPT_ITM_PASSWORD 			2			/* 密码键盘*/
#define VT_VAL_CONFIG_INPT_ITM_ASCII 				3			/* 英文键盘*/

#define VT_LEN_CONFIG_UNIT_PAGER 					12			/* 页码字段长度， 字节*/
#define VT_VAR_CONFIG_UNIT_TITLE					0x0100		/* 标题		62字节   */
#define VT_VAR_CONFIG_UNIT_LINE4					0x017C		/* 第四行内容	62字节   */

#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE1		0x01F0  	/* 第一行高亮图片，0关闭，1显示   */
#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE2		0x01F1		/* 第二行高亮图片，0关闭，1显示   */
#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE3		0x01F2  	/* 第三行高亮图片，0关闭，1显示   */
#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE4		0x01F3  	/* 第四行高亮图片，0关闭，1显示   */
#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE5		0x01F4  	/* 第五行高亮图片，0关闭，1显示   */
#define VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE6		0x01F5   	/* 第六行高亮图片，0关闭，1显示   */

#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE1_NON_2ND	0x3021		/* 显示项目第一行第一位（可编辑）   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE1_2ND		0x3020     	/* 显示项目第一行第二位（可编辑）   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE2_NON_2ND	0x3031     	/* 显示项目第二行第一位（可编辑）   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE2_2ND		0x3030     	/* 显示项目第二行第二位（可编辑）   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE3_NON_2ND	0x3041     	/* 显示项目第三行第一位（可编辑）   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE3_2ND		0x3040     	/* 显示项目第三行第二位（可编辑）   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE4_NON_2ND	0x3051   	/* 显示项目第四行第一位（可编辑）   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE4_2ND		0x3050    	/* 显示项目第四行第二位（可编辑）   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE5_NON_2ND	0x3061     	/* 显示项目第五行第一位（可编辑）   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE5_2ND		0x3060     	/* 显示项目第五行第二位（可编辑）   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE6_NON_2ND	0x3071     	/* 显示项目第六行第一位（可编辑）   */
#define VT_VAL_CONFIG_PNLKEY_CLICK_LINE6_2ND		0x3070     	/* 显示项目第六行第二位（可编辑）   */

#define VT_VAL_CONFIG_PNLKEY_CLICK_BACK				0x4000		/* 返回按键 */
#define VT_VAL_CONFIG_PNLKEY_CLICK_UP				0x4001		/* 向上按键 */
#define VT_VAL_CONFIG_PNLKEY_CLICK_DOWN				0x4002		/* 向下按键 */

/* 消息、采集页 */
#define VT_VAR_MSGACQ_UNIT_LINE1					0x0300		/* 第一行*/
#define VT_VAR_MSGACQ_UNIT_LINE4 					0x0360		/* 第四行*/
#define VT_VAR_MSGACQ_UNIT_LINE7 					0x03C0		/* 第七行*/
#define VT_VAR_MSGACQ_DEBUG_BTN_ENABLE				0x03FF		/* 调试按钮是否可用 */
#define VT_VAR_MSGACQ_COLOR_LINE1					0x0453		/* 第一行颜色描述指针 */
#define VT_VAR_MSGACQ_COLOR_LINE2					0x0463		/* 第二行颜色描述指针 */
#define VT_VAR_MSGACQ_COLOR_LINE3					0x0473		/* 第三行颜色描述指针 */
#define VT_VAR_MSGACQ_COLOR_LINE4					0x0483		/* 第四行颜色描述指针 */
#define VT_VAR_MSGACQ_COLOR_LINE5					0x0493		/* 第五行颜色描述指针 */
#define VT_VAR_MSGACQ_COLOR_LINE6					0x04A3		/* 第六行颜色描述指针 */
#define VT_VAR_MSGACQ_COLOR_LINE7					0x04B3		/* 第七行颜色描述指针 */
#define VT_VAR_MSGACQ_COLOR_LINE8					0x04C3		/* 第八行颜色描述指针 */

#define VT_VAL_MSGACQ_PNLKEY_CLICK_UP	 			0x4001     	/* 向上按键 */
#define VT_VAL_MSGACQ_PNLKEY_CLICK_DOWN	 			0x4002     	/* 向下按键 */

typedef struct {
	uint8 u8Title[VT_LEN_ITEM_CHAR];  			/* 62字节	标题*/
	uint8 u8TextLines[3][VT_LEN_ITEM_CHAR]; 	/* 3*62字节  3行文本*/
}CONF_DAT_FRAME1;

typedef struct {
	uint8 u8TextLines[3][VT_LEN_ITEM_CHAR];  	/* 3*62字节 3行文本*/
	uint16 uEditerStyleLines[6];				/* 6行编辑图片样式，0隐藏1单编辑器2双编辑器3大单编辑器	6*2字节   */
	uint8 u8UNIT_PAGER[12];						/* 页码 65535/65535		12字节   */
}CONF_DAT_FRAME2;

/***************************************************************************
						internal functions declaration
***************************************************************************/
BOOL CheckRxBuf_VT(UART_COMM* pUartComm, uint16 uDevAddr);
void HandlePageWelcome_VT(uint8 u8UartPort);
void HandlePageConf_VT(uint8 u8UartPort, uint16 uPnlKey);
void HandlePageMsgAcq_VT(uint8 u8UartPort, uint16 uPnlKey);
void DealGuiConfWinBack(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);
void DealGuiConfWinUp(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);
void DealGuiConfWinDown(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);

void LeftAlignString_MCGS(uint8* pDest, uint32 u32TextBufBLen);
ITEM_DATA_TYPE GetItemUIType(CONF_n_STAT_PROP* pDataAcsProp);

/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: ViewTech主页面、辅助页面、调试页面原型
| G/Out var		:
| Author		: Wang Renfei			Date	: 2020-2-15
\=========================================================================*/
__weak void HandlePageMain_VT(uint8 u8UartPort, uint16 uPnlKey)
{
}
__weak void HandlePageAssi_VT(uint8 u8UartPort)
{
}
__weak void HandlePageDebug_VT(uint8 u8UartPort, uint16 uPnlKey)
{
}
__weak void HandlePageSystem_VT(uint8 u8UartPort, uint16 uPnlKey)
{
}
__weak uint16 FindMsgAcqIDColor(DATA_PAGE DataPage, uint16 uMsgID)			/* MsgID 转 颜色 函数 */
{
	return VT_VAL_COLOR_YELLOW;
}
__weak void HandleExtScn_VT(uint8 u8UartPort)
{
}

/*==========================================================================
| Description	: 模块初始化
| G/Out var		:
| Author		: Wang Renfei			Date	: 2008-3-4
\=========================================================================*/
void TryViewTech(uint8 u8UartPort)
{
	BOOL bSuccess = FALSE;
	ITEM_PAGE_REQ* pPageReq = &g_UartComm[u8UartPort].ItemPageReq;
	OpenUartComm(u8UartPort, 115200UL, 0, 0);

	/* 尝试和两块屏幕通信，同时也是在读取屏幕类型 */
	if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_VAR, VT_VAR_DEVICE_TYPE, 1)) {
		pPageReq->VTCommVer = (VT_COMM_VER)g_UartComm[u8UartPort].u8TRBuf[VT_BUF_RDVAR+1];
		bSuccess = TRUE;
	}
	bSuccess |= ReadData_VT(u8UartPort, VT_DEV_EXT, VIEW_TECH_READ_VAR, VT_VAR_DEVICE_TYPE, 1);

	/* 只要有一块屏幕通信成功就算成功 */
	if(bSuccess) {
		/* 全局变量初始化 */
		pPageReq->u8PageRefresh1Hz_100ms = 0;
		pPageReq->u8SlowRefresh_1Hz = 0;
		pPageReq->u32ItemNo = 0;
		pPageReq->u8InputingType = 0;
		pPageReq->bUnprocessedInput = FALSE;
		pPageReq->u8Tmr_OffScn_1s = OFFSCREEN_TIMEOUT;
		pPageReq->DataPage = DATA_PAGE_MAIN;

		pPageReq->ExtPage = DATA_PAGE_TEMP_SINGLE;
		pPageReq->u8Tmr_ExtOffScn_1s = OFFSCREEN_TIMEOUT;
		pPageReq->u8ExtItemNo = 0;
	}
}

/*==========================================================================
| Description	: 驱动ViewTech屏幕任务
| G/Out var		:
| Author		: Wang Renfei			Date	: 2017-3-11
\=========================================================================*/
void RunViewTech(uint8 u8UartPort)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	ITEM_PAGE_REQ* pItemPageReq = &pUartComm->ItemPageReq;
	pUartComm->uTmr_Run_ms = 100;

	/* 主屏处理：根据触屏状态驱动息屏 */
	if(g_CommConf.u32GuiIdleTimeOut == 0) {
		WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_LED_CTL, 0x40, 1);			/* 确定点亮屏幕 */
	} else if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_REG, VT_REG_TOUCH_STATUS, 1)) {
		if((pUartComm->u8TRBuf[VT_BUF_RDREG] == 1) || (pUartComm->u8TRBuf[VT_BUF_RDREG] == 3)   /* (1按下，2抬起，3按下中，其他无效)*/
		    || ((g_Ctr.bBeepAbornmal || g_Ctr.bBeepProgram) && (pItemPageReq->DataPage == DATA_PAGE_MAIN))) /* 主页面报警 */
		{
			pItemPageReq->u8Tmr_OffScn_1s = g_CommConf.u32GuiIdleTimeOut;
			WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_LED_CTL, 0x40, 1);		/* 确定点亮屏幕 */

		/* 欢迎页面下，超过10秒进入熄屏 */
		} else if((pItemPageReq->DataPage != DATA_PAGE_WELCOME) && (pItemPageReq->u8Tmr_OffScn_1s == 0)) {	/* 如已超时,则跳转到欢迎页 */
			pItemPageReq->u8Tmr_OffScn_1s = OFFSCREEN_TIMEOUT;
            pItemPageReq->DataPage = DATA_PAGE_WELCOME;
			WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_LED_CTL, 0x40, 1);		/* 确定点亮屏幕 */
		}
	}

	/* 主屏处理：读取主输入按键的值，并检查是否需要换页 */
	if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_VAR, VT_VAR_PNLKEY, 1)) {
		/* PanelKey检测 */
		uint16 uPnlKey = g_UartComm[u8UartPort].u8TRBuf[VT_BUF_RDVAR]*0x100 + g_UartComm[u8UartPort].u8TRBuf[VT_BUF_RDVAR+1];	/* 获得PnlKey */
		if(uPnlKey) {
			/* 清除状态按钮之外的PNLKEY */
			if((uPnlKey < VT_VAL_PNLKEY_STATUS_BTN_MIN_VAL) || (uPnlKey > VT_VAL_PNLKEY_STATUS_BTN_MAX_VAL)) {
				ClearVar_VT(u8UartPort, VT_DEV_MON, VT_VAR_PNLKEY, 1);
			}
			/* 小于 VT_VAL_PNLKEY_MAX_PAGE_VAL 是页面切换指令，按钮ID值就是待切换VTPage */
			if(uPnlKey < VT_VAL_PNLKEY_MAX_PAGE_VAL) {
                /* 把PnlKey映射到DataPage */
                DATA_PAGE DataPage = DATA_PAGE_NULL;
                for(; (DataPage < MAX_DATA_PAGE) && (cnst_uDataPage2VTPageBtn[DataPage] != uPnlKey); DataPage++) {
                }
                if(DataPage >= MAX_DATA_PAGE) {
                    DataPage = DATA_PAGE_NULL;
                }
                /* 页面切换 */
			    if(pItemPageReq->DataPage != DataPage) {
                    /* 有些页面有专用初始化函数，内含pItemPageReq->DataPage = DataPage */
                    if((DataPage == DATA_PAGE_CONF) || (DataPage == DATA_PAGE_MSG) || (DataPage == DATA_PAGE_ACQ)) {
                        ProcItemPageInstr(DATA_USER_VIEWTECH, DataPage, 0xFFFFFFFFUL, NULL, NULL, pItemPageReq);
                    } else {    /* 通用初始化 */
                        pItemPageReq->DataPage = DataPage;
                    }
                    pItemPageReq->u8PageRefresh1Hz_100ms = 0;
                }
			}
		}

		/* 根据当前页面类型处理页面内容和请求 */
		switch(pItemPageReq->DataPage) {
			case DATA_PAGE_NULL:
			case DATA_PAGE_WELCOME:								/* 欢迎页面 */
				HandlePageWelcome_VT(u8UartPort);
				break;

			case DATA_PAGE_CONF:								/* 配置页面 */
				HandlePageConf_VT(u8UartPort, uPnlKey);
				break;

			case DATA_PAGE_MSG:									/* 消息页面 */
			case DATA_PAGE_ACQ:									/* 采集页面 */
				HandlePageMsgAcq_VT(u8UartPort, uPnlKey);
				break;

			case DATA_PAGE_MAIN:								/* 主页面 */
				HandlePageMain_VT(u8UartPort, uPnlKey);
				break;

			case DATA_PAGE_ASSIT:								/* 辅助页面 */
				HandlePageAssi_VT(u8UartPort);
				break;

			case DATA_PAGE_SYS:
				HandlePageSystem_VT(u8UartPort, uPnlKey);
				break;

			case DATA_PAGE_DEBUG:								/* 调试页面 */
				HandlePageDebug_VT(u8UartPort, uPnlKey);
				break;

			default:
				pItemPageReq->DataPage = DATA_PAGE_WELCOME;
		}

		/* 确保当前页面 */
		WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_PIC_ID, DataPage2VTPage(pItemPageReq->VTCommVer, pItemPageReq->DataPage), 2);

		/* 驱动蜂鸣器 */
		if((g_Ctr.bBeepAbornmal || g_Ctr.bBeepProgram) && (!CheckDebugVersion())) {
			WriteReg_VT(u8UartPort, VT_DEV_MON, 0x02, 20, 1);			/* 响0.2s */
		}
	}
	
	/* 处理扩展屏的输入和显示 */
#if ((DEVICE_TYPE == V5_YKF3) || (DEVICE_TYPE == V5_YKF5))
	HandleExtScn_VT(u8UartPort);
#endif

	/* 更新计时器 */
	Task_sleep(OS_TICK_KHz*pUartComm->uTmr_Run_ms);			/* 为防止刷新过快，屏幕有时对触控反应变慢，加入最小间隔 */
	if(pItemPageReq->u8PageRefresh1Hz_100ms == 0) {		        /* 产生1Hz信号 */
		pItemPageReq->u8PageRefresh1Hz_100ms = 10 - 1;          /* 0~9, 周期是 10*100ms = 1s */
		if(pItemPageReq->u8SlowRefresh_1Hz) {                   /* 0~3, 周期是 4 秒 */
			pItemPageReq->u8SlowRefresh_1Hz--;
		} else {
			pItemPageReq->u8SlowRefresh_1Hz = 3;
		}
		if(pItemPageReq->u8Tmr_OffScn_1s) {
    		pItemPageReq->u8Tmr_OffScn_1s--;
		}
		if(pItemPageReq->u8Tmr_ExtOffScn_1s) {
    		pItemPageReq->u8Tmr_ExtOffScn_1s--;
		}
	} else {
		pItemPageReq->u8PageRefresh1Hz_100ms--;
	}
}

/* 填充TxBuf并发送给VIEWTECH，并立即读取返回数据
 * ViewTechCmd支持VIEW_TECH_READ_REG/VIEW_TECH_READ_VAR
 * 当u8Act为VIEW_TECH_READ_DATA时，uAddr高8位会被传给VIEWTECH，
 * 返回值:TRUE：成功取到数据, FALSE:没取到数据 */
BOOL ReadData_VT(uint8 u8UartPort, uint16 uDevAddr, VIEW_TECH_CMD ViewTechCmd, uint16 uDataAddr, uint16 uWordLen)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	uint8* pTxBuf = pUartComm->u8TRBuf;
	*pTxBuf++ = uDevAddr/0x100;
	*pTxBuf++ = uDevAddr%0x100;
	pTxBuf++;
	*pTxBuf++ = ViewTechCmd;
	if(ViewTechCmd == VIEW_TECH_READ_VAR) { /*操作为16位字长时*/
		*pTxBuf++ = (uint8) (uDataAddr / 0x100);
	}
	*pTxBuf++ = (uint8) (uDataAddr & 0xFF);
	*pTxBuf++ = uWordLen;
	SendData_VT(u8UartPort, pTxBuf);
	if(!ReadFromUart(u8UartPort, 20)) {
	    return FALSE;
	} else {    /* 检查返回的数据 */
        uint16 uRxBufPt = pUartComm->uRxBufPt;
        uint16 uCRC = (uint16)(pUartComm->u8TRBuf[uRxBufPt - 2])*0x100 + pUartComm->u8TRBuf[uRxBufPt - 1];

        if((pUartComm->u8TRBuf[0]*0x100 + pUartComm->u8TRBuf[1] == uDevAddr)      /* 地址 */
            /* 数据长度对:需要更细致的校验，否则两台设备485连接，自动模式下会都停留在VT模式 */
            && (((pUartComm->u8TRBuf[2] == 2) && (uRxBufPt == 7) && (pUartComm->u8TRBuf[4] == 0xFF))   /* CRC校验结果应答 */
                || ((ViewTechCmd == VIEW_TECH_READ_REG) && (uRxBufPt == uWordLen + 8))
                || ((ViewTechCmd == VIEW_TECH_READ_VAR) && (uRxBufPt == uWordLen*2 + 9)))
            && (CalCRC16ForViewTech(pUartComm->u8TRBuf + 3, uRxBufPt - 5) == uCRC))     /* CRC */
        {
            pUartComm->uTimer_WatchDog_ms = 0;  /* 通讯成功 */
            pUartComm->bUartFuncTrySuc = TRUE;
            return TRUE;
        } else {
            return FALSE;
        }
	}
}

/* 发送1、2字节短指令到VIEWTECH屏幕基础寄存器区域，并立即返回。
 * 填充TxBuf并发送给VIEWTECH，并立即读取返回数据,
 * u8Len非1即2 */
void WriteReg_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint16 uValue, uint8 u8ByteLen)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	uint8* pTxBuf = pUartComm->u8TRBuf;
	*pTxBuf++ = uDevAddr/0x100;
	*pTxBuf++ = uDevAddr%0x100;
	pTxBuf++;
	*pTxBuf++ = VIEW_TECH_WRITE_REG;
	*pTxBuf++ = (uint8)(uDataAddr & 0xFF);
	if(u8ByteLen > 1) {
		*pTxBuf++ = uValue/0x100;
	}
	*pTxBuf++ = uValue & 0xFF;
	SendData_VT(u8UartPort, pTxBuf);
}
/* 发送指令到VIEWTECH屏幕扩展寄存器区，支持最长255字节的写入，并立即返回。
 * 填充TxBuf并发送给VIEWTECH，并立即读取返回数据
 * */
void WriteVar_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint8 *pValue, uint8 u8WordLen)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	uint8* pTxBuf = pUartComm->u8TRBuf;
	*pTxBuf++ = uDevAddr/0x100;
	*pTxBuf++ = uDevAddr%0x100;
	pTxBuf++;
	*pTxBuf++ = VIEW_TECH_WRITE_VAR;
	*pTxBuf++ = uDataAddr/0x100;
	*pTxBuf++ = uDataAddr%0x100;;
	for( ; u8WordLen != 0; u8WordLen--) {
		*pTxBuf++ = *pValue++;
		*pTxBuf++ = *pValue++;
	}
	SendData_VT(u8UartPort, pTxBuf);
}

/* 复位面板按钮 */
void ClearVar_VT(uint8 u8UartPort, uint16 uDevAddr, uint16 uDataAddr, uint8 u8WordLenToClear)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	uint8* pTxBuf = pUartComm->u8TRBuf;
	*pTxBuf++ = uDevAddr/0x100;
	*pTxBuf++ = uDevAddr%0x100;
	pTxBuf++;
	*pTxBuf++ = VIEW_TECH_WRITE_VAR;
	*pTxBuf++ = uDataAddr/0x100;
	*pTxBuf++ = uDataAddr&0xFF;
	while(u8WordLenToClear--) {
		*pTxBuf++ = 0;
		*pTxBuf++ = 0;
	}
	SendData_VT(u8UartPort, pTxBuf);
}

uint16 DataPage2VTPage(VT_COMM_VER VTCommVer, DATA_PAGE DataPage)
{
    if(VTCommVer < VT_COMM_VER_MERGE) {
        return cnst_uDataPage2VTPage[0][DataPage];
    } else {
        return cnst_uDataPage2VTPage[1][DataPage];
    }
}

/* 处理Viewtech欢迎页请求响应 */
void HandlePageWelcome_VT(uint8 u8UartPort)
{
    /* 如果报警，则切换到主页面 */
	if(g_Ctr.bBeepAbornmal || g_Ctr.bBeepProgram) {
        g_UartComm[u8UartPort].ItemPageReq.DataPage = DATA_PAGE_MAIN;

    /* 读取屏幕类型 */
	} else if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_VAR, VT_VAR_DEVICE_TYPE, 1)) {
		g_UartComm[u8UartPort].ItemPageReq.VTCommVer = (VT_COMM_VER)g_UartComm[u8UartPort].u8TRBuf[VT_BUF_RDVAR+1];
		if((g_UartComm[u8UartPort].ItemPageReq.u8Tmr_OffScn_1s == 0) && g_CommConf.u32GuiIdleTimeOut) {
			WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_LED_CTL, 0, 1);	/* 息屏 */
		}
	}
}

/*==========================================================================
| Description	: 处理配置页面请求响应
	行页面显示，大约有8行文本 或者 6行文本+1行标题(标题占两行)，页面数据，第0行为标题，第1~9行为文本，一行数据为60Byte
	由于单次数据长度最长为255-8 = 247，因此一次最多显示一半，整个页面因此需要分成2~3个窗口传输
| G/Out var		:
| Author		: Wang Renfei			Date	: 2017-3-11
\=========================================================================*/
void HandlePageConf_VT(uint8 u8UartPort, uint16 uPnlKey)
{
	ITEM_PAGE_REQ *pPageReq = &g_UartComm[u8UartPort].ItemPageReq;
	uint8* pTRBuf = g_UartComm[u8UartPort].u8TRBuf;
	int16 iLineNo = -1;
	uint8 u8BytesToWrite[12];

	uint8 u8ItemCharLen = VT_LEN_ITEM_CHAR;
	if(pPageReq->VTCommVer == VT_VAL_DEV_SDW043) {
		u8ItemCharLen = 46;
	}

	/* 记录未处理的输入 */
	if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_REG, VT_REG_INPUT_STATUS, 1) && pTRBuf[VT_BUF_RDREG]) {	/* 如果结果非0代表输入中 */
		pPageReq->bUnprocessedInput = TRUE;
	} else if(pPageReq->bUnprocessedInput) {
		/* 如果是录入结束状态，则处理先前的输入 */
		if(ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_VAR, VT_VAR_KEYBOARD_STATUS, 2)
			&& (pTRBuf[VT_BUF_RDVAR] == 0x5A))			/*第一字节是输入状态 */
		{
			uint8 u8InputByteLen = pTRBuf[VT_BUF_RDVAR+1];	 /*第二字节是输入长度*/
			/*读取在寄存器VIEWADD_KEYBOARD_START的输入内容*/
			if(u8InputByteLen && ReadData_VT(u8UartPort, VT_DEV_MON, VIEW_TECH_READ_VAR, VT_VAR_KEYBOARD_INPUT, u8InputByteLen/2 + 1)) {
				uint8* pU8Instr = NULL;
				if (pPageReq->u8InputingType == VT_VAL_CONFIG_INPT_ITM_SECOND) { /*隐含意思：点击标题、第一个值、密码类型 都是针对第一个值的修改*/
					pTRBuf[6] = 0; /*借用前一位写0，标识要修改第二个值*/
					pU8Instr = pTRBuf + 6;
					u8InputByteLen++;
				} else {
					pU8Instr = pTRBuf + 7;
				}
				*(pU8Instr + u8InputByteLen) = 0; /*字符串末尾的0xFF替换成标准的0*/
				ProcItemPageInstr(DATA_USER_VIEWTECH, pPageReq->DataPage, pPageReq->u32ItemNo, &pU8Instr, pU8Instr + u8InputByteLen, pPageReq);
			}
			/*取消高亮*/
			memset(u8BytesToWrite, 0, 12);
			WriteVar_VT(u8UartPort, VT_DEV_MON, VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE1, u8BytesToWrite, 6); /* 高亮图片，0关闭，1显示   */
		}
		pPageReq->bUnprocessedInput = FALSE;
	} else {	/* 如果没有进行中的键盘输入 */
		/* 屏幕按钮处理 */
		uint16 uItemNo;
		CONF_n_STAT_PROP* pDataAcsProp;
		ITEM_DATA_TYPE ItemUIType;
		switch(uPnlKey) {
			case VT_VAL_CONFIG_PNLKEY_CLICK_BACK: /* 返回按键 */
				if(pPageReq->uConfPageTopicItemNo > 0) {	/* 非首页才需要执行返回 */
					DealGuiConfWinBack(DATA_USER_VIEWTECH, pPageReq);
				} else {
					/* 返回辅助页面 */
					pPageReq->DataPage = DATA_PAGE_ASSIT;
					return;
				}
				break;

			case VT_VAL_CONFIG_PNLKEY_CLICK_UP: /* 向上按键 */
				DealGuiConfWinUp(DATA_USER_VIEWTECH, pPageReq);
				break;

			case VT_VAL_CONFIG_PNLKEY_CLICK_DOWN: /* 向下按键 */
				DealGuiConfWinDown(DATA_USER_VIEWTECH, pPageReq);
				break;

			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE1_NON_2ND: /* 显示项目第一行第一位点击事件   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE1_2ND: /* 显示项目第一行第二位点击事件   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE2_NON_2ND: /* 显示项目第二行第一位点击事件   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE2_2ND: /* 显示项目第二行第二位点击事件   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE3_NON_2ND: /* 显示项目第三行第一位点击事件   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE3_2ND: /* 显示项目第三行第二位点击事件   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE4_NON_2ND: /* 显示项目第四行第一位点击事件   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE4_2ND: /* 显示项目第四行第二位点击事件   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE5_NON_2ND: /* 显示项目第五行第一位点击事件   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE5_2ND: /* 显示项目第五行第二位点击事件   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE6_NON_2ND: /* 显示项目第六行第一位点击事件   */
			case VT_VAL_CONFIG_PNLKEY_CLICK_LINE6_2ND: /* 显示项目第六行第二位点击事件   */
				iLineNo = ((uPnlKey - VT_VAL_CONFIG_PNLKEY_CLICK_LINE1_2ND) >> 4); /* start from 0*/
				uItemNo = pPageReq->uConfWinFirstLineItemNo + iLineNo;
				pDataAcsProp = pBootLoaderConf->pConfProp + uItemNo;
				ItemUIType = GetItemUIType(pDataAcsProp);
				if(ItemUIType == ITEM_UI_CLICK) {
					uint8* pU8Instr = (uint8*)"click";
					ProcItemPageInstr(DATA_USER_VIEWTECH, pPageReq->DataPage, uItemNo, &pU8Instr, pU8Instr + 6, pPageReq);
				} else if((ItemUIType == ITEM_UI_ONEBOX)
						|| (ItemUIType == ITEM_UI_TWOBOX)
						|| (ItemUIType == ITEM_UI_ONEDBOX)
						|| (ItemUIType == ITEM_UI_ANSI)			/* 保彩辉修改：否则ANSI类型无法编辑 */
						|| (ItemUIType == ITEM_UI_LOGIN)) 
				{
					/* 高亮该行 */
					u8BytesToWrite[0] = 0;
					u8BytesToWrite[1] = 1;
					WriteVar_VT(u8UartPort, VT_DEV_MON, VT_VAR_CONFIG_UNIT_HIGHLIGHTER_LINE1 + iLineNo, u8BytesToWrite, 1); /* 高亮图片，0关闭，1显示   */

					/*判断数字录入还是ascii录入*/
					switch((pBootLoaderConf->pConfProp + uItemNo)->ItemType.DataType) {
						case ITEM_DAT_ANSI:
						case ITEM_DAT_RMTS:
							pPageReq->u8InputingType = VT_VAL_CONFIG_INPT_ITM_ASCII;
							WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_KEYBOARD_CTL, VT_VAL_KEYBOARD_ASCII, 1);
							break;

						default:
							if(ItemUIType == ITEM_UI_LOGIN) {		/*呼叫密码键盘  */
								pPageReq->u8InputingType = VT_VAL_CONFIG_INPT_ITM_PASSWORD;
								WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_KEYBOARD_CTL, VT_VAL_KEYBOARD_PWD, 1);
							} else { 								/*呼叫数字键盘  */
								if(ItemUIType == ITEM_UI_TWOBOX && (uPnlKey & 3)==0) {
									pPageReq->u8InputingType = VT_VAL_CONFIG_INPT_ITM_SECOND;
								} else {
									pPageReq->u8InputingType = VT_VAL_CONFIG_INPT_ITM_FIRST;
								}
								WriteReg_VT(u8UartPort, VT_DEV_MON, VT_REG_KEYBOARD_CTL, VT_VAL_KEYBOARD_NUM, 1);
							}
					}
					/*开启输入状态*/
					pPageReq->u32ItemNo = uItemNo;
				}
				break;

			default:
				;
		}
	}

	/* 需要中速刷新的 */
	if(pPageReq->u8PageRefresh1Hz_100ms == 0) {
		pTRBuf = g_UartComm[u8UartPort].u8TRBuf;
    	*pTRBuf++ = VT_DEV_MON/0x100;
    	*pTRBuf++ = VT_DEV_MON%0x100;
		pTRBuf++;
		*pTRBuf++ = VIEW_TECH_WRITE_VAR;

		/* 第一帧 */
		*pTRBuf++ = VT_VAR_CONFIG_UNIT_TITLE/0x100;
		*pTRBuf++ = VT_VAR_CONFIG_UNIT_TITLE&0xFF;
		CONF_DAT_FRAME1 *pConfFrame1 = (CONF_DAT_FRAME1*)(pTRBuf);
		InitDataWithZero(pTRBuf, sizeof(CONF_DAT_FRAME1));
		/* 标题行 */
		CONF_n_STAT_PROP* pTopicAcsProp = pBootLoaderConf->pConfProp + pPageReq->uConfPageTopicItemNo;
		pTRBuf = PrintLocalConfItem(DATA_USER_VIEWTECH, pTopicAcsProp, pConfFrame1->u8Title, u8ItemCharLen*0.72);	/*0.72是标题段落和正文字体大小比例修正*/
		/* 第1~3行 */
		uint16 uCurLineItemNo = pPageReq->uConfWinFirstLineItemNo;
		uint16 uCurPageMaxItemNo = pTopicAcsProp->LkId_Val_ItN + pPageReq->uConfPageTopicItemNo;
		for(iLineNo = 0; iLineNo < 3; iLineNo++) {
			if(uCurLineItemNo <= uCurPageMaxItemNo) {
				pTRBuf = PrintLocalConfItem(DATA_USER_VIEWTECH, pBootLoaderConf->pConfProp + uCurLineItemNo, pConfFrame1->u8TextLines[iLineNo], u8ItemCharLen);
			}
			uCurLineItemNo++;
		}
		SendData_VT(u8UartPort, (uint8*)(pConfFrame1 + 1));	/* 发送（标题+1～3行一帧） */

		/* 第二帧 */
		pTRBuf = &g_UartComm[u8UartPort].u8TRBuf[4];
		*pTRBuf++ = VT_VAR_CONFIG_UNIT_LINE4/0x100;
		*pTRBuf++ = VT_VAR_CONFIG_UNIT_LINE4&0xFF;
		CONF_DAT_FRAME2 *pConfFrame2 = (CONF_DAT_FRAME2*)(pTRBuf);
		InitDataWithZero(pTRBuf, sizeof(CONF_DAT_FRAME2));
		/* 第4~6行 */
		uCurLineItemNo = pPageReq->uConfWinFirstLineItemNo + 3;
		for(iLineNo = 0; iLineNo < 3; iLineNo++) {
			if(uCurLineItemNo <= uCurPageMaxItemNo) {
				pTRBuf = PrintLocalConfItem(DATA_USER_VIEWTECH, pBootLoaderConf->pConfProp + uCurLineItemNo, pConfFrame2->u8TextLines[iLineNo], u8ItemCharLen);
			}
			uCurLineItemNo++;
		}

		/* 填充六行编辑图片样式，0隐藏1单编辑器2双编辑器	2字节   */
		uCurLineItemNo = pPageReq->uConfWinFirstLineItemNo;
		for(iLineNo = 0; iLineNo < 6; iLineNo++) {
			switch(GetItemUIType(pBootLoaderConf->pConfProp + uCurLineItemNo)) {
				case ITEM_UI_ONEDBOX:
				case ITEM_UI_ANSI:
					pConfFrame2->uEditerStyleLines[iLineNo] = 0x03<<8;
					break;

				case ITEM_UI_TWOBOX:
					pConfFrame2->uEditerStyleLines[iLineNo] = 0x02<<8;
					break;
				case ITEM_UI_ONEBOX:
					pConfFrame2->uEditerStyleLines[iLineNo] = 0x01<<8;
					break;
				case ITEM_UI_TOPIC:
				case ITEM_UI_TEXT:
				case ITEM_UI_CLICK:
				case ITEM_UI_LOGIN:
				default:
					pConfFrame2->uEditerStyleLines[iLineNo] = 0x00<<8;
			}
			uCurLineItemNo++;
		}
		/* 填充页码 65535/65535	12字节   */
		pTRBuf = pConfFrame2->u8UNIT_PAGER;
		PrintU32WithLen(&pTRBuf, pPageReq->uConfWinFirstLineItemNo - pPageReq->uConfPageTopicItemNo, 5);
		*pTRBuf++ = '/';
		PrintU32WithLen(&pTRBuf, pTopicAcsProp->LkId_Val_ItN, 5);
		SendData_VT(u8UartPort, (uint8*)(pConfFrame2 + 1));	/* 4～6行+剩下的内容一帧 */
	}
}

/*处理消息\采集页面请求响应*/
void HandlePageMsgAcq_VT(uint8 u8UartPort, uint16 uPnlKey)
{
	ITEM_PAGE_REQ *pPageReq = &g_UartComm[u8UartPort].ItemPageReq;
	BOOL bPnlWinUpOrWinDown = FALSE;

	/* 屏幕按钮处理 */
	switch(uPnlKey) {
		case VT_VAL_MSGACQ_PNLKEY_CLICK_UP:		/* 按了向上箭头 */
			if(pPageReq->VTCommVer == VT_VAL_DEV_SDW043) {
				pPageReq->u32ItemNo += 6;
			} else {
				pPageReq->u32ItemNo += 16;
			}
			bPnlWinUpOrWinDown = TRUE;
			break;

		case  VT_VAL_MSGACQ_PNLKEY_CLICK_DOWN:	/* 按了向下箭头 */
			if(pPageReq->u32ItemNo > 1) {
				bPnlWinUpOrWinDown = TRUE;
			}
			break;

		default:;
	}
	
	/* 需要刷新页面的，页面切换的时候，pPageReq->u32ItemNo会被初始化成 0xFFFFFFFFUL */
	if((pPageReq->u32ItemNo == 0xFFFFFFFFUL) || bPnlWinUpOrWinDown) {
		uint16 uMsgAcqID[3];
		int8 i;
		uint8* pTxBufEnd = &g_UartComm[u8UartPort].u8TRBuf[UART_TR_BUF_BLEN] - 2;	/* 需要空出CRC */
		uint8* pTRBuf = g_UartComm[u8UartPort].u8TRBuf;		
    	*pTRBuf++ = VT_DEV_MON/0x100;
    	*pTRBuf++ = VT_DEV_MON%0x100;
		pTRBuf++;
		*pTRBuf++ = VIEW_TECH_WRITE_VAR;

		/* 第一帧 */
		*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE1/0x100;
		*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE1&0xFF;
		int32 i32ReadItemNum = -3;
		GetTextForMsgOrAcq(DATA_USER_VIEWTECH, pPageReq->DataPage, &pPageReq->u32ItemNo, &i32ReadItemNum, &pTRBuf, pTxBufEnd);
		SendData_VT(u8UartPort, pTRBuf);	/* 发送 */

		/* 设置前3行字体颜色 */
		for(i = 0; i < 3; i++) {			/* 记录MsgAcqID */
			uMsgAcqID[i] = *((uint16*)(&g_UartComm[u8UartPort].u8TRBuf[6 + VT_LEN_ITEM_BYTE*(i+1) - 2]));
		}
		if(pPageReq->VTCommVer == VT_VAL_DEV_SDW043) {
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE1, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[0]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE2, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[0]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE3, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[1]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE4, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[1]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE5, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[2]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE6, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[2]));
		} else {		/* 非4.3寸屏打印出第二帧和第三帧，即剩余的5条消息 */
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE1, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[0]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE2, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[1]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE3, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[2]));
			/* 第二帧 */
			pTRBuf = &g_UartComm[u8UartPort].u8TRBuf[4];
			*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE4/0x100;
			*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE4&0xFF;
			i32ReadItemNum = -3;
			GetTextForMsgOrAcq(DATA_USER_VIEWTECH, pPageReq->DataPage, &pPageReq->u32ItemNo, &i32ReadItemNum, &pTRBuf, pTxBufEnd);
			SendData_VT(u8UartPort, pTRBuf);	/* 发送 */

			/* 设置中3行字体颜色 */
			for(i = 0; i < 3; i++) {			/* 记录MsgAcqID */
				uMsgAcqID[i] = *((uint16*)(&g_UartComm[u8UartPort].u8TRBuf[6 + VT_LEN_ITEM_BYTE*(i+1) - 2]));
			}
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE4, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[0]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE5, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[1]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE6, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[2]));

			/* 第三帧 */
			pTRBuf = &g_UartComm[u8UartPort].u8TRBuf[4];
			*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE7/0x100;
			*pTRBuf++ = VT_VAR_MSGACQ_UNIT_LINE7&0xFF;
			i32ReadItemNum = -2;
			GetTextForMsgOrAcq(DATA_USER_VIEWTECH, pPageReq->DataPage, &pPageReq->u32ItemNo, &i32ReadItemNum, &pTRBuf, pTxBufEnd);
			SendData_VT(u8UartPort, pTRBuf);	/* 发送 */
			/* 设置后2行字体颜色 */
			for(i = 0; i < 2; i++) {			/* 记录MsgAcqID */
				uMsgAcqID[i] = *((uint16*)(&g_UartComm[u8UartPort].u8TRBuf[6 + VT_LEN_ITEM_BYTE*(i+1) - 2]));
			}
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE7, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[0]));
			SetMsgAcqLineColor(u8UartPort, VT_VAR_MSGACQ_COLOR_LINE8, FindMsgAcqIDColor(pPageReq->DataPage, uMsgAcqID[1]));
		}
	}
	/* 检查调试按钮是否可用 */
	uint16 uDbgEnb = g_Ctr.bEngDebug;		/* 扩展成16位 */
	WriteVar_VT(u8UartPort, VT_DEV_MON, VT_VAR_MSGACQ_DEBUG_BTN_ENABLE, (uint8*)&uDbgEnb, 1);
}

/*==========================================================================
| Description	: 把待发送缓冲区的数据, 加上长度标志与CRC, 发送出去
| G/Out var		:
| Author		: Wang Renfei			Date	: 2017-3-11
\=========================================================================*/
void SendData_VT(uint8 u8UartPort, uint8* pTxBuf)
{
	uint8* pU8TRBufStart = g_UartComm[u8UartPort].u8TRBuf;	/* 用指针方式更快 */

	/* 填写发送数据长度与CRC */
	uint32 u32NeedTxByte = pTxBuf - pU8TRBufStart;
	if(u32NeedTxByte > 3) {
		uint16 uCRC = CalCRC16ForViewTech(pU8TRBufStart + 3, u32NeedTxByte - 3);
		*pTxBuf++ = uCRC/0x100;
		*pTxBuf++ = uCRC&0xFF;
		u32NeedTxByte += 2;
		pU8TRBufStart[2] = u32NeedTxByte - 3;
	} else {
		pU8TRBufStart[2] = 0;
	}
	WriteToUart(u8UartPort, u32NeedTxByte);	/* 发送数据 */
}

/* 拷贝最多256个字符串，带长度限制，strcpy费内存*/
uint8 PrintStringFromText_VT(uint8 *pDstBuf, const char *pSrcText, uint8 u8MaxLen)
{
    uint8 u8Count = 0;
    for(; u8Count < u8MaxLen; u8Count++) {
        if(*pSrcText == 0) {
			*pDstBuf = 0;
            return u8Count + 1;
        } else {
            *pDstBuf++ = *pSrcText++;
        }
    }
    return u8MaxLen;
}

#if TEMP_SEN_PLC_NUM
/* 打印温度名称,成功则返回最后填充位置+1,如果温度类型是TEMP_SEN_NUL,则什么都不打印,直接返回pText */
uint8* PrintTemperatureName(uint8* pText, TEMPERATURE_DATA* pTempData, char TailChar)
{
	int16 i8Placement = pTempData->i8Placement;
	/* 传感器断线判断 */
	if(i8Placement != TEMP_SEN_NUL) {
		if(i8Placement < 0) {
			i8Placement = 0 - i8Placement;
		}
		if(i8Placement >= TEMP_SEN_PLC_NUM) {	/* 防止后续访问溢出 */
			i8Placement = TEMP_SEN_PLC_ANY;
		}

		/* 提示符部分 */
		const char* pVarName = cnst_TempPlacementName[i8Placement][(LANG_TYPE)g_Sys.u32LocalLang];
		while(*pVarName) {
			*pText++ = *pVarName++;
		}
		uint8 u8PlaceNo = pTempData->u8PlaceNo;
		if(u8PlaceNo) {
			if(u8PlaceNo >= 10) {
				*pText++ = '0' + u8PlaceNo/10;
				u8PlaceNo = u8PlaceNo%10;
			}
			*pText++ = '0' + u8PlaceNo;
		}
	}
    *pText++ = (uint8)TailChar;
	return pText;
}

/* 打印温度数值,成功则返回最后填充位置+1,如果温度类型是TEMP_SEN_NUL,则什么都不打印,直接返回pText*/
uint8* PrintTemperatureValue(uint8* pText, TEMPERATURE_DATA* pTempData, const char* pDiscText)
{
	int16 i8Placement = pTempData->i8Placement;
	if(i8Placement != TEMP_SEN_NUL) {
		/* 温度数值部分 */
		if((i8Placement < 0)) { 	/* 如传感器断线则打印pDiscText内容 */
			while(*pDiscText) {
				*pText++ = *pDiscText++;
			}
		} else {
			PrintI16(&pText, pText+10, pTempData->iDegree_100mC, 1);
		}
	}
	*pText++ = 0;
	return pText;
}
#endif
/*==========================================================================
| MCGS屏幕ItemPage部分，用于 统计&配置&事件消息&采集参数
\=========================================================================*/
/*==========================================================================
| 以下函数为MCGS屏幕进行ItemPage访问所需要的
| 用于显示: 配置页面、统计页面等具有行/项目特征的页面
| 用于保存:
\=========================================================================*/
/*==========================================================================
| Description	: 填充ItemPage的响应行
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2015-12-26
\=========================================================================*/
void FillItemPageAck_MCGS(ITEM_PAGE_REQ* pItemPageReq, uint8* pMCGSItemPageAck)
{
	#define MCGS_TOPIC_TEXT_BLEN	30
	CONF_n_STAT_PROP* pDataAcsProp;
	uint8* pTextStart = pMCGSItemPageAck + MCGS_ITEM_PAGE_PAYLOAD_BSTART;	/* 需要在前面空出ItemType */
	uint32 u32ItemNo = pItemPageReq->u32ItemNo;
	ITEM_DATA_TYPE ItemDataType;

	/* 不同类型的页面处理 */
	if((pItemPageReq->DataPage == DATA_PAGE_MSG) || (pItemPageReq->DataPage == DATA_PAGE_ACQ)) {
		uint8* pText = pTextStart + MCGS_ITEM_PAGE_ADDI_BLEN;		/* 需要空出调整的空间，MCGS屏幕汉字必须16位对齐 */
		int32 i32ItemNum_InNeed_OutSuc;

		if((u32ItemNo == 0xFFFFFFFFUL) || pItemPageReq->bMsgAcqPageWinDown) {	/* 复位 或 往后读 */
			i32ItemNum_InNeed_OutSuc = -1;
		} else {											/* 往后读 */
			i32ItemNum_InNeed_OutSuc = 1;
		}
		GetTextForMsgOrAcq(DATA_USER_MCGS, pItemPageReq->DataPage, &u32ItemNo, &i32ItemNum_InNeed_OutSuc, &pText, pText + MCGS_LEN_ITEM_CHAR);
		LeftAlignString_MCGS(pTextStart, MCGS_LEN_ITEM_CHAR);
		pItemPageReq->u32ItemNo = u32ItemNo;
		pItemPageReq->bMsgAcqPageWinDown = TRUE;
		ItemDataType = ITEM_UI_TEXT;

	} else if(pItemPageReq->DataPage == DATA_PAGE_CONF) {
		BOOL bBlankLine = FALSE;
			/* 当前显示ItemNo范围检查：超过了当前显示窗口 */
		if((u32ItemNo < pItemPageReq->uConfWinFirstLineItemNo) || (u32ItemNo > pItemPageReq->uConfWinFirstLineItemNo + 5)) {
			u32ItemNo = pItemPageReq->uConfPageTopicItemNo;
		}	/* 当前显示ItemNo范围检查：超过了当前页面最后一行(页面小于窗口),显示为空白行 */
		else if(u32ItemNo > pItemPageReq->uConfPageTopicItemNo + pBootLoaderConf->pConfProp[pItemPageReq->uConfPageTopicItemNo].LkId_Val_ItN) {
			bBlankLine = TRUE;
		}

		/* 打印当前行 */
		if(bBlankLine) {	/* 超出了当前页面，显示为空白行 */
			memset((void*)pTextStart, ' ', MCGS_LEN_ITEM_CHAR);
			ItemDataType = ITEM_UI_TEXT;
		} else {
			pDataAcsProp = pBootLoaderConf->pConfProp + u32ItemNo;
			PrintLocalConfItem(DATA_USER_MCGS, pDataAcsProp, pTextStart, MCGS_LEN_ITEM_CHAR);
			ItemDataType = GetItemUIType(pDataAcsProp);
		}
		
		/* 下一行准备 */
		if(u32ItemNo == pItemPageReq->uConfPageTopicItemNo) {
			pItemPageReq->u32ItemNo = pItemPageReq->uConfWinFirstLineItemNo;
		} else {
			pItemPageReq->u32ItemNo = u32ItemNo + 1;
		}
	} else {
		InitDataWithZero(pMCGSItemPageAck, MCGS_ITEM_PAGE_LINE_WLEN*2);
	}

	/* 完成传输 */
	pMCGSItemPageAck[MCGS_ITEM_PAGE_ITEMNO_BSTART + 0] = u32ItemNo/0x1000000UL;			/* 调整高低字节顺序 */
	pMCGSItemPageAck[MCGS_ITEM_PAGE_ITEMNO_BSTART + 1] = (u32ItemNo/0x10000UL)%0x100UL;	/* 调整高低字节顺序 */
	pMCGSItemPageAck[MCGS_ITEM_PAGE_ITEMNO_BSTART + 2] = (u32ItemNo/0x100UL)%0x100UL;	/* 调整高低字节顺序 */
	pMCGSItemPageAck[MCGS_ITEM_PAGE_ITEMNO_BSTART + 3] = u32ItemNo%0x100UL;				/* 调整高低字节顺序 */
	pMCGSItemPageAck[MCGS_ITEM_PAGE_TYPE_BSTART + 0] = 0;
	pMCGSItemPageAck[MCGS_ITEM_PAGE_TYPE_BSTART + 1] = ItemDataType;
}

/*==========================================================================
| Description	: 把数据调整成合适MCGS的ItemPage的格式--由于mcgs屏幕软件处理能力有限，因此传输的数据需要进行如下约定:
			1. 汉字需要在一个Modbus/RTU的16bit数据里，而不能分散在两个
			2. 空余的地方插入空格
			3. 由于需要额外的添零空间，因此需要额外的ITEMPAGE_ADDI_BLEN长度内存用于数据整理(数据一开始就是靠后放置的)
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2017-07-22
\=========================================================================*/
void LeftAlignString_MCGS(uint8* pDest, uint32 u32TextBufBLen)
{
	uint8* pSrc;
	int32 i;

	pSrc = pDest + MCGS_ITEM_PAGE_ADDI_BLEN;
	for(i = 0; (i < u32TextBufBLen) && (*pSrc); i++) {
		if((i & 1) && (pDest[i-1] <= 0x7F) && (*pSrc > 0x7F)) {
			pDest[i] = 0;
		} else {
			pDest[i] = *pSrc;
			pSrc++;
		}
	}
	for( ; i < u32TextBufBLen; i++) {
		pDest[i] = ' ';
	}
}
#endif

/*==========================================================================
| Description	: 把实际配置所用的类型(ITEM_DATA_TYPE)转化为UI显示所需要的类型(ITEM_UI_TYPE, 是ITEM_DATA_TYPE的一个子集)
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2020-02-19
\=========================================================================*/
ITEM_DATA_TYPE GetItemUIType(CONF_n_STAT_PROP* pDataAcsProp)
{
	if(pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK) {
		if((pDataAcsProp->pData == NULL) || (*((uint32*)pDataAcsProp->pData) == 0) || g_Ctr.bEngDebug) {
			return ITEM_UI_CLICK;
		} else {
			return ITEM_UI_LOGIN;
		}
	} else {
		return (ITEM_DATA_TYPE)(pDataAcsProp->ItemType.DataType>>4);
	}
}

/*==========================================================================
| Description	: 以下函数为mqtt配置页面接口，核心为 CONF_n_STAT_PROP 格式
分为两种操作：配置、同步
配置页面：用于产生配置页面，方便操作人员阅读，但一次仅访问一个配置页面
配置同步：用于产生同步数据，可读性差，主要用于配置的备份/恢复，但可以一次访问全部配置数据

页面接口函数:
	ProcMqttConfInstr()	: 处理外部配置指令，调用 ProcItemPageInstr()函数处理
	ProcMqttConfSync()	: 处理外部同步指令
	QueryAndPubMqttConfResAndPage()	: 查询并发布Mqtt页面处理指令结果
	
配置接口
	ProcItemPageInstr()	: 处理外部写接口
	PrintMqttConfPage()	: 打印Mqtt配置页面
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2017-12-25
\=========================================================================*/
void ProcMqttConfInstr(uint8* pU8Msg, uint8* pU8MsgEnd)
{
	if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP == 0) {	/* 处理接口空 */
		uint32 u32ItemNo = 0;
		pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 3); 		/* 修正到no的值位置 */
		if(CompareMsg(pU8Msg, "ini")) { 							/* id为ini代表复位整体配置页面 */
			u32ItemNo = 0xFFFFFFFFUL;								/* 0xFFFFFFFFUL代表初始化页面 */
		} else {
			u32ItemNo = ReadU32(&pU8Msg, pU8MsgEnd);				/* 获得ItemNo, 该函数会自动忽略前面的非数字字符 */
		}
		pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 4); 		/* 到指令的参数区 */
		g_MqttItemProc.uPageOpItemNo = u32ItemNo;
		g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP
			= ProcItemPageInstr(DATA_USER_NET, DATA_PAGE_CONF, u32ItemNo, &pU8Msg, pU8MsgEnd, &g_MqttItemProc.ItemPageReq);
		/* 如果结果无需等待,则立即通知pub任务发布结果 */
		if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP <= 0) {
			Semaphore_post(SEM_MqttPubReq);
		}
	}
}

void ProcMqttConfSync(uint8* pU8Msg, uint8* pU8MsgEnd)
{
	if(g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP == 0) {	/* 处理接口空 */
		/* 初始化 */
		g_MqttItemProc.uStartConfID = 0;
		g_MqttItemProc.uEndConfID = 0;

		/* 开始处理 */
		pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 1);		/* 指向第一个字段 */
		if(CompareInstr(&pU8Msg, pU8MsgEnd, "instr\":\"backup")) {	/* 仅请求备份数据的指令 */
			g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = -1;
			Semaphore_post(SEM_MqttPubReq);		//Semaphore_post(SEM_MqttPubReq);
			return;
		} else if((!CompareMsg(pU8Msg, "device_type"))				/* 设备型号检查 */
				|| ((pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 2)) >= pU8MsgEnd)
				|| (!CompareMsg(pU8Msg, DEVICE_TYPE_String)))
		{
			g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
				Semaphore_post(SEM_MqttPubReq);	//Semaphore_post(SEM_MqttPubReq);
			return;
		} else {													/* 处理配置恢复指令 */
			pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '{', 1);	/* 跳过型号、版本、序列号字段 */
			uint16 uConfID = ReadU32(&pU8Msg, pU8MsgEnd);
			g_MqttItemProc.uStartConfID = uConfID;	                /* 为了获取起始ID而如此处理 */
			while(uConfID && (pU8Msg < pU8MsgEnd)) {
				if(pU8Msg[1] != ':') {	/* 消息格式错误 */
					g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
					return;
				}
				
				/* 根据ConfID搜索ItemNo */
				CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
				uint16 uItemNo;
				for(uItemNo = 0; (uItemNo < pBootLoaderConf->u32ItemNum) && (pDataAcsProp->u32ItemID != uConfID); uItemNo++) {
					pDataAcsProp++;
				}

				BOOL bArray = (pU8Msg[2] == '[');
				/* 找到对应的ItemNo，处理指令. 需要排除板上存储的部分 */
				if((uItemNo < pBootLoaderConf->u32ItemNum) && (pDataAcsProp->ItemType.SaveGrp != SAVE_GRP_BRD)) {
					pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, '"', 2);
					ProcItemPageInstr(DATA_USER_NET, DATA_PAGE_CONF, uItemNo, &pU8Msg, pU8MsgEnd, NULL);
				} else if(bArray) {
					pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, ']', 1);
				} else {
					pU8Msg = SkipCharInString(pU8Msg, pU8MsgEnd, ',', 1);
				}

				/* 为下一次准备 */
				g_MqttItemProc.uEndConfID = uConfID;
				uConfID = ReadU32(&pU8Msg, pU8MsgEnd);
			}
			g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = 5;
			Semaphore_post(g_DataAccessBlock.Sem_Op);		//Semaphore_post(SEM_NvMemAcs);
		}
	}
}

/*==========================================================================
| Description	: 发布MqttConf写结果以及MqttConf页面，包括配置页面操作、配置同步操作
				这个写成一个独立的函数，是为了让MqttPubTask()显得简洁
| G/Out var		:
| Author		: King			Date	: 2017-12-28
\=========================================================================*/
/* 打印当前配置页面
    {"no":"21", "title":"通讯1串口配置", "cont":{
        {"no":"22", "prompt":"软件版本号", "text":"2.2.55"},
        {"no":"23", "prompt":"互感器变比", "set":["200"]}}}    */
void PrintMqttConfPage(uint8** ppTxBuf, uint8* pTxBufEnd);
BOOL PrintMqttConfItemValue(uint8** ppTxBuf, uint8* pTxBufEnd, CONF_n_STAT_PROP* pDataAcsProp, uint32 u32ItemData);
BOOL QueryAndPubMqttConfResAndPage(MQTT_COMM* pMqttComm)
{
	/* 配置页面操作：检查结果 */
    if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP > 1) {      /* 等待MdlDataAccess保存结果,>1是为避免落到0,让pub任务误会已经响应完了上一个请求 */
        g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP--;
    } else if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP == 1) {
        g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;      /* 等完了tryCnt,也没能等来MdlDataAccess传来的完成信号,判定为失败 */
    }
	
	/* 配置页面操作：发布结果 */
	if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP < 0) {
		/* 发布配置页面 */
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
		uint8 u8QoS = 0;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
		/* 配置页面发布:打印Topic */
		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
		PrintStringNoOvChk(&pTxBuf, "/PAGE/CONF");
		uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
		}
		PrintMqttConfPage(&pTxBuf, &pMqttComm->u8TRBuf[MQTT_TR_BUF_BLEN]);
		/* 配置页面发布:传输 */
		if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {	/* QoS修改必须在本段代码开始的位置 */
			return FALSE;
		}

		/* 发布写配置结果 */
		pTxBuf = pMqttComm->u8TRBuf + 5; /* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
		u8QoS = 0;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
		/* 写配置结果发布:打印Topic */
		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
		PrintStringNoOvChk(&pTxBuf, "/RES/CONF");
		uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
		}
		*pTxBuf++ = '{';		/* JSon开始 */
		if(g_MqttItemProc.uPageOpItemNo == 0xFFFF) {
			PrintStringToJson(&pTxBuf, "no", "ini");	/* no */
		} else {
			PrintU32DatToJson(&pTxBuf, "no", g_MqttItemProc.uPageOpItemNo, 0);	/* no */
		}
		/* 打印结果 */
		if(g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP == -1) {
			PrintStringToJson(&pTxBuf, "res", "success");
		} else {
			PrintStringToJson(&pTxBuf, "res", "fail");
		}
		pTxBuf--;
		*pTxBuf++ = '}';	/* Json结尾 */
		/* 写配置页面结果发布:传输 */
		if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) { 	/* QoS修改必须在本段代码开始的位置 */
			return FALSE;
		}
		g_MqttItemProc.i8PageOpST_Idle0_SucN1_FailN2_MaxWaitCntP = 0; 			/* 代表已经处理结束 */
	}

	/* 配置同步操作：检查结果 */
    if(g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP > 1) {      /* 等待MdlDataAccess保存结果,>1是为避免落到0,让pub任务误会已经响应完了上一个请求 */
        g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP--;
    } else if(g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP == 1) {
        g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;      /* 等完了tryCnt,也没能等来MdlDataAccess传来的完成信号,判定为失败 */
    }

	/* 配置同步操作：发布结果 */
	if(g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP < 0) {
		/* 发布配置页面 */
		CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
		uint16 uItemNum = pBootLoaderConf->u32ItemNum;
		int8 u8PageNum = 0;
		while(uItemNum) {
			uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
			uint8 u8QoS = 1;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
			/* 写配置结果发布:打印Topic */
			PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
			PrintStringNoOvChk(&pTxBuf, "/PAGE/SYNC");
			uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
			if(u8QoS) {
				pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
			}
			*pTxBuf++ = '{';		/* JSon开始 */
			PrintStringToJson(&pTxBuf, "device_type", DEVICE_TYPE_String);
			PrintSoftVerToJson(&pTxBuf, "soft_ver", SOFTWARE_VER);
			PrintU32DatToJson(&pTxBuf, "sn", g_Sys.SerialNo.u32Dat, 0);
			PrintU32DatToJson(&pTxBuf, "page_no", ++u8PageNum, 0);
			PrintStringNoOvChk(&pTxBuf, "\"conf\":{");
			while(uItemNum) {
				if((pDataAcsProp->pData !=	NULL) && (pDataAcsProp->ItemType.SaveGrp != SAVE_GRP_NUL)) {
					if(&pMqttComm->u8TRBuf[MQTT_TR_BUF_BLEN] - pTxBuf < 100) {	/* 确保了后续仍然有数据 */
						break;
					} else {	/* 单个项目最长   44byte-- "5b":["32b"],  */
						uint8* pTxBufBak = pTxBuf;
						*pTxBuf++ = '"';
						PrintU32WithLen(&pTxBuf, pDataAcsProp->u32ItemID, 4);
						*pTxBuf++ = '"';
						if(PrintMqttConfItemValue(&pTxBuf, &pMqttComm->u8TRBuf[MQTT_TR_BUF_BLEN], pDataAcsProp, *((uint32*)pDataAcsProp->pData))) {
							*pTxBuf++ = ',';
						} else {
							pTxBuf = pTxBufBak;
						}
					}
				}
				pDataAcsProp++;
				uItemNum--;
			}
			pTxBuf--;
			*pTxBuf++ = '}';	/* 与 "\"conf\":{" 配对 */
			*pTxBuf++ = ',';
			PrintBoolToJson(&pTxBuf, "to_be_continued", uItemNum);	/* 是否还有一页 */
			pTxBuf--;
			*pTxBuf++ = '}';	/* Json结尾 */
			if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) {		/* QoS修改必须在本段代码开始的位置 */
				return FALSE;
			}
		}
		
		/* 发布配置同步结果 */
		uint8* pTxBuf = pMqttComm->u8TRBuf + 5; /* 固定头部预留，1Byte类型+2Byte总长度(最大长度16383)+2B Topic长度 */
		uint8 u8QoS = 0;			/* 修改本次发布的QoS必须在这里进行，否则本段代码可能运行出错 */
		/* 写配置结果发布:打印Topic */
		PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
		PrintStringNoOvChk(&pTxBuf, "/RES/SYNC");
		uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
		if(u8QoS) {
			pTxBuf += 2;	/* 根据发布的消息QoS，预留MsgID部分, QoS=0无需，但保留该行代码以保持一致性 */
		}
		*pTxBuf++ = '{';		/* JSon开始 */
		/* 打印结果 */
		if(g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP == -1) {
			PrintStringToJson(&pTxBuf, "res", "success");
		} else {
			PrintStringToJson(&pTxBuf, "res", "fail");
		}
		PrintU32DatToJson(&pTxBuf, "start_load_id", g_MqttItemProc.uStartConfID, 0);
		PrintU32DatToJson(&pTxBuf, "end_load_id", g_MqttItemProc.uEndConfID, 0);
		PrintU32DatToJson(&pTxBuf, "conf_page_num", u8PageNum, 0);
		pTxBuf--;
		*pTxBuf++ = '}';	/* Json结尾 */
		/* 写配置页面结果发布:传输 */
		if(!PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS)) { 	/* QoS修改必须在本段代码开始的位置 */
			return FALSE;
		}
		g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = 0; 	/* 代表已经处理结束 */
	}

	return TRUE;
}

/* 打印当前配置页面
	{"no":"21", "title":"通讯1串口配置", "cont":{
		{"no":"22", "prompt":"软件版本号", "text":"2.2.55"},
		{"no":"23", "prompt":"互感器变比", "set":["200"]}}}    */
void PrintAcqU32F32Conf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp);
void PrintTempConf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp);
void PrintAcqSummConf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp);
void PrintSysFuncRes(uint8** ppTxBuf, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp, LANG_TYPE Language);
void PrintMqttConfPage(uint8** ppTxBuf, uint8* pTxBufEnd)
{
	uint8* pTxBuf = *ppTxBuf;
	uint16 uItemNo = g_MqttItemProc.ItemPageReq.uConfPageTopicItemNo;
	LANG_TYPE Language = g_MqttItemProc.ItemPageReq.Language;			/* 锁存，避免在后面的运行中被更改 */

	CONF_n_STAT_PROP* pDataAcsProp = &pBootLoaderConf->pConfProp[uItemNo];
	if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TOPIC) {			/* 输入检查: 判断是不是页面Topic */
		/* 打印Topic行 */
		*pTxBuf++ = '{';
		PrintU32DatToJson(&pTxBuf, "no", uItemNo, 0);	/* id */
		PrintStringToJson(&pTxBuf, "title", pDataAcsProp->pName[Language]);		/* title名称 */
		PrintStringNoOvChk(&pTxBuf, "\"cont\":[");

		/* 页面数据读取 */
		uint32 u32PageData = 0;
		if(pDataAcsProp->pData != NULL) {
			u32PageData = *((uint32*)pDataAcsProp->pData);
		}

		/* 打印页面其余行 */
		uint32 u32ItemData = 0;		/* 用来处理U32类数据 */
		uint16 uBitStart = 0;
		int16 i;
		for(i = pDataAcsProp->LkId_Val_ItN; i > 0; i--) {
			uItemNo++;
			pDataAcsProp++;

			*pTxBuf++ = '{';
			PrintU32DatToJson(&pTxBuf, "no", uItemNo, 0);			/* 打印当前行id */
			if(pDataAcsProp->pData != NULL) {
				u32ItemData = *((uint32*)pDataAcsProp->pData);
			} else if(pDataAcsProp->ItemType.i8SgnDigits_u8BitNum	/* 确定为位变量 */
					&& ((pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL)
						|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM)
						|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32)
						|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_RO)))
			{
				uint16 uBitEnd = uBitStart + pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
				u32ItemData = (u32PageData & ((1<<uBitEnd) - 1))>>uBitStart;
				uBitStart = uBitEnd;
			} else {												/* 预留，免得未被赋值 */
				u32ItemData = pDataAcsProp->LkId_Val_ItN;
			}
			
			if((pDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_U32) || (pDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_F32)) {
				PrintAcqU32F32Conf(DATA_USER_NET, Language, &pTxBuf, pTxBufEnd, pDataAcsProp);
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TEMPSENCONF) {
				PrintTempConf(DATA_USER_NET, Language, &pTxBuf, pTxBufEnd, pDataAcsProp);
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_SUMM) {
				PrintAcqSummConf(DATA_USER_NET, Language, &pTxBuf, pTxBufEnd, pDataAcsProp);
			} else {
				PrintStringToJson(&pTxBuf, "prompt", pDataAcsProp->pName[Language]);	/* 提示符 */
			}
		
			/* 值部分 */
			if(pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK) {
				if(1 || (pDataAcsProp->pData == NULL) || (*((uint32*)pDataAcsProp->pData) == 0)) {	/* 2021.7.2 目前网络配置我们内部使用，忽视密码 */
					PrintStringToJson(&pTxBuf, "jump", "");
				} else {
					PrintStringToJson(&pTxBuf, "jump", "***");
				}
				pTxBuf--;
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_S32) {
				PrintU32DatToJson(&pTxBuf, "jump", *((uint32*)pDataAcsProp->pData), 0);
				pTxBuf--;
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL) {
				PrintBoolToJson(&pTxBuf, "box", u32ItemData);
				pTxBuf--;
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM) {
				PrintU32DatToJson(&pTxBuf, "radio", u32ItemData, 0);	/* 枚举值部分 */
				/* 枚举项目部分 */
				PrintStringNoOvChk(&pTxBuf, "\"cont\":[");
				pDataAcsProp++;
				uint16 uCount = 0;
				for(uCount = 0; pDataAcsProp->ItemType.DataType == ITEM_DAT_ECONT; uCount++) {
					*pTxBuf++ = '{';
					PrintEnumContToJson(&pTxBuf, pDataAcsProp->pName[Language], pDataAcsProp->LkId_Val_ItN, 0);
					pTxBuf--;
					*pTxBuf++ = '}';
					*pTxBuf++ = ',';
					pDataAcsProp++;
				}
				i -= uCount;
				uItemNo += uCount;
				pDataAcsProp--;
				pTxBuf--;
				*pTxBuf++ = ']';
			} else {
				if((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TEXT) {
					PrintStringNoOvChk(&pTxBuf, "\"text\"");
				} else {
					PrintStringNoOvChk(&pTxBuf, "\"set\"");
				}
				PrintMqttConfItemValue(&pTxBuf, pTxBufEnd, pDataAcsProp, u32ItemData);
			}
			*pTxBuf++ = '}';
			*pTxBuf++ = ',';
		}
		
		/* 打印结尾部分 */
		pTxBuf--;
		*pTxBuf++ = ']';
		*pTxBuf++ = '}';
	}

	*ppTxBuf = pTxBuf;
}

BOOL PrintMqttConfItemValue(uint8** ppTxBuf, uint8* pTxBufEnd, CONF_n_STAT_PROP* pDataAcsProp, uint32 u32ItemData)
{
	int64 i64Data;

	/* 预先准备好浮点数的有效位数，非浮点数不需要 */
	int8 i8SgnDigits = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
	if(i8SgnDigits == 0) {
		i8SgnDigits = 4;	/* 默认使用4位有效数字 */
	}

	uint8* pTxBuf = *ppTxBuf;
	*pTxBuf++ = ':';
    if((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TWOBOX) {
        *pTxBuf++ = '[';
    }
	*pTxBuf++ = '"';

	switch(pDataAcsProp->ItemType.DataType) {
		/* PrintMqttConfPage()函数不含这三个，这个是SYNC操作时候使用 */
		case ITEM_DAT_TOPIC:
		case ITEM_DAT_BOOL:
		case ITEM_DAT_ENUM:
			PrintU32(&pTxBuf, pTxBufEnd, u32ItemData, 0);
			break;

        case ITEM_DAT_FUNC:     /* 数据操作的执行输出 */
            PrintSysFuncRes(&pTxBuf, pTxBufEnd, pDataAcsProp, g_MqttItemProc.ItemPageReq.Language);
            break;
            
        case ITEM_DAT_U32_RO:
        case ITEM_DAT_U32_ID_RO:
		case ITEM_DAT_U32:
		case ITEM_DAT_ACQ_U32:
		case ITEM_DAT_TEMPSENCONF:
			PrintU32(&pTxBuf, pTxBufEnd, u32ItemData, pDataAcsProp->ItemType.uBcdQ);
			break;

		case ITEM_DAT_IPv4:
			PrintIPv4(&pTxBuf, pTxBufEnd, *((uint32*)pDataAcsProp->pData));
			break;

	#if SUPPORT_IEC101 || SUPPORT_IEC104
		case ITEM_DAT_TELE_ITEM:
			PrintTeleSigMetConf(&pTxBuf, pTxBufEnd, (TELE_101_104_ITEM_CONF*)pDataAcsProp->pData);
			break;
	#endif

		case ITEM_DAT_F32:
		case ITEM_DAT_ACQ_F32:
			PrintF32(&pTxBuf, pTxBufEnd, *((float32*)pDataAcsProp->pData), i8SgnDigits);
			break;

		case ITEM_DAT_I64:
        case ITEM_DAT_I64_RO:
			Swi_disable();
			i64Data = (*((int64*)pDataAcsProp->pData));
			Swi_enable();
			PrintI64(&pTxBuf, pTxBufEnd, i64Data, pDataAcsProp->ItemType.uBcdQ);
			break;

		case ITEM_DAT_D2U32:
			PrintU32(&pTxBuf, pTxBufEnd, *((uint32*)pDataAcsProp->pData), pDataAcsProp->ItemType.uBcdQ);
        	*pTxBuf++ = '"';
			*pTxBuf++ = ',';
        	*pTxBuf++ = '"';
			PrintU32(&pTxBuf, pTxBufEnd, *(((uint32*)pDataAcsProp->pData) + 1), pDataAcsProp->ItemType.uBcdQ);
			break;

		case ITEM_DAT_D2F32:
			PrintF32(&pTxBuf, pTxBufEnd, *((float32*)pDataAcsProp->pData), i8SgnDigits);
        	*pTxBuf++ = '"';
			*pTxBuf++ = ',';
        	*pTxBuf++ = '"';
			PrintF32(&pTxBuf, pTxBufEnd, *(((float32*)pDataAcsProp->pData) + 1), i8SgnDigits);
			break;

		case ITEM_DAT_SUMM:
			PrintI32(&pTxBuf, pTxBufEnd, *((int32*)pDataAcsProp->pData +0), 0);
        	*pTxBuf++ = '"';
			*pTxBuf++ = ',';
        	*pTxBuf++ = '"';
			PrintF32(&pTxBuf, pTxBufEnd, *((float32*)pDataAcsProp->pData + 1), i8SgnDigits);
			break;

		case ITEM_DAT_pTIME:
			PrintPriceTime(&pTxBuf, pTxBufEnd, *((uint32*)pDataAcsProp->pData + 0));
        	*pTxBuf++ = '"';
			*pTxBuf++ = ',';
        	*pTxBuf++ = '"';
			PrintPriceTime(&pTxBuf, pTxBufEnd, *((uint32*)pDataAcsProp->pData + 1));
			break;

		case ITEM_DAT_RMTS:
		case ITEM_DAT_DIGITSTR:
		case ITEM_DAT_ANSI:
        case ITEM_DAT_ANSI_RO:
			PrintString(&pTxBuf, pTxBufEnd, (char*)(pDataAcsProp->pData));
			break;

		case ITEM_DAT_TXT:	/* 必须要这个，要不然就会进入default了 */
			break;

		case ITEM_DAT_SOFTVER:
			PrintSoftVer(&pTxBuf, pTxBufEnd, pDataAcsProp->LkId_Val_ItN);
			break;

		case ITEM_DAT_T64_RO:
			Swi_disable();
			PrintT64(DATA_USER_NET, &pTxBuf, pTxBufEnd, (REAL_TIME_VAR*)pDataAcsProp->pData);
			Swi_enable();
			break;

		case ITEM_DAT_T32_RO:
			PrintT32(DATA_USER_NET, &pTxBuf, pTxBufEnd, *(uint32*)pDataAcsProp->pData);
			break;

		default:
			return FALSE;
	}
	
	*pTxBuf++ = '"';
	if((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TWOBOX) {
        *pTxBuf++ = ']';
    }
	*ppTxBuf = pTxBuf;

	return TRUE;
}

/*==========================================================================
	以下是公共的工具函数
\=========================================================================*/
/*==========================================================================
| Description	: 用于把CONF_n_STAT_PROP格式的conf(stat)项目打印成文本, 				主要用于本地MCGS, ViewTech屏幕，文件存储
		F32最大10byte, U32最大10byte, IPV4 15byte, U64最大20byte, T64带毫秒21byte, mqtt订阅的TOPIC是18byte
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2018-07-31
\=========================================================================*/
uint8* PrintLocalConfItem(DATA_USER DataUser, CONF_n_STAT_PROP* pDataAcsProp, uint8* pBuf, uint16 uTextByteLen)
{
	CONF_n_STAT_PROP* pDataAcsPropBak = pDataAcsProp;
	uint8* pBufStart = pBuf;
	int16 i;
	
	uint8* pDataSegEnd = pBufStart + uTextByteLen;				/* 数据段结尾, 先指向BufEnd，再根据不同的类型扣除 */
	uint8* pPromptSegEnd = pDataSegEnd;							/* 提示符段结尾, 先指向BufEnd，再根据不同的类型扣除 */
	if(DataUser == DATA_USER_FILE) {
		PrintU32WithLen(&pBuf, pDataAcsProp->u32ItemID, 4);		/* 形成4位格式 */
		*pBuf++ = ' ';
		*pBuf++ = ' ';	/* 为了兼容 配置有提示符 的版本 */
#if SUPPORT_SCREEN
	} else if(DataUser == DATA_USER_MCGS) {	/* mcgs屏幕Buf要比传输长度多出MCGS_ITEM_PAGE_ADDIPAYLOAD_BSTART以便调整 */
		pBuf += MCGS_ITEM_PAGE_ADDI_BLEN;
		pPromptSegEnd += MCGS_ITEM_PAGE_ADDI_BLEN;
#endif
	}

	/* 显示位置调整 */
	if((pDataAcsProp->ItemType.DataType == ITEM_DAT_TXT)
		|| ((pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM) && (DataUser != DATA_USER_FILE)))
	{
	} else if((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TWOBOX) {
		pPromptSegEnd -= 22;
		pDataSegEnd -= 11;
	} else if((DataUser == DATA_USER_FILE)
			|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ANSI_RO)			
			|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_T64_RO)
			|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_T32_RO)
			|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_I64_RO)
			|| ((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_ONEDBOX)
			|| ((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_ANSI))
	{
		pPromptSegEnd -= 22;
	} else {
		pPromptSegEnd -= 11;
	}

	/* 提示符部分 */
	if(g_Sys.u32LocalLang >= MAX_LANG_TYPE) {
		g_Sys.u32LocalLang = CHINESE;
	}
	if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TEMPSENCONF) {
		PrintTempConf(DataUser, (LANG_TYPE)g_Sys.u32LocalLang, &pBuf, pPromptSegEnd, pDataAcsProp);
	} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_SUMM) {
		PrintAcqSummConf(DataUser, (LANG_TYPE)g_Sys.u32LocalLang, &pBuf, pPromptSegEnd, pDataAcsProp);
	} else if(DataUser == DATA_USER_FILE) {		/* 文件存储配置无需提示符，以减少存储量 */
	} else if((pDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_U32) || (pDataAcsProp->ItemType.DataType == ITEM_DAT_ACQ_F32)) {
		PrintAcqU32F32Conf(DataUser, (LANG_TYPE)g_Sys.u32LocalLang, &pBuf, pPromptSegEnd, pDataAcsProp);
	} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TOPIC) {
		PrintTopicString(&pBuf, uTextByteLen, pDataAcsProp->pName[g_Sys.u32LocalLang]); /* 屏幕字体总共占据30byte宽度 */
	} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ECONT) {
		*pBuf++ = ' ';
		*pBuf++ = ' ';
		PrintString(&pBuf, pPromptSegEnd, pDataAcsProp->pName[g_Sys.u32LocalLang]);
	} else {
		PrintString(&pBuf, pPromptSegEnd, pDataAcsProp->pName[g_Sys.u32LocalLang]);
	}
	if(0) {
#if SUPPORT_SCREEN
	} else if(DataUser == DATA_USER_MCGS) {
		*pBuf = 0;
		pPromptSegEnd -= MCGS_ITEM_PAGE_ADDI_BLEN;			/* 修正回去 */
		LeftAlignString_MCGS(pBufStart, pPromptSegEnd - pBufStart);	/* MCGS需要把汉字放在一个Word里面 */
		pBuf = pPromptSegEnd;
#endif
	} else if(DataUser == DATA_USER_FILE) {
	} else {						/* 填满当前段 */
		for(i = pPromptSegEnd - pBuf; i > 0; i--) {
			*pBuf++ = ' ';
		}
	}
	
	/* 如果当前变量是ECONT, 需要把当前指针移到ENUM，这才是真正的变量位置 */
	uint32 u32ItemData;
	if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ECONT) {
		do {
			pDataAcsProp--;
		} while(pDataAcsProp->ItemType.DataType != ITEM_DAT_ENUM);
	}
	if(pDataAcsProp->pData != NULL) {
		u32ItemData = *((uint32*)pDataAcsProp->pData);
	/* FILE无需打印单个位变量 */
	} else if((DataUser == DATA_USER_FILE) || (pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK)) {
		u32ItemData = 0;
	} else {									/* 计算位变量 */
		uint16 uVarBitNum = 0;					/* 当前变量位宽 */
		uint16 uBitStart = 0;					/* 当前变量起始位 */
		/* 搜索直至当前页面Topic */
		while(pDataAcsProp->ItemType.DataType != ITEM_DAT_TOPIC) {
			/* 确定为位变量 */
			if((pDataAcsProp->pData == NULL) 
				&& pDataAcsProp->ItemType.i8SgnDigits_u8BitNum
				&& ((pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_RO)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_ID_RO)))
			{
				if(uVarBitNum == 0) {
					uVarBitNum = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
				} else {
					uBitStart += pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
				}
			}
			pDataAcsProp--;
		}
		if(pDataAcsProp->pData == NULL) {
			u32ItemData = 0;
		} else {
			u32ItemData = ((*((uint32*)pDataAcsProp->pData))>>uBitStart) & ((1<<uVarBitNum) - 1);
		}
	}
	pDataAcsProp = pDataAcsPropBak;

	/* 预先准备好浮点数的有效位数，非浮点数不需要 */
	int8 i8SgnDigits = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
	if(i8SgnDigits == 0) {
		i8SgnDigits = 4;	/* 默认使用4位有效数字 */
	}
	
	/* 如果有数据项，则打印数据项 */
	int64 i64Data;
	switch(pDataAcsProp->ItemType.DataType) {
		case ITEM_DAT_LINK:
			if(u32ItemData && (!g_Ctr.bEngDebug)) {
				PrintString(&pBuf, pDataSegEnd, "  ******");
			}
			break;
			
        case ITEM_DAT_FUNC:     /* 数据操作的执行输出 */
            PrintSysFuncRes(&pBuf, pDataSegEnd, pDataAcsProp, (LANG_TYPE)g_Sys.u32LocalLang);
            break;

		case ITEM_DAT_TOPIC:	/* TOPIC有数据才打印 */
			if(pDataAcsProp->pData == NULL) {
				break;
			}
		case ITEM_DAT_ENUM:		/* 如果是DATA_USER_UI，pDataSegEnd已经修改到指向pBufEnd */
		case ITEM_DAT_S32:
		case ITEM_DAT_U32:
		case ITEM_DAT_U32_ID_RO:
		case ITEM_DAT_ACQ_U32:
		case ITEM_DAT_U32_RO:
		case ITEM_DAT_TEMPSENCONF:
			PrintU32(&pBuf, pDataSegEnd, u32ItemData, pDataAcsProp->ItemType.uBcdQ);
			break;

		case ITEM_DAT_IPv4:
			PrintIPv4(&pBuf, pDataSegEnd, *((uint32*)pDataAcsProp->pData));
			break;

	#if SUPPORT_IEC101 || SUPPORT_IEC104
		case ITEM_DAT_TELE_ITEM:
			PrintTeleSigMetConf(&pBuf, pDataSegEnd, (TELE_101_104_ITEM_CONF*)pDataAcsProp->pData);
			break;
	#endif
	
		case ITEM_DAT_F32:
		case ITEM_DAT_ACQ_F32:
			PrintF32(&pBuf, pDataSegEnd, *((float32*)pDataAcsProp->pData), i8SgnDigits);		/* 使用4位有效数字 */
			break;
	
		case ITEM_DAT_I64:
        case ITEM_DAT_I64_RO:
			Swi_disable();
			i64Data = *((int64*)pDataAcsProp->pData);
			Swi_enable();
			PrintI64(&pBuf, pDataSegEnd, i64Data, pDataAcsProp->ItemType.uBcdQ);
			break;
	
		case ITEM_DAT_D2U32:
			PrintU32(&pBuf, pDataSegEnd, *((uint32*)pDataAcsProp->pData), pDataAcsProp->ItemType.uBcdQ);
			if(DataUser == DATA_USER_FILE) {
				*pBuf++ = ' ';
			} else {
				for(i = pDataSegEnd - pBuf; i > 0; i--) {
					*pBuf++ = ' ';
				}
			}
			pDataSegEnd += 11;
			PrintU32(&pBuf, pDataSegEnd, *(((uint32*)pDataAcsProp->pData) + 1), pDataAcsProp->ItemType.uBcdQ);
			break;
			
		case ITEM_DAT_D2F32:
			PrintF32(&pBuf, pDataSegEnd, *((float32*)pDataAcsProp->pData), i8SgnDigits);		/* 使用4位有效数字 */
			if(DataUser == DATA_USER_FILE) {
				*pBuf++ = ' ';
			} else {
				for(i = pDataSegEnd - pBuf; i > 0; i--) {
					*pBuf++ = ' ';
				}
			}
			pDataSegEnd += 11;
			PrintF32(&pBuf, pDataSegEnd, *(((float32*)pDataAcsProp->pData) + 1), i8SgnDigits);/* 使用4位有效数字 */
			break;
			
		case ITEM_DAT_SUMM:
			PrintI32(&pBuf, pDataSegEnd, *((int32*)pDataAcsProp->pData + 0), 0);
			if(DataUser == DATA_USER_FILE) {
				*pBuf++ = ' ';
			} else {
				for(i = pDataSegEnd - pBuf; i > 0; i--) {
					*pBuf++ = ' ';
				}
			}
			pDataSegEnd += 11;
			PrintF32(&pBuf, pDataSegEnd, *((float32*)pDataAcsProp->pData + 1), i8SgnDigits);	/* 使用4位有效数字 */
			break;
		
		case ITEM_DAT_pTIME:
			PrintPriceTime(&pBuf, pDataSegEnd, *((uint32*)pDataAcsProp->pData + 0));
			if(DataUser == DATA_USER_FILE) {
				*pBuf++ = ' ';
			} else {
				for(i = pDataSegEnd - pBuf; i > 0; i--) {
					*pBuf++ = ' ';
				}
			}
			pDataSegEnd += 11;
			PrintPriceTime(&pBuf, pDataSegEnd, *((uint32*)pDataAcsProp->pData + 1));
			break;
			
		case ITEM_DAT_RMTS:
		case ITEM_DAT_DIGITSTR:
		case ITEM_DAT_ANSI:
		case ITEM_DAT_ANSI_RO:
			Swi_disable();
			PrintString(&pBuf, pDataSegEnd, (char*)(pDataAcsProp->pData));
			Swi_enable();
			break;

		case ITEM_DAT_ECONT:
			if(pDataAcsProp->LkId_Val_ItN == u32ItemData) {
				PrintString(&pBuf, pDataSegEnd, " ●");
			} else {
				PrintString(&pBuf, pDataSegEnd, " ○");
			}
			break;
			
		case ITEM_DAT_BOOL:
			if(DataUser == DATA_USER_FILE) {
				PrintU32(&pBuf, pDataSegEnd, u32ItemData, 0);
			} else if(u32ItemData) {
				PrintString(&pBuf, pDataSegEnd, " ■");
			} else {
				PrintString(&pBuf, pDataSegEnd, " □");
			}
			break;

		case ITEM_DAT_T64_RO:
			Swi_disable();
			PrintT64(DataUser, &pBuf, pDataSegEnd, (REAL_TIME_VAR*)pDataAcsProp->pData);
			Swi_enable();
			break;

		case ITEM_DAT_T32_RO:
			PrintT32(DataUser, &pBuf, pDataSegEnd, *(uint32*)pDataAcsProp->pData);
			break;
			
		case ITEM_DAT_SOFTVER:
			PrintSoftVer(&pBuf, pDataSegEnd, pDataAcsProp->LkId_Val_ItN);
			break;
		
		default:
			break;
	}

	/* 添加结尾 */
	if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH)) {
		for(i = pDataSegEnd - pBuf; i > 0; i--) {
			*pBuf++ = ' ';
		}
	} else if(DataUser == DATA_USER_FILE) {	/* FILE其实不需要考虑溢出 */
		*pBuf++ = 0x0D;
		*pBuf++ = 0x0A;
 	} else if(pBuf < pDataSegEnd) {
		*pBuf++ = 0;
	}

	/* 返回 */
	return pBuf;
}

/*==========================================================================
| Description	: 把具有自动采集功能的配置转化为ItemPage所需要的文本
	显示在屏幕上: 导前时间(s)当前:0.1; 设定:
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2019-5-15
\=========================================================================*/
void PrintAcqU32F32Conf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp)
{
	if(DataUser == DATA_USER_NET) {
		PrintStringNoOvChk(ppText, "\"prompt\":\"");
	}

	PrintString(ppText, pTextEnd, pDataAcsProp->pName[Language]);
	
	if((DataUser == DATA_USER_MCGS) || (DataUser == DATA_USER_VIEWTECH) ||  (DataUser == DATA_USER_NET)) {
		if(Language == CHINESE) {
			PrintString(ppText, pTextEnd, " 测量:");
		} else if(Language == ENGLISH) {
			PrintString(ppText, pTextEnd, " Cali:");
		}
		/* 计算浮点数的有效位数 */
		int8 i8SgnDigits = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
		if(i8SgnDigits == 0) {
			i8SgnDigits = 4;	/* 默认使用4位有效数字 */
		}
		PrintF32(ppText, pTextEnd, *((float32*)(pDataAcsProp->pData) + 1), i8SgnDigits);
		
		if(DataUser == DATA_USER_NET) {
			*(*ppText)++ = '"';
			*(*ppText)++ = ',';
		}
	}
}

/*==========================================================================
| Description	: 把温度传感器配置打印为文本
显示在屏幕上: 01#温度传感器当前温度:127.8 连接失败计数:999999999 放置位置:1 							    	//其中"01#温度传感器"是PropText内容
存储在文件中: 01#温度传感器SN:0123456789ABCDEF 连接失败计数:999999999 放置位置:1							//其中"01#温度传感器"是PropText内容
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-06-20
\=========================================================================*/
void PrintTempConf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp)
{
#if MAX_TEMP_SEN_NUM
	if(DataUser == DATA_USER_NET) {
		PrintStringNoOvChk(ppText, "\"prompt\":\"");
	}

	if(DataUser == DATA_USER_FILE) {
		Swi_disable();
		uint64 u64TempSenRom = ((TEMP_SEN_CONF*)(pDataAcsProp->pData))->u64TempSenRom;
		Swi_enable();
		PrintH64(ppText, pTextEnd, u64TempSenRom);
		*(*ppText)++ = ' ';
		PrintU32(ppText, pTextEnd, ((TEMP_SEN_CONF*)(pDataAcsProp->pData))->u32Count_TempSen_Change, 0);
		*(*ppText)++ = ' ';
	} else {
		char* pPropText = pDataAcsProp->pName[Language];
		PrintString(ppText, pTextEnd, pPropText);
		if(Language == CHINESE) {
			PrintString(ppText, pTextEnd, "当前温度:");
		} else if(Language == ENGLISH) {
			PrintString(ppText, pTextEnd, "Temp(c):");
		}
		/* 传感器温度 */
		uint8 u8TempSenNo = (*pPropText - '0')*10 + (*(pPropText+1) - '0');
		u8TempSenNo--;
		if(u8TempSenNo < MAX_TEMP_SEN_NUM) {
			PrintI16(ppText, pTextEnd, g_AnaRes.Temperature[u8TempSenNo].iDegree_100mC, 1);
		} else {
			*(*ppText)++ = '0';
		}

		if(Language == CHINESE) {
			PrintString(ppText, pTextEnd, " 断线计数:");
			PrintU32(ppText, pTextEnd, ((TEMP_SEN_CONF*)(pDataAcsProp->pData))->u32Count_TempSen_Change, 0);
			PrintString(ppText, pTextEnd, " 放置位置:");
		} else if(Language == ENGLISH) {
			PrintString(ppText, pTextEnd, " Count:");
			PrintU32(ppText, pTextEnd, ((TEMP_SEN_CONF*)(pDataAcsProp->pData))->u32Count_TempSen_Change, 0);
			PrintString(ppText, pTextEnd, " Placement:");
		}

		if(DataUser == DATA_USER_NET) {
			*(*ppText)++ = '"';
			*(*ppText)++ = ',';
		}
	}
#endif
}

/*==========================================================================
| Description	: 把具有自动采集、综合功能的配置转化为ItemPage所需要的文本
	显示在屏幕上: 水工参数K1当前:0.1综合:0.0987; (组数:初始值):
| In/Out/G var	: 
| Author		: Wang Renfei			Date	: 2016-10-19
\=========================================================================*/
void PrintAcqSummConf(DATA_USER DataUser, LANG_TYPE Language, uint8** ppText, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp)
{
	int8 i8SgnDigits = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
	if(i8SgnDigits == 0) {
		i8SgnDigits = 4;	/* 默认使用4位有效数字 */
	}

	if(DataUser == DATA_USER_FILE) {
		PrintF32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->fAcqSummary, i8SgnDigits);
		*(*ppText)++ = ' ';
		PrintU32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->u32SummaryRealNum, 0);
		*(*ppText)++ = ' ';
	} else {
		if(DataUser == DATA_USER_NET) {
			PrintStringNoOvChk(ppText, "\"prompt\":\"");
		}
		
		PrintString(ppText, pTextEnd, pDataAcsProp->pName[Language]);
		if(Language == CHINESE) {
			PrintString(ppText, pTextEnd, "测");
		} else if(Language == ENGLISH) {
			PrintString(ppText, pTextEnd, "Now");
		}
		PrintF32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->fCurAcq, i8SgnDigits);
	
		if(Language == CHINESE) {
			PrintString(ppText, pTextEnd, "综");
			PrintF32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->fAcqSummary, i8SgnDigits);
			PrintString(ppText, pTextEnd, "实");
			PrintU32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->u32SummaryRealNum, 0);
			PrintString(ppText, pTextEnd, ";组初");
		} else if(Language == ENGLISH) {
			PrintString(ppText, pTextEnd, "Comp");
			PrintF32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->fAcqSummary, i8SgnDigits);
			PrintString(ppText, pTextEnd, "Real");
			PrintU32(ppText, pTextEnd, ((ACQ_SUMM*)(pDataAcsProp->pData))->u32SummaryRealNum, 0);
			PrintString(ppText, pTextEnd, ";NumIni");
		}
		
		if(DataUser == DATA_USER_NET) {
			*(*ppText)++ = '"';
			*(*ppText)++ = ',';
		}
	}
}

/* 打印 g_DataAcsIntf.i8SysFunc 的操作结果 */
void PrintSysFuncRes(uint8** ppTxBuf, uint8* pTextEnd, CONF_n_STAT_PROP* pDataAcsProp, LANG_TYPE Language)
{
    /* 输入检查 */
    if((Language >= MAX_LANG_TYPE) || (pDataAcsProp->LkId_Val_ItN >= MAX_SYS_FUNC)) {
        return;
    }
    int8 i8DataOperation = g_DataAcsIntf.i8SysFunc[pDataAcsProp->LkId_Val_ItN];
    if(i8DataOperation < 0) {           /* 文字形式的输出 */
        if(i8DataOperation < DATA_ACS_RES_MAX) {
            PrintStringNoOvChk(ppTxBuf, cnst_DataAcsOprRes[0 - DATA_ACS_RES_MAX][Language]);
        } else {                        /* 执行结果 */
            PrintStringNoOvChk(ppTxBuf, cnst_DataAcsOprRes[0 - i8DataOperation][Language]);
        }
    } else if(i8DataOperation > 0) {    /* 数值，形成倒计时 */
        PrintU32(ppTxBuf, pTextEnd, i8DataOperation, 0);
    }
}

/*==========================================================================
| Description	: 用于处理DATA_USER_GUI下的conf,stat,acq,msg指令以及DATA_USER_NET下conf指令(stat包含在conf页面之中)
		1. 初始化指令: ItemNo写入0xFFFFFFFFUL
		2. CONF(含STAT)页面，Topic下支持如下页面操作指令：
		   a. english, chinese 设定页面显示语言，用于mqtt配置
		   b. back: 返回上一级页面
		   c. up, down当前页面显示窗口上下滚动，并尽量让枚举列表在一个窗口内显示，用于UI配置
		   d. exit: 退出配置状态，即启动参数存储
		3. S32(写任意值)\LINK(如加密须写入正确密码，否则写任意值)进入下一级页面
		4. BOOL支持true(真),false(假),click(取反)
		5. ECONT支持true,click，意义都是把当前的ECONT值赋给列表所指向的ENUM变量
			当然也可以直接写该ENUM变量
		6. 其他配置类型直接写对应的值，U32类型可以支持浮点数输入
| In/Out/G var	: 返回值: -1:成功 -2:失败 >0:等待时间(保存尝试次数)
| Author		: Wang Renfei			Date	: 2018-07-31
\=========================================================================*/
int8 ProcItemPageInstr(DATA_USER DataUser, DATA_PAGE DataPage, uint32 u32ItemNo, uint8** ppU8Instr, uint8* pU8InstrEnd, ITEM_PAGE_REQ* pItemPageReq)
{
	CONF_n_STAT_PROP* pDataAcsProp;
	int16 i;
	int8 i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = 0;

	/* 初始化页面 */
	if(pItemPageReq == NULL) {
		pDataAcsProp = pBootLoaderConf->pConfProp + u32ItemNo;
	} else if((DataPage != pItemPageReq->DataPage) || (u32ItemNo == 0xFFFFFFFFUL)) {
		pItemPageReq->DataPage = DataPage;
		if((DataPage == DATA_PAGE_MSG) || (DataPage == DATA_PAGE_ACQ)) {	/* 无需响应后面的写入指令 */
			pItemPageReq->u32ItemNo = 0xFFFFFFFFUL;
			return -1;
		}

		u32ItemNo = 0;									/* 无需更新pDataAcsProp */
		pDataAcsProp = pBootLoaderConf->pConfProp;
		pItemPageReq->u32ItemNo = u32ItemNo;
		pItemPageReq->uConfPageTopicItemNo = u32ItemNo;
		pItemPageReq->uConfWinFirstLineItemNo = u32ItemNo + 1;
	} else if((DataPage == DATA_PAGE_MSG) || (DataPage == DATA_PAGE_ACQ)) {			/* 无需响应后面的写入指令 */
		pItemPageReq->bMsgAcqPageWinDown = (u32ItemNo < pItemPageReq->u32ItemNo);
		pItemPageReq->u32ItemNo = u32ItemNo;
		return -1;
	} else if((u32ItemNo < pItemPageReq->uConfPageTopicItemNo)							/* 输入检查 */
			|| (u32ItemNo > pItemPageReq->uConfPageTopicItemNo + pBootLoaderConf->pConfProp[pItemPageReq->uConfPageTopicItemNo].LkId_Val_ItN))
	{
		return -2;
	} else {
		pDataAcsProp = pBootLoaderConf->pConfProp + u32ItemNo;
	}
	
	/* 配置页面数据处理 */
	if(ppU8Instr == NULL) {
		return -1;
	}
	uint8* pU8Instr = *ppU8Instr;
	if(pU8Instr >= pU8InstrEnd) {
	} else if((*pU8Instr == 0) || (*pU8Instr == '"')) {	/* 第一个字符串没数 */
		pU8Instr++;		/* 调整指针，跳过该配置结尾 */
	} else {
		i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -1;
		uint32 u32ItemData = 0;
		BOOL bEditConfValueInstr = FALSE;
		
		/* SYNC操作页面维护指令 */
		if(pItemPageReq == NULL) {
			bEditConfValueInstr = TRUE;
			if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TOPIC) {
				if(!GetU32(&pU8Instr, pU8InstrEnd, &u32ItemData, 0)) {
					i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
				}
			}
		} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_TOPIC) {	/* 处理 refresh,back,up,down */
			if(CompareInstr(&pU8Instr, pU8InstrEnd, "refresh")) { 			/* 标志结果为-1就会导致刷新 */
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "exit")) {
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "back")) {		/* 返回到该窗口初始位置，因此所有跳转都必须在第一个窗口 */
				if(pItemPageReq->uConfPageTopicItemNo > 0) {	/* 2021-08-21 非首页才需要执行返回 */
					void DealGuiConfWinBack(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);
					DealGuiConfWinBack(DataUser, pItemPageReq);
//				} else {	/* 如果加上此逻辑,iconfig会出现异常 */
//					i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = 0;	/* 否则返回失败 */
				}
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "up")) {
				void DealGuiConfWinUp(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);
				DealGuiConfWinUp(DataUser, pItemPageReq);
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "down")) {
				void DealGuiConfWinDown(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq);
				DealGuiConfWinDown(DataUser, pItemPageReq);
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "chinese")) {
				pItemPageReq->Language = CHINESE;
			} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "english")) {
				pItemPageReq->Language = ENGLISH;
			} else {
				i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
			}
		} else if((pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK)	/* 处理进入下一级页面 */
				|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_S32))
		{
			if((pDataAcsProp->ItemType.DataType == ITEM_DAT_S32)		/* 无密码情况，需要输入"click" */
				|| g_Ctr.bEngDebug 
				|| (DataUser == DATA_USER_NET)							/* 2021.7.2 目前网络配置我们内部使用，忽视密码 */
				|| (pDataAcsProp->pData == NULL)
				|| (*((uint32*)pDataAcsProp->pData) == 0)
				|| ((pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK)	/* 密码验证 */
					&& (*((uint32*)pDataAcsProp->pData) == ReadU32(&pU8Instr, pU8InstrEnd))))
			{
			    for(i = pBootLoaderConf->u32ItemNum - 1; 
			            (i >= 0) 
			            && ((pBootLoaderConf->pConfProp[i].ItemType.DataType != ITEM_DAT_TOPIC)  /* 确保进入TOPIC */
			                || (pDataAcsProp->LkId_Val_ItN != pBootLoaderConf->pConfProp[i].u32ItemID)); 
			                i--) 
			    { }
			    if(i >= 0) {
    				pItemPageReq->uConfPageTopicItemNo = i;
    				pItemPageReq->uConfWinFirstLineItemNo = pItemPageReq->uConfPageTopicItemNo + 1;
    		    } else {
                    i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
    		    }
			} else {
				i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
			}
        } else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_FUNC) {
            if(RegisterSysFunc((SYS_FUNCTION)pDataAcsProp->LkId_Val_ItN, 
                    (pDataAcsProp->pData == NULL) || (*(uint32*)pDataAcsProp->pData == ReadU32(&pU8Instr, pU8InstrEnd))))
            {
                i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = 5;
            } else {
                i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
            }
		} else {
			bEditConfValueInstr = TRUE;
		}

		/* 数据修改指令 */
		if(bEditConfValueInstr) {
			uint16 uBitStart = 0;
			uint16 uVarBitWidth = 0;
			BOOL bGetSuc = TRUE;

			/* ECONT需要预先处理，因为处理完后需要把指针移到ENUM */
			BOOL bItemIsECONT = FALSE;
			if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ECONT) {
				bItemIsECONT = TRUE;
				if(CompareInstr(&pU8Instr, pU8InstrEnd, "click") || CompareInstr(&pU8Instr, pU8InstrEnd, "true")) {	/* 选中当前项目 */
					u32ItemData = pDataAcsProp->LkId_Val_ItN;
					do {
						pDataAcsProp--;
					} while(pDataAcsProp->ItemType.DataType != ITEM_DAT_ENUM);	/* 把当前指针移到ENUM，这才是真正的变量位置 */
				} else {
					bGetSuc = FALSE;
				}
			}

			/* 当前变量是位变量，需要计算起始位和位宽 */
			if((pItemPageReq != NULL)
				&& (pDataAcsProp->pData == NULL) 
				&& pDataAcsProp->ItemType.i8SgnDigits_u8BitNum
				&& ((pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32)))
			{
				CONF_n_STAT_PROP* pDataAcsPropBak = pDataAcsProp;		/* 备份该指针，以免下面计算过程的修改 */
				uVarBitWidth = pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
				pDataAcsProp--;
				for(i = pDataAcsProp - (pBootLoaderConf->pConfProp + pItemPageReq->uConfPageTopicItemNo); i > 0; i--) {
					if((pDataAcsProp->pData == NULL) 	/* 确定为位变量 */
						&& pDataAcsProp->ItemType.i8SgnDigits_u8BitNum
						&& ((pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL)
							|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM)
							|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32)
							|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_RO))) 
					{
						uBitStart += pDataAcsProp->ItemType.i8SgnDigits_u8BitNum;
					}
					pDataAcsProp--;
				}
				pDataAcsProp = pDataAcsPropBak;
			}

			/* 部分项目预先处理 */
			if(bItemIsECONT) {		/* 指针已经调整到ENUM，防止后面进入ENUM */
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM) {
				bGetSuc = GetU32(&pU8Instr, pU8InstrEnd, &u32ItemData, 0);
			} else if((pDataAcsProp->ItemType.DataType == ITEM_DAT_U32) || (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32_RO)) {
				bGetSuc = GetU32(&pU8Instr, pU8InstrEnd, &u32ItemData, pDataAcsProp->ItemType.uBcdQ);
			} else if(pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL) {
				if(CompareInstr(&pU8Instr, pU8InstrEnd, "true")) {
					u32ItemData = TRUE;
				} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "false")) {
					u32ItemData = FALSE;
				} else if(CompareInstr(&pU8Instr, pU8InstrEnd, "click")) {
					if(pDataAcsProp->pData != NULL) {
						u32ItemData = !(*((uint32*)pDataAcsProp->pData));
					} else if(pDataAcsProp->ItemType.i8SgnDigits_u8BitNum && (pItemPageReq != NULL)) {
						u32ItemData = *((uint32*)((pBootLoaderConf->pConfProp + pItemPageReq->uConfPageTopicItemNo)->pData));
						u32ItemData = (u32ItemData>>uBitStart) & ((1<<uVarBitWidth) - 1);
						u32ItemData = !u32ItemData;
					} else {
						bGetSuc = FALSE;
					}
				} else {
					u32ItemData = (ReadU32(&pU8Instr, pU8InstrEnd) != 0);
				}
			} 

			/* 位变量，则把数据存储在topic所在的行 */
			if(bGetSuc && (pDataAcsProp->pData == NULL) && (pItemPageReq != NULL)
				&& pDataAcsProp->ItemType.i8SgnDigits_u8BitNum
				&& ((pDataAcsProp->ItemType.DataType == ITEM_DAT_BOOL)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM)
					|| (pDataAcsProp->ItemType.DataType == ITEM_DAT_U32)))
			{
				pDataAcsProp = pBootLoaderConf->pConfProp + pItemPageReq->uConfPageTopicItemNo;	/* 修改pDataAcsProp指向Topic行 */
				
				/* 计算页面变量 */
				u32ItemData = ((*(uint32*)pDataAcsProp->pData)									/* 取出Topic的变量 */
							  & ((0 - (1<<(uBitStart + uVarBitWidth))) | ((1<<uBitStart) - 1)))	/* 清空变量所占据的位空间 */
				 			  | ((u32ItemData & ((1<<uVarBitWidth) - 1))<<uBitStart);			/* 规格化当前变量，不要占据超过位宽 */
			}

			/* 进行赋值 */
			if((!bGetSuc) || (pDataAcsProp->pData == NULL)		/* 失败：写数据空 */
				|| (((pItemPageReq != NULL)				/* 只读变量，仅配置同步，且非SAVE_GRP_BRD数据，才可写入 */
						|| (pDataAcsProp->ItemType.SaveGrp == SAVE_GRP_NUL)
						|| (pDataAcsProp->ItemType.SaveGrp == SAVE_GRP_BRD))
					&& ((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TEXT)
					&& (pDataAcsProp->ItemType.DataType != ITEM_DAT_ENUM)))
			{
				i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
			} else {											/* 数据不空 */
				uint8* pU8Dat;
				switch(pDataAcsProp->ItemType.DataType) {
					case ITEM_DAT_TOPIC:
					case ITEM_DAT_U32:
					case ITEM_DAT_U32_RO:
					case ITEM_DAT_BOOL:
					case ITEM_DAT_ENUM:
						*((uint32*)pDataAcsProp->pData) = u32ItemData;
						break;
						
					case ITEM_DAT_TEMPSENCONF:
					case ITEM_DAT_ACQ_U32:
					case ITEM_DAT_D2U32:
						bGetSuc = GetU32(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData, pDataAcsProp->ItemType.uBcdQ);
						break;
					
					case ITEM_DAT_IPv4:
						bGetSuc = GetIPv4(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData);
						break;
					
					case ITEM_DAT_pTIME:
						bGetSuc = GetPriceTime(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData);
						break;
					
				#if SUPPORT_IEC101 || SUPPORT_IEC104
					case ITEM_DAT_TELE_ITEM:
						bGetSuc = GetTeleSigMetConfFromText(&pU8Instr, pU8InstrEnd, (TELE_101_104_ITEM_CONF*)pDataAcsProp->pData);
						break;
				#endif
					
					case ITEM_DAT_F32:
					case ITEM_DAT_ACQ_F32:
					case ITEM_DAT_D2F32:
						bGetSuc = GetF32(&pU8Instr, pU8InstrEnd, (float32*)pDataAcsProp->pData);
						break;
					
					case ITEM_DAT_SUMM:
						bGetSuc = GetI32(&pU8Instr, pU8InstrEnd, (int32*)pDataAcsProp->pData, 0);
						InformAcqSummConfChange(pDataAcsProp->pData);
						break;
					
                    case ITEM_DAT_I64_RO:
					case ITEM_DAT_I64:
						bGetSuc = GetI64(&pU8Instr, pU8InstrEnd, (int64*)pDataAcsProp->pData, pDataAcsProp->ItemType.uBcdQ);
						break;
					
					case ITEM_DAT_RMTS:
						if(DataUser != DATA_USER_NET) {	/* 如果是UI配置,仅替换序列号部分 */
							/* 计算输入字符串长度 */
							pU8Dat = SkipCharInString(pU8Instr, pU8InstrEnd, 0, 1);
							i = pU8Dat - pU8Instr;
							pU8Dat--;
							if(*pU8Dat == 0) {
								i--;
							}
							/* 计算原始配置序列号长度 */
							pU8Dat = (uint8*)pDataAcsProp->pData;
							pU8Dat = SkipCharInString(pU8Dat, pU8Dat + CONF_ANSI_BYTE_LEN, '/', 1);
							if(i == SkipCharInString(pU8Dat, (uint8*)pDataAcsProp->pData + CONF_ANSI_BYTE_LEN, '/', 1) - pU8Dat - 1)  {
								for( ; i > 0; i--) {
									*pU8Dat++ = *pU8Instr++;
								}
							} else {
								bGetSuc = FALSE;
							}
							break;
						}	/* 否则，和ITEM_DAT_ANSI一样处理 */
					case ITEM_DAT_DIGITSTR:
					case ITEM_DAT_ANSI:
					case ITEM_DAT_ANSI_RO:
						pU8Dat = (uint8*)pDataAcsProp->pData;
						Swi_disable();
						if(DataUser == DATA_USER_NET) {
							for(i = CONF_ANSI_BYTE_LEN - 1; (i > 0) && (*pU8Instr != '"'); i--) {
								*pU8Dat++ = *pU8Instr++;
							}
						} else {
							for(i = CONF_ANSI_BYTE_LEN - 1; (i > 0) && (*pU8Instr != 0); i--) {
								*pU8Dat++ = *pU8Instr++;
							}
						}
						*pU8Dat++ = 0;						
						Swi_enable();
						bGetSuc = (i != 0);		/* 说明Buf够长 */
						break;

					case ITEM_DAT_T64_RO:
						Swi_disable();
						bGetSuc = GetT64(DataUser, &pU8Instr, pU8InstrEnd, (REAL_TIME_VAR*)pDataAcsProp->pData);
						Swi_enable();
						break;

					case ITEM_DAT_T32_RO:
						bGetSuc = GetT32(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData);
						break;

					default:
						bGetSuc = FALSE;
						break;
				}
				
				if(bGetSuc) {
					i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = 5;
				} else {
					i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
				}
			}
		}

		/* 修改指令指针，指向当前指令结尾 */
		if(DataUser == DATA_USER_NET) {
			/* 指令存在三种形式: "name":"1","name":["3,4"],"name":["3","4"] */
			for(i = pU8InstrEnd - pU8Instr; (i > 0) && (*pU8Instr != ','); i--) {	/* 搜索',' [有两种情况会出现逗号,1:变量之间，2:D2类变量内] */
				pU8Instr++;
			}
			/* 调整指针，跳过该配置结尾 */
			if((pU8Instr[1] == '"') && ((pDataAcsProp->ItemType.DataType>>4) == ITEM_UI_TWOBOX)) {	/* "name":["3","4"]情况 */
				pU8Instr += 2;
			} else {	/* "name":"1","name":["3,4"]情况 */
				pU8Instr++;
			}
		} else {
			for(i = pU8InstrEnd - pU8Instr; (i > 0) && (*pU8Instr != 0); i--) {
				pU8Instr++;
			}
			pU8Instr++;	/* 调整指针，跳过该配置结尾 */
		}
	}

	/* 是否第二个配置项 */
	if((pU8Instr < pU8InstrEnd) && (*pU8Instr != 0) && (*pU8Instr != '"')) {
		BOOL bGetSuc = FALSE;
		switch(pDataAcsProp->ItemType.DataType) {
			case ITEM_DAT_D2U32:
				bGetSuc = GetU32(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData + 1, pDataAcsProp->ItemType.uBcdQ);
				break;

			case ITEM_DAT_SUMM:				
				bGetSuc = GetF32(&pU8Instr, pU8InstrEnd, (float32*)pDataAcsProp->pData + 1);
				InformAcqSummConfChange(pDataAcsProp->pData);
				break;
				
			case ITEM_DAT_D2F32:
				bGetSuc = GetF32(&pU8Instr, pU8InstrEnd, (float32*)pDataAcsProp->pData + 1);
				break;
			
			case ITEM_DAT_pTIME:
				bGetSuc = GetPriceTime(&pU8Instr, pU8InstrEnd, (uint32*)pDataAcsProp->pData + 1);
				break;

			default:
				break;
		}

		if(!bGetSuc) {
			i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = -2;
		} else if(i8ST_Idle0_SucN1_FailN2_MaxWaitCntP == 0) {	/* 前面没有失败，这里又成功了 */
			i8ST_Idle0_SucN1_FailN2_MaxWaitCntP = 5;
		}
	}

	/* 处理完成 */
 /* 存储数据:由于存储介质(tf卡/外部I2C接口的EEPROM等)往往低速，为了防止通讯程序超时，需要把存储部分独立成一个任务
	通讯程序和存储任务间通过"变量+信号量"进行传递 */
	if((i8ST_Idle0_SucN1_FailN2_MaxWaitCntP > 0) && (pDataAcsProp->ItemType.SaveGrp != SAVE_GRP_NUL)) {
		g_DataAcsIntf.bConfHaveUnsavedChange[pDataAcsProp->ItemType.SaveGrp] = TRUE;
		g_MqttItemProc.i8SyncST_Idle0_SucN1_FailN2_MaxWaitCntP = 5;		/* 有修改参数，就需要重新同步(备份)数据 */
		Semaphore_post(g_DataAccessBlock.Sem_Op);		//Semaphore_post(SEM_NvMemAcs);
	}

	if(i8ST_Idle0_SucN1_FailN2_MaxWaitCntP == 0) {		/* 指令无任何参数 */
		return -2;
	} else {
		*ppU8Instr = pU8Instr;
		return i8ST_Idle0_SucN1_FailN2_MaxWaitCntP;
	}
}

/* GUI配置页面返回 */
void DealGuiConfWinBack(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq)
{
	uint16 uConfID = pBootLoaderConf->pConfProp[pItemPageReq->uConfPageTopicItemNo].u32ItemID;
	int16 i;

	CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp;
	for(i = pBootLoaderConf->u32ItemNum; i > 0; i--) {	/* 搜索跳转到该行的LINK或者S32 */
		if(((pDataAcsProp->ItemType.DataType == ITEM_DAT_LINK) || (pDataAcsProp->ItemType.DataType == ITEM_DAT_S32))
			&& (pDataAcsProp->LkId_Val_ItN == uConfID))
		{
			break;
		}
		pDataAcsProp++;
	}
	pDataAcsProp--;
	/* 再搜索该页面的Topic */
	for(i = pDataAcsProp - pBootLoaderConf->pConfProp; (i >= 0) && (pDataAcsProp->ItemType.DataType != ITEM_DAT_TOPIC); i--) {
		pDataAcsProp--;
	}
	pItemPageReq->uConfPageTopicItemNo = pDataAcsProp - pBootLoaderConf->pConfProp;
	pItemPageReq->uConfWinFirstLineItemNo = pItemPageReq->uConfPageTopicItemNo + 1;
	if(DataUser == DATA_USER_MCGS) {
		pItemPageReq->u32ItemNo = pItemPageReq->uConfPageTopicItemNo;
	}
}

/* GUI配置页面上翻页: 翻页的时候，最好让枚举类型在一页里面 */
void DealGuiConfWinUp(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq)
{
	CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp + pItemPageReq->uConfWinFirstLineItemNo;
	int16 i;
	BOOL bEnumCmplt = FALSE;		/* 能形成完整的枚举列表 */
	
	/* 如果当前窗口第一行是ECONT，则说明前面是一个ENUM。需要搜索到当前窗口从第一行开始连续的最后一个ECONT
	   然后往前看ENUM, 如果能把当前"截断"的枚举列表放在一个页面里面，就以最后一个ECONT放在将显示窗口的末尾
	   否则，就不考虑维持枚举列表的完整性 */
	if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ECONT) {
		pDataAcsProp++;
		for(i = 5; i > 0; i--) {	/* 搜索当前窗口从第一行开始连续的最后一个ECONT */
			if(pDataAcsProp->ItemType.DataType != ITEM_DAT_ECONT) {
				pDataAcsProp--;
				break;
			}
			pDataAcsProp++;
		}
		if(i) { 	/* 说明当前窗口最后一行不是ECONT, 有形成完整枚举列表的可能性 */
			for(i = 2; i <= 6; i++) {	/* 当前指向最后一个ECONT, 往前搜索ENUM，i等于当前列表项目 */
				pDataAcsProp--;
				if(pDataAcsProp->ItemType.DataType == ITEM_DAT_ENUM) {
					break;
				}
			}
			if(i <= 6) {	/* (6 - i)是枚举列表前面还可以放置多少个项目 */
				pItemPageReq->uConfWinFirstLineItemNo = pDataAcsProp - pBootLoaderConf->pConfProp - (6 - i);
				bEnumCmplt = TRUE;
			}
		}
	} 
	if(!bEnumCmplt) {	/* 不考虑形成完整的枚举列表 */
		i = pItemPageReq->uConfWinFirstLineItemNo - 6;
		if(i < (int16)(pItemPageReq->uConfPageTopicItemNo + 1)) {		/* 防止前面减出负值 */
			pItemPageReq->uConfWinFirstLineItemNo = pItemPageReq->uConfPageTopicItemNo + 1;
		} else {
			pItemPageReq->uConfWinFirstLineItemNo = i;
		}
	}
	if(DataUser == DATA_USER_MCGS) {
		pItemPageReq->u32ItemNo = pItemPageReq->uConfPageTopicItemNo;
	}
}

/* GUI配置页面下翻页: 翻页的时候，最好让枚举类型在一页里面 */
void DealGuiConfWinDown(DATA_USER DataUser, ITEM_PAGE_REQ* pItemPageReq)
{
	CONF_n_STAT_PROP* pDataAcsProp = pBootLoaderConf->pConfProp + pItemPageReq->uConfPageTopicItemNo;

	/* 计算到结尾需要翻多少条 */
	int16 i = pItemPageReq->uConfPageTopicItemNo + pDataAcsProp->LkId_Val_ItN + 1 - (pItemPageReq->uConfWinFirstLineItemNo + 6);
	if(i > 6) { 	/* 一次翻页最多不超过6 */
		i = 6;
	}
	if(i > 0) {
		/* 未结束的枚举类型 */
		if(pBootLoaderConf->pConfProp[pItemPageReq->uConfWinFirstLineItemNo + 6].ItemType.DataType == ITEM_DAT_ECONT) {
			pDataAcsProp = pBootLoaderConf->pConfProp + pItemPageReq->uConfWinFirstLineItemNo + 1;
			while((i > 0) && (pDataAcsProp->ItemType.DataType != ITEM_DAT_ENUM)) {
				pDataAcsProp++;
				i--;
			}
			pItemPageReq->uConfWinFirstLineItemNo = pDataAcsProp - pBootLoaderConf->pConfProp;
		} else {
			pItemPageReq->uConfWinFirstLineItemNo += i;
		}
	}
	if(DataUser == DATA_USER_MCGS) {
		pItemPageReq->u32ItemNo = pItemPageReq->uConfPageTopicItemNo;
	}
}

/******************************** FILE END ********************************/

/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved. ģ��V1.3
 *
 * File			: MdlUARTnModbus.c
 * Author		: Wang Renfei
 * Description	: 
 
 ֧��RS485�����ϵĶ��ֹ���: һ��ModbusRTU�ӻ���ViewTech��Ļ������ˮλ(ModbusRTU���� 1200bps)
 ÿһ�ֹ��ܰ���: Try**, Run** ������Ҫ������
 Try**	 : �Ը�Ӧ�������е�ͨѶ����(�粨����)��ʼ��Ӳ��, ���շ��������ݣ���ȷ����ǰģʽ�Ƿ�ǡ����
 Run**	 : ���ݳ�֡�������н���

 ���ֹ����Զ�ʶ���㷨:
 ÿһ�γɹ���ͨѶ(CRC�����ȷ)���͸�λ��Ӧ��ͨѶʧ��(���Ź�)��ʱ����������Ź������������ʶ��(����Try**����)

 ���ݲ�ͬ��ͨѶЭ��(��Ҫ��ModbusRTU, ViewTech����ͨѶЭ��), ����TxData**, CheckRxBufCrcFor**��������
 TxData**  			: ���ظ�Э���CRC�����ݳ��ȵȣ���������ݷ���
 CheckRxBufCrcFor** : ���ݸ�Э���RxBuf����CRC���

 ���⣬���Ľ����ṩ��RS485ͨѶ��ص�������������:
 OpenUartComm			: ��װ��UART_open���ڵĴ�RS485�ĺ���
 ReadDataFromRS485	: ��RS485��ȡ����

 * Date			: 2015-10-22
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 **************************************************************************/

/***************************************************************************
 						macro definition
***************************************************************************/
#define _MDL_RS485_n_MODBUS_C_		/* exclude redefinition */

/***************************************************************************
 						include files
***************************************************************************/
/* ��Ŀ����ͷ�ļ� */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"

/* ����ϵͳͷ�ļ� */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/* �����Լ��¼�ģ��ͷ�ļ� */
#include "MdlUARTnModbus.h"
#include "MdlGUI.h"
#include "BoardSupport.h"

#if SUPPORT_V4_MASTER
#include "MdlV4Master.h"
#endif
#if SUPPORT_IEC101
#include "Mdl101n104.h"
#endif
#if SUPPORT_FLOW_METER
#include "MdlFlow.h"
#endif
#ifdef CAN_BASE
#include "MdlCan.h"
#endif
#if (SUPPORT_TRIPHASE_METER || SUPPORT_PGA460 || SUPPORT_WTRLVL_OPT_TX || SUPPORT_WTRLVL_OPT_RX || SUPPORT_RADIO_SUB || SUPPORT_RADIO_PUB || SUPPORT_GPS || SUPPORT_AIR_COMPRESSOR || SUPPORT_GNL_MODBUS_METER || SUPPORT_PUMP)
#include "MdlExtUartApp.h"
#endif
#ifdef SUPPORT_RAKE_FLOOD
#include "MdlRakeFloodWell.h"
#endif

/***************************************************************************
 						global variables definition
***************************************************************************/
#define RS485_TR_FLASH_PACK_BLEN		(UART_TR_BUF_BLEN - 16)

/* ModbusͨѶ��ַ */
#define MODADD_SIM_XM_START					0			/* ģ��ˮλ�Ǳ�� */
#define MODADD_SIM_XM_END					1000

#define MODADD_MISC_START					1000		/* ��������� */
#define MODADD_DEVICE_START					1000		/* �豸��Ϣ */
	#define MODADD_DEVICE_TYPE				1000		/* �豸�ͺ� */
	#define MODADD_DEVICE_CONFNO			1001		/* �豸����--���ͺ� */
	#define MODADD_DEVICE_HARD_VER			1002		/* Ӳ���汾�ţ�����ͬ�豸�ͺ� */
	#define MODADD_DEVICE_SOFT_VER			1003		/* ����汾�� */
	#define MODADD_DEVICE_SERNOH			1004		/* ���кŸ�λ */
	#define MODADD_DEVICE_SERNOL			1005		/* ���кŵ�λ */
	#define MODADD_DEVICE_CPUTYPE			1006		/* CPU�ͺţ����ڿ� */
#define MODADD_DEVICE_END 					1010
	
#define MODADD_CTR_START				    1010		/* ���������� */
#define MODADD_CTR_END					    1100
	
#define MODADD_MCGS_ITEMPAGE_START			1120		/* MCGS��Ļ conf,msg,acq��ʾ(ItemPage) */
#define MODADD_MCGS_ITEMPAGE_END			1153
	
#define MODADD_SYS_OPR_START				1190		/* ϵͳ����ҳ�棬ÿһ������һ��32bit��ַ(������modbus��ַ)���ṩ�������롢״̬��ѯ */
#define MODADD_SYS_OPR_END					1200
#define MODADD_MISC_END						1200

#define MODADD_MSG_START                    1200
#define MODADD_MSG_END                      1300

#define MODADD_MSR_RES_START				9200		/* �����������(g_MsrRes,R) */
#define MODADD_MSR_RES_END					9600

#define MODADD_ANA_RES_START				9600		/* ������������(g_AnaRes,R) */
#define MODADD_ANA_RES_END					10000

/* 10000~60000�������ڵ��ԣ���ʱ���� */
#define MODADD_DEBUG_CHN_START				0x3000		/* 12288 �����ڵ��ԣ���ַ���ܻᱻ�޸ģ�ʵ�ʷ�����Ҫ��ע */
#define MODADD_DEBUG_CHN_END				0x3800
/* 10000~60000�������ڵ��ԣ���ʱ���� */

#define MODADD_MON_CMD_START				60000		/* ��ص�ַ */
#define MODADD_MON_KEY_ID					60000		/* ң�ذ�ťID */
#define MODADD_MON_CMD_END					61000

#define MODADD_UPDATE_START					61000		/* �������:ͨ��RS485�����������Ҫ�����а�����Ϊ���԰� */
#define MODADD_UPDATE_END					61128

#define MODADD_CODE_TEST_START				0xF000		/* CodeTest���� */
#define MODADD_CODE_TEST_U32_END			0xF100
#define MODADD_CODE_TEST_U16_END			0xF200
#define MODADD_CODE_TEST_END				0xF200

#define MODADD_MASTER_START					0xFF00		/* ��������:0xFF00�������� */
#define MODADD_MASTER_END					0xFFFF

/***************************************************************************
						internal functions declaration
***************************************************************************/
#if (DEVICE_TYPE == V5_YYT3)
void TrySerialServer(uint8 u8UartPort);		/* ���ڷ�����������֧��modbusTCP������� */
void RunSerialServer(uint8 u8UartPort);
#endif

void TryModbusRTUSlave(uint8 u8UartPort, uint32 u32BaudRate, uint16 uMaxDelay_ms);
void RunModbusRTUSlave(uint8 u8UartPort, uint16 uMaxDelay_ms);

void TrySubSoftUpdate(uint8 u8UartPort);
void RunSubSoftUpdate(uint8 u8UartPort);

/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: 
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2017-9-19
\=========================================================================*/
void InitUartComm(void)
{
	int16 i;

	/* ��ͬ�Ĳ�ƷRS485 App��ʼ�� */
#if(DEVICE_TYPE == V5_YKF2)
	g_UartComm[0].UartApp = RS485_MCGS_RTU_SLAVE;
	g_UartComm[1].UartApp = UART_VIEW_TECH;
#elif(SOFT_RUN1_TEST0 && ((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4)))
	g_UartComm[0].UartApp = RS485_APP_NULL;
#else
	for(i = 0; i < MAX_UART_NUM; i++) {
		g_UartComm[i].UartApp = RS485_MCGS_RTU_SLAVE;
	}
#endif

	/* RS485ͨ�ó�ʼ�� */
	for(i = 0; i < MAX_UART_NUM; i++) {
		g_UartComm[i].Handle = NULL;
		g_UartComm[i].bUartFuncTrySuc = FALSE;				/* ��ʼ������־����ʧ�� */
		g_UartComm[i].uTimer_WatchDog_ms = 0xFFFF;			/* ��ʼ����ʹ�ú��������߳���״̬ */
		g_UartComm[i].LastUartApp = RS485_APP_AUTO;
		g_UartComm[i].u8RS485AppAutoTablePt = 0;
		g_UartComm[i].uLastBaud_100bps = 0;

        InitDataWithZero((uint8*)(&g_UartComm[i].ItemPageReq), sizeof(ITEM_PAGE_REQ));
	}
#if SUPPORT_MODBUS_TCP
	for(i = 0; i < MODBUS_TCP_COMM_MAX_NUM; i++) {
		g_ModbusTcpComm[i].bFree = TRUE;
		g_ModbusTcpComm[i].i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = 0;
		g_ModbusTcpComm[i].uRxBufPt = 0;
		g_ModbusTcpComm[i].uTxBufPt = 0;
	}
#endif
    InitDataWithZero((uint8*)(&g_MiscCommData.ItemPageReq), sizeof(ITEM_PAGE_REQ));
	
	
#if SUPPORT_V4_MASTER
	InitV4Master();
#endif
}

/*==========================================================================
| Description	: 
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2015-9-27
\=========================================================================*/
/* RS485�˿ڹ��ܱ� */
#if (DEVICE_TYPE == V5_YBT2)
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {UART_APP_MAX, RS485_APP_AUTO, RS485_YKFR1_XMS_9600};
#elif((DEVICE_TYPE == V5_YBT3) || (DEVICE_TYPE == V5_YBT4))
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_YKFR1_XMS_9600, RS485_APP_AUTO, RS485_IEC101};
#elif((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3))
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_APP_AUTO, UART_VIEW_TECH};
#elif(DEVICE_TYPE == V5_YBU4)
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_APP_NULL, UART_VIEW_TECH, UART_WATERLVL_OPT_TX};
#elif(DEVICE_TYPE == V5_YYT3)
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_APP_NULL, RS485_APP_NULL, RS485_APP_NULL, UART_WATERLVL_OPT_RX, UART_GPS};
#elif(DEVICE_TYPE == V5_YYT4)		/* for debug */
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_APP_NULL, RS485_APP_NULL, RS485_APP_NULL, UART_APP_GPRS};
#elif(DEVICE_TYPE == YKC)
const UART_APP cnst_UartAppConf[MAX_UART_NUM] = {RS485_MCGS_RTU_SLAVE};
#endif

/* RS485�Զ����ñ� */
#if (!SOFT_RUN1_TEST0)
const UART_APP cnst_RS485AutoTable[] = {RS485_MCGS_RTU_SLAVE, UART_VIEW_TECH};
#elif ((DEVICE_TYPE == V5_YBT2) || (DEVICE_TYPE == V5_YBT3) || (DEVICE_TYPE == V5_YBT4) || (DEVICE_TYPE == V5_YYT3))
const UART_APP cnst_RS485AutoTable[] = {RS485_SOFT_UPDATE, RS485_MCGS_RTU_SLAVE, RS485_YKFR2_38400, RS485_WATERLV_PGA460};
#elif Is_YKF(DEVICE_TYPE)
const UART_APP cnst_RS485AutoTable[] = {RS485_SOFT_UPDATE, RS485_MCGS_RTU_SLAVE, UART_VIEW_TECH, RS485_V5YKF_PERIPHERAL, RS485_DEBUG_PACK};
#elif((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))
const UART_APP cnst_RS485AutoTable[] = {RS485_SOFT_UPDATE, RS485_TRIPHASE_METER, RS485_MCGS_RTU_SLAVE, RS485_WATERLV_PGA460};
#elif(DEVICE_TYPE == YKC)
const UART_APP cnst_RS485AutoTable[] = {RS485_MCGS_RTU_SLAVE, RS485_APP_NULL};
#else
const UART_APP cnst_RS485AutoTable[] = {RS485_SOFT_UPDATE, RS485_MCGS_RTU_SLAVE, UART_VIEW_TECH};
#endif

void UartCommTask(void const * argument)
{
	uint8 u8UartPort = (uint8)((uint32)argument);
	if(u8UartPort >= MAX_UART_NUM) {	/* ������, Ŀ���Ǳ���YKF2,YKF3 app.cfg��ͬ */
		return;
	}

	/* �������� */
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	for(;;) {
#if (!SOFT_RUN1_TEST0)	/* ���������ͨѶ����ѡ��mcgs����vt��ĻS */
	UART_APP UartAppConf = RS485_APP_AUTO;
#else	/* ���ݲ�ͬ���豸��ȡ��ǰ�� RS485AppConf */
	#if SUPPORT_BEIDOU_HUB
		UART_APP UartAppConf = RS485_BEIDOU_HUB;
	#elif(Is_YKF(DEVICE_TYPE) || (DEVICE_TYPE == V5_YYD) || (DEVICE_TYPE == V5_YYD2) || (DEVICE_TYPE == V5_YYG))
        UART_APP UartAppConf = RS485_APP_AUTO;
	#else
        UART_APP UartAppConf = RS485_APP_NULL;
        if(0) {
      #if UART_APP_CONF_NUM   /* �п�������ģ����տ����õ��� */
		} else if(u8UartPort < UART_APP_CONF_NUM) {
			UartAppConf = (UART_APP)g_CommConf.u32RS485App[u8UartPort];	
      #endif
        } else {
            UartAppConf = cnst_UartAppConf[u8UartPort];
		}
	#endif
#endif

		/* ��������ѡ��RS485App�����RS485AppConf==RS485_APP_MAX���򲻸���APP */
		if(UartAppConf == RS485_APP_AUTO) {
			if(pUartComm->uTimer_WatchDog_ms >= 3000) {	/* �Զ�����£�����3�뼴��ʼ����--��ʱ���޸ģ�����mcgs��Ļ�����ʱ��Ͽ� */
				/* ����ͨѶ���ã��������ڶ�����  ����Ҫ���¿�ʼ����RS485App	 */
				if(pUartComm->bUartFuncTrySuc) {
					pUartComm->bUartFuncTrySuc = FALSE;
					pUartComm->u8RS485AppAutoTablePt = 0;
				} else {										/* �������Զ����� */
					pUartComm->u8RS485AppAutoTablePt++;
					if(pUartComm->u8RS485AppAutoTablePt >= sizeof(cnst_RS485AutoTable)/sizeof(UART_APP)) {
						pUartComm->u8RS485AppAutoTablePt = 0;
					}
				}
				pUartComm->UartApp = cnst_RS485AutoTable[pUartComm->u8RS485AppAutoTablePt];
			}
		} else if(UartAppConf < UART_APP_MAX) {
			pUartComm->UartApp = UartAppConf;
		}

		/* ��Ҫ���г�ʼ������������App */
		if(pUartComm->UartApp == RS485_APP_NULL) {
			Task_sleep(OS_TICK_KHz*200);
		} else if((pUartComm->uTimer_WatchDog_ms >= 5000)	/* �̶�App������ʱ5s--������Զ����,��ǰ����� */
		#if (DEVICE_TYPE == V5_YYT3)						/* �Ƿ�Appδ�ı䣬���ǲ����ʱ��� */
				|| ((pUartComm->UartApp == RS485_MODBUS_RTU_SLAVE) 
					&& (g_CommConf.u32RS485Baud_100bps[u8UartPort] != pUartComm->uLastBaud_100bps))
		#endif
				|| (pUartComm->LastUartApp != pUartComm->UartApp))
		{
			OFF_INDICATOR_RS485(u8UartPort);
			switch(pUartComm->UartApp) {
#if SUPPORT_SCREEN
				case UART_VIEW_TECH:
					TryViewTech(u8UartPort);
					break;
#endif
		#if (SUPPORT_V4_MASTER && SOFT_RUN1_TEST0)
				case RS485_YKFR1_XMS_9600:
					TryYkfR1XmsMaster(u8UartPort, UartAppConf);
					break;
				case RS485_YKFR2_38400:
					TryYkfR2Master(u8UartPort);
					break;
		#endif
				case RS485_MCGS_RTU_SLAVE:
		#if SOFT_RUN1_TEST0
					TryModbusRTUSlave(u8UartPort, 38400UL, 300);
		#else
					TryModbusRTUSlave(u8UartPort, 115200UL, 2000);
		#endif
					break;
		#if(DEVICE_TYPE == V5_YYT3)
				case RS485_DATA_RADIO_SUB:
					TryRadioDataSub(u8UartPort);
					break;
				case RS485_SERIAL_SERVER:
					TrySerialServer(u8UartPort);
					break;
				case RS485_MODBUS_RTU_SLAVE:
					TryModbusRTUSlave(u8UartPort, g_CommConf.u32RS485Baud_100bps[u8UartPort]*100UL, 200);
					pUartComm->uLastBaud_100bps = g_CommConf.u32RS485Baud_100bps[u8UartPort];
					break;
				case RS485_AIR_COMPRESSOR:
					TryAirCompressor(u8UartPort);
					break;
		#elif((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))
				case RS485_MODBUS_RTU_SLAVE:
					TryModbusRTUSlave(u8UartPort, 1200, 200);
					break;

		#elif(Is_YKF(DEVICE_TYPE) && SOFT_RUN1_TEST0)
				case RS485_V5YKF_PERIPHERAL:
					TryV5YkfPeripheral(u8UartPort);
					break;
		#endif
		#if SUPPORT_IEC101
				case RS485_IEC101:
					TryIEC101(u8UartPort);
					break;
		#endif
		#if SUPPORT_FLOW_METER
				case RS485_FLOW_METER:
					TryDrainMeter(u8UartPort);
					break;
				case RS485_BEIDOU_TERM:
				case RS485_BEIDOU_HUB:
					TryBeiDou(u8UartPort);
					break;
		#endif
		#if SOFT_RUN1_TEST0
				case RS485_SOFT_UPDATE:
					TrySubSoftUpdate(u8UartPort);
					break;
		#endif
				case RS485_DEBUG_PACK:
					TryDebugPack(u8UartPort);
					break;
		#if SUPPORT_TRIPHASE_METER
				case RS485_TRIPHASE_METER:
					TryTriPhaseMeter(u8UartPort);
					break;
		#endif
		#if SUPPORT_PGA460
				case RS485_WATERLV_PGA460:
					TryPga460(u8UartPort);
					break;
		#endif
		#if SUPPORT_WTRLVL_OPT_TX
				case UART_WATERLVL_OPT_TX:
					TryWtrLvlOptTx(u8UartPort);
					break;
		#endif
		#if SUPPORT_WTRLVL_OPT_RX
				case UART_WATERLVL_OPT_RX:
					TryWtrLvlOptRx(u8UartPort);
					break;
		#endif
		#if SUPPORT_PUMP
				case RS485_EXT_DIN_RELAY:
//					TryExtDinRelay(u8UartPort);
					break;
		#endif
		#if SUPPORT_GNL_MODBUS_METER
				case RS485_APP_GNLMODBUS_MASTER:
					TryGnlModbusMaster(u8UartPort, g_CommConf.u32RS485Baud_100bps[u8UartPort]*100UL);
					pUartComm->uLastBaud_100bps = g_CommConf.u32RS485Baud_100bps[u8UartPort];
					break;
		#endif
				default:
					Task_sleep(OS_TICK_KHz*20);
					break;
			}
		} else {	/* �Ծ���ĳ��App��ʽ����ͨѶ */
			switch(pUartComm->UartApp) {
#if SUPPORT_SCREEN
				case UART_VIEW_TECH:
					RunViewTech(u8UartPort);
					break;
#endif
		#if SUPPORT_V4_MASTER
				case RS485_YKFR1_XMS_9600:
					RunYkfR1XmsMaster(u8UartPort);
					break;
				case RS485_YKFR2_38400:
					RunYkfR2Master(u8UartPort);
					break;
		#endif
		#if(DEVICE_TYPE == V5_YYT3)
				case RS485_DATA_RADIO_SUB:
					RunRadioDataSub(u8UartPort);
					break;
				case RS485_SERIAL_SERVER:
					RunSerialServer(u8UartPort);
					break;
				case RS485_AIR_COMPRESSOR:
					RunAirCompressor(u8UartPort);
					break;
		#elif(Is_YKF(DEVICE_TYPE) && SOFT_RUN1_TEST0)
				case RS485_V5YKF_PERIPHERAL:
					RunV5YkfPeripheral(u8UartPort);
					break;
		#endif
				case RS485_MCGS_RTU_SLAVE:
				case RS485_MODBUS_RTU_SLAVE:
					RunModbusRTUSlave(u8UartPort, 100);
					break;
		#if SUPPORT_IEC101
				case RS485_IEC101:
					RunIEC101(u8UartPort);
					break;
		#endif
		#if SUPPORT_FLOW_METER
				case RS485_FLOW_METER:
					RunDrainMeter(u8UartPort);
					break;
				case RS485_BEIDOU_TERM:
					RunBeiDouTerm(u8UartPort);
					break;
			#if SUPPORT_BEIDOU_HUB
				case RS485_BEIDOU_HUB:
					RunBeiDouHub(u8UartPort);
					break;
			#endif
		#endif
		#if SOFT_RUN1_TEST0
				case RS485_SOFT_UPDATE:
					RunSubSoftUpdate(u8UartPort);
					break;
		#endif
				case RS485_DEBUG_PACK:
					RunDebugPack(u8UartPort);
					break;
		#if SUPPORT_TRIPHASE_METER
				case RS485_TRIPHASE_METER:
					RunTriPhaseMeter(u8UartPort);
					break;
		#endif
		#if SUPPORT_PGA460
				case RS485_WATERLV_PGA460:
					RunPga460(u8UartPort);
					break;
		#endif
		#if SUPPORT_WTRLVL_OPT_TX
				case UART_WATERLVL_OPT_TX:
					RunWtrLvlOptTx(u8UartPort);
					break;
		#endif
		#if SUPPORT_WTRLVL_OPT_RX
				case UART_WATERLVL_OPT_RX:
					RunWtrLvlOptRx(u8UartPort);
					break;
		#endif
		#if SUPPORT_PUMP
				case RS485_EXT_DIN_RELAY:
					//RunExtDinRelay(u8UartPort);
					break;
		#endif
		#if SUPPORT_GNL_MODBUS_METER
				case RS485_APP_GNLMODBUS_MASTER:
					RunGnlModbusMaster(u8UartPort);
					break;
		#endif
				default:
					pUartComm->uTimer_WatchDog_ms = 0xFFFF;
					pUartComm->bUartFuncTrySuc = FALSE;
					Task_sleep(OS_TICK_KHz*1000);
					break;
			}
		}		
		pUartComm->LastUartApp = pUartComm->UartApp;
	}
}

/*==========================================================================
| Description	: 
| G/Out var		:
| Author		: Wang Renfei			Date	: 2015-9-27
\=========================================================================*/
void DrvUartTick_1KHz(void)
{
	int16 i;

	/* ͨѶ����ʱ�� */
	for(i = 0; i < MAX_UART_NUM; i++) {
		if(g_UartComm[i].uTimer_WatchDog_ms < 0xFFFF) {
			g_UartComm[i].uTimer_WatchDog_ms++;
		}
		if(g_UartComm[i].uTmr_Run_ms) {
			g_UartComm[i].uTmr_Run_ms--;
		}
	}

	/* ά����ťkey */
#if CTR_BTN_TOTAL_NUM
	for(i = 0; i < CTR_BTN_TOTAL_NUM; i++) {
	    if(g_MiscCommData.u8Tmr_PnlCtrBtn_Valid_ms[i]) {
    	    g_MiscCommData.u8Tmr_PnlCtrBtn_Valid_ms[i]--;
        }
	}
#endif

#if SUPPORT_V4_MASTER
	DrvV4MasterTick_1KHz();
#endif
}


/*==========================================================================
| ����ΪModbusRTUͨѶ���֣���ͨ�� ModbusSlave, ����ˮλ����(ModbusMaster), �Լ���������(TxDataForModbusRTU)
\=========================================================================*/
typedef enum {
	MODBUS_READ_REG 		= 3,
	MODBUS_READ_IN_REG		= 4,
	MODBUS_WRITE_REG		= 6,
	MODBUS_WRITE_REGS		= 16
}MODBUS_COMMAND_VAR;
#define MON_ID_PRDC						0xFFFF
#define MODBUS_NODE_ADD_UI				0xFF

/*==========================================================================
| Description	: ModbusRTUͨѶͨ�ú���
	TxDataForModbusRTU()			: ����ModbusRTUģʽ���CRC������������
	CheckRxBufForModbusRTUMaster()	: ��ModbusRTU������ʽ ���RxBuf�Ƿ���ȷ: ����CRC, �Լ�ͷ����Ϣ
	RdDataFrModbusRTU()				: ModbusRTU���� ��Modbus�����豸��ȡ����
	WrDataToModbusRTU()				: ModbusRTU���� ��Modbus�����豸д������
	WaitDataFromModbusRTU()			: ModbusRTU�ӻ� �ȴ�Modbus���������ĺϸ�ָ��(���CRCУ��)
| G/Out var		:
| Author		: Wang Renfei			Date	: 2017-3-25
\=========================================================================*/
void TxDataForModbusRTU(uint8 u8UartPort, uint8* pTxBuf)
{
	uint8* pU8TRBufStart = g_UartComm[u8UartPort].u8TRBuf;	/* ��ָ�뷽ʽ���� */

	/* CRC���� */
	uint16 uCRC = CalCRC16ForModbus(pU8TRBufStart, pTxBuf - pU8TRBufStart);
	*pTxBuf++ = uCRC/0x100;
	*pTxBuf++ = uCRC&0xFF;

	/* �������� */
	WriteToUart(u8UartPort, pTxBuf - pU8TRBufStart);
}

UART_ACS_RES CheckCrcForModbusRTU(uint8* pRxBufStart, uint16 uRxNum)
{
    if(uRxNum < 3) {
        return UART_NUM_NOT_ENOUGH;
    } else if(CalCRC16ForModbus(pRxBufStart, uRxNum - 2) != pRxBufStart[uRxNum - 2]*0x100 + pRxBufStart[uRxNum - 1]) {
        return UART_CRC_ERR;
    } else {
		return UART_ACS_SUC;
	}
}

UART_ACS_RES CheckRxBufForModbusRTUMaster(UART_COMM* pUartComm, uint8 u8DevAdd, uint8 u8ModCmd, uint16 uReaddRegLen)
{
    UART_ACS_RES AcsRes;
	if((AcsRes = CheckCrcForModbusRTU(pUartComm->u8TRBuf, pUartComm->uRxBufPt)) <= 0) {
	    return AcsRes;
	} else if(pUartComm->u8TRBuf[0] != u8DevAdd) {
	    return UART_MOD_DEVADD_NEQ;
	} else if(pUartComm->u8TRBuf[1] != u8ModCmd) {
	    return UART_MOD_CMD_NEQ;
	} else {
		if((1 <=  u8ModCmd) && (u8ModCmd <= 4)) {
			/* ����������Ӧ�÷��ص��ֽ����� */
			if(u8ModCmd <= 2) {
				if(uReaddRegLen % 8 == 0) {
					uReaddRegLen = uReaddRegLen/8;
				} else {
					uReaddRegLen = uReaddRegLen/8 + 1;
				}
			} else {
				uReaddRegLen = uReaddRegLen*2;
			}
			
			/* У�� */
			if((pUartComm->u8TRBuf[2] + 5 == pUartComm->uRxBufPt) && (pUartComm->u8TRBuf[2] == uReaddRegLen)) {
				return UART_ACS_SUC;
			}
		} else if(u8ModCmd == 0x10) {
			if((pUartComm->uRxBufPt == 8) || ((uint16)pUartComm->u8TRBuf[2]*0x100 + pUartComm->u8TRBuf[3] == MODADD_UPDATE_START)) {
				return UART_ACS_SUC;
			}
		} else if(u8ModCmd == 6) {
			if(pUartComm->uRxBufPt == 8) {
				return UART_ACS_SUC;
			}
		}
        return UART_MOD_RD_FAIL;
	}
}

UART_ACS_RES RdDataFrModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms, uint8 u8DevAdd, uint16 uRegAdd, uint16 uRegLen, uint16* pU16Dat, MOD_ACS_TYPE ModAcsType)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];

	/* ���ɲ�ѯ֡ */
	uint8* pTxBuf = pUartComm->u8TRBuf;
	*pTxBuf++ = u8DevAdd;
	uint8 u8ModCmd;
	if(ModAcsType == MOD_RD_01B) {
		u8ModCmd = 0x01;
	} else if(ModAcsType == MOD_RD_02B) {
		u8ModCmd = 0x02;
	} else if((ModAcsType == MOD_RD_03B) || (ModAcsType == MOD_RD_03W) || (ModAcsType == MOD_RD_03D) || (ModAcsType == MOD_RD_03D2143)) {
		u8ModCmd = 0x03;
	} else {
		u8ModCmd = 0x04;
	}
	*pTxBuf++ = u8ModCmd;
	*pTxBuf++ = uRegAdd/0x100;
	*pTxBuf++ = uRegAdd&0xFF;
	*pTxBuf++ = uRegLen/0x100;
	*pTxBuf++ = uRegLen&0xFF;

	/* ���ݷ��� */
	TxDataForModbusRTU(u8UartPort, pTxBuf);

	/* �������ݲ����д��� */
	UART_ACS_RES AcsRes;
	if(!ReadFromUart(u8UartPort, uMaxDelay_ms)) {
	    return UART_RD_TIME_OUT;
	} else if((AcsRes = CheckRxBufForModbusRTUMaster(pUartComm, u8DevAdd, u8ModCmd, uRegLen)) <= 0) {
	    return AcsRes;
	} else {
		pUartComm->uTimer_WatchDog_ms = 0;		/* ͨѶ�ɹ� */
		pUartComm->bUartFuncTrySuc = TRUE;

		if(pU16Dat != NULL) {	/* ���ݷ�ָ��� */
			/* ���ݿ��� */
			uint8 u8RcvByteLen = CalMinU16(pUartComm->u8TRBuf[2], uRegLen*2);
			uint8* pRxBuf = pUartComm->u8TRBuf + 3;
			uint8* pU8Dat;
			switch(ModAcsType) {
				case MOD_RD_01B:
				case MOD_RD_02B:
					memcpy(pU16Dat, (const void*)pRxBuf, u8RcvByteLen);
					break;
			
				case MOD_RD_03W:
				case MOD_RD_04W:
					for( ; u8RcvByteLen >= 2; u8RcvByteLen -= 2) {
						*pU16Dat = pRxBuf[0]*0x100 + pRxBuf[1];
						pU16Dat++;
						pRxBuf += 2;
					}
					break;
					
				case MOD_RD_03D:
				case MOD_RD_04D:
					for( ; u8RcvByteLen >= 4; u8RcvByteLen -= 4) {
						*((uint32*)pU16Dat) = (((uint32)pRxBuf[0])<<24) + (((uint32)pRxBuf[1])<<16) + (((uint32)pRxBuf[2])<<8) + (((uint32)pRxBuf[3])<<0);
						pU16Dat += 2;
						pRxBuf += 4;
					}
					break;

				case MOD_RD_04D_I2F:	/* ��ȡ��������int32��ת��float32����buf */
					for( ; u8RcvByteLen >= 4; u8RcvByteLen -= 4) {
						int32 i32Dat = (((uint32)pRxBuf[0])<<24) + (((uint32)pRxBuf[1])<<16) + (((uint32)pRxBuf[2])<<8) + (((uint32)pRxBuf[3])<<0);
						 *((float32*)pU16Dat) = i32Dat;
						pU16Dat += 2;
						pRxBuf += 4;
					}
					break;	

				case MOD_RD_03D2143:
				case MOD_RD_04D2143:
					for( ; u8RcvByteLen >= 2; u8RcvByteLen -= 4) {
						*((uint32*)pU16Dat) = (((uint32)pRxBuf[2])<<24) + (((uint32)pRxBuf[3])<<16) + (((uint32)pRxBuf[0])<<8) + (((uint32)pRxBuf[1])<<0);
						pU16Dat += 2;
						pRxBuf += 4;
					}
					break;

				case MOD_RD_03L:
				case MOD_RD_04L:
					pU8Dat = (uint8*)pU16Dat;
					for( ; u8RcvByteLen >= 8; u8RcvByteLen -= 8) {
						pU8Dat[7] = *pRxBuf++;
						pU8Dat[6] = *pRxBuf++;
						pU8Dat[5] = *pRxBuf++;
						pU8Dat[4] = *pRxBuf++;
						pU8Dat[3] = *pRxBuf++;
						pU8Dat[2] = *pRxBuf++;
						pU8Dat[1] = *pRxBuf++;
						pU8Dat[0] = *pRxBuf++;
						pU8Dat += 8;
					}
					break;

				default:
					memcpy(pU16Dat, (const void*)pRxBuf, u8RcvByteLen);
					break;
			}
		}
		
		return UART_ACS_SUC;
	}
}

UART_ACS_RES WrDataToModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms, uint8 u8DevAdd, uint16 uRegAdd, uint16 uRegLen, uint16* pU16Dat, MOD_ACS_TYPE ModAcsType)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	uint8* pTxBuf = pUartComm->u8TRBuf;
	uint8 u8ModCmd = 0;

	/* ������ */
	if((uRegLen == 0)
		|| ((ModAcsType == MOD_WR_10W) && (uRegLen > (UART_TR_BUF_BLEN-10)/2))
		|| ((ModAcsType == MOD_WR_10D) && (uRegLen > (UART_TR_BUF_BLEN-10)/4)))
	{
		return UART_ACS_SUC;		/* ��Ȼ����Χ�����ǲ���ΪͨѶʧ�� */

	/* ���ɷ�����Ϣ֡ */
	} else if(uRegLen == 1) {		/* ���������0x05 0x06 */
		if(ModAcsType == MOD_WR_05B) {          /* 05ָ��д  д������Ȧֵ */
			u8ModCmd = 0x05;
		} else if(ModAcsType == MOD_WR_06W) {   /* 06ָ��д  д�����Ĵ���ֵ,16bit���,��2-1˳�� */
            u8ModCmd = 0x06;
		} else {
		    return UART_MOD_WR_FAIL;
		}
		*pTxBuf++ = u8DevAdd;
		*pTxBuf++ = u8ModCmd;
		*pTxBuf++ = uRegAdd/0x100;
		*pTxBuf++ = uRegAdd&0xFF;
		*pTxBuf++ = (*pU16Dat)/0x100;
		*pTxBuf++ = (*pU16Dat)&0xFF;
		
	} else {    /* ����ʾ���Ϳ�����0x0F 0x10  */
		switch(ModAcsType) {
			case MOD_WR_0FB:			/* 0Fָ��д  д�����Ȧֵ */
				u8ModCmd = 0x0F;
				break;

			case MOD_WR_10B:			/* 10ָ��д  д����Ĵ���ֵ,�������ֽ���,��˾�ڲ��ɶ�ʹ�� */
			case MOD_WR_10W:			/* 10ָ��д  д����Ĵ���ֵ,16bit���,��2-1˳�� */
			case MOD_WR_10D:			/* 10ָ��д  д����Ĵ���ֵ,��4-3-2-1˳��, ����ȫ���� */
			case MOD_WR_10D2143:		/* 10ָ��д  д����Ĵ���ֵ,TODO:��2-1-4-3˳��,��ʱ��֧�� */
				u8ModCmd = 0x10;
				break;

			default:
                return UART_MOD_WR_FAIL;
		}
		/* �����0x0F 0x10��Ӧ���� */
		*pTxBuf++ = u8DevAdd;
		*pTxBuf++ = u8ModCmd;
		*pTxBuf++ = uRegAdd/0x100;
		*pTxBuf++ = uRegAdd&0xFF;
		*pTxBuf++ = uRegLen/0x100;
		*pTxBuf++ = uRegLen&0xFF;
		*pTxBuf++ = uRegLen*2;
		if(ModAcsType == MOD_WR_10W) {
			CopyL16DatToTxBuf(&pTxBuf, (uint8*)pU16Dat, uRegLen, uRegLen);	/* ���������֤��ǰ��������黷�� */
		} else if(ModAcsType == MOD_WR_10D) {
			CopyL32DatToTxBuf(&pTxBuf, (uint8*)pU16Dat, uRegLen, uRegLen);	/* ���������֤��ǰ��������黷�� */
		} else {	/* MOD_WR_0FW / MOD_WR_10B / MOD_WR_10D2143*/
			memcpy(pTxBuf, (const void*)pU16Dat, uRegLen*2);
			pTxBuf += uRegLen*2;
		}
	}

	/* ���ݷ��� */
	TxDataForModbusRTU(u8UartPort, pTxBuf);

	/* �������ݲ����д��� */
    UART_ACS_RES AcsRes = UART_ACS_NULL;
	if(u8DevAdd == 0) {			/* �㲥���ݣ�û�з������ݣ���������Ҫ�ȴ���ǰ֡���� */
		Task_sleep((4*pUartComm->uTROneByteTime_us*OS_TICK_KHz + 500)/1000UL);
		pUartComm->bUartFuncTrySuc = TRUE;
		return UART_ACS_SUC;
	} else if(!ReadFromUart(u8UartPort, uMaxDelay_ms)) {
		return UART_RD_TIME_OUT;
	} else if((AcsRes = CheckRxBufForModbusRTUMaster(pUartComm, u8DevAdd, u8ModCmd, 0)) <= 0) {
        return AcsRes;
	} else {
		pUartComm->uTimer_WatchDog_ms = 0; 	/* ͨѶ�ɹ� */
		pUartComm->bUartFuncTrySuc = TRUE;
		return UART_ACS_SUC;
	}
}

UART_ACS_RES WaitDataFromModbusRTU(uint8 u8UartPort, uint16 uMaxDelay_ms)
{
    UART_ACS_RES AcsRes;
	if(!ReadFromUart(u8UartPort, uMaxDelay_ms)) {
	    return UART_RD_TIME_OUT;
	} else if(g_UartComm[u8UartPort].uRxBufPt < 8) {
	    return UART_NUM_NOT_ENOUGH;
	} else if((AcsRes = CheckCrcForModbusRTU(g_UartComm[u8UartPort].u8TRBuf, g_UartComm[u8UartPort].uRxBufPt)) <= 0) {
	    return AcsRes;
	} else {
		g_UartComm[u8UartPort].uTimer_WatchDog_ms = 0;
		g_UartComm[u8UartPort].bUartFuncTrySuc = TRUE;
		
		return UART_ACS_SUC;
	}
}

/*==========================================================================
| ����Ϊ���� ModbusRTU ����APPģ�麯��
\=========================================================================*/
/*==========================================================================
| Description	: modbusͨѶ�ӻ�������, ���� TryModbusRTUSlave, RunModbusRTUSlave��������
| G/Out var 	:
| Author		: Wang Renfei			Date	: 2017-8-12
\=========================================================================*/
void TryModbusRTUSlave(uint8 u8UartPort, uint32 u32BaudRate, uint16 uMaxDelay_ms)
{
	OpenUartComm(u8UartPort, u32BaudRate, 0, 0);

	RunModbusRTUSlave(u8UartPort, uMaxDelay_ms);
}

void RunModbusRTUSlave(uint8 u8UartPort, uint16 uMaxDelay_ms)
{
	/* �������� */
	if(WaitDataFromModbusRTU(u8UartPort, uMaxDelay_ms) > 0) {
		ON_INDICATOR_RS485(u8UartPort);
	#if(Is_YKF(DEVICE_TYPE) || (DEVICE_TYPE == V5_YYB) || (DEVICE_TYPE == V5_YYD) || (DEVICE_TYPE == V5_YYD2) || (DEVICE_TYPE == V5_YYG))
		RespondModbus(u8UartPort, g_CommConf.u32DevAdd);
	#elif ((DEVICE_TYPE == V5_YBT2) || (DEVICE_TYPE == V5_YBT3) || (DEVICE_TYPE == V5_YBT4))
		RespondModbus(u8UartPort, 2);
	#elif(DEVICE_TYPE == V5_YYT3)
		RespondModbus(u8UartPort, g_CommConf.u32RS485Add[u8UartPort]);
	#elif((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))
		RespondModbus(u8UartPort, g_CommConf.uRS485Add_SendIntv);
	#else
		RespondModbus(u8UartPort, 0xFF);
	#endif
	} else {
		OFF_INDICATOR_RS485(u8UartPort);
	}
}

/*==========================================================================
| Description	: ModbusͨѶ��Ϣ��Ӧ��������ӦModbus RTU/TCPģʽ ָ��
				 ������У����Ϣ֡(��CRC�Ƿ���ȷ),ִ�б���Ӧǰ��Ĭ����Ϣ��ȷ
| G/Out var 	: uModbusPort < UART�˿���(MAX_UART_NUM)������Ϊ��ModbusRTU��������Ϊ��ModbusTCP
| Author		: Wang Renfei			Date	: 2015-10-22
\=========================================================================*/
void RespondModbus(uint8 u8ModbusPort, uint8 u8DevAdd)
{
	/* Ԥ���� */
	UART_COMM* pUartComm;
	uint8* pModBufStart;		/* ָ��modbus��Ϣ֡��ʼ�����豸��ַ */
	uint16 uMsgNodeAddr;
	uint16 uRxNum;
	if(u8ModbusPort < MAX_UART_NUM) { 									/* ��Ϊ��ModbusRTU�ĵ��� */
		uMsgNodeAddr = g_UartComm[u8ModbusPort].u8TRBuf[0];
		pUartComm = &g_UartComm[u8ModbusPort];
		uRxNum = pUartComm->uRxBufPt;
		pModBufStart = pUartComm->u8TRBuf;
		if(uRxNum < 6) {		/* ���ȼ�飬�豸��ַ,ָ��,���ݵ�ַ,�������ݻ������ݳ��� */
			return;
		}
#if SUPPORT_MODBUS_TCP
	} else if(u8ModbusPort < MAX_UART_NUM + MODBUS_TCP_COMM_MAX_NUM) {	/* ��Ϊ��ModbusTCP�ĵ��� */
		u8ModbusPort -= MAX_UART_NUM;
		uMsgNodeAddr = g_ModbusTcpComm[u8ModbusPort].u8TRBuf[6];
		pUartComm = NULL;
		uRxNum = g_ModbusTcpComm[u8ModbusPort].uRxBufPt;
		g_ModbusTcpComm[u8ModbusPort].uTxBufPt = 0;
		pModBufStart = g_ModbusTcpComm[u8ModbusPort].u8TRBuf + 6;			/* ָ��NodeAddress */
		if(uRxNum < 6 + 6) {	/* ���ȼ�飬modbusTCP��ʼ6��byte + �豸��ַ,ָ��,���ݵ�ַ,�������ݻ������ݳ��� */
			return;
		}
#endif
	} else {
		return;
	}
	
	if(uMsgNodeAddr && (uMsgNodeAddr != u8DevAdd) && (uMsgNodeAddr != MODBUS_NODE_ADD_UI)) {
#if (SUPPORT_V4_MASTER || SUPPORT_V5_MASTER)
		if(u8ModbusPort < MAX_UART_NUM) {
			RespondModbusRTUForOthers(u8ModbusPort);
		}
#endif
		return;
	}
	uint8* pTRBuf = pModBufStart + 1;			/* ָ��ModbusCmd */
	uint8* pTxBufEnd = pModBufStart + UART_TR_BUF_BLEN;

	/* ����� */
	int16 iRegNum;
	uint16 uReg;
	uint16 uRegOff;
	uint16 uProcRegNum;
	BOOL bWriteFlashReq = FALSE;
	MODBUS_COMMAND_VAR ModbusCmd = (MODBUS_COMMAND_VAR)pTRBuf[0];
	uint16 uRegAddr = pTRBuf[1]*0x100 + pTRBuf[2];		/* �Ѿ�ȷ���ˣ���ôд�ǿ��Ե� */
	switch(ModbusCmd) {
		case MODBUS_READ_REG:			/* ���ڶ�ȡ����ֵ */
		case MODBUS_READ_IN_REG:		/* ���ڶ�ȡ��������ֵ */
			iRegNum = pTRBuf[3]*0x100 + pTRBuf[4];	/* �Ѿ�ȷ���ˣ���ôд�ǿ��Ե� */
			
			if(iRegNum > (pTxBufEnd - pTRBuf - 4)/2) {				/* ��������ݳ��ȳ���Buf���� */
				*pTRBuf++ |= 0x80;									/* ָ������ */
			} else {												/* �����ظ� */
				pTRBuf++;											/* ������ */
				pTRBuf++;											/* Ԥ�����ݳ��� */
				
				while(iRegNum > 0) { 	/* �������� */
					uProcRegNum = 0;
					uReg = 0;
					if(uRegAddr < MODADD_SIM_XM_END) {
						if((uRegAddr == 0) && (ModbusCmd == MODBUS_READ_IN_REG)) {
					#if(SUPPORT_NET && ((DEVICE_TYPE == V5_YBT2) || (DEVICE_TYPE == V5_YBT3) || (DEVICE_TYPE == V5_YBT4)))
							if(g_TcpIPComm.RmtSenFromMqtt.fSenData[0] < -0.05) {
								uMsgNodeAddr = 0;		/* �ú�����β�����ͻظ�����ɡ����ߡ� */
							} else {
								uReg = F32ToU16(g_TcpIPComm.RmtSenFromMqtt.fSenData[0]*g_YBTConf.fWaterSenRange*1000);
							}
					#elif((DEVICE_TYPE == V5_YBU2) || (DEVICE_TYPE == V5_YBU3) || (DEVICE_TYPE == V5_YBU4))	/* ģ��YBU��AD���̽ӿ� */
							uReg = F32ToU16((g_AnaRes.fLocalSensor + 0.25f) * 24000.0f);
					#elif(DEVICE_TYPE == V5_YYT3)	/* YYT3��ģ��ˮλ�Ǳ��ʱ����Ҫ��Ӧ��ȡ����ֵ */
							uReg = F32ToU16(g_AnaRes.LogicWtrLvl[LOGIC_WTR_TURB].fWtrLvl * 1000.0f);
						} else if((0 < uRegAddr) && (uRegAddr <= CONF_FOR_YKF_NUM) && (ModbusCmd == MODBUS_READ_REG)) {
							uReg = F32ToU16(g_YYTConf.fConfForYKF[uRegAddr - 1] * 1000.0f);
					#endif
						}
					} else if((MODADD_MISC_START <= uRegAddr) && (uRegAddr < MODADD_MISC_END)) {
						if((MODADD_DEVICE_START <= uRegAddr) && (uRegAddr < MODADD_DEVICE_END)) {
							if(uRegAddr == MODADD_DEVICE_TYPE) {
								uReg = DEVICE_TYPE;
							} else if(uRegAddr == MODADD_DEVICE_HARD_VER) {
								uReg = DEVICE_TYPE;
							} else if(uRegAddr == MODADD_DEVICE_SOFT_VER) {
								uReg = SOFTWARE_VER;
							} else if(uRegAddr == MODADD_DEVICE_SERNOL) {
								uReg = g_Sys.SerialNo.u32Dat&0xFFFF;
							} else if(uRegAddr == MODADD_DEVICE_SERNOH) {
								uReg = g_Sys.SerialNo.u32Dat/0x10000;
							}
						} else if((MODADD_CTR_START <= uRegAddr) && (uRegAddr < MODADD_CTR_END)) {
							uRegOff = uRegAddr - MODADD_CTR_START;
							uProcRegNum = CopyL32DatToTxBuf(&pTRBuf, (uint8*)(&g_Ctr) + uRegOff*2, iRegNum, sizeof(g_Ctr)/2 - uRegOff);
#if SUPPORT_SCREEN
						} else if((MODADD_MCGS_ITEMPAGE_START <= uRegAddr) && (uRegAddr < MODADD_MCGS_ITEMPAGE_END)) {
							if((pUartComm != NULL)	&& (uRegAddr == MODADD_MCGS_ITEMPAGE_START)
								&& (pTRBuf + MCGS_ITEM_PAGE_ACK_MAX_BLEN < pTxBufEnd))
							{
								FillItemPageAck_MCGS(&pUartComm->ItemPageReq, pTRBuf);
								pTRBuf += MCGS_ITEM_PAGE_LINE_WLEN*2;
								uProcRegNum = MCGS_ITEM_PAGE_LINE_WLEN;
							}
#endif
						} else if((MODADD_SYS_OPR_START <= uRegAddr) && (uRegAddr < MODADD_SYS_OPR_END)) {
							uRegOff = uRegAddr - MODADD_SYS_OPR_START;
							//uProcRegNum = CopyL32DatToTxBuf(&pTRBuf, (uint8*)g_DataAcsIntf.i8CheckGlbConf + uRegOff*2, RegNum, MAX_SYS_OPR_NUM);
						}
					} else if((MODADD_DEBUG_CHN_START  <= uRegAddr) && (uRegAddr < MODADD_DEBUG_CHN_END)) {
						CopyDebugChnToTxBuf(&pTRBuf, uRegAddr - MODADD_DEBUG_CHN_START);
						uProcRegNum = iRegNum;
				#if MAX_MSG_TYPE
                    } else if((MODADD_MSG_START <= uRegAddr) && (uRegAddr < MODADD_MSG_END)) {  /* Msg���� */
                        uProcRegNum = TxMsgForModbus(&pTRBuf, iRegNum);
				#endif
					} else if((MODADD_MSR_RES_START <= uRegAddr) && (uRegAddr < MODADD_MSR_RES_END)) {	/* g_MsrRes���� */
						uRegOff = uRegAddr - MODADD_MSR_RES_START;
						uProcRegNum = CopyL32DatToTxBuf(&pTRBuf, (uint8*)(&g_MsrRes) + uRegOff*2, iRegNum, sizeof(g_MsrRes)/2 - uRegOff);
					} else if((MODADD_ANA_RES_START <= uRegAddr) && (uRegAddr < MODADD_ANA_RES_END)) { 	/* g_CtrObj���� */
						uRegOff = uRegAddr - MODADD_ANA_RES_START;
						uProcRegNum = CopyL32DatToTxBuf(&pTRBuf, (uint8*)(&g_AnaRes) + uRegOff*2, iRegNum, sizeof(g_AnaRes)/2 - uRegOff);
					} else if((MODADD_CODE_TEST_START <= uRegAddr) && (uRegAddr < MODADD_CODE_TEST_END)) {
						if(uRegAddr < MODADD_CODE_TEST_U32_END) {
							uProcRegNum = CopyL32DatToTxBuf(&pTRBuf, (uint8*)(&g_CodeTest.u32Val), iRegNum, CODE_TEST_UINT32_NUM*2);
						} else {
							uProcRegNum = CopyL16DatToTxBuf(&pTRBuf, (uint8*)(&g_CodeTest.uVal), iRegNum, CODE_TEST_UINT16_NUM);
						}
					} else if((MODADD_MASTER_START <= uRegAddr) && (uRegAddr < MODADD_MASTER_END)) {
						uProcRegNum = CopyL16DatToTxBuf(&pTRBuf, (uint8*)(&g_PubSoftAuthCtr), iRegNum, sizeof(g_PubSoftAuthCtr)/2);
					}

					if(uProcRegNum) {
						uRegAddr += uProcRegNum;
						iRegNum -= uProcRegNum;
					} else {
						*pTRBuf++ = uReg/0x100;
						*pTRBuf++ = uReg&0xFF;
						uRegAddr++;
						iRegNum--;
					}
				}
				
				pModBufStart[2] = pTRBuf - pModBufStart - 3;	/* ������ݳ��� */
			}
			break;

		case MODBUS_WRITE_REG:
		case MODBUS_WRITE_REGS:
			/* ָ��� */
			if(ModbusCmd == MODBUS_WRITE_REG) {
				iRegNum = 1;
				pTRBuf += 3;			/* Rxָ��ָ������ */
			} else if(ModbusCmd == MODBUS_WRITE_REGS) {
				iRegNum = pTRBuf[3]*0x100 + pTRBuf[4];							/* �Ѿ�ȷ���ˣ���ôд�ǿ��Ե� */
				if((iRegNum*2 != pTRBuf[5]) 	/* ���ȴ����� */
					|| ((pUartComm != NULL) && (iRegNum*2 + 9 != uRxNum))		/* modbusRTU��CRC */
					|| ((pUartComm == NULL) && (iRegNum*2 + 13 != uRxNum)))		/* modbusTCP��CRC */
				{
					iRegNum = 0;
					pTRBuf[0] |= 0x80;
				}
				pTRBuf += 6;			/* Rxָ��ָ������ */
			}
	
			while(iRegNum > 0) {
				uReg = pTRBuf[0]*0x100 + pTRBuf[1];
				uProcRegNum = 0;

				if((MODADD_MCGS_ITEMPAGE_START <= uRegAddr) && (uRegAddr < MODADD_MCGS_ITEMPAGE_END)) {
					if((uRegAddr == MODADD_MCGS_ITEMPAGE_START) && (iRegNum >= 3)) { /* ������RS485ͨѶ�˿ڵ��ò��� */
						pTRBuf[iRegNum*2] = 0;	/* �����ַ��������� */
						uint32 u32ItemNo = (uint32)pTRBuf[2]*0x1000000UL + (uint32)pTRBuf[3]*0x10000UL
											+ (uint32)pTRBuf[4]*0x100UL + (uint32)pTRBuf[5];
						DATA_PAGE DataPage = (DATA_PAGE)pTRBuf[1];
						pTRBuf += 6;
						if(pUartComm == NULL) {
							ProcItemPageInstr(DATA_USER_MCGS, DataPage, u32ItemNo, &pTRBuf, pTRBuf + (iRegNum-3)*2, &g_MiscCommData.ItemPageReq);
						} else {
							ProcItemPageInstr(DATA_USER_MCGS, DataPage, u32ItemNo, &pTRBuf, pTRBuf + (iRegNum-3)*2, &pUartComm->ItemPageReq);
						}
						uProcRegNum = iRegNum;
					}
				} else if((MODADD_MON_CMD_START <= uRegAddr) && (uRegAddr < MODADD_MON_CMD_END)) {
				    uProcRegNum = WriteMosbusMon(uRegAddr, pTRBuf, iRegNum);
				    pTRBuf += uProcRegNum*2;
			#if SOFT_RUN1_TEST0
				} else if((MODADD_MASTER_START <= uRegAddr) && (uRegAddr < MODADD_MASTER_END)) {
					ProcVersionMasterBCD(pTRBuf, iRegNum);
					uProcRegNum = iRegNum;
			#else
				} else if((MODADD_UPDATE_START <= uRegAddr) && (uRegAddr < MODADD_UPDATE_END)) {
					if(iRegNum*2 == sizeof(FLASH_REQ)) { 	/* дȫ��FlashReq */
						FLASH_REQ FlashReq;
						memcpy(&FlashReq, (const void*)pTRBuf, iRegNum*2);		/* ������ֽ�˳�������modbus��ʽ */
						pTRBuf = pModBufStart + 8;	/* ����32bit���� */
						pTRBuf += PubSoftware(&FlashReq, (uint32*)pTRBuf, RS485_TR_FLASH_PACK_BLEN, 0);
						uProcRegNum = iRegNum;
						bWriteFlashReq = TRUE;
					}
			#endif
				}

				if(uProcRegNum) {
					uRegAddr += uProcRegNum;
					iRegNum -= uProcRegNum;
				} else {
					uRegAddr++;
					iRegNum--;
					pTRBuf += 2;
				}
			}
			if(!bWriteFlashReq) {	/* �������дFlashReq, ������06����0x10ָ��ظ����ݳ��ȶ���6 */
				pTRBuf = pModBufStart + 6;
			}
			break;

		default:
			break;
	}
	
	/* ��Ϣ���Ϳ��� */
	if(uMsgNodeAddr) {	/* ��ַ��0�������ǹ㲥��ַ */
		if(pUartComm != NULL) {	/* modbusRTU */
			TxDataForModbusRTU(u8ModbusPort, pTRBuf);
		} else {					/* modbusTCP */
#if SUPPORT_MODBUS_TCP
			g_ModbusTcpComm[u8ModbusPort].uTxBufPt = pTRBuf - g_ModbusTcpComm[u8ModbusPort].u8TRBuf;
			g_ModbusTcpComm[u8ModbusPort].u8TRBuf[5] = g_ModbusTcpComm[u8ModbusPort].uTxBufPt - 6;
#endif
		}
	}
}

extern uint16 ProcLocalCmdInstr(uint8* pRxBuf, uint16 uRegOff, int16 RegNum);
uint16 WriteMosbusMon(uint16 uRegAddr, uint8* pRxBuf, uint16 uRegLen)
{
    uint16 uProcRegNum = 0;
	if((MODADD_MON_CMD_START <= uRegAddr) && (uRegAddr <= MODADD_MON_CMD_END)) {
        uint16 uRegOff = uRegAddr - MODADD_MON_CMD_START;
        if(uRegOff < 4) {           /* 60000~60003�����в�Ʒ�������� */
        #if CTR_BTN_MODTCP_NUM
            if(uRegOff == 0) {      /* 60000����Ļ��ť */
				uint16 PnlCtrBtn = pRxBuf[0]*0x100 + pRxBuf[1];
				int8 i;
				for(i = 0; i < CTR_BTN_MODTCP_NUM; i++) {
				    if(PnlCtrBtn & (1<<i)) {
				        g_MiscCommData.u8Tmr_PnlCtrBtn_Valid_ms[i] = KEY_VALID_TIME_ms;
				    }
				}
            }
        #endif
            uProcRegNum = 4 - uRegOff;
            if(uProcRegNum < uRegLen) {
                uProcRegNum = uRegLen;
            }
        } else {                    /* ÿ����Ʒ��Ҫ���Լ���ProcLocalCmdInstr() */
            uProcRegNum = ProcLocalCmdInstr(pRxBuf, uRegOff - 4, uRegLen);
        }
    }
    return uProcRegNum;
}

#if (SUPPORT_V4_MASTER || SUPPORT_V5_MASTER)
/*==========================================================================
| ������Modbus���������豸����(������V4Master, V5Master)��ͨѶ�������������º�����
RespondModbusTCPForOthers(): ModbusTCP
RespondModbusRTUForOthers(): ModbusRTU
RespondModbusForV4()   : �ṩ��RespondModbusTCPForV4(), RespondModbusRTUForOthers() ���ã�����豸�������
\=========================================================================*/
typedef enum {
	RESPOND_OTHERS_MODBUS_RES_SUC = 0,		/* �ɹ� */
	RESPOND_OTHERS_MODBUS_RES_V4_NO_DEV,	/* V4�쳣: ��Buf�е��豸��modbusTCPҪ�������, modbusRTU����Ӧ */
	RESPOND_OTHERS_MODBUS_RES_V4_NO_REG,	/* V4�쳣: �����豸������ram�е����ݣ�modbusTCPҪ�������, modbusRTU���� */
	RESPOND_OTHERS_MODBUS_RES_V5_NO_DEV,	/* V5�쳣: ��Buf�е��豸��modbusTCPҪ�������, modbusRTU����Ӧ */
	RESPOND_OTHERS_MODBUS_RES_V5_NO_REG,	/* V5�쳣: �����豸������ram�е����ݣ�modbusTCPҪ�������, modbusRTU���� */
	RESPOND_OTHERS_MODBUS_RES_FAIL,			/* ʧ��(V4,V5): �����豸�����쳣��modbusTCP����������ʣ�ֱ�ӱ��� */
}RESPOND_OTHERS_MODBUS_RES;
RESPOND_OTHERS_MODBUS_RES ModbusReadFromRamForOthers(uint8** ppModbusRTUBuf);

void RespondModbusTCPForOthers(MODBUS_TCP_COMM* pModbusTcpComm, BOOL bBlock)
{
	if(pModbusTcpComm->uRxBufPt < 6 + 6) {	/* ���ȼ�飬modbusTCP��ʼ6��byte + �豸��ַ,ָ��,���ݵ�ַ,�������ݻ������ݳ��� */
		pModbusTcpComm->uRxBufPt = 0;		/* ���ȴ��󣬲���Ӧ */
	} else {
		pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = 0;	/* ���֮ǰ���������δ�����ģ�����--����������ڵ�RS485��������ȼ����� */
		uint8* pTRBuf = pModbusTcpComm->u8TRBuf + 6;	/* ָ���豸��ַ����modbusRTU���� */
		RESPOND_OTHERS_MODBUS_RES Res = ModbusReadFromRamForOthers(&pTRBuf);

		if(Res == RESPOND_OTHERS_MODBUS_RES_SUC) {	/* ram�����������ݣ��������ݳ��ȣ�����־����ָ�� */
			pModbusTcpComm->uTxBufPt = pTRBuf - pModbusTcpComm->u8TRBuf;
			pModbusTcpComm->u8TRBuf[5] = pModbusTcpComm->uTxBufPt - 6;
		/* ���ʵ����ݲ���ram�� */
		} else if((Res == RESPOND_OTHERS_MODBUS_RES_V4_NO_DEV) || (Res == RESPOND_OTHERS_MODBUS_RES_V4_NO_REG)) {
			pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = 3;	/* �ύ��RS485������ */
			if(!bBlock) {				/* ��������ֱ�ӷ��� */
				pModbusTcpComm->uTxBufPt = 0;
				return;
			} else {					/* �������ȴ���� */
				int16 i;
				for(i = 10; (i > 0) && (pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP > 0); i--) {
					Task_sleep(OS_TICK_KHz*20);
				}
		
				if(pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP == -1) {
					Res = RESPOND_OTHERS_MODBUS_RES_SUC;	/* ����ָ��uTxBufPt, ���ݳ���u8TRBuf[5]�Ѿ���V4Master������������ */
				} else {
					Res = RESPOND_OTHERS_MODBUS_RES_FAIL;
				}
			}
	#if SUPPORT_V5_MASTER
		} else if(Res == RESPOND_OTHERS_MODBUS_RES_V5_NO_REG) {
			if(TranModbusByCan(&pTRBuf)) {	/* �������ݳ��ȣ�����־����ָ�� */
				pModbusTcpComm->uTxBufPt = pTRBuf - pModbusTcpComm->u8TRBuf;
				pModbusTcpComm->u8TRBuf[5] = pModbusTcpComm->uTxBufPt - 6;
				Res = RESPOND_OTHERS_MODBUS_RES_SUC;
			}
	#endif
		}

		if(Res != RESPOND_OTHERS_MODBUS_RES_SUC) {		/* ���ͨѶʧ��(���������ʧ��)����Ӧ���� */
			pModbusTcpComm->u8TRBuf[5] = 4; 			/* ����Ӧ����3�������ǲ�����������������Ҫ4 */
			pModbusTcpComm->u8TRBuf[7] |= 0x80;
			pModbusTcpComm->u8TRBuf[8] = 0x0B;
			pModbusTcpComm->uTxBufPt = 9;
		}
	}
}

/* ��֧��03,04ָ��, ���ȼ���ڵ���ǰ�Ѿ����ˣ��������û�б�Ҫ�ظ� */
void RespondModbusRTUForOthers(uint8 u8ModbusPort)
{
	uint8* pTRBuf = g_UartComm[u8ModbusPort].u8TRBuf;
	uint16 uRes = ModbusReadFromRamForOthers(&pTRBuf);
	if(uRes == RESPOND_OTHERS_MODBUS_RES_SUC) {
		TxDataForModbusRTU(u8ModbusPort, pTRBuf);
	} else if((uRes == RESPOND_OTHERS_MODBUS_RES_V4_NO_REG) || (uRes == RESPOND_OTHERS_MODBUS_RES_FAIL)) {
		g_UartComm[u8ModbusPort].u8TRBuf[1] |= 0x80;
		pTRBuf = &g_UartComm[u8ModbusPort].u8TRBuf[6];
		TxDataForModbusRTU(u8ModbusPort, pTRBuf);
	}
}

/* �����豸����γ�modbus֡ */
RESPOND_OTHERS_MODBUS_RES ModbusReadFromRamForOthers(uint8** ppModbusRTUBuf)
{
	uint8* pTRBufStart = *ppModbusRTUBuf;	/* ָ��modbusRTU�������������豸��ַλ�� */
	uint8* pTxBuf = pTRBufStart + 3;		/* �ճ����ݳ��ȣ�ָ����������� */
	uint8 u8DevAdd = pTRBufStart[0];
	uint8 u8MchNo = u8DevAdd/10 - 1;		/* ���������±� */
	uint8 u8ModCmd = pTRBufStart[1];
	uint16 uRegAdd = pTRBufStart[2]*0x100 + pTRBufStart[3];
	int16 iRegLen = pTRBufStart[4]*0x100 + pTRBufStart[5];
	uint16 uProcRegNum = 0;

	if(u8DevAdd < 10) {
		if(u8DevAdd == MODDEV_ADD_XMG) {	/* ��XMGˮλ */
			if((iRegLen*2 + 10 > UART_TR_BUF_BLEN) || ((u8ModCmd != 0x03) && (u8ModCmd != 0x04))) {
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
		#if(DEVICE_TYPE == V5_YYT3) 	/* YYT3��ģ��ˮλ�Ǳ��ʱ����Ҫ��Ӧ��ȡ����ֵ */
			} else if(uRegAdd < 2) {
				CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_AnaRes.LogicWtrLvl[LOGIC_WTR_TURB].fWtrLvl, iRegLen, 2);
			} else if(uRegAdd < (CONF_FOR_YKF_NUM + 1)*2) {
				uRegAdd -= 2;
				CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_YYTConf.fConfForYKF[uRegAdd/2], iRegLen, CONF_FOR_YKF_NUM*2 - uRegAdd);
		#else
			} else if(uRegAdd < 2) {
				CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_AnaRes.fTurbWaterLevel, iRegLen, 2);
			} else if(!RespondModbusForReadXmgConf(&pTxBuf, uRegAdd - 2, iRegLen)) {
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
		#endif
			}
		} else {
			return RESPOND_OTHERS_MODBUS_RES_V4_NO_DEV;
		}
	} else if(u8DevAdd%10 == MODDEV_ADDi_YKF) {			/* ������YKF������,��ַ����10,20,30... */
		if(0) {
	#if SUPPORT_V4_MASTER
		} else if((u8MchNo < MAX_YKF_NUM_V4) && g_YKFnXMSData[u8MchNo].u8YkfR2TryCnt_0Fail) { 	/* V4_YKF���������� */
			if((u8ModCmd != 0x03) && (u8ModCmd != 0x04)) {
				return RESPOND_OTHERS_MODBUS_RES_V4_NO_REG;
			} else if(iRegLen*2 + 10 > UART_TR_BUF_BLEN) {	/* buf��� */
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
			} else {
				BOOL bFindDataInRam = FALSE;
				while(iRegLen > 0) {
					if((1000 <= uRegAdd) && (uRegAdd < 1000 + YKF_REG_LEN_1000)) {
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)(&g_YKFnXMSData[u8MchNo].uYKFReg1000[uRegAdd - 1000]), iRegLen, 1000 + YKF_REG_LEN_1000 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((1140 <= uRegAdd) && (uRegAdd < 1140 + YKF_REG_LEN_1140)) {
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)(&g_YKFnXMSData[u8MchNo].uYKFReg1140[uRegAdd - 1140]), iRegLen, 1140 + YKF_REG_LEN_1140 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9800 <= uRegAdd) && (uRegAdd < 9800 + YKF_REG_LEN_9800)) {
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)(&g_YKFnXMSData[u8MchNo].uYKFReg9800[uRegAdd - 9800]), iRegLen, 9800 + YKF_REG_LEN_9800 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9800 + YKF_REG_LEN_9800 <= uRegAdd) && (uRegAdd < 9900)) {
						if(iRegLen > 100 - YKF_REG_LEN_9800) {
							uProcRegNum = 100 - YKF_REG_LEN_9800;
						} else {
							uProcRegNum = iRegLen;
						}
						memset(pTxBuf, 0, uProcRegNum*2);
						pTxBuf += uProcRegNum*2;
					} else if((9900 <= uRegAdd) && (uRegAdd < 9900 + YKF_REG_LEN_9900)) {
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)(&g_YKFnXMSData[u8MchNo].uYKFReg9900[uRegAdd - 9900]), iRegLen, 9900 + YKF_REG_LEN_9900 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if(bFindDataInRam) {
						*pTxBuf++ = 0;
						*pTxBuf++ = 0;
						uProcRegNum = 1;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_V4_NO_REG;
					}
					uRegAdd += uProcRegNum;
					iRegLen -= uProcRegNum;
				}
			}
	#endif
	#if SUPPORT_V5_MASTER
		} else if((u8MchNo < MAX_YKF_NUM) && g_V5YKFData[u8MchNo].u8TryCnt_0Fail) {
			if((u8ModCmd != 0x03) && (u8ModCmd != 0x04)) {
				return RESPOND_OTHERS_MODBUS_RES_V5_NO_REG;
			} else if(iRegLen*2 + 10 > UART_TR_BUF_BLEN) {	/* buf��� */
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
			} else {
				BOOL bFindDataInRam = FALSE;
				while(iRegLen > 0) {
					if((1010 <= uRegAdd) && (uRegAdd < 1010 + V5_CTR_LEN_1010)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_V5YKFData[u8MchNo].u32Ctr1010[(uRegAdd - 1010)/2]), iRegLen, 1010 + V5_CTR_LEN_1010*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9200 <= uRegAdd) && (uRegAdd < 9200 + YKF_MSR_LEN_9200)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_V5YKFData[u8MchNo].u32MsrRes9200[(uRegAdd - 9200)/2]), iRegLen, 9200 + YKF_MSR_LEN_9200*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9600 <= uRegAdd) && (uRegAdd < 9600 + YKF_ANA_LEN_9600)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_V5YKFData[u8MchNo].u32AnaRes9600[(uRegAdd - 9600)/2]), iRegLen, 9600 + YKF_ANA_LEN_9600*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if(bFindDataInRam) {
						*pTxBuf++ = 0;
						*pTxBuf++ = 0;
						uProcRegNum = 1;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_V5_NO_REG;
					}
					uRegAdd += uProcRegNum;
					iRegLen -= uProcRegNum;
				}
			}
	#endif
		} else {
			return RESPOND_OTHERS_MODBUS_RES_FAIL;
		}
	} else if(u8DevAdd%10 == MODDEV_ADDi_XMS) { 				/* XMS�¶�Ѳ����, ��ַ����13,23,33... */
		if(((u8ModCmd != 0x03) && (u8ModCmd != 0x04))		/* ��֧�ֵ�ָ��(��֧��03,04ָ��) */
			|| (iRegLen*2 + 10 > UART_TR_BUF_BLEN))			/* Buf������ */
		{
			return RESPOND_OTHERS_MODBUS_RES_FAIL;
	#if SUPPORT_V4_MASTER
		} else if((u8MchNo < MAX_YKF_NUM_V4) && (uRegAdd < g_YKFnXMSData[u8MchNo].uXMSSenNum)) {	/* ���¶ȴ��������� */
			CopyL16DatToTxBuf(&pTxBuf, (uint8*)(&g_YKFnXMSData[u8MchNo].uXMSData[uRegAdd]), iRegLen, g_YKFnXMSData[u8MchNo].uXMSSenNum - uRegAdd);
	#endif
		} else {
			return RESPOND_OTHERS_MODBUS_RES_FAIL;
		}
	} else if(u8DevAdd%10 == MODDEV_ADDi_YYB) {				/* ������YYB������,��ַ����17,27,37... */
		if(0) {
	#if SUPPORT_V4_MASTER
		} else if((u8MchNo < MAX_RCX_NUM) && g_RcxData[u8MchNo].uRcxTryCnt_0Fail) {
			if((u8ModCmd != 0x03) && (u8ModCmd != 0x04)) {
				return RESPOND_OTHERS_MODBUS_RES_V4_NO_REG;
			} else if(iRegLen*2 + 10 > UART_TR_BUF_BLEN) {	/* buf��� */
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
			} else {
				while(iRegLen > 0) {
					if(uRegAdd == 0x800) {							/* ���� */
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)&g_RcxData[u8MchNo].uDIn0x800, iRegLen, 1);
					} else if(uRegAdd == 0xB00) { 					/* ���� */
						uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, (uint8*)&g_RcxData[u8MchNo].uRelay0xB00, iRegLen, 1);
					} else if((uRegAdd == 0xA00) || (uRegAdd == 0xA01)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_RcxData[u8MchNo].u32EngP_0x0A00_KWH, iRegLen, 2);
					} else if((uRegAdd == 0xA04) || (uRegAdd == 0xA05)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_RcxData[u8MchNo].u32EngQ_0x0A04_KVarH, iRegLen, 2);
					} else if((uRegAdd == 0xA02) || (uRegAdd == 0xA03)) {
						*pTxBuf++ = 0;
						*pTxBuf++ = 0;
						uProcRegNum = 1;
					} else if((0x600 <= uRegAdd) && (uRegAdd < 0x600 + RCX_REG_LEN_0x600)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)&g_RcxData[u8MchNo].fReg0x600[(uRegAdd-0x600)/2], iRegLen, 0x600 + RCX_REG_LEN_0x600 - uRegAdd);
					} else {
						return RESPOND_OTHERS_MODBUS_RES_FAIL;
					}
					uRegAdd += uProcRegNum;
					iRegLen -= uProcRegNum;
				}
			}
	#endif
	#if SUPPORT_V5_MASTER
		} else if((u8MchNo < MAX_YYB_NUM) && g_YYBData[u8MchNo].u8TryCnt_0Fail) {
			if((u8ModCmd != 0x03) && (u8ModCmd != 0x04)) {
				return RESPOND_OTHERS_MODBUS_RES_V5_NO_REG;
			} else if(iRegLen*2 + 10 > UART_TR_BUF_BLEN) {	/* buf��� */
				return RESPOND_OTHERS_MODBUS_RES_FAIL;
			} else {
				BOOL bFindDataInRam = FALSE;
				while(iRegLen > 0) {
					if((1010 <= uRegAdd) && (uRegAdd < 1010 + V5_CTR_LEN_1010)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_YYBData[u8MchNo].u32Ctr1010[(uRegAdd - 1010)/2]), iRegLen, 1010 + V5_CTR_LEN_1010*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9200 <= uRegAdd) && (uRegAdd < 9200 + YYB_MSR_LEN_9200)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_YYBData[u8MchNo].u32MsrRes9200[(uRegAdd - 9200)/2]), iRegLen, 9200 + YYB_MSR_LEN_9200*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if((9600 <= uRegAdd) && (uRegAdd < 9600 + YYB_ANA_LEN_9600)) {
						uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, (uint8*)(&g_YYBData[u8MchNo].u32AnaRes9600[(uRegAdd - 9600)/2]), iRegLen, 9600 + YYB_ANA_LEN_9600*2 - uRegAdd);
						bFindDataInRam = TRUE;
					} else if(bFindDataInRam) {
						*pTxBuf++ = 0;
						*pTxBuf++ = 0;
						uProcRegNum = 1;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_V5_NO_REG;
					}
					uRegAdd += uProcRegNum;
					iRegLen -= uProcRegNum;
				}
			}
	#endif
		} else {
			return RESPOND_OTHERS_MODBUS_RES_FAIL;
		}
#if SUPPORT_TRIPHASE_METER		/* �๦�������� */
	} else if(u8DevAdd%10 == MODDEV_ADDi_TPM) {
		if((0x03 == u8ModCmd || 0x04 == u8ModCmd) && (iRegLen*2 + 10 <= UART_TR_BUF_BLEN) && u8MchNo < MAX_TRIPHA_METER_NUM) {			/* �����ָ�� */
			BOOL bFindDataInRam = FALSE;
			while(iRegLen > 0) {
				if(TRIPHA_MTR_REPUB_REG_DIO == uRegAdd) {		/* ���뿪�� */
					if(g_TriPhasMeter[u8MchNo].uTryCnt_0Fail) {		
						*(uint32*)pTxBuf = g_TriPhasMeter[u8MchNo].uDIO;
						pTxBuf += 4;
						uProcRegNum = 2;
						bFindDataInRam = TRUE;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_FAIL;
					}
#if SUPPORT_PUMP
				} else if((uRegAdd >= PUMP_CTL_STATE) && (uRegAdd <= PUMP_RELAY)) {			/* ��վ����״̬��ͨѶ״̬�����롢������ */
					uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, ((uint8*)&g_PumpCtr[u8MchNo])+((uRegAdd-PUMP_CTL_STATE)*2), 2, 2);
				} else if((uRegAdd >= PUMP_RUN_MODE) && (uRegAdd <= PUMP_READY_STOP_COUNTDOWN)) {		/* ��վ����״̬2 */
					uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, ((uint8*)&g_PumpCtr[u8MchNo])+((uRegAdd-PUMP_CTL_STATE)*2), 1, 1);
				} else if((uRegAdd >= PUMP_ACT_WATER_LEVEL) && (uRegAdd <= PUMP_STOP_WATER_LEVEL)) {		/* ������\ͣ��Ԥ��ˮλ */
					uProcRegNum = CopyL32DatToTxBuf(&pTxBuf, ((uint8*)&g_PumpCommConf)+((uRegAdd-PUMP_ACT_WATER_LEVEL)*2), 2, 2);
				} else if((uRegAdd >= PUMP_BTN_ECHO) && (uRegAdd <= PUMP_BTN_LOCK)) {
					uProcRegNum = CopyL16DatToTxBuf(&pTxBuf, ((uint8*)&g_PumpCtr[u8MchNo].uPnlCtrBtnEcho)+((uRegAdd-PUMP_BTN_ECHO)*2), 2, 2);
#endif
				} else if((uRegAdd >= TRIPHA_MTR_REPUB_REG_UA) && (uRegAdd <= TRIPHA_MTR_REPUB_REG_FRQ)) {
					float32 fValue;
					if(GetValFromTriPhasMtr_RepubAddr(u8MchNo, uRegAdd, &fValue)) {			/* ��ȡ��ѹ\����\����\Ƶ��\��������ֵ */
						*(uint32*)pTxBuf = __rev(*((uint32*)&fValue));
						pTxBuf += 4;
						uProcRegNum = 2;
						bFindDataInRam = TRUE;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_FAIL;
					}
				} else if((uRegAdd >= TRIPHA_MTR_REPUB_REG_KWH) && (uRegAdd <= TRIPHA_MTR_REPUB_REG_RVAR)) {
					uint32 u32Value;
					if(GetWattFromTriPhasMtr_RepubAddr(u8MchNo, uRegAdd, &u32Value)) {			/* ��ȡ���ֵ */
						*(uint32*)pTxBuf = __rev(u32Value);
						pTxBuf += 4;
						uProcRegNum = 2;
						bFindDataInRam = TRUE;
					} else {
						return RESPOND_OTHERS_MODBUS_RES_FAIL;
					}
				} else if(bFindDataInRam) {
					*pTxBuf++ = 0;
					*pTxBuf++ = 0;
					uProcRegNum = 1;
				} else {
					return RESPOND_OTHERS_MODBUS_RES_FAIL;
				}
				uRegAdd += uProcRegNum;
				iRegLen -= uProcRegNum;
			}
		} else if((0x06 == u8ModCmd) || (0x10 == u8ModCmd)) {		/* 06 10 ָ�� */
			uint8 u8DevNo = u8DevAdd / 10;
			if((uRegAdd == PUMP_CTR_BTN) && (g_PumpCtr[u8DevNo / 10].u8CtrMode > PUMP_RUN_MODE_HAND)) {
				if((iRegLen & (1<<PUMP_CTR_BTN_AUTO_BitNo)) && (g_PumpCtr[u8DevNo].u8CtrMode == PUMP_RUN_MODE_MON_LOCAL)) {
					g_PumpCtr[u8DevNo].u8CtrMode = PUMP_RUN_MODE_UNKNOWN;
				} else if(iRegLen & (1<<PUMP_CTR_BTN_START_BitNo)) {
					g_PumpCtr[u8DevNo].u8CtrMode = PUMP_RUN_MODE_MON_LOCAL;
					ProcPumpCtrKey(PUMP_CTR_BTN_START_BitNo,u8DevNo);  			/* �˴��Ƿ���Ҫ�жϲ����ɹ���	��Ҫ���ɹ�����Ϣ�Ļ���Ӧ��Ҫ��ModBus��Ӧ */
				} else if(iRegLen &(1<<PUMP_CTR_BTN_STOP_BitNo)) {					/* ֹͣ�ã�д���Զ���ȡ����Ȩ�� */
					g_PumpCtr[u8DevNo].u8CtrMode = PUMP_RUN_MODE_MON_LOCAL;
					ProcPumpCtrKey(PUMP_CTR_BTN_STOP_BitNo,u8DevNo);
				} else if(iRegLen &(1<<PUMP_CTR_BTN_CANCEL_BitNo)) {				/* ȡ�� */
					ProcPumpCtrKey(PUMP_CTR_BTN_CANCEL_BitNo,u8DevNo);
				} else if(iRegLen &(1<<PUMP_CTR_BTN_OK_BitNo)) {					/* ȷ�� */
					ProcPumpCtrKey(PUMP_CTR_BTN_OK_BitNo,u8DevNo);
				}
			}
		} else {
			return RESPOND_OTHERS_MODBUS_RES_FAIL;		/* ��֧��03,04,06,0x10ָ�� */
		}
#endif
	} else {
		return RESPOND_OTHERS_MODBUS_RES_V4_NO_DEV;
	}

	pTRBufStart[2] = pTxBuf - pTRBufStart - 3;	/* ������ݳ��� */
	*ppModbusRTUBuf = pTxBuf;
	return RESPOND_OTHERS_MODBUS_RES_SUC;
}
#endif

#if(DEVICE_TYPE == V5_YYT3)
/*==========================================================================
| Description	: ���ڷ�������modbusTCPתmodubsRTUͨѶ����
	TrySerialServer(): ���Գ�Ϊ���ڷ�����
	RunSerialServer(): ���ڷ�������������
| G/Out var		:
| Author		: Wang Renfei			Date	: 2015-9-27
\=========================================================================*/
void TrySerialServer(uint8 u8UartPort)
{
	OFF_INDICATOR_RS485(u8UartPort);
	OpenUartComm(u8UartPort, g_CommConf.u32RS485Baud_100bps[u8UartPort]*100, 0, OS_TICK_KHz*7);	/* 5byteʱ�䣬���λ��������2Byte���ݳ��ȣ�����1200bps�¹������� (20220703�ᶼ��ˮ�ӵ�վ��Ҫ����7ms)*/
	g_UartComm[u8UartPort].uTimer_WatchDog_ms = 0;
	g_UartComm[u8UartPort].bUartFuncTrySuc = TRUE;
}

/* �������з�YKF��modbusTCPͨ������ */
void RunSerialServer(uint8 u8UartPort)
{
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];
	MODBUS_TCP_COMM* pModbusTcpComm = g_ModbusTcpComm;
	pUartComm->uTmr_Run_ms = 10;

	int8 i;
	for(i = 0; i < MODBUS_TCP_COMM_MAX_NUM; i++) {
		/* ����ModbusTCP�Ǿ�������ͨѶ����������״̬�ȴ��ظ�����˱���ͨѶ���ʧ�ܾ������ٴγ��ԣ��Ҳ�����YKF�Ķ�д���� */
		while((pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP > 0) && (pModbusTcpComm->u8TRBuf[6]%10 != MODDEV_ADDi_YKF)) {
			if(pModbusTcpComm->uRxBufPt <= 6) {
				pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = -2;
				OFF_INDICATOR_RS485(u8UartPort);
			} else {
				uint16 uReadRegLen = (uint16)pModbusTcpComm->u8TRBuf[10]*0x100 + (uint16)pModbusTcpComm->u8TRBuf[11];	/* �����ָ���ȡ�ļĴ������� */
				uint16 uCopyLen = pModbusTcpComm->uRxBufPt - 6;
				memcpy(pUartComm->u8TRBuf, &pModbusTcpComm->u8TRBuf[6], uCopyLen); /* ����modbusTCP��Ϣ֡ǰ��6��byte */
				TxDataForModbusRTU(u8UartPort, &(pUartComm->u8TRBuf[uCopyLen]));
				if(ReadFromUart(u8UartPort, 100)
					&& (CheckRxBufForModbusRTUMaster(pUartComm, pModbusTcpComm->u8TRBuf[6], pModbusTcpComm->u8TRBuf[7], uReadRegLen) > 0))	/*>*/
				{
					ON_INDICATOR_RS485(u8UartPort);
					pUartComm->uTimer_WatchDog_ms = 0; 		/* ͨѶ�ɹ� */
					pUartComm->bUartFuncTrySuc = TRUE;

					/* ���ݿ��� */
					Swi_disable();	/* ��ֹ������i8RandAcs_Idle0_SucN1_FailN2_MaxTryCntP������ */
					if(pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP > 0) {
						uCopyLen = pUartComm->uRxBufPt - 2;		/* ����������CRC���� */
						pModbusTcpComm->u8TRBuf[5] = uCopyLen;		/* ���ݳ��� */
						memcpy(&pModbusTcpComm->u8TRBuf[6], pUartComm->u8TRBuf, uCopyLen);
						pModbusTcpComm->uTxBufPt = uCopyLen + 6;	/* uTRBufPt��0��������ͨѶ�ɹ� */
						pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = -1;
					}
					Swi_enable();
				} else {
					OFF_INDICATOR_RS485(u8UartPort);
					Swi_disable();
					if(pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP == 1) {
						pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP = -2;
					} else if(pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP > 1) {
						pModbusTcpComm->i8ModbusRTUAcs_Idle0_SucN1_FailN2_MaxTryCntP--;
					}
					Swi_enable();
				}
			}
			Task_sleep(OS_TICK_KHz*20);		/* TODO 104���Ժùر������������̬ȥ������û����  */
		}
		pModbusTcpComm++;
	}
	Task_sleep(OS_TICK_KHz*pUartComm->uTmr_Run_ms);
}
#endif

/*==========================================================================
| Description	: ���ͨ��RS485������: �������Ǵӻ������Ķ�������
| G/Out var		: 
| Author		: Wang Renfei			Date	: 2017-08-28
\=========================================================================*/
void TrySubSoftUpdate(uint8 u8UartPort)
{
	uint16 uDeviceType, uSoftVersion;
	
	OpenUartComm(u8UartPort, 115200UL, 0, 0);
	if((RdDataFrModbusRTU(u8UartPort, 10, 0xFF, MODADD_DEVICE_TYPE, 1, &uDeviceType, MOD_RD_04W) > 0)
	    && (uDeviceType == DEVICE_TYPE)
		&& (RdDataFrModbusRTU(u8UartPort, 10, 0xFF, MODADD_DEVICE_SOFT_VER, 1, &uSoftVersion, MOD_RD_04W) > 0))
	{
		IniSoftUpdate(uSoftVersion);
	} else {
		g_UartComm[u8UartPort].uTimer_WatchDog_ms = 0xFFFF;
		g_UartComm[u8UartPort].bUartFuncTrySuc = FALSE;
	}
}

void RunSubSoftUpdate(uint8 u8UartPort)
{
	FLASH_REQ FlashReq;
	UART_COMM* pUartComm = &g_UartComm[u8UartPort];

	/* ��д FlashReq �Ļظ��д��� FlashPack */
	CreateFlashReq(&FlashReq, RS485_TR_FLASH_PACK_BLEN);
	if(WrDataToModbusRTU(u8UartPort, 10, 0xFF, MODADD_UPDATE_START, sizeof(FLASH_REQ)/2, (uint16*)(&FlashReq), MOD_WR_10B) > 0) {
		if(!UpdateSoft((uint32*)&pUartComm->u8TRBuf[8], pUartComm->uRxBufPt - 10)) {	
			pUartComm->UartApp = RS485_MCGS_RTU_SLAVE;
		}
	}
}

/******************************** FILE END ********************************/

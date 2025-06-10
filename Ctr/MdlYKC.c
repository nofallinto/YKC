//���س���ʱ��û��AC��ͷʱ�������������նȲ���������AC��ͷʱ����������ʾû�Ӱ��ӣ�������stlink����3.3������3.28v��������AC��ͷ3.29v������ԭ�򻹲�֪���������ҷ������¹���
//������˾ܴ���ԭ��
//    A,  ����ɫ���ϲ����е�ɬ���֡�Ħ�������󣬵�����ʱ����ѹ��ʱ����ײ���ػᰴ����ȫ���²������ã���������Ե��ǰ��ͬʱ̽��ʱ��Ҫͬ��������ײ����ͬʱ��⵽��ײ�����⵽�ߣ���ɹ����ֵĸ��ʾ͸�С��
//    B���������������ײ����stall�������濨סʱ�еĻ�������û�г������õ�1A��ô���ҷſ���0.5A��������������װ��ԭ����ܷ���ϲ�ͬ��ԭ���еĻ����е�û��2800Pa������Ĵ������סʱ����û��ô�󣬴Ӷ�û�ɹ���⵽stall��������ѹ��������ܷ�����������ϵ���Ͱ�װ�����и�����϶�ܱղ���Ҳ�й�ϵ��




#if SOFT_RUN1_TEST0
/***************************************************************************
 * CopyRight(c)		YoPlore	, All rights reserved
 *
 * File			: stencil.c
 * Author		: Wang Renfei
 * Description	: c file's stencil, V1.1 inherit from V1.0
 *				: ģ�黯������ṹ��ÿһ��ģ����ͷ�ļ��ж����ⲿ�ӿڱ���,�ⲿ��������,ģ����÷�����������Դ�ļ��ж���ģ���ڲ�����������
 * Comments		:
 * Date			: 2006-11-27
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 * V0.1 | 2023-12-08  | king | 1. ��ʼ��
 **************************************************************************/

/***************************************************************************
 						macro definition
***************************************************************************/
#define _MDL_YKC_C_		/* exclude redefinition */

/***************************************************************************
 						include files
***************************************************************************/
#include <string.h>
#include <math.h>

/* ��Ŀ����ͷ�ļ� */
#include "GlobalVar.h"
#include "MdlSys.h"
#include "LibMath.h"

/* �����Լ��¼�ģ��ͷ�ļ� */
#include "MdlNet.h"
#include "MdlDataAccess.h"
#include "BoardSupport.h"
#include "MdlUARTnModbus.h"
#include "MdlGUI.h"
#include "BMI270.h"		/* IMU */
#include "BMP280.h"		/* ��ѹ */

#include "MdlYKC.h"

/* ����ģ�� */
/***************************************************************************
 						global variables definition
***************************************************************************/
const CONF_n_STAT_PROP cnst_YkcConfProp[] = {                                              /* 1 2 3 4 5 6 7 8 9 A B C D E F 1 2 3 4 5 6 7 8 9 A B C D E F */
    {1001, {ITEM_DAT_TOPIC, 0, 0, SAVE_GRP_NUL}, 3, NULL,                      {"ϵͳ��ͨѶ����", "Default"}},
    {1002, {ITEM_DAT_SOFTVER, 0, 0, SAVE_GRP_NUL}, SOFTWARE_VER, NULL,                {"����汾", "Soft Version"}},
    {1003, {ITEM_DAT_U32_ID_RO, 0, 0, SAVE_GRP_BRD}, 0, (void*)&g_Sys.SerialNo,       {"���к�", "S/N:"}},
    {1004, {ITEM_DAT_T32_RO, 0, 0, SAVE_GRP_BRD}, 0, (void*)&g_Sys.u32License_Seconds,{"��Ȩ����(������ʱ)", "deadline"}},
};

/* EEProm��ַ���� */
/* EEProm��ַ����, �б���TI����λ��������˳��洢��ST���÷�BANK�洢���������� */
#define EEPROM_ADDR_BRD_MAIN		(EEADD_DEV_CONF_START						+ 0*64)
#define EEPROM_ADDR_BRD_BAK			(EEPROM_ADDR_BRD_MAIN+FLASH_HALF_BLEN)
#define EEPROM_ADDR_MCONF_MAIN		(EEPROM_ADDR_BRD_MAIN						+ 5*64)
#define EEPROM_ADDR_MCONF_BAK		(EEPROM_ADDR_MCONF_MAIN+FLASH_HALF_BLEN)
extern void _c_int00(void);
volatile const SECTION(".BootLoaderConf") BOOT_LOADER_CONF cnst_BootLoaderConf = {
	(uint32)&_c_int00, (CONF_n_STAT_PROP*)cnst_YkcConfProp, sizeof(cnst_YkcConfProp)/sizeof(CONF_n_STAT_PROP), MAX_SAVE_GRP_NUM,
	{{0, 0, 0, 0},
	{EEPROM_ADDR_BRD_MAIN, EEPROM_ADDR_BRD_BAK, 0, 0},
	{EEPROM_ADDR_MCONF_MAIN, EEPROM_ADDR_MCONF_BAK, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0}}
};


typedef enum {
	CLN_ST_IDLE				= 0,		/* ����:��ձöϵ� */
	CLN_ST_STOP				= 1,		/* ��ͣ */
	CLN_ST_HANG				= 2,		/* ����:��ձ��ϵ磬���ǻ�û��������̨�� */
	CLN_ST_END				= 3,		/* ������ */
	CLN_ST_JUMP				= 4,		/* ѭ����ת */

	/* ����Ϊ�������� */
	CLN_ST_DRIVE			= 5,		/* ֱ��+��ת */
	CLN_ST_JTURN			= 6,		/* J�� (ֱ��ת��) */
	CLN_ST_CTURN			= 7,		/* S�� (��U����) */
	/* ̽�����أ����ش�������Ҫ�����ϴ��� */
	CLN_ST_EXP_FORWARD_L	= 8,		/* ǰ��̽������� */
	CLN_ST_EXP_FORWARD_R	= 9,		/* ǰ��̽���ұ��� */
	CLN_ST_EXP_BACK_L		= 10,		/* ����̽������� */
	CLN_ST_EXP_BACK_R		= 11,		/* ����̽���ұ��� */
	CLN_ST_BLANK			= 12,		/* �հ� */

	CLN_ST_TEST
}WIN_CLN_STATE;

/* ��һ���������������ų����Ķ������� */
typedef struct {
	WIN_CLN_STATE WinClnST;
	int8 i8Angle;				/* Ŀ��Ƕȣ�0��Ϊ0�㣬-45��Ϊ9�㣬+45��Ϊ3�㣩����[-120��,120��] */
	int16 iForward;				/* ǰ�����ߺ��ˣ����Ƽ����� */
	int8 i8Spray;				/* ��ˮ��0���磬1��ǰ�磬-1����� */
	uint8	u8Rsvd;
	uint16	uRsvd;
}CLN_ST_SEQ;
/* ������ CLN_ST_SEQ.iForward ����ָ�����֮�ⶼ�ǺϷ���Ч�ľ���ָ�� */
#define FORWORD_INSTR_ReturnX	32001	/* ����Xԭ�� */
#define FORWORD_INSTR_ReturnY	32002		/* ����Yԭ�� */
#define FORWORD_INSTR_INF		32000	/* ��ǰ�ߵ�ͷ */
#define FORWORD_INSTR_nINF		-32000	/* ���˵�ͷ */
#define VALID_FORWORD_DISTANCE(x) ((FORWORD_INSTR_nINF < x) && (x < FORWORD_INSTR_INF))
/* ������ CLN_ST_SEQ.iSpray ָ�� */
#define SPRAY_INSTR_SPRAY_NONE		0
#define SPRAY_INSTR_SPRAY_FRONT		1
#define SPRAY_INSTR_SPRAY_BACK		(-1)

const CLN_ST_SEQ cnst_ClnSTSeq[] = {
	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},
	{CLN_ST_DRIVE,			45, 	0,						SPRAY_INSTR_SPRAY_NONE},	/* 1. У׼:��ת45�� */
	{CLN_ST_DRIVE, 			-45,	0,						SPRAY_INSTR_SPRAY_NONE},	/* 2. У׼:��ת45�� */
	{CLN_ST_DRIVE, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 3. У׼:�ص�0�� */
	{CLN_ST_DRIVE, 			0, 		FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_FRONT},	/* 4. �˶����� */
	{CLN_ST_DRIVE, 			0, 		-1800,					SPRAY_INSTR_SPRAY_NONE},	/* 5. ���� */
	{CLN_ST_DRIVE, 			50, 	0,						SPRAY_INSTR_SPRAY_NONE},	/* 6. ��ת50�� */
	{CLN_ST_JTURN, 			90, 	1,						SPRAY_INSTR_SPRAY_FRONT},	/* 7. ǰ��J��:���ٽǶ� */
	{CLN_ST_EXP_FORWARD_R, 	90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_NONE},	/* 8. ��������ǰ�� */
	{CLN_ST_EXP_BACK_R, 	90,	 	FORWORD_INSTR_nINF,		SPRAY_INSTR_SPRAY_BACK},	/* 9. �������غ��� */
	{CLN_ST_EXP_FORWARD_R, 	90, 	FORWORD_INSTR_INF, 		SPRAY_INSTR_SPRAY_NONE},	/* 10. �ٴ���������ǰ����Ϊ��ֹ(��������ʱû���ǵ�������)��(��Ƚ�խʱ�����Ͻǵ���������)û������ */

	/* ������Z��� */
	{CLN_ST_JTURN, 			60, 	-1,						SPRAY_INSTR_SPRAY_NONE},	/* 11. ����J��:�Ӵ�Ƕ� */
	{CLN_ST_CTURN, 			90, 	-4000,					SPRAY_INSTR_SPRAY_NONE},	/* 12. ����S��:���ٽǶ� */
	{CLN_ST_DRIVE, 			90, 	FORWORD_INSTR_nINF,		SPRAY_INSTR_SPRAY_BACK},	/* 13. ���˵�ͷ */
	{CLN_ST_CTURN, 			120, 	4000,					SPRAY_INSTR_SPRAY_NONE},	/* 14. ǰ��S��:�Ӵ�Ƕ� */
	{CLN_ST_CTURN, 			90, 	4000,					SPRAY_INSTR_SPRAY_NONE},	/* 15. ǰ��S��:���ٽǶ� */
	{CLN_ST_DRIVE, 			90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_FRONT},	/* 16. ǰ����ͷ */
	{CLN_ST_CTURN, 			60, 	-4000,					SPRAY_INSTR_SPRAY_NONE},	/* 17. ����S��:�Ӵ�Ƕ� */
	{CLN_ST_CTURN, 			90, 	-4000,					SPRAY_INSTR_SPRAY_NONE},	/* 18. ǰ��S��:���ٽǶ� */
	{CLN_ST_JUMP, 			0, 		-6,						SPRAY_INSTR_SPRAY_NONE},	/* 19. ��ת��Z��࿪ͷ */


	/* �ײ����:��������(ǰ��)S�����ߺ���ǰ�����ٺ��� */
	{CLN_ST_JTURN, 			90, 	1,						SPRAY_INSTR_SPRAY_NONE},	/* 20. ǰ��J��:���ٽǶ� */
	{CLN_ST_EXP_FORWARD_L, 	90, 	FORWORD_INSTR_INF,		SPRAY_INSTR_SPRAY_FRONT},	/* 21. ǰ�������Ͻ� */
	{CLN_ST_EXP_BACK_L, 	90, 	FORWORD_INSTR_nINF,		SPRAY_INSTR_SPRAY_NONE},	/* 22. �������غ��� */
	{CLN_ST_JTURN, 			60, 	1,						SPRAY_INSTR_SPRAY_NONE},	/* 23. ǰ��J��:�Ӵ�Ƕ� */
	{CLN_ST_DRIVE, 			90, 	4000,					SPRAY_INSTR_SPRAY_NONE},	/* 24. ǰ��J��:���ٽǶ� */
	{CLN_ST_DRIVE, 			90, 	FORWORD_INSTR_ReturnX,	SPRAY_INSTR_SPRAY_NONE},	/* 25. X�᷵��ԭ�� */
	{CLN_ST_DRIVE,	 		0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 26. ��ת������ */
	{CLN_ST_DRIVE, 			0, 		FORWORD_INSTR_ReturnY,	SPRAY_INSTR_SPRAY_NONE},	/* 27. Y�᷵��ԭ�� */
	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 28. ������� */

	/* �ײ����:��������(����)S�����ߺ��Ⱥ��ˣ���ǰ�� */
	{CLN_ST_JTURN, 			90,	 	-1, 					SPRAY_INSTR_SPRAY_NONE},	/* 29. ����J��:���ٽǶ� */
	{CLN_ST_EXP_BACK_L, 	90, 	FORWORD_INSTR_nINF, 	SPRAY_INSTR_SPRAY_BACK},	/* 30. �������غ��� */
	{CLN_ST_EXP_FORWARD_L, 	90, 	FORWORD_INSTR_INF, 		SPRAY_INSTR_SPRAY_NONE},	/* 31. ��������ǰ�� */
	{CLN_ST_JTURN, 			120, 	-1, 					SPRAY_INSTR_SPRAY_NONE},	/* 32. ����J��:�Ӵ�Ƕ� */
	{CLN_ST_DRIVE, 			90, 	-4000, 					SPRAY_INSTR_SPRAY_NONE},	/* 33. ����S��:���ٽǶ� */
	{CLN_ST_DRIVE, 			90, 	FORWORD_INSTR_ReturnX, 	SPRAY_INSTR_SPRAY_NONE},	/* 34. X�᷵��ԭ�� */
	{CLN_ST_DRIVE, 			0, 		0, 						SPRAY_INSTR_SPRAY_NONE},	/* 35. ��ת������ */
	{CLN_ST_DRIVE, 			0, 		FORWORD_INSTR_ReturnY, 	SPRAY_INSTR_SPRAY_NONE},	/* 36. Y�᷵��ԭ�� */
	{CLN_ST_END, 			0, 		0, 						SPRAY_INSTR_SPRAY_NONE},	/* 37. ������� */

	/* ����1 */
	{CLN_ST_DRIVE, 			45, 	0,						SPRAY_INSTR_SPRAY_NONE},	/* 38. ��ת45�� */
	{CLN_ST_JTURN, 			90, 	1,						SPRAY_INSTR_SPRAY_NONE},	/* 39. ǰ��J��:���ٽǶ� */
	{CLN_ST_END, 			0, 		0,						SPRAY_INSTR_SPRAY_NONE},	/* 40. ������� */
};
#define CLN_SEQ_START_No		1
#define CLN_SEQ_Z_BEGIN_No		11
#define CLN_SEQ_WIN_BTM_R2L_No	20			/* �ײ���࣬�������� */
#define CLN_SEQ_WIN_BTM_L2R_No	29			/* �ײ���࣬�������� */
#define CLN_SEQ_TEST_No			38
#define CLN_SEQ_TEST2_No		39
#define CLN_SEQ_TEST3_No		40

typedef struct {
	/* ������λ�ã�������λ��Ϊԭ�� */
	int32 i32X;			/* ��0.01����Ϊ��λ */
	int32 i32Y;			
	float32 fAngle;		/* �����˳���12�㷽��Ϊ0�ȣ�0��Ϊ0�㣬-45��Ϊ9�㣬+45��Ϊ3�㣩 */

	/* ��ǰ���˶����� */
	int32 i32x;
	int32 i32y;

	int32 i32TripCntForSpray;			/* һ����ɨ�ű�״̬�����е��Ĵ��г��ۻ��� Ϊ������ˮʱ����ͳ�ƣ� */

	/* ��ǰ״̬ */
	WIN_CLN_STATE WinClnST, WinClnST_beforeSTOP;
	uint8 u8ClnSeqNo, u8ClnSeqNo_last;			/*  */
	uint16 uTimer_StrStpOn_10ms;		/* ��ͣ�����³���ʱ�� */
	uint16 uTimer_WinClnST;				/* ��ĳ��״̬������ʱ�� */
	uint16 uTimer_DInSig_10ms[MAX_DIN_NUM];
	uint32 u32DinDat_last;

	/* �˶�Ŀ�� */
	int32 i32Forward;					/* ǰ������: >0ǰ�� <0���� */
	float32 fAimedAngle;				/* ͷ����Ŀ����ԽǶȣ�0��Ϊ0�㣬-45��Ϊ9�㣬+45��Ϊ3�㣩 */

	/* �Ĵ����� */
	float32 fDragConfR2L;				/* �����ֵ��������� */
	float32 fVr2lAimErr_last;

	/* ������ */
	float32 fRightDuty;
	float32 fLeftDuty;

	/* �����Ĵ������תʶ������������� */
	/* �Ĵ����������ʱ�����������һ��ʱ��󣬶�ת�жϲ�������ǰ����ʱ��Ϊ�������˼�ʱ��Ϊ�� */
	int8 i8Timer_RightAct;
	int8 i8Timer_LeftAct;
	BOOL bRightStall;					/* �Ҳ��Ĵ�ͣ�Ϳ�ס */
	BOOL bLeftStall;					/* �Ҳ��Ĵ�ͣ�Ϳ�ס */

	/* ��ձ� */
	float32 fAirPressure_PumpIdle;		/* ��ձ�ͣʱ�Ŀ���ѹ�� */
	float32 fVacuumAimErr_last;			/* Խ�ӽ�0˵���ŷ�����Խ�� */
	float32 fPumpDuty;					/* ��ձ�PWMռ�ձ� */
	uint8 u8Tmr_PumpFullSpeed_10ms;		/* �ж���ձ��������㹻ʱ����������������Ҫ���ȫ�٣�0��ʾʱ�����㹻�õ���ȫ�� */
	uint8 u8Tmr_VacummSuff_10ms;		/* ��ձ����������ʱ�������ں���˲ʱ����ѹ���� */
	int8 i8Tmr_Spray_Front_10ms;		/* ����ϴҺ(ǰ)��ʱ��, -1����˼�ʱ������δʹ�� */
	int8 i8Tmr_Spray_Back_10ms;			/* ����ϴҺ(��)��ʱ��, -1����˼�ʱ������δʹ�� */

	uint8 u8DebugNo;					/* ���ڵ��� */

	BOOL bDisableCliff;
}YKC_CTR;
YKC_CTR g_YKCCtr;

/* �����ÿ���----------------------------------------*/
#define CLIFF_DEACTIVE 	FALSE			/* ��ʱ�ر����¿��أ���Ϊ���� */
#define PUMP_DEACTIVE	FALSE			/* �ر���ձ� */
#define LOW_AIR_P		TRUE			/* ��������ˮƽ��2300Pa�������Ǳ�׼��2800Pa */
/* �����ÿ���----------------------------------------*/

/* 1000���Ĵ������壬����Լ37mm; 27����/mm; �Ĵ����Լ230mm, ������Ϊ265mm*265mm������ */
#define MCH_LENGTH_mm				265.0f		/* ������ */
#define TRACK_DISTANCE_mm			230.0f		/* �Ĵ���� */
#define PULSE_FORWARD_mm			0.037f		/* ÿ����ǰ������ */
#define TRACK_DISTANCE_pulse		(TRACK_DISTANCE_mm/PULSE_FORWARD_mm) /* �Ĵ���࣬���Ĵ��н�����Ϊ��λ��Լ6216������ */
#define PI							3.1415926f
#define VACUUM_AIM_ERR_TOLERANCE	0.15f
#define PUMP_TICK_TO_FULL_SPEED		200			/* ��ձÿ���ȫ������ʱ������λ10ms�� */
#define PUMP_TICK_VACUMM_SUFF		100			/* ��ѹ�����ж�ʱ������λ10ms��  */
#define PUMP_IDEL_DUTY				0.3f		/* IDELʱά�ֺܵ͵���������ֹ���� */
#define SPRAY_TIMEOUT_10ms			100			/* ��ˮ��ʱ */
#if LOW_AIR_P
			#define AIMED_AIR_P	2100
#else
			#define AIMED_AIR_P	2800
#endif

/***************************************************************************
						internal functions declaration
***************************************************************************/
void NextWinClnSTBySeq(uint8 u8NewSeqNo_0Auto);

/***************************************************************************
 							functions definition
***************************************************************************/
/*==========================================================================
| Description	: 
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2023-12-8
\=========================================================================*/
void InitMdlCtr(void)		/* rename from InitYKC() */
{
	/* ��ʼ������ */


	/* ��ʼ������ӿڱ��� */

	/* ��ʼ������ӿڱ��� */
	InitDataWithZero((uint8*)(&g_MsrRes), sizeof(g_MsrRes));
	InitDataWithZero((uint8*)(&g_AnaRes), sizeof(g_AnaRes));
	InitDataWithZero((uint8*)(&g_YKCCtr), sizeof(g_YKCCtr));
	InitDataWithZero((uint8*)(&g_Ctr), sizeof(g_Ctr));
	g_YKCCtr.fDragConfR2L = 0.88;
	g_YKCCtr.u8Tmr_PumpFullSpeed_10ms = PUMP_TICK_TO_FULL_SPEED;
	g_YKCCtr.u8Tmr_VacummSuff_10ms = PUMP_TICK_VACUMM_SUFF;
	g_YKCCtr.i8Tmr_Spray_Front_10ms = -1;
	g_YKCCtr.i8Tmr_Spray_Back_10ms = -1;

	g_YKCCtr.fPumpDuty = 0;
	g_YKCCtr.i32TripCntForSpray = 0;
	/* ��ʼ���²�ģ�� */

	/* ����Ӳ�� */
	InitSample();		/* ���ڲ�����Ҫ���buf��Ȼ��źü��㣬����������� */

	/* ��ʼ������ģ�� */
	InitTtsSpeaking();
	
	GPIO_write(OTHER_DEV_DOutNo, 1);		/* ʹ���ܱ��豸3.3v���� */
	/* ��ʼ�����ᴫ���� */
    if(!Bmi270Init()) {
		/* IMU��ʼ��ʧ��*/
    	TtsSpeak(VOICE_IMU_ABN, FALSE);
		configASSERT(FALSE);
	}
	/* ��ʼ����ѹ�� */
    if(!Bmp280Init()) {
		/* IMU��ʼ��ʧ��*/
    	TtsSpeak(VOICE_AIR_PRES_ABN, FALSE);
		configASSERT(FALSE);
	}

	/* ��ʼ������ң����  TIM5Ƶ��1MHz, ARR==65535*/
	extern TIM_HandleTypeDef htim5;
	HAL_TIM_Base_Start_IT(&htim5);	 			/* ������ʱ���жϣ�֮����û����main.c�����Ϊ���ڳ�ʼ������֮���ٿ�ʼ */
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);  /* ������ʱ�����벶��֮����û����main.c�����Ϊ���ڳ�ʼ������֮���ٿ�ʼ */
}

/*==========================================================================
| Description	: 
	�˶����Ʒ�Ϊ���˶����ߡ��˶�ִ����۲�
	
	�˶���Ϊ�����������߼�����(�ɶ�������������)
	����������ֱ��+ת��
			
	������λ�ã�x,y,a����x,y �������㣬ÿһ���˶�������Ĵ��˶����롢�˶�������£�a�ǲ������Ƕȣ�ͨ��imu����

	�˶����ߣ����嶯����{}��ǣ��˶�Ŀ����[]��ǣ�
	�˶������㣺
		������{ԭ����ת}��������[���ϣ���ֱ�ڵ���]��
		Ȼ��{ֱ��(����������)}��ֱ������
		Ȼ��{��ͷSת��}��
		Ȼ��{����}��ֱ����ͷ����{����}��ֱ����ͷ��
		��N��-ֱ��-N��-ֱ�У�ֱ��N����ǰ������
		
��ɾ��
		ֱ�У������Ĵ�ͬ���˶�����֤�߹���ͬ�ĳ���
			��������ֱ��ˮƽ��б�������˶��켣���˶�����Ҫ����G���򲻱�
			Ч�����ǶȲ��䣬�����˶�
		ԭ����ת�������Ĵ������˶�����֤�߹���ͬ�ĳ���
			Ч�������Ĳ������Ƕȷ����ı�
		����ת�䣺һ���Ĵ���ֹ������һ���Ĵ��˶�
			Ч������һ���Ĵ�����Ϊԭ�����ת�������ķ�����ת�˶�
		˫��ת�䣺�����Ĵ��Բ�ͬ�ٶ����У�Ӧ����ָ��
			Ч����ת��뾶�ϵ���ת�����
	��϶�����
		��ͷת�䣺ֱ��(�˺�**)+ԭ����ת
		���ߣ������������أ�����ת���Ի�ȡС�Ƕȳ��ű�����ʻ��ʹ����λ(������)���ڲ��ϴ���״̬
		̽�ߣ�̽�����أ����ַ�����һ���Ǵ��ߵ����
		N/Z�䣺ֱ���������-����ת��-ֱ��(��ʵ��б��)-����ת�䣬��������ĵ���ת��Ӧ����˫��ת��

		
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2023-12-8
\=========================================================================*/
/* �Ĵ������ж�ֵ */
BOOL IsMeetWheelStallCur(float32 fCurrent)
{
//	float32 fVacummRate = (g_YKCCtr.fAirPressure_PumpIdle - g_MsrRes.fAirPressure) / AIMED_AIR_P;
//	/* 2107PaʱΪ0.76A, 2800PaʱΪ1A, �����ѹ��ΪAIMED_AIR_Pʱ���Ĵ���ס������������ж����� */
//	if((fVacummRate < 1.0f) && (fVacummRate > 0.5f)) {		/* >0.5�Ƿ�ֹ��ն�ż������һ�£������ڱ����гɿ�ס */
//		return fCurrent > fVacummRate;
//	} else {
//		return fCurrent > 1.0f;
//	}
	return fCurrent > 0.5f;
}

void RunMdlCtr(void)		/* rename from RunYKC */
{
//	g_YKCCtr.bDisableCliff = FALSE;

	ControlLed(0);
	int8 i;

	/* ����ֵ */
	ProcDInFilter();		/* �����˲� */

//#if CLIFF_DEACTIVE
	if(g_YKCCtr.bDisableCliff) {
		g_Ctr.u32DinDat |= (1 << CLIFF_FRONT_LEFT_DInNo) | (1 << CLIFF_FRONT_RIGHT_DInNo) | (1 << CLIFF_REAR_LEFT_DInNo) | (1 << CLIFF_REAR_RIGHT_DInNo);
	}
//#endif

	CalDCSig(DCPWR_VOL_DCSigNo, 10, 1, &g_MsrRes.fDCPwrVol, NULL);
	CalDCSig(RIGHT_CUR_DCSigNo, 10, 1, &g_MsrRes.fRightCur, NULL);
	CalDCSig(LEFT_CUR_DCSigNo, 10, 1, &g_MsrRes.fLeftCur, NULL);
	g_MsrRes.fCPUTemperature = CalCPUTemperature(CPU_TEMP_DCSigNo);
	g_MsrRes.fVrefint = CalVrefint(VREF_INT_DCSigNo);
	g_MsrRes.fPumpFreq = ProcPulseSig(BUMP_MOTOR_PulseNo);
	g_MsrRes.fRightFreq = ProcPulseSig(RIGHT_MOTOR_PulseNo);
	g_MsrRes.fLeftFreq = ProcPulseSig(LEFT_MOTOR_PulseNo);

	/* ���� */
	UpdateAir();		/* ������ѹ�� */
	UpdateIMU();		/* ����IMU״̬ */

	/* ����: ��Ҫ���������������ת�л���ʱ�����Ĵ������ת>=1.5A��������������0.5A */
	if(g_YKCCtr.fRightDuty > 0) {
		if(g_YKCCtr.i8Timer_RightAct < 0) {
			g_YKCCtr.i8Timer_RightAct = 1;
		} else if(g_YKCCtr.i8Timer_RightAct < 100) {
			g_YKCCtr.i8Timer_RightAct++;
		}
	} else if(g_YKCCtr.fRightDuty < 0) {
		if(g_YKCCtr.i8Timer_RightAct > 0) {
			g_YKCCtr.i8Timer_RightAct = -1;
		} else if(g_YKCCtr.i8Timer_RightAct > -100) {
			g_YKCCtr.i8Timer_RightAct--;
		}
	} else {
		g_YKCCtr.i8Timer_RightAct = 0;
	}
	if(g_YKCCtr.fLeftDuty > 0) {
		if(g_YKCCtr.i8Timer_LeftAct < 0) {
			g_YKCCtr.i8Timer_LeftAct = 1;
		} else if(g_YKCCtr.i8Timer_LeftAct < 100) {
			g_YKCCtr.i8Timer_LeftAct++;
		}
	} else if(g_YKCCtr.fLeftDuty < 0) {
		if(g_YKCCtr.i8Timer_LeftAct > 0) {
			g_YKCCtr.i8Timer_LeftAct = -1;
		} else if(g_YKCCtr.i8Timer_LeftAct > -100) {
			g_YKCCtr.i8Timer_LeftAct--;
		}
	} else {
		g_YKCCtr.i8Timer_LeftAct = 0;
	}

	/* �ж��Ĵ��Ƿ�ס */
	g_YKCCtr.bRightStall = (abs(g_YKCCtr.i8Timer_RightAct) > 10) && (IsMeetWheelStallCur(g_MsrRes.fRightCur));
	g_YKCCtr.bLeftStall = (abs(g_YKCCtr.i8Timer_LeftAct) > 10) && (IsMeetWheelStallCur(g_MsrRes.fLeftCur));
//	BOOL bCurOv = InverseTimeVerify(ABN_T_INV_MOTOR_RIGHT_No, 0, 3, 0.05, g_MsrRes.fRightCur, 1, MSG_NULL)
//				|| InverseTimeVerify(ABN_T_INV_MOTOR_LEFT_No, 0, 3, 0.05, g_MsrRes.fLeftCur, 1, MSG_NULL);
	
	
	/* ���붨ʱ�����¿��أ��ͷ���Ϊ���ź� */
	for(i = MAX_DIN_NUM - 1; i > HIT_REAR_RIGHT_DInNo; i--) {
		if(g_Ctr.u32DinDat & (1<<i)) {
			g_YKCCtr.uTimer_DInSig_10ms[i] = 0;
		} else if(g_YKCCtr.uTimer_DInSig_10ms[i] < 0xFFFF) {
			g_YKCCtr.uTimer_DInSig_10ms[i]++;
		}
	}
	/* ���붨ʱ���������أ�������Ϊ���ź� */
	for(i = HIT_REAR_RIGHT_DInNo; i >= 0; i--) {
		if((g_Ctr.u32DinDat & (1<<i)) == 0) {
			g_YKCCtr.uTimer_DInSig_10ms[i] = 0;
		} else if(g_YKCCtr.uTimer_DInSig_10ms[i] < 0xFFFF) {
			g_YKCCtr.uTimer_DInSig_10ms[i]++;
		}
	}
#define SINGLE_SIG_VALID_TIME_10ms 20

	/* ˢ���г� */
	UInt HwiKey = Hwi_disable();
	int16 iRightCount = g_MsrRes.uRightCount;
	int16 iLeftCount = g_MsrRes.uLeftCount;
	g_MsrRes.uRightCount = 0;
	g_MsrRes.uLeftCount = 0;
    Hwi_restore(HwiKey);
	if(g_YKCCtr.fRightDuty < 0) {
		iRightCount = 0 - iRightCount;		/* ��תʱ���ø������� */
	}
	if(g_YKCCtr.fLeftDuty < 0) {
		iLeftCount = 0 - iLeftCount;		/* ��תʱ���ø������� */
	}

	/* ���㵱ǰ�Ƕ� */
	float32 fAngle = g_MsrRes.fPitch;
	float32 fAngleAvr = ((g_YKCCtr.fAngle + fAngle)/2)*(PI/180);		/* ����ƽ���Ƕȣ����ȣ� */

	/* ˢ�µ�ǰλ�� */
	float32 fTrip = (iRightCount + iLeftCount)*0.5f;
	g_YKCCtr.i32X += F32ToI32(fTrip*sinf(fAngleAvr));
	g_YKCCtr.i32Y += F32ToI32(fTrip*cosf(fAngleAvr));

	/* �۲��˶�Ŀ�����������н� */
	int32 i32Forward = g_YKCCtr.i32Forward;
	/* �н�Ŀ��Ϊ0(CLN_ST_JҲ��û����ȷ���н�Ŀ��)���򿴽Ƕ��Ƿ�ɨ��Ŀ��--����ط����߼������һ������ */
	if((i32Forward == 0) || (g_YKCCtr.WinClnST == CLN_ST_JTURN)) {
		if(((g_YKCCtr.fAngle < g_YKCCtr.fAimedAngle) && (g_YKCCtr.fAimedAngle < fAngle))
			|| ((g_YKCCtr.fAngle > g_YKCCtr.fAimedAngle) && (g_YKCCtr.fAimedAngle > fAngle)))
		{
			g_YKCCtr.fAimedAngle = fAngle;
		}
	} else {
		i32Forward -= F32ToI32(fTrip);
		if(((g_YKCCtr.i32Forward > 0) && (i32Forward < 0))	/* �н���ͷ--����ǡ�õ�0 */
			|| ((g_YKCCtr.i32Forward < 0) && (i32Forward > 0)))
		{
			i32Forward = 0;
			g_YKCCtr.fAimedAngle = fAngle;	/* �н���λ�����عܽǶ�(���½Ƕ�����ƫ��)������Ϊ��ɵ�ǰ���� */
		} else if((g_YKCCtr.WinClnST == CLN_ST_CTURN)	/* CTurnת�ǵ�λҲ�ǿ��Ե� */
			&& (((g_YKCCtr.fAngle < g_YKCCtr.fAimedAngle) && (g_YKCCtr.fAimedAngle < fAngle))
				|| ((g_YKCCtr.fAngle > g_YKCCtr.fAimedAngle) && (g_YKCCtr.fAimedAngle > fAngle))))
		{
			i32Forward = 0;
			g_YKCCtr.fAimedAngle = fAngle;
		}
	}
	g_YKCCtr.fAngle = fAngle;
	g_YKCCtr.i32Forward = i32Forward;

	/* ά����ʱ�� */
	if(g_YKCCtr.uTimer_WinClnST < 0xFFFF) {
		g_YKCCtr.uTimer_WinClnST++;
	}

	/* ������ֹͣ */
	if(g_YKCCtr.uTimer_DInSig_10ms[START_STOP_CMD_DInNo] == 1) {	/* ��ť������ */
		if(g_YKCCtr.WinClnST == CLN_ST_IDLE) {
			g_YKCCtr.u8DebugNo++;
			g_YKCCtr.WinClnST = CLN_ST_HANG;
			TtsSpeak(VOICE_WELCOM, FALSE);		/* ��ӭʹ�� */
			//g_YKCCtr.WinClnST = CLN_ST_TEST;
		} else if(g_YKCCtr.WinClnST == CLN_ST_STOP) {
			g_YKCCtr.WinClnST = g_YKCCtr.WinClnST_beforeSTOP;
		} else {
			g_YKCCtr.WinClnST_beforeSTOP = g_YKCCtr.WinClnST;
			g_YKCCtr.WinClnST = CLN_ST_STOP;
		}
	} else if(g_YKCCtr.uTimer_DInSig_10ms[START_STOP_CMD_DInNo] > 250) {	/* ��ť������2.5�� */
		if(g_YKCCtr.WinClnST != CLN_ST_IDLE) {
			TtsSpeak(VOICE_BYEBYE, FALSE);
			g_YKCCtr.WinClnST = CLN_ST_IDLE;
			g_YKCCtr.u8Tmr_PumpFullSpeed_10ms = PUMP_TICK_TO_FULL_SPEED;
		}
	}

	/* �˶����� */
	switch(g_YKCCtr.WinClnST) {
		case CLN_ST_IDLE:			/* ����:��ձöϵ� */
			ControlLed(LED_COLOR_BLUE);
			g_YKCCtr.u8ClnSeqNo = 0;
			break;

		case CLN_ST_STOP:			/* ��ͣ:��ձô��磬��Ҫ�������� */
			ControlLed(LED_COLOR_YELLOW);
			break;

		case CLN_ST_END:			/* ������:��ձô��磬��Ҫ�������� */
//			ControlLed(LED_COLOR_GREEN);
			TtsSpeak(VOICE_FINISHCLEAN, FALSE);
			g_YKCCtr.u8ClnSeqNo = 0;
			//g_YKCCtr.fAimedAngle = 0;
			//g_YKCCtr.i32Forward = 0;
			break;

		case CLN_ST_HANG:			/* ����:��ձ��ϵ磬���ǻ�û��������̨�� */
			ControlLed(LED_COLOR_RED);
			if((g_YKCCtr.uTimer_DInSig_10ms[START_STOP_CMD_DInNo] == 0) && (g_YKCCtr.u8Tmr_PumpFullSpeed_10ms == 0)) { 		/* �ſ����ذ�ť */
				if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]			/* �ĸ������� */
					&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo]
					&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]
					&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo])
				{
					TtsSpeak(VOICE_CLIFF_DETECTED, FALSE);
				} else {
					NextWinClnSTBySeq(CLN_SEQ_START_No);
				}
			}
			break;
		
		case CLN_ST_JUMP:
			NextWinClnSTBySeq(g_YKCCtr.u8ClnSeqNo + cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward);
			break;

		case CLN_ST_BLANK:
			ControlLed(LED_COLOR_CRAN);
			NextWinClnSTBySeq(0);
			break;

		case CLN_ST_TEST:
			//NextWinClnSTBySeq(CLN_SEQ_TEST_No);
			break;

		case CLN_ST_DRIVE:
			ControlLed(LED_COLOR_CRAN);
			if(((g_YKCCtr.i32Forward == 0) && (g_YKCCtr.fAimedAngle == g_YKCCtr.fAngle))
				|| ((g_YKCCtr.i32Forward > 0) 
					&& ((g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| ((g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] || g_YKCCtr.bLeftStall)
							&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] || g_YKCCtr.bRightStall))
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]
							&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo])))
				|| ((g_YKCCtr.i32Forward < 0)
					&& ((g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| ((g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] || g_YKCCtr.bLeftStall)
							&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] || g_YKCCtr.bRightStall))
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]
							&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo]))))
			{
				NextWinClnSTBySeq(0);
			}
			break;

		case CLN_ST_JTURN:
			ControlLed(LED_COLOR_CRAN);
			if((fabsf(g_YKCCtr.fAimedAngle - g_YKCCtr.fAngle) < 5)
				|| ((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward > 0)
					&& (((g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] || g_YKCCtr.bLeftStall)
							&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] || g_YKCCtr.bRightStall))
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]
							&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo])))
				|| ((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward < 0)
					&& (((g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] || g_YKCCtr.bLeftStall)
							&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] || g_YKCCtr.bRightStall))
						|| (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]
							&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo]))))
			{
				NextWinClnSTBySeq(0);
			}
			break;

		case CLN_ST_CTURN:
			ControlLed(LED_COLOR_CRAN);
			/* �����ˣ�ǰ���Ļ�����ǰ���������أ����˵Ļ�������������� */
			if((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward > 0)
				&& (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_FRONT_DInNo]))
			{
				NextWinClnSTBySeq(CLN_SEQ_WIN_BTM_R2L_No);
			} else if((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward < 0)
				&& (g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_REAR_DInNo]))
			{
				NextWinClnSTBySeq(CLN_SEQ_WIN_BTM_L2R_No);
			} else if(g_YKCCtr.fAimedAngle == g_YKCCtr.fAngle) {
				NextWinClnSTBySeq(0);
			} 
			break;

		case CLN_ST_EXP_FORWARD_L:	/* ����ǰ�У������Ҫ�����ϴ��� */
		case CLN_ST_EXP_FORWARD_R:	/* ����ǰ�У��ұ���Ҫ�����ϴ��� */
			ControlLed(LED_COLOR_CRAN);
			/* �����Ա�̣���Ӧ���ߵ��ⲽ */
			if(g_YKCCtr.u8ClnSeqNo >= sizeof(cnst_ClnSTSeq)/sizeof(CLN_ST_SEQ)) {
				/* ��ͷ: ���´����������գ�����ǰ����ײ������������ */
			} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]	/* ���������� */
						&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo])
					|| ((g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] || g_YKCCtr.bLeftStall)
						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] || g_YKCCtr.bRightStall)))
					//|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
					//|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms))
			{
				NextWinClnSTBySeq(0);
			} else if(g_YKCCtr.WinClnST == CLN_ST_EXP_FORWARD_L) {/* ����ǰ�У������Ҫ�����ϴ��� */
				/* ��ǰ���������ҹ� */
				if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo]
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_FRONT_DInNo])
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
					g_YKCCtr.i32Forward = 1000;
				/* ��Ҳû������������� */
				} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo] == 0)
						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_REAR_DInNo] == 0))
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
					g_YKCCtr.i32Forward = 1000;
				} else {
					g_YKCCtr.i32Forward = 1000;
				}
			} else {	/* ����ǰ�У��ұ���Ҫ�����ϴ��� */
				/* ��ǰ����������� */
				if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo]
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_FRONT_DInNo])
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
					g_YKCCtr.i32Forward = 1000;
					g_CodeTest.u32Val[9]++;
				/* ��Ҳû�����������ҹ� */
				} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo] == 0)
						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_REAR_DInNo] == 0))
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
					g_YKCCtr.i32Forward = 1000;
					g_CodeTest.u32Val[10]++;
				} else {
					g_YKCCtr.i32Forward = 1000;
					g_CodeTest.u32Val[11]++;
				}
			}
			break;

		case CLN_ST_EXP_BACK_L:	/* �������У������Ҫ�����ϴ��� */
		case CLN_ST_EXP_BACK_R:	/* �������У��ұ���Ҫ�����ϴ��� */
			ControlLed(LED_COLOR_CRAN);
			/* �����Ա�̣���Ӧ���ߵ��ⲽ */
			if(g_YKCCtr.u8ClnSeqNo >= sizeof(cnst_ClnSTSeq)/sizeof(CLN_ST_SEQ)) {
				/* ��ͷ: ���´����������գ�����ǰ����ײ������������ */
			} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]	/* ���������� */
						&& g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo])
					|| ((g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] || g_YKCCtr.bLeftStall)
						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] || g_YKCCtr.bRightStall)))
					//|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
					//|| (g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms))
			{	
				NextWinClnSTBySeq(0);
			} else if(g_YKCCtr.WinClnST == CLN_ST_EXP_BACK_L) {/* �������У������Ҫ�����ϴ��� */
				/* ������������ҹ� */
				if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_LEFT_DInNo]
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_REAR_DInNo])
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
					g_YKCCtr.i32Forward = -1000;
				/* ǰ��Ҳû������������� */
				} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_LEFT_DInNo] == 0)
						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_FRONT_DInNo] == 0))
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
					g_YKCCtr.i32Forward = -1000;					
				} else {	/* ���򣬳���� */
					g_YKCCtr.i32Forward = -1000;
				}
			} else {	/* �������У��ұ���Ҫ�����ϴ��� */
				/* �Һ������������ */
				if(g_YKCCtr.uTimer_DInSig_10ms[CLIFF_REAR_RIGHT_DInNo]
					|| g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_REAR_DInNo])
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle - 5;
					g_YKCCtr.i32Forward = -1000;
				/* ǰ��Ҳû�����������ҹ� */
				} else if((g_YKCCtr.uTimer_DInSig_10ms[CLIFF_FRONT_RIGHT_DInNo] == 0)
						&& (g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_FRONT_DInNo] == 0))
				{
					g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle + 5;
					g_YKCCtr.i32Forward = -1000;
				} else {
					g_YKCCtr.i32Forward = -1000;
				}
			}
			break;

		default:
			g_YKCCtr.WinClnST = CLN_ST_STOP;
			break;
	}

	/* ��ʱ�Ա������룬���⵽�˱��ػ����� */
	if(((g_Ctr.u32DinDat & (1<<CLIFF_FRONT_LEFT_DInNo)) == 0)	/* ����ȫ���գ�Ӧ���Ƿ������� */
		&& ((g_Ctr.u32DinDat & (1<<CLIFF_FRONT_RIGHT_DInNo)) == 0)
		&& ((g_Ctr.u32DinDat & (1<<CLIFF_REAR_LEFT_DInNo)) == 0)
		&& ((g_Ctr.u32DinDat & (1<<CLIFF_REAR_RIGHT_DInNo)) == 0))
	{		
	} else if(((g_YKCCtr.i32Forward > 0)	/* ǰ�� */
		&& ((((g_Ctr.u32DinDat & (1<<CLIFF_FRONT_LEFT_DInNo)) == 0)	/* ���������� */
				&& ((g_Ctr.u32DinDat & (1<<CLIFF_FRONT_RIGHT_DInNo)) == 0))
			|| ((g_Ctr.u32DinDat & (1<<HIT_FRONT_LEFT_DInNo))
				&& (g_Ctr.u32DinDat & (1<<HIT_FRONT_RIGHT_DInNo)))))
	|| ((g_YKCCtr.i32Forward < 0)	/* ���� */
		&& ((((g_Ctr.u32DinDat & (1<<CLIFF_REAR_LEFT_DInNo)) == 0)	/* ���������� */
				&& ((g_Ctr.u32DinDat & (1<<CLIFF_REAR_RIGHT_DInNo)) == 0))
			|| ((g_Ctr.u32DinDat & (1<<HIT_REAR_LEFT_DInNo))
				&& (g_Ctr.u32DinDat & (1<<HIT_REAR_RIGHT_DInNo))))))
	{
		g_YKCCtr.i32Forward = 0;
		//g_YKCCtr.WinClnST = CLN_ST_STOP;
	}

	/* �˶�ִ��:�����˶�Ŀ�꣬���㵱ǰ��������Ҫ���Ĵ����� */
	float32 fRightTrip = 0;
	float32 fLeftTrip = 0;
	float32 fd = 0;
	if(g_YKCCtr.WinClnST < CLN_ST_DRIVE) {
		g_YKCCtr.fRightDuty = 0;
		g_YKCCtr.fLeftDuty = 0;
	} else {
		float32 fForward;
		if(g_YKCCtr.WinClnST == CLN_ST_JTURN) {
			/* d/D = 2*tan(A)*H/(L - tan(A)*(L-H))
			   d: �����Ĵ�����
			   D: ����Ĵ��ٶ�(ȫ��)
			   A: �����������(Զ��/����)�ļн�
			   H: �Ĵ����
			   L: ������		 */
			/* �����������߼н� */
			float32 fAngle = fabsf(g_YKCCtr.fAngle - 90);	/* ��׼����ˮƽ����(90��) */
			/* ǰ�����������λ���ض�����Ҫ�ر���Ҫ��Ȼ���ǰ��������г̿��������������ǰ��������ײ��������ź� */
			if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward > 0) {
				if(g_YKCCtr.fAimedAngle > g_YKCCtr.fAngle) {/* ǰ������:ע����ǰ��Ҫ�����в䣬�����������Ҫ�Ӵ�ת��Ƕ� */
					if(g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_FRONT_DInNo]) {
						fAngle += 10;
					}
				} else {	/* ǰ������:ע����ǰ��Ҫ�����в䣬�����������Ҫ�Ӵ�ת��Ƕ� */
					if(g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_FRONT_DInNo]) {
						fAngle += 10;
					}
				}
			} else {
				if(g_YKCCtr.fAimedAngle < g_YKCCtr.fAngle) {/* ��������:ע���Һ�Ҫ�����в䣬�����������Ҫ�Ӵ�ת��Ƕ� */
					if(g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_REAR_DInNo]) {
						fAngle += 10;
					}
				} else {	/* ��������:ע�����Ҫ�����в䣬�����������Ҫ�Ӵ�ת��Ƕ� */
					if(g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_REAR_DInNo]) {
						fAngle += 10;
					}
				}
			}

			/* �������ֲ��� */
			float32 fTanA = tanf(fAngle*PI/180);
			fd = 2*fTanA*TRACK_DISTANCE_mm/(MCH_LENGTH_mm - fTanA*(MCH_LENGTH_mm - TRACK_DISTANCE_mm));
			g_YKCCtr.u8DebugNo = 2;
			if(fd > 1) {
				fd = 1;
			} else if(fd < 0.1f*g_YKCCtr.u8DebugNo) {
				fd = 0.1f*g_YKCCtr.u8DebugNo;
			}

			/* ǰ������ */
			fForward = 2000;	/* 1��Լ2K�����壬����0.1��Ԥ�� */
			if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward < 0) {
				fForward = 0 - fForward;
			}
			/* ���� */
			if((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward > 0) ^ (g_YKCCtr.fAimedAngle < g_YKCCtr.fAngle)) {
				fRightTrip = fForward;
				fLeftTrip = fForward*(1 - fd);
			} else {	/* ���� */
				fRightTrip = fForward*(1 - fd);
				fLeftTrip = fForward;
			}
		} else {
			float32 fTurnAngle = g_YKCCtr.fAimedAngle - g_YKCCtr.fAngle;
			if(fTurnAngle > 180) {
				fTurnAngle -= 360;
			} else if(fTurnAngle < -180) {
				fTurnAngle += 360;
			}
			/* ���������г̣�������30�ȶ�Ӧ��1627������ */
			float32 fTripErr = (TRACK_DISTANCE_pulse/2)*fTurnAngle*(PI/180.0f);

			fForward = fabsf(g_YKCCtr.i32Forward);
			/* ��ֹת�ǹ��󡢹�С��ת��һ����S�䣬4000ת30��(1627����)��fabsf(fForward/fTripErr) = 2.5 */
			if(fForward > 10*fabsf(fTripErr)) {	/* ��Ϊg_YKCCtr.i32Forward������1e10�����½Ƕȹ��� */
				fForward = 10*fabsf(fTripErr);
			} else if(g_YKCCtr.i32Forward && (fForward < 1.25*fabsf(fTripErr))) {	
				fForward = 1.25*fabsf(fTripErr);
			}
			if(g_YKCCtr.i32Forward < 0) {	/* �ָ����� */
				fForward = 0 - fForward;
			}
			fRightTrip = fForward + fTripErr;
			fLeftTrip = fForward - fTripErr;
		}

		/* �����ٶ�:�ٶȱ�ֵ */
		float32 fRightDuty, fLeftDuty;
		/* ĳһ���Ĵ������巽��һ�£���������г����һ��㣬�����Ƕȷ����˶�����Ϊ�˱�������˷���������������ǰ������һ�µ��Ĵ����� */
		if((fabsf(fRightTrip) < fabsf(fLeftTrip)*0.01)
			|| ((fRightTrip > 0) && (fForward < 0))
			|| ((fRightTrip < 0) && (fForward > 0))
			|| ((g_YKCCtr.WinClnST != CLN_ST_JTURN)
				&& (((g_YKCCtr.i32Forward > 0) && g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo])
					|| ((g_YKCCtr.i32Forward < 0) && g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo]))))
		{
			fRightDuty = 0;
			fLeftDuty = 1;
		} else if((fabsf(fLeftTrip) < fabsf(fRightTrip)*0.01)
			|| ((fLeftTrip > 0) && (fForward < 0))
			|| ((fLeftTrip < 0) && (fForward > 0))
			|| ((g_YKCCtr.WinClnST != CLN_ST_JTURN)
				&& (((g_YKCCtr.i32Forward > 0) && g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo])
					|| ((g_YKCCtr.i32Forward < 0) && g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo]))))
		{
			fRightDuty = 1;
			fLeftDuty = 0;
		} else {	/* �����ֵ */
			float32 fAim_R2L = fabsf(fRightTrip/fLeftTrip);
			float32 fPID_P = 0;
			float32 fPID_I = 0;
			if((g_YKCCtr.u8ClnSeqNo != g_YKCCtr.u8ClnSeqNo_last)
				|| (g_MsrRes.fLeftFreq == 0) || (g_MsrRes.fRightFreq == 0)
				|| (g_YKCCtr.fLeftDuty == 0) || (g_YKCCtr.fRightDuty == 0))
			{
				fAim_R2L *= g_YKCCtr.fDragConfR2L;
				g_YKCCtr.fVr2lAimErr_last = 0;
			} else {
				float32 fVr2lAimErr = fAim_R2L - g_MsrRes.fRightFreq/g_MsrRes.fLeftFreq;
				fPID_I = fVr2lAimErr*0.1;
				fPID_P = (fVr2lAimErr - g_YKCCtr.fVr2lAimErr_last) * VACUUM_AIM_ERR_TOLERANCE;
				fAim_R2L = fabsf(g_YKCCtr.fRightDuty/g_YKCCtr.fLeftDuty) + fPID_P + fPID_I;
				g_YKCCtr.fVr2lAimErr_last = fVr2lAimErr;
			}

			if(fAim_R2L > 10) {	/* �޷� */
				fAim_R2L = 10;
			} else if(fAim_R2L < 0) {
				fAim_R2L = 0;
			}
			if(fAim_R2L > 1) {
				fRightDuty = 1;
				fLeftDuty = 1/fAim_R2L;
			} else {
				fRightDuty = fAim_R2L;
				fLeftDuty = 1;
			}
		}
		/* �����ٶ�:�������� */
		if(fRightTrip < 0) {
			fRightDuty = 0 - fRightDuty;
		}
		if(fLeftTrip < 0) {
			fLeftDuty = 0 - fLeftDuty;
		}
		g_YKCCtr.fRightDuty = fRightDuty;
		g_YKCCtr.fLeftDuty = fLeftDuty;
	}
	
	/* ��Ӧң������Ԥ */
	UpdateIRCtrl();
	if(g_IRCtrl.u8TryCnt) {
		switch(g_IRCtrl.u8BtnPressing) {
		case IR_BTN_0:
			Board_DrvPump(0.1f);
			break;
		case IR_BTN_1:
			Board_DrvPump(0.3f);
			break;
		case IR_BTN_2:
			Board_DrvPump(0.5f);
			break;
		case IR_BTN_3:
			Board_DrvPump(1.0f);
			break;
		case IR_BTN_4:
			break;
		case IR_BTN_5:
			break;
		case IR_BTN_6:
			break;
		case IR_BTN_7:
			break;
		case IR_BTN_8:
			g_YKCCtr.bDisableCliff = FALSE;
			break;
		case IR_BTN_9:
			g_YKCCtr.bDisableCliff = TRUE;
			break;
		case IR_BTN_START:
			/* �����ˮ */
			GPIO_write(TOP_WATER_JET_RelayNo, 1);
			if(g_YKCCtr.i8Tmr_Spray_Front_10ms == -1) {
				g_YKCCtr.i8Tmr_Spray_Front_10ms = SPRAY_TIMEOUT_10ms;
			}
			break;
		case IR_BTN_SHARP:
			/* �ұ���ˮ */
			GPIO_write(BOT_WATER_JET_RelayNo, 1);
			if(g_YKCCtr.i8Tmr_Spray_Back_10ms == -1) {
				g_YKCCtr.i8Tmr_Spray_Back_10ms = SPRAY_TIMEOUT_10ms;
			}
			break;
		case IR_BTN_UP:
			g_YKCCtr.fLeftDuty = 1;
			g_YKCCtr.fRightDuty = 1;
			break;
		case IR_BTN_DOWN:
			g_YKCCtr.fLeftDuty = -1;
			g_YKCCtr.fRightDuty = -1;
			break;
		case IR_BTN_LEFT:
			g_YKCCtr.fLeftDuty = 1;
			g_YKCCtr.fRightDuty = -1;
			break;
		case IR_BTN_RIGHT:
			g_YKCCtr.fLeftDuty = -1;
			g_YKCCtr.fRightDuty = 1;
			break;
		case IR_BTN_OK:
			g_YKCCtr.fLeftDuty = 0;
			g_YKCCtr.fRightDuty = 0;
			break;
		}
	}

	//�����ã��������������ɫ
	//�����ã��������������ɫ
	//�����ã��������������ɫ
	//�����ã��������������ɫ
	//�����ã������ס����ɫ
	//�����ã�����ҿ�ס����ɫ
#if 0
	if((g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
			&& g_YKCCtr.uTimer_DInSig_10ms[HIT_FRONT_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
	{
		ControlLed(LED_COLOR_YELLOW);
	} else if((g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_LEFT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
			&& g_YKCCtr.uTimer_DInSig_10ms[HIT_REAR_RIGHT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
	{
		ControlLed(LED_COLOR_GREEN);
	} else if((g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_FRONT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
			&& g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_L_REAR_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
	{
		ControlLed(LED_COLOR_BLUE);
	} else if((g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_FRONT_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
			&& g_YKCCtr.uTimer_DInSig_10ms[HIT_SIDE_R_REAR_DInNo] > SINGLE_SIG_VALID_TIME_10ms)
	{
		ControlLed(LED_COLOR_CRAN);
	} else if(g_YKCCtr.bLeftStall) {
		ControlLed(LED_COLOR_PINK);
	} else if(g_YKCCtr.bRightStall) {
		ControlLed(LED_COLOR_RED);
	}
#endif
	Board_DrvWheel(g_YKCCtr.fRightDuty, g_YKCCtr.fLeftDuty);	/* ִ�� */

	/* ��ձ� */
	float32 fPumpPID_P, fPumpPID_I;
	if(g_YKCCtr.WinClnST == CLN_ST_IDLE) {
		g_YKCCtr.fPumpDuty = PUMP_IDEL_DUTY;
		g_YKCCtr.fAirPressure_PumpIdle = g_MsrRes.fAirPressure;
	} else {
		if(g_YKCCtr.fPumpDuty == 0) {	/* ������ */
			g_YKCCtr.fPumpDuty = 1;
		} else {
			float32 fVacuumAimErr = (g_MsrRes.fAirPressure + AIMED_AIR_P - g_YKCCtr.fAirPressure_PumpIdle)/AIMED_AIR_P;
			fPumpPID_P = fVacuumAimErr*0.02;
			fPumpPID_I = (fVacuumAimErr - g_YKCCtr.fVacuumAimErr_last)*0.01;
			g_YKCCtr.fPumpDuty += fPumpPID_P + fPumpPID_I;
			g_YKCCtr.fVacuumAimErr_last = fVacuumAimErr;
			if(g_YKCCtr.fPumpDuty > 1) {
				g_YKCCtr.fPumpDuty = 1;
			} else if(g_YKCCtr.fPumpDuty < 0) {
				g_YKCCtr.fPumpDuty = 0;
			}
			/* ����������������CLN_ST_STOP */
			if((g_YKCCtr.u8Tmr_PumpFullSpeed_10ms == 0) && (g_YKCCtr.fVacuumAimErr_last >= VACUUM_AIM_ERR_TOLERANCE)) {
				if(g_YKCCtr.u8Tmr_VacummSuff_10ms > 0) {		/* ����ż������ѹƫ�� */
					g_YKCCtr.u8Tmr_VacummSuff_10ms--;
				} else {		/* ��ѹ����ƫ�� */
					if(g_YKCCtr.WinClnST != CLN_ST_STOP) {
						g_YKCCtr.WinClnST_beforeSTOP = g_YKCCtr.WinClnST;
						g_YKCCtr.WinClnST = CLN_ST_STOP;
					}
					TtsSpeak(VOICE_AIR_PRES_LOW, FALSE);
				}
			} else {
				g_YKCCtr.u8Tmr_VacummSuff_10ms = PUMP_TICK_VACUMM_SUFF;
			}
		}
		if(g_YKCCtr.u8Tmr_PumpFullSpeed_10ms) {
			g_YKCCtr.u8Tmr_PumpFullSpeed_10ms--;
		}
	}

#if PUMP_DEACTIVE
	Board_DrvPump(0);		/* ִ�� */
#else
	Board_DrvPump(g_YKCCtr.fPumpDuty);		/* ִ�� */
#endif

	uint32 u32Flag = g_Ctr.u32DinDat + (g_YKCCtr.bRightStall<<14) + (g_YKCCtr.bLeftStall<<15)
					 + (g_YKCCtr.WinClnST<<16) + (g_YKCCtr.u8ClnSeqNo<<24);
#if 0
	Rec_DebugPack(u32Flag, g_YKCCtr.fRightDuty, g_YKCCtr.fLeftDuty, g_YKCCtr.i32Forward, g_YKCCtr.fAimedAngle, 
				g_YKCCtr.fAngle, fRightTrip, fLeftTrip, g_MsrRes.fRightFreq, g_MsrRes.fLeftFreq, g_MsrRes.fPitch, fd);
#elif 1
	Rec_DebugPack(u32Flag, g_YKCCtr.fRightDuty, g_YKCCtr.fLeftDuty, g_YKCCtr.i32Forward, g_YKCCtr.fAimedAngle, 
				g_YKCCtr.fAngle, fRightTrip, fLeftTrip, g_MsrRes.fRightFreq, g_MsrRes.fLeftFreq, g_MsrRes.fRightCur, g_MsrRes.fLeftCur);

#elif 0
	Rec_DebugPack(u32Flag, g_MsrRes.fAirPressure, fPumpPID_P, fPumpPID_I, g_YKCCtr.fPumpDuty, g_MsrRes.fPumpFreq, 0, 0, 0, 0, 0, 0);
#else
	Rec_DebugPack(u32Flag, g_MsrRes.iAccX, g_MsrRes.iAccY, g_MsrRes.iAccZ, g_YKCCtr.fAngle, g_MsrRes.fPitch, g_MsrRes.fRoll, 0, 0, 0, 0, 0);
#endif

	/* Ϊ������ˮʱ����ͳ�� */
	if((cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Spray != SPRAY_INSTR_SPRAY_NONE)) {	/* �ýű��������ˮ���� */
		/* ˢ��������ˮ������г��ۻ��� */
		g_YKCCtr.i32TripCntForSpray -= ((abs(iLeftCount) + abs(iRightCount)) / 2);		/* ���ֲ�����ֵ */
		if((g_YKCCtr.u8ClnSeqNo != g_YKCCtr.u8ClnSeqNo_last) || (g_YKCCtr.i32TripCntForSpray <= 0)) {		/* ���л��˽ű���Ż��ۻ�����һ������ */
			if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Spray == SPRAY_INSTR_SPRAY_FRONT) {
				GPIO_write(TOP_WATER_JET_RelayNo, 1);
			} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Spray == SPRAY_INSTR_SPRAY_BACK) {
				GPIO_write(BOT_WATER_JET_RelayNo, 1);
			}
			if(g_YKCCtr.i8Tmr_Spray_Front_10ms == -1) {
				g_YKCCtr.i8Tmr_Spray_Front_10ms = SPRAY_TIMEOUT_10ms;
			}
			if(g_YKCCtr.i8Tmr_Spray_Back_10ms == -1) {
				g_YKCCtr.i8Tmr_Spray_Back_10ms = SPRAY_TIMEOUT_10ms;
			}
			g_YKCCtr.i32TripCntForSpray = 15000;		/* ������ˮ������� */
		}
	}

	/* ����ǰ��ˮ����״̬�� -1 δʹ�� / 0 ��ͣ��/ n ��ʼ��ʱ */
	if(g_YKCCtr.i8Tmr_Spray_Front_10ms == 0) {
		GPIO_write(TOP_WATER_JET_RelayNo, 0);
		if(!GPIO_read(LOW_LIQUID_WRN_DInNo)) {
			TtsSpeak(VOICE_LIQUID_LOW, TRUE);
		}
		g_YKCCtr.i8Tmr_Spray_Front_10ms = -1;
	} else if(g_YKCCtr.i8Tmr_Spray_Front_10ms > 0) {
		g_YKCCtr.i8Tmr_Spray_Front_10ms--;
	}
	/* ���º���ˮ���״̬�� -1 δʹ�� / 0 ��ͣ��/ n ��ʼ��ʱ */
	if(g_YKCCtr.i8Tmr_Spray_Back_10ms == 0) {
		GPIO_write(BOT_WATER_JET_RelayNo, 0);
		if(!GPIO_read(LOW_LIQUID_WRN_DInNo)) {
			TtsSpeak(VOICE_LIQUID_LOW, TRUE);
		}
		g_YKCCtr.i8Tmr_Spray_Back_10ms = -1;
	} else if(g_YKCCtr.i8Tmr_Spray_Back_10ms > 0) {
		g_YKCCtr.i8Tmr_Spray_Back_10ms--;
	}

	UpdateTTS();	/* ��������ģ�� */

	g_YKCCtr.u32DinDat_last = g_Ctr.u32DinDat;
	g_YKCCtr.u8ClnSeqNo_last = g_YKCCtr.u8ClnSeqNo;

	g_CodeTest.fVal[69] = g_YKCCtr.fAirPressure_PumpIdle - g_MsrRes.fAirPressure;
	g_CodeTest.fVal[70] = GPIO_read(TOP_WATER_JET_RelayNo);
	g_CodeTest.fVal[71] = GPIO_read(BOT_WATER_JET_RelayNo);
}

/* �������н��ж���
	0	: �������н���
	��0	: ָ����� */
void NextWinClnSTBySeq(uint8 u8NewSeqNo_0Auto)
{
	if((u8NewSeqNo_0Auto != 0)	/* ָ���µ����У����ں�֮ǰ����� */
		/* ��ǰ�Ƿ���ĳ������������ */
		|| ((g_YKCCtr.u8ClnSeqNo < sizeof(cnst_ClnSTSeq)/sizeof(CLN_ST_SEQ))
			&& (cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].WinClnST != CLN_ST_END)))
	{
		if(u8NewSeqNo_0Auto != 0) {		/* ��0������ָ���Ľ��� */
			g_YKCCtr.u8ClnSeqNo = u8NewSeqNo_0Auto;
		} else {						/* 0������ */
			g_YKCCtr.u8ClnSeqNo++;
		}
		/* ��һ����������ǵ�ǰ�������н�β���򷵻� */
		if((g_YKCCtr.u8ClnSeqNo >= sizeof(cnst_ClnSTSeq)/sizeof(CLN_ST_SEQ)) 
			|| (cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].WinClnST == CLN_ST_END))
		{
			g_YKCCtr.i32Forward = 0;
			g_YKCCtr.fAimedAngle = g_YKCCtr.fAngle;
			g_YKCCtr.WinClnST = CLN_ST_END;
		} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].WinClnST == CLN_ST_BLANK) {
			g_YKCCtr.i32Forward = 0;
			g_YKCCtr.fAimedAngle = g_YKCCtr.fAngle;
			g_YKCCtr.WinClnST = CLN_ST_BLANK;			
		} else {	/* �������µĶ������г�ʼ�� */
			if(VALID_FORWORD_DISTANCE(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward)) {
				g_YKCCtr.i32Forward = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward;
			} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward == FORWORD_INSTR_INF) {
				g_YKCCtr.i32Forward = 1000000000;	//Լ37KM
			} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward == FORWORD_INSTR_nINF) {
				g_YKCCtr.i32Forward = -1000000000;	//Լ37KM
			} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward == FORWORD_INSTR_ReturnX) {
				g_YKCCtr.i32Forward = 0 - g_YKCCtr.i32X;
			} else if(cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].iForward == FORWORD_INSTR_ReturnY) {
				g_YKCCtr.i32Forward = 0 - g_YKCCtr.i32Y;
			} else {
				g_YKCCtr.i32Forward = 0;
			}
			g_YKCCtr.fAimedAngle = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].i8Angle;
			g_YKCCtr.WinClnST = cnst_ClnSTSeq[g_YKCCtr.u8ClnSeqNo].WinClnST;
		}
	}
}

/*==========================================================================
| Description	: 
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2012-12-8
\=========================================================================*/
void DrvMdlCtrTick_1KHz(void)		/* rename from DrvYKCTick_1KHz */
{
}


__weak void GetAuthKey(AUTH_TYPE AuthType, uint32* pAuthKey)
{
	/* ���ڶ�������, ������������ */
	uint16 uAuthGrpNo = AuthType/AUTH_TYPE_NUM;
	AuthType = (AUTH_TYPE)(AuthType%AUTH_TYPE_NUM);

	/* ������� */
	if(uAuthGrpNo == 0) {					/* ��0������ */
		switch(AuthType) {
			case AUTH_KEY_FLASH_REQ_KEY:	/* 1�� */
				*pAuthKey++ = 0x767C2584;
				break;

			case AUTH_AES_FLASH_REQ:		/* 4�� */
				*pAuthKey++ = 0x266946E7;
				*pAuthKey++ = 0x571E4325;
				*pAuthKey++ = 0x9857C686;
				*pAuthKey++ = 0x7A26B3C7;
				break;

			case AUTH_AES_FLASH_DATA:		/* 4�� */
				*pAuthKey++ = 0x295692F5;
				*pAuthKey++ = 0x3A2B4896;
				*pAuthKey++ = 0x8284411E;
				*pAuthKey++ = 0x7F525561;
				break;

			case AUTH_AES_PACK_SERIAL:		/* 4�� */
				*pAuthKey++ = 0x533A4B17;
				*pAuthKey++ = 0x7B992D81;
				*pAuthKey++ = 0x47E09593;
				*pAuthKey++ = 0x562F8395;
				break;

			case AUTH_AES_PACK_LICENSE: 	/* 4�� */
				*pAuthKey++ = 0x577A3339;
				*pAuthKey++ = 0x11B75C26;
				*pAuthKey++ = 0x82536B22;
				*pAuthKey++ = 0x8A2B931B;
				break;

			case AUTH_KEY_FLASH_INTEGRITY_V1:
			case AUTH_KEY_FLASH_INTEGRITY_V2:
				*pAuthKey++ = 1;
				*pAuthKey++ = 1;
				break;

		}
	} else if(uAuthGrpNo == 1) {			/* ��1������ */
		switch(AuthType) {
			case AUTH_KEY_FLASH_REQ_KEY:	/* 1�� */
				*pAuthKey++ = 4;
				break;

			case AUTH_AES_FLASH_REQ:		/* 4�� */
				*pAuthKey++ = 4;
				*pAuthKey++ = 3;
				*pAuthKey++ = 2;
				*pAuthKey++ = 1;
				break;

			case AUTH_AES_FLASH_DATA:		/* 4�� */
				*pAuthKey++ = 1;
				*pAuthKey++ = 2;
				*pAuthKey++ = 3;
				*pAuthKey++ = 4;
				break;

			case AUTH_AES_PACK_SERIAL:		/* 4�� */
				*pAuthKey++ = 5;
				*pAuthKey++ = 6;
				*pAuthKey++ = 7;
				*pAuthKey++ = 8;
				break;

			case AUTH_AES_PACK_LICENSE: 	/* 4�� */
				*pAuthKey++ = 4;
				*pAuthKey++ = 3;
				*pAuthKey++ = 2;
				*pAuthKey++ = 1;
				break;

			case AUTH_KEY_FLASH_INTEGRITY_V1:
			case AUTH_KEY_FLASH_INTEGRITY_V2:
				*pAuthKey++ = 1;
				*pAuthKey++ = 1;
				break;

		}
	}
}

__weak BOOL ChkAuthKey(AUTH_TYPE AuthType, uint32* pAuthKey)
{
	/* ���ڶ�������, ������������ */
	uint16 uAuthGrpNo = AuthType/AUTH_TYPE_NUM;
	AuthType = (AUTH_TYPE)(AuthType%AUTH_TYPE_NUM);

	/* �Ƚ����� */
	if(AuthType == AUTH_KEY_FLASH_REQ_KEY) {	/* 1��,�����GetAuthKey()������һ�� */
		return ((uAuthGrpNo == 0) && (pAuthKey[0] == 0x767C2584)) || ((uAuthGrpNo == 1) && (pAuthKey[0] == 4));
	}
	return FALSE;
}

/*==========================================================================
| Description	: Mqtt���ݷ���
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2023/12/8
\=========================================================================*/
BOOL PubSpecialData(MQTT_COMM* pMqttComm)
{
	/* ������Ϣ�����������ͺš�Ӳ���汾�š�����汾�š�IP�ȣ���QoS1���� */
	/* ������Ϣ����:��ʼ�� */
	uint8* pTxBuf = pMqttComm->u8TRBuf + 5;	/* �̶�ͷ��Ԥ����1Byte����+2Byte�ܳ���(��󳤶�16383)+2B Topic���� */
	uint8 u8QoS = 0;	/* �޸ı��η�����QoS������������У����򱾶δ���������г��� */
	/* ������Ϣ����:��ӡTopic */
	PrintStringNoOvChk(&pTxBuf, pMqttComm->broker.clientid);
	PrintStringNoOvChk(&pTxBuf, "/DATA");
	uint16 uMsgIDPt = pTxBuf - pMqttComm->u8TRBuf;
	if(u8QoS) {
		pTxBuf += 2;	/* ���ݷ�������ϢQoS��Ԥ��MsgID����, QoS=0���裬���������д����Ա���һ���� */
	}
	/* ������Ϣ����:��ӡ���ݳ�Json��ʽ */
	*pTxBuf++ = '{';					/* Json��ʼ */
	PrintStringNoOvChk(&pTxBuf, "\"status\":\"������\"");
	*pTxBuf++ = '}';					/* Json��β */
	/* ������Ϣ����:���� */
	return PublishMqtt(pMqttComm, pTxBuf, uMsgIDPt, u8QoS);	/* QoS�޸ı����ڱ��δ��뿪ʼ��λ�� */
}

uint16 ProcLocalCmdInstr(uint8* pRxBuf, uint16 uRegOff, int16 RegNum)
{
	return 0;
}

/*==========================================================================
| Description	: Mqtt����ָ��
| In/Out/G var	:
| Author		: Wang Renfei			Date	: 2023-12-8
\=========================================================================*/
BOOL ProcMqttCmdInstr(MQTT_COMM* pMqttComm, uint8* pU8Msg, uint8* pU8MsgEnd)
{
	return FALSE;	
}
#endif
/******************************** FILE END ********************************/

/*************************************************************************** 
 * CopyRight(c)		YoPlore	, All rights reserved
 *				: 
 * File			: DrvST.h
 * Author		: Wang Renfei
 * Description	:
 * Comments		:
 * Date			: 2014-11-24
 *
 * Revision Control
 *  Ver | yyyy-mm-dd  | Who  | Description of changes
 * -----|-------------|------|--------------------------------------------
 * 		|			  |		 | 
 **************************************************************************/

/***************************************************************************
 				ͷ����exclude redefinition and c++ environment
***************************************************************************/
#ifndef _MSP_F103_H_
#define _MSP_F103_H_

/*===========================================================================
 * �������ļ�
 *==========================================================================*/
//#include "BoardSupport.h"
//#include "MdlNet.h"

#ifdef _MSP_F103_C_
	#define EXT
#else
	#define EXT extern
#endif

#define FLASH_IVT_BLEN		(2*1024)	/* �ж�������(IVT)���� */
#define EEPROM_BLEN			(6*1024)	/*(TI:96*64) FLASH��EEPROMģ�������� */
#define BOOT_VER_BLEN		(2*1024)	/* FLASH��FLASH_BOOT_VER������ */

/************   exclude redefinition    ****************/
#undef EXT				/* release EXT */
#endif /* _DRV_ST_H_ */

//===========================================================================
//	File Name	: POS_PGV_Macro.h
//	Description	: 
//===========================================================================



//===========================================================================
//	Include Files
//===========================================================================

//#include "stm32f10x_type.h"
//#include "macro.def"
// #include "sys_def.h"
#include <math.h>
#include <stdlib.h>


//===========================================================================
//	Local Definitions
//===========================================================================

//===========================================================================
//	Extern func. 
//===========================================================================

//===========================================================================
//	Local Variables
//===========================================================================
typedef struct PGVSENSORVAL
{
     double  XPS;                   // ���� X ��ǥ
     double  YPS;                   // ���� Y ��ǥ 
     double  X_OFFSET;
     double  Y_OFFSET;
     int16_t  ANGLE;
     
     u_int16_t TagNo;
     
     u_int16_t Dir; //left, right, straight   - 1,2,3 
     u_int16_t DirOld;
     
     u_int16_t Color;
     u_int16_t ColorOld;
     
     u_int16_t SensorErr;
} PosSensorVal;

      //------------------------------------------------------------------ 
     //STATE    
     // 0x0000 = Good
     // 0x0001 = Good(warning)
     // 0x0002 = code condition error(code distance chk)
     // 0x0004 = No DIR. decision(Set POS.Sensor DIR.)
     // 0x0008 = No Color decision(Set Color choice)
     // 0x0010 = Out of Range
     // 0x0020 = No Position
     // 0x0040 = Timeout(communication error)
     // 0x0080 = chk_sum error ;
     // 0x1000 = internal error (Recommend to change sensors)
     // 0x2000 = reserved
     // 0x4000 = reserved
     // 0x8000 = reserved
     //--------------------------------------------------------------------
     
//===========================================================================
//	functions
//===========================================================================

// void POS_CommTimer_Tick();
// void POS_CommTimer_Reset(unsigned int nTime);
// int POS_CommTimer_IsExpired();

// void POS_INIT_VALUES(PosSensorVal* (PCVRSTValue));
// void POS_BUFFER_INIT(PosSensorVal* (PCVRSTValue));

// void POS_REQUEST(PosSensorVal* (PCVRSTValue));
// void POS_DIR_REQUEST(PosSensorVal* (PCVRSTValue));
// void POS_COLOR_REQUEST(PosSensorVal* (PCVRSTValue));
// void POS_REQUEST_SELECTOR(PosSensorVal* (PCVRSTValue));

// int POS_TAG_DETECT(PosSensorVal* (PCVRSTValue));
// int32_t POS_GET_TAG_INFO(PosSensorVal* (PCVRSTValue));
// int16_t POS_GET_ANGLE_INFO(PosSensorVal* (PCVRSTValue));
// double POS_GET_XPOS_INFO(PosSensorVal* (PCVRSTValue));
// double POS_GET_YPOS_INFO(PosSensorVal* (PCVRSTValue));
// int32_t POS_GET_ERR_INFO();
// u_int16_t POS_GET_INFO(PosSensorVal* (PCVRSTValue)); 

// void POS_DIR_CHK(PosSensorVal* (PCVRSTValue));
// void POS_COLOR_CHK(PosSensorVal* (PCVRSTValue));

// u_int16_t POS_CHKSUM_DATA();
// u_int16_t POS_ERR_CHK(u16 *ChkSum_Data);
// u_int16_t POS_DATA_ANALYSIS(PosSensorVal* (PCVRSTValue));
// void POS_ERR_FILTER(PosSensorVal* (PCVRSTValue), u16 *state, u16 filterNum);

// u_int16_t POS_SENSOR_RECEIVE(PosSensorVal* (PCVRSTValue));
// void POSTION_SENSOR_OPERATION(PosSensorVal* (PCVRSTValue));

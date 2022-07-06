//===========================================================================
//	File Name	: PNFPosSensor.h
//	Description	: To use PNF sensors at ease
//===========================================================================



//===========================================================================
//	Include Files
//===========================================================================

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <queue>


//===========================================================================
//	Local Definitions
//===========================================================================

//for Communcication Timeout 
//It is highly recommand timer to config '1ms'
#define POS_TIMEOUT 500

//for Request Comm.
#define CMD_NULL 0X00



//Select Units
//#define Unit_0.1_Milimeters
//#define Unit_1_Milimeters

//===========================================================================
//	Extern func. 
//===========================================================================

//===========================================================================
//	Local Variables
//===========================================================================


namespace Nyamkani
{

     enum OperatingSystem
     {
          ubuntu = 0,
          stm32 = 1,
     };

     enum Communication
     {
          rs485 = 0,
          usb = 1,
          ethernet = 2,
     };


     enum PNFPosCmd
     {
          //--------------------------------------------------PGV100 Commands
          //Write Comm. cmd
          PGV100_Straight_Request = 0,                //for Reqeusting  changing  straight  direction
          PGV100_Left_Request,                        //for Reqeusting  changing  left  direction
          PGV100_Right_Request,                       //for Reqeusting  changing  right direction

          PGV100_Red_Request,                         //for Reqeusting  changing  RED direction
          PGV100_Green_Request,                       //for Reqeusting  changing  GREEN direction
          PGV100_Blue_Request,                        //for Reqeusting  changing  BLUE direction

          PGV100_Pos_Request,                         //for Reqeusting messages    from head to receive POSITON 

          //Write Param. cmd
          PGV100_Change_X_OFFSET,
          PGV100_Change_Y_OFFSET,
          PGV100_Change_ANGLE_OFFSET,
          PGV100_Change_Direction,
          PGV100_Change_Color, 

          //--------------------------------------------------PCV80 Commands
          //Write Comm. cmd
          //PCV80_Pos_Request,                         //for Reqeusting messages    from head to receive POSITON 
          
          
          //Write Comm. cmd

          //Read All params
          PGV100_View_ALL_Read_Val,



     };



     class PNFPosSensor
     {
          public:
               PNFPosSensor();
               ~PNFPosSensor();


          private:
          

     }



}





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

/*
 * PNFPosSensor.h
 *
 *  Created on: Jul 5, 2022
 *      Author: studio3s
 */

#ifndef INC_EXTINC_PNFPOSSENSOR_H_
#define INC_EXTINC_PNFPOSSENSOR_H_

//===========================================================================
//	Include Files
//===========================================================================

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <queue>
#include <thread>

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
//	Global Variables
//===========================================================================

//these four-values must be in stm32f7xx_it.h or .c
std::vector<uint16_t> g_pnf_read_buffer_5;
std::vector<uint16_t> g_pnf_read_buffer_6;
uint16_t g_pnf_buffer_length_5;
uint16_t g_pnf_buffer_length_6;

//===========================================================================
//	Local Variables
//===========================================================================


namespace Nyamkani
{
     enum Communication
     {
          rs485 = 0,
          usb = 1,
          ethernet = 2,
     };

     enum PGV100_Dir
	 {
    	 straight = 0,
		 left = 1,
		 right = 2,
	 };

     enum PGV100_Color
	 {
    	 red = 0,
		 green = 1,
		 blue = 2,
	 };

     enum PNFSensorType
	 {
    	 pcv80 = 0,
		 pgv100 = 1,
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

          //--------------------------------------------------PCV80 Commands
		  PCV80_Pos_Requset,                    //for Reqeusting messages    from head to receive POSITON


          //Write Comm. cmd

          //Read All params
          PGV100_View_ALL_Read_Val,
     };

	class PNFPosSensor
	{
		  public:
			   //Basic constructor
			   PNFPosSensor();

			   PNFPosSensor(uint8_t PNFSensorType, uint8_t Commtype, uint8_t Port,
						 double X_Offset, double Y_Offset, double Angle_Offset);

			   ~PNFPosSensor();

		  private:
				//---------------------------------------------------------------------------pgv100 output. declation
				//total Number of sensors
				//static int Total_PNF_Sensor_Num;

				//to see useful values
				double xpos_;
				double ypos_;
				double angle_;
				uint16_t tagNo_;
				uint8_t dir_;
				uint8_t color_;
				uint32_t err_;

				//---------------------------------------------------------------------------pgv100 parameters. declation
				//params
				double  x_offset_;
				double  y_offset_;
				double angle_offset_;
				//uint8_t OStype_;
				uint8_t comm_type_;
				uint8_t port_;
				uint8_t sensor_type_;
				//Receive Buffer

				std::vector<uint16_t>* pos_buf_ = NULL;      // Response POS data buffer
				uint16_t* pos_read_buffer_length_ = NULL;

				//Limitation
				int32_t POS_AREA_MAX = 10000;      // Max. range of tape value                                       // PCV센서 범위 Maximum(mm)
				int32_t POS_AREA_MIN = (-100);   // Min. range of tape value

				//To check vaild Condition
				uint8_t NomCnd_Cnt = 0;
				uint8_t ErrCnd_Cnt = 0;


				//---------------------------------------------------------------------------485 Comm. cmds declation
				//Values for request cmd
				std::vector<std::string> RequestCmd;
				std::queue<int> RequestQueue;

				///First Time setup
				void ConstructRequsetCmd();
				void ConstructCommunicationSetup();
				void ConstructDefaultParam();

				//Initialization for work-loop
				void Init_Read_Buffer();

				//-----------------------------------------------------------------------Change parameters

				void Change_Buffer_size(uint16_t Buffer_Size);
				PNFPosSensor& Change_XOffset(double X_Offset);
				PNFPosSensor& Change_YOffset(double Y_Offset);
				PNFPosSensor& Change_Angle_Offset(double Angle_Offset);


				//----------------------------------------------------------------------Requset command
				void Request_Change_Dir_straight();
				void Request_Change_Dir_left();
				void Request_Change_Dir_right();

				void Request_Change_Color_yellow();
				void Request_Change_Color_red();
				void Request_Change_Color_blue();

				void Request_Get_PGV100_Pos();
				void Request_Get_PCV80_Pos();


				//---------------------------------------------------------------return value functions
				double Get_XPos() const;
				double Get_YPos() const;
				double Get_Angle() const;
				uint16_t Get_TagNo() const;
				uint8_t Get_Dir() const;
				uint8_t Get_Color() const;
				uint32_t Get_SensorErr() const;

				double X_Offset() const;
				double Y_Offset() const;
				double Angle_Offset() const;
				//uint8_t OStype() const;
				uint8_t Commtype() const;
				uint8_t Port() const;
				uint8_t PNFSensorType() const;

				//---------------------------------------------------------------Command queue functions
				//queue system functions
				void Queue_Save_Request(int cmd);
				void Queue_Delete_Request();

				//---------------------------------------------------------------send or read functions
				void Work_Send_Request();
				void Work_Receive_Response();

				//---------------------------------------------------------------Processing data
				uint16_t Process_Checksum_Data();
				uint16_t Process_Check_Err();



				bool Process_Is_Tag_Detected();
				uint16_t Process_Get_Tag_Number();
				double Process_Get_Angle_Info();
				double Process_Get_XPos_Info();
				double Process_Get_YPos_Info();
				uint32_t Process_Get_ERR_Info();
				uint16_t Process_Get_Total_Info();

				int main_loop();

     };

}






#endif /* INC_EXTINC_PNFPOSSENSOR_H_ */

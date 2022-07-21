/*
 * PNFPosSensor.h
 *
 *  Created on: Jul 5, 2022
 *      Author: studio3s
 */

#ifndef INC_EXTINC_PNFPOSSENSOR_H_
#define INC_EXTINC_PNFPOSSENSOR_H_

#pragma once
//===========================================================================
//	Include Files
//===========================================================================

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <queue>
//#include <thread>

//===========================================================================
//	Local Definitions
//===========================================================================


//===========================================================================
//	Extern func.
//===========================================================================


//===========================================================================
//	Global Variables
//===========================================================================



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

     enum Port
     {
          port_5 = 5,
		  port_6 = 6,
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

     enum PNFBufferLength
	 {
    	 pgv100color = 2,
		 pgv100dir = 3,
		 pcv80pos = 9,
		 pgv100pos = 21,
	 };

     enum PNFErrorList
	 {
	     //STATE
    	  good = 0x0000,
		  readheadtilted = 0x0001,  		//Read head tilted 180°.(pcv80 only)
		  codeconditionerr = 0x0002, 		//code condition error(code distance chk)
	      nodirectiondeclare = 0x0004,    	//No DIR. decision(Set POS.Sensor DIR.)
	      nocolordeclare = 0x0008,  		//No Color decision(Set Color choice)
	      outofrange = 0x0010,      		//Out of Range
	      noposition = 0x0020,      		//No Position
	      commtimeout = 0x0040,  			//Timeout(communication error)
	      checksumerr = 0x0080,  			//chk_sum error ;
	      internalFatal = 0x1000,  			//internal error (Recommend to change sensors)
	      //0x2000 = reserved
	      //0x4000 = reserved
	      //0x8000 = reserved
	     //--------------------------------------------------------------------
	 };


     enum PNFPosCmd
     {
          //--------------------------------------------------PGV100 Commands
          //Write Comm. cmd
          PGV100_Straight_Request = 0,                //for Reqeusting  changing  straight  direction
          PGV100_Left_Request = 1,                        //for Reqeusting  changing  left  direction
          PGV100_Right_Request= 2,                       //for Reqeusting  changing  right direction

          PGV100_Red_Request = 3,                         //for Reqeusting  changing  RED direction
          PGV100_Green_Request = 4,                       //for Reqeusting  changing  GREEN direction
          PGV100_Blue_Request = 5,                        //for Reqeusting  changing  BLUE direction

          PGV100_Pos_Request = 6,                         //for Reqeusting messages    from head to receive POSITON

          //--------------------------------------------------PCV80 Commands
		  PCV80_Pos_Requset = 7,                    //for Reqeusting messages    from head to receive POSITON


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
				double xpos_ = 0;
				double ypos_ = 0;
				double angle_ = 0;
				uint16_t tagNo_ = 0;
				uint8_t dir_ = 0;
				uint8_t color_ = 0;
				uint16_t err_ = 0;

				//---------------------------------------------------------------------------pgv100 parameters. declation
				//params
				double  x_offset_;
				double  y_offset_;
				double angle_offset_;
				//uint8_t OStype_;
				uint8_t comm_type_;
				uint8_t port_;
				uint8_t sensor_type_;
				uint8_t max_filter_cnt_ = 5;
				uint8_t now_filter_cnt_= 0;

				//uart transmit Buffer
				USART_TypeDef *USARTx = NULL;

				//Receive Buffer
				std::vector<uint16_t>* pos_buf_ = NULL;      // Response POS data buffer
				uint16_t* pos_read_buffer_counter_ = NULL;
				uint16_t* comm_time_ = NULL;
				uint16_t max_read_buf_size_;
				uint16_t reset_time_= 0;


				//Limitation
				int32_t POS_AREA_MAX = 10000;      // Max. range of tape value                                       // PCV센서 범위 Maximum(mm)
				int32_t POS_AREA_MIN = (-100);   // Min. range of tape value

				//---------------------------------------------------------------------------485 Comm. cmds declation
				//Values for request cmd
				std::vector<std::string> RequestCmd;
				std::queue<int> RequestQueue;

				///First Time setup
				void RegisterRequsetCmd();
				void RegisterCommunicationSetup();
				void RegisterDefaultParam();

				//Initialization for work-loop
				void InitReadBuffer();

				//void CommTimerTick();
				void CommTimerReset();
				bool CommTimerIsExpired();


				//---------------------------------------------------------------Command queue functions
				//queue system functions
				void QueueSaveRequest(int cmd);
				void QueueDeleteRequest();
				void QueueRepeatPosReqeust();

				//---------------------------------------------------------------send or read functions
				void WorkSendRequest();
				void WorkReceiveResponse();

				//---------------------------------------------------------------Processing data
				uint16_t ProcessChecksumData(std::vector<uint16_t> temp_buf);
				uint16_t ProcessCheckErr(std::vector<uint16_t> temp_buf);



				bool ProcessIsTagDetected(std::vector<uint16_t> temp_buf);
				uint16_t ProcessGetTagNumber(std::vector<uint16_t> temp_buf);
				double ProcessGetAngleInfo(std::vector<uint16_t> temp_buf);
				double ProcessGetXPosInfo(std::vector<uint16_t> temp_buf);
				double ProcessGetYPosInfo(std::vector<uint16_t> temp_buf);
				uint8_t ProcessGetDirectionInfo(std::vector<uint16_t> temp_buf);
				uint8_t ProcessGetColorInfo(std::vector<uint16_t> temp_buf);

				bool IsValueFiltered();
				void FilterCountUp();
				void FilterStatusChanged();

				uint32_t ProcessGetERRInfo(std::vector<uint16_t> temp_buf);
				uint16_t ProcessGetTotalInfo();

		  public:
				//public
				//---------------------------------------------------------------return value functions
				double GetXPos() const;
				double GetYPos() const;
				double GetAngle() const;
				uint16_t GetTagNo() const;
				uint8_t GetDir() const;
				uint8_t GetColor() const;
				uint32_t GetSensorErr() const;

				double GetXOffset() const;
				double GetYOffset() const;
				double GetAngleOffset() const;
				//uint8_t OStype() const;
				uint8_t GetCommtype() const;
				uint8_t GetPort() const;
				uint8_t GetPNFSensorType() const;
				//-----------------------------------------------------------------------Change parameters

				//void ChangeBuffersize(uint16_t Buffer_Size);
				PNFPosSensor& SetXOffset(double X_Offset);
				PNFPosSensor& SetYOffset(double Y_Offset);
				PNFPosSensor& SetAngleOffset(double Angle_Offset);


				//----------------------------------------------------------------------Requset command
				void RequestChangeDirstraight();
				void RequestChangeDirleft();
				void RequestChangeDirright();

				void RequestChangeColoryellow();
				void RequestChangeColorred();
				void RequestChangeColorblue();

				void RequestGetPGV100Pos();
				void RequestGetPCV80Pos();

				//main
				uint16_t Initializaition();
				uint16_t main_loop();
     };

}






#endif /* INC_EXTINC_PNFPOSSENSOR_H_ */

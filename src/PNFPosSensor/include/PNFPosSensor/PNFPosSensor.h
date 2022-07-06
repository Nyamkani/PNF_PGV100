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

			   PNFPosSensor(int Commtype, int Port,
						 double X_Offset, double Y_Offset, double Angle_Offset);

			   ~PNFPosSensor();

			   void main_loop();

		  private:
			   //---------------------------------------------------------------------------pgv100 output. declation
			   //total Number of sensors
			   static uint8_t Total_PNF_Sensor_Num;

			   //to see useful values
			   double XPos_;
			   double YPos_;
			   double Angle_;
			   uint16_t tagNo_;
			   uint8_t NowDir_;
			   uint8_t NowColor_;
			   uint32_t SensorErr_;

			   //---------------------------------------------------------------------------pgv100 parameters. declation
			   //params
			   double  X_Offset_;
			   double  Y_Offset_;
			   double Angle_Offset_;
			   int OStype_;
			   int Commtype_;
			   int Port_;
			   int PNFSensorType_;
			   //Receive Buffer

			   std::vector<uint16_t> POS_BUF_;      // Response POS data buffer


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
			   void Construct_Requset_Cmd();
			   void Construct_Communication_Setup();
			   void Construct_Default_Param();

				//Initialization for work-loop
				void PNFPosSensor::Init_Read_Buffer();


				//-----------------------------------------------------------------------Change parameters
				PNFPosSensor& Change_XOffset(double X_Offset);
				PNFPosSensor& Change_YOffset(double Y_Offset);
				PNFPosSensor& Change_Angle_Offset(double Angle_Offset);


				//----------------------------------------------------------------------Requset command
				void Change_Dir_straight();
				void Change_Dir_left();
				void Change_Dir_right();

				void Change_Color_yellow();
				void Change_Color_red();
				void Change_Color_blue();

				void Get_Pos();



				//to see useful values
				double Get_XPos_() const;
				double Get_YPos_() const;
				double Get_Angle_() const;
				uint16_t Get_TagNo_() const;
				uint8_t Get_Dir() const;
				uint8_t Get_Color() const;
				uint32_t Get_SensorErr_() const;

			   void PNFPosSensor::Queue_Save_Request(int cmd);
			   void PNFPosSensor::Queue_Delete_Request();
			   void Work_Receive_Response();






     };

}






#endif /* INC_EXTINC_PNFPOSSENSOR_H_ */

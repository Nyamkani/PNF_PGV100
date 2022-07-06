/*
 * PNFPosSensor.cpp
 *
 *  Created on: Jul 5, 2022
 *      Author: studio3s
 */


#include <Extinc/PNFPosSensor.h>
#include "Extinc/api_init.h"

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "lwip.h"


namespace Nyamkani
{
	//--------------------------------------------------------------------------------------------------class PNFPosSensor:
	//Consturctors
	PNFPosSensor::PNFPosSensor()
	{
		X_Offset_ = 0.0;
		Y_Offset_ = (0.0);
		Angle_Offset_ = (0.0);
		Total_PNF_Sensor_Num++;
	}

	PNFPosSensor::PNFPosSensor(int PNFSensorType, int Commtype, int Port,
			 double X_Offset, double Y_Offset, double Angle_Offset)
	{
		PNFSensorType_ = PNFSensorType;
		Commtype_ = Commtype;
		Port_ = Port;
		X_Offset_ = X_Offset;
		Y_Offset_ = Y_Offset;
		Angle_Offset_ = Angle_Offset;
		Total_PNF_Sensor_Num++;
	}

	PNFPosSensor::~PNFPosSensor(){Total_PNF_Sensor_Num--;}

	//----------------------------------------------------------------------------------------Functions
	/*main frame*/
	/*Construct level - network, first parmas. declation -> Construct_xxx*/
	/*Initialize level - init buffers, check cmd -> Init_xxx*/
	/*Work level - write and read -> Work_xxx*/
	/*Parsing and Store level - parsing and check errors->Process_xxx*/
	/*---------------------------------------------------*/
	/*Queue - queue systems ->Queue*/

	//--------------------------------------------------------------Construct level - network, first parmas. declation
	void PNFPosSensor::Construct_Requset_Cmd()
	{
		RequestCmd[PGV100_Straight_Request] = "EC13";
		RequestCmd[PGV100_Left_Request] = "E817";
		RequestCmd[PGV100_Right_Request] = "E41B";
		RequestCmd[PGV100_Red_Request] = "906F";
		RequestCmd[PGV100_Green_Request] = "880";
		RequestCmd[PGV100_Blue_Request] = "C43B";
		RequestCmd[PGV100_Pos_Request] = "C837";
		RequestCmd[PCV80_Pos_Requset] = "A05F";
	}

	void PNFPosSensor::Construct_Communication_Setup()
	{
		if(this->Port_ == 5) MX_UART5_Init();
		else  MX_USART6_UART_Init();
	}

	void PNFPosSensor::Construct_Default_Param()
	{
		//Change_XOffset(1);
		//Change_YOffset(1);
		//Change_Angle_Offset(1);
		Change_Dir_straight();

	}


	//Initialization for work-loop
	void PNFPosSensor::Init_Read_Buffer()
	{
		uint16_t Buffer_Size = 0;
		int& cmdstr = RequestQueue.front();
		if(cmdstr == PGV100_Pos_Request) {Buffer_Size = 21;}
		else if (cmdstr == PCV80_Pos_Requset){Buffer_Size = 8;}
		else {Buffer_Size = 3;}

		POS_BUF_.assign(Buffer_Size,0);
	}



	//--------------------------------------------------------------Change parameters
	//Common use
	void PNFPosSensor::Change_Buffer_size(uint16_t Buffer_Size) {this->Buffer_Size_ = Buffer_Size;}

	//param use
	void PNFPosSensor::Change_XOffset(double X_Offset) {this->X_Offset_ = X_Offset;}
	void PNFPosSensor::Change_YOffset(double Y_Offset) {this->Y_Offset_ = Y_Offset;}

	//pgv100 param only
	void PNFPosSensor::Change_Angle_Offset(double Angle_Offset)  { this->Angle_Offset_ = Angle_Offset;}


	//---------------------------------------------------------------Requset command
	//pgv100 only
	void PNFPosSensor::Change_Dir_straight() {Queue_Save_Request(PGV100_Straight_Request);}
	void PNFPosSensor::Change_Dir_left() {Queue_Save_Request(PGV100_Left_Request);}
	void PNFPosSensor::Change_Dir_right() {Queue_Save_Request(PGV100_Right_Request);}

	void PNFPosSensor::Change_Color_yellow()  {Queue_Save_Request(PGV100_Red_Request);}
	void PNFPosSensor::Change_Color_red()  {Queue_Save_Request(PGV100_Green_Request);}
	void PNFPosSensor::Change_Color_blue()  {Queue_Save_Request(PGV100_Blue_Request);}

	void PNFPosSensor::Get_PGV100_Pos()  {Queue_Save_Request(PGV100_Pos_Request);}

	//pcv80 only
	void PNFPosSensor::Get_PCV80_Pos()  {Queue_Save_Request(PCV80_Pos_Requset);}



	//---------------------------------------------------------------return value functions
	double PNFPosSensor::Get_XPos_() const {return XPos_;}
	double PNFPosSensor::Get_YPos_() const {return YPos_;}
	double PNFPosSensor::Get_Angle_() const {return Angle_;}
	uint16_t PNFPosSensor::Get_TagNo_() const {return tagNo_;}
	uint8_t PNFPosSensor::Get_Dir() const {return NowDir_;}
	uint8_t PNFPosSensor::Get_Color() const {return NowColor_;}
	uint32_t PNFPosSensor::Get_SensorErr_() const {return SensorErr_;}



	//---------------------------------------------------------------Command queue functions
	//queue system functions
	void PNFPosSensor::Queue_Save_Request(int cmd){this->RequestQueue.push(cmd);}
	void PNFPosSensor::Queue_Delete_Request(){this->RequestQueue.pop();}


	//---------------------------------------------------------------send or read functions
	void PNFPosSensor::Work_Send_Request()
	{
		/*write functions start*/
		/*Usart_Transmit(USARTx, RequestCmd[RequestQueue.front()], sizeof(RequestCmd[RequestQueue.front()]))*/
		/*write functions end*/
	}

	void PNFPosSensor::Work_Receive_Response()
	{
		/*write functions*/
		/*Usart_Transmit()*/

	}









	int PNFPosSensor::main_loop()
	{
		try
		{
			Init_Read_Buffer();
			Work_Send_Request();
			while(1/*when interrupt count reached ()*/)
			{
				Work_Receive_Response();
			}
			//Process_Data();
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
			//Error handling
		}
	}



}

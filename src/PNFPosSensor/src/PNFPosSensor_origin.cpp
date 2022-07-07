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
		Y_Offset_ = 0.0;
		Angle_Offset_ = 0.0;
		//Total_PNF_Sensor_Num++;
	}

	PNFPosSensor::PNFPosSensor(uint8_t PNFSensorType, uint8_t Commtype, uint8_t Port,
			 double X_Offset, double Y_Offset, double Angle_Offset)
	{
		PNFSensorType_ = PNFSensorType;
		Commtype_ = Commtype;
		Port_ = Port;
		X_Offset_ = X_Offset;
		Y_Offset_ = Y_Offset;
		Angle_Offset_ = Angle_Offset;
		//Total_PNF_Sensor_Num++;
	}

	PNFPosSensor::~PNFPosSensor(){/*Total_PNF_Sensor_Num--;*/}

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
		Request_Change_Dir_straight();

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
	//void PNFPosSensor::Change_Buffer_size(uint16_t Buffer_Size) {this->Buffer_Size_ = Buffer_Size;}

	//param use
	PNFPosSensor& PNFPosSensor::Change_XOffset(double X_Offset)
	{
		this->X_Offset_ = X_Offset;
		return* this;
	}

	PNFPosSensor& PNFPosSensor::Change_YOffset(double Y_Offset)
	{
		this->Y_Offset_ = Y_Offset;
		return* this;
	}

	//pgv100 param only
	PNFPosSensor& PNFPosSensor::Change_Angle_Offset(double Angle_Offset)
	{
		this->Angle_Offset_ = Angle_Offset;
		return* this;
	}


	//---------------------------------------------------------------Request command
	//pgv100 only
	void PNFPosSensor::Request_Change_Dir_straight() {Queue_Save_Request(PGV100_Straight_Request);}
	void PNFPosSensor::Request_Change_Dir_left() {Queue_Save_Request(PGV100_Left_Request);}
	void PNFPosSensor::Request_Change_Dir_right() {Queue_Save_Request(PGV100_Right_Request);}

	void PNFPosSensor::Request_Change_Color_yellow() {Queue_Save_Request(PGV100_Red_Request);}
	void PNFPosSensor::Request_Change_Color_red() {Queue_Save_Request(PGV100_Green_Request);}
	void PNFPosSensor::Request_Change_Color_blue() {Queue_Save_Request(PGV100_Blue_Request);}

	void PNFPosSensor::Request_Get_PGV100_Pos() {Queue_Save_Request(PGV100_Pos_Request);}

	//pcv80 only
	void PNFPosSensor::Request_Get_PCV80_Pos() {Queue_Save_Request(PCV80_Pos_Requset);}



	//---------------------------------------------------------------return value functions
	double PNFPosSensor::Get_XPos() const {return XPos_;}
	double PNFPosSensor::Get_YPos() const {return YPos_;}
	double PNFPosSensor::Get_Angle() const {return Angle_;}
	uint16_t PNFPosSensor::Get_TagNo() const {return tagNo_;}
	uint8_t PNFPosSensor::Get_Dir() const {return NowDir_;}
	uint8_t PNFPosSensor::Get_Color() const {return NowColor_;}
	uint32_t PNFPosSensor::Get_SensorErr() const {return SensorErr_;}

	double PNFPosSensor::X_Offset() const {return X_Offset_;}
	double PNFPosSensor::Y_Offset() const {return Y_Offset_;};
	double PNFPosSensor::Angle_Offset() const {return Angle_Offset_;}
	//uint8_t PNFPosSensor::OStype() const {return OStype_;};
	uint8_t PNFPosSensor::Commtype() const {return Commtype_;}
	uint8_t PNFPosSensor::Port() const {return Port_;}
	uint8_t PNFPosSensor::PNFSensorType() const {return PNFSensorType_;}




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

	//---------------------------------------------------------------Processing data
	uint16_t PNFPosSensor::Process_Checksum_Data()
	{
		 uint16_t i = 0;
		 uint16_t j = 0;
		 uint16_t temp = 0;
		 uint16_t ChkSum_Data = 0;
		 uint16_t Buffer_Size = POS_BUF_.size();
		 uint16_t even_cnt[Buffer_Size]={0,};

	     for(i=0;i<(Buffer_Size-1);i++) //Buf 0~20 21��.
	     {
	          temp = POS_BUF_[i];
	          for(j=0; j<8; j++) if((temp>>j)&0x01) even_cnt[j]+=1;//8bit, even
	     }
	     for(i=0;i<(Buffer_Size-1);i++)
	     {
	          if((even_cnt[i]&0x01)==0x01) ChkSum_Data|= ((uint16_t)1 << i);
	     }
	     return ChkSum_Data;
	}

	uint16_t PNFPosSensor::Process_Check_Err()
	{
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
		 uint16_t state = 0x0000;

	     if(POS_BUF_[20] == Process_Checksum_Data())    //Checksum error pass or not(POS_BUF[20] <--- check sum buffer)
	     {
	          if((POS_BUF_[0]&0x01)==0x01)    //Err Occured
	          {
	               uint16_t errcode =  Process_Get_ERR_Info();
	               if(errcode>1000) state |= 0x1000;        //Internal Fatal Error  (��ü)
	               else if(errcode==2) state |= 0x0002;     //code condition error(code distance chk)
	               else if(errcode==5) state |= 0x0004;     //No clear position can be determined(�Ÿ�����)
	               else if(errcode==6) state |= 0x0008;     // No Color decision(Set Color choice)
	          }
	          else if((POS_BUF_[0]&0x02)) state |= 0x0020;    //No Position Error
	     }
	     else state |= 0x0080;        //check sum error
	     //if(POS_CommTimer_IsExpired()) state |= 0x0040;        //Timeout(communication error)
	     return state;
	}




	bool PNFPosSensor::Process_Is_Tag_Detected()
	{
		if((POS_BUF_[1] & 0x40) == 1) return true;
		else return false;
	}

	uint16_t PNFPosSensor::Process_Get_Tag_Number()
	{
	     int16_t TagNum = 0;
		 (TagNum)=(int32_t)POS_BUF_[17];
		 (TagNum)|=(int32_t)POS_BUF_[16]<<7;
		 (TagNum)|=(int32_t)POS_BUF_[15]<<14;
		 (TagNum)|=(int32_t)POS_BUF_[14]<<21;
		 return TagNum;
	}

	double PNFPosSensor::Process_Get_Angle_Info()
	{
		  uint16_t ANGLE=(uint16_t)POS_BUF_[11];
	     (ANGLE)|=(uint16_t)POS_BUF_[10] << 7;

	     (ANGLE)=(uint16_t)(((ANGLE)/10));
	     if((ANGLE)> 180.0f) ANGLE-=360.0f; //makes x-axis zero centered
	     return ANGLE + this->Angle_Offset_;
	}

	double PNFPosSensor::Process_Get_XPos_Info()
	{
	     double XPOS;

	     int32_t XPosition_DATA=(int32_t)POS_BUF_[5];
	     (XPosition_DATA)|=(int32_t)POS_BUF_[4]<<7;
	     (XPosition_DATA)|=(int32_t)POS_BUF_[3]<<14;
	     (XPosition_DATA)|=(int32_t)(POS_BUF_[2]&0x07)<<21;
	     XPOS=(double)(XPosition_DATA/(10000.0f));                   //To make units milimeters to meters

	     //for making X-axis center to zero
	     if(PNFSensorType_== pgv100)
	     {
	    	 if(XPOS>=10) XPOS = (double)(XPOS-((double)(pow(2,24)-1)/(10000.0f))-(this->X_Offset_));
	    	 else XPOS = (XPOS-(this->X_Offset_));
	     }

		if(XPOS >= this->POS_AREA_MIN && XPOS <= this->POS_AREA_MAX) return XPOS;
		else return this->XPos_;  //	  else { state |= 0x0010;} //Out of Range
	}

	double PNFPosSensor::Process_Get_YPos_Info()
	{
	     double YPOS;

	     int32_t YPosition_DATA=(int32_t)POS_BUF_[7];//Y Buf
	     (YPosition_DATA)|=((int32_t)POS_BUF_[6])<<7;
	     YPOS=(double)(YPosition_DATA/(10000.0f));              //To make units milimeters to meters

	     //for making Y-axis center to zero
	     if(PNFSensorType_== pgv100)
	     {
			 if(YPOS>=0.1) YPOS = (double)(YPOS-((double)(16383.0)/(10000.0f))-(this->Y_Offset_));
			 else YPOS = (YPOS-(this->Y_Offset_));
	     }
	     return YPOS;
	}

	uint32_t PNFPosSensor::Process_Get_ERR_Info()
	{
	   uint32_t ERR_DATA = (int32_t)POS_BUF_[5];
	  (ERR_DATA)|=(int32_t)POS_BUF_[4]<<7;
	  (ERR_DATA)|=(int32_t)POS_BUF_[3]<<14;
	  (ERR_DATA)|=(int32_t)(POS_BUF_[2]&0x07)<<21;
	  return ERR_DATA;
	}

	uint16_t PNFPosSensor::Process_Get_Total_Info()
	{
		uint16_t state = 0;

		if(Process_Is_Tag_Detected()) this->tagNo_ = Process_Get_Tag_Number(); 	  //--- Get TAG INFO
		else this->tagNo_ = 0;

		this->Angle_ = Process_Get_Angle_Info(); 	 			  //--- Get ANGLE INFO
		this->XPos_ = Process_Get_XPos_Info(); 	 				  //--- Get X POSITION
		this->YPos_ = Process_Get_YPos_Info(); 					  //--- Get Y POSITION
		return state;

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
		catch(...)
		{
			//std::cerr << e.what() << '\n';
			//Error handling
		}
	}



}

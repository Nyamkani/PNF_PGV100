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

/* Global Variables ------------------------------------------------------------------*/


/* Local Variables ------------------------------------------------------------------*/





namespace Nyamkani
{
	//--------------------------------------------------------------------------------------------------class PNFPosSensor:
	//Consturctors
	PNFPosSensor::PNFPosSensor()
	{
		x_offset_ = 0.0;
		y_offset_ = 0.0;
		angle_offset_ = 0.0;
		//Total_PNF_Sensor_Num++;
	}

	PNFPosSensor::PNFPosSensor(uint8_t PNFSensorType, uint8_t Commtype, uint8_t Port,
			 double X_Offset, double Y_Offset, double Angle_Offset)
	{
		sensor_type_ = PNFSensorType;
		comm_type_ = Commtype;
		port_ = Port;
		x_offset_ = X_Offset;
		y_offset_ = Y_Offset;
		angle_offset_ = Angle_Offset;
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
	void PNFPosSensor::ConstructRequsetCmd()
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

	void PNFPosSensor::ConstructCommunicationSetup()
	{
		if(this->port_ == 5)
		{
			MX_UART5_Init();
			if(pos_buf_ == nullptr) pos_buf_ = &g_pnf_read_buffer_5;
			if(pos_read_buffer_length_ == nullptr) pos_read_buffer_length_ = &g_pnf_buffer_length_5;
		}
		else if(this->port_ == 6)
		{
			MX_USART6_UART_Init();
			if(pos_buf_ == nullptr) pos_buf_ = &g_pnf_read_buffer_6;
			if(pos_read_buffer_length_ == nullptr) pos_read_buffer_length_ = &g_pnf_buffer_length_6;
		}
		//else {throw }
	}

	void PNFPosSensor::ConstructDefaultParam()
	{
		//Change_XOffset(1);
		//Change_YOffset(1);
		//Change_Angle_Offset(1);
		Request_Change_Dir_straight();

	}


	//Initialization for work-loop
	void PNFPosSensor::Init_Read_Buffer()
	{
		int& cmdstr = RequestQueue.front();
		if(cmdstr == PGV100_Pos_Request) {*pos_read_buffer_length_ = pgv100pos;}
		else if (cmdstr == PCV80_Pos_Requset){*pos_read_buffer_length_ = pcv80pos;}
		else if (cmdstr == (PGV100_Straight_Request|PGV100_Left_Request|PGV100_Right_Request)) {*pos_read_buffer_length_ = pgv100dir;}
		else {*pos_read_buffer_length_ = pgv100color;}
		pos_buf_->assign(*pos_read_buffer_length_,0);
	}

	//bool PNFPosSintensor::CommTimeOut()
	//{
//
	//	return
	//}

	//uint16_t PNFPosSintensor::CommTimeReduce()
	//{
//
	//	return
	//}


	//--------------------------------------------------------------Change parameters
	//Common use
	//void PNFPosSensor::Change_Buffer_size(uint16_t Buffer_Size) {this->Buffer_Size_ = Buffer_Size;}

	//param use
	PNFPosSensor& PNFPosSensor::Change_XOffset(double X_Offset)
	{
		this->x_offset_ = X_Offset;
		return* this;
	}

	PNFPosSensor& PNFPosSensor::Change_YOffset(double Y_Offset)
	{
		this->y_offset_ = Y_Offset;
		return* this;
	}

	//pgv100 param only
	PNFPosSensor& PNFPosSensor::Change_Angle_Offset(double Angle_Offset)
	{
		this->angle_offset_ = Angle_Offset;
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
	double PNFPosSensor::Get_XPos() const {return xpos_;}
	double PNFPosSensor::Get_YPos() const {return ypos_;}
	double PNFPosSensor::Get_Angle() const {return angle_;}
	uint16_t PNFPosSensor::Get_TagNo() const {return tagNo_;}
	uint8_t PNFPosSensor::Get_Dir() const {return dir_;}
	uint8_t PNFPosSensor::Get_Color() const {return color_;}
	uint32_t PNFPosSensor::Get_SensorErr() const {return err_;}

	double PNFPosSensor::X_Offset() const {return x_offset_;}
	double PNFPosSensor::Y_Offset() const {return y_offset_;};
	double PNFPosSensor::Angle_Offset() const {return angle_offset_;}
	//uint8_t PNFPosSensor::OStype() const {return OStype_;};
	uint8_t PNFPosSensor::Commtype() const {return comm_type_;}
	uint8_t PNFPosSensor::Port() const {return port_;}
	uint8_t PNFPosSensor::PNFSensorType() const {return sensor_type_;}




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
		 uint16_t temp = 0;
		 uint16_t ChkSum_Data = 0;
		 uint16_t even_cnt[*pos_read_buffer_length_]={0,};

		for(uint8_t i=0; i<7; i++)
		{
			for (std::vector<uint16_t>::iterator itr = pos_buf_->begin(); itr != pos_buf_->end()-1; ++itr)
			{
			   temp = *itr;
			   if((temp>>i)&0x01) even_cnt[i]+=1;//8bit, even
			}
			ChkSum_Data |= (even_cnt[i]%2) * (1<<i);
		}
	     return ChkSum_Data;
	}

	uint16_t PNFPosSensor::Process_Check_Err()
	{
	      //------------------------------------------------------------------
	     //STATE
	     // 0x0000 = Good
	     // 0x0001 = Read head tilted 180°.(pcv80 only)
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

	     if((*pos_buf_)[(*pos_read_buffer_length_)-1] == Process_Checksum_Data())    //Checksum error pass or not(POS_BUF[20] <--- check sum buffer)
	     {
	          if((*pos_buf_)[0] & 0x01)    //Err Occured
	          {
	               uint16_t errcode =  Process_Get_ERR_Info();
	               if(errcode>1000) state |= internalFatal;        //Internal Fatal Error
	               else if(errcode==1) state |= readheadtilted;     //read head tilted 180°.(pcv80 only)
	               else if(errcode==2) state |= codeconditionerr;     //code condition error(code distance chk)
				   else if(errcode==5) state |= nodirectiondeclare;     //No clear position can be determined(�Ÿ�����)
				   else if(errcode==6) state |= nocolordeclare;     // No Color decision(Set Color choice)
	          }
	          else if((*pos_buf_)[0]&0x02) state |= noposition;    //No Position Error
	     }
	     else state |= checksumerr;        //check sum error
	     //if(POS_CommTimer_IsExpired()) state |= commtimeout;        //Timeout(communication error)
	     return state;
	}

	bool PNFPosSensor::Process_Is_Tag_Detected()
	{
		if((*pos_buf_)[1] & 0x40) return true;
		else return false;
	}

	uint16_t PNFPosSensor::Process_Get_Tag_Number()
	{
	     int16_t TagNum = 0;
		 (TagNum)=(int32_t)(*pos_buf_)[17];
		 (TagNum)|=(int32_t)(*pos_buf_)[16]<<7;
		 (TagNum)|=(int32_t)(*pos_buf_)[15]<<14;
		 (TagNum)|=(int32_t)(*pos_buf_)[14]<<21;
		 return TagNum;
	}

	double PNFPosSensor::Process_Get_Angle_Info()
	{
		  uint16_t ANGLE=(uint16_t)(*pos_buf_)[11];
	     (ANGLE)|=(uint16_t)(*pos_buf_)[10] << 7;

	     (ANGLE)=(uint16_t)(((ANGLE)/10));
	     if((ANGLE)> 180.0f) ANGLE-=360.0f; //makes x-axis zero centered
	     return ANGLE + this->angle_offset_;
	}

	double PNFPosSensor::Process_Get_XPos_Info()
	{
		uint8_t a,b,c,d;
		if(sensor_type_== pgv100){a = 4; b = 3; c = 2; d = 1;}
		else if (sensor_type_== pcv80){a = 5; b = 4; c = 3; d = 2;}

		int32_t XPosition_DATA=(int32_t)(*pos_buf_)[a];
		(XPosition_DATA)|=(int32_t)(*pos_buf_)[b] << 7;
		(XPosition_DATA)|=(int32_t)(*pos_buf_)[c] << 14;
		(XPosition_DATA)|=(int32_t)((*pos_buf_)[d]&0x07) << 21;
		double XPOS=(double)(XPosition_DATA/(10000.0f));                   //To make units milimeters to meters

	     //for making X-axis center to zero
	     if(sensor_type_== pgv100)
	     {
	    	 if(XPOS>=10) XPOS = (double)(XPOS-((double)(pow(2,24)-1)/(10000.0f))-(this->x_offset_));
	    	 else XPOS = (XPOS-(this->x_offset_));
	     }

		if(XPOS >= this->POS_AREA_MIN && XPOS <= this->POS_AREA_MAX) return XPOS;
		else return this->xpos_;  //	  else { state |= 0x0010;} //Out of Range
	}

	double PNFPosSensor::Process_Get_YPos_Info()
	{
		int32_t YPosition_DATA=(int32_t)(*pos_buf_)[7];//Y Buf
		(YPosition_DATA)|=((int32_t)(*pos_buf_)[6]) << 7;
		double YPOS=(double)(YPosition_DATA/(10000.0f));              //To make units milimen));

		//for making Y-axis center to zero
		if(this->sensor_type_== pgv100)
		{
			if(YPOS>=0.1) YPOS = (double)(YPOS-((double)(16383.0)/(10000.0f))-(this->y_offset_));
			else YPOS = (YPOS-(this->y_offset_));
		}
		return YPOS;
	}

	//for Chking Modes
	uint8_t PNFPosSensor::Process_Get_Direction_Info()
	{

		if(Process_Is_Tag_Detected()) {return (uint8_t)(*pos_buf_)[1]&0x03;}
		else return this_>dir_;
	}

	uint8_t PNFPosSensor::Process_Get_Color_Info()
	{
		if((*pos_buf_)[0]&0x07 && (*pos_buf_)[1]&0x07) return (uint8_t)(*pos_buf_)[1]&0x07;
		else return this->color_;
	}

	uint32_t PNFPosSensor::Process_Get_ERR_Info()
	{
	   uint32_t ERR_DATA = (int32_t)(*pos_buf_)[5];
	  (ERR_DATA)|=(int32_t)(*pos_buf_)[4] << 7;
	  (ERR_DATA)|=(int32_t)(*pos_buf_)[3] << 14;
	  (ERR_DATA)|=(int32_t)((*pos_buf_)[2]&0x07) << 21;
	  return ERR_DATA;
	}

	uint16_t PNFPosSensor::Process_Get_Total_Info()
	{
		if(*pos_read_buffer_length_ != pgv100color)  //response the change colors
		{
			//First error check
			uint16_t error_state = Process_Check_Err();
			if(error_state <= 0x02)
			{
				if((*pos_read_buffer_length_) = pgv100pos ||(*pos_read_buffer_length_ = pcv80pos))
				{

				}

			}


		}
		else
		{
			uint16_t color_buf_ = Process_Get_Color_Info();
		}

		//First error check
		uint16_t error_state = Process_Check_Err();
		if(error_state <= 0x02)
		{
			uint16_t dir_buf_ = Process_Get_Direction_Info();


			uint16_t tagno_buf_ = 0;
			if(Process_Is_Tag_Detected()) tagno_buf_ = Process_Get_Tag_Number();
			uint16_t angle_buf_ = Process_Get_Angle_Info(); 	 			  //--- Get ANGLE INFO
			uint16_t xpos_buf_ = Process_Get_XPos_Info(); 	 				  //--- Get X POSITION
			uint16_t ypos_buf_ = Process_Get_YPos_Info(); 					  //--- Get Y POSITION

			//uint16_t color_buf_;

			if()


		}
	return error_state;


	}

	int PNFPosSensor::work_loop()
	{
		//One time initialization here

			Init_Read_Buffer();
			Work_Send_Request();
			while(Buffer_Size || )
			{
				Work_Receive_Response();
			}
			Process_Data();
	}

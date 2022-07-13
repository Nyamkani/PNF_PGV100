/*
 * PNFPosSensor.cpp
 *
 *  Created on: Jul 5, 2022
 *      Author: studio3s
 */

#include <Extinc/PNFPosSensor.h>
#include "Extinc/api_init.h"
#include "Extinc/LLUart.h"

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "lwip.h"

/* Global Variables ------------------------------------------------------------------*/

//CommTime releated value

uint16_t g_pnf_comm_time_5;
uint16_t g_pnf_comm_time_6;
//these four-values must be in stm32f7xx_it.h or .c
std::vector<uint16_t> g_pnf_read_buffer_5;
std::vector<uint16_t> g_pnf_read_buffer_6;
uint16_t g_pnf_buffer_counter_5;
uint16_t g_pnf_buffer_counter_6;

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
		RequestCmd[PGV100_Straight_Request] = "0xEC0x13";
		RequestCmd[PGV100_Left_Request] = "0xE80x17";
		RequestCmd[PGV100_Right_Request] = "0xE40x1B";
		RequestCmd[PGV100_Red_Request] = "0x900x6F";
		RequestCmd[PGV100_Green_Request] = "0x880x77";
		RequestCmd[PGV100_Blue_Request] = "0xC40x3B";
		RequestCmd[PGV100_Pos_Request] = "0xC80x37";
		RequestCmd[PCV80_Pos_Requset] = "0xA00x5F";
	}

	void PNFPosSensor::ConstructCommunicationSetup()
	{
		if(this->port_ == 5)
		{
			MX_UART5_Init();
			if(pos_buf_ == nullptr) pos_buf_ = &g_pnf_read_buffer_5;
			if(pos_read_buffer_counter_ == nullptr) pos_read_buffer_counter_ = &g_pnf_buffer_counter_5;
			if(comm_time_ == nullptr) comm_time_ = &g_pnf_comm_time_5;
			if(USARTx == nullptr ) USARTx = UART5;
		}
		else if(this->port_ == 6)
		{
			MX_USART6_UART_Init();
			if(pos_buf_ == nullptr) pos_buf_ = &g_pnf_read_buffer_6;
			if(pos_read_buffer_counter_ == nullptr) pos_read_buffer_counter_ = &g_pnf_buffer_counter_6;
			if(comm_time_ == nullptr) comm_time_ = &g_pnf_comm_time_6;
			if(USARTx == nullptr ) USARTx = USART6;
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
		if(cmdstr == PGV100_Pos_Request) {this->max_read_buf_size_ = pgv100pos;}
		else if (cmdstr == PCV80_Pos_Requset){this->max_read_buf_size_ = pcv80pos;}
		else if (cmdstr == (PGV100_Straight_Request|PGV100_Left_Request|PGV100_Right_Request)) {this->max_read_buf_size_ = pgv100dir;}
		else {this->max_read_buf_size_ = pgv100color;}

		//resize vector length
		pos_buf_->assign(this->max_read_buf_size_, 0);
		//initialize numbering
		*pos_read_buffer_counter_ = 0;
	}

	//--------------------------------------------------------------Sensor Utils
	//for timeout Func.

	//due to using other files, this function may not be in class
	void CommTimerTick(uint16_t *comm_time_)
	{
		if (*comm_time_ > 0) (*comm_time_)--;
	}

	void PNFPosSensor::CommTimerReset()
	{
		*comm_time_= this->reset_time_;
	}

	bool PNFPosSensor::CommTimerIsExpired()
	{
		return (*comm_time_ == 0);
	}


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
	void PNFPosSensor::Queue_Repeat_Pos_Reqeust()
	{
		if(RequestQueue.empty())
		{
			if(sensor_type_ == pcv80) Request_Get_PCV80_Pos();
			else Request_Get_PGV100_Pos();
		}
	}

	//---------------------------------------------------------------send or read functions
	void PNFPosSensor::Work_Send_Request()
	{
		/*write functions start*/
		Usart_Transmit(this->USARTx, RequestCmd[RequestQueue.front()]);
		/*write functions end*/
	}

	void PNFPosSensor::Work_Receive_Response()
	{
		/*Read functions*/
		Usart_Receive(this->USARTx);

	}

	//---------------------------------------------------------------Processing data
	//for Error checking
	uint16_t PNFPosSensor::Process_Checksum_Data()
	{
		 uint16_t temp = 0;
		 uint16_t ChkSum_Data = 0;
		 uint16_t even_cnt[max_read_buf_size_]={0,};

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

	     if((*pos_buf_)[(max_read_buf_size_)-1] == Process_Checksum_Data())    //Checksum error pass or not(POS_BUF[20] <--- check sum buffer)
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
	     if(CommTimerIsExpired()) state |= commtimeout;        //Timeout(communication error)
	     return state;
	}

	uint32_t PNFPosSensor::Process_Get_ERR_Info()
	{
		uint8_t a,b,c,d;
		if(sensor_type_== pgv100){a = 4; b = 3; c = 2; d = 1;}
		else if (sensor_type_== pcv80){a = 5; b = 4; c = 3; d = 2;}

		uint32_t ERR_DATA = (int32_t)(*pos_buf_)[a];
		(ERR_DATA)|=(int32_t)(*pos_buf_)[b] << 7;
		(ERR_DATA)|=(int32_t)(*pos_buf_)[c] << 14;
		(ERR_DATA)|=(int32_t)((*pos_buf_)[d]&0x07) << 21;
		return ERR_DATA;
	}



	//for getting data
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

	uint8_t PNFPosSensor::Process_Get_Direction_Info()
	{

		if(Process_Is_Tag_Detected()) {return (uint8_t)(*pos_buf_)[1]&0x03;}
		else return this->dir_;
	}

	uint8_t PNFPosSensor::Process_Get_Color_Info()
	{
		if((*pos_buf_)[0]&0x07 && (*pos_buf_)[1]&0x07) return (uint8_t)(*pos_buf_)[1]&0x07;
		else return this->color_;
	}

	//for post filtering
	bool PNFPosSensor::IsValueFiltered()
	{
		return (this->now_filter_cnt_>= this->max_filter_cnt_);
	}

	void PNFPosSensor::FilterCountUp()
	{
		if(this->now_filter_cnt_< this->max_filter_cnt_) this->now_filter_cnt_++;
	}

	void PNFPosSensor::FilterStatusChanged(){this->now_filter_cnt_ = 0;}


	//finally we got combined function
	uint16_t PNFPosSensor::Process_Get_Total_Info()
	{
		if(max_read_buf_size_ != pgv100color)  //response the change colors
		{
			uint16_t prev_err_ = this->err_;
			uint16_t now_err_ = Process_Check_Err();

			//Error filters
			if(prev_err_ != this->err_) FilterStatusChanged();

			if(IsValueFiltered())
			{
				if(now_err_ < 0x01)
				{
					switch(this->max_read_buf_size_)
					{
						case pgv100dir:
							this->dir_ = Process_Get_Direction_Info();
							break;
						case pcv80pos:
							this->xpos_ = Process_Get_XPos_Info(); 	 				  //--- Get X POSITION
							this->ypos_ = Process_Get_YPos_Info(); 					  //--- Get Y POSITION
							break;
						case pgv100pos:
							if(Process_Is_Tag_Detected()) this->tagNo_ = Process_Get_Tag_Number();
							else this->tagNo_ = 0;
							this->dir_ = Process_Get_Direction_Info();
							this->angle_ = Process_Get_Angle_Info(); 	 			  //--- Get ANGLE INFO
							this->xpos_ = Process_Get_XPos_Info(); 	 				  //--- Get X POSITION
							this->ypos_ = Process_Get_YPos_Info(); 					  //--- Get Y POSITION
							break;
					}
					this->err_ = 0x00;
				}
				else {this->err_ = now_err_;}
			}
		}
		else{ this->color_ = Process_Get_Color_Info();}
		return this->err_;
	}


	uint16_t PNFPosSensor::work_loop()
	{
		//One time initialization here
		CommTimerReset();
		Init_Read_Buffer();
		Work_Send_Request();
		while((*pos_read_buffer_counter_) <= this->max_read_buf_size_)  //when the buffer reached specific values
		{
			Work_Receive_Response();
			if(CommTimerIsExpired()) break;
		}
		Queue_Delete_Request();
		Queue_Repeat_Pos_Reqeust();
		return Process_Get_Total_Info(); //return errors
	}
}

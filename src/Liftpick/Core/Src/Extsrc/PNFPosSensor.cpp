/*
 * PNFPosSensor.cpp
 *
 *  Created on: Jul 5, 2022
 *      Author: studio3s
 */

#include <Extinc/PNFPosSensor.h>
#include <Extinc/TransmitTools.h>
#include "Extinc/api_init.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Global Variables ------------------------------------------------------------------*/
//*WARNING* DO NOT CHANGE GLOBAL VARIABLE NAME

//CommTime releated value -> this must be in here and timer tick function
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
	void PNFPosSensor::RegisterRequsetCmd()
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

	void PNFPosSensor::RegisterCommunicationSetup()
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

	void PNFPosSensor::RegisterDefaultParam()
	{
		//Change_XOffset(1);
		//Change_YOffset(1);
		//Change_Angle_Offset(1);
		RequestChangeDirstraight();

	}


	//Initialization for work-loop
	void PNFPosSensor::InitReadBuffer()
	{
		//int& cmdstr = RequestQueue.front();
		switch(RequestQueue.front())
		{
			//position
			case PGV100_Pos_Request: this->max_read_buf_size_ = pgv100pos; break;
			case PCV80_Pos_Requset: this->max_read_buf_size_ = pcv80pos; break;
			//Directions
			case PGV100_Straight_Request: this->max_read_buf_size_ = pgv100dir; break;
			case PGV100_Left_Request: this->max_read_buf_size_ = pgv100dir; break;
			case PGV100_Right_Request: this->max_read_buf_size_ = pgv100dir; break;
			//Colors
			case PGV100_Red_Request: this->max_read_buf_size_ = pgv100color; break;
			case PGV100_Green_Request: this->max_read_buf_size_ = pgv100color; break;
			case PGV100_Blue_Request: this->max_read_buf_size_ = pgv100color; break;
		}
		/*
		if(cmdstr == PGV100_Pos_Request) {this->max_read_buf_size_ = pgv100pos;}
		else if (cmdstr == PCV80_Pos_Requset){this->max_read_buf_size_ = pcv80pos;}
		else if (cmdstr == (PGV100_Straight_Request|PGV100_Left_Request|PGV100_Right_Request)) {this->max_read_buf_size_ = pgv100dir;}
		else {this->max_read_buf_size_ = pgv100color;}
*/
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


	//--------------------------------------------------------------Set parameters
	//Common use
	//void PNFPosSensor::Change_Buffer_size(uint16_t Buffer_Size) {this->Buffer_Size_ = Buffer_Size;}

	//param use
	PNFPosSensor& PNFPosSensor::SetXOffset(double X_Offset)
	{
		this->x_offset_ = X_Offset;
		return* this;
	}

	PNFPosSensor& PNFPosSensor::SetYOffset(double Y_Offset)
	{
		this->y_offset_ = Y_Offset;
		return* this;
	}

	//pgv100 param only
	PNFPosSensor& PNFPosSensor::SetAngleOffset(double Angle_Offset)
	{
		this->angle_offset_ = Angle_Offset;
		return* this;
	}


	//---------------------------------------------------------------Request command
	//pgv100 only
	void PNFPosSensor::RequestChangeDirstraight() {QueueSaveRequest(PGV100_Straight_Request);}
	void PNFPosSensor::RequestChangeDirleft() {QueueSaveRequest(PGV100_Left_Request);}
	void PNFPosSensor::RequestChangeDirright() {QueueSaveRequest(PGV100_Right_Request);}

	void PNFPosSensor::RequestChangeColoryellow() {QueueSaveRequest(PGV100_Red_Request);}
	void PNFPosSensor::RequestChangeColorred() {QueueSaveRequest(PGV100_Green_Request);}
	void PNFPosSensor::RequestChangeColorblue() {QueueSaveRequest(PGV100_Blue_Request);}

	void PNFPosSensor::RequestGetPGV100Pos() {QueueSaveRequest(PGV100_Pos_Request);}

	//pcv80 only
	void PNFPosSensor::RequestGetPCV80Pos() {QueueSaveRequest(PCV80_Pos_Requset);}



	//---------------------------------------------------------------return value functions
	double PNFPosSensor::GetXPos() const {return xpos_;}
	double PNFPosSensor::GetYPos() const {return ypos_;}
	double PNFPosSensor::GetAngle() const {return angle_;}
	uint16_t PNFPosSensor::GetTagNo() const {return tagNo_;}
	uint8_t PNFPosSensor::GetDir() const {return dir_;}
	uint8_t PNFPosSensor::GetColor() const {return color_;}
	uint32_t PNFPosSensor::GetSensorErr() const {return err_;}

	double PNFPosSensor::GetXOffset() const {return x_offset_;}
	double PNFPosSensor::GetYOffset() const {return y_offset_;};
	double PNFPosSensor::GetAngleOffset() const {return angle_offset_;}
	//uint8_t PNFPosSensor::OStype() const {return OStype_;};
	uint8_t PNFPosSensor::GetCommtype() const {return comm_type_;}
	uint8_t PNFPosSensor::GetPort() const {return port_;}
	uint8_t PNFPosSensor::GetPNFSensorType() const {return sensor_type_;}




	//---------------------------------------------------------------Command queue functions
	//queue system functions
	void PNFPosSensor::QueueSaveRequest(int cmd){this->RequestQueue.push(cmd);}
	void PNFPosSensor::QueueDeleteRequest(){this->RequestQueue.pop();}
	void PNFPosSensor::QueueRepeatPosReqeust()
	{
		if(RequestQueue.empty())
		{
			if(sensor_type_ == pcv80) RequestGetPCV80Pos();
			else RequestGetPGV100Pos();
		}
	}

	//---------------------------------------------------------------send or read functions
	void PNFPosSensor::WorkSendRequest(){LLUsartTransmit(this->USARTx, RequestCmd[RequestQueue.front()]);}
	void PNFPosSensor::WorkReceiveResponse(){LLUsartReceive(this->USARTx);}

	//---------------------------------------------------------------Processing data
	//for Error checking
	uint16_t PNFPosSensor::ProcessChecksumData(std::vector<uint16_t> temp_buf)
	{
		uint16_t temp = 0;
		uint16_t ChkSum_Data = 0;
		uint16_t even_cnt[max_read_buf_size_]={0,};

		for(uint8_t i=0; i<8; i++)
		{

			for (auto& index : temp_buf)
			{
				temp = index;
				if((temp>>i)&0x01) even_cnt[i]+=1;//8bit, even
				temp = 0;
			}


			//for (std::vector<uint16_t>::iterator itr = pos_buf_->begin(); itr != pos_buf_->end()-1; ++itr)
			//{
			//   temp = *itr;
			//   if((temp>>i)&0x01) even_cnt[i]+=1;//8bit, even
			//}
			ChkSum_Data |= (even_cnt[i]%2) * (1<<i);
		}
	     return ChkSum_Data;
	}

	uint16_t PNFPosSensor::ProcessCheckErr(std::vector<uint16_t> temp_buf)
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

		 if(CommTimerIsExpired()) { state |= commtimeout;}       //Timeout(communication error)
		 else
		 {
			 if((temp_buf)[(max_read_buf_size_)-1] == ProcessChecksumData(temp_buf))    //Checksum error pass or not(POS_BUF[20] <--- check sum buffer)
			 {
				  if((temp_buf)[0] & 0x01)    //Err Occured
				  {
					   uint16_t errcode =  ProcessGetERRInfo(temp_buf);
					   if(errcode >= 1000) state |= internalFatal;        //Internal Fatal Error
					   else if(errcode==1) state |= readheadtilted;     //read head tilted 180°.(pcv80 only)
					   else if(errcode==2) state |= codeconditionerr;     //code condition error(code distance chk)
					   else if(errcode==5) state |= nodirectiondeclare;     //No clear position can be determined(�Ÿ�����)
					   else if(errcode==6) state |= nocolordeclare;     // No Color decision(Set Color choice)
				  }
				  else if((temp_buf)[0]&0x02) state |= noposition;    //No Position Error
			 }
			 else state |= checksumerr;        //check sum error
		 }
	     return state;
	}

	uint32_t PNFPosSensor::ProcessGetERRInfo(std::vector<uint16_t> temp_buf)
	{
		uint8_t a,b,c,d;
		if(sensor_type_== pgv100){a = 4; b = 3; c = 2; d = 1;}
		else if (sensor_type_== pcv80){a = 5; b = 4; c = 3; d = 2;}

		uint32_t err_data = (temp_buf)[a];
		(err_data)|=(temp_buf)[b] << 7;
		(err_data)|=(temp_buf)[c] << 14;
		(err_data)|=((temp_buf)[d]&0x07) << 21;
		return err_data;
	}



	//for getting data
	bool PNFPosSensor::ProcessIsTagDetected(std::vector<uint16_t> temp_buf)
	{
		if((temp_buf)[1] & 0x40) return true;
		else return false;
	}

	uint16_t PNFPosSensor::ProcessGetTagNumber(std::vector<uint16_t> temp_buf)
	{
	     uint16_t tag_num = 0;
		 (tag_num)=(temp_buf)[17];
		 (tag_num)|=(temp_buf)[16]<<7;
		 (tag_num)|=(temp_buf)[15]<<14;
		 (tag_num)|=(temp_buf)[14]<<21;
		 return tag_num;
	}

	double PNFPosSensor::ProcessGetAngleInfo(std::vector<uint16_t> temp_buf)
	{
		 uint16_t angle = (temp_buf)[11];
	     (angle) |= (temp_buf)[10] << 7;

	     double angle_d = (double)angle/10;
	     if((angle_d)> 180.0f) angle_d-=360.0f; //makes x-axis zero centered
	     return angle_d + this->angle_offset_;
	}

	double PNFPosSensor::ProcessGetXPosInfo(std::vector<uint16_t> temp_buf)
	{
		uint8_t a,b,c,d;
		if(sensor_type_== pgv100){a = 4; b = 3; c = 2; d = 1;}
		else if (sensor_type_== pcv80){a = 5; b = 4; c = 3; d = 2;}

		int32_t XPosition_DATA=(int32_t)(temp_buf)[a];
		(XPosition_DATA)|=(int32_t)(temp_buf)[b] << 7;
		(XPosition_DATA)|=(int32_t)(temp_buf)[c] << 14;
		(XPosition_DATA)|=(int32_t)((temp_buf)[d]&0x07) << 21;
		double xpos=(double)(XPosition_DATA/(10000.0f));                   //To make units milimeters to meters

	     //for making X-axis center to zero
	     if(sensor_type_== pgv100)
	     {
	    	 if(xpos>=10) xpos = (double)(xpos-((double)(pow(2,24)-1)/(10000.0f))-(this->x_offset_));
	    	 else xpos = (xpos-(this->x_offset_));
	     }

		if(xpos >= this->POS_AREA_MIN && xpos <= this->POS_AREA_MAX) return xpos;
		else return this->xpos_;  //	  else { state |= 0x0010;} //Out of Range
	}

	double PNFPosSensor::ProcessGetYPosInfo(std::vector<uint16_t> temp_buf)
	{
		int32_t YPosition_DATA = (temp_buf)[7];//Y Buf
		(YPosition_DATA) |= ((temp_buf)[6]) << 7;
		double ypos=(double)(YPosition_DATA/(10000.0f));              //To make units milimen));

		//for making Y-axis center to zero
		if(this->sensor_type_== pgv100)
		{
			if(ypos>=0.1) ypos = (double)(ypos-((double)(16383.0)/(10000.0f))-(this->y_offset_));
			else ypos = (ypos-(this->y_offset_));
		}
		return ypos;
	}

	uint8_t PNFPosSensor::ProcessGetDirectionInfo(std::vector<uint16_t> temp_buf)
	{
		return (uint8_t)(temp_buf)[1]&0x03;
	}

	uint8_t PNFPosSensor::ProcessGetColorInfo(std::vector<uint16_t> temp_buf)
	{
		if((temp_buf)[0]&0x07 && (temp_buf)[1]&0x07) return (uint8_t)(temp_buf)[1]&0x07;
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
	uint16_t PNFPosSensor::ProcessGetTotalInfo()
	{
		//Copying temp. data from received data
		std::vector<uint16_t> temp_buf_ = *(this->pos_buf_);

		if(max_read_buf_size_ != pgv100color)  //response for changing colors
		{
			uint16_t prev_err_ = this->err_;
			uint16_t now_err_ = ProcessCheckErr(temp_buf_);

			//Error filters
			if(prev_err_ != now_err_) FilterStatusChanged();
			else FilterCountUp();

			if(IsValueFiltered())
			{
				if(now_err_ == good)
				{
					switch(this->max_read_buf_size_)
					{
						case pgv100dir:
							this->dir_ = ProcessGetDirectionInfo(temp_buf_);
							break;
						case pcv80pos:
							this->xpos_ = ProcessGetXPosInfo(temp_buf_); 	 				  //--- Get X POSITION
							this->ypos_ = ProcessGetYPosInfo(temp_buf_); 					  //--- Get Y POSITION
							break;
						case pgv100pos:
							if(ProcessIsTagDetected(temp_buf_)) this->tagNo_ = ProcessGetTagNumber(temp_buf_);
							else this->tagNo_ = 0;
							this->dir_ = ProcessGetDirectionInfo(temp_buf_);
							this->angle_ = ProcessGetAngleInfo(temp_buf_); 	 			  //--- Get ANGLE INFO
							this->xpos_ = ProcessGetXPosInfo(temp_buf_); 	 				  //--- Get X POSITION
							this->ypos_ = ProcessGetYPosInfo(temp_buf_); 					  //--- Get Y POSITION
							break;
					}
					this->err_ = good;
				}
				else {this->err_ = now_err_;}
			}
		}
		else{ this->color_ = ProcessGetColorInfo(temp_buf_);}
		return this->err_;
	}

	uint16_t PNFPosSensor::Initializaition()
	{
		RegisterRequsetCmd();
		RegisterCommunicationSetup();
		RegisterDefaultParam();
		return 0;
	}



	uint16_t PNFPosSensor::main_loop()
	{
		//One time initialization here
		CommTimerReset();
		InitReadBuffer();
		WorkSendRequest();
		while(*(this->pos_read_buffer_counter_) <= this->max_read_buf_size_)  //when the buffer reached specific values
		{
			WorkReceiveResponse();
			if(CommTimerIsExpired()) break;
		}
		QueueDeleteRequest();
		QueueRepeatPosReqeust();
		return ProcessGetTotalInfo(); //return errors
	}

}

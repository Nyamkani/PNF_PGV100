//===========================================================================
//	File Name	: PNFPosSensor.c
//	Description : To use PNF sensors at ease
//===========================================================================


//===========================================================================
//	Include Files
//===========================================================================

#include "PNFPosSensor.hpp"   

//===========================================================================
//	Local Definitions
//===========================================================================


//===========================================================================
//	Local Variables
//===========================================================================

namespace Nyamkani
{
     class PNFPosSensor
     {
          public:
               //Basic constructor
               PNFPosSensor() : X_Offset_(0.0), Y_Offset_(0.0), Angle_Offset_(0.0){}
               
               //Easy offset options for stm32
               PNFPosSensor(double X_Offset, double Y_Offset, double Angle_Offset)
               {
                    OStype_= OperatingSystem::stm32;
                    Commtype_ = Communication::rs485;
                    X_Offset_ = X_Offset;
                    Y_Offset_ = X_Offset;
                    Angle_Offset_ = Angle_Offset;
               }

               PNFPosSensor(int OStype, int Commtype, int Port,  
                         double X_Offset, double Y_Offset, double Angle_Offset)
               {
                    OStype_ = OStype;
                    Commtype_ = Commtype;
                    Port_ = Port;

                    X_Offset_ = X_Offset;
                    Y_Offset_ = Y_Offset;
                    Angle_Offset_ = Angle_Offset;
               }

               ~PNFPosSensor();



          private:
               //---------------------------------------------------------------------------pgv100 output. declation
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

               //---------------------------------------------------------------------------pgv100 working option. declation
               uint8_t WillDir_; //left, right, straight   - 1,2,3 
               uint8_t WillColor_;


               //Receive Buffer
               std::uint16_t Buffer_Size;
               std::vector<uint16_t> POS_BUF;      // Response POS data buffer


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

               //Funcion for Related Request
               void Init_Requset_Cmd()
               {
                    RequestCmd[PGV100_Straight_Request] = "EC13";
                    RequestCmd[PGV100_Left_Request] = "EC13";  //????
                    RequestCmd[PGV100_Right_Request] = "E41B";
                    RequestCmd[PGV100_Red_Request] = "906F";
                    RequestCmd[PGV100_Green_Request] = "880";
                    RequestCmd[PGV100_Blue_Request] = "C43B";
                    RequestCmd[PGV100_Pos_Request] = "C837";
               }

               void Init_Communication_Setup()
               {
                    switch(OStype_) 
                    {
                         case: OperatingSystem::ubuntu  /*code here*/  break;
                         case: OperatingSystem::stm32   
                              switch(Port_)
                              {    
                                   case: 5  /*MX_UART5_UART_Init();*/ break;
                                   case: 6  /*MX_USART6_UART_Init();*/ break;
                                   default: /*exception code here*/ break;
                              }
                              break;
                         default: /*exception code here*/ break;
                    }
               }

               void Init_Default_Param()
               {    
                    //Change_XOffset(1);
                    //Change_YOffset(1);
                    //Change_Angle_Offset(1);
               }

               int Change_XOffset(double X_Offset) {return this->X_Offset_ = X_Offset;}

               int Change_YOffset(double Y_Offset) {return this->Y_Offset_ = Y_Offset;}

               int Change_Angle_Offset(double Angle_Offset)  {return this->Angle_Offset_ = Angle_Offset;}




               void Save_Request_In_queue(int cmd){RequestQueue.push(cmd);}

               void Do_Request() 
               {
                    


               }

               int Do_Get_Answer() {return read(serial_port, &POS_BUF, sizeof(POS_BUF));}

               void Manage_Cmd_Queue()
               {
                    RequestQueue.erase(RequestQueue.begin());
                    RequestQueue.resize(RequestQueue.back());
               }











               void Default_Init();





     };

}






//Defines and Declations
u16 POS_BUF[POS_PGV100_TOTAL_BYTES]={0};      // Response POS data buffer

u16 POS_BUF_PTR=0;                 //
u16 POS_SEQUENCE=0;        // response result check

int32_t POS_AREA_MAX=10000;                                          // PCV���� ���� Maximum(mm)
int32_t POS_AREA_MIN=(-100);   // PCV���� ���� Minimum 

//To check vaild Condition
u16 NomCnd_Cnt=0;  
u16 ErrCnd_Cnt=0;

//for timeout Func.
volatile unsigned int POS_CommTimer_Remain = 0;

//Values for request cmd
char PGV100_Pos_Request[3]={0xC8,0x37,CMD_NULL};   //for Reqeusting messages    from head to receive POSITON

char PGV100_Straight_Request[3]={0xEC,0x13,CMD_NULL}; //for Reqeusting  changing  straight  direction
char PGV100_Left_Request[3]={0xE8,0x17,CMD_NULL}; //for Reqeusting  changing  left  direction
char PGV100_Right_Request[3]={0xE4,0x1B,CMD_NULL}; //for Reqeusting  changing  right direction

char PGV100_Red_Request[3]={0x90,0x6F,CMD_NULL}; //for Reqeusting  changing  RED direction
char PGV100_Green_Request[3]={0x88,0x77,CMD_NULL}; //for Reqeusting  changing  GREEN direction
char PGV100_Blue_Request[3]={0xC4,0x3B,CMD_NULL}; //for Reqeusting  changing  BLUE direction


/****Declare Main Structure Start****/

PosSensorVal PGV100SensorVal;

/****Declare Main Structure End****/

//===========================================================================
//	Local Functions Definition
//===========================================================================

//-------------Sensor COMM.
//for Requesting datas
void COM2_PutStr(char *pt) 
{
     u16 wtime=0;

     while(*pt) 
     {
          USART2->DR = (u16)*pt++;
             
          wtime=0;
          do 
          {
               if (++wtime > 10000) 
               {	                    // TX Timeout
                    USART2_Init();
                    
                    //ClearBufPtr();
                    DPrintf("> TX0 Reset 1\n");
                    break;
               }
               Delay_us(1);
          } 
          while((USART2->SR & 0x00C0)!=0xC0);	// TXE, TC
          //while((USART2->SR & (USART_FLAG_TC|USART_FLAG_TXE)) == RESET);
	}
	
	//USART1->CR1 |= (1<<5);		// RXNE Interrupt Enable
}



//-------------Sensor Utils
//for timeout Func.
void POS_CommTimer_Tick()
{
	if (POS_CommTimer_Remain > 0) POS_CommTimer_Remain--;
}

void POS_CommTimer_Reset(unsigned int nTime)
{
	POS_CommTimer_Remain= nTime;
}

int POS_CommTimer_IsExpired()
{
	return (POS_CommTimer_Remain == 0);
}



//-------------Sensor Inits. Seqeunce
//for Init Structures
void POS_INIT_VALUES(PosSensorVal* (PCVRSTValue))
{
     //use this Inital values or get datas from ROM
     PCVRSTValue->XPS = 0;
     PCVRSTValue->YPS = 0;
     PCVRSTValue->X_OFFSET = 0;
     PCVRSTValue->Y_OFFSET = 0;
     PCVRSTValue->ANGLE = 0;
     
     PCVRSTValue->TagNo = 0;
     
     PCVRSTValue->Dir = 0;
     PCVRSTValue->DirOld = 0;
     
     PCVRSTValue->Color = 0;
     PCVRSTValue->ColorOld = 0;
    
     PCVRSTValue->SensorErr = 0;
     POS_SEQUENCE = 1;
}

//buffer init.
void POS_BUFFER_INIT(PosSensorVal* (PCVRSTValue))
{
     u16  i;

     POS_SEQUENCE=2;
     POS_BUF_PTR=0;
     for(i=0;i<(POS_PGV100_TOTAL_BYTES);i++) {POS_BUF[i]=0;}
     POS_CommTimer_Reset(POS_TIMEOUT);
}



//-------------Sensor Request Seqeunce(for "POS_REQUEST_SEELECTOR" function)
//for Request pos.
void POS_REQUEST(PosSensorVal* (PCVRSTValue))
{
     COM2_PutStr(PGV100_Pos_Request);  
}

//for following line-tape and Matrix data
void POS_DIR_REQUEST(PosSensorVal* (PCVRSTValue))
{
     if(PCVRSTValue->Dir==0) PCVRSTValue->Dir=3;
     switch(PCVRSTValue->Dir)
     {
          case 1: COM2_PutStr(PGV100_Right_Request); break;
          case 2: COM2_PutStr(PGV100_Left_Request); break;
          case 3: COM2_PutStr(PGV100_Straight_Request); break;
          default: break;
     }
}

//for following Colored tape
void POS_COLOR_REQUEST(PosSensorVal* (PCVRSTValue))
{
     switch(PCVRSTValue->Color)
     {
          case 1: COM2_PutStr(PGV100_Red_Request); break;
          case 2: COM2_PutStr(PGV100_Green_Request); break;
          case 3: COM2_PutStr(PGV100_Blue_Request); break;
          default: break;
     }
}

//for Chk condition and do seperated funcc.
void POS_REQUEST_SELECTOR(PosSensorVal* (PCVRSTValue))
{
     POS_SEQUENCE=3;
     if((PCVRSTValue->Dir != PCVRSTValue->DirOld)||(PCVRSTValue->Dir==0)) {POS_DIR_REQUEST(PCVRSTValue);}
     else if(PCVRSTValue->Color != 0) {POS_COLOR_REQUEST(PCVRSTValue);}
     else {POS_REQUEST(PCVRSTValue);}
}



//-------------Sensor Get Infomations(for "POS_GET_INFO" function)
int POS_TAG_DETECT(PosSensorVal* (PCVRSTValue))
{
     int result=0;
     if(POS_BUF[1]&0x40) result=1;
     return result;
}

int32_t POS_GET_TAG_INFO(PosSensorVal* (PCVRSTValue))
{
     int32_t TagNum=0;
     (TagNum)=(int32_t)POS_BUF[17];
     (TagNum)|=(int32_t)POS_BUF[16]<<7;
     (TagNum)|=(int32_t)POS_BUF[15]<<14;
     (TagNum)|=(int32_t)POS_BUF[14]<<21;
     return TagNum;
}

int16_t POS_GET_ANGLE_INFO(PosSensorVal* (PCVRSTValue))
{
     int16_t ANGLE=0;
     (ANGLE)=(int16_t)POS_BUF[11];
     (ANGLE)|=(int16_t)POS_BUF[10]<<7;
     (ANGLE)=(int16_t)(((ANGLE)/10));
     if((ANGLE)> 180) ANGLE-=360; //makes x-axis zero centered
     return ANGLE;
}

double POS_GET_XPOS_INFO(PosSensorVal* (PCVRSTValue))
{
     int32_t XPosition_DATA = 0;
     double XPOS;
       
     (XPosition_DATA)=(int32_t)POS_BUF[5];
     (XPosition_DATA)|=(int32_t)POS_BUF[4]<<7;
     (XPosition_DATA)|=(int32_t)POS_BUF[3]<<14;
     (XPosition_DATA)|=(int32_t)(POS_BUF[2]&0x07)<<21;
     XPOS=(double)(XPosition_DATA/(10000.0f));                   //To make units milimeters to meters
     
     //for making X-axis center to zero
     if(XPOS>=10) XPOS = (double)(XPOS-((double)(pow(2,24)-1)/(10000.0f))-(PCVRSTValue->X_OFFSET));
     else XPOS = (XPOS-(PCVRSTValue->X_OFFSET));
    
     return XPOS;
}

double POS_GET_YPOS_INFO(PosSensorVal* (PCVRSTValue))
{
     int32_t YPosition_DATA = 0;
     double YPOS;
     
     (YPosition_DATA)=(int32_t)POS_BUF[7];//Y Buf
     (YPosition_DATA)|=((int32_t)POS_BUF[6])<<7;
     YPOS=(double)(YPosition_DATA/(10000.0f));              //To make units milimeters to meters
              
     //for making Y-axis center to zero
     if(YPOS>=0.1) YPOS = (double)(YPOS-((double)(16383.0)/(10000.0f))-(PCVRSTValue->Y_OFFSET));
     else YPOS = (YPOS-(PCVRSTValue->Y_OFFSET));
     
     return YPOS;
}

int32_t POS_GET_ERR_INFO()
{
     int32_t ERR_DATA = 0;
     (ERR_DATA)=(int32_t)POS_BUF[5];
     (ERR_DATA)|=(int32_t)POS_BUF[4]<<7;
     (ERR_DATA)|=(int32_t)POS_BUF[3]<<14;
     (ERR_DATA)|=(int32_t)(POS_BUF[2]&0x07)<<21;
     return ERR_DATA;
}

uint16_t POS_GET_INFO(PosSensorVal* (PCVRSTValue)) 
{
     uint16_t state = 0;
     double POS_X_COORDINATE=0.0f; 
     double POS_Y_COORDINATE=0.0f;
     u16 TagNum=0;
     int16_t ANGLE=0;
       
     //--- TAG INFO
     if(POS_TAG_DETECT(PCVRSTValue)==1) TagNum = POS_GET_TAG_INFO(PCVRSTValue);       
                   
     //--- ANGLE INFO
     ANGLE = POS_GET_ANGLE_INFO(PCVRSTValue); 
      
     //--- X POSITION                          
     POS_X_COORDINATE = POS_GET_XPOS_INFO(PCVRSTValue);
                              
     //--- Y POSITION    
     POS_Y_COORDINATE = POS_GET_YPOS_INFO(PCVRSTValue);
        
     //���� ���� In-range 
     if(POS_X_COORDINATE >= POS_AREA_MIN && POS_X_COORDINATE <= POS_AREA_MAX)
     {
          PCVRSTValue->XPS = POS_X_COORDINATE;    // X ��ǥ
          PCVRSTValue->YPS = POS_Y_COORDINATE;    // Y ��ǥ 
          PCVRSTValue->TagNo = (TagNum);             //Tag info
          PCVRSTValue->ANGLE = (ANGLE);              //Angle Info
     } 
     else state |= 0x0010; //Out of Range
     return state;
}



//-------------Analysis data and Check errors(for "POS_DATA_ANALYSIS" function)
//for Chking Modes
void POS_DIR_CHK(PosSensorVal* (PCVRSTValue))
{
     u16 LANE_DATA = 0;
     if((PCVRSTValue->Dir != PCVRSTValue->DirOld))
     {
          (LANE_DATA)|=(u16)(POS_BUF[1]&0x03);
          switch(LANE_DATA)
          {
               case 1: PCVRSTValue->DirOld = 1; break;
               case 2: PCVRSTValue->DirOld = 2; break;
               case 3: PCVRSTValue->DirOld = 3; break;
               default: break;
          }
     }
}

void POS_COLOR_CHK(PosSensorVal* (PCVRSTValue))
{
     PCVRSTValue->ColorOld = PCVRSTValue->Color;
//     u16 COLOR_DATA = 0;
//     if((PCVRSTValue->Color != PCVRSTValue->ColorOld))
//     {
//          (COLOR_DATA)|=(u16)(PCV_SINGLE_BUF[1]&0x03);
//          switch(COLOR_DATA)
//          {
//               case 1: PCVRSTValue->DirOld = 1; break;
//               case 2: PCVRSTValue->DirOld = 2; break;
//               case 3: PCVRSTValue->DirOld = 3; break;
//               default: break;
//          }
//     }
}

//CHKSUM calculator
uint16_t POS_CHKSUM_DATA()
{
     u16  i=0;
     u16  j=0;
     u16  even_cnt[POS_PGV100_TOTAL_BYTES]={0,};
     u16  ChkSum_Data=0;
     u16  temp=0;
     for(i=0;i<9;i++) even_cnt[i]=0;
     
     for(i=0;i<(POS_PGV100_TOTAL_BYTES-1);i++) //Buf 0~20 21��. 
     {
          temp=POS_BUF[i];
          for(j=0;j<8;j++)  //8bit data
          {
               if((temp>>j)&0x01) even_cnt[j]+=1;//even 
          }
          //ChkSum_Data=0;
     } 
     for(i=0;i<(POS_PGV100_TOTAL_BYTES-1);i++)
     {//buf 0~20 8��(�Է¹޴� 21�� ������-1) 
          if((even_cnt[i]&0x01)==0x01) ChkSum_Data|= ((u16)1<<i);//Ȧ���̸� 1, ¦���̸� 0
     }
     return ChkSum_Data;
}

uint16_t POS_ERR_CHK(u16 *ChkSum_Data)
{
     uint16_t state=0;
     int32_t errcode=0;

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
     
     ///to define addintional errors
     //if((PCVRSTValue->SUM_diff)>300.0f){SysERR_SAVE(SysERR_PCV100_TAPE);}//���׳�ƽ �ڵ� ������ err
     
     
     if(POS_BUF[20]==(*ChkSum_Data))   //POS_BUF[20] <--- check sum buffer 
     {
          if((POS_BUF[0]&0x01)==0x01)    //Err Occured
          {
               errcode =  POS_GET_ERR_INFO();       
               if(errcode>1000) state |= 0x1000;        //Internal Fatal Error  (��ü)
               else if(errcode==2) state |= 0x0002;     //code condition error(code distance chk)
               else if(errcode==5) state |= 0x0004;     //No clear position can be determined(�Ÿ�����)
               else if(errcode==6) state |= 0x0008;     // No Color decision(Set Color choice)
          }
          else if((POS_BUF[0]&0x02)) state |= 0x0020;    //No Position Error
     } 
     else state |= 0x0080;        //check sum error
     if(POS_CommTimer_IsExpired()) state |= 0x0040;        //Timeout(communication error)
     return state;
}

uint16_t POS_DATA_ANALYSIS(PosSensorVal* (PCVRSTValue))
{ 
     u16 state=0;
     u16 ChkSum_Data=0;

     POS_DIR_CHK(PCVRSTValue);          // DIRETION CHECK
     POS_COLOR_CHK(PCVRSTValue);        // COLOR CHECK
     ChkSum_Data = POS_CHKSUM_DATA();           //Calculate chksum
     state |= POS_ERR_CHK(&ChkSum_Data);        //ERR Chk
     
     if(state==0x0000) state |= POS_GET_INFO(PCVRSTValue);

     return state;
}



//-------------for filtering noises and vailding data (for "POS_SENSOR_RECEIVE" function)
void POS_ERR_FILTER(PosSensorVal* (PCVRSTValue), u16 *state, u16 filterNum) 
{
     if((*state)<=0x0001)        //Good �Ǵ� Good(warning)
     {
          ErrCnd_Cnt=0;
          if(NomCnd_Cnt > filterNum) PCVRSTValue->SensorErr = (*state);
          else NomCnd_Cnt++;
     }
     else                     //report Err to structures
     {
          NomCnd_Cnt=0;
          if(ErrCnd_Cnt > filterNum) PCVRSTValue->SensorErr = (*state);
          else ErrCnd_Cnt++;
     }  
}

uint16_t POS_SENSOR_RECEIVE(PosSensorVal* (PCVRSTValue)) 
{    
     u16 state;
     POS_SEQUENCE=1;
     state = POS_DATA_ANALYSIS(PCVRSTValue);          // ANALYSIS THE GETTED INFOMATION  
     POS_ERR_FILTER(PCVRSTValue, &state, 5);      
     return state;
}


//-------------Using this funcions for getting Pos.
//Main functions
void POSTION_SENSOR_OPERATION(PosSensorVal* (PCVRSTValue))
{
     switch(POS_SEQUENCE)
     {
          case 0: POS_INIT_VALUES(PCVRSTValue); break;         //get valeus From ROM. Development in progress
          case 1: POS_BUFFER_INIT(PCVRSTValue); break;
          case 2: POS_REQUEST_SELECTOR(PCVRSTValue); break;
          case 3: if(POS_CommTimer_IsExpired()) POS_SEQUENCE=6; break;  
//          case 4: reserved(); break;
//          case 5: reserved(); break;
//          case 4: reserved break;
//          case 5: reserved break;     
          case 6: POS_SENSOR_RECEIVE(PCVRSTValue); break;
           default: break;
     }
}




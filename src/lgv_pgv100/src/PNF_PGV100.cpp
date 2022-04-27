//declare standard c++14 headers
#include <iostream>
#include <sstream>
#include <deque>
#include <thread>
#include <chrono>

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


using std::thread;

//declare own ros2 headers
#include "rclcpp/rclcpp.hpp"
#include "PNF_PGV100.hpp"

#define POS_PGV100_TOTAL_BYTES 21
#define CMD_NULL 0X00



namespace Nyamkani
{

    struct Params
    {

    };

    enum State
    {
        Init = 0,
        BufferInit = 1,
        Request = 2,
        WaitAnswer = 3,
        Analysis = 4,

        Completed = 10,
        Error = 11,
    };
    
    enum ErrCode 
    {
        Good = 0x0000,
        Warning = 0x0001,
        ERR_01 = 0x0002,   //Prevent from spiking position value 
        ERR_02 = 0x0002,   //Prevent from spiking position value 
        ERR_03 = 0x0004,   //Prevent from having a no dirction setting
        ERR_04 = 0x0008,   //Prevent from having a no color setting
        ERR_05 = 0x0010,   //Prevent from having condtions in Out of Range
        ERR_06 = 0x0020,   //Prevent from having a no postion
        ERR_07 = 0x0040,   //Prevent from comm. errors(Time out) 
        ERR_08 = 0x0080,   //Prevent from having chk_sum error
        ERR_09 = 0x0100,   //Reserved
        ERR_10 = 0x0200,   //Reserved
        ERR_11 = 0x0400,   //Reserved
        ERR_12 = 0x0800,   //Reserved
        ERR_13 = 0x1000,   //Reserved
        ERR_14 = 0x2000,   //Reserved
        ERR_15 = 0x4000,   //Reserved
        ERR_16 = 0x8000,   //Reserved
    };

    enum PGV_Cmd
    {
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

            //Read All params
            PGV100_View_ALL_Read_Val,

    };

    class MODULE_PGV100
    {
        private:
            //---------------------------------------------------------------------------pgv100 output. declation
            //to see useful values
            double  XPS;               
            double  YPS;   
            double ANGLE;
            u_int16_t TagNo;
            u_int16_t NowDir;
            u_int16_t NowColor;
            u_int16_t SensorErr;

            //---------------------------------------------------------------------------pgv100 parameters. declation
            //params
            double  X_OFFSET;
            double  Y_OFFSET;
            double ANGLE_OFFSET;

            //---------------------------------------------------------------------------pgv100 working option. declation
            u_int16_t WillDir; //left, right, straight   - 1,2,3 
            u_int16_t WillColor;

            u_int16_t POS_BUF[POS_PGV100_TOTAL_BYTES] = { 0 };      // Response POS data buffer

            int32_t POS_AREA_MAX = 10000;                                          // PCV센서 범위 Maximum(mm)
            int32_t POS_AREA_MIN = (-100);   // PCV센서 범위 Minimum 

            //To check vaild Condition
            u_int16_t NomCnd_Cnt = 0;
            u_int16_t ErrCnd_Cnt = 0;
            

            //---------------------------------------------------------------------------485 Comm. declation
            //485 comm
            int serial_port;
            // Create new termios struct, we call it 'tty' for convention
            struct termios tty;

            //DATA_RECEIVE_HANDLER onDataReceived;
            State state = Init;
            thread executer;


            //---------------------------------------------------------------------------485 Comm. cmds declation
            //Values for request cmd
            std::vector<std::string> RequestCmd;
            std::vector<int> RequestQueue;
            
            //Funcion for Related Request
            void Init_Requset_Cmd()
            {
                RequestCmd[PGV100_Straight_Request] = "0xEC0x13";
                RequestCmd[PGV100_Left_Request] = "0xEC0x13";
                RequestCmd[PGV100_Right_Request] = "0xE40x1B";
                RequestCmd[PGV100_Red_Request] = "0x900x6F";
                RequestCmd[PGV100_Green_Request] = "0x880x7";
                RequestCmd[PGV100_Blue_Request] = "0xC40x3B";
                RequestCmd[PGV100_Pos_Request] = "0xC80x37";
            }

            void Save_Request_In_queue(int cmd){RequestQueue.push_back(cmd);}

            void Do_Write_Request() {if((RequestQueue.size()!=0)) write(serial_port, &RequestCmd[RequestQueue.front()], sizeof(RequestCmd[RequestQueue.front()])); }

            int Do_Get_Answer() {return read(serial_port, &POS_BUF, sizeof(POS_BUF));}

            void Manage_Cmd_Queue()
            {
                //std::vector<int>::iterator front_itr = RequestQueue.begin();
                //std::vector<int>::iterator end_itr = RequestQueue.end();
                RequestQueue.erase(RequestQueue.begin());
                RequestQueue.resize(RequestQueue.back());
            }


            
            //bool LoadParameter(char* path)
            //{
                // if (true)
                // {
                //     //throw std::exception("File is not exist");
                // }
                // else
                // {

                // }

   
            //}
            int Init_485_Comm()
            {
                // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
                serial_port = open("/dev/ttyUSB0", O_RDWR);

                // Create new termios struct, we call it 'tty' for convention
                //struct termios tty;

                // Read in existing settings, and handle any error
                if(tcgetattr(serial_port, &tty) != 0) {
                    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
                    return 1;
                }

                tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
                tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
                tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
                tty.c_cflag |= CS8; // 8 bits per byte (most common)
                tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
                tty.c_cflag |= CREAD | CLOCAL; // Turn on READ &0000 ignore ctrl lines (CLOCAL = 1)

                tty.c_lflag &= ~ICANON;
                tty.c_lflag &= ~ECHO; // Disable echo
                tty.c_lflag &= ~ECHOE; // Disable erasure
                tty.c_lflag &= ~ECHONL; // Disable new-line echo
                tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
                tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
                tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

                tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
                tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
                // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
                // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

                tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
                tty.c_cc[VMIN] = 0;

                // Set in/out baud rate to be 9600
                cfsetispeed(&tty, B115200);
                cfsetospeed(&tty, B115200);

                // Save tty settings, also checking for error
                if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
                    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
                    return 1;
                }    
                //////////////////////////////////////////////////////////////////////////////////////////////////////
                    // Write to serial port
                    //unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
                    //write(serial_port, msg, sizeof(msg));

                    // Allocate memory for read buffer, set size according to your needs
                    //char read_buf [256];

                    // Normally you wouldn't do this memset() call, but since we will just receive
                    // ASCII data for this example, we'll set everything to 0 so we can
                    // call printf() easily.
                    //memset(&read_buf, '\0', sizeof(read_buf));

                    // Read bytes. The behaviour of read() (e.g. does it block?,
                    // how long does it block for?) depends on the configuration
                    // settings above, specifically VMIN and VTIME
                    //int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

                    // // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
                    // if (num_bytes < 0) {
                    //     printf("Error reading: %s", strerror(errno));
                    //     return 1;
                    // }
                    //////////////////////////////////////////////////////////////////////////////////////////////////////
                }


            void Init_Parameter()
            {
                XPS = 0;
                YPS = 0;
                X_OFFSET = 0;
                Y_OFFSET = 0;
                ANGLE = 0;
 
                TagNo = 0;
 
                NowDir = 0;
                WillDir = 0;
 
                NowColor = 0;
                WillColor = 0;
 
                SensorErr = 0;
            }

            void Init_Buffer()
            {
                memset(&POS_BUF, '\0', sizeof(POS_BUF));
            }

            int POS_REQUEST()
            {
                return PGV100_Pos_Request;
            }

            //for following line-tape and Matrix data
            int DIR_REQUEST()
            {
                if(this->NowDir==0) this->WillDir=3;
                switch(this->WillDir)
                {            
                    case 1: return PGV100_Right_Request; break;
                    case 2: return PGV100_Left_Request; break;
                    case 3: return PGV100_Left_Request; break;
                    default: break;
                }
                return -1;
            }

            //for following Colored tape
            int COLOR_REQUEST()
            {
                switch(this->NowColor)
                {
                    case 1: return PGV100_Red_Request; break;
                    case 2: return PGV100_Green_Request; break;
                    case 3: return PGV100_Blue_Request; break;
                    default: break;
                }
                return -1; 
            }

            //for Chk condition and do seperated funcc.
            void Request_Selector()
            {
                this->state = WaitAnswer;
                if((this->Dir != this->DirOld)||(this->Dir==0)) {DIR_REQUEST();}
                else if(this->Color != 0) {COLOR_REQUEST();}
                else {POS_REQUEST();}
            }


            //-------------Sensor Get Infomations(for "POS_GET_INFO" function)
            int POS_TAG_DETECT()
            {
                int result=0;
                if(POS_BUF[1]&0x40) result=1;
                return result;
            }

            int32_t POS_GET_TAG_INFO()
            {
                int32_t TagNum=0;
                (TagNum)=(int32_t)POS_BUF[17];
                (TagNum)|=(int32_t)POS_BUF[16]<<7;
                (TagNum)|=(int32_t)POS_BUF[15]<<14;
                (TagNum)|=(int32_t)POS_BUF[14]<<21;
                return TagNum;
            }

            int16_t POS_GET_ANGLE_INFO()
            {
                int16_t ANGLE=0;
                (ANGLE)=(int16_t)POS_BUF[11];
                (ANGLE)|=(int16_t)POS_BUF[10]<<7;
                (ANGLE)=(int16_t)(((ANGLE)/10));
                if((ANGLE)> 180) ANGLE-=360; //makes x-axis zero centered
                return ANGLE;
            }

            double POS_GET_XPOS_INFO()
            {
                int32_t XPosition_DATA = 0;
                double XPOS;
                
                (XPosition_DATA)=(int32_t)POS_BUF[5];
                (XPosition_DATA)|=(int32_t)POS_BUF[4]<<7;
                (XPosition_DATA)|=(int32_t)POS_BUF[3]<<14;
                (XPosition_DATA)|=(int32_t)(POS_BUF[2]&0x07)<<21;
                XPOS=(double)(XPosition_DATA/(10000.0f));                   //To make units milimeters to meters
                
                //for making X-axis center to zero
                if(XPOS>=10) XPOS = (double)(XPOS-((double)(pow(2,24)-1)/(10000.0f))-(this->X_OFFSET));
                else XPOS = (XPOS-(this->X_OFFSET));
                
                return XPOS;
            }

            double POS_GET_YPOS_INFO()
            {
                int32_t YPosition_DATA = 0;
                double YPOS;
                
                (YPosition_DATA)=(int32_t)POS_BUF[7];//Y Buf
                (YPosition_DATA)|=((int32_t)POS_BUF[6])<<7;
                YPOS=(double)(YPosition_DATA/(10000.0f));              //To make units milimeters to meters
                        
                //for making Y-axis center to zero
                if(YPOS>=0.1) YPOS = (double)(YPOS-((double)(16383.0)/(10000.0f))-(this->Y_OFFSET));
                else YPOS = (YPOS-(this->Y_OFFSET));
                
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

            uint16_t POS_GET_INFO() 
            {
                uint16_t state = 0;
                double POS_X_COORDINATE=0.0f; 
                double POS_Y_COORDINATE=0.0f;
                u_int16_t TagNum=0;
                int16_t ANGLE=0;
                
                //--- TAG INFO
                if(POS_TAG_DETECT()) TagNum = POS_GET_TAG_INFO();       
                            
                //--- ANGLE INFO
                ANGLE = POS_GET_ANGLE_INFO(); 
                
                //--- X POSITION                          
                POS_X_COORDINATE = POS_GET_XPOS_INFO();
                                        
                //--- Y POSITION    
                POS_Y_COORDINATE = POS_GET_YPOS_INFO();
                    
                //동작 범위 In-range 
                if(POS_X_COORDINATE >= POS_AREA_MIN && POS_X_COORDINATE <= POS_AREA_MAX)
                {
                    this->XPS = POS_X_COORDINATE;    // X 좌표
                    this->YPS = POS_Y_COORDINATE;    // Y 좌표 
                    this->TagNo = (TagNum);             //Tag info
                    this->ANGLE = (ANGLE);              //Angle Info
                } 
                else state |= 0x0010; //Out of Range
                return state;
            }



            //-------------Analysis data and Check errors(for "POS_DATA_ANALYSIS" function)
            //for Chking Modes
            void POS_DIR_CHK()
            {
                u_int16_t LANE_DATA = 0;
                if((this->Dir != this->DirOld))
                {
                    (LANE_DATA)|=(u_int16_t)(POS_BUF[1]&0x03);
                    switch(LANE_DATA)
                    {
                        case 1: this->DirOld = 1; break;
                        case 2: this->DirOld = 2; break;
                        case 3: this->DirOld = 3; break;
                        default: break;
                    }
                }
            }

            // void POS_COLOR_CHK()
            // {
            //     this->ColorOld = this->Color;
            //     u_int16_t COLOR_DATA = 0;
            //     if((this->Color != this->ColorOld))
            //     {
            //          (COLOR_DATA)|=(u_int16_t)(POS_BUF[1]&0x03);
            //          switch(COLOR_DATA)
            //          {
            //               case 1: this->DirOld = 1; break;
            //               case 2: this->DirOld = 2; break;
            //               case 3: this->DirOld = 3; break;
            //               default: break;
            //          }
            //     }
            // }

            //CHKSUM calculator
            uint16_t POS_CHKSUM_DATA()
            {
                u_int16_t  i=0;
                u_int16_t  j=0;
                u_int16_t  even_cnt[POS_PGV100_TOTAL_BYTES]={0,};
                u_int16_t  ChkSum_Data=0;
                u_int16_t  temp=0;
                for(i=0;i<9;i++) even_cnt[i]=0;
                
                for(i=0;i<(POS_PGV100_TOTAL_BYTES-1);i++) //Buf 0~20 21개. 
                {
                    temp=POS_BUF[i];
                    for(j=0;j<8;j++)  //8bit data
                    {
                        if((temp>>j)&0x01) even_cnt[j]+=1;//even 
                    }
                    //ChkSum_Data=0;
                } 
                for(i=0;i<(POS_PGV100_TOTAL_BYTES-1);i++)
                {//buf 0~20 8개(입력받는 21개 데이터-1) 
                    if((even_cnt[i]&0x01)==0x01) ChkSum_Data|= ((u_int16_t)1<<i);//홀수이면 1, 짝수이면 0
                }
                return ChkSum_Data;
            }

            uint16_t POS_ERR_CHK(u_int16_t *ChkSum_Data)
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
                //if((PCVRSTValue->SUM_diff)>300.0f){SysERR_SAVE(SysERR_PCV100_TAPE);}//마그네틱 코드 테이프 err
                
                
                if(POS_BUF[20]==(*ChkSum_Data))   //POS_BUF[20] <--- check sum buffer 
                {
                    if((POS_BUF[0]&0x01)==0x01)    //Err Occured
                    {
                        errcode =  POS_GET_ERR_INFO();       
                        if(errcode>1000) state |= 0x1000;        //Internal Fatal Error  (교체)
                        else if(errcode==2) state |= 0x0002;     //code condition error(code distance chk)
                        else if(errcode==5) state |= 0x0004;     //No clear position can be determined(거리수정)
                        else if(errcode==6) state |= 0x0008;     // No Color decision(Set Color choice)
                    }
                    else if((POS_BUF[0]&0x02)) state |= 0x0020;    //No Position Error
                } 
                else state |= 0x0080;        //check sum error
                if(POS_CommTimer_IsExpired()) state |= 0x0040;        //Timeout(communication error)
                return state;
            }

            uint16_t POS_DATA_ANALYSIS()
            { 
                u_int16_t state=0;
                u_int16_t ChkSum_Data=0;

                POS_DIR_CHK();          // DIRETION CHECK
                POS_COLOR_CHK();        // COLOR CHECK
                ChkSum_Data = POS_CHKSUM_DATA();           //Calculate chksum
                state |= POS_ERR_CHK(&ChkSum_Data);        //ERR Chk
                
                if(state==0x0000) state |= POS_GET_INFO();

                return state;
            }



            //-------------for filtering noises and vailding data (for "POS_SENSOR_RECEIVE" function)
            void POS_ERR_FILTER(u16 *state, u16 filterNum) 
            {
                if((*state)<=0x0001)        //Good 또는 Good(warning)
                {
                    ErrCnd_Cnt=0;
                    if(NomCnd_Cnt > filterNum) this->SensorErr = (*state);
                    else NomCnd_Cnt++;
                }
                else                     //report Err to structures
                {
                    NomCnd_Cnt=0;
                    if(ErrCnd_Cnt > filterNum) this->SensorErr = (*state);
                    else ErrCnd_Cnt++;
                }  
            }

            uint16_t POS_SENSOR_RECEIVE() 
            {    
                u_int16_t state;
                POS_SEQUENCE=1;
                state = POS_DATA_ANALYSIS();          // ANALYSIS THE GETTED INFOMATION  
                POS_ERR_FILTER(PCVRSTValue, &state, 5);      
                return state;
            }



            void WORK_LOOP()
            {
                Init_Buffer();
                Request_Selector();
                while (true)
                {




                    if (this->state > 10)
                    //매직 플래그를 사용해서
                    //완료 후의 상태는 10 이상으로 정의
                    {
                        break;
                    }
                }
                //onDataReceived(POS_BUF);
                executer.join();
            }

            void StopLoop()
            {
                this->state = Error;
            }

        public:

            void System_Initiation()
            {
               //bool LoadParameter(char* path);
               this->Init485Comm();
               this->InitValues();
            }

            // void AddListener(DATA_RECEIVE_HANDLER callback)
            // {
            //     //this->onDataReceived = callback;
            // }

            void Start()
            {
                /*
                * thread 객체가 실행 가능한 상태에 있을 때 조인 가능하다고 표현한다. 디폴트로 생성된 thread 객체는 조인 불가능하다. 조인 가능한 thread 객체를 제거하려면, 먼저 그 객체의 join 이나 detach 부터 호출해야 한다. 
                * join 은 스레드 작업이 끝날 때까지 기다리고, detach 는 OS 내부의 스레드와 분리한다. 둘다 스레드를 조인 불가능한 상태로 전환시킨다. 
                * 조인 가능한 상태의 thread 객체를 제거하면, 그 객체의 소멸자가 std::terminate 를 호출해서, 애플리케이션마자 종료시킨다. 
                */
                thread executer(WORK_LOOP);


            };

            void Stop()
            {
                /*
                    리퀘스트 중지를 위한 메서드
                    중지작업 전에 필요한 호출이 있다면 구현   
                */

                StopLoop();
            }


            ~MODULE_PGV100()
            {
                std::terminate(executer); 
            };
    }

}


int main(int argc, char *argv[])
{
    

  rclcpp::init(argc, argv);
  //auto sick_nav350_publisher = std::make_shared<sick_nav350>();
  //signal(SIGINT,ExitHandler);
  //int ret = sick_nav350_publisher->work_loop();
  auto instance = new Nyamkani::MODULE_PGV100();
  instance->Init();
  instance->Start();





  




  rclcpp::shutdown();
  //return ret;
  //return 0;
}
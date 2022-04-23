//declare standard c++14 headers
#include <iostream>
#include <sstream>
#include <deque>
#include <thread>

using std::thread;

//declare own ros2 headers
#include "rclcpp/rclcpp.hpp"
#include "PNF_PGV100.hpp"

namespace Nyamkani
{

    struct Params
    {




    };


    enum State
    {
        NotInited = 0,
        Inited = 1,
        BufferInited = 2,
        Working = 3,

        Completed = 10,
        Error = 11,
    };
    
    enum ErrCode 
    {
        Good = 0x00,
        ERR_01 = 100

        
    };

    typedef void DATA_RECEIVE_HANDLER*(u16* ptr)

    class MODULE_PGV100
    {
        private:
            
            double  XPS;               
            double  YPS;                
            double  X_OFFSET;
            double  Y_OFFSET;
            int16_t  ANGLE;

            u_int16_t TagNo;

            u_int16_t Dir; //left, right, straight   - 1,2,3 
            u_int16_t DirOld;

            u_int16_t Color;
            u_int16_t ColorOld;

            u_int16_t SensorErr;


            u16 POS_BUF[POS_PGV100_TOTAL_BYTES] = { 0 };      // Response POS data buffer

            u16 POS_BUF_PTR = 0;                 //
            u16 POS_SEQUENCE = 0;        // response result check

            int32_t POS_AREA_MAX = 10000;                                          // PCV센서 범위 Maximum(mm)
            int32_t POS_AREA_MIN = (-100);   // PCV센서 범위 Minimum 

            //To check vaild Condition
            u16 NomCnd_Cnt = 0;
            u16 ErrCnd_Cnt = 0;
            
           
            DATA_RECEIVE_HANDLER onDataReceived;
            State state = NotInited;
            thread executer;
           
            bool LoadParameter(char* path)
            {
                if (true)
                {
                    throw std::exception("File is not exist");
                }
                else
                {

                }

   
            }

            
            void InitValues()
            {
                XPS = 0;
                YPS = 0;
                X_OFFSET = 0;
                Y_OFFSET = 0;
                ANGLE = 0;
 
                TagNo = 0;
 
                Dir = 0;
                DirOld = 0;
 
                Color = 0;
                ColorOld = 0;
 
                SensorErr = 0;
                POS_SEQUENCE = 1;

            }

            void INIT_BUFFER()
            {
                u16  i;

                POS_SEQUENCE = 2;
                POS_BUF_PTR = 0;
                for (i = 0; i < (POS_PGV100_TOTAL_BYTES); i++) { POS_BUF[i] = 0; }
                POS_CommTimer_Reset(POS_TIMEOUT);
            }



            void WORK_LOOP()
            {
                /*
                    메인 작업 
                
                
                */


                INIT_BUFFER();
                while (true)
                {




                    if (this.state > 10)
                    //매직 플래그를 사용해서
                    //완료 후의 상태는 10 이상으로 정의
                    {
                        break;
                    }
                }
                onDataReceived(POS_BUF);
                executer.join();
            }

            void StopLoop()
            {
                this->state = Error;
            }

        public:

            void Init(char* path)
            {
                try
                {
                    //초기화 메서드 실행

                    this->InitValues();
                    this->LoadParameter(path);




                }
                catch ()
                {
                    cout << "File is not exist"
                }
               
            };

            void AddListener(DATA_RECEIVE_HANDLER callback)
            {
                this->onDataReceived = callback;
            }

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
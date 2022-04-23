//declare standard c++14 headers
#include <iostream>
#include <sstream>
#include <deque>
#include <thread>


//declare own ros2 headers
#include "rclcpp/rclcpp.hpp"
#include "PNF_PGV100.hpp"

namespace Nyamkani
{
    class MODULE_PGV100
    {
        private:
            thread executer;
            PosSensorVal* instPtr;


        public:

            void Init()
            {
                POS_INIT_VALUES(instPtr);

                

            };



            void Request()
            {
                POS_BUFFER_INIT(instPtr);


            };


            ~MODULE_PGV100()
            {

                
            };


    
    }




}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  //auto sick_nav350_publisher = std::make_shared<sick_nav350>();
  //signal(SIGINT,ExitHandler);
  //int ret = sick_nav350_publisher->work_loop();

  rclcpp::shutdown();
  //return ret;
  //return 0;
}
/*
 * IntegratedSensorManager.h
 *
 *  Created on: Jul 15, 2022
 *      Author: studio3s
 */

#ifndef INC_EXTINC_INTEGRATEDSENSORMANAGER_H_
#define INC_EXTINC_INTEGRATEDSENSORMANAGER_H_


#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <queue>

#include <Extinc/CommonSensor.h>
#include <Extinc/PNFPosSensor.h>


namespace Nyamkani
{
	enum pickliftmask
	{
		AORG_L = 0x0001,    	//ARM origin sensor left   0x0001
		AORG_R = 0x0002,  		//ARM origin sensor right  0x0002
		WORG_L = 0x0004,		//WIDTH origin sensor left  0x0004
		WORG_R = 0x0008,		//WIDTH origin sensor right  0x0008
		TORG_L = 0x0010,		//TURN origin sensor left  0x0010
		TORG_C = 0x0020,		//TURN origin sensor center  0x0020
		TORG_R = 0x0040,		//TURN origin sensor right 0x0040
		LORG_U = 0x0080,		//LIFT upper limit sensor  0x0080
		LORG_C = 0x0100,		//LIFT origin sensor  0x0100
		LORG_L = 0x0200,		//LIFT lower limit sensor  0x0200
		//Reserved = 0x0400,
		//Reserved = 0x0800,
		IN_L = 0x1000,		//in load check sensor left  0x1000
		IN_R = 0x2000,		//in load check sensor right  0x2000
		OUT_L = 0x4000,		//out load check sensor left  0x4000
		OUT_R = 0x8000,		//out load check sensor right  0x8000
	};


	class SensorManager
	{
		private:
			static bool bDestroyed_;
			static SensorManager* pIns_;

			//sensor vectors
			std::vector<CommonSensor*> common_sensors_;
			std::vector<PNFPosSensor*> pnf_pos_sensors_;

			//uint16_t commonsensorindex_;
			//uint16_t pnfsensorindex_;

			uint16_t common_sensor_data_;
			//uint16_t PNF_sensor_data_;


			SensorManager();
			SensorManager(const SensorManager& other);
			SensorManager& operator=(const SensorManager& ref);
			~SensorManager();

			static void ManagerCreate();
			static void ManagerDelete();
			void DeleteAllSensor();


		public:
			static SensorManager& GetInstance();


		//Sensor itselfs
		private:
			uint16_t common_sensors_status_;

		public:
			void RegisterCommonSensor();
			void CommonSensorsGetValue();
			uint16_t CommonSensorsGetData();
			void RegisterPNFPosSensor();
			void PNFPosSensorInitialize();
			void PNFPosSensorGetValue();
	};



}





#endif /* INC_EXTINC_INTEGRATEDSENSORMANAGER_H_ */

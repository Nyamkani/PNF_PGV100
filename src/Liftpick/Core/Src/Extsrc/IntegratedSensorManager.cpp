/*
 * IntegratedSensorManager.cpp
 *
 *  Created on: Jul 15, 2022
 *      Author: studio3s
 */



#include "Extinc/IntegratedSensorManager.h"




namespace Nyamkani
{
	//--------------------------------------------------------------------------------------------------Single instance:
	bool SensorManager::bDestroyed_ = false;
	SensorManager* SensorManager::pIns_ = NULL;


	SensorManager::SensorManager() {};
	SensorManager::SensorManager(const SensorManager& other){};
	SensorManager& SensorManager::operator=(const SensorManager& ref) {return *this;};
	SensorManager::~SensorManager(){bDestroyed_ = true;}

	void SensorManager::ManagerCreate()
	{
		SensorManager ins;
		pIns_ = &ins;
	}

	void SensorManager::ManagerDelete()
	{
		pIns_->~SensorManager();
	}

	SensorManager& SensorManager::GetInstance()
	{
		if(bDestroyed_)
		{
			pIns_ = new SensorManager();
			// new(pIns) =  SensorManager;
			atexit(ManagerDelete);
			bDestroyed_ = false;
		}
		else if (pIns_ == NULL)
		{
			ManagerCreate();
		}
	     return *pIns_;
	}


	//---------------------------------------------------------------------sensor function
	void SensorManager::DeleteAllSensor()
	{
		for (auto& index : common_sensors_) delete (index);
		common_sensors_.clear();
		for (auto& index : pnf_pos_sensors_) delete (index);
		pnf_pos_sensors_.clear();
	}







	//common sensor
	void SensorManager::RegisterCommonSensor()
	{
		common_sensors_.emplace_back(new CommonSensor(AORG_L, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_1));
		common_sensors_.emplace_back(new CommonSensor(AORG_R, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_2));
		common_sensors_.emplace_back(new CommonSensor(WORG_L, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_3));
		common_sensors_.emplace_back(new CommonSensor(WORG_R, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_4));
		common_sensors_.emplace_back(new CommonSensor(TORG_L, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_5));
		common_sensors_.emplace_back(new CommonSensor(TORG_C, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_6));
		common_sensors_.emplace_back(new CommonSensor(TORG_R, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_7));
		common_sensors_.emplace_back(new CommonSensor(LORG_U, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_8));
		common_sensors_.emplace_back(new CommonSensor(LORG_C, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_9));
		common_sensors_.emplace_back(new CommonSensor(LORG_L, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_10));
		common_sensors_.emplace_back(new CommonSensor(IN_L, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_11));
		common_sensors_.emplace_back(new CommonSensor(IN_R, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_12));
		common_sensors_.emplace_back(new CommonSensor(OUT_L, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_13));
		common_sensors_.emplace_back(new CommonSensor(OUT_R, ActiveH, Filter_5, GPIOA, LL_GPIO_PIN_14));
		common_sensors_.resize(common_sensors_.size());
	}

	void SensorManager::CommonSensorsGetValue()
	{
		for (auto& index : common_sensors_) index->main_loop();
	}

	uint16_t SensorManager::CommonSensorsGetData()
	{
		uint16_t common_sensor_data = 0;
		for (auto& index : common_sensors_)
		{
			common_sensor_data |=  ((index->GetSensorIndex()) * (index->GetSensorData()));
		}
		return common_sensor_data;
	}








	//PNFPos sensor
	void SensorManager::RegisterPNFPosSensor()
	{
		pnf_pos_sensors_.emplace_back(new PNFPosSensor(pgv100, rs485, port_5, 10.0, 10.0, 15.0));
		//pnf_pos_sensors_.emplace_back(new PNFPosSensor(1, 0, 6, 10.0, 10.0, 15.0));
		pnf_pos_sensors_.resize(pnf_pos_sensors_.size());
	}


	void SensorManager::PNFPosSensorInitialize()
	{
		for (auto& index : pnf_pos_sensors_) index->Initializaition();
	}

	void SensorManager::PNFPosSensorGetValue()
	{
		for (auto& index : pnf_pos_sensors_) index->main_loop();
	}








}

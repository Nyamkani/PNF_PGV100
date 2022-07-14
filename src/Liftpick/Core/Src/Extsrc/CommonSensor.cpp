/*
 * CommonSensor.cpp
 *
 *  Created on: Jul 14, 2022
 *      Author: studio3s
 */


#include "Extinc/CommonSensor.h"

namespace Nyamkani
{
	CommonSensor::CommonSensor(){}

	CommonSensor::CommonSensor(uint16_t SensorIndex, bool active_type,
							   uint8_t max_filter_cnt, GPIO_TypeDef* GPIOx,
							   uint32_t PinMask)
	{
		this->SensorIndex_ = SensorIndex;
		this->active_type_ = active_type;
		this->max_filter_cnt_ = max_filter_cnt;
		if(this->GPIOx_ == nullptr ) this->GPIOx_ = GPIOx;
		this->PinMask_ = PinMask;
	}

	bool CommonSensor::GetSensorValue() const {return this->output_;}


	bool CommonSensor::CheckSensorValue()
	{
		if(LL_GPIO_IsInputPinSet(this->GPIOx_, this->PinMask_) == 1) (this->state) = 1;
		else (this->state) = 0;

		if(!active_type_) this->state = !(this->state);
		return this->state;
	}

	bool CommonSensor::IsValueFiltered()
	{
		return (this->now_filter_cnt_>= this->max_filter_cnt_);
	}

	void CommonSensor::FilterCountUp()
	{
		if(this->now_filter_cnt_< this->max_filter_cnt_) this->now_filter_cnt_++;
	}

	void CommonSensor::FilterStatusChanged(){this->now_filter_cnt_ = 0;}


	void CommonSensor::main_loop()
	{
		bool prev_val = this->state;

		if(prev_val != CheckSensorValue()) FilterStatusChanged();
		else FilterCountUp();

		if(IsValueFiltered()) this->output_ = prev_val;
	}
}

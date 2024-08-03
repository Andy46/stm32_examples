/*
 * motor.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

#include <stdint.h>

#include "tim.h"

namespace HARDWARE::SENSORS
{

class N20_ENCODER
{
private:
	TIM_HandleTypeDef& tim;
	int32_t pos;
	int16_t previousCount;

public:
	N20_ENCODER(TIM_HandleTypeDef& tim);
	virtual ~N20_ENCODER() = default;

	void init();

	void configure();

	int32_t updatePos();
	int32_t getPos();
};

} /* namespace HARDWARE::ACTUATORS */

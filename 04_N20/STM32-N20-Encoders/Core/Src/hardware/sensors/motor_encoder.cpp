/*
 * motor_encoder.cpp
 *
 *  Created on: Aug 3, 2024
 *      Author: agamb
 */

#include "hardware/sensors/motor_encoder.h"

namespace
{

inline uint16_t getSTMEncoderCount(TIM_HandleTypeDef& tim) { return tim.Instance->CNT; }

} /* namespace */

namespace HARDWARE::SENSORS
{

N20_ENCODER::N20_ENCODER(TIM_HandleTypeDef& tim) :
		tim(tim), pos(0), previousCount(0)
{

}


void N20_ENCODER::init()
{
	HAL_TIM_Encoder_Start(&tim, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	pos = 0;
	previousCount = getSTMEncoderCount(tim);
}

void N20_ENCODER::configure()
{

}

int32_t N20_ENCODER::updatePos()
{
	const int16_t currentCount = static_cast<int16_t>(getSTMEncoderCount(tim));

	int32_t deltaCount = currentCount - previousCount;

	pos += deltaCount;

	previousCount = currentCount;
	return pos;
}

int32_t N20_ENCODER::getPos()
{
	return pos;
}

} /* namespace HARDWARE::SENSORS */

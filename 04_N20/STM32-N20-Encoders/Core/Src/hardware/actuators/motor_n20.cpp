/*
 * motor_n20.cpp
 *
 *  Created on: Aug 3, 2024
 *      Author: agamb
 */

#include "hardware/actuators/motor_n20.h"

namespace HARDWARE::ACTUATORS
{

N20::N20(std::shared_ptr<DRV8833> driver, DRV8833::MOTOR_ID motorID,
		 std::shared_ptr<SENSORS::N20_ENCODER> encoder) :
		 driver(driver), motorID(motorID), encoder(encoder)
{

}

} /* namespace HARDWARE::ACTUATORS */

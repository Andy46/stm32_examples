/*U67786
 * motor_n20.h
 *
 *  Created on: Aug 3, 2024
 *      Author: agamb
 */
#pragma once

#include <memory>

#include "hardware/actuators/motor_driver.h"
#include "hardware/sensors/motor_encoder.h"

namespace HARDWARE::ACTUATORS
{

class N20
{
public:


private:
	std::shared_ptr<ACTUATORS::DRV8833> driver;
	ACTUATORS::DRV8833::MOTOR_ID motorID;

	std::shared_ptr<SENSORS::N20_ENCODER> encoder;

public:

	N20(std::shared_ptr<DRV8833> driver, DRV8833::MOTOR_ID motorID,
			std::shared_ptr<SENSORS::N20_ENCODER> encoder);
	virtual ~N20() = default;




};

}

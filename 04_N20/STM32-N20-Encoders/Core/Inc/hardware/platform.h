/*
 * platform.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

#include "main.h"

#include <stddef.h>
#include <stdint.h>

#include "actuators/motor_n20.h"
#include "extra/gpio.h"

namespace HARDWARE
{

class Platform
{
private:
	std::shared_ptr<EXTRA::GPIO> led;

	std::shared_ptr<ACTUATORS::N20> motor_left;
	std::shared_ptr<ACTUATORS::N20> motor_right;

	std::shared_ptr<SENSORS::N20_ENCODER> encoder_left;
	std::shared_ptr<SENSORS::N20_ENCODER> encoder_right;

public:
	Platform(std::shared_ptr<EXTRA::GPIO> led,
			 std::shared_ptr<ACTUATORS::N20> motor_left,
	 	 	 std::shared_ptr<ACTUATORS::N20> motor_right,
			 std::shared_ptr<SENSORS::N20_ENCODER> encoder_left,
			 std::shared_ptr<SENSORS::N20_ENCODER> encoder_right);
	virtual ~Platform() = default;

	void init();

	void checkStatus();

	void run_test();

};

} /* namespace HARDWARE */

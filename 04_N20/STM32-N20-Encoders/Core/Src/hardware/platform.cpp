/*
 * platform.cpp
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#include "hardware/platform.h"

/* C/C++ includes */
#include <stdio.h>

namespace HARDWARE
{

Platform::Platform(std::shared_ptr<EXTRA::GPIO> led,
				   std::shared_ptr<ACTUATORS::N20> motor_left,
		 	 	   std::shared_ptr<ACTUATORS::N20> motor_right,
				   std::shared_ptr<SENSORS::N20_ENCODER> encoder_left,
				   std::shared_ptr<SENSORS::N20_ENCODER> encoder_right) :
		led(led),
		motor_left(motor_left),     motor_right(motor_right),
		encoder_left(encoder_left), encoder_right(encoder_right)
{

}

void Platform::init()
{
	led->clear();

//	motor_left->init();
//	motor_left->configure();
//
//	motor_right->init();
//	motor_right->configure();

	encoder_left->init();
	encoder_left->configure();

	encoder_right->init();
	encoder_right->configure();
}

void Platform::checkStatus()
{
	printf("\n=== Platform status ===\n");
}

void Platform::run_test()
{
	printf("\n=== Starting tests===\n");

	bool led_on = false;

	printf("EncoderLeft - EncoderRight\n");
	while (1)
	{
		// Toggle LED
		if (led_on)
		{
			led->clear();
			led_on = false;
		}
		else
		{
			led->set();
			led_on = true;
		}

		printf("%11d - %12d\n", (int) encoder_left->updatePos(), (int) encoder_right->updatePos());

		HAL_Delay(20);
	}
}
} /* namespace HARDWARE */

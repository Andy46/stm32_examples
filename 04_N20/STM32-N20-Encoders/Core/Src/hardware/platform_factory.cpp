/*
 * platformfactory.h
 *
 *  Created on: Jul 20, 2024
 *      Author: agamb
 */

#include "hardware/platform_factory.h"

#include <memory>
#include <array>

// STM32
//#include "main.h"
#include "tim.h"

// Hardware
#include "hardware/platform.h"

// Actuators
#include "hardware/actuators/motor_n20.h"
#include "hardware/actuators/motor_driver.h"

// Sensors
//#include "hardware/sensors/encoder.h"

// Extra
#include "hardware/extra/stm32_gpio.h"

namespace HARDWARE
{

Platform PlatformFactory::getPlatform()
{

	/******************/
	/* Communications */
	/******************/

	// Onboard LED
	auto led = std::make_shared<EXTRA::STM32_GPIO> (LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);

	// Encoders
	auto encoderA = std::make_shared<SENSORS::N20_ENCODER> (htim2);
	auto encoderB = std::make_shared<SENSORS::N20_ENCODER> (htim3);

	// Drivers
	auto driver = std::make_shared<ACTUATORS::DRV8833> ();

	// Motor
	auto motorA = std::make_shared<ACTUATORS::N20> (driver, ACTUATORS::DRV8833::MOTOR_A, encoderA);
	auto motorB = std::make_shared<ACTUATORS::N20> (driver, ACTUATORS::DRV8833::MOTOR_B, encoderB);

	// Create the platform and return it
	return Platform (led, motorA, motorB, encoderA, encoderB);
}

} /* namespace HARDWARE */

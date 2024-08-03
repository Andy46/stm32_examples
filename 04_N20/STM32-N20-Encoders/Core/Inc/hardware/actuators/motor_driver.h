/*
 * motor_driver.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

namespace HARDWARE::ACTUATORS
{

class DRV8833
{
public:

	enum MOTOR_ID
	{
		MOTOR_A = 0,
		MOTOR_B
	};

private:

public:

	DRV8833();
	virtual ~DRV8833() = default;



};

} /* namespace HARDWARE::ACTUATORS */

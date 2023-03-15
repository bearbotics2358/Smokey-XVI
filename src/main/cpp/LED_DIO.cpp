/* LED_DIO.cpp - Control LED lights representing desired game piece

 */

#include <stdio.h> // printf
#include <stdlib.h> // atoi
#include <Prefs.h>
#include "LED_DIO.h"

LED_DIO::LED_DIO(int DIO_pin):
	a_Output(DIO_pin)
{
	Init();
}

void LED_DIO::Init()
{
	SetTargetType(target_type_enum::CONE);
}

void LED_DIO::Update()
{
	// refresh the output pin
	// this should not be needed
	SetTargetType(target_type);
}

void LED_DIO::SetTargetType(target_type_enum target_type_param)
{
	target_type = target_type_param;
	if(target_type == target_type_enum::CONE) {
		// set the output HIGH for a CONE
		a_Output.Set(true);
	} else {
		// set the output LOW for a CUBE
		a_Output.Set(false);
	}
}

target_type_enum LED_DIO::GetTargetType()
{
	return target_type;
}

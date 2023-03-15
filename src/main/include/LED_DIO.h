/* LED_DIO.cpp - Control LED lights representing desired game piece

 */

#ifndef H_LED_DIO
#define H_LED_DIO

#include <frc/DigitalOutput.h>
#include "TOF_protocol.h" //yay i can use CONE and CUBE

#define BUFF_SIZE 256

class LED_DIO
{
public:

	LED_DIO(int DIO_pin);
	virtual ~LED_DIO() = default;

	void Init();
	void Update();

	void SetTargetType(target_type_enum target_type_param);
	target_type_enum GetTargetType();

private:
	frc::DigitalOutput a_Output;
	target_type_enum target_type = target_type_enum::CONE;
} ;

#endif

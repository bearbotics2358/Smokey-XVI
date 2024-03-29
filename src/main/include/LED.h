/* LED.cpp - Control LED lights represnting desired game piece

 */

#ifndef H_LED
#define H_LED

#include "TOF_protocol.h" //yay i can use CONE and CUBE
#include <frc/SerialPort.h>

#define BUFF_SIZE 256

class LED
{
public:

	LED();
	virtual ~LED() = default;

	void Init();
	void Update();

	void ProcessReport();
	enum target_range_enum GetTargetRangeIndicator();
	void SetTargetType(target_type_enum target_type_param);
	target_type_enum GetTargetType();

private:
#ifdef COMP_BOT  // Not available on the practice bot
	frc::SerialPort m_serial;
#endif
	char rx_buff[BUFF_SIZE];
	int rx_index = 0;
	target_type_enum target_type = target_type_enum::CONE;
} ;

#endif

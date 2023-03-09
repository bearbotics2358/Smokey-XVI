/* LED.cpp - Control LED lights represnting desired game piece

 */

#include <stdio.h> // printf
#include <stdlib.h> // atoi
#include <Prefs.h>
#include "LED.h"

LED::LED():
	m_serial(BAUD_RATE_ARDUINO, USB_PORT_ARDUINO, DATA_BITS_ARDUINO, PARITY_ARDUINO, STOP_BITS_ARDUINO)
	// comments from 2018:
	// USB1 is the onboard port closest to the center of the rio
	// I dunno which one USB2 is yet. (Rio docs aren't very helpful)

{
	Init();
}

void LED::Init()
{
	int i;

	rx_index = 0;
	for(i = 0; i < BUFF_SIZE; i++) {
		rx_buff[i] = 0;
	}
	SetTargetType(target_type_enum::CONE);
}

void LED::Update()
{
	// call this routine periodically to check for any readings and store
	// into result registers

	// get report if there is one
	// every time called, and every time through loop, get repotr chars if available
	// and add to rx buffer
	// when '\r' (or '\t') found, process reading
	
	while (m_serial.GetBytesReceived() > 0) {
		m_serial.Read(&rx_buff[rx_index], 1);

		printf("LED LED LED LED: %c\n", rx_buff[rx_index]);
 
		
		if((rx_buff[rx_index] == '\r') 
			 || (rx_buff[rx_index] == '\n')) {

			// process report
			if(rx_index == 0) {
				// no report
				continue;
			}

			// terminate the report string
			rx_buff[rx_index] = 0;

			ProcessReport();
			
			// printf("LED report: rx_buff\n");

			// reset for next report
			rx_index = 0;
		} else {
			// have not received end of report yet
			if(rx_index < BUFF_SIZE - 1) {
				rx_index++;
			}
		}
	}
}

// instead of atoi(), UltrasonicSerial used strtol(&readBuffer[1], (char **)NULL, 10);


void LED::ProcessReport()
{
	// parse report
	// no action needed, no report expected
}

void LED::SetTargetType(target_type_enum target_type_param)
{
	char cmd[8];
	strncpy(cmd, "1,1,1\r\n", 8);
	target_type = target_type_param;
	// lazy way to build a message
	cmd[4] = target_type ? '1' : '0';
	m_serial.Write(cmd, strlen(cmd));
	m_serial.Flush();
}

target_type_enum LED::GetTargetType()
{
	return target_type;
}

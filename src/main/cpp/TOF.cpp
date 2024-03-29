/* TOF.cpp - receive measurements and status from Feather connected to TOF array and
	 control game element type

*/

#include <stdio.h> // printf
#include <stdlib.h> // atoi
#include <Prefs.h>
#include "TOF.h"
#include <frc/smartdashboard/SmartDashboard.h>

TOF::TOF()
#ifdef COMP_BOT  // Not available on the practice bot
:
	m_serial(BAUD_RATE_TOF, USB_PORT_TOF, DATA_BITS_TOF, PARITY_TOF, STOP_BITS_TOF)
#endif
	// comments from 2018:
	// USB1 is the onboard port closest to the center of the rio
	// I dunno which one USB2 is yet. (Rio docs aren't very helpful)

{
	Init();
}

void TOF::Init()
{
	int i;

	rx_index = 0;
	for(i = 0; i < BUFF_SIZE; i++) {
		rx_buff[i] = 0;
	}
	range = 9999; 
	target_range_indicator = target_range_enum::TARGET_NOT_PRESENT;
	target_type = target_type_enum::CONE;
}

void TOF::Update()
{
#ifdef COMP_BOT  // Not available on the practice bot
	// call this routine periodically to check for any readings and store
	// into result registers

  // get report if there is one
  // every time called, and every time through loop, get repotr chars if available
	// and add to rx buffer
  // when '\r' (or '\t') found, process reading
	
	while (m_serial.GetBytesReceived() > 0) {
		m_serial.Read(&rx_buff[rx_index], 1);

		// printf("TOF: %c\n", rx_buff[rx_index]);

		
    if((rx_buff[rx_index] == '\r') 
			 || (rx_buff[rx_index] == '\n')) {

      // process report
      if(rx_index == 0) {
        // no report
        continue;
      }

			// terminate the report string
			rx_buff[rx_index] = 0;

			printf("TOF report: %s\n", rx_buff);

			ProcessReport();
			

      // reset for next report
      rx_index = 0;
    } else {
      // have not received end of report yet
      if(rx_index < BUFF_SIZE - 1) {
        rx_index++;
      }
    }
  }
#endif
}

// instead of atoi(), UltrasonicSerial used strtol(&readBuffer[1], (char **)NULL, 10);


void TOF::ProcessReport()
{
	// parse report
  // get msg_type and data_len
  int msg_type = atoi(strtok(rx_buff, ","));
  int data_len = atoi(strtok(NULL, ","));

  switch(msg_type) {
	case TOF_RIO_msgs_enum::RANGE:
		if(data_len >= 2) {
			target_range_indicator = (target_range_enum)atoi(strtok(NULL, ","));
			frc::SmartDashboard::PutNumber("TOF range indicator: ", target_range_indicator);
			range = atoi(strtok(NULL, ","));
		}
		break;

	case TOF_RIO_msgs_enum::HISTOGRAM:
	case TOF_RIO_msgs_enum::ARM_ANGLE:
	case TOF_RIO_msgs_enum::RAW_PIXEL_DATA:
	case TOF_RIO_msgs_enum::RESERVED:
		break;
		
	default:
		// do nothing
		printf("unkown report type\n");
		break;
	}
}

enum target_range_enum TOF::GetTargetRangeIndicator()
{
	return target_range_indicator;
}
	

int TOF::GetMM()
{
	return range;
}

float TOF::GetInches()
{
	return (1.0 * range)/25.4;
}

void TOF::SetTargetType(target_type_enum target_type_param)
{
#ifdef COMP_BOT  // Not available on the practice bot
	char cmd[8];
	strncpy(cmd, "1,1,1\r\n", 8);
	target_type = target_type_param;
	// lazy way to build a message
	cmd[4] = target_type ? '1' : '0';
	m_serial.Write(cmd, strlen(cmd));
	m_serial.Flush();
#endif
}

target_type_enum TOF::GetTargetType()
{
	return target_type;
}

void TOF::EnableHistogram(int enable)
{
#ifdef COMP_BOT  // Not available on the practice bot
	char cmd[8];
	strncpy(cmd, "2,1,1\r\n", 8);
	cmd[4] = enable ? '1' : '0';
	m_serial.Write(cmd, strlen(cmd));
	m_serial.Flush();
#endif
}

void TOF::EnableRawPixelData(int enable)
{
#ifdef COMP_BOT  // Not available on the practice bot
	char cmd[8];
	strncpy(cmd, "4,1,1\r\n", 8);
	cmd[4] = enable ? '1' : '0';
	m_serial.Write(cmd, strlen(cmd));
	m_serial.Flush();
#endif
}

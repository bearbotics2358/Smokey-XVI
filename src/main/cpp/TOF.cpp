/* TOF.cpp - receive measurements and status from Feather connected to TOF array and
	 control game element type

*/

#include <stdio.h> // printf
#include <stdlib.h> // atoi
#include "TOF.h"

TOF::TOF():
	a_Ultra(BAUD_RATE_TOF, USB_PORT_TOF, DATA_BITS_TOF, PARITY_TOF, STOP_BITS_TOF)
	// USB1 is the onboard port closest to the center of the rio
	// I dunno which one USB2 is yet. (Rio docs aren't very helpful)

{
	Init();
}

TOF::Init()
{
	int i;

	rx_index = 0;
	for(i = 0; i < BUFF_SIZE; i++) {
		rx_buff[i] = 0;
	}
	range = 9999; 
	target_range = target_range_enum::TARGET_NOT_PRESENT;
	target_type = target_type_enum::CONE;
}

void TOF::Update(){
	// call this routine periodically to check for any readings and store
	// into result registers

  // get report if there is one
  // every time called, and every time through loop, get repotr chars if available
	// and add to rx buffer
  // when '\r' (or '\t') found, process reading
	
	while (a_Ultra.GetBytesReceived() > 0) {
		a_Ultra.Read(&rx_buff[rx_index], 1);
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
			
			// printf("TOF report: rx_buff\n");

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

target_range_enum TOF:GetTargetRangeIndicator()
{
	return target_range_indicator;
}
	

int TOF::GetMM()
{
	return range;
}

float TOF::GetInches()
{
	return ((float)range)/25.4);
}

void TOF::SetTargetType(target_type_enum target_type_param)
{
	target_type = target_type_param;
	// lazy way to build a message
	char cmd = target_type ? "1,1,1\r\n" : "1,1,0\r\n";
	a_Ultra.Write(cmd, strlen(cmd);
	a_Ultra.Flush();
}

void TOF::EnableHistogram(int enable)
{
	// lazy way to build a message
	char cmd = enable ? "2,1,1\r\n" : "2,1,0\r\n";
	a_Ultra.Write(cmd, strlen(cmd);
	a_Ultra.Flush();
}

void TOF::EnableRawPixelData(int enable)
{
	// lazy way to build a message
	char cmd = enable ? "4,1,1\r\n" : "4,1,0\r\n";
	a_Ultra.Write(cmd, strlen(cmd);
	a_Ultra.Flush();
}

/* TOF.cpp - receive measurements and status from Feather connected to TOF array and
	 control game element type

*/

#ifndef H_TOF
#define H_TOF

#include <WPILib.h>
#include "TOF_protocol.h"

#define BUFF_SIZE 256

class TOF
{
 public:

	TOF();
	virtual ~TOF() = default;

	void Init();
	void Update();

	int GetMM(int port);
	float GetInches(int port);

 private:
	SerialPort a_TOF;
	char rx_buff[BUFF_SIZE];
	int rx_index = 0;
	int range = 9999; // range in mm
	int target_range_indicator = target_range_enum::TARGET_NOT_PRESENT;
	target_type_enum target_type = target_type_enum::CONE;
} ;

#endif

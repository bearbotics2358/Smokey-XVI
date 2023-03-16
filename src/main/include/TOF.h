/* TOF.cpp - receive measurements and status from Feather connected to TOF array and
	 control game element type

*/

#ifndef H_TOF
#define H_TOF

#include "TOF_protocol.h"
#include <frc/SerialPort.h>

#define BUFF_SIZE 256

class TOF
{
 public:

	TOF();
	virtual ~TOF() = default;

	void Init();
	void Update();

	void ProcessReport();
	enum target_range_enum GetTargetRangeIndicator();
	void SetTargetType(target_type_enum target_type_param);
	target_type_enum GetTargetType();
	void EnableHistogram(int enable);
	void EnableRawPixelData(int enable);

	int GetMM();
	float GetInches();

 private:
#ifdef COMP_BOT  // Not available on the practice bot
	frc::SerialPort m_serial;
#endif
	char rx_buff[BUFF_SIZE];
	int rx_index = 0;
	int range = 9999; // range in mm
	target_range_enum target_range_indicator = target_range_enum::TARGET_NOT_PRESENT;
	target_type_enum target_type = target_type_enum::CONE;
} ;

#endif

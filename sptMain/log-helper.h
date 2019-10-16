#ifndef LOG_HELPER_H
#define LOG_HELPER_H

#include "ns3/core-module.h"
#include <vector>
namespace ns3 {

class LogHelper{
public:
	LogHelper(uint16_t array[]);
	~LogHelper();
	void SetLogLevel(LogLevel &l0, LogLevel &l1, LogLevel &l2, LogLevel &l3,
			LogLevel &l4, LogLevel &l5, LogLevel &l6, LogLevel &l7, LogLevel &l8);
private:
	uint16_t m_logIndex[9];
};
}

#endif /*LOG_HELPER_H*/

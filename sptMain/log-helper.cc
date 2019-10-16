#include "ns3/log.h"
#include "log-helper.h"

namespace ns3 {

LogHelper::LogHelper(uint16_t array[]) {
	for (uint32_t i = 0; i < 9; i++) {
		m_logIndex[i] = array[i];
	}
}

LogHelper::~LogHelper(){

}

void LogHelper::SetLogLevel(LogLevel &l0,LogLevel &l1, LogLevel &l2, LogLevel &l3,
		LogLevel &l4, LogLevel &l5, LogLevel &l6, LogLevel &l7, LogLevel &l8){         //参数表不是传进来的吗？为什么放在等号左边？
	switch (m_logIndex[0]) {
		case 0:
			l0 = LOG_NONE;
			break;
		case 1:
			l0 = LOG_LEVEL_ERROR;
			break;
		case 7:
			l0 = LOG_LEVEL_DEBUG;
			break;
		case 15:
			l0 = LOG_LEVEL_INFO;
			break;
		case 31:
			l0 = LOG_LEVEL_FUNCTION;
			break;
		case 63:
			l0 = LOG_LEVEL_LOGIC;
			break;
		default:
			l0 = LOG_LEVEL_DEBUG;
			break;
		}

		switch (m_logIndex[1]) {
		case 0:
			l1 = LOG_NONE;
			break;
		case 1:
			l1 = LOG_LEVEL_ERROR;
			break;
		case 7:
			l1 = LOG_LEVEL_DEBUG;
			break;
		case 15:
			l1 = LOG_LEVEL_INFO;
			break;
		case 31:
			l1 = LOG_LEVEL_FUNCTION;
			break;
		case 63:
			l1 = LOG_LEVEL_LOGIC;
			break;
		default:
			l1 = LOG_LEVEL_ERROR;
			break;
		}

		switch (m_logIndex[2]) {
		case 0:
			l2 = LOG_NONE;
			break;
		case 1:
			l2 = LOG_LEVEL_ERROR;
			break;
		case 7:
			l2 = LOG_LEVEL_DEBUG;
			break;
		case 15:
			l2 = LOG_LEVEL_INFO;
			break;
		case 31:
			l2 = LOG_LEVEL_FUNCTION;
			break;
		case 63:
			l2 = LOG_LEVEL_LOGIC;
			break;
		default:
			l2 = LOG_LEVEL_ERROR;
			break;
		}

		switch (m_logIndex[3]) {
		case 0:
			l3 = LOG_NONE;
			break;
		case 1:
			l3 = LOG_LEVEL_ERROR;
			break;
		case 7:
			l3 = LOG_LEVEL_DEBUG;
			break;
		case 15:
			l3 = LOG_LEVEL_INFO;
			break;
		case 31:
			l3 = LOG_LEVEL_FUNCTION;
			break;
		case 63:
			l3 = LOG_LEVEL_LOGIC;
			break;
		default:
			l3 = LOG_LEVEL_ERROR;
			break;
		}

		switch (m_logIndex[4]) {
		case 0:
			l4 = LOG_NONE;
			break;
		case 1:
			l4 = LOG_LEVEL_ERROR;
			break;
		case 7:
			l4 = LOG_LEVEL_DEBUG;
			break;
		case 15:
			l4 = LOG_LEVEL_INFO;
			break;
		case 31:
			l4 = LOG_LEVEL_FUNCTION;
			break;
		case 63:
			l4 = LOG_LEVEL_LOGIC;
			break;
		default:
			l4 = LOG_LEVEL_ERROR;
			break;
		}

		switch (m_logIndex[5]) {
		case 0:
			l5 = LOG_NONE;
			break;
		case 1:
			l5 = LOG_LEVEL_ERROR;
			break;
		case 7:
			l5 = LOG_LEVEL_DEBUG;
			break;
		case 15:
			l5 = LOG_LEVEL_INFO;
			break;
		case 31:
			l5 = LOG_LEVEL_FUNCTION;
			break;
		case 63:
			l5 = LOG_LEVEL_LOGIC;
			break;
		default:
			l5 = LOG_LEVEL_ERROR;
			break;
		}
		switch (m_logIndex[6]) {
		case 0:
			l6 = LOG_NONE;
			break;
		case 1:
			l6 = LOG_LEVEL_ERROR;
			break;
		case 7:
			l6 = LOG_LEVEL_DEBUG;
			break;
		case 15:
			l6 = LOG_LEVEL_INFO;
			break;
		case 31:
			l6 = LOG_LEVEL_FUNCTION;
			break;
		case 63:
			l6 = LOG_LEVEL_LOGIC;
			break;
		default:
			l6 = LOG_NONE;
			break;
		}

		switch (m_logIndex[7]) {
		case 0:
			l7 = LOG_NONE;
			break;
		case 1:
			l7 = LOG_LEVEL_ERROR;
			break;
		case 7:
			l7 = LOG_LEVEL_DEBUG;
			break;
		case 15:
			l7 = LOG_LEVEL_INFO;
			break;
		case 31:
			l7 = LOG_LEVEL_FUNCTION;
			break;
		case 63:
			l7 = LOG_LEVEL_LOGIC;
			break;
		default:
			l7 = LOG_LEVEL_ERROR;
			break;
		}
		switch (m_logIndex[8]) {
		case 0:
			l8 = LOG_NONE;
			break;
		case 1:
			l8 = LOG_LEVEL_ERROR;
			break;
		case 7:
			l8 = LOG_LEVEL_DEBUG;
			break;
		case 15:
			l8 = LOG_LEVEL_INFO;
			break;
		case 31:
			l8 = LOG_LEVEL_FUNCTION;
			break;
		case 63:
			l8 = LOG_LEVEL_LOGIC;
			break;
		default:
			l8 = LOG_LEVEL_ERROR;
			break;
		}
}

}

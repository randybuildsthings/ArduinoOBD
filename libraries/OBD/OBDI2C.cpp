/*************************************************************************
* Arduino Library for OBD-II UART/I2C Adapter
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* (C)2012-2014 Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#endif
#include <Wire.h>
#include "OBD.h"

/*************************************************************************
* OBD-II I2C Adapter
*************************************************************************/

void COBDI2C::begin(byte addr)
{
	m_addr = addr;
	Wire.begin();
	memset(obdPid, 0, sizeof(obdPid));
	memset(obdInfo, 0, sizeof(obdInfo));
}

bool COBDI2C::init()
{
	m_state = OBD_CONNECTING;
	sendCommand(CMD_QUERY_STATUS);

	char recvbuf[MAX_PAYLOAD_SIZE];
	for (byte n = 0; n < 3; n++) {
		memset(recvbuf, 0, sizeof(recvbuf));
		receive(recvbuf);
		if (!memcmp(recvbuf, "OBD ", 4))
			break;
	}
	if (recvbuf[4] == 'Y') {
		memcpy(pidmap, recvbuf + 16, sizeof(pidmap));
		m_state = OBD_CONNECTED;
		return true;
	} else {
		m_state = OBD_DISCONNECTED;
		return false;
	}
}

bool COBDI2C::read(byte pid, int& result)
{
	sendQuery(pid);
	dataIdleLoop();
	return getResult(pid, result);
}

void COBDI2C::write(char* s)
{
	COMMAND_BLOCK cmdblock = {millis(), CMD_SEND_AT_COMMAND};
	Wire.beginTransmission(m_addr);
	Wire.write((byte*)&cmdblock, sizeof(cmdblock));
	Wire.write(s);
	Wire.endTransmission();
}

bool COBDI2C::sendCommand(byte cmd, uint8_t data, byte* payload, byte payloadBytes)
{
	COMMAND_BLOCK cmdblock = {millis(), cmd, data};
	Wire.beginTransmission(m_addr);
	bool success = Wire.write((byte*)&cmdblock, sizeof(COMMAND_BLOCK)) == sizeof(COMMAND_BLOCK);
	if (payload) Wire.write(payload, payloadBytes);
	Wire.endTransmission();
	return success;
}

byte COBDI2C::receive(char* buffer, int timeout)
{
	uint32_t start = millis();
	byte offset = 0;
	do {
		Wire.requestFrom((byte)m_addr, (byte)MAX_PAYLOAD_SIZE, (byte)1);

		bool hasEnd = false;
		for (byte i = 0; i < MAX_PAYLOAD_SIZE; i++) {
			if ((buffer[offset + i] = Wire.read()) == 0)
				hasEnd = true;
		}

		if (buffer[0] == 0) {
			// data not ready
			dataIdleLoop();
			continue;
		}

		offset += MAX_PAYLOAD_SIZE;
		if (!hasEnd) {
			continue;
		}

		return offset;
	} while(millis() - start < OBD_TIMEOUT_LONG);
	return 0;
}

bool COBDI2C::btInit(uint16_t baudrate)
{
	return sendCommand(CMD_UART_BEGIN, baudrate / 1200);
}

bool COBDI2C::btSend(byte* data, byte length)
{
	return sendCommand(CMD_UART_SEND, 0, data, length);
}

bool COBDI2C::btReceive(byte* buffer, byte bufsize)
{
	if (!sendCommand(CMD_UART_RECV, bufsize)) return false;
	memset(buffer, 0, MAX_PAYLOAD_SIZE);
	delay(10);
	Wire.requestFrom((byte)m_addr, (byte)MAX_PAYLOAD_SIZE, (byte)1);
	Wire.readBytes((char*)buffer, MAX_PAYLOAD_SIZE);
	return true;
}

bool COBDI2C::gpsQuery(GPS_DATA* gpsdata)
{
	if (!sendCommand(CMD_GPS_QUERY, 0)) return false;
	Wire.requestFrom((byte)m_addr, (byte)MAX_PAYLOAD_SIZE, (byte)1);
	Wire.readBytes((char*)gpsdata, MAX_PAYLOAD_SIZE);
	return true;
}

void COBDI2C::gpsSetup(uint32_t baudrate, const char* cmds)
{
	sendCommand(CMD_GPS_SETUP, baudrate / 1200, (byte*)cmds, cmds ? strlen(cmds) : 0);
}

void COBDI2C::setPID(byte pid)
{
	byte n = 0;
	for (; n < MAX_PIDS && obdPid[n]; n++) {
		if (obdPid[n] == pid)
			return;
	}
	if (n == MAX_PIDS) {
		memmove(obdPid, obdPid + 1, sizeof(obdPid[0]) * (MAX_PIDS - 1));
		n = MAX_PIDS - 1;
	}
	obdPid[n] = pid;
}

void COBDI2C::applyPIDs()
{
	sendCommand(CMD_APPLY_OBD_PIDS, 0, (byte*)obdPid, sizeof(obdPid));
	delay(200);
}

void COBDI2C::loadData()
{
	sendCommand(CMD_LOAD_OBD_DATA);
	dataIdleLoop();
	Wire.requestFrom((byte)m_addr, (byte)MAX_PAYLOAD_SIZE, (byte)0);
	Wire.readBytes((char*)obdInfo, MAX_PAYLOAD_SIZE);
}

uint16_t COBDI2C::getData(byte pid, int& result)
{
    byte n;
    for (n = 0; n < MAX_PIDS && obdPid[n] != pid; n++);
    if (n == MAX_PIDS)
        return -1;

    PID_INFO* pi = obdInfo + n;
	switch (pid) {
	case PID_RPM:
		result = pi->value >> 2;
		break;
	case PID_FUEL_PRESSURE:
		result = (int)pi->value * 3;
		break;
	case PID_COOLANT_TEMP:
	case PID_INTAKE_TEMP:
	case PID_AMBIENT_TEMP:
	case PID_ENGINE_OIL_TEMP:
		result = (int)pi->value - 40;
		break;
	case PID_THROTTLE:
	case PID_ENGINE_LOAD:
	case PID_FUEL_LEVEL:
	case PID_ABSOLUTE_ENGINE_LOAD:
	case PID_ETHANOL_PERCENTAGE:
	case PID_HYBRID_BATTERY_PERCENTAGE:
		result = pi->value * 100 / 255; // %
		break;
	case PID_MAF_FLOW:
		result = pi->value / 100;
		break;
	case PID_TIMING_ADVANCE:
		result = (int)pi->value / 2 - 64;
		break;
	case PID_CONTROL_MODULE_VOLTAGE: // V
		result = pi->value / 1000;
		break;
	case PID_ENGINE_FUEL_RATE: // L/h
		result = pi->value / 20;
		break;
	case PID_ENGINE_TORQUE_PERCENTAGE: // %
		result = (int)pi->value - 125;
		break;
	default:
		result = pi->value;
	}
	return result;
}

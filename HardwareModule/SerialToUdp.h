#pragma once
#include "../CommonModule/Interfaces.h"
#include "../CommonModule/UdpServer.h"
class SerialToUdp :
	public ISerial, public UdpServer
{
private:
	ISerialListener *messageCallback = NULL;
public:
	SerialToUdp(boost::asio::io_service &io, const std::string &host, unsigned short port);
	virtual void SendCommand(int id, const std::string &cmd, int param = INT_MAX);
	virtual void WriteString(const std::string &s);
	virtual void MessageReceived(const std::string & message);
	virtual void SetMessageHandler(ISerialListener* callback) {
		messageCallback = callback;
	}
	virtual void DataReceived(const std::string & message) {};

	~SerialToUdp();
};


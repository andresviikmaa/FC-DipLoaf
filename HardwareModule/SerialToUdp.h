#pragma once
#include "../CommonModule/Interfaces.h"
#include "../CommonModule/UdpServer.h"
#include "../CommonModule/ThreadedClass.h"
#include <mutex>
class SerialToUdp :
	public ISerial, public UdpServer, public ThreadedClass
{
private:
	ISerialListener *messageCallback = NULL;
	std::mutex writeLock;
	std::mutex readLock;
protected:
	boost::asio::io_service &io;
public:
	SerialToUdp(boost::asio::io_service &io, const std::string &host, unsigned short port);
	virtual void SendCommand(int id, const std::string &cmd, int param = INT_MAX);
	virtual void WriteString(const std::string &s);
	virtual bool MessageReceived(const std::string & message);
	virtual void SetMessageHandler(ISerialListener* callback) {
		messageCallback = callback;
	}
	virtual void DataReceived(const std::string & message) {};
	virtual void Run();
	virtual ~SerialToUdp();
};


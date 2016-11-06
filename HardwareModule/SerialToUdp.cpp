#include "SerialToUdp.h"



SerialToUdp::SerialToUdp(boost::asio::io_service &io, const std::string &host, unsigned short port): UdpServer(io, host, port)
{
}
void SerialToUdp::SendCommand(int id, const std::string &cmd, int param) {
	std::ostringstream oss;

	oss << id << ":" << cmd;
	if (param < INT_MAX) oss << param;
	oss << "\n";
	WriteString(oss.str());
}

void SerialToUdp::WriteString(const std::string &s) {
	this->SendMessage(s);
}
void SerialToUdp::MessageReceived(const std::string & message) {
	if (messageCallback != nullptr) {
		messageCallback->DataReceived(message);
	}
}


SerialToUdp::~SerialToUdp()
{
}

#include "SerialToUdp.h"
#include <mutex>



SerialToUdp::SerialToUdp(boost::asio::io_service &io, const std::string &host, unsigned short port): UdpServer(io, host, port)
, io(io), ThreadedClass("SerialToUdp")
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
	std::lock_guard<std::mutex> lock(writeLock);
	this->SendMessage(s);
}
void SerialToUdp::MessageReceived(const std::string & message) {
	if (messageCallback != nullptr) {
		messageCallback->DataReceived(message);
	}
}

void SerialToUdp::Run() {
	while (!stop_thread) {
		io.reset();
		io.run();
	}
}
SerialToUdp::~SerialToUdp()
{
	io.stop();
	WaitForStop();
}

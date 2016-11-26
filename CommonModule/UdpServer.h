#pragma once
//#include "types.h"
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#define BUF_SIZE 4000
class UdpServer
{
public:
	UdpServer(boost::asio::io_service &io, unsigned short port, bool master);
	UdpServer(boost::asio::io_service &io, const std::string &host, unsigned short port);
	UdpServer(boost::asio::io_service &io, const std::string &host, unsigned short port1, unsigned short port2);
	~UdpServer();
protected:
	void start_receive();

	void handle_receive(const boost::system::error_code& error,
		std::size_t /*bytes_transferred*/);
	void start_receive2();

	void handle_receive2(const boost::system::error_code& error,
		std::size_t /*bytes_transferred*/);

	void handle_send(boost::shared_ptr<std::string> /*message*/,
		const boost::system::error_code& /*error*/,
		std::size_t /*bytes_transferred*/);

	virtual bool MessageReceived(const std::string & message) { return false;  };
	virtual bool MessageReceived(const boost::array<char, BUF_SIZE>& buffer, size_t size) { return false; };
	void SendMessage(const std::string &message);
	void SendData(const char * data, size_t size);
private:
	boost::asio::ip::udp::socket recv_socket;
	boost::asio::ip::udp::socket broadcast_socket;
	boost::asio::ip::udp::endpoint broadcast_endpoint;
	boost::asio::ip::udp::endpoint recv_endpoint;
	boost::array<char, BUF_SIZE> recv_buffer_;

};


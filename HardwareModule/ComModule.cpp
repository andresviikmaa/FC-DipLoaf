#include "ComModule.h"

ComModule::ComModule(ISerial *pSerialPort)
{
	m_pWheels = new WheelController(this, 4);
	m_pCoilGun = new CoilBoard(this);
	m_pRefCom = new LLAPReceiver(this);
	pSerialPort->SetMessageHandler(this);

}


ComModule::~ComModule()
{
	delete m_pWheels;
	delete m_pCoilGun;
	delete m_pRefCom;
}


void ComModule::DataReceived(const std::string & message) {

}
void ComModule::SendCommand(int id, const std::string &cmd, int param) {

}

void ComModule::WriteString(const std::string &s) {

}

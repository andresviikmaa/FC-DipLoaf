#pragma once
#include "../CommonModule/Interfaces.h"
class SerialToUdp :
	public ISerial
{
public:
	SerialToUdp();
	~SerialToUdp();
};


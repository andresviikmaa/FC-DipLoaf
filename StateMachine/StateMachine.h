#pragma once
#include "../CommonModule/Types.h"
#include "../CommonModule/ThreadedClass.h"
#include "../CommonModule/Interfaces.h"
#include "../HardwareModule/HardwareInterfaces.h"

#include "DriveInstruction.h"
#include <atomic>

class Idle : public DriveInstruction
{
private:
	boost::posix_time::ptime idleStart;
public:
	Idle() : DriveInstruction("IDLE"){};
	void onEnter(){
		m_pCom->Drive(0,0,0);
	}
	virtual DriveMode step(double dt){ return DRIVEMODE_IDLE; }
};

class Crash : public DriveInstruction
{
private:
	boost::posix_time::ptime idleStart;
public:
	Crash() : DriveInstruction("CRASH"){};
	void onEnter(){
//		m_pCom->Kick(2700);
		m_pCom->Drive(0, 0, 0);
		//Sleep(1000);
	}
	virtual DriveMode step(double dt){ 
		return DRIVEMODE_CRASH;
	}
};

class BorderToClose : public DriveInstruction
{
private:
	boost::posix_time::ptime idleStart;
public:
	BorderToClose() : DriveInstruction("BORDER_TO_CLOSE"){};
	void onEnter(){
		m_pCom->Drive(0, 0, 0);
	}
	virtual DriveMode step(double dt){ 
		return DRIVEMODE_BORDER_TO_CLOSE; 
	}
};

class StateMachine: public IStateMachine
{
public:
	typedef std::map<DriveMode, DriveInstruction*> TDriveModes;
	const TDriveModes driveModes;
	std::atomic_bool testMode;
	DriveMode preCrashState = DRIVEMODE_IDLE;
private:
	TDriveModes::const_iterator curDriveMode;
	ICommunicationModule *m_pComModule;

	std::atomic_bool drive;
	boost::mutex mutex;

	boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	//boost::posix_time::ptime lastUpdate = time - boost::posix_time::seconds(60);
	DriveMode lastDriveMode = DRIVEMODE_IDLE;
	DriveMode driveMode = DRIVEMODE_IDLE;
	DriveMode testDriveMode = DRIVEMODE_IDLE;

protected:
	void Step(double dt);
public:
	StateMachine(ICommunicationModule *pComModule, const TDriveModes &driveModes);
	void setTestMode(DriveMode mode);
	void enableTestMode(bool enable);
	virtual ~StateMachine();
	std::string GetDebugInfo();
	virtual void Enable(bool enable) {};
};


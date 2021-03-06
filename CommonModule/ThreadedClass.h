#pragma once
#include <boost/thread/thread.hpp>
#include <atomic>
#include <chrono>
#include <thread>

class ThreadedClass
{
public:
	ThreadedClass(const std::string &name = ""); 
	virtual void Start();
	void WaitForStop();
	bool HasError() { return stop_thread; };

	virtual ~ThreadedClass();
	virtual void Enable(bool enable) {
		if (enable && !running){
			Start();
		}
		else if (!enable && running) {
			WaitForStop();
		}
	}
	std::atomic_bool running;
	void Sleep(int ms) {
		std::chrono::milliseconds dura(ms);
		std::this_thread::sleep_for(dura);
	}
	std::string name;
protected:
	boost::thread_group threads;
	std::atomic_bool stop_thread;
	virtual void Run2();
	virtual void Run() = 0;

};


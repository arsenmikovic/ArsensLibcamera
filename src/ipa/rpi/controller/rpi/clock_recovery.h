#include <chrono>
#include <list>
using namespace std::chrono_literals;

namespace RPiController {

class ClockRecovery
{
public:	
	std::list<int64_t> errorValues_;
	std::list<int64_t> clientSequence_;
	void updateValues(std::chrono::microseconds delta);
	std::chrono::microseconds trendingError(std::chrono::microseconds LastWallClock, std::chrono::microseconds ClientWallClock, std::chrono::microseconds lastPayloadFrameDuration, unsigned int sequence);
	int64_t modelledWallClock(int64_t WallClock, int64_t KernelWallClock, unsigned int sequence);
	void clear();
	void initialize(int64_t wallClock, int64_t kernel, unsigned int syncperiod, unsigned int listsize);
	void initialize(unsigned int syncperiod, unsigned int listsize);
	
	ClockRecovery(unsigned int syncperiod, unsigned int listsize)
	{
		syncPeriod_ = syncperiod;
		listSize_ = listsize;
	}
	
	ClockRecovery() {}
		
	ClockRecovery(int64_t wallClock, int64_t kernel, unsigned int syncperiod, unsigned int listsize)
	{
		initialized_ = true;
		wallClockBaseline_ = wallClock;
		kernelBaseline_ = kernel;
		syncPeriod_ = syncperiod;
		listSize_ = listsize;
	}
private:
	float slope();
	double xsum_ = 0;
	double ysum_ = 0;
	double xysum_ = 0;
	double x2sum_ = 0;
	int64_t wallClockBaseline_ = 0;
	int64_t kernelBaseline_ = 0;
	unsigned int listSize_ = 100;
	unsigned int syncPeriod_ = 30;
	bool initialized_ = false;
};
} //namespace RPiController
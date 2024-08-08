#include "clock_recovery.h"

#include <cctype>
#include <chrono>
#include <fcntl.h>
#include <list>
#include <map>
#include <strings.h>
#include <unistd.h>
#include <vector>

#include <libcamera/base/log.h>

#include "sync_status.h"

using namespace std::chrono_literals;
using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiClockRec)


float ClockRecovery::slope()
{
	//returns slope for least squares line through points
	double sl = (errorValues_.size() * xysum_ - xsum_ * ysum_) / (errorValues_.size() * x2sum_ - xsum_ * xsum_);
	return sl;
}

void ClockRecovery::updateValues(std::chrono::microseconds correction)
{
	std::list<int64_t>::iterator iterror = errorValues_.begin();
	std::list<int64_t> update;

	for (; iterror != errorValues_.end(); iterror++) {
		update.push_back((*iterror) - correction.count());
 	}
	ysum_ -= (double)correction.count() * update.size();
	xysum_ -= correction.count() * xsum_;
	errorValues_ = update;
}

std::chrono::microseconds ClockRecovery::trendingError(std::chrono::microseconds lastWallClock,
														std::chrono::microseconds clientWallClock,
														std::chrono::microseconds lastPayloadFrameDuration,
														unsigned int sequence)
{
	std::chrono::microseconds delta = (clientWallClock) - (lastWallClock);
	unsigned int mul = (delta + lastPayloadFrameDuration / 2) / lastPayloadFrameDuration;
	std::chrono::microseconds delta_mod = delta - mul * lastPayloadFrameDuration;
	
	int y = delta_mod.count();
	int x = sequence;
	//using rolling sums for x, y, x*y, and x^2
	if (errorValues_.size() == listSize_) {
		xsum_ -= clientSequence_.front() * 1.0;
		ysum_ -= errorValues_.front() * 1.0;
		xysum_ -= errorValues_.front() * clientSequence_.front() * 1.0;
		x2sum_ -= clientSequence_.front() * clientSequence_.front() * 1.0;

		errorValues_.pop_front();
		clientSequence_.pop_front();
	}
	errorValues_.push_back(y);
	clientSequence_.push_back(x);

	xsum_ += clientSequence_.back() * 1.0;
	ysum_ += errorValues_.back() * 1.0;
	xysum_ += errorValues_.back() * clientSequence_.back() * 1.0;
	x2sum_ += clientSequence_.back() * clientSequence_.back() * 1.0;
	LOG(RPiClockRec, Info) << "Current offset and sequence number: "
						   << " NEXT " << y << "," << x;
	int trending = errorValues_.front() + slope() * (errorValues_.size() - 1) * syncPeriod_;
	LOG(RPiClockRec, Info) << trending << " list size "<< errorValues_.size();
	return std::chrono::microseconds(trending);
}

/* This is different from trending error function only becasue we use values that are RealValue - BaseLineValue in order to avoid hugeee numbers */
int64_t ClockRecovery::modelledWallClock(int64_t WallClock, int64_t KernelWallClock, unsigned int sequence)
{
	int64_t y = (WallClock - wallClockBaseline_) - (KernelWallClock - kernelBaseline_) / 1000;
	int64_t x = sequence;

	if (errorValues_.size() == listSize_) {
		xsum_ -= errorValues_.front() * 1.0;
		ysum_ -= errorValues_.front() * 1.0;
		xysum_ -= errorValues_.front() * clientSequence_.front() * 1.0;
		x2sum_ -= clientSequence_.front() * clientSequence_.front() * 1.0;

		errorValues_.pop_front();
		clientSequence_.pop_front();
	}
	errorValues_.push_back(y);
	clientSequence_.push_back(x);

	xsum_ += clientSequence_.back() * 1.0;
	ysum_ += errorValues_.back() * 1.0;
	xysum_ += errorValues_.back() * clientSequence_.back() * 1.0;
	x2sum_ += clientSequence_.back() * clientSequence_.back() * 1.0;

	int64_t trending = errorValues_.front() + slope() * (errorValues_.size() - 1) * syncPeriod_ + (KernelWallClock - kernelBaseline_) / 1000;

	if (errorValues_.size() > 5) {
		return trending + wallClockBaseline_;
	} else {
		return WallClock;
	}

}

void ClockRecovery::clear()
{
	errorValues_.clear();
	clientSequence_.clear();
	xsum_ = 0;
	ysum_ = 0;
	xysum_ = 0;
	x2sum_ = 0;
}

/* Initilizes only once so the baseline values dont change*/
void ClockRecovery::initialize(int64_t wallClock, int64_t kernel, unsigned int syncperiod, unsigned int listsize)
{
	if (!initialized_) {
		initialized_ = true;
		wallClockBaseline_ = wallClock;
		kernelBaseline_ = kernel;
		syncPeriod_ = syncperiod;
		listSize_ = listsize;
	}
}

void ClockRecovery::initialize(unsigned int syncperiod, unsigned int listsize)
{
	if (!initialized_) {
		initialized_ = true;
		syncPeriod_ = syncperiod;
		listSize_ = listsize;
	}
}
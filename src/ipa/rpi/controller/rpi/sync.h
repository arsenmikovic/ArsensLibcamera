/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * sync.h - sync algorithm
 */
#pragma once

#include <arpa/inet.h>
#include <netinet/ip.h>

#include "../sync_algorithm.h"

namespace RPiController {

struct SyncPayload {
	/* Wall clock time for this frame */
	uint64_t wallClock;
	/* Capture sequence number */
	uint64_t sequence;
	/* Wall clock time for the next sync period */
	uint64_t nextWallClock;
	/* Capture sequence number at the next sync period */
	uint64_t nextSequence;
};

class Sync : public SyncAlgorithm
{
public:
	Sync(Controller *controller);
	~Sync();
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void initialise() override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;
	void setFrameDuration(libcamera::utils::Duration frameDuration) override;

private:
	Mode mode_;
	std::string group_;
	uint16_t port_;
	uint32_t syncPeriod_;
	struct sockaddr_in addr_;
	int socket_;
	libcamera::utils::Duration frameDuration_;
	unsigned int frameCount_;
};

} /* namespace RPiController */

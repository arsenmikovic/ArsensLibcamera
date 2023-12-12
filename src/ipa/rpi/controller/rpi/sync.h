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

class Sync : public SyncAlgorithm
{
public:
	Sync(Controller *controller);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void initialise() override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;

private:
	Mode mode_;
	std::string group_;
	uint16_t port_;

	struct sockaddr_in addr_;
	int socket_;
};

} /* namespace RPiController */

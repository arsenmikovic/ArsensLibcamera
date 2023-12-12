/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi Ltd
 *
 * sdn.cpp - SDN (spatial denoise) control algorithm
 */
#include "sync.h"

#include <cctype>
#include <fcntl.h>
#include <map>
#include <strings.h>
#include <unistd.h>
#include <vector>

#include <libcamera/base/log.h>

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiSync)

#define NAME "rpi.sync"

const char *DefaultGroup = "239.255.255.250";
constexpr unsigned int DefaultPort = 32723;

Sync::Sync(Controller *controller)
	: SyncAlgorithm(controller), mode_(Mode::Off), socket_(-1)
{
}

char const *Sync::name() const
{
	return NAME;
}

int Sync::read(const libcamera::YamlObject &params)
{
	static const std::map<std::string, Mode> modeMapping = {
		{ "off", Mode::Off },
		{ "client", Mode::Client },
		{ "server", Mode::Server },
	};

	std::string mode = params["mode"].get<std::string>("off");
	std::transform(mode.begin(), mode.end(), mode.begin(),
		       [](unsigned char c) { return std::tolower(c); });

	auto it = modeMapping.find(mode);
	if (it == modeMapping.end()) {
		LOG(RPiSync, Error) << "Invalid mode specificed: " << mode;
		return -EINVAL;
	}
	mode_ = it->second;

	group_ = params["group"].get<std::string>(DefaultGroup);
	port_ = params["port"].get<uint16_t>(DefaultPort);

	return 0;
}

void Sync::initialise()
{
	socket_ = socket(AF_INET, SOCK_DGRAM, 0);
	if (socket_ < 0) {
		LOG(RPiSync, Error) << "Unable to create server socket.";
		return;
	}

	memset(&addr_, 0, sizeof(addr_));
        addr_.sin_family = AF_INET;
        addr_.sin_addr.s_addr = mode_ == Mode::Client ? htonl(INADDR_ANY) : inet_addr(group_.c_str());
        addr_.sin_port = htons(port_);

	if (mode_ == Mode::Client) {
		/* Set to non-blocking. */
		int flags = fcntl(socket_, F_GETFL, 0);
		fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

		unsigned int en = 1;
		if (setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &en, sizeof(en)) < 0) {
			LOG(RPiSync, Error) << "Unable to set socket options";
			goto err;
		}

                struct ip_mreq mreq {};
		mreq.imr_multiaddr.s_addr = inet_addr(group_.c_str());
		mreq.imr_interface.s_addr = htonl(INADDR_ANY);
		if (setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
			LOG(RPiSync, Error) << "Unable to set socket options";
			goto err;
		}

		if (bind(socket_, (struct sockaddr *) &addr_, sizeof(addr_)) < 0) {
			LOG(RPiSync, Error) << "Unable to bind client socket.";
			goto err;
		}
	}

	return;

err:
	close(socket_);
	socket_ = -1;
}

void Sync::process([[maybe_unused]] StatisticsPtr &stats, [[maybe_unused]] Metadata *imageMetadata)
{
	static int count = 0;
	std::string msg = "Frame: " + std::to_string(count++);

	if (mode_ == Mode::Server) {
		if (sendto(socket_, msg.c_str(), msg.size(), 0, (const sockaddr *)&addr_, sizeof(addr_)) < 0)
			LOG(RPiSync, Error) << "Send error!";
                else
                        LOG(RPiSync, Info) << "Sent message: " << msg;
	} else if (mode_ == Mode::Client) {
		std::vector<char> rx(256);
		socklen_t addrlen = sizeof(addr_);

		if (recvfrom(socket_, rx.data(), rx.size(), 0, (struct sockaddr *) &addr_, &addrlen) < 0)
				LOG(RPiSync, Error) << "Receive error!";
                else
                        	LOG(RPiSync, Info) << "Receive message: " << std::string(rx.begin(), rx.end());
	}
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Sync(controller);
}
static RegisterAlgorithm reg(NAME, &create);

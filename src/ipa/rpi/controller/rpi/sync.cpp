/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi Ltd
 *
 * sdn.cpp - SDN (spatial denoise) control algorithm
 */
#include "sync.h"

#include <cctype>
#include <chrono>
#include <fcntl.h>
#include <map>
#include <strings.h>
#include <unistd.h>
#include <vector>

#include <libcamera/base/log.h>

#include "sync_status.h"

using namespace std::chrono_literals;
using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiSync)

#define NAME "rpi.sync"

const char *DefaultGroup = "239.255.255.250";
constexpr unsigned int DefaultPort = 32723;
constexpr unsigned int DefaultSyncPeriod = 30;
constexpr unsigned int DefaultReadyFrame = 1000;

Sync::Sync(Controller *controller)
	: SyncAlgorithm(controller), mode_(Mode::Off), socket_(-1), frameDuration_(0s), frameCount_(0)
{
}

Sync::~Sync()
{
	if (socket_ >= 0)
		close(socket_);
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
	syncPeriod_ = params["sync_period"].get<uint32_t>(DefaultSyncPeriod);
	readyFrame_ = params["ready_frame"].get<uint32_t>(DefaultReadyFrame);

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

void Sync::switchMode([[maybe_unused]] CameraMode const &cameraMode, [[maybe_unused]] Metadata *metadata)
{
	syncReady_ = false;
	frameCount_ = 0;
	readyCountdown_ = 0;
}

void Sync::process([[maybe_unused]] StatisticsPtr &stats, Metadata *imageMetadata)
{
	SyncPayload payload;
	SyncParams local {};
	SyncStatus status {};
	imageMetadata->get("sync.params", local);

	if (!frameDuration_) {
		LOG(RPiSync, Error) << "Sync frame duration not set!";
		return;
	}

	if (mode_ == Mode::Server) {

		if (!syncReady_ && !(readyFrame_ - frameCount_)) {
			LOG(RPiSync, Info) << "Sync ready at frame " << frameCount_;
			syncReady_ = true;
		}
		else
			syncReady_ = false;

		if (!(frameCount_ % syncPeriod_)) {
			static int lastWallClock = local.wallClock;
			payload.sequence = local.sequence;
			payload.wallClock = local.wallClock;
			payload.nextSequence = local.sequence + syncPeriod_;
			payload.nextWallClock = local.wallClock + frameDuration_.get<std::micro>() * syncPeriod_;
			payload.readyFrame = std::max<int32_t>(0, readyFrame_ - frameCount_);

			int jitter = payload.wallClock - lastWallClock;
			lastWallClock = payload.nextWallClock;

			if (sendto(socket_, &payload, sizeof(payload), 0, (const sockaddr *)&addr_, sizeof(addr_)) < 0)
				LOG(RPiSync, Error) << "Send error!";
			else
				LOG(RPiSync, Info) << "Sent message: seq " << payload.sequence << " ts " << payload.wallClock << " jitter " << jitter << "us"
						<< " : next seq " << payload.nextSequence << " ts " << payload.nextWallClock << " : ready frame " << payload.readyFrame;
		}
	} else if (mode_ == Mode::Client) {

		static int frames = 0;
		socklen_t addrlen = sizeof(addr_);

		static bool seen_packet = false;

		while (true) {
			int64_t lastWallClock = lastPayload_.nextWallClock;
			int ret = recvfrom(socket_, &lastPayload_, sizeof(lastPayload_), 0, (struct sockaddr *)&addr_, &addrlen);

			if (ret > 0) {
				seen_packet =  true;
				int jitter = lastPayload_.wallClock - lastWallClock;
				LOG(RPiSync, Info) << "Receive message: seq " << lastPayload_.sequence << " ts " << lastPayload_.wallClock
						<< " server jitter " << jitter << "us"
						<< " : next seq " << lastPayload_.nextSequence << " ts " << lastPayload_.nextWallClock
						<< " est duration " << (lastPayload_.nextWallClock - lastPayload_.wallClock) * 1us / (lastPayload_.nextSequence - lastPayload_.sequence)
						<< " : readyFrame " << lastPayload_.readyFrame;
					state_ = State::Correcting;
					frames = 0;						

					if (!syncReady_ && !readyCountdown_)
						readyCountdown_ = lastPayload_.readyFrame + lastPayload_.sequence;

				} else
					break;
		}

		std::chrono::microseconds lastPayloadFrameDuration = (lastPayload_.nextWallClock - lastPayload_.wallClock) * 1us / (lastPayload_.nextSequence - lastPayload_.sequence);
		std::chrono::microseconds delta = (local.wallClock * 1us) - ((lastPayload_.wallClock * 1us) + frames * lastPayloadFrameDuration);
		unsigned int mul = (delta + lastPayloadFrameDuration / 2) / lastPayloadFrameDuration;
		std::chrono::microseconds delta_mod = delta - mul * lastPayloadFrameDuration;
		LOG(RPiSync, Info) << "Current frame : seq " << local.sequence << " ts " << local.wallClock << "us"
				   << " frame offset " << frames << " est delta " << delta << " (mod) " << delta_mod << " ready countdown " << readyCountdown_;

		if (state_ == State::Correcting)  {
			status.frameDurationOffset = delta_mod;
			state_ = State::Stabilising;
			LOG(RPiSync, Info) << "Correcting offset " << status.frameDurationOffset;

		} else if (state_ == State::Stabilising) {
			status.frameDurationOffset = 0s;
			LOG(RPiSync, Info) << "Stabilising duration ";		
			state_ = State::Idle;
		}

		if (!syncReady_ && seen_packet) {
			syncReady_ = !readyCountdown_;
			if (syncReady_)
				LOG(RPiSync, Info) << "Sync ready at frame " << frameCount_;
		}				

		frames++;
		if (readyCountdown_)
			readyCountdown_--;
	}

	status.ready = syncReady_;
	imageMetadata->set("sync.status", status);
	frameCount_++;
}

void Sync::setFrameDuration(libcamera::utils::Duration frameDuration)
{
	frameDuration_ = frameDuration;
};

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Sync(controller);
}
static RegisterAlgorithm reg(NAME, &create);

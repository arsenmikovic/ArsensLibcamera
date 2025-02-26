#include "sync.h"

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

using namespace std;
using namespace std::chrono_literals;
using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiSync)

#define NAME "rpi.sync"

const char *DefaultGroup = "239.255.255.250";
constexpr unsigned int DefaultPort = 10000;
constexpr unsigned int DefaultSyncPeriod = 30;
constexpr unsigned int DefaultReadyFrame = 1000;
constexpr unsigned int DefaultLineFitting = 100;

/* Returns IP adress of the devide we are on */
std::string local_address_IP()
{
    const char* google_dns_server = "8.8.8.8";
    int dns_port = 53;

    struct sockaddr_in serv;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);

    //Socket could not be created
    if (sock < 0)
    {
        LOG(RPiSync, Debug) << "Socket error";
    }

    memset(&serv, 0, sizeof(serv));
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = inet_addr(google_dns_server);
    serv.sin_port = htons(dns_port);

    int err = connect(sock, (const struct sockaddr*)&serv, sizeof(serv));
    if (err < 0)
    {
        LOG(RPiSync, Debug) << "Socket error";
    }

    struct sockaddr_in name;
    socklen_t namelen = sizeof(name);
    err = getsockname(sock, (struct sockaddr*)&name, &namelen);

    char buffer[80];
    const char* p = inet_ntop(AF_INET, &name.sin_addr, buffer, 80);
	if (p != NULL){}
    close(sock);
	return buffer;
}



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
/* This reads from json file and intitiaises server and client */
int Sync::read(const libcamera::YamlObject &params)
{
	group_ = params["group"].get<std::string>(DefaultGroup);
	port_ = params["port"].get<uint16_t>(DefaultPort);
	syncPeriod_ = params["sync_period"].get<uint32_t>(DefaultSyncPeriod);
	readyFrame_ = params["ready_frame"].get<uint32_t>(DefaultReadyFrame);
	lineFitting_ = params["line_fitting"].get<uint32_t>(DefaultLineFitting);
	
	return 0;
}


void Sync::initialiseSocket()
{
	socket_ = socket(AF_INET, SOCK_DGRAM, 0);
	if (socket_ < 0) {
		LOG(RPiSync, Error) << "Unable to create server socket.";
		return;
	}
	LOG(RPiSync, Info) << "Setting socket.";
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


/*
 * Camera sync algorithm.
 *     Server - there is a single server that sends framerate timing information over the network to any
 *         clients that are listening. It also signals when it will send a "everything is synchronised, now go"
 *         message back to the algorithm.
 *     Client - there may be many clients, either on the same Pi or different ones. They match their
 *         framerates to the server, and indicate when to "go" at the same instant as the server. 
 */
void Sync::process([[maybe_unused]] StatisticsPtr &stats, Metadata *imageMetadata)
{
	SyncPayload payload;
	SyncParams local{};
	SyncStatus status{};
	imageMetadata->get("sync.params", local);
	if (!frameDuration_) {
		LOG(RPiSync, Error) << "Sync frame duration not set!";
		return;
	}
	if (mode_ == Mode::Off) {
		return;
	}
	/* Initialises sockets when the mdoe_ is set, becasue it needs to know whether it is setting up client or server */
	if (!socketInitialised_) {
		socketInitialised_ = true;
		Sync::initialiseSocket();
		nextSensorTimestamp_ = local.wallClock;
	}

	if (mode_ == Mode::Server) {
		/* 
		This varibale is the modelled wall clock
		We use the local.wall as baseline that swe subtract from every future value to avoid 12 digit numbers 
		(because if we used just wall clock as values in the list, it blows up, so we use WClock-Baselien)
		 */
		trendingClock_.initialize(local.wallClock, local.sensorTimestamp, syncPeriod_, lineFitting_);

		/* Takse care of lost frames so to updates the counters correctly */
		int frameDiff = (local.wallClock - lastWallClock_ - frameDuration_.get<std::micro>() * 0.5) / frameDuration_.get<std::micro>();
		if (frameDiff > 0 && lastWallClock_) {
			frameCount_ += frameDiff;
		}
		lastWallClock_ = local.wallClock;

		if (!syncReady_ && !(readyFrame_ - frameCount_)) {
			if (local.wallClock > syncTime_ - frameDuration_.get<std::micro>() * 0.5 && syncTime_) {
				LOG(RPiSync, Info) << "Sync ready is true";
				syncReady_ = true;
				/* lag tells us how late from expected start we acctually started */
				lag_ = local.wallClock - syncTime_;
				if (local.wallClock > syncTime_ + frameDuration_.get<std::micro>() * 0.5) {
					LOG(RPiSync, Warning) << "Frame has been lost, sync started with lag of: " << lag_ << " us";
				} else {
					LOG(RPiSync, Info) << "Sync started without lag";
				}
			}
		} else if (!syncReady_) {
			syncTime_ = local.wallClock + frameDuration_.get<std::micro>() * (readyFrame_ - frameCount_);
		}

		if (!(frameCount_ % syncPeriod_)) {
			/* Preparing and sending packet */
			payload.sequence = local.sequence;
			payload.wallClock = trendingClock_.modelledWallClock(local.wallClock, local.sensorTimestamp, local.sequence);
			payload.sensorTimestamp = local.sensorTimestamp;
			payload.nextSequence = local.sequence + syncPeriod_;
			payload.nextWallClock = payload.wallClock + frameDuration_.get<std::micro>() * syncPeriod_;
			payload.readyFrame = std::max<int32_t>(0, readyFrame_ - frameCount_);
			int64_t jitter = payload.wallClock - nextSensorTimestamp_;
			nextSensorTimestamp_ = payload.nextWallClock;

			if (sendto(socket_, &payload, sizeof(payload), 0, (const sockaddr *)&addr_, sizeof(addr_)) < 0) {
				LOG(RPiSync, Error) << "Send error! " << strerror(errno);
			} else {
				LOG(RPiSync, Info) << "Sent message: seq " << payload.sequence << " jitter " << jitter << "us" 
									<< " : ready frame " << payload.readyFrame;
			}
		}
	} else if (mode_ == Mode::Client) {
		/* 
		Initialising trending error whic follows the modelled error after sync 1000 frames
		Trensing clock as before modells the wall clock to remove jitter
		We use the local.wall clcok in ordeer to use smaller numbers and not 12 digit ons in the modelling, we subtract this constant initial wall clcok from all values
		*/
		trendingError_.initialize(syncPeriod_, lineFitting_);
		trendingClock_.initialize(local.wallClock, local.sensorTimestamp, syncPeriod_, lineFitting_);
		socklen_t addrlen = sizeof(addr_);

		while (true) {
			int ret = recvfrom(socket_, &lastPayload_, sizeof(lastPayload_), 0, (struct sockaddr *)&addr_, &addrlen);

			if (ret > 0) {
				/* 
				Check the ip adress of server and client and tells us whether we are working on same or different pi.
				This ony happens once, IP server is extracted from packet, and IP client using function local_address_IP
				*/
				if (!IPCheck_) {
					char srcIP[INET_ADDRSTRLEN];
					inet_ntop(AF_INET, &(addr_.sin_addr), srcIP, INET_ADDRSTRLEN);
					std::string serverIP = srcIP;

					std::string clientIP = local_address_IP();

					if (serverIP == clientIP) {
						usingWallClock_ = false;
						LOG(RPiSync, Info) << "Using server time stamp ";
					} else {
						LOG(RPiSync, Info) << "Using modelled wall clock ";
					}
					LOG(RPiSync, Info) <<"Sever ip: " << serverIP << " client ip: " <<clientIP; 
					IPCheck_ = true;
				}

				if (!syncReady_) {
					state_ = State::Correcting;
				}

				frames_ = 0;	

				/* If we use one pi, we use server Timestamp. If two pi's then modelled wall clock*/
				if (usingWallClock_) {
					modelledWallClockValue_ = trendingClock_.modelledWallClock(local.wallClock, local.sensorTimestamp, local.sequence);
					lastWallClockValue_ = lastPayload_.wallClock;
				} else {
					modelledWallClockValue_ = (local.sensorTimestamp) / 1000;
					lastWallClockValue_ = (lastPayload_.sensorTimestamp) / 1000;
				}

				lastPayloadFrameDuration_ = (lastPayload_.nextWallClock - lastPayload_.wallClock) * 1us / (lastPayload_.nextSequence - lastPayload_.sequence);
				std::chrono::microseconds delta = (modelledWallClockValue_ * 1us) - (lastWallClockValue_ * 1us);
				unsigned int mul = (delta + lastPayloadFrameDuration_ / 2) / lastPayloadFrameDuration_;
				
				delta_mod_ = delta - mul * lastPayloadFrameDuration_;				

				if (!syncReady_)
					readyCountdown_ = lastPayload_.readyFrame + frameCount_;
				
				if (!syncReady_ && lastPayload_.readyFrame) {
					expected_ = lastPayload_.wallClock * 1us + lastPayload_.readyFrame * lastPayloadFrameDuration_;
				}
			} else {
				
				break;
			}
		}
		if (syncReady_ && !frames_) {
			/* Modelled error, trending Error creates new value to add to list and updates slope to calculate this trending value*/
			delta_mod_ = trendingError_.trendingError(lastWallClockValue_ * 1us, modelledWallClockValue_ * 1us, lastPayloadFrameDuration_, local.sequence);
			if (abs(delta_mod_) > 50us) {
				trendingError_.updateValues(delta_mod_);
				state_ = State::Correcting;
			}
		}

		if (state_ == State::Correcting) {
			LOG(RPiSync, Info) << "Correcting by " << delta_mod_;

			status.frameDurationOffset = delta_mod_;
			state_ = State::Stabilising;
		} else if (state_ == State::Stabilising) {

			status.frameDurationOffset = 0s;		
			state_ = State::Idle;
		}

		/* We now say sync ready when the wall clock is in the range of he expected sync wall clock */
		if (!syncReady_ && local.wallClock * 1us > expected_ - lastPayloadFrameDuration_ / 2 && expected_ != 0us) {
			syncReady_ = true;
			LOG(RPiSync, Error) << "Sync ready is true";
			if (usingWallClock_) {
				LOG(RPiSync, Debug) << "Using trending wall clock";
				LOG(RPiSync, Debug) << "Wall clock is " << local.wallClock;
			}
			trendingClock_.clear();
			lag_ = local.wallClock - expected_.count();
			if (local.wallClock * 1us > expected_ + lastPayloadFrameDuration_ / 2) {
				LOG(RPiSync, Warning) << "Frame has been lost, sync started with lag of: " << lag_ << " us";
			} else {
				LOG(RPiSync, Info) << "Sync started without lag";
			}
		}
		frames_++;
	}
	status.syncLag = lag_;
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
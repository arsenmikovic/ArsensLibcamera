/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi Ltd
 *
 * device_status.h - device (image sensor) status
 */
#pragma once

#include <chrono>
#include <iostream>
#include <optional>

#include <libcamera/base/utils.h>

/*
 * Definition of "device metadata" which stores things like shutter time and
 * analogue gain that downstream control algorithms will want to know.
 */

struct DeviceStatus {
		/* Wall clock time for this frame */
	std::chrono::microseconds wallClock;
	/* Capture sequence number */
        uint64_t sequence;
};

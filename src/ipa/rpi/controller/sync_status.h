/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * sync_status.h - Sync algorithm params and status structures
 */
#pragma once

#include <libcamera/base/utils.h>

struct SyncParams {
	/* Wall clock time for this frame */
	uint64_t wallClock;
	/* Capture sequence number */
	uint64_t sequence;
};

struct SyncStatus {
	libcamera::utils::Duration frameDurationOffset;
	bool ready;
};


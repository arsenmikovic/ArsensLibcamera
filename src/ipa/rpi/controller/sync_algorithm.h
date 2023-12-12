/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * sync_algorithm.h - Camera sync algorithm interface
 */
#pragma once

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace RPiController {

class SyncAlgorithm : public Algorithm
{
public:
	enum class Mode {
		Off,
		Server,
		Client,
	};

	SyncAlgorithm(Controller *controller)
		: Algorithm(controller) {}
};

} /* namespace RPiController */

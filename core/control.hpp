/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd. / Dustin Kerstein
 *
 * control.hpp
 */

#pragma once

class Control
{
public:
	Control() = default;
	static bool enableBuffer;
	static int frames;
};

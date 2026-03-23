// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

// Bridge header: Vec4 is now Lane4 (SIMD utility type).
// The Vec4 name will be reclaimed for the 4D spatial vector type.
// For now, Vec4 = Lane4 for backward compatibility.

#include <Jolt/Math/Lane4.h>

JPH_NAMESPACE_BEGIN

using Vec4 = Lane4;
using Vec4Arg = Lane4Arg;

JPH_NAMESPACE_END

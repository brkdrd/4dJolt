// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Math/DVec4.h>

JPH_NAMESPACE_BEGIN

#ifdef JPH_DOUBLE_PRECISION

// Define real to double
using Real = double;
using Real3 = Double3;
using RVec3 = DVec4;
using RVec3Arg = DVec4Arg;

#define JPH_RVECTOR_ALIGNMENT JPH_DVECTOR_ALIGNMENT

// 4D precision-dependent spatial vector (preparation for physics phase)
using RVec4 = DVec4;
using RVec4Arg = DVec4Arg;

#else

// Define real to float
using Real = float;
using Real3 = Float3;
using RVec3 = Vec3;
using RVec3Arg = Vec3Arg;

#define JPH_RVECTOR_ALIGNMENT JPH_VECTOR_ALIGNMENT

// 4D precision-dependent spatial vector (preparation for physics phase)
using RVec4 = Vec4;
using RVec4Arg = Vec4Arg;

#endif // JPH_DOUBLE_PRECISION

// Put the 'real' operator in a namespace so that users can opt in to use it:
// using namespace JPH::literals;
namespace literals {
	constexpr Real operator ""_r (long double inValue) { return Real(inValue); }
};

JPH_NAMESPACE_END

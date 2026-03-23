// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

JPH_NAMESPACE_BEGIN

class Vec3;
class DVec3;
class Lane4;
using Vec4 = Lane4; // Bridge: Vec4 is now Lane4 (SIMD utility)
class UVec4;
class BVec16;
class Quat;
class Mat44;
class DMat44;

// Types to use for passing arguments to functions
using Vec3Arg = const Vec3;
#ifdef JPH_USE_AVX
	using DVec3Arg = const DVec3;
#else
	using DVec3Arg = const DVec3 &;
#endif
using Lane4Arg = const Lane4;
using Vec4Arg = Lane4Arg;
using UVec4Arg = const UVec4;
using BVec16Arg = const BVec16;
using QuatArg = const Quat;
using Mat44Arg = const Mat44 &;
using DMat44Arg = const DMat44 &;

JPH_NAMESPACE_END

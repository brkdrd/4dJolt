// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Core/HashCombine.h>

JPH_NAMESPACE_BEGIN

/// Class that holds 4 doubles. Used as a storage class. Convert to DVec4 for calculations.
class [[nodiscard]] Double4
{
public:
	JPH_OVERRIDE_NEW_DELETE

				Double4() = default; ///< Intentionally not initialized for performance reasons
				Double4(const Double4 &inRHS) = default;
	Double4 &	operator = (const Double4 &inRHS) = default;
				Double4(double inX, double inY, double inZ, double inW) : x(inX), y(inY), z(inZ), w(inW) { }

	double		operator [] (int inCoordinate) const
	{
		JPH_ASSERT(inCoordinate < 4);
		return *(&x + inCoordinate);
	}

	bool		operator == (const Double4 &inRHS) const
	{
		return x == inRHS.x && y == inRHS.y && z == inRHS.z && w == inRHS.w;
	}

	bool		operator != (const Double4 &inRHS) const
	{
		return x != inRHS.x || y != inRHS.y || z != inRHS.z || w != inRHS.w;
	}

	double		x;
	double		y;
	double		z;
	double		w;
};

static_assert(std::is_trivial<Double4>(), "Is supposed to be a trivial type!");

JPH_NAMESPACE_END

// Create a std::hash/JPH::Hash for Double4
JPH_MAKE_HASHABLE(JPH::Double4, t.x, t.y, t.z, t.w)

// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

JPH_NAMESPACE_BEGIN

/// Class that holds 8 float values. Convert to Vec8 to perform calculations.
class [[nodiscard]] Float8
{
public:
	JPH_OVERRIDE_NEW_DELETE

				Float8() = default; ///< Intentionally not initialized for performance reasons
				Float8(const Float8 &inRHS) = default;
				Float8(float inX, float inY, float inZ, float inW, float inE, float inF, float inG, float inH) : mValue { inX, inY, inZ, inW, inE, inF, inG, inH } { }
	Float8 &	operator = (const Float8 &inRHS) = default;

	float		operator [] (int inCoordinate) const
	{
		JPH_ASSERT(inCoordinate < 8);
		return mValue[inCoordinate];
	}

	bool		operator == (const Float8 &inRHS) const
	{
		for (int i = 0; i < 8; i++)
			if (mValue[i] != inRHS.mValue[i])
				return false;
		return true;
	}

	bool		operator != (const Float8 &inRHS) const
	{
		return !(*this == inRHS);
	}

	float		mValue[8];
};

static_assert(std::is_trivial<Float8>(), "Is supposed to be a trivial type!");

JPH_NAMESPACE_END

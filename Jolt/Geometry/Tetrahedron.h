// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

JPH_NAMESPACE_BEGIN

/// A simple tetrahedron (3-simplex) and its material. Used as the surface element for 4D solids.
class Tetrahedron
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Constructor
					Tetrahedron() = default;
					Tetrahedron(const Float4 &inV1, const Float4 &inV2, const Float4 &inV3, const Float4 &inV4, uint32 inMaterialIndex = 0, uint32 inUserData = 0) : mV { inV1, inV2, inV3, inV4 }, mMaterialIndex(inMaterialIndex), mUserData(inUserData) { }
					Tetrahedron(Vec4Arg inV1, Vec4Arg inV2, Vec4Arg inV3, Vec4Arg inV4, uint32 inMaterialIndex = 0, uint32 inUserData = 0) : mMaterialIndex(inMaterialIndex), mUserData(inUserData) { inV1.StoreFloat4(&mV[0]); inV2.StoreFloat4(&mV[1]); inV3.StoreFloat4(&mV[2]); inV4.StoreFloat4(&mV[3]); }

	/// Get center of tetrahedron
	Vec4			GetCentroid() const
	{
		return (Vec4(mV[0]) + Vec4(mV[1]) + Vec4(mV[2]) + Vec4(mV[3])) * 0.25f;
	}

	/// Vertices
	Float4			mV[4];
	uint32			mMaterialIndex = 0;
	uint32			mUserData = 0;				///< User data that can be used for anything by the application, e.g. for tracking the original index of the tetrahedron
};

using TetrahedronList = Array<Tetrahedron>;

JPH_NAMESPACE_END

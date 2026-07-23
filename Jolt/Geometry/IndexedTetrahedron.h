// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Core/HashCombine.h>

JPH_NAMESPACE_BEGIN

/// Tetrahedron with 32-bit indices (3-simplex, used as surface element for 4D solids)
class IndexedTetrahedronNoMaterial
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Constructor
					IndexedTetrahedronNoMaterial() = default;
	constexpr		IndexedTetrahedronNoMaterial(uint32 inI1, uint32 inI2, uint32 inI3, uint32 inI4) : mIdx { inI1, inI2, inI3, inI4 } { }

	/// Check if two tetrahedra are identical
	bool			operator == (const IndexedTetrahedronNoMaterial &inRHS) const
	{
		return mIdx[0] == inRHS.mIdx[0] && mIdx[1] == inRHS.mIdx[1] && mIdx[2] == inRHS.mIdx[2] && mIdx[3] == inRHS.mIdx[3];
	}

	/// Check if two tetrahedra are equivalent (using the same vertices in an even permutation)
	bool			IsEquivalent(const IndexedTetrahedronNoMaterial &inRHS) const
	{
		// The 12 even permutations of (0,1,2,3)
		return (mIdx[0] == inRHS.mIdx[0] && mIdx[1] == inRHS.mIdx[1] && mIdx[2] == inRHS.mIdx[2] && mIdx[3] == inRHS.mIdx[3])
			|| (mIdx[0] == inRHS.mIdx[0] && mIdx[1] == inRHS.mIdx[2] && mIdx[2] == inRHS.mIdx[3] && mIdx[3] == inRHS.mIdx[1])
			|| (mIdx[0] == inRHS.mIdx[0] && mIdx[1] == inRHS.mIdx[3] && mIdx[2] == inRHS.mIdx[1] && mIdx[3] == inRHS.mIdx[2])
			|| (mIdx[0] == inRHS.mIdx[1] && mIdx[1] == inRHS.mIdx[0] && mIdx[2] == inRHS.mIdx[3] && mIdx[3] == inRHS.mIdx[2])
			|| (mIdx[0] == inRHS.mIdx[1] && mIdx[1] == inRHS.mIdx[2] && mIdx[2] == inRHS.mIdx[0] && mIdx[3] == inRHS.mIdx[3])
			|| (mIdx[0] == inRHS.mIdx[1] && mIdx[1] == inRHS.mIdx[3] && mIdx[2] == inRHS.mIdx[2] && mIdx[3] == inRHS.mIdx[0])
			|| (mIdx[0] == inRHS.mIdx[2] && mIdx[1] == inRHS.mIdx[0] && mIdx[2] == inRHS.mIdx[1] && mIdx[3] == inRHS.mIdx[3])
			|| (mIdx[0] == inRHS.mIdx[2] && mIdx[1] == inRHS.mIdx[1] && mIdx[2] == inRHS.mIdx[3] && mIdx[3] == inRHS.mIdx[0])
			|| (mIdx[0] == inRHS.mIdx[2] && mIdx[1] == inRHS.mIdx[3] && mIdx[2] == inRHS.mIdx[0] && mIdx[3] == inRHS.mIdx[1])
			|| (mIdx[0] == inRHS.mIdx[3] && mIdx[1] == inRHS.mIdx[0] && mIdx[2] == inRHS.mIdx[2] && mIdx[3] == inRHS.mIdx[1])
			|| (mIdx[0] == inRHS.mIdx[3] && mIdx[1] == inRHS.mIdx[1] && mIdx[2] == inRHS.mIdx[0] && mIdx[3] == inRHS.mIdx[2])
			|| (mIdx[0] == inRHS.mIdx[3] && mIdx[1] == inRHS.mIdx[2] && mIdx[2] == inRHS.mIdx[1] && mIdx[3] == inRHS.mIdx[0]);
	}

	/// Check if tetrahedron is degenerate (zero 3-volume, dimension-agnostic via 3x3 Gram determinant)
	bool			IsDegenerate(const VertexList &inVertices) const
	{
		Vec4 v0(inVertices[mIdx[0]]);
		Vec4 v1(inVertices[mIdx[1]]);
		Vec4 v2(inVertices[mIdx[2]]);
		Vec4 v3(inVertices[mIdx[3]]);

		Vec4 e0 = v1 - v0;
		Vec4 e1 = v2 - v0;
		Vec4 e2 = v3 - v0;

		// 3x3 Gram determinant: det(G) = det([[e0.e0, e0.e1, e0.e2], [e1.e0, e1.e1, e1.e2], [e2.e0, e2.e1, e2.e2]])
		float d00 = e0.Dot(e0), d01 = e0.Dot(e1), d02 = e0.Dot(e2);
		float d11 = e1.Dot(e1), d12 = e1.Dot(e2);
		float d22 = e2.Dot(e2);
		float det = d00 * (d11 * d22 - d12 * d12) - d01 * (d01 * d22 - d12 * d02) + d02 * (d01 * d12 - d11 * d02);
		return det < 1.0e-12f;
	}

	/// Get center of tetrahedron
	Vec4			GetCentroid(const VertexList &inVertices) const
	{
		return (Vec4(inVertices[mIdx[0]]) + Vec4(inVertices[mIdx[1]]) + Vec4(inVertices[mIdx[2]]) + Vec4(inVertices[mIdx[3]])) * 0.25f;
	}

	/// Get the hash value of this structure
	uint64			GetHash() const
	{
		static_assert(sizeof(IndexedTetrahedronNoMaterial) == 4 * sizeof(uint32), "Class should have no padding");
		return HashBytes(this, sizeof(IndexedTetrahedronNoMaterial));
	}

	uint32			mIdx[4];
};

/// Tetrahedron with 32-bit indices and material index
class IndexedTetrahedron : public IndexedTetrahedronNoMaterial
{
public:
	using IndexedTetrahedronNoMaterial::IndexedTetrahedronNoMaterial;

	/// Constructor
	constexpr		IndexedTetrahedron(uint32 inI1, uint32 inI2, uint32 inI3, uint32 inI4, uint32 inMaterialIndex, uint inUserData = 0) : IndexedTetrahedronNoMaterial(inI1, inI2, inI3, inI4), mMaterialIndex(inMaterialIndex), mUserData(inUserData) { }

	/// Check if two tetrahedra are identical
	bool			operator == (const IndexedTetrahedron &inRHS) const
	{
		return mMaterialIndex == inRHS.mMaterialIndex && mUserData == inRHS.mUserData && IndexedTetrahedronNoMaterial::operator==(inRHS);
	}

	/// Get the hash value of this structure
	uint64			GetHash() const
	{
		static_assert(sizeof(IndexedTetrahedron) == 6 * sizeof(uint32), "Class should have no padding");
		return HashBytes(this, sizeof(IndexedTetrahedron));
	}

	uint32			mMaterialIndex = 0;
	uint32			mUserData = 0;				///< User data that can be used for anything by the application, e.g. for tracking the original index of the tetrahedron
};

using IndexedTetrahedronNoMaterialList = Array<IndexedTetrahedronNoMaterial>;
using IndexedTetrahedronList = Array<IndexedTetrahedron>;

JPH_NAMESPACE_END

// Create a std::hash for IndexedTetrahedronNoMaterial and IndexedTetrahedron
JPH_MAKE_STD_HASH(JPH::IndexedTetrahedronNoMaterial)
JPH_MAKE_STD_HASH(JPH::IndexedTetrahedron)

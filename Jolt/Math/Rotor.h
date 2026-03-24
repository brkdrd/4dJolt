// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Math/Vec4.h>
#include <Jolt/Math/Vec8.h>

JPH_NAMESPACE_BEGIN

/// Rotor class for 4D rotations using Clifford algebra Cl(4,0).
///
/// A rotor is an element of the even subalgebra with 8 components:
///   R = s + B12*e12 + B13*e13 + B14*e14 + B23*e23 + B24*e24 + B34*e34 + P*e1234
///
/// Component layout in Vec8: [s, e12, e13, e14, e23, e24, e34, e1234]
///   - s     = scalar part (grade 0)
///   - e_ij  = bivector components (grade 2), representing rotation planes
///   - e1234 = pseudoscalar (grade 4)
///
/// For a unit rotor (|R| = 1), the sandwich product R*v*~R rotates a vector v in 4D.
/// Simple rotations (in a single plane) have P = 0 and behave like quaternions.
/// Double rotations (isoclinic) have nonzero pseudoscalar and rotate in two planes simultaneously.
class [[nodiscard]] alignas(JPH_VECTOR_ALIGNMENT) Rotor
{
public:
	JPH_OVERRIDE_NEW_DELETE

	///@name Constructors
	///@{
	inline						Rotor() = default; ///< Intentionally not initialized for performance reasons
								Rotor(const Rotor &inRHS) = default;
	Rotor &						operator = (const Rotor &inRHS) = default;

	/// Construct from 8 explicit components [s, e12, e13, e14, e23, e24, e34, e1234]
	inline						Rotor(float inS, float inE12, float inE13, float inE14,
									  float inE23, float inE24, float inE34, float inE1234)
									: mValue(inS, inE12, inE13, inE14, inE23, inE24, inE34, inE1234) { }

	/// Construct from Vec8
	inline explicit				Rotor(Vec8Arg inV) : mValue(inV) { }

	/// Construct from Float8
	inline explicit				Rotor(const Float8 &inV) : mValue(inV) { }
	///@}

	///@name Tests
	///@{

	/// Check if two rotors are exactly equal
	inline bool					operator == (RotorArg inRHS) const								{ return mValue == inRHS.mValue; }

	/// Check if two rotors are different
	inline bool					operator != (RotorArg inRHS) const								{ return mValue != inRHS.mValue; }

	/// If this rotor is close to inRHS
	inline bool					IsClose(RotorArg inRHS, float inMaxDistSq = 1.0e-12f) const		{ return mValue.IsClose(inRHS.mValue, inMaxDistSq); }

	/// If the length of this rotor is 1 +/- inTolerance
	inline bool					IsNormalized(float inTolerance = 1.0e-5f) const					{ return mValue.IsNormalized(inTolerance); }

	/// If any component of this rotor is a NaN
	inline bool					IsNaN() const													{ return mValue.IsNaN(); }

	///@}
	///@name Get components
	///@{

	/// Get scalar part (grade 0)
	JPH_INLINE float			GetScalar() const												{ return mValue[0]; }

	/// Get bivector components (grade 2)
	JPH_INLINE float			GetE12() const													{ return mValue[1]; }
	JPH_INLINE float			GetE13() const													{ return mValue[2]; }
	JPH_INLINE float			GetE14() const													{ return mValue[3]; }
	JPH_INLINE float			GetE23() const													{ return mValue[4]; }
	JPH_INLINE float			GetE24() const													{ return mValue[5]; }
	JPH_INLINE float			GetE34() const													{ return mValue[6]; }

	/// Get pseudoscalar part (grade 4)
	JPH_INLINE float			GetPseudoscalar() const											{ return mValue[7]; }

	/// Get the underlying Vec8
	JPH_INLINE Vec8				GetVec8() const													{ return mValue; }

	///@}
	///@name Default rotors
	///@{

	/// Identity rotor (no rotation): [1, 0, 0, 0, 0, 0, 0, 0]
	JPH_INLINE static Rotor		sIdentity()														{ return Rotor(1, 0, 0, 0, 0, 0, 0, 0); }

	/// Zero rotor
	JPH_INLINE static Rotor		sZero()															{ return Rotor(Vec8::sZero()); }

	///@}

	/// Create a simple rotation rotor from two orthonormal vectors defining the rotation plane
	/// and an angle in radians. R = cos(angle/2) + sin(angle/2) * (inPlaneA ^ inPlaneB)
	/// @param inPlaneA First unit vector in the rotation plane
	/// @param inPlaneB Second unit vector in the rotation plane (must be orthogonal to inPlaneA)
	/// @param inAngle Rotation angle in radians
	static JPH_INLINE Rotor		sRotation(Vec4Arg inPlaneA, Vec4Arg inPlaneB, float inAngle);

	/// Create a double rotation (isoclinic) from two planes and two angles.
	/// This rotates simultaneously in two orthogonal planes.
	/// @param inPlaneA1, inPlaneB1 First rotation plane (two orthonormal vectors)
	/// @param inAngle1 First rotation angle
	/// @param inPlaneA2, inPlaneB2 Second rotation plane (two orthonormal vectors, orthogonal to first plane)
	/// @param inAngle2 Second rotation angle
	static JPH_INLINE Rotor		sDoubleRotation(Vec4Arg inPlaneA1, Vec4Arg inPlaneB1, float inAngle1,
												Vec4Arg inPlaneA2, Vec4Arg inPlaneB2, float inAngle2);

	///@name Length / normalization
	///@{

	/// Squared length of rotor
	JPH_INLINE float			LengthSq() const												{ return mValue.LengthSq(); }

	/// Length of rotor
	JPH_INLINE float			Length() const													{ return mValue.Length(); }

	/// Normalize the rotor
	JPH_INLINE Rotor			Normalized() const												{ return Rotor(mValue.Normalized()); }

	///@}
	///@name Arithmetic
	///@{

	/// Negate
	JPH_INLINE Rotor			operator - () const												{ return Rotor(-mValue); }

	/// Add
	JPH_INLINE Rotor			operator + (RotorArg inRHS) const								{ return Rotor(mValue + inRHS.mValue); }

	/// Subtract
	JPH_INLINE Rotor			operator - (RotorArg inRHS) const								{ return Rotor(mValue - inRHS.mValue); }

	/// Multiply by scalar
	JPH_INLINE Rotor			operator * (float inValue) const								{ return Rotor(mValue * inValue); }
	friend JPH_INLINE Rotor		operator * (float inValue, RotorArg inRHS)						{ return Rotor(inRHS.mValue * inValue); }

	/// Divide by scalar
	JPH_INLINE Rotor			operator / (float inValue) const								{ return Rotor(mValue * (1.0f / inValue)); }

	/// Geometric product (rotor composition): this * inRHS
	JPH_INLINE Rotor			operator * (RotorArg inRHS) const;

	///@}

	/// Rotate a 4D vector using the sandwich product: R * v * ~R
	JPH_INLINE Vec4				operator * (Vec4Arg inValue) const;

	/// Dot product between two rotors (useful for SLERP)
	JPH_INLINE float			Dot(RotorArg inRHS) const										{ return mValue.Dot(inRHS.mValue); }

	/// Reverse (conjugate) of the rotor: negate the bivector parts, keep scalar and pseudoscalar.
	/// For a unit rotor, ~R is the inverse rotation.
	JPH_INLINE Rotor			Reversed() const;

	/// Get inverse rotor: ~R / |R|^2
	JPH_INLINE Rotor			Inversed() const;

	/// Linear interpolation between two rotors
	JPH_INLINE Rotor			LERP(RotorArg inDestination, float inFraction) const;

	/// Spherical linear interpolation between two rotors
	JPH_INLINE Rotor			SLERP(RotorArg inDestination, float inFraction) const;

	/// Convert this rotor to a 4x4 rotation matrix
	JPH_INLINE Mat44			ToRotationMatrix() const;

	/// Store as Float8
	JPH_INLINE void				StoreFloat8(Float8 *outV) const									{ mValue.StoreFloat8(outV); }

	/// To String
	friend ostream &			operator << (ostream &inStream, RotorArg inR)
	{
		inStream << inR.mValue;
		return inStream;
	}

	/// 8 component storage: [s, e12, e13, e14, e23, e24, e34, e1234]
	Vec8						mValue;
};

static_assert(std::is_trivial<Rotor>(), "Is supposed to be a trivial type!");

JPH_NAMESPACE_END

#include "Rotor.inl"

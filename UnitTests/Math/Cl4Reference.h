// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Math/Rotor.h>
#include <Jolt/Math/Bivec.h>

/// Brute force reference implementation of the full Clifford algebra Cl(4,0).
///
/// Used by RotorTests / BivecTests to validate the hand derived formulas in Rotor and Bivec
/// against an implementation whose correctness follows directly from the algebra axioms.
///
/// A multivector stores 16 blade coefficients indexed by a 4 bit mask: bit i set means basis
/// vector e_{i+1} is part of the blade. So mask 0 = scalar, 0b0011 = e12, 0b1111 = e1234.
/// The metric is Euclidean (e_i^2 = +1). All arithmetic is done in double precision so the
/// reference is strictly more accurate than the float code under test.
namespace Cl4Ref {

/// Blade masks for the Rotor component order [s, e12, e13, e14, e23, e24, e34, e1234]
static constexpr JPH::uint cRotorMask[8] = { 0b0000, 0b0011, 0b0101, 0b1001, 0b0110, 0b1010, 0b1100, 0b1111 };

/// Blade masks for the Bivec component order [e12, e13, e14, e23, e24, e34]
static constexpr JPH::uint cBivecMask[6] = { 0b0011, 0b0101, 0b1001, 0b0110, 0b1010, 0b1100 };

/// Blade masks for vector components [e1, e2, e3, e4]
static constexpr JPH::uint cVecMask[4] = { 0b0001, 0b0010, 0b0100, 0b1000 };

/// Full multivector of Cl(4,0)
struct MV
{
	double				mC[16] = { };

	MV					operator + (const MV &inRHS) const
	{
		MV result;
		for (int i = 0; i < 16; ++i)
			result.mC[i] = mC[i] + inRHS.mC[i];
		return result;
	}

	MV					operator - (const MV &inRHS) const
	{
		MV result;
		for (int i = 0; i < 16; ++i)
			result.mC[i] = mC[i] - inRHS.mC[i];
		return result;
	}

	MV					operator * (double inRHS) const
	{
		MV result;
		for (int i = 0; i < 16; ++i)
			result.mC[i] = mC[i] * inRHS;
		return result;
	}

	double				MaxAbs() const
	{
		double result = 0.0;
		for (double c : mC)
			result = std::max(result, std::abs(c));
		return result;
	}

	double				MaxAbsDiff(const MV &inRHS) const
	{
		double result = 0.0;
		for (int i = 0; i < 16; ++i)
			result = std::max(result, std::abs(mC[i] - inRHS.mC[i]));
		return result;
	}
};

/// Sign needed to reorder the concatenation of two canonically ordered blades into canonical
/// (ascending index) order. Standard bit counting algorithm; valid for a Euclidean metric.
inline int ReorderSign(JPH::uint inA, JPH::uint inB)
{
	int sum = 0;
	inA >>= 1;
	while (inA != 0)
	{
		sum += int(JPH::CountBits(inA & inB));
		inA >>= 1;
	}
	return (sum & 1) != 0? -1 : 1;
}

/// Geometric product
inline MV Mul(const MV &inA, const MV &inB)
{
	MV result;
	for (JPH::uint a = 0; a < 16; ++a)
		if (inA.mC[a] != 0.0)
			for (JPH::uint b = 0; b < 16; ++b)
				if (inB.mC[b] != 0.0)
					result.mC[a ^ b] += ReorderSign(a, b) * inA.mC[a] * inB.mC[b];
	return result;
}

/// Reverse: a k-blade is negated when k (k - 1) / 2 is odd
inline MV Reverse(const MV &inA)
{
	MV result;
	for (JPH::uint m = 0; m < 16; ++m)
	{
		JPH::uint grade = JPH::CountBits(m);
		result.mC[m] = ((grade * (grade - 1) / 2) & 1) != 0? -inA.mC[m] : inA.mC[m];
	}
	return result;
}

/// Keep only blades of grade inGrade
inline MV GradePart(const MV &inA, JPH::uint inGrade)
{
	MV result;
	for (JPH::uint m = 0; m < 16; ++m)
		if (JPH::CountBits(m) == inGrade)
			result.mC[m] = inA.mC[m];
	return result;
}

/// Exponential via power series (converges for any bivector with the angles used in tests)
inline MV Exp(const MV &inB)
{
	MV result;
	result.mC[0] = 1.0;
	MV term = result;
	for (int k = 1; k <= 64; ++k)
	{
		term = Mul(term, inB) * (1.0 / k);
		result = result + term;
		if (term.MaxAbs() < 1.0e-18)
			break;
	}
	return result;
}

inline MV FromRotor(JPH::RotorArg inR)
{
	MV result;
	JPH::Vec8 v = inR.GetVec8();
	for (int i = 0; i < 8; ++i)
		result.mC[cRotorMask[i]] = double(v[i]);
	return result;
}

inline JPH::Rotor ToRotor(const MV &inA)
{
	return JPH::Rotor(
		float(inA.mC[cRotorMask[0]]), float(inA.mC[cRotorMask[1]]), float(inA.mC[cRotorMask[2]]), float(inA.mC[cRotorMask[3]]),
		float(inA.mC[cRotorMask[4]]), float(inA.mC[cRotorMask[5]]), float(inA.mC[cRotorMask[6]]), float(inA.mC[cRotorMask[7]]));
}

inline MV FromBivec(const JPH::Bivec &inB)
{
	MV result;
	for (int i = 0; i < 6; ++i)
		result.mC[cBivecMask[i]] = double(inB[i]);
	return result;
}

inline MV FromVec4(JPH::Vec4Arg inV)
{
	MV result;
	for (int i = 0; i < 4; ++i)
		result.mC[cVecMask[i]] = double(inV[i]);
	return result;
}

/// Max abs difference between the grade 1 part of inA and a Vec4
inline double Grade1Diff(const MV &inA, JPH::Vec4Arg inV)
{
	double result = 0.0;
	for (int i = 0; i < 4; ++i)
		result = std::max(result, std::abs(inA.mC[cVecMask[i]] - double(inV[i])));
	return result;
}

/// Sandwich product R v ~R (returns the full multivector, caller inspects grades)
inline MV Sandwich(const MV &inR, const MV &inV)
{
	return Mul(Mul(inR, inV), Reverse(inR));
}

/// R ~R for a valid rotor must be 1 (scalar 1, everything else 0). Returns the scalar part.
inline double RotorNormScalar(JPH::RotorArg inR)
{
	MV r = FromRotor(inR);
	return Mul(r, Reverse(r)).mC[0];
}

/// The e1234 component of R ~R ("defect"). Must be 0 for a valid element of Spin(4);
/// length normalization alone does not guarantee this.
inline double RotorNormDefect(JPH::RotorArg inR)
{
	MV r = FromRotor(inR);
	return Mul(r, Reverse(r)).mC[0b1111];
}

} // namespace Cl4Ref

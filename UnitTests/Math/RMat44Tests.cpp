// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include <Jolt/Math/RMat44.h>
#include <Jolt/Math/Rotor.h>

// RMat44 is the 4D rigid transform (Mat44 rotation + RVec4 translation) that replaces the old
// homogeneous 4x4 (CLAUDE.md Phase A step 2). These tests pin its composition semantics before
// the Shape / Broadphase / PhysicsSystem layers (Phase B) are built on top of it.
TEST_SUITE("RMat44Tests")
{
	// A fixed generic rotor (product of two simple rotations) used throughout
	static Rotor sTestRotor()
	{
		return Rotor::sRotation(Vec4::sAxisX(), Vec4::sAxisY(), 0.7f) * Rotor::sRotation(Vec4::sAxisZ(), Vec4::sAxisW(), -1.3f);
	}

	TEST_CASE("TestRMat44IdentityZeroTranslation")
	{
		RVec4 p(1, 2, 3, 4);
		CHECK_APPROX_EQUAL(RMat44::sIdentity() * Vec4(1, 2, 3, 4), p);
		CHECK(RMat44::sZero().GetRotation() == Mat44::sZero());

		RMat44 t = RMat44::sTranslation(RVec4(10, 20, 30, 40));
		CHECK_APPROX_EQUAL(t * Vec4(1, 2, 3, 4), RVec4(11, 22, 33, 44));
		CHECK_APPROX_EQUAL(t.GetTranslation(), RVec4(10, 20, 30, 40));
	}

	TEST_CASE("TestRMat44RotationTranslation")
	{
		Rotor r = sTestRotor();
		RVec4 t(1, -2, 3, -4);
		RMat44 m = RMat44::sRotationTranslation(r, t);

		// m * p == R * p + t, where the rotation must agree with the rotor sandwich product
		Vec4 p(0.5f, -1.5f, 2.5f, -3.5f);
		CHECK_APPROX_EQUAL(m * p, RVec4(r * p) + t, 1.0e-5f);

		// Rotation-only variants
		CHECK(RMat44::sRotation(r).GetRotation().IsClose(Mat44::sRotation(r), 1.0e-10f));
		CHECK_APPROX_EQUAL(m.Multiply3x3(p), r * p, 1.0e-5f);
		CHECK_APPROX_EQUAL(m.Multiply3x3Transposed(r * p), p, 1.0e-5f);
	}

	// Regression test for CLAUDE.md P0 bug #8 (see RotorTests): RMat44::GetRotor forwards to
	// Mat44::GetRotor, which used to recover the wrong sign for the second rotation plane of a
	// double rotation. sTestRotor is a double rotation, so this exercises it.
	TEST_CASE("TestRMat44GetRotor")
	{
		Rotor r = sTestRotor();
		RMat44 m = RMat44::sRotationTranslation(r, RVec4(1, -2, 3, -4));
		bool rotor_close = m.GetRotor().IsClose(r, 1.0e-5f) || m.GetRotor().IsClose(-r, 1.0e-5f);
		CHECK(rotor_close);
	}

	TEST_CASE("TestRMat44Compose")
	{
		RMat44 m1 = RMat44::sRotationTranslation(sTestRotor(), RVec4(1, 2, 3, 4));
		RMat44 m2 = RMat44::sRotationTranslation(Rotor::sRotation(Vec4::sAxisY(), Vec4::sAxisW(), 0.4f), RVec4(-4, -3, -2, -1));
		Vec4 p(0.5f, -1.5f, 2.5f, -3.5f);

		// Composition applies right to left
		RVec4 expected = m1 * Vec4(Vec4(m2 * p));
		CHECK_APPROX_EQUAL((m1 * m2) * p, expected, 1.0e-4f);

		// Composing with a rotation-only Mat44 keeps the translation
		Mat44 rot = Mat44::sRotation(Rotor::sRotation(Vec4::sAxisX(), Vec4::sAxisZ(), 0.3f));
		RMat44 m3 = m1 * rot;
		CHECK_APPROX_EQUAL(m3.GetTranslation(), m1.GetTranslation());
		CHECK_APPROX_EQUAL(m3 * p, m1 * Vec4(rot * Lane4(p)), 1.0e-4f);
	}

	TEST_CASE("TestRMat44PrePostTranslated")
	{
		RMat44 m = RMat44::sRotationTranslation(sTestRotor(), RVec4(1, 2, 3, 4));
		Vec4 p(0.5f, -1.5f, 2.5f, -3.5f);
		Vec4 d(2, -1, 0.5f, 3);

		// PreTranslated: translate in local space, i.e. R * (p + d) + t
		CHECK_APPROX_EQUAL(m.PreTranslated(d) * p, m * Vec4(p + d), 1.0e-4f);

		// PostTranslated: translate in world space, i.e. (R * p + t) + d
		CHECK_APPROX_EQUAL(m.PostTranslated(RVec4(2, -1, 0.5f, 3)) * p, (m * p) + RVec4(2, -1, 0.5f, 3), 1.0e-4f);
	}

	TEST_CASE("TestRMat44PrePostScaled")
	{
		RMat44 m = RMat44::sRotationTranslation(sTestRotor(), RVec4(1, 2, 3, 4));
		Vec4 p(0.5f, -1.5f, 2.5f, -3.5f);
		Vec4 s(2, 3, 0.5f, -1);

		// PreScaled: scale in local space, i.e. R * (s * p) + t
		CHECK_APPROX_EQUAL(m.PreScaled(s) * p, m * Vec4(s * p), 1.0e-4f);

		// PostScaled: scale in world space, i.e. s * (R * p + t)
		CHECK_APPROX_EQUAL(Vec4(m.PostScaled(s) * p), s * Vec4(m * p), 1.0e-4f);

		// sScale constructs a diagonal linear part
		CHECK_APPROX_EQUAL(RMat44::sScale(s) * p, RVec4(s * p), 1.0e-6f);
	}

	TEST_CASE("TestRMat44Inversed")
	{
		Rotor r = sTestRotor();
		RVec4 t(1, -2, 3, -4);
		RMat44 m = RMat44::sRotationTranslation(r, t);
		Vec4 p(0.5f, -1.5f, 2.5f, -3.5f);

		// Rigid inverse round trips a point
		RMat44 inv = m.InversedRotationTranslation();
		CHECK_APPROX_EQUAL(inv * Vec4(Vec4(m * p)), RVec4(p), 1.0e-4f);
		CHECK_APPROX_EQUAL(m * Vec4(Vec4(inv * p)), RVec4(p), 1.0e-4f);

		// Static constructor must agree with the member function
		RMat44 inv2 = RMat44::sInverseRotationTranslation(r, t);
		CHECK(inv2.GetRotation().IsClose(inv.GetRotation(), 1.0e-10f));
		CHECK_APPROX_EQUAL(inv2.GetTranslation(), inv.GetTranslation(), 1.0e-5f);

		// General inverse handles scale
		RMat44 ms = m.PreScaled(Vec4(2, 3, 0.5f, 4));
		CHECK_APPROX_EQUAL(ms.Inversed() * Vec4(Vec4(ms * p)), RVec4(p), 1.0e-3f);
	}

	TEST_CASE("TestRMat44Decompose")
	{
		Rotor r = sTestRotor();
		RVec4 t(1, -2, 3, -4);
		Vec4 s(2, 3, 0.5f, 4); // positive scale (decomposition is only unique up to sign)
		RMat44 m = RMat44::sRotationTranslation(r, t).PreScaled(s);

		Vec4 out_scale;
		RMat44 decomposed = m.Decompose(out_scale);
		CHECK_APPROX_EQUAL(out_scale, s, 1.0e-4f);
		CHECK(decomposed.GetRotation().IsClose(Mat44::sRotation(r), 1.0e-8f));
		CHECK_APPROX_EQUAL(decomposed.GetTranslation(), t, 1.0e-5f);
	}
}

// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include <Jolt/Math/Vec8.h>

// Vec8 is the SIMD storage backing Rotor and Bivec (CLAUDE.md Phase A step 2).
TEST_SUITE("Vec8Tests")
{
	TEST_CASE("TestVec8Construct")
	{
		Vec8 v(1, 2, 3, 4, 5, 6, 7, 8);
		for (int i = 0; i < 8; ++i)
			CHECK(v[uint(i)] == float(i + 1));

		CHECK(Vec8(Lane4(1, 2, 3, 4), Lane4(5, 6, 7, 8)) == v);
		CHECK(v.GetLow() == Lane4(1, 2, 3, 4));
		CHECK(v.GetHigh() == Lane4(5, 6, 7, 8));

		Vec8 zero = Vec8::sZero();
		for (int i = 0; i < 8; ++i)
			CHECK(zero[uint(i)] == 0.0f);

		Vec8 rep = Vec8::sReplicate(3.5f);
		for (int i = 0; i < 8; ++i)
			CHECK(rep[uint(i)] == 3.5f);
	}

	TEST_CASE("TestVec8SetComponent")
	{
		Vec8 v = Vec8::sZero();
		for (uint i = 0; i < 8; ++i)
			v.SetComponent(i, float(i));
		CHECK(v == Vec8(0, 1, 2, 3, 4, 5, 6, 7));
	}

	TEST_CASE("TestVec8Comparison")
	{
		Vec8 a(1, 2, 3, 4, 5, 6, 7, 8);
		CHECK(a == Vec8(1, 2, 3, 4, 5, 6, 7, 8));
		CHECK(a != Vec8(1, 2, 3, 4, 5, 6, 7, 9));
		CHECK(a.IsClose(Vec8(1, 2, 3, 4, 5, 6, 7, 8.0001f), 1.0e-6f));
		CHECK(!a.IsClose(Vec8(1, 2, 3, 4, 5, 6, 7, 9), 1.0e-6f));
		CHECK(!a.IsNaN());
	}

	TEST_CASE("TestVec8Arithmetic")
	{
		Vec8 a(1, 2, 3, 4, 5, 6, 7, 8), b(8, 7, 6, 5, 4, 3, 2, 1);
		CHECK(a + b == Vec8::sReplicate(9.0f));
		CHECK(a - b == Vec8(-7, -5, -3, -1, 1, 3, 5, 7));
		CHECK(-a == Vec8(-1, -2, -3, -4, -5, -6, -7, -8));
		CHECK(a * b == Vec8(8, 14, 18, 20, 20, 18, 14, 8));
		CHECK(a * 2.0f == Vec8(2, 4, 6, 8, 10, 12, 14, 16));
		CHECK(2.0f * a == a * 2.0f);
	}

	TEST_CASE("TestVec8DotLengthNormalize")
	{
		Vec8 a(1, 2, 3, 4, 5, 6, 7, 8), b(8, 7, 6, 5, 4, 3, 2, 1);
		CHECK_APPROX_EQUAL(a.Dot(b), 120.0f);
		CHECK_APPROX_EQUAL(a.LengthSq(), 204.0f);
		CHECK_APPROX_EQUAL(Vec8(2, 0, 0, 0, 0, 0, 0, 0).Length(), 2.0f);
		CHECK_APPROX_EQUAL(a.Length(), sqrt(204.0f), 1.0e-4f);

		CHECK(!a.IsNormalized());
		Vec8 n = a.Normalized();
		CHECK(n.IsNormalized());
		CHECK_APPROX_EQUAL(n.Length(), 1.0f, 1.0e-6f);
		CHECK(n.IsClose(a * (1.0f / a.Length()), 1.0e-10f));
	}

	TEST_CASE("TestVec8StoreFloat8")
	{
		Vec8 v(1, -2, 3, -4, 5, -6, 7, -8);
		Float8 f;
		v.StoreFloat8(&f);
		CHECK(Vec8(f) == v);
	}
}

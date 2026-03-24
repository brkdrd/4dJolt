// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include <Jolt/Math/DVec4.h>
#include <Jolt/Core/StringTools.h>

TEST_SUITE("DVec4Tests")
{
	TEST_CASE("TestDVec4Zero")
	{
		DVec4 v = DVec4::sZero();

		CHECK(v.GetX() == 0);
		CHECK(v.GetY() == 0);
		CHECK(v.GetZ() == 0);
	}

	TEST_CASE("TestDVec4Axis")
	{
		CHECK(DVec4::sAxisX() == DVec4(1, 0, 0));
		CHECK(DVec4::sAxisY() == DVec4(0, 1, 0));
		CHECK(DVec4::sAxisZ() == DVec4(0, 0, 1));
	}

	TEST_CASE("TestVec3NaN")
	{
		DVec4 v = DVec4::sNaN();

		CHECK(isnan(v.GetX()));
		CHECK(isnan(v.GetY()));
		CHECK(isnan(v.GetZ()));
		CHECK(v.IsNaN());

		v.SetComponent(0, 0);
		CHECK(v.IsNaN());
		v.SetComponent(1, 0);
		CHECK(v.IsNaN());
		v.SetComponent(2, 0);
		CHECK(!v.IsNaN());
	}

	TEST_CASE("TestDVec4ConstructComponents")
	{
		DVec4 v(1, 2, 3);

		// Test component access
		CHECK(v.GetX() == 1);
		CHECK(v.GetY() == 2);
		CHECK(v.GetZ() == 3);

		// Test component access by [] operators
		CHECK(v[0] == 1);
		CHECK(v[1] == 2);
		CHECK(v[2] == 3);

		// Test == and != operators
		CHECK(v == DVec4(1, 2, 3));
		CHECK(v != DVec4(1, 2, 4));

		// Set the components
		v.SetComponent(0, 4);
		v.SetComponent(1, 5);
		v.SetComponent(2, 6);
		CHECK(v == DVec4(4, 5, 6));

		// Set the components again
		v.SetX(7);
		v.SetY(8);
		v.SetZ(9);
		CHECK(v == DVec4(7, 8, 9));

		// Set all components
		v.Set(10, 11, 12);
		CHECK(v == DVec4(10, 11, 12));
	}

	TEST_CASE("TestVec4ToDVec4")
	{
		CHECK(DVec4(Vec4(1, 3, 5, 7)) == DVec4(1, 3, 5));
	}

	TEST_CASE("TestDVec4Replicate")
	{
		CHECK(DVec4::sReplicate(2) == DVec4(2, 2, 2));
	}

	TEST_CASE("TestDVec4ToVec3")
	{
		CHECK(Vec3(DVec4(1, 3, 5)) == Vec3(1, 3, 5));

		// Check rounding up and down
		CHECK(DVec4(2.0, 0x1.0000000000001p1, -0x1.0000000000001p1).ToVec3RoundUp() == Vec3(2.0, 0x1.000002p1f, -2.0));
		CHECK(DVec4(2.0, 0x1.0000000000001p1, -0x1.0000000000001p1).ToVec3RoundDown() == Vec3(2.0, 2.0, -0x1.000002p1f));
	}

	TEST_CASE("TestVec3MinMax")
	{
		DVec4 v1(1, 5, 3);
		DVec4 v2(4, 2, 6);

		CHECK(DVec4::sMin(v1, v2) == DVec4(1, 2, 3));
		CHECK(DVec4::sMax(v1, v2) == DVec4(4, 5, 6));
	}

	TEST_CASE("TestDVec4Clamp")
	{
		DVec4 v1(1, 2, 3);
		DVec4 v2(4, 5, 6);
		DVec4 v(-1, 3, 7);

		CHECK(DVec4::sClamp(v, v1, v2) == DVec4(1, 3, 6));
	}

	TEST_CASE("TestDVec4Trues")
	{
		CHECK(DVec4(DVec4::cFalse, DVec4::cFalse, DVec4::cFalse).GetTrues() == 0b0000);
		CHECK(DVec4(DVec4::cTrue, DVec4::cFalse, DVec4::cFalse).GetTrues() == 0b0001);
		CHECK(DVec4(DVec4::cFalse, DVec4::cTrue, DVec4::cFalse).GetTrues() == 0b0010);
		CHECK(DVec4(DVec4::cTrue, DVec4::cTrue, DVec4::cFalse).GetTrues() == 0b0011);
		CHECK(DVec4(DVec4::cFalse, DVec4::cFalse, DVec4::cTrue).GetTrues() == 0b0100);
		CHECK(DVec4(DVec4::cTrue, DVec4::cFalse, DVec4::cTrue).GetTrues() == 0b0101);
		CHECK(DVec4(DVec4::cFalse, DVec4::cTrue, DVec4::cTrue).GetTrues() == 0b0110);
		CHECK(DVec4(DVec4::cTrue, DVec4::cTrue, DVec4::cTrue).GetTrues() == 0b0111);

		CHECK(!DVec4(DVec4::cFalse, DVec4::cFalse, DVec4::cFalse).TestAnyTrue());
		CHECK(DVec4(DVec4::cTrue, DVec4::cFalse, DVec4::cFalse).TestAnyTrue());
		CHECK(DVec4(DVec4::cFalse, DVec4::cTrue, DVec4::cFalse).TestAnyTrue());
		CHECK(DVec4(DVec4::cTrue, DVec4::cTrue, DVec4::cFalse).TestAnyTrue());
		CHECK(DVec4(DVec4::cFalse, DVec4::cFalse, DVec4::cTrue).TestAnyTrue());
		CHECK(DVec4(DVec4::cTrue, DVec4::cFalse, DVec4::cTrue).TestAnyTrue());
		CHECK(DVec4(DVec4::cFalse, DVec4::cTrue, DVec4::cTrue).TestAnyTrue());
		CHECK(DVec4(DVec4::cTrue, DVec4::cTrue, DVec4::cTrue).TestAnyTrue());

		CHECK(!DVec4(DVec4::cFalse, DVec4::cFalse, DVec4::cFalse).TestAllTrue());
		CHECK(!DVec4(DVec4::cTrue, DVec4::cFalse, DVec4::cFalse).TestAllTrue());
		CHECK(!DVec4(DVec4::cFalse, DVec4::cTrue, DVec4::cFalse).TestAllTrue());
		CHECK(!DVec4(DVec4::cTrue, DVec4::cTrue, DVec4::cFalse).TestAllTrue());
		CHECK(!DVec4(DVec4::cFalse, DVec4::cFalse, DVec4::cTrue).TestAllTrue());
		CHECK(!DVec4(DVec4::cTrue, DVec4::cFalse, DVec4::cTrue).TestAllTrue());
		CHECK(!DVec4(DVec4::cFalse, DVec4::cTrue, DVec4::cTrue).TestAllTrue());
		CHECK(DVec4(DVec4::cTrue, DVec4::cTrue, DVec4::cTrue).TestAllTrue());
	}

	TEST_CASE("TestDVec4Comparisons")
	{
		CHECK(DVec4::sEquals(DVec4(1, 2, 3), DVec4(1, 4, 3)).GetTrues() == 0b101); // Can't directly check if equal to (true, false, true) because true = -NaN and -NaN != -NaN
		CHECK(DVec4::sLess(DVec4(1, 2, 4), DVec4(1, 4, 3)).GetTrues() == 0b010);
		CHECK(DVec4::sLessOrEqual(DVec4(1, 2, 4), DVec4(1, 4, 3)).GetTrues() == 0b011);
		CHECK(DVec4::sGreater(DVec4(1, 2, 4), DVec4(1, 4, 3)).GetTrues() == 0b100);
		CHECK(DVec4::sGreaterOrEqual(DVec4(1, 2, 4), DVec4(1, 4, 3)).GetTrues() == 0b101);
	}

	TEST_CASE("TestDVec4FMA")
	{
		CHECK(DVec4::sFusedMultiplyAdd(DVec4(1, 2, 3), DVec4(4, 5, 6), DVec4(7, 8, 9)) == DVec4(1 * 4 + 7, 2 * 5 + 8, 3 * 6 + 9));
	}

	TEST_CASE("TestDVec4Select")
	{
		const double cTrue2 = BitCast<double>(uint64(1) << 63);
		const double cFalse2 = BitCast<double>(~uint64(0) >> 1);

		CHECK(DVec4::sSelect(DVec4(1, 2, 3), DVec4(4, 5, 6), DVec4(DVec4::cTrue, DVec4::cFalse, DVec4::cTrue)) == DVec4(4, 2, 6));
		CHECK(DVec4::sSelect(DVec4(1, 2, 3), DVec4(4, 5, 6), DVec4(DVec4::cFalse, DVec4::cTrue, DVec4::cFalse)) == DVec4(1, 5, 3));
		CHECK(DVec4::sSelect(DVec4(1, 2, 3), DVec4(4, 5, 6), DVec4(cTrue2, cFalse2, cTrue2)) == DVec4(4, 2, 6));
		CHECK(DVec4::sSelect(DVec4(1, 2, 3), DVec4(4, 5, 6), DVec4(cFalse2, cTrue2, cFalse2)) == DVec4(1, 5, 3));
	}

	TEST_CASE("TestDVec4BitOps")
	{
		// Test all bit permutations
		DVec4 v1(BitCast<double, uint64>(0b0011), BitCast<double, uint64>(0b00110), BitCast<double, uint64>(0b001100));
		DVec4 v2(BitCast<double, uint64>(0b0101), BitCast<double, uint64>(0b01010), BitCast<double, uint64>(0b010100));

		CHECK(DVec4::sOr(v1, v2) == DVec4(BitCast<double, uint64>(0b0111), BitCast<double, uint64>(0b01110), BitCast<double, uint64>(0b011100)));
		CHECK(DVec4::sXor(v1, v2) == DVec4(BitCast<double, uint64>(0b0110), BitCast<double, uint64>(0b01100), BitCast<double, uint64>(0b011000)));
		CHECK(DVec4::sAnd(v1, v2) == DVec4(BitCast<double, uint64>(0b0001), BitCast<double, uint64>(0b00010), BitCast<double, uint64>(0b000100)));
	}

	TEST_CASE("TestDVec4Close")
	{
		CHECK(DVec4(1, 2, 3).IsClose(DVec4(1.001, 2.001, 3.001), 1.0e-4));
		CHECK(!DVec4(1, 2, 3).IsClose(DVec4(1.001, 2.001, 3.001), 1.0e-6));

		CHECK(DVec4(1.001, 0, 0).IsNormalized(1.0e-2));
		CHECK(!DVec4(0, 1.001, 0).IsNormalized(1.0e-4));

		CHECK(DVec4(-1.0e-7, 1.0e-7, 1.0e-8).IsNearZero(1.0e-12));
		CHECK(!DVec4(-1.0e-7, 1.0e-7, -1.0e-5).IsNearZero(1.0e-12));
	}

	TEST_CASE("TestDVec4Operators")
	{
		CHECK(-DVec4(1, 2, 3) == DVec4(-1, -2, -3));

		DVec4 neg_zero = -DVec4::sZero();
		CHECK(neg_zero == DVec4::sZero());

	#ifdef JPH_CROSS_PLATFORM_DETERMINISTIC
		// When cross platform deterministic, we want to make sure that -0 is represented as 0
		CHECK(BitCast<uint64>(neg_zero.GetX()) == 0);
		CHECK(BitCast<uint64>(neg_zero.GetY()) == 0);
		CHECK(BitCast<uint64>(neg_zero.GetZ()) == 0);
	#endif // JPH_CROSS_PLATFORM_DETERMINISTIC

		CHECK(DVec4(1, 2, 3) + Vec3(4, 5, 6) == DVec4(5, 7, 9));
		CHECK(DVec4(1, 2, 3) - Vec3(6, 5, 4) == DVec4(-5, -3, -1));

		CHECK(DVec4(1, 2, 3) + DVec4(4, 5, 6) == DVec4(5, 7, 9));
		CHECK(DVec4(1, 2, 3) - DVec4(6, 5, 4) == DVec4(-5, -3, -1));

		CHECK(DVec4(1, 2, 3) * DVec4(4, 5, 6) == DVec4(4, 10, 18));
		CHECK(DVec4(1, 2, 3) * 2 == DVec4(2, 4, 6));
		CHECK(4 * DVec4(1, 2, 3) == DVec4(4, 8, 12));

		CHECK(DVec4(1, 2, 3) / 2 == DVec4(0.5, 1.0, 1.5));
		CHECK(DVec4(1, 2, 3) / DVec4(2, 8, 24) == DVec4(0.5, 0.25, 0.125));

		DVec4 v = DVec4(1, 2, 3);
		v *= DVec4(4, 5, 6);
		CHECK(v == DVec4(4, 10, 18));
		v *= 2;
		CHECK(v == DVec4(8, 20, 36));
		v /= 2;
		CHECK(v == DVec4(4, 10, 18));
		v += DVec4(1, 2, 3);
		CHECK(v == DVec4(5, 12, 21));
		v -= DVec4(1, 2, 3);
		CHECK(v == DVec4(4, 10, 18));
		v += Vec3(1, 2, 3);
		CHECK(v == DVec4(5, 12, 21));
		v -= Vec3(1, 2, 3);
		CHECK(v == DVec4(4, 10, 18));

		CHECK(DVec4(2, 4, 8).Reciprocal() == DVec4(0.5, 0.25, 0.125));
	}

	TEST_CASE("TestDVec4Abs")
	{
		CHECK(DVec4(1, -2, 3).Abs() == DVec4(1, 2, 3));
		CHECK(DVec4(-1, 2, -3).Abs() == DVec4(1, 2, 3));
	}

	TEST_CASE("TestDVec4Dot")
	{
		CHECK(DVec4(2, 3, 4).Dot(DVec4(5, 6, 7)) == double(2 * 5 + 3 * 6 + 4 * 7));
	}

	TEST_CASE("TestDVec4Length")
	{
		CHECK(DVec4(2, 3, 4).LengthSq() == double(4 + 9 + 16));
		CHECK(DVec4(2, 3, 4).Length() == sqrt(double(4 + 9 + 16)));
	}

	TEST_CASE("TestDVec4Sqrt")
	{
		CHECK(DVec4(13, 15, 17).Sqrt() == DVec4(sqrt(13.0), sqrt(15.0), sqrt(17.0)));
	}

	TEST_CASE("TestDVec4Equals")
	{
		CHECK_FALSE(DVec4(13, 15, 17) == DVec4(13, 15, 19));
		CHECK(DVec4(13, 15, 17) == DVec4(13, 15, 17));
		CHECK(DVec4(13, 15, 17) != DVec4(13, 15, 19));
	}

	TEST_CASE("TestDVec4LoadStoreDouble3Unsafe")
	{
		double d4[4] = { 1, 2, 3, 4 };
		Double3 &d3 = *(Double3 *)d4;
		DVec4 v = DVec4::sLoadDouble3Unsafe(d3);
		DVec4 v2(1, 2, 3);
		CHECK(v == v2);

		Double3 d3_out;
		DVec4(1, 2, 3).StoreDouble3(&d3_out);
		CHECK(d3 == d3_out);
	}

	TEST_CASE("TestDVec4Cross")
	{
		CHECK(DVec4(1, 0, 0).Cross(DVec4(0, 1, 0)) == DVec4(0, 0, 1));
		CHECK(DVec4(0, 1, 0).Cross(DVec4(1, 0, 0)) == DVec4(0, 0, -1));
		CHECK(DVec4(0, 1, 0).Cross(DVec4(0, 0, 1)) == DVec4(1, 0, 0));
		CHECK(DVec4(0, 0, 1).Cross(DVec4(0, 1, 0)) == DVec4(-1, 0, 0));
		CHECK(DVec4(0, 0, 1).Cross(DVec4(1, 0, 0)) == DVec4(0, 1, 0));
		CHECK(DVec4(1, 0, 0).Cross(DVec4(0, 0, 1)) == DVec4(0, -1, 0));
	}

	TEST_CASE("TestDVec4Normalize")
	{
		CHECK(DVec4(3, 2, 1).Normalized() == DVec4(3, 2, 1) / sqrt(9.0 + 4.0 + 1.0));
	}

	TEST_CASE("TestDVec4Sign")
	{
		CHECK(DVec4(1.2345, -6.7891, 0).GetSign() == DVec4(1, -1, 1));
		CHECK(DVec4(0, 2.3456, -7.8912).GetSign() == DVec4(1, 1, -1));
	}

	TEST_CASE("TestDVec4ConvertToString")
	{
		DVec4 v(1, 2, 3);
		CHECK(ConvertToString(v) == "1, 2, 3");
	}
}

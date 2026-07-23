// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include <Jolt/Geometry/RayTriangle.h>

TEST_SUITE("RayTriangleTests")
{
	TEST_CASE("TestRayTriangleHitXYZ")
	{
		// Triangle in XYZ plane (W=0)
		Vec4 v0(0, 0, 0, 0);
		Vec4 v1(2, 0, 0, 0);
		Vec4 v2(0, 2, 0, 0);

		// Ray along Z hitting the triangle interior
		Vec4 origin(0.5f, 0.5f, -3, 0);
		Vec4 direction(0, 0, 1, 0);
		float t = RayTriangle(origin, direction, v0, v1, v2);
		CHECK_APPROX_EQUAL(t, 3.0f, 1.0e-5f);
	}

	TEST_CASE("TestRayTriangleHitW")
	{
		// Triangle in XYW plane (Z=0)
		Vec4 v0(0, 0, 0, 0);
		Vec4 v1(2, 0, 0, 0);
		Vec4 v2(0, 0, 0, 2);

		// Ray along Y hitting the triangle
		Vec4 origin(0.5f, -5, 0, 0.5f);
		Vec4 direction(0, 1, 0, 0);
		float t = RayTriangle(origin, direction, v0, v1, v2);
		CHECK_APPROX_EQUAL(t, 5.0f, 1.0e-5f);
	}

	TEST_CASE("TestRayTriangleMissOutside")
	{
		Vec4 v0(0, 0, 0, 0);
		Vec4 v1(1, 0, 0, 0);
		Vec4 v2(0, 1, 0, 0);

		// Ray that misses the triangle (outside u+v > 1 region)
		Vec4 origin(0.8f, 0.8f, -1, 0);
		Vec4 direction(0, 0, 1, 0);
		float t = RayTriangle(origin, direction, v0, v1, v2);
		CHECK(t == FLT_MAX);
	}

	TEST_CASE("TestRayTriangleMissBehind")
	{
		Vec4 v0(0, 0, 0, 0);
		Vec4 v1(1, 0, 0, 0);
		Vec4 v2(0, 1, 0, 0);

		// Ray pointing away from triangle
		Vec4 origin(0.25f, 0.25f, 1, 0);
		Vec4 direction(0, 0, 1, 0); // moving away
		float t = RayTriangle(origin, direction, v0, v1, v2);
		CHECK(t == FLT_MAX);
	}

	TEST_CASE("TestRayTriangleMiss4D")
	{
		// Triangle in XYZ plane (W=0)
		Vec4 v0(0, 0, 0, 0);
		Vec4 v1(1, 0, 0, 0);
		Vec4 v2(0, 1, 0, 0);

		// Ray that would hit in 3D (projects onto the triangle) but misses in 4D
		// because origin has non-zero W and direction is in Z only
		Vec4 origin(0.25f, 0.25f, -1, 1);
		Vec4 direction(0, 0, 1, 0);
		float t = RayTriangle(origin, direction, v0, v1, v2);
		// The ray moves in Z, but origin has W=1 while triangle is at W=0.
		// The residual in W will be 1.0 for any t, so this should be a miss.
		CHECK(t == FLT_MAX);
	}

	TEST_CASE("TestRayTriangleParallel")
	{
		Vec4 v0(0, 0, 0, 0);
		Vec4 v1(1, 0, 0, 0);
		Vec4 v2(0, 1, 0, 0);

		// Ray parallel to triangle
		Vec4 origin(0.25f, 0.25f, 1, 0);
		Vec4 direction(1, 0, 0, 0);
		float t = RayTriangle(origin, direction, v0, v1, v2);
		CHECK(t == FLT_MAX);
	}

	TEST_CASE("TestRayTriangleVertex")
	{
		Vec4 v0(0, 0, 0, 0);
		Vec4 v1(1, 0, 0, 0);
		Vec4 v2(0, 1, 0, 0);

		// Ray hitting vertex v0
		Vec4 origin(0, 0, -2, 0);
		Vec4 direction(0, 0, 1, 0);
		float t = RayTriangle(origin, direction, v0, v1, v2);
		CHECK_APPROX_EQUAL(t, 2.0f, 1.0e-5f);
	}

	TEST_CASE("TestRayTriangle4DSOA")
	{
		// Test the SOA version with 4 triangles
		Vec4 origin(0.25f, 0.25f, -3, 0);
		Vec4 direction(0, 0, 1, 0);

		// Triangle 0: should hit at t=3
		// Triangle 1: should hit at t=5
		// Triangle 2: should miss (origin outside)
		// Triangle 3: should hit at t=1
		Vec4 v0x(0, 0, 3, 0);
		Vec4 v0y(0, 0, 3, 0);
		Vec4 v0z(0, 0, 0, 0);
		Vec4 v0w(0, 0, 0, 0);

		Vec4 v1x(2, 2, 0.01f, 2);
		Vec4 v1y(0, 0, 0, 0);
		Vec4 v1z(0, 2, 0, -2);
		Vec4 v1w(0, 0, 0, 0);

		Vec4 v2x(0, 0, 0, 0);
		Vec4 v2y(2, 2, 0.01f, 2);
		Vec4 v2z(0, 2, 0, -2);
		Vec4 v2w(0, 0, 0, 0);

		Vec4 result = RayTriangle4(origin, direction, v0x, v0y, v0z, v0w, v1x, v1y, v1z, v1w, v2x, v2y, v2z, v2w);

		CHECK_APPROX_EQUAL(result.GetX(), 3.0f, 1.0e-3f); // Triangle 0
		CHECK_APPROX_EQUAL(result.GetY(), 5.0f, 1.0e-3f); // Triangle 1
		CHECK(result.GetZ() == FLT_MAX);                    // Triangle 2 - miss
		CHECK_APPROX_EQUAL(result.GetW(), 1.0f, 1.0e-3f);  // Triangle 3
	}
}

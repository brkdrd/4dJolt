// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include <Jolt/Geometry/AABox.h>

// 4D AABox tests, focused on the support / supporting-face queries that BoxShape (tesseract)
// builds on and that feed the contact-manifold pipeline.
TEST_SUITE("AABoxTests")
{
	TEST_CASE("TestAABox4DVolumeAndSurface")
	{
		AABox box(-Vec4(1, 2, 3, 4), Vec4(1, 2, 3, 4));
		CHECK_APPROX_EQUAL(box.GetVolume(), 2.0f * 4.0f * 6.0f * 8.0f); // hypervolume = product of edge lengths
	}

	TEST_CASE("TestAABoxGetSupport")
	{
		AABox box(-Vec4(1, 2, 3, 4), Vec4(1, 2, 3, 4));

		// Support returns the corner whose signs match the direction
		CHECK(box.GetSupport(Vec4(1, 1, 1, 1)) == Vec4(1, 2, 3, 4));
		CHECK(box.GetSupport(Vec4(-1, -1, -1, -1)) == Vec4(-1, -2, -3, -4));
		CHECK(box.GetSupport(Vec4(-1, 1, -1, 1)) == Vec4(-1, 2, -3, 4));

		// Each axis direction picks the corresponding extreme on that axis
		CHECK(box.GetSupport(Vec4::sAxisW()).GetW() == 4.0f);
		CHECK(box.GetSupport(-Vec4::sAxisW()).GetW() == -4.0f);
	}

	// In 4D the supporting face of a hyperbox is a 3D cube (8 vertices) lying on the cell whose
	// outward normal faces inDirection the most. The fixed coordinate must be mMax for a positive
	// direction component (consistent with GetSupport) -- this pins the sign of that choice.
	TEST_CASE("TestAABoxGetSupportingFace")
	{
		Vec4 he(1, 2, 3, 4);
		AABox box(-he, he);

		for (int axis = 0; axis < 4; ++axis)
			for (int sign = -1; sign <= 1; sign += 2)
			{
				Vec4 dir = Vec4::sZero();
				dir[uint(axis)] = float(sign);

				StaticArray<Vec4, 32> face;
				box.GetSupportingFace(dir, face);

				// A cube cell of the tesseract has 8 vertices
				CHECK(face.size() == 8);

				// All lie on the extreme of the dominant axis matching the direction sign, and
				// GetSupport must be one of the returned face vertices
				float expected = sign > 0? he[uint(axis)] : -he[uint(axis)];
				bool all_on_cell = true, contains_support = false;
				Vec4 support = box.GetSupport(dir);
				for (const Vec4 &v : face)
				{
					all_on_cell = all_on_cell && v[uint(axis)] == expected;
					contains_support = contains_support || v == support;
				}
				CHECK(all_on_cell);
				CHECK(contains_support);
			}
	}
}

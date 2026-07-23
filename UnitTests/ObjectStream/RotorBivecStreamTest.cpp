// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include <Jolt/Core/Factory.h>
#include <Jolt/Math/Rotor.h>
#include <Jolt/Math/Bivec.h>
#include <Jolt/ObjectStream/ObjectStreamOut.h>
#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Jolt/ObjectStream/SerializableObject.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>

// Round-trips the 4D rotation primitives (Vec4 / Rotor / Bivec) through the text and binary
// object streams (Phase B step 5). Rotor and Bivec were newly registered as ObjectStream
// primitives; this pins that they serialize losslessly.
JPH_SUPPRESS_WARNINGS_STD_BEGIN
#include <sstream>
JPH_SUPPRESS_WARNINGS_STD_END

namespace RotorBivecStreamTestNS {

class RotorSer
{
	JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_NO_EXPORT, RotorSer)
public:
	Vec4	mVec4 = Vec4::sZero();
	Rotor	mRotor = Rotor::sIdentity();
	Bivec	mBivec = Bivec::sZero();
};

JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(RotorSer)
{
	JPH_ADD_ATTRIBUTE(RotorSer, mVec4)
	JPH_ADD_ATTRIBUTE(RotorSer, mRotor)
	JPH_ADD_ATTRIBUTE(RotorSer, mBivec)
}

} // namespace RotorBivecStreamTestNS

using RotorBivecStreamTestNS::RotorSer;

TEST_SUITE("RotorBivecStreamTest")
{
	static void sRoundTrip(ObjectStreamOut::EStreamType inType)
	{
		Factory::sInstance->Register(JPH_RTTI(RotorSer));

		RotorSer in;
		in.mVec4 = Vec4(1.5f, -2.5f, 3.5f, -4.5f);
		in.mRotor = Rotor::sRotation(Vec4::sAxisX(), Vec4::sAxisY(), 0.7f) * Rotor::sRotation(Vec4::sAxisZ(), Vec4::sAxisW(), -1.3f);
		in.mBivec = Bivec(0.1f, -0.2f, 0.3f, -0.4f, 0.5f, -0.6f);

		stringstream stream;
		CHECK(ObjectStreamOut::sWriteObject(stream, inType, in));

		RotorSer *out = nullptr;
		CHECK(ObjectStreamIn::sReadObject(stream, out));
		CHECK(out != nullptr);
		if (out != nullptr)
		{
			CHECK(out->mVec4.IsClose(in.mVec4, 1.0e-10f));
			CHECK(out->mRotor.GetVec8().IsClose(in.mRotor.GetVec8(), 1.0e-10f));
			CHECK(out->mBivec.GetVec8().IsClose(in.mBivec.GetVec8(), 1.0e-10f));
			delete out;
		}
	}

	TEST_CASE("TestRotorBivecStreamText")
	{
		sRoundTrip(ObjectStreamOut::EStreamType::Text);
	}

	TEST_CASE("TestRotorBivecStreamBinary")
	{
		sRoundTrip(ObjectStreamOut::EStreamType::Binary);
	}
}

// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVertexIterator.h>
#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#ifdef JPH_DEBUG_RENDERER
	#include <Jolt/Renderer/DebugRenderer.h>
#endif // JPH_DEBUG_RENDERER

JPH_NAMESPACE_BEGIN

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(BoxShapeSettings)
{
	JPH_ADD_BASE_CLASS(BoxShapeSettings, ConvexShapeSettings)

	JPH_ADD_ATTRIBUTE(BoxShapeSettings, mHalfExtent)
	JPH_ADD_ATTRIBUTE(BoxShapeSettings, mConvexRadius)
}

ShapeSettings::ShapeResult BoxShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new BoxShape(*this, mCachedResult);
	return mCachedResult;
}

BoxShape::BoxShape(const BoxShapeSettings &inSettings, ShapeResult &outResult) :
	ConvexShape(EShapeSubType::Box, inSettings, outResult),
	mHalfExtent(inSettings.mHalfExtent),
	mConvexRadius(min(inSettings.mConvexRadius, inSettings.mHalfExtent.ReduceMin()))
{
	// Check half extents
	if (inSettings.mHalfExtent.ReduceMin() < 0.0f)
	{
		outResult.SetError("Invalid half extent");
		return;
	}

	// Check convex radius
	if (inSettings.mConvexRadius < 0.0f)
	{
		outResult.SetError("Invalid convex radius");
		return;
	}

	// Result is valid
	outResult.Set(this);
}

class BoxShape::Box final : public Support
{
public:
					Box(const AABox &inBox, float inConvexRadius) :
		mBox(inBox),
		mConvexRadius(inConvexRadius)
	{
		static_assert(sizeof(Box) <= sizeof(SupportBuffer), "Buffer size too small");
		JPH_ASSERT(IsAligned(this, alignof(Box)));
	}

	virtual Vec4	GetSupport(Vec4Arg inDirection) const override
	{
		return mBox.GetSupport(inDirection);
	}

	virtual float	GetConvexRadius() const override
	{
		return mConvexRadius;
	}

private:
	AABox			mBox;
	float			mConvexRadius;
};

const ConvexShape::Support *BoxShape::GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec4Arg inScale) const
{
	// Scale our half extents
	Vec4 scaled_half_extent = inScale.Abs() * mHalfExtent;

	switch (inMode)
	{
	case ESupportMode::IncludeConvexRadius:
	case ESupportMode::Default:
		{
			// Make box out of our half extents
			AABox box = AABox(-scaled_half_extent, scaled_half_extent);
			JPH_ASSERT(box.IsValid());
			return new (&inBuffer) Box(box, 0.0f);
		}

	case ESupportMode::ExcludeConvexRadius:
		{
			// Reduce the box by our convex radius
			float convex_radius = ScaleHelpers::ScaleConvexRadius(mConvexRadius, inScale);
			Vec4 convex_radius4 = Vec4::sReplicate(convex_radius);
			Vec4 reduced_half_extent = scaled_half_extent - convex_radius4;
			AABox box = AABox(-reduced_half_extent, reduced_half_extent);
			JPH_ASSERT(box.IsValid());
			return new (&inBuffer) Box(box, convex_radius);
		}
	}

	JPH_ASSERT(false);
	return nullptr;
}

void BoxShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec4Arg inDirection, Vec4Arg inScale, RMat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	// In 4D the supporting face of a tesseract is a 3D cube (8 vertices)
	Vec4 scaled_half_extent = inScale.Abs() * mHalfExtent;
	AABox box(-scaled_half_extent, scaled_half_extent);
	box.GetSupportingFace(inDirection, outVertices);

	// Transform to world space (narrow to single precision; the face is used relative to the body)
	for (Vec4 &v : outVertices)
	{
		RVec4 world = inCenterOfMassTransform * v;
		v = Vec4(float(world.GetX()), float(world.GetY()), float(world.GetZ()), float(world.GetW()));
	}
}

MassProperties BoxShape::GetMassProperties() const
{
	MassProperties p;
	p.SetMassAndInertiaOfSolidBox(2.0f * mHalfExtent, GetDensity());
	return p;
}

Vec4 BoxShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec4Arg inLocalSurfacePosition) const
{
	JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	// Get component that is closest to the surface of the box
	int index = (inLocalSurfacePosition.Abs() - mHalfExtent).Abs().GetLowestComponentIndex();

	// Calculate normal
	Vec4 normal = Vec4::sZero();
	normal[uint(index)] = inLocalSurfacePosition[uint(index)] > 0.0f? 1.0f : -1.0f;
	return normal;
}

#ifdef JPH_DEBUG_RENDERER
void BoxShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec4Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	// TODO(4D): DebugRenderer::DrawBox expects a 3D transform. Deferred until the renderer is migrated to 4D.
	(void)inRenderer; (void)inCenterOfMassTransform; (void)inScale; (void)inColor; (void)inUseMaterialColors; (void)inDrawWireframe;
}
#endif // JPH_DEBUG_RENDERER

bool BoxShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	// Test hit against box
	float fraction = max(RayAABox(inRay.mOrigin, RayInvDirection(inRay.mDirection), -mHalfExtent, mHalfExtent), 0.0f);
	if (fraction < ioHit.mFraction)
	{
		ioHit.mFraction = fraction;
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}
	return false;
}

void BoxShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	float min_fraction, max_fraction;
	RayAABox(inRay.mOrigin, RayInvDirection(inRay.mDirection), -mHalfExtent, mHalfExtent, min_fraction, max_fraction);
	if (min_fraction <= max_fraction // Ray should intersect
		&& max_fraction >= 0.0f // End of ray should be inside box
		&& min_fraction < ioCollector.GetEarlyOutFraction()) // Start of ray should be before early out fraction
	{
		// Better hit than the current hit
		RayCastResult hit;
		hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
		hit.mSubShapeID2 = inSubShapeIDCreator.GetID();

		// Check front side
		if (inRayCastSettings.mTreatConvexAsSolid || min_fraction > 0.0f)
		{
			hit.mFraction = max(0.0f, min_fraction);
			ioCollector.AddHit(hit);
		}

		// Check back side hit
		if (inRayCastSettings.mBackFaceModeConvex == EBackFaceMode::CollideWithBackFaces
			&& max_fraction < ioCollector.GetEarlyOutFraction())
		{
			hit.mFraction = max_fraction;
			ioCollector.AddHit(hit);
		}
	}
}

void BoxShape::CollidePoint(Vec4Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	if (Vec4::sLessOrEqual(inPoint.Abs(), mHalfExtent).TestAllTrue())
		ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator.GetID() });
}

void BoxShape::CollideSoftBodyVertices(RMat44Arg inCenterOfMassTransform, Vec4Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	// TODO(4D): CollideSoftBodyVertexIterator still exposes Vec3 positions. Once the soft-body
	// subsystem is migrated to 4D, rewrite this loop to operate on Vec4 directly.
	(void)inCenterOfMassTransform; (void)inScale; (void)inVertices; (void)inNumVertices; (void)inCollidingShapeIndex;
}

void BoxShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec4Arg inPositionCOM, RotorArg inRotation, Vec4Arg inScale) const
{
	// TODO(4D): the tesseract boundary is 8 cubic cells, each a tetrahedral mesh; the
	// GetTrianglesContext pipeline still assumes a triangle mesh. Stubbed like SphereShape until
	// the 4D boundary tessellation is defined.
	(void)ioContext; (void)inBox; (void)inPositionCOM; (void)inRotation; (void)inScale;
}

int BoxShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float4 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	// TODO(4D): See GetTrianglesStart.
	(void)ioContext; (void)inMaxTrianglesRequested; (void)outTriangleVertices; (void)outMaterials;
	return 0;
}

void BoxShape::SaveBinaryState(StreamOut &inStream) const
{
	ConvexShape::SaveBinaryState(inStream);

	inStream.Write(mHalfExtent);
	inStream.Write(mConvexRadius);
}

void BoxShape::RestoreBinaryState(StreamIn &inStream)
{
	ConvexShape::RestoreBinaryState(inStream);

	inStream.Read(mHalfExtent);
	inStream.Read(mConvexRadius);
}

void BoxShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Box);
	f.mConstruct = []() -> Shape * { return new BoxShape; };
	f.mColor = Color::sGreen;
}

JPH_NAMESPACE_END

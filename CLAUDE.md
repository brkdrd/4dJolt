# 4D Jolt — Porting Jolt Physics to Four Spatial Dimensions

## What this project is

A fork of Jolt Physics **v5.5.1** (fork point = `master` = `6576239a`) being converted into a
4-spatial-dimension rigid-body physics engine on branch `4d`. The guiding idea: keep Jolt's
architecture and change only what dimension-dependence forces us to change. Most of the
*geometry* does generalize cleanly (GJK, EPA, quickhull, SAT, ray tests). Most of the
*rotational dynamics and contact resolution* does **not** — see "Things that do not trivially
generalize" below. Treat that section as load-bearing: underestimating it is how this project
got stuck.

- `master` = pristine upstream 3D Jolt, kept as reference. Never commit 4D work to it.
- `4d` = the working branch. As of 2026-07-22 most of the Geometry/Physics/ObjectStream work
  is **uncommitted working-tree changes** on top of commit `6f770d01 "Math for 4D"`.

## Build & test

```bash
cmake -S Build -B build -DCMAKE_BUILD_TYPE=Release   # already configured in ./build (gcc, Release)
cmake --build build -j$(nproc)                        # currently FAILS, see status below
./build/UnitTests                                     # binary in repo is STALE (pre-4D tests) — do not trust it
```

**Current build status (verified 2026-07-22): broken, ~3,270 errors** with `make -k` (down
from ~3,800: AABox include fix in Phase A, then the TransformedShape/Shape base fixes and the
BoxShape/SphereShape ports in Phase B step 6). The trivial first-blocker cascade is gone; every
remaining error is in an un-migrated layer, so the counts are now meaningful.
Error census by area: ConstraintPart headers (~840, the largest), DebugRenderer (~260), Shape
hierarchy (~700), SoftBody/Character/Hair/PhysicsSystem (~200). Dominant error classes:
`Vec3 ↔ Bivec` conversions, `Body::GetInverseInertia` removed, `Mat44::Multiply3x3` removed,
`RVec4 → RVec3` conversions. i.e. the un-migrated layers no longer compile against the
migrated Body/Math API. This is Phase B work.

## The 4D type system (established, do not re-litigate)

| 3D Jolt | 4D Jolt | Notes |
|---|---|---|
| `Vec3` | `Vec4` | Spatial 4D vector. Thin subclass of `Lane4`; adds `sAxisW`, ternary `sCross(a,b,c)` (generalized cross product), `GetNormalizedPerpendicular`. |
| `Vec4` (SIMD util) | `Lane4` | Pure SIMD 4-lane utility (matrix columns, batch trig). NOT spatial. |
| `Quat` | `Rotor` | Rotation = even subalgebra of Cl(4,0): 8 floats `[s, e12, e13, e14, e23, e24, e34, e1234]` in a `Vec8`. |
| (new) | `Bivec` | 6-component bivector `[e12,e13,e14,e23,e24,e34]` (Vec8 storage). Angular velocity, torque, constraint rotation planes. |
| `Mat44` (rot+trans) | `Mat44` | Pure 4×4 linear map (rotation, optionally scale). **No translation column.** |
| `DMat44` | `RMat44` | Rigid transform = `Mat44` rotation + `RVec4` translation. Method names like `Multiply3x3` kept for diff-friendliness but operate on the full 4×4. |
| `DVec3` | `DVec4` | Double-precision spatial vector. ⚠ still half-converted, see bug list. |
| `RVec3` | `RVec4` | = `Vec4` or `DVec4` depending on `JPH_DOUBLE_PRECISION` (`Real.h`). |
| `Float3`/`Double3` | `Float4`/`Double4` | POD storage. `Float8` = POD for Vec8/Rotor. |
| Triangle-based geometry | Tetrahedron-based | `Tetrahedron.h`, `IndexedTetrahedron.h`, `RayTetrahedron.h`. Convex hull cells = tetrahedra, ridges = triangles. GJK simplex = up to 5 vertices (pentachoron). |

`Vec3.{h,inl,cpp}`, `Quat.{h,inl}`, and `Quat`-overloads in `Mat44`/`RMat44` still exist as
"backward compat" — they are what the un-migrated layers compile against. They should be
deleted once nothing references them; do not build new code on them.

### Cl(4,0) conventions

Basis index order for Rotor: `[1, e12, e13, e14, e23, e24, e34, e1234]`. `eᵢ² = +1`,
`e1234² = +1`, all bivectors square to −1. Reverse negates only grade 2. Full 8×8 geometric
product table and derived formulas live in the auto-memory file `project_4d_rotor_formulas.md`
and are implemented in `Jolt/Math/Rotor.inl` (scalar code). Sandwich `R v R̃` computes the full
odd multivector then extracts grade 1.

**Key fact used everywhere:** Spin(4) ≅ SU(2)×SU(2). Any bivector B splits by pseudoscalar
duality (B̃ = B·e1234) into commuting self-dual and anti-self-dual parts B± = ½(B ± B̃), each
of which squares to a non-positive scalar and exponentiates like a quaternion:
`exp(B) = exp(B₊)·exp(B₋)`. This is the correct way to implement the general rotor exponential
and logarithm, rotor normalization (normalize each SU(2) factor), and SLERP.
`Mat44::GetRotor()` (`Mat44.inl:673`) already uses this isomorphism and is a good reference.

## Current state by module

### Math (`Jolt/Math/`) — scaffolding solid, dynamics core now fixed & tested
Lane4/Vec4/Vec8/Mat44/RMat44/Float8/Double4 are solid. The Cl(4,0) rotor core is now correct
and covered by the reference-validated suite (37/37): the SU(2)×SU(2) split drives
`Rotor::Normalized`/`IsNormalized` (defect-aware), `Rotor::sExp`/`Log` (general 4D exp/log),
and `Mat44::GetRotor` (the anti-self-dual sign bug is fixed). Rotor/Bivec are still **entirely
scalar** (no SIMD on any ISA — a v1.1 perf item, not a correctness one); Vec8 has SSE/AVX/NEON
but no RVV. `Bivec.h`/`RMat44.h` are now tracked and registered in `Jolt.cmake`.

### Geometry (`Jolt/Geometry/`) — the strongest part of the project
Verified as a genuine, careful 4D generalization (not a rename): ClosestPoint (Gram-matrix
barycentrics incl. closest-point-on-pentachoron), GJK (5-vertex simplex), EPA (tetrahedral
cells, triangle ridges, pentachoron seed), ConvexHullBuilder (4D quickhull, cell adjacency by
shared triangle, 4-volume = det/24), OrientedBox (complete 56-axis SAT for two 4-boxes),
AABox/AABox4, Ray{AABox,Sphere,Triangle,Tetrahedron}. `RayTriangle` correctly treats the 4D
ray-vs-2-simplex system as overdetermined (normal equations + residual). Known exceptions:
`Plane::sIntersectPlanes` (sign bug), `MortonCode` (still 3D bit-spread), `ClipPoly` (still 2D
polygon clipping — see conceptual gaps), EPA degenerate-cell path leaves `mNormal`
uninitialized. Both hull builders dropped upstream's coplanar-facet merging (acceptable but
numerically weaker).

### Physics — Body subsystem migrated, everything else untouched
Migrated & internally consistent: `Body/*` (15/15 files: Body, BodyCreationSettings,
BodyInterface, BodyManager, MotionProperties, MassProperties, AllowedDOFs = 4 trans + 6 rot
DOFs), plus a thin slice of Collision: `Shape.h/.cpp`, `ConvexShape.h`, `SphereShape`,
`ConvexHullShape.cpp`, `TransformedShape` (Rotor + RVec4), `RayCast/ShapeCast/CollideShape/
AABoxCast` settings structs, `ScaleHelpers.h`.

MassProperties correctly stores the 4×4 second-moment tensor `M_ij = ∫ρ xᵢxⱼ dV` with correct
Rotate/Translate/Scale and the mapping to 6 bivector moments `I_ij = M_ii + M_jj`.

`BoxShape` is now ported to 4D (tesseract, Phase B step 6, 2026-07-22): Vec4 half extent,
tesseract mass properties, 4D AABox support / supporting-face (a cubic cell), 4D ray cast.
Its 4D content is verified standalone (mass = ∏edges·density, diagonal second moments
m·Sᵢ²/12, support corners, supporting-cell selection). **Caveat:** `BoxShape.cpp` compiles
clean on its own, but the shape *base* layer (`Shape.cpp`, `ConvexShape.cpp`, `SphereShape.cpp`)
still has un-migrated errors, so no shape TU links yet — porting individual shapes is ahead of
the base-layer cleanup.

**Untouched (still 3D, does not compile):** BroadPhase (QuadTree), the other Shape classes
(Capsule, Cylinder, Mesh, HeightField, Compound, Decorated, Plane, Triangle, Tapered*…),
the entire constraint/contact solver (`Constraints/`, `ContactConstraintManager`, all
`ConstraintPart/*`), `PhysicsSystem.cpp` stepping loop, islands, DebugRenderer, SoftBody,
Character, Vehicle, Ragdoll, Hair, TestFramework/Samples/JoltViewer.

### ObjectStream — Rotor/Bivec now registered as primitives (Phase B step 5, 2026-07-22)
`Rotor` and `Bivec` are first-class ObjectStream primitives (text + binary), and
`BodyCreationSettings::mRotation/mAngularVelocity` are re-enabled as `JPH_ADD_ATTRIBUTE`s — the
TU compiles clean and round-trips again (verified by `UnitTests/ObjectStream/
RotorBivecStreamTest.cpp`). Still 3D-lossy here: `DVec4` text/binary serialization writes only
3 components (drops W — part of bug #6), as does the `DVec4` DeterminismLog operator.

### Tests — algebra suite now exists (test-first), rest still a gap
`UnitTests/Geometry/*` were genuinely rewritten for 4D (pentachoron/hypercube hull tests, 4D
GJK/EPA cases, new Ray*Tests) — good.

The Clifford-algebra core now has a reference-validated test-first suite (added 2026-07-22):
`UnitTests/Math/RotorTests.cpp`, `BivecTests.cpp`, `Vec8Tests.cpp`, `RMat44Tests.cpp`, all
checked against a brute-force Cl(4,0) implementation in `UnitTests/Math/Cl4Reference.h` (16-blade
multivectors, geometric product from first principles). These are registered in
`UnitTests/UnitTests.cmake`. **All 37 cases now pass (1564 assertions).** Writing them caught a
bug the read-only audit missed and had praised (GetRotor, #8); the P0/P1 math bugs they pinned
(#1 exp/log, #2 defect normalization, #8 GetRotor, #10 sAnd aliasing) are now all fixed and the
former EXPECTED-TO-FAIL cases are green regression tests. **Do not weaken a test to go green —
fix the code.**

Running them: the full `UnitTests` target cannot link yet (the physics library doesn't build),
so the suite is verified via a standalone runner that compiles just the header-only Math tests
against the Core/Math sources that do build. Reproduce with (from repo root, all one line):
```
g++ -O2 -std=c++17 -mavx2 -mbmi -mpopcnt -mlzcnt -mf16c -mfma -mfpmath=sse -pthread \
  -DNDEBUG -DJPH_OBJECT_STREAM -DJPH_PROFILE_ENABLED -DJPH_USE_AVX -DJPH_USE_AVX2 \
  -DJPH_USE_F16C -DJPH_USE_FMADD -DJPH_USE_LZCNT -DJPH_USE_SSE4_1 -DJPH_USE_SSE4_2 -DJPH_USE_TZCNT \
  -IUnitTests -I. UnitTests/Math/{RotorTests,BivecTests,Vec8Tests,RMat44Tests}.cpp \
  <a main defining DOCTEST_CONFIG_IMPLEMENT + Trace + RegisterDefaultAllocator> \
  Jolt/Core/{Memory,IssueReporting,Profiler,TickCounter,StringTools,Color}.cpp Jolt/Math/Vec3.cpp \
  -o math_tests && ./math_tests
```
Once the library links, delete the standalone runner and run these through the normal
`UnitTests` binary. The committed `build/UnitTests` binary predates all 4D tests; never cite it.

Still missing: `Mat44Tests`/`Vec3Tests`/`QuatTests` test removed/legacy APIs and need rewriting
or deletion (`DMat44Tests.cpp` was de-registered from cmake — its `DMat44.h` is gone); Lane4 has
no dedicated suite (it is exercised transitively through Vec4Tests); no physics-level dynamics
tests yet (Phase C).

## Verified bugs (fix before building anything on top)

**Fixed 2026-07-22 (this pass, all validated by the Math test suite): #1, #2, #8, #10, and the
Body.inl half of #9; plus the build blocker and repo hygiene (#12, mostly).** Still open:
#3, #4, #5, #6, #7, #11, and the `Mat44::sRotationXY..` half of #9.

P0 — correctness of core dynamics/geometry:
1. **[FIXED]** ~~Rotor exponential only handles simple rotations.~~ `Rotor::sExp(BivecArg)` /
   `Rotor::Log()` now implement the general 4D exp/log via the SU(2)×SU(2) split
   (`Rotor.inl`), and `Body::AddRotationStep`/`SubRotationStep` call `sExp` instead of the old
   inline single-plane formula. Validated against the brute-force reference for random double
   rotations (`TestRotorExpLog`).
2. **[FIXED]** ~~Rotor normalization ignores the e1234 defect.~~ `Normalized()`/`IsNormalized()`
   now project onto Spin(4) by normalizing each SU(2) quaternion factor (removes the e1234
   defect), and `LERP` normalizes its result (`Rotor.inl`). Tracked by
   `TestRotorIsNormalizedDetectsDefect` / `TestRotorNormalizedRemovesDefect` /
   `TestRotorLERPReturnsValidRotor`.
3. **`MotionProperties::MultiplyWorldSpaceInverseInertiaByVector` is a stub**
   (`MotionProperties.inl:61-110`): computes the world→body rotor then discards it
   (`(void)combined`), applying the diagonal inertia as if the body were axis-aligned. Every
   constraint impulse and buoyancy call goes through this. Same missing adjoint in the
   gyroscopic path (`MotionProperties.inl:130`).
4. **`Plane::sIntersectPlanes` sign bug** (`Plane.h:56-99`): returns (−x, y, −z, w) — the
   `det = n4·sCross(n1,n2,n3)` convention is −det[n1;n2;n3;n4] and the alternating numerator
   signs don't compensate. Its own unit test (`TestPlaneIntersectPlanes`) fails once rebuilt.
   Verified by hand 2026-07-22.
5. **`MortonCode` is still 3D** (`MortonCode.h`): stride-3 masks (`0x09249249`…) with 4 axes →
   bit collisions (x bit at position 3 collides with w), plus `+` instead of `|` propagates
   carries. Needs an 8-bit stride-4 spread (masks ending `0x11111111`). Degrades BVH quality
   silently.
6. **`DVec4` still enforces the 3D "W mirrors Z" invariant**: `CheckW()` asserts Z==W under
   `JPH_FLOATING_POINT_EXCEPTIONS_ENABLED` (`DVec4.inl:113`), 3-arg ctor duplicates Z into W,
   `sAxisX/Y/Z` use the 3-arg form. Any genuine W≠Z translation (i.e. all of 4D) trips it in
   double-precision builds. Purge the invariant.
7. **EPA degenerate cell**: `EPAConvexHullBuilder.h:668-713` can leave `mNormal`/`mLambda`
   uninitialized on a degenerate tetrahedron yet still link the cell; later `IsFacing` reads
   garbage.
8. **[FIXED]** ~~`Mat44::GetRotor` recovers the wrong sign for the e23/e24/e34 (and e1234)
   components.~~ Its internal `(qL,qR)` were labeled mirror to the rotor's SU(2) convention, so
   Step 5 negated the anti-self-dual components; the fix applies the qL↔qR swap (`Mat44.inl`
   GetRotor). Discovered by the new suite (the earlier read-only audit had *praised* GetRotor).
   `sRotation(GetRotor(M)) == M` now holds for simple, double and generic rotors —
   `TestRotorMatrixRoundTrip` / `TestRMat44GetRotor`.

P1 — determinism / hygiene:
9. **[PARTIAL]** C-library `sin/cos` instead of `JPH::Sin/Cos` breaks Jolt's determinism
   guarantee (`Trigonometry.h:9`). Fixed in the `Body.inl` rotation step (now routes through
   `Rotor::sExp`, which uses `JPH::Sin/Cos`). **Still open:** `Mat44::sRotationXY/XZ/XW/YZ/YW/ZW`
   (`Mat44.inl:123-168`) still call C-library `sin/cos`.
10. **[FIXED]** ~~`Bivec::sAnd` type-puns floats through `reinterpret_cast<uint32*>`.~~ Now masks
    per component via `memcpy` (`Bivec.h`). The prior version returned its first argument
    unmodified at -O2/-O3 (strict-aliasing UB). Tracked by `TestBivecSAnd`.
11. `Body::GetBodyCreationSettings` (`Body.cpp:383`) squeezes 6 bivector inertia moments into
    a diagonal Mat44 and drops the three W-plane moments — lossy round-trip.
12. **[MOSTLY FIXED]** Repo hygiene: `build/` de-committed + `/build/` added to `.gitignore`;
    `Bivec.h`/`RMat44.h`/`Tetrahedron.h`/`IndexedTetrahedron.h`/`RayTetrahedron.h` + new test
    files now tracked; `Bivec.h` registered in `Jolt.cmake`. **Still open:** the bulk of the
    Geometry/Physics/ObjectStream migration is still uncommitted working-tree changes — commit
    in reviewable slices.

## Things that do NOT trivially generalize (the real design work left)

1. **Contact manifolds.** In 3D the contact patch between convex bodies is a 2D polygon; Jolt
   clips face polygons (`ClipPoly`) and reduces to ≤4 points. In 4D the patch is a 3D
   *polyhedron* living in the contact hyperplane. `ClipPoly.h` still clips 2D polygons.
   Needed: supporting-face extraction returning a polyhedron (or point cloud), convex
   polyhedron-vs-polyhedron clipping (Sutherland–Hodgman generalized to halfspace-clipping a
   polyhedron), and manifold reduction to ~5–6 points whose hull spans the patch (4 points
   can't stabilize a 3D patch; upstream's 4-point heuristic must be rethought, incl.
   `ManifoldBetweenTwoFaces`). This is the hardest open problem in the project.
2. **Angular constraint math.** 4D has 6 rotational DOFs. A "hinge axis" is a bivector plane;
   the effective-mass matrix of an angular constraint part is up to 6×6; the inertia tensor is
   properly a symmetric operator on bivectors (6×6), of which the current
   diagonal-6-in-principal-frame + rotor conjugation is the special case. Every
   `ConstraintPart/*` taking `Vec3 inAxis` needs redesign (Jacobians in bivector components),
   not a type swap. Decide early: keep diagonal+adjoint or store the full 6×6 world-space
   inverse inertia per step (upstream stores world-space `Mat44`; the analog is a 6×6).
3. **Shape zoo semantics.** Sphere→3-sphere (done; hypervolume π²r⁴/2), Box→tesseract
   (mechanical), Capsule→sphere-swept segment (fine). But: Cylinder/TaperedCylinder (which
   2-plane × which cross-section?), Triangle/Mesh shapes (surface meshes become *tetrahedral*
   boundary meshes of S³-like boundaries — `ConvexShape.h:135`, `Shape.h:347`; the whole
   `GetTrianglesContext`/DebugRenderer pipeline assumes triangles), HeightField (2D grid → 3D
   grid of heights over a hyperplane — memory grows cubically). Recommendation: v1 ships
   Sphere, Box, Capsule, ConvexHull, Plane, Compound/Decorated wrappers; cut Mesh/HeightField/
   Cylinder/Triangle from the build until needed.
4. **Inside/outside & winding tests.** Parity ray-tests and winding conventions were audited
   as 3D-invalid in spots (`Shape.cpp:319` TODO). Check every "odd number of hits = inside".
5. **Visualization.** DebugRenderer, TestFramework, Samples, JoltViewer are inherently 3D
   renderers. For the engine to *ship* you don't need them — you need the headless library +
   UnitTests + PerformanceTest. Recommend: make DebugRenderer compile via projection/slice
   stubs (or `JPH_DEBUG_RENDERER` off), and treat real 4D visualization (slice w=const, or
   perspective projection to 3D) as a separate downstream project.
6. **Scope cuts for v1.** SoftBody, Character, Vehicle, Ragdoll, Hair are large, deeply
   3D-coupled (Vehicle especially — wheels are inherently 3D), and none are needed to call the
   core engine "working". Exclude them from `Jolt.cmake` for v1 rather than porting.

## Roadmap to a shippable v1

Definition of "shipped v1": headless library builds warning-clean; UnitTests all green
(including new Rotor/Bivec/dynamics conservation tests); a 4D HelloWorld (tesseract falling
onto a hyperplane floor, stacks of tesseracts stable, spheres bouncing) runs deterministically;
PerformanceTest runs on a 4D scene.

**Phase A — stabilize the foundation. Steps 1, 2, 4 DONE (2026-07-22); step 3 mostly done.**
1. DONE: repo hygiene — `build/` de-committed + gitignored, untracked 4D leaf sources tracked,
   `Bivec.h` registered in `Jolt.cmake`. Still open: commit the bulk migration in reviewable
   slices (the working tree still has ~140 modified files).
2. DONE: reference-validated algebra tests written test-first (RotorTests/BivecTests/Vec8Tests/
   RMat44Tests + `Cl4Reference.h`), registered in `UnitTests.cmake`; `DMat44Tests` de-registered.
   Still TODO here: rewrite `Mat44Tests` for the new API, delete/rewrite `Vec3Tests`/`QuatTests`
   (with the legacy types), add a dedicated Lane4 suite.
3. Math P0/P1 bugs fixed against those tests (all 37 cases green): #1 exp/log, #2 defect
   normalization, #8 GetRotor, #10 sAnd, and the Body.inl half of #9. **Still open:** #3
   (`MultiplyWorldSpaceInverseInertiaByVector` adjoint), #4 (`Plane::sIntersectPlanes` — covered
   by existing `TestPlaneIntersectPlanes`), #5 (`MortonCode` — needs a new no-bit-collision test),
   #6 (`DVec4` Z==W invariant), #7 (EPA degenerate cell), #11 (lossy inertia round-trip), and the
   `Mat44::sRotationXY..` half of #9.
4. DONE: build blocker fixed — `AABox.h` now includes `RMat44.h`; error count ~3,800 → ~3,400
   and the remainder is all Phase B un-migrated-layer work.

**Phase B — restore an end-to-end simulating core:**
5. DONE (2026-07-22): ObjectStream primitives for Rotor/Bivec (text + binary), excluded
   `BodyCreationSettings` attributes re-enabled, DeterminismLog operators for Rotor/Bivec added.
   Verified by `RotorBivecStreamTest`. Remaining nit: `DVec4` serialization still drops W (bug #6).
6. Shape layer for the v1 shape set. IN PROGRESS (2026-07-22):
   - DONE: `BoxShape` → tesseract (mass properties, 4D support fn + cubic-cell supporting face,
     4D ray cast), mirroring `SphereShape`. Both convex *leaf* shapes now compile clean with the
     renderer off (drop the 3D `GetTrianglesContext.h` include, stub GetTriangles/Draw/SoftBody,
     build diagonal inertia without the removed `Mat44::sScale`). Bugs fixed while porting:
     `AABox::GetSupportingFace` sign (picked the wrong cell), `TransformedShape::ToRotor` typo;
     `Shape::GetSubmergedVolume` inBaseOffset made unconditional for renderer-off consistency.
   - **KEY FINDING — the real blocker is the convex-collision core, not a "base cleanup".** The
     shape system is tightly coupled: `Shape.cpp` pulls in every shape header (Scaled/Compound/
     Decorated/…), which are abstract until their signatures match the migrated base; and
     `ConvexShape.cpp` registers `sCollideConvexVsConvex` / `sCastConvexVsConvex` into
     `CollisionDispatch` (still `Vec3`/`Mat44` typedefs) and its collide/cast bodies ARE the
     contact pipeline. So linking any shape ⇒ migrating `CollisionDispatch` (small, ~200 lines,
     mechanical) **and** the convex collide/cast core, which overlaps step 8 (contact manifolds,
     the hardest open problem). Sequence: migrate `CollisionDispatch` signatures → migrate the
     convex collide/cast core (this is really step 8) → then the wrapper/other shape headers so
     `Shape.cpp` compiles → then link + test.
   - TODO: Capsule, Plane, Compound/Decorated/Scaled/RotatedTranslated/OffsetCOM wrappers; stub or
     cmake-exclude Mesh/HeightField/Cylinder/Tapered*/Triangle/SoftBodyShape; a permanent BoxShape
     mass-properties unit test (needs the physics test framework, currently unlinkable).
7. BroadPhase: QuadTree AABB logic → 4D AABox (mostly mechanical; AABox4 SIMD already done).
8. Contact manifold pipeline (conceptual gap 1): supporting-face polyhedra, halfspace clipping,
   ≥5-point manifold reduction, `ContactConstraintManager`.
9. Constraint parts (conceptual gap 2): contact normal + friction first (friction in 4D has a
   3D tangent space → 3 friction directions or a tangent-space solve), then
   Point/Distance/Axis parts; defer Hinge/Slider/SixDOF/etc. redesign to v1.1 unless needed.
10. `PhysicsSystem.cpp` stepping loop, islands, sleeping (mostly mechanical renames onto the
    migrated Body API).

**Phase C — validate & ship:**
11. Physics-level tests: momentum/energy/angular-momentum (bivector L) conservation, 4D
    Dzhanibekov analog for the gyroscopic integrator, restitution/friction, stack stability,
    determinism (same-input hash via DeterminismLog), double-precision build, PerformanceTest
    scene.
12. Port HelloWorld to 4D as the reference sample. Decide DebugRenderer story (slice/projection
    or off).
13. Only after v1: SIMD for Rotor/Bivec (profile first), NEON/RVV parity for Vec8, coplanar
    facet merging in the hull builders, the cut modules, real 4D viewer.

## Working conventions for this repo

- **Never trust, always test the algebra.** Every hand-derived Clifford formula (product
  tables, exp/log, adjoints, inertia maps) must have a unit test against a brute-force
  reference implementation before it is used elsewhere. This rule exists because the project
  accumulated an untested algebra core and three verified sign/convention bugs.
- Follow the type table above; never introduce new uses of `Vec3`/`Quat`.
- Use `JPH::Sin/Cos/Tan/ASin/...` (never `std::`/C-library trig) in simulation code —
  determinism.
- Keep upstream file layout and naming so `git diff master` stays reviewable; when a 3D
  concept is intentionally renamed/removed, note it in the header comment as `TODO(4D):` or
  a short "replaces X" line.
- Commit in small reviewable slices; never commit `build/` output; keep new files registered
  in `Jolt.cmake`/`UnitTests.cmake` at creation time.
- Auto-memory files (`project_4d_*.md`) hold the derived Cl(4,0) formulas and progress
  snapshots; update them when module status changes materially.

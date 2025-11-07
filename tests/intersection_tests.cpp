/**
 * @file intersection_tests.cpp
 * @author CS171 TA
 * @brief Unit tests for TriangleIntersect and AABB::intersect
 * @version 0.2
 * @date 2025-11-07
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "config_template.h"
#include "rdr/accel.h"
#include "rdr/interaction.h"
#include "rdr/rdr.h"
#include "rdr/shape.h"

using namespace RDR_NAMESPACE_NAME;

namespace {
constexpr Float kEps     = static_cast<Float>(1e-6);
constexpr Float kLooseEp = static_cast<Float>(1e-4);

inline Vec3f Unit(const Vec3f &v) {
  return Normalize(v);
}

inline void ExpectNear3(const Vec3f &a, const Vec3f &b, Float eps = kLooseEp) {
  EXPECT_NEAR(a.x, b.x, eps);
  EXPECT_NEAR(a.y, b.y, eps);
  EXPECT_NEAR(a.z, b.z, eps);
}
}  // namespace

// ------------------------ AABB::intersect ------------------------

TEST(AABB, AxisAligned_EnterExit_PositiveAndNegativeDirs) {
  AABB box(Vec3f(-1, -1, -1), Vec3f(1, 1, 1));
  Float t_in = 0, t_out = 0;

  // +X
  {
    Ray r(Vec3f(-2, 0, 0), Vec3f(1, 0, 0));
    ASSERT_TRUE(box.intersect(r, &t_in, &t_out));
    EXPECT_NEAR(t_in, 1.0, kEps);
    EXPECT_NEAR(t_out, 3.0, kEps);
  }

  // -X
  {
    Ray r(Vec3f(2, 0, 0), Vec3f(-1, 0, 0));
    ASSERT_TRUE(box.intersect(r, &t_in, &t_out));
    EXPECT_NEAR(t_in, 1.0, kEps);
    EXPECT_NEAR(t_out, 3.0, kEps);
  }

  // +Y
  {
    Ray r(Vec3f(0, -2, 0), Vec3f(0, 1, 0));
    ASSERT_TRUE(box.intersect(r, &t_in, &t_out));
    EXPECT_NEAR(t_in, 1.0, kEps);
    EXPECT_NEAR(t_out, 3.0, kEps);
  }

  // -Z
  {
    Ray r(Vec3f(0, 0, 2), Vec3f(0, 0, -1));
    ASSERT_TRUE(box.intersect(r, &t_in, &t_out));
    EXPECT_NEAR(t_in, 1.0, kEps);
    EXPECT_NEAR(t_out, 3.0, kEps);
  }
}

// ------------------------ TriangleIntersect ------------------------

TEST(TriangleIntersect, Basic) {
  auto mesh       = Memory::alloc<TriangleMeshResource>();
  mesh->vertices  = {Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, 1, 0)};
  mesh->v_indices = {0, 1, 2};

  // From below (+z)
  {
    Ray r(Vec3f(0.2, 0.3, -1.0), Vec3f(0, 0, 1));
    SurfaceInteraction si;
    EXPECT_TRUE(TriangleIntersect(r, 0, mesh, si));
    ExpectNear3(si.p, Vec3f(0.2, 0.3, 0.0));
  }

  // From above (-z)
  {
    Ray r(Vec3f(0.2, 0.3, 1.0), Vec3f(0, 0, -1));
    SurfaceInteraction si;
    EXPECT_TRUE(TriangleIntersect(r, 0, mesh, si));
    ExpectNear3(si.p, Vec3f(0.2, 0.3, 0.0));
  }
}

TEST(TriangleIntersect, MissCases) {
  auto mesh       = Memory::alloc<TriangleMeshResource>();
  mesh->vertices  = {Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, 1, 0)};
  mesh->v_indices = {0, 1, 2};

  // Hit plane but outside triangle
  {
    Ray r(Vec3f(2.0, 2.0, -1.0), Vec3f(0, 0, 1));
    SurfaceInteraction si;
    EXPECT_FALSE(TriangleIntersect(r, 0, mesh, si));
  }

  // Parallel to triangle plane
  {
    Ray r(Vec3f(0.2, 0.3, -1.0), Vec3f(1, 0, 0));
    SurfaceInteraction si;
    EXPECT_FALSE(TriangleIntersect(r, 0, mesh, si));
  }
}

TEST(TriangleIntersect, TimeWindow_RejectionAndClamping) {
  auto mesh       = Memory::alloc<TriangleMeshResource>();
  mesh->vertices  = {Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, 1, 0)};
  mesh->v_indices = {0, 1, 2};

  // Intersection at t=1; choose window that excludes it
  {
    Ray r(Vec3f(0.2, 0.3, -1.0), Vec3f(0, 0, 1));
    r.setTimeRange(1.1, 2.0);
    SurfaceInteraction si;
    EXPECT_FALSE(TriangleIntersect(r, 0, mesh, si));
  }

  // Starting inside plane (on it) moving away -> often treated as hit at t=0 if
  // direction is into plane
  {
    Ray r(Vec3f(0.2, 0.3, 0.0), Vec3f(0, 0, 1));
    r.setTimeRange(0.0, 2.0);
    SurfaceInteraction si;
    // Depending on implementation, this may be counted as a grazing hit or
    // rejected. We only assert that, if reported as a hit, the point lies on
    // the triangle.
    bool hit = TriangleIntersect(r, 0, mesh, si);
    if (hit) {
      ExpectNear3(si.p, Vec3f(0.2, 0.3, 0.0));
    }
  }
}

TEST(TriangleIntersect, TriangleInXZPlane_Hit) {
  // Triangle in the XZ plane (y=0)
  auto mesh       = Memory::alloc<TriangleMeshResource>();
  mesh->vertices  = {Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, 0, 1)};
  mesh->v_indices = {0, 1, 2};

  // Ray rising in +Y direction to pierce the triangle
  Ray r(Vec3f(0.2, -1.0, 0.3), Vec3f(0, 1, 0));
  SurfaceInteraction si;

  ASSERT_TRUE(TriangleIntersect(r, 0, mesh, si));
  ExpectNear3(si.p, Vec3f(0.2, 0.0, 0.3));
}

TEST(TriangleIntersect, DegenerateTriangle_ReturnsFalse) {
  // Collinear vertices -> area = 0
  auto mesh       = Memory::alloc<TriangleMeshResource>();
  mesh->vertices  = {Vec3f(0, 0, 0), Vec3f(0.5, 0.5, 0.0), Vec3f(1, 1, 0.0)};
  mesh->v_indices = {0, 1, 2};

  Ray r(Vec3f(0.25, 0.25, -1.0), Vec3f(0, 0, 1));
  SurfaceInteraction si;
  EXPECT_FALSE(TriangleIntersect(r, 0, mesh, si));
}

TEST(TriangleIntersect, MultiTriangleMesh_HitCorrectTriangle) {
  // Two triangles forming a square in XY: tri0 = (0,0)-(1,0)-(0,1), tri1 =
  // (1,1)-(1,0)-(0,1)
  auto mesh      = Memory::alloc<TriangleMeshResource>();
  mesh->vertices = {
      Vec3f(0, 0, 0),  // 0
      Vec3f(1, 0, 0),  // 1
      Vec3f(0, 1, 0),  // 2
      Vec3f(1, 1, 0)   // 3
  };
  // tri0: (0,1,2), tri1: (3,1,2)
  mesh->v_indices = {0, 1, 2, 3, 1, 2};

  // Hit a point that belongs only to tri1 (upper-right half)
  {
    Ray r(Vec3f(0.8, 0.8, -1.0), Vec3f(0, 0, 1));
    SurfaceInteraction si0, si1;

    bool hit0 = TriangleIntersect(r, 0, mesh, si0);
    bool hit1 = TriangleIntersect(r, 1, mesh, si1);

    EXPECT_FALSE(hit0);
    EXPECT_TRUE(hit1);
    if (hit1) {
      ExpectNear3(si1.p, Vec3f(0.8, 0.8, 0.0));
    }
  }
}

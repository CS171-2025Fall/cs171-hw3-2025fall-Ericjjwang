#include "rdr/accel.h"

#include "rdr/canary.h"
#include "rdr/interaction.h"
#include "rdr/math_aliases.h"
#include "rdr/platform.h"
#include "rdr/shape.h"
#include <limits>

RDR_NAMESPACE_BEGIN

/* ===================================================================== *
 *
 * AABB Implementations
 *
 * ===================================================================== */

bool AABB::isOverlap(const AABB &other) const {
  return ((other.low_bnd[0] >= this->low_bnd[0] &&
              other.low_bnd[0] <= this->upper_bnd[0]) ||
             (this->low_bnd[0] >= other.low_bnd[0] &&
                 this->low_bnd[0] <= other.upper_bnd[0])) &&
         ((other.low_bnd[1] >= this->low_bnd[1] &&
              other.low_bnd[1] <= this->upper_bnd[1]) ||
             (this->low_bnd[1] >= other.low_bnd[1] &&
                 this->low_bnd[1] <= other.upper_bnd[1])) &&
         ((other.low_bnd[2] >= this->low_bnd[2] &&
              other.low_bnd[2] <= this->upper_bnd[2]) ||
             (this->low_bnd[2] >= other.low_bnd[2] &&
                 this->low_bnd[2] <= other.upper_bnd[2]));
}

bool AABB::intersect(const Ray &ray, Float *t_in, Float *t_out) const {
  // TODO(HW3): implement ray intersection with AABB.
  // ray distance for two intersection points are returned by pointers.
  //
  // This method should modify t_in and t_out as the "time"
  // when the ray enters and exits the AABB respectively.
  //
  // And return true if there is an intersection, false otherwise.
  //
  // Useful Functions:
  // @see Ray::safe_inverse_direction
  //    for getting the inverse direction of the ray.
  // @see Min/Max/ReduceMin/ReduceMax
  //    for vector min/max operations.
  // 手动计算安全的倒数方向
  Vec3f inv_dir;
  for (int i = 0; i < 3; i++) {
    if (ray.direction[i] != 0) {
      inv_dir[i] = 1.0f / ray.direction[i];
    } else {
      inv_dir[i] = std::numeric_limits<Float>::max();
    }
  }
  
  // 计算每个轴上的进入和退出时间
  Vec3f t1 = (low_bnd - ray.origin) * inv_dir;
  Vec3f t2 = (upper_bnd - ray.origin) * inv_dir;
  
  // 对每个轴，确保t_min <= t_max
  Vec3f t_min = Min(t1, t2);
  Vec3f t_max = Max(t1, t2);
  
  // 找到最大的进入时间和最小的退出时间
  Float t_enter = ReduceMax(t_min);
  Float t_exit = ReduceMin(t_max);
  
  // 检查是否相交
  if (t_enter > t_exit || t_exit < ray.t_min || t_enter > ray.t_max) {
    return false;
  }
  
  // 设置输出参数
  *t_in = Max(t_enter, ray.t_min);
  *t_out = Min(t_exit, ray.t_max);
  
  return *t_in <= *t_out;
}

/* ===================================================================== *
 *
 * Accelerator Implementations
 *
 * ===================================================================== */

bool TriangleIntersect(Ray &ray, const uint32_t &triangle_index,
    const ref<TriangleMeshResource> &mesh, SurfaceInteraction &interaction) {
  using InternalScalarType = Double;
  using InternalVecType    = Vec<InternalScalarType, 3>;

  AssertAllValid(ray.direction, ray.origin);
  AssertAllNormalized(ray.direction);

  const auto &vertices = mesh->vertices;
  const Vec3u v_idx(&mesh->v_indices[3 * triangle_index]);
  assert(v_idx.x < mesh->vertices.size());
  assert(v_idx.y < mesh->vertices.size());
  assert(v_idx.z < mesh->vertices.size());

  InternalVecType dir = Cast<InternalScalarType>(ray.direction);
  InternalVecType v0  = Cast<InternalScalarType>(vertices[v_idx[0]]);
  InternalVecType v1  = Cast<InternalScalarType>(vertices[v_idx[1]]);
  InternalVecType v2  = Cast<InternalScalarType>(vertices[v_idx[2]]);

  // TODO(HW3): implement ray-triangle intersection test.
  // You should compute the u, v, t as InternalScalarType
  //
  //   InternalScalarType u = ...;
  //   InternalScalarType v = ...;
  //   InternalScalarType t = ...;
  //
  // And exit early with `return false` if there is no intersection.
  //
  // The intersection points is denoted as:
  // (1 - u - v) * v0 + u * v1 + v * v2 == ray.origin + t * ray.direction
  // where the left side is the barycentric interpolation of the triangle
  // vertices, and the right side is the parametric equation of the ray.
  //
  // You should also make sure that:
  // u >= 0, v >= 0, u + v <= 1, and, ray.t_min <= t <= ray.t_max
  //
  // Useful Functions:
  // You can use @see Cross and @see Dot for determinant calculations.

  // Delete the following lines after you implement the function
  // Möller–Trumbore 算法实现
  InternalVecType edge1 = v1 - v0;
  InternalVecType edge2 = v2 - v0;
  InternalVecType pvec = Cross(dir, edge2);
  
  InternalScalarType det = Dot(edge1, pvec);
  
  // 如果行列式接近0，光线与三角形平行
  if (std::abs(det) < 1e-8) {
    return false;
  }
  
  InternalScalarType inv_det = InternalScalarType(1) / det;
  
  InternalVecType tvec = Cast<InternalScalarType>(ray.origin) - v0;
  InternalScalarType u = Dot(tvec, pvec) * inv_det;
  
  // u坐标超出三角形范围
  if (u < 0 || u > 1) {
    return false;
  }
  
  InternalVecType qvec = Cross(tvec, edge1);
  InternalScalarType v = Dot(dir, qvec) * inv_det;
  
  // v坐标超出三角形范围或u+v>1
  if (v < 0 || u + v > 1) {
    return false;
  }
  
  InternalScalarType t = Dot(edge2, qvec) * inv_det;
  
  // 检查t是否在光线有效范围内
  if (t < ray.t_min || t > ray.t_max) {
    return false;
  }

  // 设置交互信息
  CalculateTriangleDifferentials(interaction,
      {static_cast<Float>(1 - u - v), static_cast<Float>(u),
          static_cast<Float>(v)},
      mesh, triangle_index);
  AssertNear(interaction.p, ray(t));
  assert(ray.withinTimeRange(t));
  ray.setTimeMax(t);
  return true;
}

void Accel::setTriangleMesh(const ref<TriangleMeshResource> &mesh) {
  // Build the bounding box
  AABB bound(Vec3f(Float_INF, Float_INF, Float_INF),
      Vec3f(Float_MINUS_INF, Float_MINUS_INF, Float_MINUS_INF));
  for (auto &vertex : mesh->vertices) {
    bound.low_bnd   = Min(bound.low_bnd, vertex);
    bound.upper_bnd = Max(bound.upper_bnd, vertex);
  }

  this->mesh  = mesh;   // set the pointer
  this->bound = bound;  // set the bounding box
}

void Accel::build() {}

AABB Accel::getBound() const {
  return bound;
}

bool Accel::intersect(Ray &ray, SurfaceInteraction &interaction) const {
  bool success = false;
  for (int i = 0; i < mesh->v_indices.size() / 3; i++)
    success |= TriangleIntersect(ray, i, mesh, interaction);
  return success;
}

RDR_NAMESPACE_END

/**
 * @file bvh_tests.cpp
 * @brief Simple tests for BVH implementation
 * @version 0.1
 */

#include <gtest/gtest.h>

#include "rdr/accel.h"
#include "rdr/bvh_tree.h"
#include "rdr/ray.h"
#include "rdr/rdr.h"

using namespace RDR_NAMESPACE_NAME;

// Simple test object for BVH
class TestObject {
public:
  TestObject(const Vec3f &center, Float size) : center_(center), size_(size) {
    // Create AABB for the object
    Vec3f half_size(size_ / 2, size_ / 2, size_ / 2);
    aabb_ = AABB(center - half_size, center + half_size);
  }

  AABB getAABB() const { return aabb_; }

  Vec3f getCenter() const { return center_; }

  Float getSize() const { return size_; }

private:
  Vec3f center_;
  Float size_;
  AABB aabb_;
};

// Wrapper class to make TestObject compatible with BVHTree interface
class TestNode : public BVHNodeInterface<TestObject> {
public:
  TestNode(const TestObject &obj) : data_(obj) {}

  AABB getAABB() const override { return data_.getAABB(); }

  const TestObject &getData() const override { return data_; }

private:
  TestObject data_;
};

TEST(BVH, BasicConstruction) {
  BVHTree<TestNode> bvh_tree;

  // Add some test objects to the tree
  bvh_tree.push_back(TestNode(TestObject(Vec3f(0, 0, 0), 1.0f)));
  bvh_tree.push_back(TestNode(TestObject(Vec3f(2, 0, 0), 1.0f)));
  bvh_tree.push_back(TestNode(TestObject(Vec3f(0, 2, 0), 1.0f)));
  bvh_tree.push_back(TestNode(TestObject(Vec3f(0, 0, 2), 1.0f)));

  // Build the BVH
  bvh_tree.build();

  // Check that the tree has been built
  ASSERT_GT(bvh_tree.size(), 0);

  // Test intersection with a ray
  Ray test_ray(Vec3f(-1, 0, 0), Vec3f(1, 0, 0));  // Ray from left to right

  bool hit_occurred = false;
  int hit_count     = 0;

  // Use the intersect function with a simple callback
  bvh_tree.intersect(test_ray, [&](const Ray &ray, const TestObject &obj) {
    hit_count++;
    // Check if the ray intersects with the object
    AABB obj_aabb = obj.getAABB();
    Float t_in, t_out;
    if (obj_aabb.intersect(ray, &t_in, &t_out)) {
      hit_occurred = true;
    }
    return false;  // Continue checking other nodes
  });

  // We expect at least one intersection since the ray should hit the object at
  // (0,0,0)
  EXPECT_TRUE(hit_occurred);
  EXPECT_GT(hit_count, 0);
}

TEST(BVH, SingleObject) {
  BVHTree<TestNode> bvh_tree;

  // Add a single test object
  bvh_tree.push_back(TestNode(TestObject(Vec3f(0, 0, 0), 1.0f)));

  // Build the BVH
  bvh_tree.build();

  // Verify tree has been built
  ASSERT_EQ(bvh_tree.size(), 1);

  // Test intersection
  Ray test_ray(Vec3f(-2, 0, 0), Vec3f(1, 0, 0));  // Ray approaching the object

  bool intersected = false;
  bvh_tree.intersect(test_ray, [&](const Ray &ray, const TestObject &obj) {
    AABB obj_aabb = obj.getAABB();
    Float t_in, t_out;
    if (obj_aabb.intersect(ray, &t_in, &t_out) && t_in >= 0) {
      intersected = true;
    }
    return false;
  });

  EXPECT_TRUE(intersected);
}

TEST(BVH, EmptyTree) {
  BVHTree<TestNode> bvh_tree;

  // Test intersecting on an empty tree (should not crash)
  Ray test_ray(Vec3f(0, 0, 0), Vec3f(1, 0, 0));

  bool callback_called = false;
  bool result =
      bvh_tree.intersect(test_ray, [&](const Ray &ray, const TestObject &obj) {
        callback_called = true;
        return false;
      });

  // Should not have hit anything
  EXPECT_FALSE(result);
  EXPECT_FALSE(callback_called);
}

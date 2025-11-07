/**
 * @file sdtree.h
 * @author ShanghaiTech CS171 TAs
 * @brief An epic game between Decision Tree(likely) and Gaussian Mixture Model
 * (and Neural Network later).
 * @version 0.1
 * @date 2023-08-05
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __SDTREE_H__
#define __SDTREE_H__

#include <queue>

#include "rdr/accel.h"
#include "rdr/rdr.h"

RDR_NAMESPACE_BEGIN

namespace detail_ {
// see
// https://github.com/OpenPathGuidingLibrary/openpgl/blob/main/openpgl/directional/dqt/SphereToSquare.h
// a unit area on the sphere is mapped to a unit area on the square
struct SphereToSquareTransformation {
  static Vec2f directionToPoint(const Vec3f &direction) {
    // dA = sin(theta) dtheta dphi
    // |J|_{(u, v) -> (theta, phi)} = 4*pi/sin(theta)
    // then dA = 4*pi du dv
    AssertAllNormalized(direction);
    const Vec2f &spherical_direction = InverseSphericalDirection(direction);
    const Float &cos_theta           = std::cos(spherical_direction[0]);

    const Float u = (cos_theta + 1) / 2;                // [0, 1]
    const Float v = spherical_direction[1] / (2 * PI);  // [0, 1]
    AssertAllNonNegative(spherical_direction, u, v);
    return {u, v};
  }

  static Vec3f pointToDirection(const Vec2f &point) {
    const Float &u = point[0];
    const Float &v = point[1];
    AssertAllNonNegative(u, v);

    const Float cos_theta = 2 * u - 1;
    const Float sin_theta = sqrt(1 - cos_theta * cos_theta);
    const Float phi       = 2 * PI * v;  // [0, 2*pi]
    AssertAllValid(cos_theta, sin_theta, phi);

    return SphericalDirection(sin_theta, cos_theta, phi);
  }

  /// dA = 4*pi du dv
  static Float jacobian(const Vec2f &) { return 4 * PI; }
};
}  // namespace detail_

/// An implementation of light-weight directional quad tree
/// @see
/// https://jannovak.info/publications/PathGuide/PathGuide_supplementary.pdf
class DirectionalQuadTree final {
public:
  using IndexType          = uint64_t;
  using TransformationType = detail_::SphereToSquareTransformation;

  constexpr static Float SPLIT_RATIO       = 0.001;
  constexpr static IndexType INVALID_INDEX = 0;
  constexpr static int MAX_DEPTH           = 12;
  Float filter_power{2.2};

  friend class GuidedPathIntegrator;

  struct QuadNode {
    bool is_leaf{false};
    Float weight{0.0};
    TAABB<Vec2f> bound{};

    Vec3f flux{0.0};

    // All invalid
    std::array<IndexType, 4> children{
        INVALID_INDEX, INVALID_INDEX, INVALID_INDEX, INVALID_INDEX};
  };

  // The rule of five
  ~DirectionalQuadTree() = default;
  DirectionalQuadTree(std::pmr::memory_resource *upstream)
      : root_index(INVALID_INDEX), upstream(upstream), nodes(upstream) {
    root_index = nodes.size();
    nodes.push_back(QuadNode{
        true, EPS /* boostrap weight */, TAABB<Vec2f>(Vec2f{0.0}, Vec2f{1.0})});
    syncSamples();
  }

  DirectionalQuadTree(const DirectionalQuadTree &other)
      : root_index(other.root_index),
        upstream(other.upstream),
        nodes(other.nodes, upstream) {}

  DirectionalQuadTree(DirectionalQuadTree &&other) noexcept
      : DirectionalQuadTree(other.upstream) {
    swap(other);
  }

  DirectionalQuadTree &operator=(const DirectionalQuadTree &other) {
    // Use the tmp-and-move idiom
    DirectionalQuadTree tmp(other);
    *this = std::move(tmp);
    return *this;
  }

  DirectionalQuadTree &operator=(DirectionalQuadTree &&other) noexcept {
    swap(other);
    return *this;
  }

  uint32_t size() const { return nodes.size(); }

  Float getSumWeight() const { return nodes[root_index].weight; }

  /// Commit a world direction to the quad tree. Notice that without calling
  /// syncSamples(), the quad tree's structure is not guaranteed to be ready for
  /// sampling.
  void commitWorldDirection(const Vec3f &direction, const Vec3f &weight) {
    const Vec2f &point = TransformationType::directionToPoint(direction);
    AssertAllNormalized(direction);
    commitLocalPoint(root_index, point, weight);
  }

  /// Sync the samples in the quad tree, i.e., rebuild the tree with the new
  /// weights
  void syncSamples(Float split_ratio = SPLIT_RATIO) {
    splitOrPruneNode(root_index, split_ratio, 0);
  }

  /// Sample a world direction from the quad tree and write the pdf
  Vec3f sampleWorldDirection(Sampler &sampler, Float &pdf) const {
    Float pdf_local          = NAN;
    IndexType discrete_index = 0;
    const Vec2f &point =
        sampleLocalPoint(root_index, sampler, pdf_local, discrete_index, 0);
    pdf = pdf_local / TransformationType::jacobian(point);
    return TransformationType::pointToDirection(point);
  }

  /// Evaluate the PDF given a world direction
  float pdfWorldDirection(const Vec3f &direction) const {
    const Vec2f &point = TransformationType::directionToPoint(direction);
    return pdfLocalPoint(point) / TransformationType::jacobian(point);
  }

  /// Temporary function for debugging and testing
  /// TODO: delete this function or mark friend
  Vec2f sampleLocalPointDebug(
      Sampler &sampler, Float &pdf, IndexType &discrete_index) const {
    return sampleLocalPoint(root_index, sampler, pdf, discrete_index, 0);
  }

  /// Visualize the quadtree to an image
  void visualizeToImage(const fs::path &path) const;

  void sanityCheck() const { sanityCheck(root_index, 0); }

private:
  IndexType root_index;
  std::pmr::memory_resource *upstream;
  std::pmr::deque<QuadNode>
      nodes;  // Temporarily use std::deque to mitigate use-after-free

  void commitLocalPoint(
      IndexType node_index, const Vec2f &point, const Vec3f &weight);
  Vec2f sampleLocalPoint(const IndexType &node_index, Sampler &sampler,
      Float &pdf, IndexType &discrete_index, int depth) const;
  float pdfLocalPoint(const Vec2f &point) const;
  void splitOrPruneNode(
      const IndexType &node_index, Float split_ratio, int depth);
  IndexType findLeaf(IndexType node_index, const Vec2f &point) const;

  void swap(DirectionalQuadTree &other) {
    std::swap(root_index, other.root_index);
    std::swap(upstream, other.upstream);
    std::swap(nodes, other.nodes);
  }

  /// Paint the quad tree to an image
  void recursiveVisualizer(
      const IndexType &node_index, Film &canvas, int depth) const;

  void sanityCheck(const IndexType &node_index, int depth) const;

  /// Determine the index of the child node that contains the point (fast)
  /// Assume that x is within bound
  static int child(const Vec2f &point, const TAABB<Vec2f> &bound) {
    int index          = 0;
    const Vec2f center = bound.getCenter();
    index |= static_cast<int>(point.x >= center.x);
    index |= (static_cast<int>(point.y >= center.y) << 1);
    return index;
  }
};

class SpatialBinaryTree final {
public:
  using IndexType          = uint64_t;
  using TransformationType = detail_::SphereToSquareTransformation;

  constexpr static IndexType INVALID_INDEX = 0;

  friend class GuidedPathIntegrator;

  // The rule of five
  SpatialBinaryTree(AABB bound, std::pmr::memory_resource *upstream);
  ~SpatialBinaryTree() = default;
  SpatialBinaryTree(const SpatialBinaryTree &other);
  SpatialBinaryTree(SpatialBinaryTree &&other) noexcept
      : SpatialBinaryTree(other.root_bound, other.upstream) {
    swap(other);
  }

  SpatialBinaryTree &operator=(const SpatialBinaryTree &other) {
    // Use the tmp-and-move idiom
    SpatialBinaryTree tmp(other);
    *this = std::move(tmp);
    return *this;
  }

  SpatialBinaryTree &operator=(SpatialBinaryTree &&other) noexcept {
    swap(other);
    return *this;
  }

  /// Reset the state of the spatial binary tree
  void clear();

  /// After committing all the samples, call this function to optimize the tree
  void optimize(uint64_t split_threshold);

  /// Commit a sample to the spatial binary tree
  void commitSample(const Vec3f &position, const Vec3f &direction,
      const Vec3f &measurement, Sampler &sampler, int thread_id = 0);

  /// Find the closest leaf node to the given position
  struct LeafNode;
  const LeafNode &findClosestLeaf(const Vec3f &position) const;

  std::size_t numLeafNodes() const { return leaf_nodes.size(); }
  std::size_t numInternalNodes() const { return internal_nodes.size(); }

private:
  /// A sample in the spatial binary tree (before commit)
  struct Sample {
    Vec3f position;
    Vec3f direction;
    Vec3f measurement;
  };

  // We split into LeafNode and InternalNode for
  // memory-efficiency since they have very different memory layout
  // (LeafNode is much more heavier)
  struct InternalNode {
    InternalNode() = default;
    InternalNode(AABB bound) : bound(bound) {}
    AABB bound;
    int split_dim{0};
    Float split_val{0};
    std::array<bool, 2> is_child_leaf{false, false};
    std::array<IndexType, 2> children{INVALID_INDEX, INVALID_INDEX};
  };

  void swap(SpatialBinaryTree &other) {
    std::swap(root_bound, other.root_bound);
    std::swap(upstream, other.upstream);
    std::swap(leaf_nodes, other.leaf_nodes);
    std::swap(internal_nodes, other.internal_nodes);
    std::swap(leaf_nodes_update_mutex, other.leaf_nodes_update_mutex);
    std::swap(internal_nodes_update_mutex, other.internal_nodes_update_mutex);
  }

  static AABB boundTransformation(const AABB &in_bound) {
    constexpr Float scale_factor = 1.2;

    // Scale by the center
    const Vec3f center = in_bound.getCenter();
    return {(in_bound.low_bnd - center) * scale_factor + center,
        (in_bound.upper_bnd - center) * scale_factor + center};
  }

  AABB root_bound;
  std::pmr::memory_resource *upstream;
  std::pmr::vector<LeafNode> leaf_nodes;
  std::pmr::vector<InternalNode> internal_nodes;

  ref<std::mutex> leaf_nodes_update_mutex;
  ref<std::mutex> internal_nodes_update_mutex;

  /// Create the initial states of the nodes
  void initNodes();

  /// Subdivide the node into 2 children
  /// @note This function is not thread-safe for now. To be refractored.
  void splitAndPushdownNode(const IndexType &leaf_node_index);

  /// Find the closest leaf node's index to the given position
  IndexType findClosestLeafIndex(const Vec3f &position) const;
};

struct SpatialBinaryTree::LeafNode {
  LeafNode(std::pmr::memory_resource *upstream)
      : bound(), mutex(make_ref<std::mutex>()), quad_tree(upstream) {}

  /// The commit mutex
  ref<std::mutex> mutex;

  /// The index of the parent node in `internal_nodes`
  IndexType parent{0};

  /// The depth of the Node
  int depth{0};

  /// The AABB of the leaf node
  AABB bound;

  /// The quad tree that stores the previous samples
  DirectionalQuadTree quad_tree;

  /// The average number of samples that are committed to this leaf node
  uint64_t num_samples{0};
  AABB sample_bound{};

  void flushSamples() {
    num_samples  = 0;
    sample_bound = AABB();
  }

  /// Thread-safe function to commit a sample to this leaf node
  void commitSample(const Sample &sample) {
    std::lock_guard<std::mutex> lock(*mutex);
    ++num_samples;
    sample_bound.unionWith(sample.position);
    quad_tree.commitWorldDirection(sample.direction, sample.measurement);
  }
};

RDR_NAMESPACE_END

#endif

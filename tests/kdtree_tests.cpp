#include <gtest/gtest.h>

#include <limits>

#include "rdr/kdtree.h"

using namespace RDR_NAMESPACE_NAME;

template <typename PointType>
PointType GenerateRandomPoint(Sampler &sampler) {
  if constexpr (std::is_same_v<PointType, Vec3f>) {
    return PointType(sampler.get1D(), sampler.get1D(), sampler.get1D());
  } else if (std::is_same_v<PointType, Vec2f>) {
    return PointType(sampler.get1D(), sampler.get1D());
  }
}

template <typename _PointType>
class KDTreeTestbed {
public:
  using PointType  = _PointType;
  using KDTreeType = KDTree<KDNode<PointType, char>, TAABB<PointType>>;
  using NodeType   = typename KDTreeType::NodeType;

  KDTreeTestbed() { tree = make_ref<KDTreeType>(); }
  ~KDTreeTestbed() = default;

  void generateAndBuild(int n_points = 16384) {
    points.clear();
    for (int i = 0; i < n_points; ++i) {
      PointType point = GenerateRandomPoint<PointType>(sampler);
      points.push_back(point);
      tree->push_back(NodeType(point, i));
    }

    tree->build();
  }

  template <typename Callback>
  void test(Callback callback) {
    callback(tree, points, sampler);
  }

public:
  ref<KDTreeType> tree;
  vector<PointType> points;

private:
  Sampler sampler;
};

#define TestALL(callback)          \
  {                                \
    KDTreeTestbed<Vec2f> testbed2; \
    testbed2.generateAndBuild();   \
    testbed2.test((callback));     \
    KDTreeTestbed<Vec3f> testbed3; \
    testbed3.generateAndBuild();   \
    testbed3.test((callback));     \
  }

TEST(KDTree, nearestNeighbor) {
  auto test_func = [](const auto &tree, const auto &points, auto &sampler) {
    int n_iter = 1024;
    while (n_iter--) {
      using PointType =
          std::remove_const_t<std::remove_reference_t<decltype(points[0])>>;

      auto point             = GenerateRandomPoint<PointType>(sampler);
      Float min_sqr_distance = std::numeric_limits<Float>::max();
      auto nearest_position =
          (*tree)[tree->nearestNeighborSearch(point, min_sqr_distance)]
              .getPosition();

      auto nearest_position_gt  = PointType();
      Float min_sqr_distance_gt = std::numeric_limits<Float>::max();
      for (int i = 0; i < points.size(); ++i) {
        if (SquareNorm(point - points[i]) < min_sqr_distance_gt) {
          nearest_position_gt = points[i];
          min_sqr_distance_gt = SquareNorm(point - points[i]);
        }
      }

      EXPECT_EQ(nearest_position, nearest_position_gt);
      EXPECT_EQ(min_sqr_distance, min_sqr_distance_gt);
    }
  };

  TestALL(test_func);
}

TEST(KDTree, fixedRadiusSearch) {
  auto test_func = [](const auto &tree, const auto &points, auto &sampler) {
    int n_iter = 1024;
    while (n_iter--) {
      using PointType =
          std::remove_const_t<std::remove_reference_t<decltype(points[0])>>;
      using NodeType = std::remove_reference_t<decltype((*tree)[0])>;

      auto point         = GenerateRandomPoint<PointType>(sampler);
      Float max_distance = 0.1;

      int num = 0, gt_num = 0;
      for (int i = 0; i < points.size(); ++i)
        if (Norm(point - points[i]) < max_distance) ++gt_num;

      tree->fixedRadiusSearch(
          point, max_distance, [&num](const NodeType &node) -> void { ++num; });
      EXPECT_EQ(num, gt_num);
    }
  };

  TestALL(test_func);
}

TEST(KDTree, kNearestNeighborSearch1) {
  auto test_func = [](const auto &tree, const auto &points, auto &sampler) {
    int n_iter = 1024;
    while (n_iter--) {
      using PointType =
          std::remove_const_t<std::remove_reference_t<decltype(points[0])>>;
      using NodeType = decltype((*tree)[0]);

      auto point             = GenerateRandomPoint<PointType>(sampler);
      Float min_sqr_distance = std::numeric_limits<Float>::max();

      PointType nearest_position;
      tree->kNearestNeighborSearch(point, 1, [&](const NodeType &node) -> void {
        nearest_position = node.getPosition();
        min_sqr_distance = SquareNorm(nearest_position - point);
      });

      auto nearest_position_gt  = PointType();
      Float min_sqr_distance_gt = std::numeric_limits<Float>::max();
      for (int i = 0; i < points.size(); ++i) {
        if (SquareNorm(point - points[i]) < min_sqr_distance_gt) {
          nearest_position_gt = points[i];
          min_sqr_distance_gt = SquareNorm(point - points[i]);
        }
      }

      EXPECT_EQ(nearest_position, nearest_position_gt);
      EXPECT_EQ(min_sqr_distance, min_sqr_distance_gt);
    }
  };

  TestALL(test_func);
}

TEST(KDTree, kNearestNeighborSearch) {
  auto test_func = [](const auto &tree, const auto &points, auto &sampler) {
    int n_iter      = 1024;
    constexpr int K = 128;

    while (n_iter--) {
      using PointType =
          std::remove_const_t<std::remove_reference_t<decltype(points[0])>>;
      using NodeType = decltype((*tree)[0]);

      auto point = GenerateRandomPoint<PointType>(sampler);
      auto comp  = [point](const auto &p1, const auto &p2) -> bool {
        return SquareNorm(point - p1) < SquareNorm(point - p2);
      };

      // not efficient implementation
      auto points_ = points;  // copy
      std::nth_element(
          points_.begin(), points_.begin() + K - 1, points_.end(), comp);
      std::sort(points_.begin(), points_.begin() + K, comp);

      int num = 0, k = K;
      tree->kNearestNeighborSearch(point, K, [&](const NodeType &node) -> void {
        EXPECT_NEAR(Norm(node.getPosition() - point), Norm(points_[--k] - point), EPS);
        ++num;
      });

      EXPECT_EQ(num, K);
    }
  };

  TestALL(test_func);
}

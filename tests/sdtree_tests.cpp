#include <gtest/gtest.h>

#include <limits>

#include "nlohmann/json.hpp"
#include "rdr/film.h"
#include "rdr/light.h"
#include "rdr/render.h"
#include "rdr/sdtree.h"
#include "rdr/texture.h"

using namespace RDR_NAMESPACE_NAME;

TEST(SDTree, DirectionalQuadTreeInit) {
  // Factory::doRegisterAllClasses();
  // DirectionalQuadTree quad_tree(std::pmr::get_default_resource());

  // const Float init_pdf = quad_tree.pdfWorldDirection(Normalize(Vec3f{1, 1,
  // 1})); EXPECT_NEAR(init_pdf, 1.0 / (4 * PI), 1e-4);
}

/*
TEST(SDTree, DirectionQuadTreeBasic) {
  Factory::doRegisterAllClasses();
  DirectionalQuadTree quad_tree(std::pmr::get_default_resource());
  EXPECT_NEAR(quad_tree.getSumWeight(), EPS, 1e-6);

  for (int i = 0; i < 12; ++i) {
    int n_samples = (1 << i);
    while (n_samples-- > 0) {
      quad_tree.commitWorldDirection(Normalize(Vec3f{1, 1, 0}), Vec3f{1.0});
      quad_tree.commitWorldDirection(Normalize(Vec3f{1, 0, 1}), Vec3f{1.0});
    }

    quad_tree.syncSamples();
  }

  // quad_tree.sanityCheck();
  InfiniteAreaLight light(Properties{});
  const std::string texture_config = R"(
    {
      "type": "image",
      "path": "/home/krr/Misc/rdr171/data/assets/venice_sunset_2k.exr",
      "tex_coordinate_generator": {
        "type": "uvmapping2d",
        "scale": [
          1.0,
          1.0
        ],
        "delta": [
          0.0,
          0.0
        ]
      }
    }
  )";
  Properties pseudo_props(nlohmann::json::parse(texture_config));
  ImageTexture texture(pseudo_props);
  light.getTexture() = &texture;

  const Vec2i resolution(128, 128);

  Film canvas          = NativeRender::prepareDebugCanvas(resolution);
  using Transformation = detail_::SphereToSquareTransformation;
  Sampler sampler;
  for (int i = 0; i < 8; ++i) {
    for (int y = 0; y < resolution.y; ++y) {
      for (int x = 0; x < resolution.x; ++x) {
        int n_samples = (1 << i);

        while (n_samples-- > 0) {
          // Generate the ray
          const Vec3f direction =
              Transformation::pointToDirection(Vec2f{x, y} / resolution);
          Ray ray(Vec3f{0.0}, Normalize(direction));

          // Accumulate the radiance
          const Vec3f L = light.Le(SurfaceInteraction{}, ray.direction);
          quad_tree.commitWorldDirection(Normalize(direction), L);
          canvas.commitSample(Vec2f{x, y}, L);

          // Reset the sampler
          sampler.resetAfterIteration();
        }
      }
    }

    quad_tree.syncSamples();
  }

  // Sample tests
  Float pdf;
  for (int i = 0; i < 128; ++i) {
    const auto &direction = quad_tree.sampleWorldDirection(sampler, pdf);
    EXPECT_NEAR(pdf, quad_tree.pdfWorldDirection(direction), 1e-2);
  }

  quad_tree.visualizeToImage("test.exr");
  canvas.exportImageToFile("test_gt.exr");
}
*/

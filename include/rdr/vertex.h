#ifndef __VERTEX_H__
#define __VERTEX_H__

#include "rdr/bsdf.h"
#include "rdr/camera.h"
#include "rdr/interaction.h"
#include "rdr/light.h"
#include "rdr/scene.h"

RDR_NAMESPACE_BEGIN

enum class EVertexType {
  Camera = 0,
  Light,
  Surface,
  ILight,  //<! Infinite Area Light
};

/**
 * A BDPT-specific vertex class.
 */
struct Vertex {
  friend class BidirectionalPathIntegrator;

  /// Basic vertex info
  /// We replicate the interaction here since interaction itself is a quite
  /// heavy class
  EVertexType type{EVertexType::Surface};
  bool isDelta{false};
  Float pdfForward{0};   // defined as PBRT's
  Float pdfReversed{0};  // defined as PBRT's
  Vec3f beta{0.0};
  Vec3f p{0.0};
  Vec3f gNormal{0.0};
  Vec3f sNormal{0.0};
  Vec2f uv{0.0};

  const Primitive *primitive{nullptr};
  const BSDF *bsdf{nullptr};
  const Light *light{nullptr};
  const Camera *camera{nullptr};

  /// Builder
  static Vertex fromLight(
      const SurfaceInteraction &interaction, const Vec3f &beta) {
    assert(interaction.isLight());
    Vertex vertex;
    vertex.type    = EVertexType::Light;
    vertex.beta    = beta;
    vertex.p       = interaction.p;
    vertex.gNormal = interaction.normal;
    vertex.sNormal = interaction.shading.n;
    vertex.initPointers(interaction);
    assert(vertex.light != nullptr);
    return vertex;
  }

  static Vertex fromILight(
      const Vec3f &ray_direction, const Vec3f &beta, const ref<Scene> &scene) {
    assert(scene->getInfiniteLight() != nullptr &&
           "Infinite Area Light is not presented while fromILight is invoked");
    ref<InfiniteAreaLight> inf_light = scene->getInfiniteLight();

    Vertex vertex;
    vertex.type = EVertexType::ILight;
    vertex.beta = beta;
    vertex.p = inf_light->getCenter() + ray_direction * inf_light->getRadius();
    vertex.gNormal = -ray_direction;
    vertex.sNormal = -ray_direction;
    vertex.light   = scene->getInfiniteLight().get();
    return vertex;
  }

  static Vertex fromCamera(
      const Camera *camera, const Ray &ray, const Vec3f &beta) {
    Vertex vertex;
    vertex.type    = EVertexType::Camera;
    vertex.beta    = beta;
    vertex.p       = camera->getPosition();  // TODO: ?
    vertex.gNormal = vertex.sNormal = Normalize(camera->forward);
    vertex.camera                   = camera;
    return vertex;
  }

  static Vertex fromSurface(
      const SurfaceInteraction &interaction, const Vec3f &beta) {
    assert(!interaction.isLight());
    Vertex vertex;
    vertex.type    = EVertexType::Surface;
    vertex.beta    = beta;
    vertex.p       = interaction.p;
    vertex.gNormal = interaction.normal;
    vertex.sNormal = interaction.shading.n;
    vertex.uv      = interaction.uv;
    vertex.initPointers(interaction);
    return vertex;
  }

  /// Query Interface
  bool isCamera() const { return type == EVertexType::Camera; }
  bool isLight() const {
    // There exist some vertices that are not light vertices but have light
    // so this is the only criterion
    return light != nullptr;
  }

  Vec3f LeTowards(const Vertex &adjacent) const {  // NOLINT
    if (light == nullptr) return Vec3f(0.0);
    const Vec3f w = Normalize(adjacent.p - p);

    Vec3f Le{0.0};  // NOLINT
    {
      // For compatibility, we generate the query interaction
      // what a bad design.
      assert(Dot(gNormal, sNormal) > 0.0);
      SurfaceInteraction interaction = generateGenericQueryInteraction();
      interaction.type               = type == EVertexType::ILight
                                         ? ESurfaceInteractionType::EInfLight
                                         : ESurfaceInteractionType::ELight;

      Le = light->Le(interaction, w);
    }

    return Le;
  }

  Vec3f bsdfEvaluate(const Vertex &wi_vertex, const Vertex &wo_vertex) const {
    if (bsdf == nullptr) return Vec3f(0.0);
    const Vec3f wi = Normalize(wi_vertex.p - p);
    const Vec3f wo = Normalize(wo_vertex.p - p);

    Vec3f f{0.0};
    {
      // Again, generate the query pseudo interaction
      SurfaceInteraction interaction = generateGenericQueryInteraction();

      // TODO: support texture and importance transport
      interaction.wi = wi;
      interaction.wo = wo;

      f = bsdf->evaluate(interaction);
    }

    return f;
  }

  Float pdfAreaSampledFromThis(const Vertex &v, const Vertex *prev) const {
    Float pdf_a{0.0};

    if (isLight()) {
      // Special case when this vertex is given on the light, we need to obtain
      // the probability of sampling this vertex in solid angle.
      // TODO: support other directional sampling strategies
      SurfaceInteraction interaction = generateGenericQueryInteraction();

      Float pdf_w = light->pdfDirection(interaction, Normalize(v.p - p));
      pdf_a       = v.pdfFromSolidAngleMeasure(pdf_w, *this);
    } else if (isCamera()) {
      // Not indended to be handeled by the vertex
      Float pdf_w = NAN;
      camera->pdf(Ray{p, Normalize(v.p - p)}, nullptr, &pdf_w);
      pdf_a = v.pdfFromSolidAngleMeasure(pdf_w, *this);
    } else {
      // generate the query pseudo interaction
      assert(prev != nullptr);
      SurfaceInteraction interaction = generateGenericQueryInteraction();

      // In path tracing, the case is given w_o, we sample the w_i
      interaction.wi = Normalize(v.p - p);
      interaction.wo = Normalize(prev->p - p);

      // Convert to area measure
      pdf_a = bsdf->pdf(interaction);
      pdf_a = v.pdfFromSolidAngleMeasure(pdf_a, *this);
    }

    return pdf_a;
  }

  // Given another(source) vertex and the pdf of sampling this vertex in solid
  // angle measure, obtain the area measure of sampling this vertex
  Float pdfFromSolidAngleMeasure(Float pdf, const Vertex &adjacent) const {
    AssertAllValid(gNormal, sNormal);
    AssertAllNormalized(gNormal, sNormal);

    const Vec3f w    = adjacent.p - p;
    const Float d_sq = SquareNorm(w);
    return pdf * abs(Dot(gNormal, Normalize(w))) / d_sq;
  }

private:
  void initPointers(const SurfaceInteraction &interaction) {
    primitive = interaction.primitive;
    bsdf      = interaction.bsdf;
    light     = interaction.light;
  }

  RDR_FORCEINLINE SurfaceInteraction generateGenericQueryInteraction() const {
    SurfaceInteraction interaction;
    interaction.setGeneral(p, gNormal);
    interaction.setUV(uv);
    interaction.setPrimitive(bsdf, light, primitive);

    // TODO: support texture
    interaction.setShading(
        sNormal, Vec3f{0.0}, Vec3f{0.0}, Vec3f{0.0}, Vec3f{0.0});
    return interaction;
  }
};

RDR_NAMESPACE_END

#endif

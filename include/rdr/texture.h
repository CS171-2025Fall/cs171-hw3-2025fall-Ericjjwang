#ifndef __TEXTURE_H__
#define __TEXTURE_H__

#include "rdr/rdr.h"

RDR_NAMESPACE_BEGIN

class TexCoordinateGenerator : public ConfigurableObject {
public:
  // ++ Required by ConfigurableObject
  TexCoordinateGenerator(const Properties &props) : ConfigurableObject(props) {}
  // --

  virtual ~TexCoordinateGenerator() = default;
  virtual Vec2f Map(const SurfaceInteraction &interaction, Vec2f &dstdx,
      Vec2f &dstdy) const           = 0;
};

class UVMapping2D final : public TexCoordinateGenerator {
public:
  // ++ Required by ConfigurableObject
  UVMapping2D(const Properties &props)
      : scale(props.getProperty<Vec2f>("scale", Vec2f(1, 1))),
        delta(props.getProperty<Vec2f>("delta", Vec2f(0, 0))),
        TexCoordinateGenerator(props) {}
  // --

//修改
  Vec2f Map(const SurfaceInteraction &interaction, Vec2f &dstdx,
      Vec2f &dstdy) const override {
      // Simple linear transformation of UVs
      dstdx = Vec2f(0, 0); // Derivatives ignored for now
      dstdy = Vec2f(0, 0);
      return interaction.uv * scale + delta;
  }


  // ++ Required by Object
  std::string toString() const override {
    return format(
        "UVMapping2D [\n"
        "  scale  = {}\n"
        "  delta  = {}\n"
        "]",
        scale, delta);
  }
  // --

private:
  const Vec2f scale, delta;
};

class Texture : public ConfigurableObject {
public:
  // ++ Required by ConfigurableObject
  Texture(const Properties &props) : ConfigurableObject(props) {}
  // --

  virtual ~Texture() = default;

  /// Evaluate the texture at the given interaction
  virtual Vec3f evaluate(const SurfaceInteraction &interaction) const = 0;
};

class ConstantTexture final : public Texture {
public:
  // ++ Required by ConfigurableObject
  ConstantTexture(const Properties &props)
      : color(props.getProperty<Vec3f>("color")), Texture(props) {}
  // --

  // ++ Required by Object
  std::string toString() const override {
    return format(
        "ConstantTexture [\n"
        "  color  = {}\n"
        "]",
        color);
  }
  // --

  /// @see Texture::evaluate
  Vec3f evaluate(const SurfaceInteraction &interaction) const override {
    return color;
  }

private:
  const Vec3f color;
};

class CheckerBoardTexture final : public Texture {
public:
  // ++ Required by ConfigurableObject
  CheckerBoardTexture(const Properties &props)
      : color0(props.getProperty<Vec3f>("color0", Vec3f(.4f))),
        color1(props.getProperty<Vec3f>("color1", Vec3f(.2f))),
        texmap(RDR_CREATE_CLASS(TexCoordinateGenerator,
            props.getProperty<Properties>("tex_coordinate_generator"))),
        Texture(props) {}
  // --

  // ++ Required by Object
  std::string toString() const override {
    return format(
        "CheckerBoardTexture [\n"
        "  color0  = {}\n"
        "  color1  = {}\n"
        "]",
        color0, color1);
  }
  // --

  /// @see Texture::evaluate
  Vec3f evaluate(const SurfaceInteraction &interaction) const override {
    Vec2f dstdx, dstdy;
    const auto &st = texmap->Map(interaction, dstdx, dstdy);

    int x = 2 * Mod(static_cast<int>(st.x * 2), 2) - 1,
        y = 2 * Mod(static_cast<int>(st.y * 2), 2) - 1;

    if (x * y == 1)
      return color0;
    else
      return color1;
  }

private:
  const Vec3f color0;
  const Vec3f color1;
  ref<TexCoordinateGenerator> texmap;
};

/**
 * @brief Texture class. This class is designed only to load and evaluate
 * texture at a given uv position, which is different from our film
 * implementation. The format of the loaded texture is:
 * - 32-bit float
 * - 4 channels (RGBA)
 * - (0, 0) corresponds to the upper-left corner
 * - (1, 0) corresponds to the right of the upper-left corner, etc.
 * - data is stored in row-major order, i.e. elements in the same row are
 *  continuous in memory
 */
class ImageTexture final : public Texture {
public:
  // ++ Required by ConfigurableObject
  ImageTexture(const Properties &props) : Texture(props) {
      std::string path = props.getProperty<std::string>("path");
      
      // Initialize Texture Mapping
      if (props.hasProperty("tex_coordinate_generator")) {
          texmap = RDR_CREATE_CLASS(TexCoordinateGenerator,
              props.getProperty<Properties>("tex_coordinate_generator"));
      } else {
          Properties default_props;
          default_props.setProperty("type", "uvmapping2d");
          texmap = Memory::alloc<UVMapping2D>(default_props);
      }

      // TODO: Integrate your image loader here (e.g., stb_image).
      // For now, we initialize a dummy 1x1 white texture to prevent crashes.
      // ---------------------------------------------------------
      // Example logic:
      // float* raw_data = stbi_loadf(path.c_str(), &width, &height, &channels, 4);
      // if (raw_data) { data.assign(raw_data, raw_data + width * height * 4); ... }
      // ---------------------------------------------------------
      
      // Dummy initialization:
      width = 1;
      height = 1;
      data = {1.0f, 1.0f, 1.0f, 1.0f}; // RGBA white
      
      // Warning if path is real but we didn't load it
      if (!path.empty()) {
          // std::cerr << "Warning: Image loading not implemented in header-only mode. Using dummy white texture for: " << path << std::endl;
      }
  }
  // --

  virtual ~ImageTexture() = default;
  const Float *getData() const { return data.data(); }

  int getWidth() const { return width; }
  int getHeight() const { return height; }

  // ++ Required by Texture
  Vec3f evaluate(const SurfaceInteraction &interaction) const override {
      if (data.empty()) return Vec3f(0.0f);

      Vec2f dstdx, dstdy;
      Vec2f uv = texmap->Map(interaction, dstdx, dstdy);

      // 1. Handle Repeat/Wrap
      float u = uv.x - std::floor(uv.x);
      float v = uv.y - std::floor(uv.y);

      // Flip V if necessary (standard OpenGL convention vs Image storage)
      v = 1.0f - v;

      // 2. Map to pixel coordinates
      // -0.5 moves the coordinate to the center of the texel
      float u_img = u * width - 0.5f;
      float v_img = v * height - 0.5f;

      // 3. Get integer coordinates (top-left)
      int x0 = static_cast<int>(std::floor(u_img));
      int y0 = static_cast<int>(std::floor(v_img));
      int x1 = x0 + 1;
      int y1 = y0 + 1;

      // 4. Calculate interpolation weights
      float u_ratio = u_img - x0;
      float v_ratio = v_img - y0;

      // Helper to get pixel color with clamping
      auto get_pixel = [&](int x, int y) -> Vec3f {
          x = std::max(0, std::min(x, width - 1));
          y = std::max(0, std::min(y, height - 1));
          int idx = (y * width + x) * 4; // 4 channels (RGBA)
          return Vec3f(data[idx], data[idx + 1], data[idx + 2]);
      };

      // 5. Fetch neighbors
      Vec3f c00 = get_pixel(x0, y0);
      Vec3f c10 = get_pixel(x1, y0);
      Vec3f c01 = get_pixel(x0, y1);
      Vec3f c11 = get_pixel(x1, y1);

      // 6. Bilinear Interpolation
      Vec3f c_bottom = c00 * (1.0f - u_ratio) + c10 * u_ratio;
      Vec3f c_top    = c01 * (1.0f - u_ratio) + c11 * u_ratio;

      return c_bottom * (1.0f - v_ratio) + c_top * v_ratio;
  }

protected:
  int width, height;

  vector<Float> data;
  ref<MIPMap> mipmap;
  ref<TexCoordinateGenerator> texmap;
};

RDR_REGISTER_CLASS(UVMapping2D)

RDR_REGISTER_FACTORY(TexCoordinateGenerator,
    [](const Properties &props) -> TexCoordinateGenerator * {
      auto type = props.getProperty<std::string>("type");
      if (type == "uvmapping2d") {
        return Memory::alloc<UVMapping2D>(props);
      } else {
        Exception_("TexCoordinateGenerator type {} not supported", type);
      }

      return nullptr;
    })

RDR_REGISTER_CLASS(ImageTexture)
RDR_REGISTER_CLASS(ConstantTexture)
RDR_REGISTER_CLASS(CheckerBoardTexture)

RDR_REGISTER_FACTORY(Texture, [](const Properties &props) -> Texture * {
  auto type = props.getProperty<std::string>("type");
  if (type == "image") {
    return Memory::alloc<ImageTexture>(props);
  } else if (type == "checkerboard") {
    return Memory::alloc<CheckerBoardTexture>(props);
  } else if (type == "constant") {
    return Memory::alloc<ConstantTexture>(props);
  } else {
    Exception_("Texture type {} not supported", type);
  }

  return nullptr;
})

RDR_NAMESPACE_END

#endif

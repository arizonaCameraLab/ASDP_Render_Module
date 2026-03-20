/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "RenderText.h"

#include <GL/glew.h>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>

#include <freetype/freetype.h>
#include <vector>
#include <stdexcept>
#include <unordered_map>

using namespace asdp::render;

static const GLchar* flatTextureVertexShader =
  "#version 430 core\n"
  "layout(location = 0) in vec3 position;\n"
  "layout(location = 1) in vec4 vertexColor;\n"
  "layout(location = 2) in vec2 vertexTextureCoord;\n"
  "out vec4 fragmentColor;\n"
  "out vec2 textureCoord;\n"
  "void main()\n"
  "{\n"
  "   gl_Position = vec4(position,1);\n"
  "   fragmentColor = vertexColor;\n"
  "   textureCoord = vertexTextureCoord;\n"
  "}\n";

static const GLchar* flatTextureFragmentShader =
  "#version 430 core\n"
  "in vec4 fragmentColor;\n"
  "in vec2 textureCoord;\n"
  "layout(location = 0) out vec4 color;\n"
  "layout(binding = 0) uniform sampler2D tex;\n"
  "void main()\n"
  "{\n"
  "    color = fragmentColor * texture(tex, textureCoord);\n"
  "}\n";

static bool checkShaderError(GLuint shaderId, std::string& errMsg)
{
  GLint result = GL_FALSE;
  int infoLength = 0;
  glGetShaderiv(shaderId, GL_COMPILE_STATUS, &result);
  glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &infoLength);
  if (result == GL_FALSE) {
    std::vector<GLchar> errorMessage(infoLength + 1);
    glGetShaderInfoLog(shaderId, infoLength, NULL, &errorMessage[0]);
    errMsg = errorMessage.data();
    return false;
  }
  errMsg = "";
  return true;
}

static bool checkProgramError(GLuint programId, std::string& errMsg)
{
  GLint result = GL_FALSE;
  int infoLength = 0;
  glGetProgramiv(programId, GL_LINK_STATUS, &result);
  glGetProgramiv(programId, GL_INFO_LOG_LENGTH, &infoLength);
  if (result == GL_FALSE) {
    std::vector<GLchar> errorMessage(infoLength + 1);
    glGetProgramInfoLog(programId, infoLength, NULL, &errorMessage[0]);
    errMsg = errorMessage.data();
    return false;
  }
  errMsg = "";
  return true;
}

class RenderText::Impl {
public:
  /// @brief Constructor
  /// @param windowWidth The width of the window in pixels.
  /// @param windowHeight The height of the window in pixels.
  /// @details Note: An OpenGL context must be current when this is called.  GLEW must also be initialized.
  Impl(int windowWidth, int windowHeight);

  /// @brief Destructor
  virtual ~Impl();

  void SetWindowSize(int windowWidth, int windowHeight);

  /// @brief Render the given text at the given position and size.
  /// @param text The text to render. May include newline characters.
  /// @param xLoc The x position in normalized device coordinates (-1 to 1).
  /// @param yLoc The y position in normalized device coordinates (-1 to 1).
  /// @param red The red color component (0 to 1).
  /// @param green The green color component (0 to 1).
  /// @param blue The blue color component (0 to 1).
  /// @param alpha The alpha (transparency) component (0 to 1).
  /// @return Empty string on success, error message on failure.
  std::string Draw(const std::string text, float xLoc, float yLoc, float red, float green, float blue, float alpha);

  /// @brief Don't let the text get too small on very high-res displays.
  float text_width() const { return 0.96f / (std::min)(m_WINDOW_WIDTH, 1920); }
  float text_height() const { return (text_width() * m_WINDOW_WIDTH) / m_WINDOW_HEIGHT; }

  FT_Library m_ft = nullptr;
  FT_Face m_face = nullptr;
#ifdef _WIN32
  std::vector<const char*> FONTS = { "C:/Windows/Fonts/arial.ttf" };
#else
  /// @todo Find one for the mac
  std::vector<const char*> FONTS = {
    "/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-R.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf"
  };
#endif

  const int FONT_SIZE = 32;
  int m_WINDOW_WIDTH = 1920;
  int m_WINDOW_HEIGHT = 1080;

  GLuint m_programId = 0;
  GLuint m_fontVertexBuffer = 0;

  // Glyph cache structures
  struct Glyph {
    GLuint tex = 0;         // OpenGL texture for this glyph
    int width = 0;          // bitmap.width
    int rows = 0;           // bitmap.rows
    int left = 0;           // bitmap_left
    int top = 0;            // bitmap_top
    FT_Pos advanceX = 0;    // glyph->advance.x
    FT_Pos advanceY = 0;    // glyph->advance.y
  };

  // Map from character code (Unicode codepoint) to cached glyph
  std::unordered_map<FT_ULong, Glyph> m_glyph_cache;

  // Ensure a glyph is loaded and uploaded to GL; returns pointer to stored Glyph (or nullptr on failure)
  const Glyph* GetGlyph(FT_ULong charcode);
};

RenderText::Impl::Impl(int windowWidth, int windowHeight)
  : m_WINDOW_WIDTH(windowWidth)
  , m_WINDOW_HEIGHT(windowHeight)
  , m_ft(nullptr)
  , m_face(nullptr)
{
  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  GLenum ret = glewInit();
  if (ret != GLEW_OK) {
    throw std::runtime_error("RenderText::Impl(): Failed to initialize GLEW: " + std::to_string(ret));
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Initialize FreeType
  if (FT_Init_FreeType(&m_ft)) {
    throw std::runtime_error("RenderText::Impl(): Could not initialize FreeType library");
  }

  // Load a font face
  for (const char* fontPath : FONTS) {
    if (FT_New_Face(m_ft, fontPath, 0, &m_face) == 0) {
      break;
    }
  }
  if (!m_face) {
    throw std::runtime_error("RenderText::Impl(): Could not load any of the specified font faces");
  }

  // Set the font size
  FT_Set_Pixel_Sizes(m_face, 0, FONT_SIZE);

  // Create the vertex buffer object for rendering text
  glGenBuffers(1, &m_fontVertexBuffer);

  // Construct the shader program for rendering text.  It will use texture unit 0 and be a
  // simple texture-mapped shader that uses vertex colors.
  if (glCreateShader == nullptr) {
    throw std::runtime_error("RenderText::RenderText(): "
      "Attempted to construct before glewInit() has been called.");
  }

  GLuint vertexShaderId = glCreateShader(GL_VERTEX_SHADER);
  const char* vertexPrograms[] = { flatTextureVertexShader };
  glShaderSource(vertexShaderId, 1, vertexPrograms, NULL);
  glCompileShader(vertexShaderId);
  std::string errMsg;
  if (!checkShaderError(vertexShaderId, errMsg)) {
    throw std::runtime_error("RenderText::Impl(): Vertex shader compilation error: " + errMsg);
    glDeleteShader(vertexShaderId);
    return;
  }

  GLuint fragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
  const char* fragmentPrograms[] = { flatTextureFragmentShader };
  glShaderSource(fragmentShaderId, 1, fragmentPrograms, NULL);
  glCompileShader(fragmentShaderId);
  if (!checkShaderError(fragmentShaderId, errMsg)) {
    throw std::runtime_error("RenderText::Impl(): Fragment shader compilation error: " + errMsg);
    glDeleteShader(vertexShaderId);
    glDeleteShader(fragmentShaderId);
    return;
  }

  // Create and link program
  m_programId = glCreateProgram();
  glAttachShader(m_programId, vertexShaderId);
  glAttachShader(m_programId, fragmentShaderId);
  glLinkProgram(m_programId);
  if (!checkProgramError(m_programId, errMsg)) {
    glDeleteShader(vertexShaderId);
    glDeleteShader(fragmentShaderId);
    glDeleteProgram(m_programId);
    m_programId = 0;
    throw std::runtime_error("RenderText::Impl(): Shader program linking error: " + errMsg);
    return;
  }

  // We no longer need the individual shaders.
  glDeleteShader(vertexShaderId);
  glDeleteShader(fragmentShaderId);
}

RenderText::Impl::~Impl()
{
  // Free glyph textures created in the cache.
  for (auto &kv : m_glyph_cache) {
    if (kv.second.tex) {
      glDeleteTextures(1, &kv.second.tex);
    }
  }
  m_glyph_cache.clear();

  if (m_face) {
    FT_Done_Face(m_face);
    m_face = nullptr;
  }
  if (m_ft) {
    FT_Done_FreeType(m_ft);
    m_ft = nullptr;
  }
  if (m_fontVertexBuffer) {
    glDeleteBuffers(1, &m_fontVertexBuffer);
    m_fontVertexBuffer = 0;
  }
  if (m_programId) {
    glDeleteProgram(m_programId);
    m_programId = 0;
  }
}

void RenderText::Impl::SetWindowSize(int windowWidth, int windowHeight) {
  m_WINDOW_WIDTH = windowWidth;
  m_WINDOW_HEIGHT = windowHeight;
}

class FontVertex {
public:
  GLfloat pos[3];
  GLfloat col[4];
  GLfloat tex[2];
};

static void addFontQuad(std::vector<FontVertex>& vertexBufferData,
  GLfloat left, GLfloat right, GLfloat top, GLfloat bottom, GLfloat depth,
  GLfloat R, GLfloat G, GLfloat B, GLfloat alpha)
{
  FontVertex v;
  v.col[0] = R; v.col[1] = G; v.col[2] = B; v.col[3] = alpha;

  // Invert the Y texture coordinate so that we draw the textures
  // right-side up.
  // Switch the order so we have clockwise front-facing.
  v.pos[0] = left; v.pos[1] = bottom; v.pos[2] = depth;
  v.tex[0] = 0; v.tex[1] = 1;
  vertexBufferData.emplace_back(v);
  v.pos[0] = right; v.pos[1] = top; v.pos[2] = depth;
  v.tex[0] = 1; v.tex[1] = 0;
  vertexBufferData.emplace_back(v);
  v.pos[0] = right; v.pos[1] = bottom; v.pos[2] = depth;
  v.tex[0] = 1; v.tex[1] = 1;
  vertexBufferData.emplace_back(v);

  v.pos[0] = left; v.pos[1] = bottom; v.pos[2] = depth;
  v.tex[0] = 0; v.tex[1] = 1;
  vertexBufferData.emplace_back(v);
  v.pos[0] = left; v.pos[1] = top; v.pos[2] = depth;
  v.tex[0] = 0; v.tex[1] = 0;
  vertexBufferData.emplace_back(v);
  v.pos[0] = right; v.pos[1] = top; v.pos[2] = depth;
  v.tex[0] = 1; v.tex[1] = 0;
  vertexBufferData.emplace_back(v);
}

// Load glyph and create GL texture, store in cache. Returns pointer to cached Glyph or nullptr on error.
const RenderText::Impl::Glyph* RenderText::Impl::GetGlyph(FT_ULong charcode)
{
  {
    auto it = m_glyph_cache.find(charcode);
    if (it != m_glyph_cache.end()) {
      return &it->second;
    }
  }

  // Not in cache: load from FreeType
  if (!m_face) return nullptr;
  if (FT_Load_Char(m_face, charcode, FT_LOAD_RENDER)) {
    return nullptr;
  }

  FT_GlyphSlot g = m_face->glyph;

  Glyph glyph;
  glyph.width = g->bitmap.width;
  glyph.rows = g->bitmap.rows;
  glyph.left = g->bitmap_left;
  glyph.top = g->bitmap_top;
  glyph.advanceX = g->advance.x;
  glyph.advanceY = g->advance.y;

  // Create GL texture for this glyph
  glGenTextures(1, &glyph.tex);
  glBindTexture(GL_TEXTURE_2D, glyph.tex);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  // Using GL_LUMINANCE to match existing code; modern GL prefers GL_RED with shader swizzle.
  glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, glyph.width, glyph.rows,
    0, GL_LUMINANCE, GL_UNSIGNED_BYTE, g->bitmap.buffer);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // Insert into cache under lock and return pointer to stored element
  {
    auto res = m_glyph_cache.emplace(charcode, std::move(glyph));
    if (!res.second) {
      // insertion failed (shouldn't happen), clean up and return existing element
      if (res.first->second.tex) {
        glDeleteTextures(1, &res.first->second.tex);
      }
      return &res.first->second;
    }
    return &res.first->second;
  }
}

std::string RenderText::Impl::Draw(const std::string text, float xLoc, float yLoc,
  float red, float green, float blue, float alpha)
{
  float sx = text_width();
  float sy = text_height();

  // Use the program for text rendering and use texture unit 0.
  glUseProgram(m_programId);
  glActiveTexture(GL_TEXTURE0);

  if (!m_face) { return "RenderText::Draw(): No font face available"; }

  std::vector<FontVertex> vertexBufferData;

  // Enable blending using alpha.
  glEnable(GL_BLEND);

  // Configure our vertex array buffer object.
  glBindBuffer(GL_ARRAY_BUFFER, m_fontVertexBuffer);
#if !defined(NDEBUG)
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    return "RenderText::Draw(): Error after binding vertex buffer: " + std::to_string(err);
  }
#endif
  {
    size_t const stride = sizeof(vertexBufferData[0]);
    // VBO
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride,
      (GLvoid*)(offsetof(FontVertex, pos)));

    // color
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, stride,
      (GLvoid*)(offsetof(FontVertex, col)));

    // texture
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride,
      (GLvoid*)(offsetof(FontVertex, tex)));
  }

  // Break the text into lines if there are newline characters, producing a
  // vector of strings.
  std::vector<std::string> lines;
  size_t start = 0;
  size_t end = text.find('\n');
  while (end != std::string::npos) {
    lines.push_back(text.substr(start, end - start));
    start = end + 1;
    end = text.find('\n', start);
  }
  lines.push_back(text.substr(start));

  // Render each line separately, adjusting y position downward for each line.
  for (size_t ln = 0; ln < lines.size(); ++ln) {
    const std::string& line = lines[ln];
    float x = xLoc;
    float y = yLoc - ln * (FONT_SIZE * sy * 1.1f);
    
    // Compute actual width (in NDC units) for the entire string by summing glyph advances,
    // and compute the maximum bitmap height across glyphs for a better background box.
    float totalWidth = 0.0f;
    int maxRows = 0;
    for (const unsigned char p : line) {
      const Glyph* cg = GetGlyph(p);
      if (!cg) continue;
      totalWidth += (cg->advanceX / 64.0f) * sx; // advance is 26.6 fixed point
      if (cg->rows > maxRows) maxRows = cg->rows;
    }

    // If no glyphs were loaded (unlikely), fall back to using '0' width times number of characters.
    if (maxRows == 0) {
      const Glyph* cg = GetGlyph('0');
      if (cg) {
        maxRows = cg->rows;
        totalWidth = (line.size() + 1) * (cg->width * sx);
      } else {
        // absolute fallback
        maxRows = FONT_SIZE;
        totalWidth = (line.size() + 1) * (FONT_SIZE * sx * 0.5f);
      }
    }

    // Blend in a black rectangle that partially covers the region behind it, and which the
    // text will be drawn above.  Flip it upside down so that its vertices will show up as
    // front facing when it is re-flipped in the addFontQuad() method.
    glBindTexture(GL_TEXTURE_2D, 0);

    float w = 0;
    if (!line.empty()) {
      // Use width of last computed glyph to size background margin, or 0 if none.
      w = (maxRows > 0) ? (static_cast<float>(maxRows) * sx) : 0;
    }
    // Oversize the height a bit to ensure we cover tall characters.
    float yMargin = 0.2f * maxRows * sy;
    float h = maxRows * sy + 2 * yMargin;
    size_t chars = (text.size() + 1);
    vertexBufferData.clear();
    glBlendFunc(GL_SRC_COLOR, GL_ONE_MINUS_SRC_ALPHA);
    addFontQuad(vertexBufferData, x, x + totalWidth, y - yMargin + h, y - yMargin, 0.5f, 0, 0, 0, 0.9f * alpha);
    glBufferData(GL_ARRAY_BUFFER,
      sizeof(vertexBufferData[0]) * vertexBufferData.size(),
      &vertexBufferData[0], GL_STATIC_DRAW);

    // Draw the quad.
    {
      GLsizei numElements = static_cast<GLsizei>(vertexBufferData.size());
      glDrawArrays(GL_TRIANGLES, 0, numElements);
    }

    // Bind the font as the active texture (we'll bind per-glyph textures as we draw).
    // Set the fixed parameters we need to render the text properly.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
#if !defined(NDEBUG)
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      return "RenderText::Draw(): Error setting fixed texture params: " + std::to_string(err);
    }
#endif

    // Blend the characters in, so we see them written above the background.
    // We use color for the alpha channel so it appears wherever the character appears.
    // Go through each character and render it.
    for (const unsigned char p : line) {

      const Glyph* cg = GetGlyph(p);
      if (!cg) {
        continue;
      }

      // Bind glyph texture
      glBindTexture(GL_TEXTURE_2D, cg->tex);

      // Because we uploaded the bitmap once when caching, just draw the quad using the cached metrics.
      float x2 = x + cg->left * sx;
      float y2 = y + cg->top * sy;
      float gw = cg->width * sx;
      float gh = cg->rows * sy;

      // Blend in the text
      glBlendFunc(GL_SRC_COLOR, GL_ONE_MINUS_SRC_COLOR);
      vertexBufferData.clear();
      addFontQuad(vertexBufferData, x2, x2 + gw, y2, y2 - gh, 0.7f, red*alpha, green*alpha, blue*alpha, 1 - alpha);
      glBufferData(GL_ARRAY_BUFFER,
        sizeof(vertexBufferData[0]) * vertexBufferData.size(),
        &vertexBufferData[0], GL_STATIC_DRAW);
#if !defined(NDEBUG)
      err = glGetError();
      if (err != GL_NO_ERROR) {
        return "RenderText::Draw(): Error buffering data: " + std::to_string(err);
      }
#endif

      // Draw the quad.
      {
        GLsizei numElements = static_cast<GLsizei>(vertexBufferData.size());
        glDrawArrays(GL_TRIANGLES, 0, numElements);
      }

      x += (cg->advanceX / 64) * sx;
      y += (cg->advanceY / 64) * sy;
    } // End of charcter
  } // End of line

  // Set things back to the defaults
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

  glBindTexture(GL_TEXTURE_2D, 0);
  glDisable(GL_BLEND);

  return "";
}

RenderText::RenderText(int windowWidth, int windowHeight)
{
  m_impl = std::make_shared<Impl>(windowWidth, windowHeight);
}

void RenderText::SetWindowSize(int windowWidth, int windowHeight)
{
  m_impl->SetWindowSize(windowWidth, windowHeight);
}

std::string RenderText::Draw(std::string text, float x, float y, float r, float g, float b, float alpha)
{
  return m_impl->Draw(text.c_str(), x, y, r, g, b, alpha);
}

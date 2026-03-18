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
  GLuint m_font_tex = 0;
  int m_WINDOW_WIDTH = 1920;
  int m_WINDOW_HEIGHT = 1080;

  GLuint m_programId = 0;
  GLuint m_fontVertexBuffer = 0;
};

RenderText::Impl::Impl(int windowWidth, int windowHeight)
  : m_WINDOW_WIDTH(windowWidth)
  , m_WINDOW_HEIGHT(windowHeight)
  , m_ft(nullptr)
  , m_face(nullptr)
  , m_font_tex(0)
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

  // Generate our font texture.
  glGenTextures(1, &m_font_tex);

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
  if (m_face) {
    FT_Done_Face(m_face);
    m_face = nullptr;
  }
  if (m_ft) {
    FT_Done_FreeType(m_ft);
    m_ft = nullptr;
  }
  if (m_font_tex) {
    glDeleteTextures(1, &m_font_tex);
    m_font_tex = 0;
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

std::string RenderText::Impl::Draw(const std::string text, float xLoc, float yLoc,
  float red, float green, float blue, float alpha)
{
  float sx = text_width();
  float sy = text_height();

  // Use the program for text rendering and use texture unit 0.
  glUseProgram(m_programId);
  glActiveTexture(GL_TEXTURE0);

  if (!m_face) { return "RenderText::Draw(): No font face available"; }
  FT_GlyphSlot g = m_face->glyph;

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
    for (const char p : line) {
      if (FT_Load_Char(m_face, p, FT_LOAD_RENDER))
        continue;
      totalWidth += (g->advance.x / 64.0f) * sx; // advance is 26.6 fixed point
      if (g->bitmap.rows > maxRows) maxRows = g->bitmap.rows;
    }

    // If no glyphs were loaded (unlikely), fall back to using '0' width times number of characters.
    if (maxRows == 0) {
      if (FT_Load_Char(m_face, '0', FT_LOAD_RENDER) == 0) {
        maxRows = g->bitmap.rows;
        totalWidth = (line.size() + 1) * (g->bitmap.width * sx);
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

    float w = g->bitmap.width * sx;
    // Oversize the height a bit to ensure we cover tall characters.
    float yMargin = 0.2 * maxRows * sy;
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

    // Bind the font as the active texture.
    glBindTexture(GL_TEXTURE_2D, m_font_tex);
#if !defined(NDEBUG)
    err = glGetError();
    if (err != GL_NO_ERROR) {
      return "RenderText::Draw(): Error binding texture: " + std::to_string(err);
    }
#endif

    // Set the fixed parameters we need to render the text properly.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
#if !defined(NDEBUG)
    err = glGetError();
    if (err != GL_NO_ERROR) {
      return "RenderText::Draw(): Error setting fixed texture params: " + std::to_string(err);
    }
#endif

    // Blend the characters in, so we see them written above the background.
    // We use color for the alpha channel so it appears wherever the character appears.
    // Go through each character and render it.
    for (const char p : line) {

      if (FT_Load_Char(m_face, p, FT_LOAD_RENDER)) {
        continue;
      }

      // Set the variable parameters we need to render the text properly.
      glPixelStorei(GL_UNPACK_ROW_LENGTH, g->bitmap.width);
#if !defined(NDEBUG)
      err = glGetError();
      if (err != GL_NO_ERROR) {
        return "RenderText::Draw(): Error setting texture params: " + std::to_string(err);
      }
#endif
      glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, g->bitmap.width, g->bitmap.rows,
        0, GL_LUMINANCE, GL_UNSIGNED_BYTE, g->bitmap.buffer);
#if !defined(NDEBUG)
      err = glGetError();
      if (err != GL_NO_ERROR) {
        return "RenderText::Draw(): Error writing texture: " + std::to_string(err);
      }
#endif
      float x2 = x + g->bitmap_left * sx;
      float y2 = y + g->bitmap_top * sy;
      float w = g->bitmap.width * sx;
      float h = g->bitmap.rows * sy;

      // Blend in the text
      glBlendFunc(GL_SRC_COLOR, GL_ONE_MINUS_SRC_COLOR);
      vertexBufferData.clear();
      addFontQuad(vertexBufferData, x2, x2 + w, y2, y2 - h, 0.7f, red*alpha, green*alpha, blue*alpha, 1 - alpha);
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

      x += (g->advance.x / 64) * sx;
      y += (g->advance.y / 64) * sy;
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

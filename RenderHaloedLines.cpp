/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "RenderHaloedLines.h"

#include <GL/glew.h>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>

#include <stdexcept>

using namespace asdp::render;

static const GLchar* flatVertexShader =
  "#version 430 core\n"
  "layout(location = 0) in vec3 position;\n"
  "layout(location = 1) in vec4 vertexColor;\n"
  "out vec4 fragmentColor;\n"
  "void main()\n"
  "{\n"
  "   gl_Position = vec4(position,1);\n"
  "   fragmentColor = vertexColor;\n"
  "}\n";

static const GLchar* flatFragmentShader =
  "#version 430 core\n"
  "in vec4 fragmentColor;\n"
  "layout(location = 0) out vec4 color;\n"
  "void main()\n"
  "{\n"
  "    color = fragmentColor;\n"
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

class RenderHaloedLines::Impl {
public:
  /// @brief Constructor
  /// @details Note: An OpenGL context must be current when this is called.  GLEW must also be initialized.
  Impl();

  /// @brief Destructor
  virtual ~Impl();

  std::string Draw(std::vector< std::array< std::array<float, 2>, 2> > lines,
    float lineWidth, float haloWidth, std::array<float, 3> lineColor,
    std::array<float, 3> haloColor, float alpha);

  GLuint m_programId = 0;
  GLuint m_vertexBuffer = 0;
};

RenderHaloedLines::Impl::Impl()
{
  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  GLenum ret = glewInit();
  if (ret != GLEW_OK) {
    throw std::runtime_error("RenderHaloedLines::Impl(): Failed to initialize GLEW: " + std::to_string(ret));
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Create the vertex buffer object for rendering lines
  glGenBuffers(1, &m_vertexBuffer);

  // Construct the shader program for rendering text.  It will use texture unit 0 and be a
  // simple texture-mapped shader that uses vertex colors.
  if (glCreateShader == nullptr) {
    throw std::runtime_error("RenderHaloedLines::Impl(): "
      "Attempted to construct before glewInit() has been called.");
  }

  GLuint vertexShaderId = glCreateShader(GL_VERTEX_SHADER);
  const char* vertexPrograms[] = { flatVertexShader };
  glShaderSource(vertexShaderId, 1, vertexPrograms, NULL);
  glCompileShader(vertexShaderId);
  std::string errMsg;
  if (!checkShaderError(vertexShaderId, errMsg)) {
    throw std::runtime_error("RenderHaloedLines::Impl(): Vertex shader compilation error: " + errMsg);
    glDeleteShader(vertexShaderId);
    return;
  }

  GLuint fragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
  const char* fragmentPrograms[] = { flatFragmentShader };
  glShaderSource(fragmentShaderId, 1, fragmentPrograms, NULL);
  glCompileShader(fragmentShaderId);
  if (!checkShaderError(fragmentShaderId, errMsg)) {
    throw std::runtime_error("RenderHaloedLines::Impl(): Fragment shader compilation error: " + errMsg);
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
    throw std::runtime_error("RenderHaloedLines::Impl(): Shader program linking error: " + errMsg);
    return;
  }

  // We no longer need the individual shaders.
  glDeleteShader(vertexShaderId);
  glDeleteShader(fragmentShaderId);
}

RenderHaloedLines::Impl::~Impl()
{
  if (m_vertexBuffer) {
    glDeleteBuffers(1, &m_vertexBuffer);
    m_vertexBuffer = 0;
  }
  if (m_programId) {
    glDeleteProgram(m_programId);
    m_programId = 0;
  }
}

class LineVertex {
public:
  GLfloat pos[3];
  GLfloat col[4];
};

static void addLine(std::vector<LineVertex>& vertexBufferData,
  GLfloat x0, GLfloat y0, GLfloat x1, GLfloat y1, GLfloat depth,
  GLfloat R, GLfloat G, GLfloat B, GLfloat alpha)
{
  LineVertex v;
  v.col[0] = R; v.col[1] = G; v.col[2] = B; v.col[3] = alpha;

  v.pos[0] = x0; v.pos[1] = y0; v.pos[2] = depth;
  vertexBufferData.emplace_back(v);
  v.pos[0] = x1; v.pos[1] = y1; v.pos[2] = depth;
  vertexBufferData.emplace_back(v);
}

std::string RenderHaloedLines::Impl::Draw(std::vector< std::array< std::array<float, 2>, 2> > lines,
  float lineWidth, float haloWidth, std::array<float, 3> lineColor,
  std::array<float, 3> haloColor, float alpha)
{
  // Use the program for text rendering and use texture unit 0.
  glUseProgram(m_programId);

  std::vector<LineVertex> vertexBufferData;

  // Enable blending using alpha.
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_COLOR, GL_ONE_MINUS_SRC_ALPHA);

  // Configure our vertex array buffer object.
  glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    return "RenderHaloedLines::Draw(): Error after binding vertex buffer: " + std::to_string(err);
  }
  {
    size_t const stride = sizeof(vertexBufferData[0]);
    // VBO
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride,
      (GLvoid*)(offsetof(LineVertex, pos)));

    // color
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, stride,
      (GLvoid*)(offsetof(LineVertex, col)));
  }

  // Render each halo separately.
  glLineWidth(haloWidth);
  for (size_t ln = 0; ln < lines.size(); ++ln) {
    auto const& line = lines[ln];

    // Blend in a background wider line.
    vertexBufferData.clear();
    addLine(vertexBufferData, line[0][0], line[0][1], line[1][0], line[1][1], 0.5f,
      haloColor[0], haloColor[1], haloColor[2], alpha);
    glBufferData(GL_ARRAY_BUFFER,
      sizeof(vertexBufferData[0]) * vertexBufferData.size(),
      &vertexBufferData[0], GL_STATIC_DRAW);

    // Draw the lines.
    GLsizei numElements = static_cast<GLsizei>(vertexBufferData.size());
    glDrawArrays(GL_LINES, 0, numElements);
  } // End of halo lines

  // Render each line separately.
  glLineWidth(lineWidth);
  for (size_t ln = 0; ln < lines.size(); ++ln) {
    auto const& line = lines[ln];

    // Blend in the foreground line.
    vertexBufferData.clear();
    addLine(vertexBufferData, line[0][0], line[0][1], line[1][0], line[1][1], 0.0f,
      lineColor[0], lineColor[1], lineColor[2], alpha);
    glBufferData(GL_ARRAY_BUFFER,
      sizeof(vertexBufferData[0]) * vertexBufferData.size(),
      &vertexBufferData[0], GL_STATIC_DRAW);

    // Draw the lines.
    GLsizei numElements = static_cast<GLsizei>(vertexBufferData.size());
    glDrawArrays(GL_LINES, 0, numElements);
  } // End of lines

  // Set things back to the defaults
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDisable(GL_BLEND);

  return "";
}

RenderHaloedLines::RenderHaloedLines()
{
  m_impl = std::make_shared<Impl>();
}

std::string RenderHaloedLines::Draw(std::vector< std::array< std::array<float, 2>, 2> > lines,
  float lineWidth, float haloWidth, std::array<float, 3> lineColor,
  std::array<float, 3> haloColor, float alpha)
{
  return m_impl->Draw(lines, lineWidth, haloWidth, lineColor, haloColor, alpha);
}

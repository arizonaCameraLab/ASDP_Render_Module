/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <string>
#include <iostream>
#include <GL/glew.h>
#include "Composite.h"

using namespace asdp::render;

Composite::Composite(std::vector<CameraRenderInfo>& cameraRenderInfo)
  : m_cameraRenderInfos(cameraRenderInfo)
{
  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    std::cerr << "Composite::Composite(): Failed to initialize GLEW\n" << std::endl;
    return;
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();
}

Composite::~Composite()
{
  // Empty destructor.
}

void Composite::Render(asdp::Time scanOutTime, std::vector<ViewRenderInfo> views)
{
  // Set up the geometry for multiple renders
  SetupRenderFrame(scanOutTime);

  // Render each view
  for (auto const& view : views) {
    // Set up the frame buffer and assign the appropriate textures.
    glBindFramebuffer(GL_FRAMEBUFFER, view.frameBuffer);
    if (view.frameBuffer != 0) {
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, view.color, 0);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, view.depth, 0);
    }

    // Set up the viewport and clear the buffers.
    glViewport(view.x, view.y, view.width, view.height);
    glClearColor(0, 0, 0, 1.0f);
    GLbitfield clearBits = 0;
    if ( (view.frameBuffer == 0) || (view.color != 0) ) { clearBits |= GL_COLOR_BUFFER_BIT; }
    if ( (view.frameBuffer == 0) || (view.depth != 0) ) { clearBits |= GL_DEPTH_BUFFER_BIT; }
    glClear(clearBits);

    // Turn on depth testing so we get proper rendering.
    if ((view.frameBuffer == 0) || (view.depth != 0)) {
      glEnable(GL_DEPTH_TEST);
      glDepthFunc(GL_LESS);
    }

    // Call the derived-class method to render the geometry into this viewpoint.
    RenderView(view.modelViewProjection.data());

    // Unset things
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  // Done with the data for a render frame
  TearDownRenderFrame();

  // Wait until the rendering has finished.
  glFinish();
}

//==================================================================================================
// Objects needed by the CompositeCube class.

class asdp::render::CompositeCube::MeshCube {
public:
  MeshCube(GLfloat scale, size_t numTriangles = 6 * 2 * 15 * 15) {
    // Figure out how many quads we have per edge.  There
    // is a minimum of 1.
    size_t numQuads = numTriangles / 2;
    size_t numQuadsPerFace = numQuads / 6;
    size_t numQuadsPerEdge = static_cast<size_t> (sqrt(numQuadsPerFace));
    if (numQuadsPerEdge < 1) { numQuadsPerEdge = 1; }

    // Construct a white square with the specified number of
    // quads as the +Z face of the cube.  We'll copy this and
    // then multiply by the correct face color, and we'll
    // adjust the coordinates by rotation to match each face.
    std::vector<GLfloat> whiteBufferData;
    std::vector<GLfloat> faceBufferData;
    for (size_t i = 0; i < numQuadsPerEdge; i++) {
      for (size_t j = 0; j < numQuadsPerEdge; j++) {

        // Modulate the color of each quad by a random luminance,
        // leaving all vertices the same color.
        GLfloat color = 0.5f + rand() * 0.5f / RAND_MAX;
        const size_t numTris = 2;
        const size_t numColors = 3;
        const size_t numVerts = 3;
        for (size_t c = 0; c < numColors * numTris * numVerts; c++) {
          whiteBufferData.push_back(color);
        }

        // Send the two triangles that make up this quad, where the
        // quad covers the appropriate fraction of the face from
        // -scale to scale in X and Y.
        GLfloat Z = scale;
        GLfloat minX = -scale + i * (2 * scale) / numQuadsPerEdge;
        GLfloat maxX = -scale + (i + 1) * (2 * scale) / numQuadsPerEdge;
        GLfloat minY = -scale + j * (2 * scale) / numQuadsPerEdge;
        GLfloat maxY = -scale + (j + 1) * (2 * scale) / numQuadsPerEdge;
        faceBufferData.push_back(minX);
        faceBufferData.push_back(maxY);
        faceBufferData.push_back(Z);

        faceBufferData.push_back(minX);
        faceBufferData.push_back(minY);
        faceBufferData.push_back(Z);

        faceBufferData.push_back(maxX);
        faceBufferData.push_back(minY);
        faceBufferData.push_back(Z);

        faceBufferData.push_back(maxX);
        faceBufferData.push_back(maxY);
        faceBufferData.push_back(Z);

        faceBufferData.push_back(minX);
        faceBufferData.push_back(maxY);
        faceBufferData.push_back(Z);

        faceBufferData.push_back(maxX);
        faceBufferData.push_back(minY);
        faceBufferData.push_back(Z);
      }
    }

    // Make a copy of the vertices for each face, then modulate
    // the color by the face color and rotate the coordinates to
    // put them on the correct cube face.

    // +Z is blue and is in the same location as the original
    // faces.
    {
      std::array<GLfloat, 3> modColor = { 0.0, 0.0, 1.0 };
      std::vector<GLfloat> myBufferData =
        colorModulate(whiteBufferData, modColor);

      // X = X, Y = Y, Z = Z
      std::array<GLfloat, 3> scales = { 1.0f, 1.0f, 1.0f };
      std::array<size_t, 3> indices = { 0, 1, 2 };
      std::vector<GLfloat> myFaceBufferData =
        vertexRotate(faceBufferData, indices, scales);

      // Catenate the colors onto the end of the
      // color buffer.
      colorBufferData.insert(colorBufferData.end(),
        myBufferData.begin(), myBufferData.end());

      // Catenate the vertices onto the end of the
      // vertex buffer.
      vertexBufferData.insert(vertexBufferData.end(),
        myFaceBufferData.begin(), myFaceBufferData.end());
    }

    // -Z is cyan and is in the opposite size from the
    // original face (mirror all 3).
    {
      std::array<GLfloat, 3> modColor = { 0.0, 1.0, 1.0 };
      std::vector<GLfloat> myBufferData =
        colorModulate(whiteBufferData, modColor);

      // X = -X, Y = -Y, Z = -Z
      std::array<GLfloat, 3> scales = { -1.0f, -1.0f, -1.0f };
      std::array<size_t, 3> indices = { 0, 1, 2 };
      std::vector<GLfloat> myFaceBufferData =
        vertexRotate(faceBufferData, indices, scales);

      // Catenate the colors onto the end of the
      // color buffer.
      colorBufferData.insert(colorBufferData.end(),
        myBufferData.begin(), myBufferData.end());

      // Catenate the vertices onto the end of the
      // vertex buffer.
      vertexBufferData.insert(vertexBufferData.end(),
        myFaceBufferData.begin(), myFaceBufferData.end());
    }

    // +X is red and is rotated -90 degrees from the original
    // around Y.
    {
      std::array<GLfloat, 3> modColor = { 1.0, 0.0, 0.0 };
      std::vector<GLfloat> myBufferData =
        colorModulate(whiteBufferData, modColor);

      // X = Z, Y = Y, Z = -X
      std::array<GLfloat, 3> scales = { 1.0f, 1.0f, -1.0f };
      std::array<size_t, 3> indices = { 2, 1, 0 };
      std::vector<GLfloat> myFaceBufferData =
        vertexRotate(faceBufferData, indices, scales);

      // Catenate the colors onto the end of the
      // color buffer.
      colorBufferData.insert(colorBufferData.end(),
        myBufferData.begin(), myBufferData.end());

      // Catenate the vertices onto the end of the
      // vertex buffer.
      vertexBufferData.insert(vertexBufferData.end(),
        myFaceBufferData.begin(), myFaceBufferData.end());
    }

    // -X is magenta and is rotated 90 degrees from the original
    // around Y.
    {
      std::array<GLfloat, 3> modColor = { 1.0, 0.0, 1.0 };
      std::vector<GLfloat> myBufferData =
        colorModulate(whiteBufferData, modColor);

      // X = -Z, Y = Y, Z = X
      std::array<GLfloat, 3> scales = { -1.0f, 1.0f, 1.0f };
      std::array<size_t, 3> indices = { 2, 1, 0 };
      std::vector<GLfloat> myFaceBufferData =
        vertexRotate(faceBufferData, indices, scales);

      // Catenate the colors onto the end of the
      // color buffer.
      colorBufferData.insert(colorBufferData.end(),
        myBufferData.begin(), myBufferData.end());

      // Catenate the vertices onto the end of the
      // vertex buffer.
      vertexBufferData.insert(vertexBufferData.end(),
        myFaceBufferData.begin(), myFaceBufferData.end());
    }

    // +Y is green and is rotated -90 degrees from the original
    // around X.
    {
      std::array<GLfloat, 3> modColor = { 0.0, 1.0, 0.0 };
      std::vector<GLfloat> myBufferData =
        colorModulate(whiteBufferData, modColor);

      // X = X, Y = Z, Z = -Y
      std::array<GLfloat, 3> scales = { 1.0f, 1.0f, -1.0f };
      std::array<size_t, 3> indices = { 0, 2, 1 };
      std::vector<GLfloat> myFaceBufferData =
        vertexRotate(faceBufferData, indices, scales);

      // Catenate the colors onto the end of the
      // color buffer.
      colorBufferData.insert(colorBufferData.end(),
        myBufferData.begin(), myBufferData.end());

      // Catenate the vertices onto the end of the
      // vertex buffer.
      vertexBufferData.insert(vertexBufferData.end(),
        myFaceBufferData.begin(), myFaceBufferData.end());
    }

    // -Y is yellow and is rotated 90 degrees from the original
    // around X.
    {
      std::array<GLfloat, 3> modColor = { 1.0, 1.0, 0.0 };
      std::vector<GLfloat> myBufferData =
        colorModulate(whiteBufferData, modColor);

      // X = X, Y = -Z, Z = Y
      std::array<GLfloat, 3> scales = { 1.0f, -1.0f, 1.0f };
      std::array<size_t, 3> indices = { 0, 2, 1 };
      std::vector<GLfloat> myFaceBufferData =
        vertexRotate(faceBufferData, indices, scales);

      // Catenate the colors onto the end of the
      // color buffer.
      colorBufferData.insert(colorBufferData.end(),
        myBufferData.begin(), myBufferData.end());

      // Catenate the vertices onto the end of the
      // vertex buffer.
      vertexBufferData.insert(vertexBufferData.end(),
        myFaceBufferData.begin(), myFaceBufferData.end());
    }
  }

  ~MeshCube() {
    if (initialized) {
      glDeleteBuffers(1, &vertexBuffer);
      glDeleteBuffers(1, &colorBuffer);
      glDeleteVertexArrays(1, &vertexArrayId);
    }
  }

  void init() {
    if (!initialized) {
      // Vertex buffer
      glGenBuffers(1, &vertexBuffer);
      glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
      glBufferData(GL_ARRAY_BUFFER,
        sizeof(vertexBufferData[0]) * vertexBufferData.size(),
        vertexBufferData.data(), GL_STATIC_DRAW);
      glBindBuffer(GL_ARRAY_BUFFER, 0);

      // Color buffer
      glGenBuffers(1, &colorBuffer);
      glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
      glBufferData(GL_ARRAY_BUFFER,
        sizeof(colorBufferData[0]) * colorBufferData.size(),
        colorBufferData.data(), GL_STATIC_DRAW);
      glBindBuffer(GL_ARRAY_BUFFER, 0);

      // Vertex array object
      glGenVertexArrays(1, &vertexArrayId);
      glBindVertexArray(vertexArrayId);
      {
        // VBO
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

        // color
        glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
      }
      glBindVertexArray(0);
      initialized = true;
    }
  }

  void draw() {
    init();

    glBindVertexArray(vertexArrayId);
    {
      glDrawArrays(GL_TRIANGLES, 0,
        static_cast<GLsizei>(vertexBufferData.size()));
    }
    glBindVertexArray(0);
  }

private:
  MeshCube(const MeshCube&) = delete;
  MeshCube& operator=(const MeshCube&) = delete;
  bool initialized = false;
  GLuint colorBuffer = 0;
  GLuint vertexBuffer = 0;
  GLuint vertexArrayId = 0;
  std::vector<GLfloat> colorBufferData;
  std::vector<GLfloat> vertexBufferData;

  // Multiply each triple of colors by the specified color.
  std::vector<GLfloat> colorModulate(std::vector<GLfloat> const& inVec,
    std::array<GLfloat, 3> const& clr) {
    std::vector<GLfloat> out;
    size_t elements = inVec.size() / 3;
    if (elements * 3 != inVec.size()) {
      // We don't have an even multiple of 3 elements, so bail.
      return out;
    }
    out = inVec;
    for (size_t i = 0; i < elements; i++) {
      for (size_t c = 0; c < 3; c++) {
        out[3 * i + c] *= clr[c];
      }
    }
    return out;
  }

  // Swizzle each triple of coordinates by the specified
  // index and then multiply by the specified scale.  This
  // lets us implement a poor-man's rotation matrix, where
  // we pick which element (0-2) and which polarity (-1 or
  // 1) to use.
  std::vector<GLfloat> vertexRotate(
    std::vector<GLfloat> const& inVec,
    std::array<size_t, 3> const& indices,
    std::array<GLfloat, 3> const& scales) {
    std::vector<GLfloat> out;
    size_t elements = inVec.size() / 3;
    if (elements * 3 != inVec.size()) {
      // We don't have an even multiple of 3 elements, so bail.
      return out;
    }
    out.resize(inVec.size());
    for (size_t i = 0; i < elements; i++) {
      for (size_t p = 0; p < 3; p++) {
        out[3 * i + p] = inVec[3 * i + indices[p]] * scales[p];
      }
    }
    return out;
  }
};

static const GLchar* cubeVertexShader =
  "#version 330 core\n"
  "layout(location = 0) in vec3 position;\n"
  "layout(location = 1) in vec3 vertexColor;\n"
  "out vec3 fragmentColor;\n"
  "uniform mat4 modelViewProjection;\n"
  "void main()\n"
  "{\n"
  "   gl_Position = modelViewProjection * vec4(position,1);\n"
  "   fragmentColor = vertexColor;\n"
  "}\n";

static const GLchar* cubeFragmentShader = "#version 330 core\n"
  "in vec3 fragmentColor;\n"
  "out vec3 color;\n"
  "void main()\n"
  "{\n"
  "    color = fragmentColor;\n"
  "}\n";

void CompositeCube::checkShaderError(GLuint shaderId, const std::string& exceptionMsg) {
  GLint result = GL_FALSE;
  int infoLength = 0;
  glGetShaderiv(shaderId, GL_COMPILE_STATUS, &result);
  glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &infoLength);
  if (result == GL_FALSE) {
    std::vector<GLchar> errorMessage(infoLength + 1);
    glGetShaderInfoLog(shaderId, infoLength, NULL, &errorMessage[0]);
    std::cerr << &errorMessage[0] << std::endl;
    throw std::runtime_error(exceptionMsg);
  }
}

void CompositeCube::checkProgramError(GLuint programId, const std::string& exceptionMsg) {
  GLint result = GL_FALSE;
  int infoLength = 0;
  glGetProgramiv(programId, GL_LINK_STATUS, &result);
  glGetProgramiv(programId, GL_INFO_LOG_LENGTH, &infoLength);
  if (result == GL_FALSE) {
    std::vector<GLchar> errorMessage(infoLength + 1);
    glGetProgramInfoLog(programId, infoLength, NULL, &errorMessage[0]);
    std::cerr << &errorMessage[0] << std::endl;
    throw std::runtime_error(exceptionMsg);
  }
}


CompositeCube::CompositeCube(double radius)
  : Composite(std::vector<CameraRenderInfo>())
  , m_radius(radius)
{
  GLuint vertexShaderId = glCreateShader(GL_VERTEX_SHADER);
  GLuint fragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);

  // vertex shader
  glShaderSource(vertexShaderId, 1, &cubeVertexShader, NULL);
  glCompileShader(vertexShaderId);
  checkShaderError(vertexShaderId, "Vertex shader compilation failed.");

  // fragment shader
  glShaderSource(fragmentShaderId, 1, &cubeFragmentShader, NULL);
  glCompileShader(fragmentShaderId);
  checkShaderError(fragmentShaderId, "Fragment shader compilation failed.");

  // linking shader program
  m_programId = glCreateProgram();
  glAttachShader(m_programId, vertexShaderId);
  glAttachShader(m_programId, fragmentShaderId);
  glLinkProgram(m_programId);
  checkProgramError(m_programId, "Shader program link failed.");

  // once linked into a program, we no longer need the shaders.
  glDeleteShader(vertexShaderId);
  glDeleteShader(fragmentShaderId);

  m_modelViewProjectionUniformId = glGetUniformLocation(m_programId, "modelViewProjection");

  // Make our geometry object, which will draw itself.
  size_t quadsPerEdge = 10;
  size_t trianglesPerSide = 2 * quadsPerEdge * quadsPerEdge;
  // 6 faces
  size_t numTriangles = static_cast<size_t>(trianglesPerSide * 6);
  m_roomCube = std::shared_ptr<MeshCube>(new MeshCube(m_radius, numTriangles));
}

CompositeCube::~CompositeCube()
{
  glDeleteProgram(m_programId);
}

void CompositeCube::SetupRenderFrame(asdp::Time scanOutTime)
{
  glUseProgram(m_programId);
}

void CompositeCube::RenderView(const float* modelViewProjection)
{
  // Set the model-view-projection matrix and draw the cube.
  glUniformMatrix4fv(m_modelViewProjectionUniformId, 1, GL_FALSE, modelViewProjection);
  m_roomCube->draw();
}

void CompositeCube::TearDownRenderFrame()
{
  glUseProgram(0);
}

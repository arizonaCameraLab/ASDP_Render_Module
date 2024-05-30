/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <string>
#include <iostream>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Composite.h"

using namespace asdp::render;

Composite::~Composite()
{
  // Empty destructor.
}

void Composite::Render(asdp::Time scanOutTime, std::vector<ViewRenderInfo> views)
{
  // Initialize for rendering if it has not already been done.  Do this while holding
  // a mutex lock so we don't have it happen in two threads as a race.
  {
    std::lock_guard<std::mutex> lock(m_initMutex);
    if (!m_initialized) {
      if (!SetupRendering()) {
        std::cerr << "Composite::Render(): Could not set up rendering" << std::endl;
        return;
      }
      m_initialized = true;
    }
  }

  // Set up the geometry for multiple renders
  SetupRenderFrame(scanOutTime);

  // Render each view
  for (size_t eye = 0; eye < views.size(); eye++) {
    const ViewRenderInfo& view = views[eye];

    // Only set up the frame buffer and clear the buffers if we're the first eye or if the
    // eyes use different frame buffers.
    if ((eye == 0) || (views[eye].frameBuffer != views[0].frameBuffer)) {
      // Bind the frame buffer and assign the appropriate textures.
      glBindFramebuffer(GL_FRAMEBUFFER, view.frameBuffer);
      if (view.frameBuffer != 0) {
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, view.colorBuffer, 0);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, view.depthBuffer, 0);
      }

      // Clear the buffers.  The clear color is sky blue to distingiush from a camera with a black texture.
      glClearColor(0.6f, 0.8f, 1.0f, 1.0f);
      GLbitfield clearBits = 0;
      if ((view.frameBuffer == 0) || (view.colorBuffer != 0)) { clearBits |= GL_COLOR_BUFFER_BIT; }
      if ((view.frameBuffer == 0) || (view.depthBuffer != 0)) { clearBits |= GL_DEPTH_BUFFER_BIT; }
      glClear(clearBits);
    }

    // Turn on depth testing so we get proper rendering.  The default frame buffer has depth.
    if ((view.frameBuffer == 0) || (view.depthBuffer != 0)) {
      glEnable(GL_DEPTH_TEST);
      glDepthFunc(GL_LESS);
    }

    // Set up the viewport for this view.
    glViewport(view.x, view.y, view.width, view.height);

    // Rotate the view to match the helicopter's orientation, looking down the +Y axis with
    // the up vector being Z.  This rotates the camera by 90 degrees around the X axis.  Because
    // we're rotating the world rather than the camera, we rotate in the opposite direction.
    glm::mat4 HelicopterRotateX = glm::rotate(glm::mat4(1.0f),
      glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    // Compute the view-projection matrix (no model described here) from the ViewRenderInfo.
    // NOTE: We translate and rotate in the opposite direction because we're moving the world rather
    // than the camera but the offset and orientation are specified for camera movement.
    // NOTE: We also do the order of operations in reverse because we're moving the world rather
    // than the camera.
    glm::mat4 ViewRotateZ = glm::rotate(HelicopterRotateX,
      glm::radians(-view.orientation[2]), glm::vec3(0.0f, 0.0f, 1.0f));
    glm::mat4 ViewRotateY = glm::rotate(ViewRotateZ,
      glm::radians(-view.orientation[1]), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 ViewRotateX = glm::rotate(ViewRotateY,
      glm::radians(-view.orientation[0]), glm::vec3(1.0f, 0.0f, 0.0f));

    // Translate the view based on the specified viewpoint (negative due to world vs. camera).
    glm::mat4 ViewTranslate = glm::translate(ViewRotateX,
      glm::vec3(-view.viewpoint[0], -view.viewpoint[1], -view.viewpoint[2]));

    // Compute the projection matrix from the ViewRenderInfo.
    double leftFrust = tan(glm::radians(view.leftHalfFOV)) * view.nearClip;
    double rightFrust = tan(glm::radians(view.rightHalfFOV)) * view.nearClip;
    double bottomFrust = tan(glm::radians(view.bottomHalfFOV)) * view.nearClip;
    double topFrust = tan(glm::radians(view.topHalfFOV)) * view.nearClip;
    glm::mat4 Projection = glm::frustum<float>(leftFrust, rightFrust, bottomFrust, topFrust,
      view.nearClip, view.farClip);
    glm::mat4 VP = Projection * ViewTranslate;

    /// @todo Adjust for helicopter motion changes from image acquisition to scan-out.

    /// @todo Adjust for shear and stretch due to head motion during scan-out.

    // Call the derived-class method to render the geometry into this viewpoint.
    RenderView(glm::value_ptr(VP));
  }

  // Unset things
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // Done with the data for a render frame
  TearDownRenderFrame();

  // Wait until the rendering has finished.
  glFinish();
}

void Composite::checkShaderError(GLuint shaderId, const std::string& exceptionMsg) {
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

void Composite::checkProgramError(GLuint programId, const std::string& exceptionMsg) {
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
    }
  }

  void init() {
    if (!initialized) {
      // Unbind any vertex array object.
      glBindVertexArray(0);

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

      initialized = true;
    }
  }

  void draw() {
    init();

    // Unbind any currently bound vertex array object.
    // We cannot use vertex array objects because we're potentially going to be called
    // from multiple OpenGL contexts in different threads and VAOs are not shared between
    // contexts.
    glBindVertexArray(0);

    // Enable the vertex attribute arrays we are going to use
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    // Bind the vertex buffer object
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    // Bind the color buffer object
    glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    // Draw our geometry
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertexBufferData.size()));
  }

private:
  MeshCube(const MeshCube&) = delete;
  MeshCube& operator=(const MeshCube&) = delete;
  bool initialized = false;
  GLuint colorBuffer = 0;
  GLuint vertexBuffer = 0;
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
R"(#version 330 core
   layout(location = 0) in vec3 position;
   layout(location = 1) in vec3 vertexColor;
   out vec3 fragmentColor;
   uniform mat4 modelViewProjection;
   void main()
   {
      gl_Position = modelViewProjection * vec4(position,1);
      fragmentColor = vertexColor;
   })";

static const GLchar* cubeFragmentShader =
R"(#version 330 core
   in vec3 fragmentColor;
   out vec3 color;
   void main()
   {
       color = fragmentColor;
   })";

CompositeCube::CompositeCube(double radius)
  : Composite()
  , m_radius(radius)
  , m_programId(0)
  , m_modelViewProjectionUniformId(0)
{
}

bool CompositeCube::SetupRendering()
{
  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    std::cerr << "CompositeCube::CompositeCube(): Failed to initialize GLEW" << std::endl;
    return false;
  }

  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Construct the shader programs.
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

  return true;
}

CompositeCube::~CompositeCube()
{
  glDeleteProgram(m_programId);
}

void CompositeCube::SetupRenderFrame(asdp::Time scanOutTime)
{
  glUseProgram(m_programId);
  glDisable(GL_CULL_FACE);
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

//==================================================================================================
// Objects needed by the CompositeCameras class.

static const GLchar* camerasVertexShader =
R"(#version 330 core
   layout (location = 0) in vec3 aPos;
   layout (location = 1) in vec2 aTexCoord;
   out vec2 TexCoord;
   uniform mat4 modelViewProjection;
   void main()
   {
      gl_Position = modelViewProjection * vec4(aPos, 1.0);
      TexCoord = vec2(aTexCoord.x, aTexCoord.y);
   })";

static const GLchar* camerasFragmentShader =
R"(#version 330 core
   out vec4 FragColor;
   in vec2 TexCoord;
   uniform sampler2D imageTexture;
   uniform sampler1D toneMapTexture;
   void main()
   {
      // Look up the intensity from the image texture and then use the tone map to get the color.
      float intensity = texture(imageTexture, TexCoord).r;
      FragColor = texture(toneMapTexture, intensity);
   })";

CompositeCameras::CompositeCameras(std::vector<CameraRenderInfo>& cameraRenderInfo, GLuint toneMaptexture)
  : Composite()
  , m_cameraRenderInfos(cameraRenderInfo)
  , m_toneMapTexture(toneMaptexture)
  , m_programId(0)
  , m_modelViewProjectionUniformId(0)
{
}

bool CompositeCameras::SetupRendering()
{
  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    std::cerr << "CompositeCameras::CompositeCameras(): Failed to initialize GLEW" << std::endl;
    return false;
  }

  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  GLuint vertexShaderId = glCreateShader(GL_VERTEX_SHADER);
  GLuint fragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);

  // vertex shader
  glShaderSource(vertexShaderId, 1, &camerasVertexShader, NULL);
  glCompileShader(vertexShaderId);
  checkShaderError(vertexShaderId, "Vertex shader compilation failed.");

  // fragment shader
  glShaderSource(fragmentShaderId, 1, &camerasFragmentShader, NULL);
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

  // Get the IDs for all of the uniform parameters we will want to change.
  m_modelViewProjectionUniformId = glGetUniformLocation(m_programId, "modelViewProjection");
  m_imageTextureId = glGetUniformLocation(m_programId, "imageTexture");
  m_toneMapTextureId = glGetUniformLocation(m_programId, "toneMapTexture");

  // Construct a Vertex Array Object for each camera that describes the positions and
  // texture coordinates along with the indices.  We'll also keep two vertex buffer objects,
  // one per camera, for the vertices and indices.
  for (auto const& cameraRenderInfo : m_cameraRenderInfos) {
    AddBufferObjects(cameraRenderInfo);
  }
  return true;
}

CompositeCameras::~CompositeCameras()
{
  for (size_t i = 0; i < m_cameraRenderInfos.size(); i++) {
    glDeleteBuffers(1, &m_vertexBufferObjects[i]);
    glDeleteBuffers(1, &m_indexBufferObjects[i]);
  }
  glDeleteProgram(m_programId);
}

static double radians(double degrees) {
  return degrees * M_PI / 180.0;
}

void CompositeCameras::AddBufferObjects(CameraRenderInfo const& cameraRenderInfo, size_t nx, size_t ny,
  GLfloat depth)
{
  double fnx = static_cast<GLfloat>(nx);
  double fny = static_cast<GLfloat>(ny);

  // Create the vertices including the texture coordinates.  Each entry will have
  // 5 floats: X, Y, Z, U, V.  We add entries that span the entire range, with one
  // more vertex in each dimension than there are quads.  We start from the lower-
  // left, move right, then move up at the end of each line.
  std::vector<GLfloat> vertices;
  for (size_t j = 0; j <= ny; j++) {
    for (size_t i = 0; i <= nx; i++) {
      // Compute the U and V normalized texture coordinates for the vertex in the range 0 to 1.
      // Because standard image textures have the origin at the upper left and OpenGL has it
      // at the lower right, we must invert the v texture coordinate.
      GLfloat u = i / fnx;
      GLfloat v = 1.0f - j / fny;

      // Compute the normalized X, Y, coordinates in the range -1 to 1.
      double xn = -1.0f + 2.0f * i / fnx;
      double yn = -1.0f + 2.0f * j / fny;

      // Compute the scaled X, Y coordinates for the four corners of the quad that place them
      // for a correctly-sized quad given the camera info to get them to scaled space.
      // The Z coordinate it along the negative Z axis at the specified depth.
      double xHalfWidth = tan(radians(cameraRenderInfo.m_fovDegrees[0]) / 2.0) * depth;
      double yHalfWidth = tan(radians(cameraRenderInfo.m_fovDegrees[1]) / 2.0) * depth;
      double xs = xn * xHalfWidth;
      double ys = yn * yHalfWidth;
      double zs = -depth;

      // Perform distortion correction on the X, Y coordinates to get to canonical view
      // space, which has a camera looking down -Z.  This provides us the location in the
      // canonical view space.  If we don't have a distortion model, we just use the X, Y, Z
      // coordinates as-is.
      std::array<double, 3> distPoint = std::array<double, 3>{xs, ys, zs};
      if (cameraRenderInfo.m_distortion != nullptr) {
        distPoint = cameraRenderInfo.m_distortion->MapPoint(distPoint);
      }
      double xc = distPoint[0];
      double yc = distPoint[1];
      double zc = distPoint[2];

      // Translate and rotate the X, Y, Z coordinates to match the camera center of projection
      // and viewing direction of this camera in the coordinate system of the camera cluster.
      // This will be the local helicopter coordinate system that maps +X helicopter from +X,
      // +Y helicopter from -Z, and +Z helicopter from +Y.
      double xh = xc;
      double yh = -zc;
      double zh = yc;

      // Rotate the points in the helicopter view space by the specified orientation change.
      glm::vec3 point(xh, yh, zh);
      glm::mat4 rotationX = glm::rotate(glm::mat4(1.0f),
        glm::radians(static_cast<GLfloat>(cameraRenderInfo.m_orientationDegrees[0])),
        glm::vec3(1.0f, 0.0f, 0.0f));
      glm::mat4 rotationY = glm::rotate(rotationX,
        glm::radians(static_cast<GLfloat>(cameraRenderInfo.m_orientationDegrees[1])),
        glm::vec3(0.0f, 1.0f, 0.0f));
      glm::mat4 rotation = glm::rotate(rotationY,
        glm::radians(static_cast<GLfloat>(cameraRenderInfo.m_orientationDegrees[2])),
        glm::vec3(0.0f, 0.0f, 1.0f));
      // Offset the points by the camera position in the helicopter view space.
      glm::mat4 total = glm::translate(rotation, glm::vec3(
        cameraRenderInfo.m_positionMeters[0], cameraRenderInfo.m_positionMeters[1], cameraRenderInfo.m_positionMeters[2]));
      glm::vec3 transformedPoint = glm::vec3(total  *glm::vec4(point, 1.0f));

      // Add the vertex
      vertices.push_back(transformedPoint[0]);
      vertices.push_back(transformedPoint[1]);
      vertices.push_back(transformedPoint[2]);
      vertices.push_back(u);
      vertices.push_back(v);
    }
  }

  // Create the indices for the triangles, three per triangle.
  std::vector<GLuint> indices;
  for (size_t j = 0; j < ny; j++) {
    for (size_t i = 0; i < nx; i++) {
      // Add the indices for the two triangles in the quad.
      size_t start = i + (nx+1) * j;
      indices.push_back(start);
      indices.push_back(start + 1);
      indices.push_back(start + (nx+1) + 1);

      indices.push_back(start);
      indices.push_back(start + (nx + 1) + 1);
      indices.push_back(start + (nx + 1));
    }
  }

  // Unbind any vertex array object.
  glBindVertexArray(0);

  // Create a vertex buffer object for the vertices.
  GLuint vertexBufferObject;
  glGenBuffers(1, &vertexBufferObject);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices[0]) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

  // Create a vertex buffer object for the indices.
  GLuint indexBufferObject;
  glGenBuffers(1, &indexBufferObject);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferObject);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0]) * indices.size(), indices.data(), GL_STATIC_DRAW);

  // Set up the vertex attributes for the vertex buffer object.
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (GLvoid*)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
  glEnableVertexAttribArray(1);

  // Save the vertex array object and the number of elements in the index buffer.
  m_vertexBufferObjects.push_back(vertexBufferObject);
  m_indexBufferObjects.push_back(indexBufferObject);
  m_numIndices.push_back(indices.size());
}

void CompositeCameras::SetupRenderFrame(asdp::Time scanOutTime)
{
  glUseProgram(m_programId);
  glDisable(GL_CULL_FACE);

  // Grab shared pointers to the camera textures to be used for all views in this frame.
  // There will be one entry per camera with the same vector index as the cameraRenderInfo.
  m_images.clear();
  for (auto const& cameraRenderInfo : m_cameraRenderInfos) {
    m_images.push_back(cameraRenderInfo.m_imageQueue->GetNewestImagePointer());
  }

  // Store the scan out time for use in rendering.
  m_scanOutTime = scanOutTime;
}

void CompositeCameras::RenderView(const float* modelViewProjection)
{
  // Set the model-view-projection matrix
  glUniformMatrix4fv(m_modelViewProjectionUniformId, 1, GL_FALSE, modelViewProjection);

  // Draw each camera, using the appropriate texture.
  for (size_t i = 0; i < m_cameraRenderInfos.size(); i++) {

    // If there is no texture, bind the default texture for the image to texture unit 0.
    GLuint texture = 0;
    if (m_images[i] != nullptr) {
      texture = m_images[i]->texture;
    }
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glUniform1i(m_imageTextureId, 0);

    // Bind the tone map texture to texture unit 1.
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_1D, m_toneMapTexture);
    glUniform1i(m_toneMapTextureId, 1);

    // Unbind any vertex array object.
    // We cannot use vertex array objects because we're potentially going to be called
    // from multiple OpenGL contexts in different threads and VAOs are not shared between
    // contexts.
    glBindVertexArray(0);

    // Enable the vertex attribute arrays we are going to use
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    // Draw the camera view using its vertex buffer objects.
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObjects[i]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (GLvoid*)0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBufferObjects[i]);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
    glDrawElements(GL_TRIANGLES, m_numIndices[i], GL_UNSIGNED_INT, 0);
  }
}

void CompositeCameras::TearDownRenderFrame()
{
  glUseProgram(0);
}

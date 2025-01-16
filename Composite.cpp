/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <string>
#include <iostream>
#include <algorithm>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
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

  // Set up the geometry for all of the views so the world is consistent across views.
  SetupRenderFrame(scanOutTime);

  // Render each view
  for (size_t eye = 0; eye < views.size(); eye++) {
    const ViewRenderInfo& view = views[eye];

    // Only set up the frame buffer and clear the buffers if we're the first eye or if the
    // eyes use different frame buffers or different color buffers.
    if ((eye == 0) || (views[eye].frameBuffer != views[0].frameBuffer) || (views[eye].colorBuffer != views[0].colorBuffer)) {
      // Bind the frame buffer and assign the appropriate textures.
      glBindFramebuffer(GL_FRAMEBUFFER, view.frameBuffer);
      if (view.frameBuffer != 0) {
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, view.colorBuffer, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, view.depthBuffer);
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
    glm::quat rotQuat;
    rotQuat.w = -view.orientation[0];
    rotQuat.x = -view.orientation[1];
    rotQuat.y = -view.orientation[2];
    rotQuat.z = -view.orientation[3];
    glm::mat4 ViewRotate = HelicopterRotateX * glm::toMat4(rotQuat);

    // Translate the view based on the specified viewpoint (negative due to world vs. camera).
    glm::mat4 ViewTranslate = glm::translate(ViewRotate,
      glm::vec3(-view.viewpoint[0], -view.viewpoint[1], -view.viewpoint[2]));

    // Compute the projection matrix from the ViewRenderInfo.
    double leftFrust = tan(glm::radians(view.leftHalfFOV)) * view.nearClip;
    double rightFrust = tan(glm::radians(view.rightHalfFOV)) * view.nearClip;
    double bottomFrust = tan(glm::radians(view.bottomHalfFOV)) * view.nearClip;
    double topFrust = tan(glm::radians(view.topHalfFOV)) * view.nearClip;
    glm::mat4 Projection = glm::frustum<float>(leftFrust, rightFrust, bottomFrust, topFrust,
      view.nearClip, view.farClip);
    glm::mat4 VP = Projection * ViewTranslate;

    // Call the derived-class method to render the geometry into this viewpoint.
    RenderView(scanOutTime, glm::value_ptr(VP));
  }

  // Unset things
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // Done with the data for a render frame.  If the code in this method requires the
  // frame rendering to have completed before it returns, it must call glFinish() or
  // use a synchronization object to ensure this.
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

  try {
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
  } catch (std::runtime_error& e) {
    std::cerr << "CompositeCube::SetupRendering(): " << e.what() << std::endl;
    return false;
  }

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

void CompositeCube::RenderView(asdp::Time scanOutTime, const float* viewProjection)
{
  // Set the model-view-projection matrix to the viewProjection matrix (no model) and draw the cube.
  glUniformMatrix4fv(m_modelViewProjectionUniformId, 1, GL_FALSE, viewProjection);
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

   mat4 axisAngleToMatrix(vec3 axis, float angle)
   {
      float c = cos(angle);
      float s = sin(angle);
      float t = 1.0 - c;

      float x = axis.x;
      float y = axis.y;
      float z = axis.z;

      mat4 mat = mat4(1.0);
      mat[0][0] = t * x * x + c;
      mat[0][1] = t * x * y - s * z;
      mat[0][2] = t * x * z + s * y;

      mat[1][0] = t * x * y + s * z;
      mat[1][1] = t * y * y + c;
      mat[1][2] = t * y * z - s * x;

      mat[2][0] = t * x * z - s * y;
      mat[2][1] = t * y * z + s * x;
      mat[2][2] = t * z * z + c;

      return mat;
   }

   layout (location = 0) in vec3 aPos;
   layout (location = 1) in vec2 aTexCoord;
   layout (location = 2) in float aVignetteGain;
   out vec2 TexCoord;
   out float vignetteGain;
   uniform mat4 viewProjection;
   uniform mat4 poseAdjust;   ///< Moves points in helicopter space to their capture-time positions.
   // The following are for the camera rotation and translation during the frame, and they are
   // in the helicopter coordinate system.
   uniform vec3 fVelocity;   ///< The change in position over a frame time from frame center
   uniform vec3 fAxis;       ///< The axis around which the camera is rotating during the frame
   uniform float fAngle;     ///< The angle of rotation around the axis during a frame time in radians
   void main()
   {
      // Determine the time within a frame that this vertex is being rendered.
      // The center vertex (Y texture coordinate 0.5) is at time 0, the top at -0.5, the bottom at 0.5.
      // Because the Y texture coordinate is 0 at the top, we need to invert it.
      float time = -(aTexCoord.y - 0.5);

      // Construct a rotation matrix for the camera's rotation around the axis during a frame time.
      mat4 delta = axisAngleToMatrix(fAxis, fAngle * time);

      // Add the scaled velocity as a translation to the position in this matrix.
      vec3 shift = fVelocity * time;
      delta[3][0] = shift.x;
      delta[3][1] = shift.y;
      delta[3][2] = shift.z;

      // Apply the matrices to the position to get the final projected position.
      // Perform the within-frame distortion first (it is in helicopter space), then the pose adjustment
      // (to previous helicopter space), and finally the view+projection.
      gl_Position = viewProjection * poseAdjust * delta * vec4(aPos, 1.0);

      // Pass the texture coordinate and vignette gain to the fragment shader.
      TexCoord = vec2(aTexCoord.x, aTexCoord.y);
      vignetteGain = aVignetteGain;
   })";

static const GLchar* camerasFragmentShader =
R"(#version 330 core
   out vec4 FragColor;
   in vec2 TexCoord;
   in float vignetteGain;
   uniform sampler2D imageTexture;
   uniform sampler1D toneMapTexture;
   uniform float offset;
   uniform float gain;
   void main()
   {
      // Look up the intensity from the image texture and then use the tone map to get the color.
      // Apply offset, gain, and vignette gain.  The texture sampler should be set to GL_CLAMP_TO_EDGE.
      float intensity = vignetteGain * gain * (offset + texture(imageTexture, TexCoord).r);
      FragColor = texture(toneMapTexture, intensity);
   })";

CompositeCameras::CompositeCameras(std::vector<CameraRenderInfo>& cameraRenderInfo, GLuint toneMaptexture,
  std::shared_ptr<PoseAdjuster> poseAdjuster, Time cameraFrameInterval,
  uint32_t renderOffsetMicroseconds, Time renderFrameInterval, RenderTimingInfo *renderTimingInfo)
  : Composite()
  , m_cameraRenderInfos(cameraRenderInfo)
  , m_toneMapTexture(toneMaptexture)
  , m_poseAdjuster(poseAdjuster)
  , m_cameraFrameInterval(cameraFrameInterval)
  , m_renderOffsetMicroseconds(renderOffsetMicroseconds)
  , m_renderFrameInterval(renderFrameInterval)
  , m_renderTimingInfo(renderTimingInfo)
  , m_programId(0)
  , m_viewProjectionUniformId(0)
  , m_poseAdjustUniformId(0)
  , m_fVelocityUniformID(0)
  , m_fAxisUniformID(0)
  , m_fAngleUniformID(0)
  , m_offsetUniformID(0)
  , m_gainUniformID(0)
  , m_globalExposureGain(cameraFrameInterval.seconds + cameraFrameInterval.microseconds * 1e-6)
  , m_imageTextureId(0)
  , m_toneMapTextureId(0)
{
}

bool CompositeCameras::SetupRendering()
{
  // Initialize GLEW in our context. It is okay to initialize it more than once.
  // NOTE: SetupRendering() is only called once for each object if it works, so we won't be initializing
  // GLEW every render frame here, only once per CompositeCameras object.
  glewExperimental = true;
  GLenum ret = glewInit();
  if (ret != GLEW_OK) {
    std::cerr << "CompositeCameras::CompositeCameras(): Failed to initialize GLEW: " << ret << std::endl;
    return false;
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  GLuint vertexShaderId = glCreateShader(GL_VERTEX_SHADER);
  GLuint fragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);

  try {
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
  } catch (std::runtime_error& e) {
    std::cerr << "CompositeCameras::SetupRendering(): " << e.what() << std::endl;
    return false;
  }

  // Get the IDs for all of the uniform parameters we will want to change.
  m_viewProjectionUniformId = glGetUniformLocation(m_programId, "viewProjection");
  m_poseAdjustUniformId = glGetUniformLocation(m_programId, "poseAdjust");
  m_fVelocityUniformID = glGetUniformLocation(m_programId, "fVelocity");
  m_fAxisUniformID = glGetUniformLocation(m_programId, "fAxis");
  m_fAngleUniformID = glGetUniformLocation(m_programId, "fAngle");
  m_offsetUniformID = glGetUniformLocation(m_programId, "offset");
  m_gainUniformID = glGetUniformLocation(m_programId, "gain");
  m_imageTextureId = glGetUniformLocation(m_programId, "imageTexture");
  m_toneMapTextureId = glGetUniformLocation(m_programId, "toneMapTexture");
  if (m_viewProjectionUniformId == -1 || m_poseAdjustUniformId == -1 || m_fVelocityUniformID == -1 ||
    m_fAxisUniformID == -1 || m_fAngleUniformID == -1 || m_imageTextureId == -1 || m_toneMapTextureId == -1 ||
    m_offsetUniformID == -1 || m_gainUniformID == -1) {
    std::cerr << "CompositeCameras::SetupRendering(): Failed to get uniform IDs" << std::endl;
    std::cerr << "  viewProjection: " << m_viewProjectionUniformId << std::endl;
    std::cerr << "  poseAdjust: " << m_poseAdjustUniformId << std::endl;
    std::cerr << "  fVelocity: " << m_fVelocityUniformID << std::endl;
    std::cerr << "  fAxis: " << m_fAxisUniformID << std::endl;
    std::cerr << "  fAngle: " << m_fAngleUniformID << std::endl;
    std::cerr << "  offset: " << m_offsetUniformID << std::endl;
    std::cerr << "  gain: " << m_gainUniformID << std::endl;
    std::cerr << "  imageTexture: " << m_imageTextureId << std::endl;
    std::cerr << "  toneMapTexture: " << m_toneMapTextureId << std::endl;
    return false;
  }

  // Construct a vertex and index buffer object for each camera that describes the positions and
  // texture coordinates along with the indices, along with a count of index buffer entries.  Make
  // the mesh if it has not already been filled in.
  for (auto &cameraRenderInfo : m_cameraRenderInfos) {
    if (cameraRenderInfo.m_mesh.nx == 0) {
      cameraRenderInfo.ComputePlanarCameraMeshInfo();
    }
    CreateBufferInfo(cameraRenderInfo, cameraRenderInfo.m_mesh);
  }
  return true;
}

CompositeCameras::~CompositeCameras()
{
  for (size_t i = 0; i < m_cameraBufferInfos.size(); i++) {
    glDeleteBuffers(1, &m_cameraBufferInfos[i].vertexBufferObject);
    glDeleteBuffers(1, &m_cameraBufferInfos[i].indexBufferObject);
  }
  glDeleteProgram(m_programId);
}

static double radians(double degrees) {
  return degrees * M_PI / 180.0;
}

void CameraRenderInfo::ComputePlanarCameraMeshInfo(size_t nx, size_t ny, GLfloat depth)
{
  // Lock the mutex to protect the mesh data.
  std::lock_guard<std::mutex> lock(m_meshMutex);

  // Pre-divide so we can use multiplications instead of divisions in the loop, which is faster.
  double fnxInv = 1 / static_cast<GLfloat>(nx);
  double fnyInv = 1 / static_cast<GLfloat>(ny);

  // Compute the scaled X, Y coordinates for the four corners of the quad that place them
  // for a correctly-sized quad given the camera info to get them to scaled space.
  // The Z coordinate is along the negative Z axis at the specified depth.
  double xHalfSpan = tan(radians(m_fovDegrees[0]) * 0.5) * depth;
  double yHalfSpan = tan(radians(m_fovDegrees[1]) * 0.5) * depth;

  // Rotate the points in the helicopter view space by the specified orientation change
  // to point them in the direction that the camera is looking.
  glm::mat4 rotationX = glm::rotate(glm::mat4(1.0f),
    glm::radians(static_cast<GLfloat>(m_orientationDegrees[0])),
    glm::vec3(1.0f, 0.0f, 0.0f));
  glm::mat4 rotationY = glm::rotate(rotationX,
    glm::radians(static_cast<GLfloat>(m_orientationDegrees[1])),
    glm::vec3(0.0f, 1.0f, 0.0f));
  glm::mat4 rotation = glm::rotate(rotationY,
    glm::radians(static_cast<GLfloat>(m_orientationDegrees[2])),
    glm::vec3(0.0f, 0.0f, 1.0f));

  // Create the vertices including the texture coordinates.  Each entry will have
  // 6 floats: X, Y, Z, U, V, vignette.  We add entries that span the entire range, with one
  // more vertex in each dimension than there are quads.  We start from the lower-
  // left, move right, then move up at the end of each line.
  std::vector<VertexInfo> vertices;
  for (size_t j = 0; j <= ny; j++) {

    for (size_t i = 0; i <= nx; i++) {
      // Compute the U and V normalized texture coordinates for the vertex in the range 0 to 1.
      // The normalized texture coordinates in the range 0 to 1 handle mapping the texture so that
      // the corners of the last pixels are at the edges of the quad.
      GLfloat u = i * fnxInv;
      GLfloat v = 1.0f - j * fnyInv;

      // Compute the normalized X, Y, coordinates in the range -1 to 1.
      double xn = -1.0f + 2.0f * i * fnxInv;
      double yn = -1.0f + 2.0f * j * fnyInv;

      // Compute the scaled X, Y coordinates for the four corners of the quad that place them
      // for a correctly-sized quad given the camera info to get them to scaled space.
      // The Z coordinate is along the negative Z axis at the specified depth.
      double xs = xn * xHalfSpan;
      double ys = yn * yHalfSpan;
      double zs = -depth;

      // Perform distortion correction on the X, Y coordinates to get to canonical view
      // space, which has a camera looking down -Z.  This provides us the location in the
      // canonical view space.  If we don't have a distortion model, we just use the X, Y, Z
      // coordinates as is.
      std::array<double, 3> distPoint = std::array<double, 3>{xs, ys, zs};
      if (m_distortion != nullptr) {
        distPoint = m_distortion->MapPoint(distPoint);
      }
      double& xc = distPoint[0];
      double& yc = distPoint[1];
      double& zc = distPoint[2];

      // Rotate the X, Y, Z coordinates to match the camera center of projection
      // and viewing direction of this camera in the coordinate system of the camera cluster.
      // This will be the local helicopter coordinate system that maps +X helicopter from +X,
      // +Y helicopter from -Z, and +Z helicopter from +Y.
      double xh = xc;
      double yh = -zc;
      double zh = yc;

      // Rotate the points in the helicopter view space by the specified orientation change
      // to point them in the direction that the camera is looking.
      glm::vec3 point(xh, yh, zh);
      glm::vec3 transformedPoint = glm::vec3(rotation * glm::vec4(point, 1.0f));

      // Add the vertex description, computing quantities as needed
      VertexInfo vertex;
      vertex.offset = transformedPoint;
      vertex.texCoord = glm::vec2(u, v);
      vertex.normalizedOffset = glm::normalize(transformedPoint);
      vertex.depth = glm::length(transformedPoint);
      vertex.vignetteGain = m_vignette->EvaluateAtPoint({ xn, yn });
      vertices.push_back(vertex);
    }
  }

  m_mesh.nx = nx;
  m_mesh.ny = ny;
  m_mesh.vertexInfo = vertices;
}

// NOTE: This must be called for each camera to produce the required buffers before rendering uses them.
void CompositeCameras::CreateBufferInfo(CameraRenderInfo const& cameraRenderInfo, MeshInfo const& mesh)
{
  // Create the vertices including the texture coordinates and vignette correction.
  std::vector<GLfloat> vertices;
  for (VertexInfo const &v : mesh.vertexInfo) {
    // Add the vertex description
    // Offset the points by the camera position in the helicopter view space.
    vertices.push_back(v.offset[0] + cameraRenderInfo.m_positionMeters[0]);
    vertices.push_back(v.offset[1] + cameraRenderInfo.m_positionMeters[1]);
    vertices.push_back(v.offset[2] + cameraRenderInfo.m_positionMeters[2]);
    vertices.push_back(v.texCoord[0]);
    vertices.push_back(v.texCoord[1]);
    vertices.push_back(v.vignetteGain);
  }

  // Create the indices for the triangles, three per triangle.
  std::vector<GLuint> indices;
  for (size_t j = 0; j < mesh.ny; j++) {
    for (size_t i = 0; i < mesh.nx; i++) {
      // Add the indices for the two triangles in the quad.
      size_t start = i + (mesh.nx+1) * j;
      indices.push_back(start);
      indices.push_back(start + 1);
      indices.push_back(start + (mesh.nx+1) + 1);

      indices.push_back(start);
      indices.push_back(start + (mesh.nx + 1) + 1);
      indices.push_back(start + (mesh.nx + 1));
    }
  }

  // Unbind any vertex array object, we won't be using these because they are not shared between contexts.
  glBindVertexArray(0);

  // Create a vertex buffer object for the vertices.
  GLuint vertexBufferObject;
  glGenBuffers(1, &vertexBufferObject);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices[0]) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);

  // Create a vertex buffer object for the indices.
  GLuint indexBufferObject;
  glGenBuffers(1, &indexBufferObject);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferObject);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0]) * indices.size(), indices.data(), GL_STATIC_DRAW);

  // Save the camera buffer information for this camera, filling in all of its elements.
  CameraBufferInfo cbi;
  cbi.mesh = mesh;
  cbi.vertexBufferObject = vertexBufferObject;
  cbi.indexBufferObject = indexBufferObject;
  cbi.numIndices = indices.size();
  m_cameraBufferInfos[cameraRenderInfo.m_ID] = cbi;
}

void CompositeCameras::UpdateVertexBuffer(CameraRenderInfo const& cameraRenderInfo)
{
  CameraBufferInfo const& cbi = m_cameraBufferInfos[cameraRenderInfo.m_ID];
  std::vector<GLfloat> vertices;

  {
    // Lock the mutex to protect the mesh data.
    std::lock_guard<std::mutex> lock(cameraRenderInfo.m_meshMutex);

    // Find the mesh information for this camera.
    MeshInfo const& mesh = cbi.mesh;

    // Create the vertices including the texture coordinates by scaling the normalized offsets
    // in the mesh by their current depth values and adding the camera center.
    vertices.reserve(6 * mesh.vertexInfo.size());
    for (VertexInfo const& v : mesh.vertexInfo) {
      // Add the vertex description
      // Offset the points by the camera position in the helicopter view space.
      vertices.push_back(v.normalizedOffset[0] * v.depth + cameraRenderInfo.m_positionMeters[0]);
      vertices.push_back(v.normalizedOffset[1] * v.depth + cameraRenderInfo.m_positionMeters[1]);
      vertices.push_back(v.normalizedOffset[2] * v.depth + cameraRenderInfo.m_positionMeters[2]);
      vertices.push_back(v.texCoord[0]);
      vertices.push_back(v.texCoord[1]);
      vertices.push_back(v.vignetteGain);
    }
  }

  // Unbind any vertex array object, we won't be using these because they are not shared between contexts.
  glBindVertexArray(0);

  // Update the vertex buffer object data with the vertices.
  glBindBuffer(GL_ARRAY_BUFFER, cbi.vertexBufferObject);
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices[0]) * vertices.size(), vertices.data());
}

static double TimeDiffMagnitude(asdp::Time t1, asdp::Time t2) {
  asdp::Time diff;
  if (t1 > t2) {
    diff = t1 - t2;
  } else {
    diff = t2 - t1;
  }
  return diff.seconds + diff.microseconds * 1.0e-6;
}

void CompositeCameras::SetupRenderFrame(asdp::Time scanOutTime)
{
  glUseProgram(m_programId);
  glDisable(GL_CULL_FACE);


  // To ensure that the set of images from all cameras are synchronized, we pull the first
  // two images from each queue and then select a set of consistent ones.
  std::vector< std::list< std::shared_ptr<ImageData> > > images;
  for (auto const& cameraRenderInfo : m_cameraRenderInfos) {
    images.push_back(cameraRenderInfo.m_imageQueue->LockNewestImages(2));
    if (images.back().size() != 2) {
      std::cerr << "Composite::SetupRenderFrame(): Could not get image pair, skipping frame" << std::endl;
      return;
    }
  }

  // Select the desired time, which is the one that will have consistent images across all cameras
  // for this frame, and consistent gaps between frames from frame to frame during replay (for live,
  // the synchronization of the camera triggers with the render time assures temporal consistency).
  asdp::Time desiredTime;
  if (m_renderOffsetMicroseconds == 0) {
    // Live: select by finding the time of the oldest image among the first (newest) image from
    // all cameras and then selecting from each pair the one whose time is closest to the
    // selected time.
    asdp::Time desiredTime = images[0].front()->imageCenterTime;
    for (size_t i = 1; i < images.size(); i++) {
      if (images[i].front()->imageCenterTime < desiredTime) {
        desiredTime = images[i].front()->imageCenterTime;
      }
    }
  } else {
    // Stored: Select by adding the frame interval to the last desired time and then verifying that
    // it is close enough to the requested offset from the scan-out time (within a frame time),
    // replacing it if not.
    desiredTime = m_lastFrameTime + m_renderFrameInterval;
    Time offset = asdp::Time(m_renderOffsetMicroseconds / 1000000, m_renderOffsetMicroseconds % 1000000);
    Time requestedTime = scanOutTime - offset;
    if (scanOutTime < offset) {
      requestedTime = Time(0, 0);
    }
    double diff = TimeDiffMagnitude(desiredTime, requestedTime);
    if (diff > m_renderFrameInterval.seconds * m_renderFrameInterval.microseconds * 1e-6) {
      desiredTime = requestedTime;
    }
  }

  // Find the image from each pair that is closest to the desired time.  Push it into the m_images
  // array and return the other image to the queue.
  for (size_t i = 0; i < images.size(); i++) {
    double diff0 = TimeDiffMagnitude(images[i].front()->imageCenterTime, desiredTime);
    double diff1 = TimeDiffMagnitude(images[i].back()->imageCenterTime, desiredTime);

    if (diff0 < diff1) {
      m_images.push_back(images[i].front());
      m_cameraRenderInfos[i].m_imageQueue->UnlockImage(images[i].back());
    } else {
      m_images.push_back(images[i].back());
      m_cameraRenderInfos[i].m_imageQueue->UnlockImage(images[i].front());
    }
  }

  // If we have a RenderTimingInfo object, fill in the times for each camera.
  if (m_renderTimingInfo != nullptr) {
    for (size_t i = 0; i < m_images.size(); i++) {
      m_renderTimingInfo->cameras[i].centerRenderTimes.push_back(m_images[i]->imageCenterTime);
    }
  }

  // Find the average of all selected image times to use for estimating future frame times.
  double sumSeconds = 0.0;
  for (size_t i = 0; i < m_images.size(); i++) {
    sumSeconds += m_images[i]->imageCenterTime.seconds + m_images[i]->imageCenterTime.microseconds / 1e6;
  }
  double averageSeconds = sumSeconds / m_images.size();
  m_lastFrameTime = asdp::Time(static_cast<int>(averageSeconds),
    static_cast<int>((averageSeconds - static_cast<int>(averageSeconds)) * 1e6));
}

void CompositeCameras::RenderView(asdp::Time scanOutTime, const float* viewProjection)
{
  // Find the frame time in floating point seconds.
  float frameTime = m_cameraFrameInterval.seconds + m_cameraFrameInterval.microseconds * 1.0e-6;

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
  glEnableVertexAttribArray(2);

  // Set the matrices and uniform parameters that are the same for all cameras
  glUniformMatrix4fv(m_viewProjectionUniformId, 1, GL_FALSE, viewProjection);

  // Compute a product of gain and exposure for each camera and then use this set to
  // determine a global to apply across all cameras, which will handle them auto-gaining
  // or auto-exposing over time and will bring them all into a consistent range.
  std::vector<float> scales;
  for (size_t c = 0; c < m_cameraRenderInfos.size(); c++) {
    float scale = 1.0f;
    float exposure = 0;
    if (m_images[c] != nullptr) { exposure = m_images[c]->exposure; }
    if (exposure != 0) {
      scale *= exposure;
    }
    float gain = 0;
    if (m_images[c] != nullptr) { gain = m_images[c]->gain; }
    if (gain != 0) {
      scale *= gain;
    }
    scales.push_back(scale);
  }
  // Select the median of the scales to use as the global gain.  Then blend it with the
  // stored one to smooth out changes.
  if (scales.size()) {
    std::sort(scales.begin(), scales.end());
    float globalExposureGain = scales[scales.size() / 2];
    m_globalExposureGain = 0.9 * m_globalExposureGain + 0.1 * globalExposureGain;
  }

  // Draw each camera, using the appropriate texture.
  for (size_t c = 0; c < m_cameraRenderInfos.size(); c++) {

    uint16_t cameraID = m_cameraRenderInfos[c].m_ID;

    // If there is no texture, bind the default texture for the image to texture unit 0.
    // Otherwise, bind the stored texture.
    GLuint texture = 0;
    if (m_images[c] != nullptr) {
      texture = m_images[c]->texture;
    }
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glUniform1i(m_imageTextureId, 0);

    // Adjust for helicopter motion changes from image acquisition to scan-out.
    // The camera points are in the helicopter coordinate system, so we need to adjust
    // from where they are (canonical position at render time) to where they were at
    // image acquisition.
    glm::dmat4 shiftPoints = glm::dmat4(1.0);
    if (scanOutTime > Time(0, m_renderOffsetMicroseconds)) {
      Time imageTime = {};
      if (m_images[c] != nullptr) { imageTime = m_images[c]->imageCenterTime; }
      shiftPoints = m_poseAdjuster->GetTransform(scanOutTime - Time(0, m_renderOffsetMicroseconds),
        imageTime);
    }
    const double* data = glm::value_ptr(shiftPoints);
    float fShift[16];
    for (size_t i = 0; i < 16; i++) {
      fShift[i] = static_cast<float>(data[i]);
    }

    // Construct the differential shift matrix to adjust the camera points to the scan-out time.
    // These must all be in the helicopter coordinate system but scaled to a single frame time.
    PoseAdjuster::VelocityEstimate velocity = m_poseAdjuster->EstimateVelocity(m_images[c]->imageCenterTime);

    std::array<GLfloat, 3> fVelocity = { velocity.vel[0] * frameTime, velocity.vel[1] * frameTime,
                                         velocity.vel[2] * frameTime };

    std::array<GLfloat, 3> fAxis = { velocity.axis[0], velocity.axis[1], velocity.axis[2] };
    GLfloat fAngle = velocity.angleRad * frameTime;

    // Set the matrices and uniform parameters specific to this camera
    glUniformMatrix4fv(m_poseAdjustUniformId, 1, GL_FALSE, fShift);
    glUniform3fv(m_fVelocityUniformID, 1, fVelocity.data());
    glUniform3fv(m_fAxisUniformID, 1, fAxis.data());
    glUniform1f(m_fAngleUniformID, fAngle);
    float offset, gain;
    m_cameraRenderInfos[c].GetColorOffsetGain(offset, gain);
    // Scale offset by the maximum color value to get it into 0-1 range.
    offset /= 65535.0f;
    // Scale gain by image exposure and gain if they are nonzero.  We divide by each -- higher gain
    // and longer exposure both brighten the values in the pixels, so we must darken the image to mix
    // with other cameras.
    float exposureValue = 0;
    if (m_images[c] != nullptr) { exposureValue = m_images[c]->exposure; }
    if (exposureValue != 0) {
      gain /= exposureValue;
    }
    float gainValue = 0;
    if (m_images[c] != nullptr) { gainValue = m_images[c]->gain; }
    if (gainValue != 0) {
      gain /= gainValue;
    }
    // Then rescale by the global exposure gain to bring all cameras into a consistent range.
    if (m_globalExposureGain > 0) {
      gain *= m_globalExposureGain;
    }
    glUniform1f(m_offsetUniformID, offset);
    glUniform1f(m_gainUniformID, gain);

    // Draw the camera using its vertex buffer objects after specifying its layout.
    glBindBuffer(GL_ARRAY_BUFFER, m_cameraBufferInfos[cameraID].vertexBufferObject);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(5 * sizeof(GLfloat)));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_cameraBufferInfos[cameraID].indexBufferObject);
    glDrawElements(GL_TRIANGLES, m_cameraBufferInfos[cameraID].numIndices, GL_UNSIGNED_INT, 0);

    // Unbind the camera image from its texture unit
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  // Unbind the tone map texture from its texture unit
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_1D, 0);
}

void CompositeCameras::TearDownRenderFrame()
{
  // Make sure we've finished using our textures before returning them.
  glFinish();
  for (size_t i = 0; i < m_cameraRenderInfos.size(); i++) {
    CameraRenderInfo const& CRI = m_cameraRenderInfos[i];
    if (m_images[i] != nullptr) {
      CRI.m_imageQueue->UnlockImage(m_images[i]);
    }
  }
  m_images.clear();
  glUseProgram(0);
}

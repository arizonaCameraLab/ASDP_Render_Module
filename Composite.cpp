/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
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

//======================================
// Added by Sang Yoon to share the head orientation information of detailed view window
//   with the overview window.
// This information is used for drawing a rectangle in the overview window
//   showing the user's head orientation of detailed view window (DrawHeadOrientation()).
// Note that the mutually exclusive access to the variables between the composite submodules
//   associated with overview and detailed view windows is controlled using a mutex (overview_mutex)
//   in Composite::Render() and DrawHeadOrientation(). In the composite submodule associated with
//   the detailed view window, the head orientation information of the detailed view window is stored
//   to the global variables, and in the composite submodule associated with the overview window,
//   the values of the global variables are read.
std::mutex g_overview_mutex;
glm::mat4 g_detailed_view_translate;
float g_detailed_view_leftf;
float g_detailed_view_rightf;
float g_detailed_view_topf;
float g_detailed_view_bottomf;
float g_detailed_view_nearf;
//======================================

//======================================
// Added by Sang Yoon to fix a bug in drawing a line where the cylindrical projection is used
glm::mat4 g_overview_translate; // model-view matrix of overview window; copied in the Render() method and used in the DrawHeadOrientation() method
//======================================

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
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, view.depthBuffer, 0);
        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE) {
          std::cerr << "Composite::Render(): Frame buffer is not complete" << std::endl;
        }
      }

      if (m_doClear) {
        // Clear the buffers.  The clear color is sky blue to distingiush from a camera with a black texture.
        glClearColor(0.6f, 0.8f, 1.0f, 1.0f);
        GLbitfield clearBits = 0;
        if ((view.frameBuffer == 0) || (view.colorBuffer != 0)) { clearBits |= GL_COLOR_BUFFER_BIT; }
        if ((view.frameBuffer == 0) || (view.depthBuffer != 0)) { clearBits |= GL_DEPTH_BUFFER_BIT; }
        glClear(clearBits);
      }
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
    rotQuat.w = view.orientation[0];
    rotQuat.x = view.orientation[1];
    rotQuat.y = view.orientation[2];
    rotQuat.z = view.orientation[3];
    rotQuat = glm::inverse(rotQuat);
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

    //======================================
    // Added by Sang Yoon to pass the information about viewing frustum and head pose of the detailed view window
    // to the composite submodule for the overview window.
    // This information is used for drawing a rectangle showing the head orientation of detailed view window in the overview window
    if (eye == 0 && m_detailed_view) {
        std::lock_guard<std::mutex> lock(g_overview_mutex);
        g_detailed_view_translate = ViewTranslate;
        g_detailed_view_leftf = leftFrust;
        g_detailed_view_rightf = rightFrust;
        g_detailed_view_topf = topFrust;
        g_detailed_view_bottomf = bottomFrust;
        g_detailed_view_nearf = view.nearClip;
    }
    //======================================

    // Call the derived-class method to render the geometry into this viewpoint.

    //======================================
    // Revised by Sang Yoon to support the cylindrical projection
    // Original:
    //RenderView(scanOutTime, glm::value_ptr(VP));
    // Revised:
    RenderView(scanOutTime, glm::value_ptr(VP), glm::value_ptr(ViewTranslate), view.leftHalfFOV, view.rightHalfFOV, view.bottomHalfFOV, view.topHalfFOV, view.nearClip, view.farClip);
    //======================================

    //======================================
    // Added by Sang Yoon to fix a bug in drawing a line where the cylindrical projection is used
    if (m_overview)
        g_overview_translate = ViewTranslate; // This matrix is used in DrawHeadOrientation() method.
    //======================================

    //======================================
    // Added by Sang Yoon to draw a rectangle to show the head orientation of detailed view window in the overview window
    // Note that in drawing this rectangle the shiftPoints or PoseAdjuster is not considered.
    if (m_overview)
        DrawHeadOrientation(view.farClip, view.width); // view.farClip is used for determining a radius of a big sphere.
    // view.width (screen width) is used for determining the line thickness of rectangle.
    //======================================
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

/// @brief Helper class that handles defining and drawing a cube.
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

   //======================================
   // Added by Sang Yoon to support a cylindrical projection
   uniform int useCP;
   uniform float lh_hfov_rad;
   uniform float rh_hfov_rad;
   uniform float bh_vfov_rad;
   uniform float th_vfov_rad;
   uniform float near;
   uniform float far;
   uniform mat4 modelViewMatrix;
   //======================================

   void main()
   {
      //======================================
      // Revised by Sang Yoon to support a cylindrical projection
      // Original: gl_Position = modelViewProjection * vec4(position,1);
      // Revised:
      if (useCP == 0) {
        gl_Position = modelViewProjection * vec4(position,1);
      } else {
        vec4 p = modelViewMatrix * vec4(position, 1.0);

        float length_xz = length(p.xz);
        float theta_x = atan(p.x, -p.z); // angle around y axis (angle in horizontal direction)
        float theta_y = atan(p.y, length_xz); // angle around bent x axis (angle in vertical direction)

        gl_Position = vec4((theta_x - lh_hfov_rad)/(rh_hfov_rad - lh_hfov_rad) * 2.0 - 1.0,
                           (theta_y - bh_vfov_rad)/(th_vfov_rad - bh_vfov_rad) * 2.0 - 1.0,
                           (length_xz - near)/(far - near) * 2.0 - 1.0,
                           1.0);
      }
      //======================================

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

  //======================================
  // Added by Sang Yoon to pass the parameters for cylindrical projection to GPU
  // (separately transfer horizontal FOV, vertical FOV, near, far, projection matrix, and model view matrix to the vertex shader).
  , m_useCPUniformId(0)
  , m_lh_hfovUniformId(0)
  , m_rh_hfovUniformId(0)
  , m_bh_vfovUniformId(0)
  , m_th_vfovUniformId(0)
  , m_nearUniformId(0)
  , m_farUniformId(0)
  , m_modelViewUniformId(0)
  //======================================
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

  //======================================
  // Added by Sang Yoon to pass the parameters used in the cylindrical projection to the vertex shader
  m_useCPUniformId = glGetUniformLocation(m_programId, "useCP");
  m_lh_hfovUniformId = glGetUniformLocation(m_programId, "lh_hfov_rad");
  m_rh_hfovUniformId = glGetUniformLocation(m_programId, "rh_hfov_rad");
  m_bh_vfovUniformId = glGetUniformLocation(m_programId, "bh_vfov_rad");
  m_th_vfovUniformId = glGetUniformLocation(m_programId, "th_vfov_rad");
  m_nearUniformId = glGetUniformLocation(m_programId, "near");
  m_farUniformId = glGetUniformLocation(m_programId, "far");
  m_modelViewUniformId = glGetUniformLocation(m_programId, "modelViewMatrix");
  //======================================

  // Make our geometry object, which will draw itself.  On the XSight, make it monochrome.
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

//======================================
// Revised by Sang Yoon to support the cylindrical projection
// The arguments used for the cylindrical projection are added: modelViewMatrix, hFOVf, vFOVf, nearf, and farf.
void CompositeCube::RenderView(asdp::Time scanOutTime, const float* viewProjection,
  const float* modelViewMatrix, const float lh_hFOVf, const float rh_hFOVf,
  const float bh_vFOVf, const float th_vFOVf, const float nearf, const float farf)
{
    if (!m_CP_enabled) // If the flag for cylindrical projection is not enabled, use the perspective projection
        // (following the original execution flow of RenderView()).
    {
        // Set the model-view-projection matrix and draw the cube.
        glUniformMatrix4fv(m_modelViewProjectionUniformId, 1, GL_FALSE, viewProjection);
        glUniform1i(m_useCPUniformId, 0);
    }
    else // If the flag for cylindrical projection is enabled, use the cylindrical projection.
    {
        glUniform1i(m_useCPUniformId, 1);
        glUniform1f(m_lh_hfovUniformId, lh_hFOVf * M_PI / 180.0);
        glUniform1f(m_rh_hfovUniformId, rh_hFOVf * M_PI / 180.0);
        glUniform1f(m_bh_vfovUniformId, bh_vFOVf * M_PI / 180.0);
        glUniform1f(m_th_vfovUniformId, th_vFOVf * M_PI / 180.0);
        glUniform1f(m_nearUniformId, nearf);
        glUniform1f(m_farUniformId, farf);
        glUniformMatrix4fv(m_modelViewUniformId, 1, GL_FALSE, modelViewMatrix);
    }
    m_roomCube->draw();
}
//======================================

void CompositeCube::TearDownRenderFrame()
{
  glUseProgram(0);
}

//======================================
// Added by Sang Yoon to draw a rectangle to show the head orientation of detailed view window in the overview window
void CompositeCube::DrawHeadOrientation(float view_farf, int screen_width)
{
    // Do nothing for CompositeCube.
}
//======================================

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
   out float depthColor;
   uniform mat4 viewProjection;
   uniform mat4 poseAdjust;   ///< Moves points in helicopter space to their capture-time positions.
   // The following are for the camera rotation and translation during the frame, and they are
   // in the helicopter coordinate system.
   uniform vec3 fVelocity;   ///< The change in position over a frame time from frame center
   uniform vec3 fAxis;       ///< The axis around which the camera is rotating during the frame
   uniform float fAngle;     ///< The angle of rotation around the axis during a frame time in radians
   uniform float depthScale; ///< If this is >= 0, scales the depth by this amount and sends to fragment shader.

   //======================================
   // Added by Sang Yoon to support a cylindrical projection
   uniform int useCP;
   uniform float lh_hfov_rad;
   uniform float rh_hfov_rad;
   uniform float bh_vfov_rad;
   uniform float th_vfov_rad;
   uniform float near;
   uniform float far;
   uniform mat4 modelViewMatrix;
   //======================================

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

      //======================================
      // Revised by Sang Yoon to support a cylindrical projection
      // Original: gl_Position = viewProjection * poseAdjust * delta * vec4(aPos, 1.0);
      // Revised:
      if (useCP == 0) {
        gl_Position = viewProjection * poseAdjust * delta * vec4(aPos, 1.0);
      } else {
        //======================================
        // Revised by Sang Yoon to fix a bug in drawing a line where the cylindrical projection is used
        // Original: vec4 p = modelViewMatrix * poseAdjust * delta * vec4(aPos, 1.0);
        // Revised:
        vec4 p = modelViewMatrix * vec4(aPos, 1.0);
        //======================================

        float length_xz = length(p.xz);
        float theta_x = atan(p.x, -p.z); // angle around y axis (angle in horizontal direction)
        float theta_y = atan(p.y, length_xz); // angle around bent x axis (angle in vertical direction)

        gl_Position = vec4((theta_x - lh_hfov_rad)/(rh_hfov_rad - lh_hfov_rad) * 2.0 - 1.0,
                           (theta_y - bh_vfov_rad)/(th_vfov_rad - bh_vfov_rad) * 2.0 - 1.0,
                           (length_xz - near)/(far - near) * 2.0 - 1.0,
                           1.0);
      }
      //======================================

      // Pass the texture coordinate and vignette gain to the fragment shader.
      TexCoord = vec2(aTexCoord.x, aTexCoord.y);
      vignetteGain = aVignetteGain;

      // If we are scaling the depth, do it here. Otherwise, send -1
      depthColor = depthScale >= 0.0 ? gl_Position.z * depthScale : -1.0f;
   })";

static const GLchar* camerasFragmentShader =
R"(#version 330 core
   out vec4 FragColor;
   in vec2 TexCoord;
   in float vignetteGain;
   in float depthColor;
   uniform sampler2D imageTexture;
   uniform sampler1D toneMapTexture;
   uniform float offset;
   uniform float gain;
   uniform float depthScale; ///< If this is >= 0, scales the depth by this amount and sends to fragment shader.
   void main()
   {
      if (depthScale >= 0.0) {
        // If the depth value has been set to a non-negative value, use it as the color.
        FragColor = vec4(depthColor, depthColor, depthColor, 1.0);
      } else {
        // Look up the intensity from the image texture and then use the tone map to get the color.
        // Apply offset, gain, and vignette gain.  The texture sampler should be set to GL_CLAMP_TO_EDGE.
        float intensity = vignetteGain * gain * (offset + texture(imageTexture, TexCoord).r);
        FragColor = texture(toneMapTexture, intensity);
      }
   })";

CompositeCameras::CompositeCameras(std::vector< std::shared_ptr<CameraRenderInfo> >& cameraRenderInfo, GLuint toneMaptexture,
  std::shared_ptr<PoseAdjuster> poseAdjuster, Time cameraFrameInterval,
  uint32_t renderOffsetMicroseconds, Time renderFrameInterval, RenderTimingInfo *renderTimingInfo,
  std::shared_ptr<asdp::render::RangeEstimator> rangeEstimator,
  double defaultStaticDepth)
  : Composite()
  , m_cameraRenderInfos(cameraRenderInfo)
  , m_toneMapTexture(toneMaptexture)
  , m_poseAdjuster(poseAdjuster)
  , m_cameraFrameInterval(cameraFrameInterval)
  , m_renderOffsetMicroseconds(renderOffsetMicroseconds)
  , m_renderFrameInterval(renderFrameInterval)
  , m_renderTimingInfo(renderTimingInfo)
  , m_rangeEstimator(rangeEstimator)
  , m_defaultStaticDepth(defaultStaticDepth)
  , m_programId(0)
  , m_viewProjectionUniformId(0)
  , m_poseAdjustUniformId(0)
  , m_fVelocityUniformID(0)
  , m_fAxisUniformID(0)
  , m_fAngleUniformID(0)
  , m_offsetUniformID(0)
  , m_gainUniformID(0)
  , m_depthScaleUniformID(0)
  , m_globalExposureGain(cameraFrameInterval.seconds + cameraFrameInterval.microseconds * 1e-6)
  , m_imageTextureId(0)
  , m_toneMapTextureId(0)

  //======================================
  // Added by Sang Yoon to pass the parameters for cylindrical projection to GPU
  // (separately transfer horizontal FOV, vertical FOV, near, far, projection matrix, and model view matrix to the vertex shader).
  , m_useCPUniformId(0)
  , m_lh_hfovUniformId(0)
  , m_rh_hfovUniformId(0)
  , m_bh_vfovUniformId(0)
  , m_th_vfovUniformId(0)
  , m_nearUniformId(0)
  , m_farUniformId(0)
  , m_modelViewUniformId(0)
  //======================================

  //======================================
  // Added by Sang Yoon to specificy the color of retangle indicating the head orientation of detailed view in the overview window
  , m_head_orientation_colorTexture(0)
  , m_head_orientation_toneMapTexture(0)
  //======================================
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
    std::cerr << "CompositeCameras::SetupRendering(): Failed to initialize GLEW: " << ret << std::endl;
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
  m_depthScaleUniformID = glGetUniformLocation(m_programId, "depthScale");
  m_imageTextureId = glGetUniformLocation(m_programId, "imageTexture");
  m_toneMapTextureId = glGetUniformLocation(m_programId, "toneMapTexture");
  if (m_viewProjectionUniformId == -1 || m_poseAdjustUniformId == -1 || m_fVelocityUniformID == -1 ||
    m_fAxisUniformID == -1 || m_fAngleUniformID == -1 || m_imageTextureId == -1 || m_toneMapTextureId == -1 ||
    m_offsetUniformID == -1 || m_gainUniformID == -1 || m_depthScaleUniformID == -1) {
    std::cerr << "CompositeCameras::SetupRendering(): Failed to get uniform IDs" << std::endl;
    std::cerr << "  viewProjection: " << m_viewProjectionUniformId << std::endl;
    std::cerr << "  poseAdjust: " << m_poseAdjustUniformId << std::endl;
    std::cerr << "  fVelocity: " << m_fVelocityUniformID << std::endl;
    std::cerr << "  fAxis: " << m_fAxisUniformID << std::endl;
    std::cerr << "  fAngle: " << m_fAngleUniformID << std::endl;
    std::cerr << "  offset: " << m_offsetUniformID << std::endl;
    std::cerr << "  depthScale: " << m_depthScaleUniformID << std::endl;
    std::cerr << "  gain: " << m_gainUniformID << std::endl;
    std::cerr << "  imageTexture: " << m_imageTextureId << std::endl;
    std::cerr << "  toneMapTexture: " << m_toneMapTextureId << std::endl;
    return false;
  }

  //======================================
  // Added by Sang Yoon to pass the parameters used for the cylindrical projection to the vertex shader
  m_useCPUniformId = glGetUniformLocation(m_programId, "useCP");
  m_lh_hfovUniformId = glGetUniformLocation(m_programId, "lh_hfov_rad");
  m_rh_hfovUniformId = glGetUniformLocation(m_programId, "rh_hfov_rad");
  m_bh_vfovUniformId = glGetUniformLocation(m_programId, "bh_vfov_rad");
  m_th_vfovUniformId = glGetUniformLocation(m_programId, "th_vfov_rad");
  m_nearUniformId = glGetUniformLocation(m_programId, "near");
  m_farUniformId = glGetUniformLocation(m_programId, "far");
  m_modelViewUniformId = glGetUniformLocation(m_programId, "modelViewMatrix");
  if (m_useCPUniformId == -1 || m_lh_hfovUniformId == -1 || m_rh_hfovUniformId == -1 || m_bh_vfovUniformId == -1 ||
      m_th_vfovUniformId == -1 || m_nearUniformId == -1 || m_farUniformId == -1 || m_modelViewUniformId == -1) {
      std::cerr << "CompositeCameras::SetupRendering(): Failed to get uniform IDs used for cylindrical projection" << std::endl;
      std::cerr << "  useCP: " << m_useCPUniformId << std::endl;
      std::cerr << "  lh_hfov_rad: " << m_lh_hfovUniformId << std::endl;
      std::cerr << "  rh_hfov_rad: " << m_rh_hfovUniformId << std::endl;
      std::cerr << "  bh_vfov_rad: " << m_bh_vfovUniformId << std::endl;
      std::cerr << "  th_vfov_rad: " << m_th_vfovUniformId << std::endl;
      std::cerr << "  near: " << m_nearUniformId << std::endl;
      std::cerr << "  far: " << m_farUniformId << std::endl;
      std::cerr << "  modelViewMatrix: " << m_modelViewUniformId << std::endl;
      return false;
  }
  //======================================

  // Construct a vertex and index buffer object for each camera that describes the positions and
  // texture coordinates along with the indices, along with a count of index buffer entries.  Make
  // the mesh if it has not already been filled in.
  for (auto &cameraRenderInfo : m_cameraRenderInfos) {
    if (cameraRenderInfo->m_mesh.nx == 0) {
      cameraRenderInfo->ComputePlanarCameraMeshInfo(100, 100, m_defaultStaticDepth);
    }
    CreateBufferInfo(*cameraRenderInfo, cameraRenderInfo->m_mesh);
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

// NOTE: This must be called for each camera to produce the required buffers before rendering uses them.
void CompositeCameras::CreateBufferInfo(CameraRenderInfo const& cameraRenderInfo, MeshInfo const& mesh)
{
  // Lock the mutex to protect the mesh data.
  std::lock_guard<std::mutex> lock(cameraRenderInfo.m_meshMutex);

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
  cbi.vertexBufferObject = vertexBufferObject;
  cbi.indexBufferObject = indexBufferObject;
  cbi.numIndices = indices.size();
  m_cameraBufferInfos[cameraRenderInfo.m_ID] = cbi;
}

void CompositeCameras::UpdateVertexBuffer(CameraRenderInfo const& cameraRenderInfo)
{
  // Lock the mutex to protect the mesh data.
  std::lock_guard<std::mutex> lock(cameraRenderInfo.m_meshMutex);

  CameraBufferInfo const& cbi = m_cameraBufferInfos[cameraRenderInfo.m_ID];
  std::vector<GLfloat> vertices;

  // Find the mesh information for this camera.
  MeshInfo const& mesh = cameraRenderInfo.m_mesh;

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

  // Figure out how many frames we must grab to cover the requested render-ahead time.
  // Grab an additional one to handle slight frame shifts.
  size_t framesToGrab = 1 + static_cast<size_t>(m_renderOffsetMicroseconds /
    (m_cameraFrameInterval.seconds * 1e6 + m_cameraFrameInterval.microseconds));

  // To ensure that the set of images from all cameras are synchronized, we pull the first
  // two images from each queue and then select a set of consistent ones.
  std::vector< std::list< std::shared_ptr<ImageData> > > images;
  for (auto const& cameraRenderInfo : m_cameraRenderInfos) {
    images.push_back(cameraRenderInfo->m_imageQueue->LockNewestImages(framesToGrab));
    if (images.back().size() != framesToGrab) {
      std::cerr << "Composite::SetupRenderFrame(): Could not get all needed images, skipping frame" << std::endl;
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
    desiredTime = images[0].front()->imageCenterTime;
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

  // Find the image from each list that is closest to the desired time.  Push it into the m_images
  // array and return the other images+/ to the queue.
  for (size_t i = 0; i < images.size(); i++) {
    auto& imList = images[i];
    auto best = imList.begin();
    double bestDiff = TimeDiffMagnitude((*best)->imageCenterTime, desiredTime);
    for (auto it = imList.begin(); it != imList.end(); ++it) {
      double diff = TimeDiffMagnitude((*it)->imageCenterTime, desiredTime);
      if (diff < bestDiff) {
        best = it;
        bestDiff = diff;
      }
    }

    for (auto it = imList.begin(); it != imList.end(); ++it) {
      if (it == best) {
        // Use this image
        m_images.push_back(*it);
      } else {
        // Unlock the images that are not selected.
        m_cameraRenderInfos[i]->m_imageQueue->UnlockImage(*it);
      }
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

//======================================
// Revised by Sang Yoon to support the cylindrical projection
// Original: void CompositeCameras::RenderView(asdp::Time scanOutTime, const float* viewProjection)
// Revised:
void CompositeCameras::RenderView(asdp::Time scanOutTime, const float* viewProjection, const float* modelViewMatrix, const float lh_hFOVf, const float rh_hFOVf, const float bh_vFOVf, const float th_vFOVf, const float nearf, const float farf)
//======================================
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

  //======================================
  // Added by Sang Yoon to support the cylindrical projection
  if (!m_CP_enabled)
    glUniform1i(m_useCPUniformId, 0);
  else
  {
    glUniform1i(m_useCPUniformId, 1);
    glUniform1f(m_lh_hfovUniformId, lh_hFOVf * M_PI/180.0);
    glUniform1f(m_rh_hfovUniformId, rh_hFOVf * M_PI/180.0);
    glUniform1f(m_bh_vfovUniformId, bh_vFOVf * M_PI/180.0);
    glUniform1f(m_th_vfovUniformId, th_vFOVf * M_PI/180.0);
    glUniform1f(m_nearUniformId, nearf);
    glUniform1f(m_farUniformId, farf);
    glUniformMatrix4fv(m_modelViewUniformId, 1, GL_FALSE, modelViewMatrix);
  }
  //======================================

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
    // Ignore cameras that don't report
    if ((gain != 0) && (exposure != 0)) {
      scales.push_back(scale);
    }
  }
  if (scales.size() == 0) {
    // If we have no cameras reporting, just set the scale to 1.
    scales.push_back(1);
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

    uint16_t cameraID = m_cameraRenderInfos[c]->m_ID;

    // If there is no texture, bind the default texture for the image to texture unit 0.
    // Otherwise, bind the stored texture.
    GLuint texture = 0;
    if (m_images[c] != nullptr) {
      texture = m_images[c]->texture;
    }
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glUniform1i(m_imageTextureId, 0);

    // Adjust for helicopter motion from image acquisition to scan-out.
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
    m_cameraRenderInfos[c]->GetColorOffsetGain(offset, gain);
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
    if (m_rangeEstimator) {
      // Get the min and max intensity values and determine the gain and offset to apply to map
      // the specified minVal and maxVal to 0 and 1.
      // The current offset is added to the value and then the gain is applied.
      // The current gain is scaled by the inverse of the fraction of the range that is used to map that
      // fraction to the range 0-1.
      // The current offset must offset by the new offset divided by the original gain (because it will
      // be multiplied by it along the way).
      double minVal, maxVal;
      std::string ret = m_rangeEstimator->GetCurrentRange(minVal, maxVal);
      if (ret.empty()) {
        // Only adjust if we have no error.
        if (maxVal > minVal) {
          offset = offset - minVal/gain;
          gain /= (maxVal - minVal);
        }
      } else {
        std::cerr << "CompositeCameras::RenderView(): Failed to get range estimate: " << ret << std::endl;
      }
    }
    glUniform1f(m_offsetUniformID, offset);
    glUniform1f(m_gainUniformID, gain);
    glUniform1f(m_depthScaleUniformID, m_cameraRenderInfos[c]->m_depthScale);

    // Draw the camera using its vertex buffer objects after specifying its layout.
    // We don't need to lock these because they will only have been updated by the callback
    // handler.  @todo If we allow asynchronous updates, we will need to lock these.
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
    CameraRenderInfo const& CRI = *m_cameraRenderInfos[i];
    if (m_images[i] != nullptr) {
      CRI.m_imageQueue->UnlockImage(m_images[i]);
    }
  }
  m_images.clear();
  glUseProgram(0);
}

//======================================
// Added by Sang Yoon to fix a bug in drawing a line where the cylindrical projection is used
// Check if the line must be split or not. If the line must be split, the split points (view_sp1 and view_sp2) are calculated and true is returned.
#define SPLIT_DIFF      0.00001
bool CheckLineSplit(double view_a1, double view_a2, double view_p1[3], double view_p2[3], double view_sp1[3], double view_sp2[3])
{
  if (view_a1 == view_a2 || view_p1[0] == view_p2[0]) return false;
  if (180.0 - view_a1 + view_a2 + 180.0 <= 180) {
    double t = view_p2[0] / (view_p2[0] - view_p1[0]);

    if (t + SPLIT_DIFF <= 1.0) {
      view_sp1[0] = view_p2[0] + (t + SPLIT_DIFF) * (view_p1[0] - view_p2[0]);
      view_sp1[1] = view_p2[1] + (t + SPLIT_DIFF) * (view_p1[1] - view_p2[1]);
      view_sp1[2] = view_p2[2] + (t + SPLIT_DIFF) * (view_p1[2] - view_p2[2]);
    }
      else
        return false;

    if (t - SPLIT_DIFF >= 0.0) {
      view_sp2[0] = view_p2[0] + (t - SPLIT_DIFF) * (view_p1[0] - view_p2[0]);
      view_sp2[1] = view_p2[1] + (t - SPLIT_DIFF) * (view_p1[1] - view_p2[1]);
      view_sp2[2] = view_p2[2] + (t - SPLIT_DIFF) * (view_p1[2] - view_p2[2]);
    }
    else
      return false;

    return true;
  }
  return false;
}

void DrawLineCP(glm::mat4 inverse_view_translate, glm::vec4 view_point1, double angle1, glm::vec4 view_point2, double angle2)
{
  bool line_split;
  double view_a1, view_a2;
  double view_p1[3], view_p2[3], view_sp1[3], view_sp2[3];
  double tmpd;

  view_a1 = angle1;
  view_a2 = angle2;
  view_p1[0] = view_point1[0]; view_p1[1] = view_point1[1]; view_p1[2] = view_point1[2];
  view_p2[0] = view_point2[0]; view_p2[1] = view_point2[1]; view_p2[2] = view_point2[2];

  if (view_a1 < view_a2) {
    // Swap a1 and a2, and also p1 and p2, so that a1 is larger than a2.
    tmpd = view_a1; view_a1 = view_a2; view_a2 = tmpd;
    tmpd = view_p1[0]; view_p1[0] = view_p2[0]; view_p2[0] = tmpd;
    tmpd = view_p1[1]; view_p1[1] = view_p2[1]; view_p2[1] = tmpd;
    tmpd = view_p1[2]; view_p1[2] = view_p2[2]; view_p2[2] = tmpd;
  }

  glm::vec4 p1 = inverse_view_translate * glm::vec4(view_p1[0], view_p1[1], view_p1[2], 1.0);
  glm::vec4 p2 = inverse_view_translate * glm::vec4(view_p2[0], view_p2[1], view_p2[2], 1.0);

  line_split = CheckLineSplit(view_a1, view_a2, view_p1, view_p2, view_sp1, view_sp2);
  if (line_split) {
    glm::vec4 sp1 = inverse_view_translate * glm::vec4(view_sp1[0], view_sp1[1], view_sp1[2], 1.0);
    glm::vec4 sp2 = inverse_view_translate * glm::vec4(view_sp2[0], view_sp2[1], view_sp2[2], 1.0);

    glBegin(GL_LINES);
    glVertex3f((GLfloat)p1[0], (GLfloat)p1[1], (GLfloat)p1[2]);
    glVertex3f((GLfloat)sp1[0], (GLfloat)sp1[1], (GLfloat)sp1[2]);
    glVertex3f((GLfloat)p2[0], (GLfloat)p2[1], (GLfloat)p2[2]);
    glVertex3f((GLfloat)sp2[0], (GLfloat)sp2[1], (GLfloat)sp2[2]);
    glEnd();
  }
  else {
    glBegin(GL_LINES);
    glVertex3f((GLfloat)p1[0], (GLfloat)p1[1], (GLfloat)p1[2]);
    glVertex3f((GLfloat)p2[0], (GLfloat)p2[1], (GLfloat)p2[2]);
    glEnd();
  }
}
//======================================

//======================================
// Added by Sang Yoon to draw a rectangle to show the head orientation of detailed view in the overview window

#define SQR(x)  ((x)*(x))

// 3D ray struct defined by 2 points
struct Ray {
  glm::vec4 from;
  glm::vec4 to;
};

// Find a ray-sphere intersection.
// Note that a single intersection point must exist since the start point of the ray is located inside the sphere.
static bool FindRaySphereIntersection(Ray *ray, double radius, double intersection[3])
{
  double a, b, c, D, t;

  a = SQR(ray->to[0] - ray->from[0]) + SQR(ray->to[1] - ray->from[1]) + SQR(ray->to[2] - ray->from[2]);
  b = 2.0*(ray->from[0]*(ray->to[0]-ray->from[0]) + ray->from[1]*(ray->to[1]-ray->from[1]) 
            +ray->from[2]*(ray->to[2]-ray->from[2]));
  c = SQR(ray->from[0]) + SQR(ray->from[1]) + SQR(ray->from[2]) - SQR(radius);

  D = b * b - 4.0 * a * c;
  if (D < 0)
    return false;

  t = (-b + sqrt(D)) / (2.0 * a);
  if (t > 0) {
    intersection[0] = (ray->to[0] - ray->from[0])*t + ray->from[0];
    intersection[1] = (ray->to[1] - ray->from[1])*t + ray->from[1];
    intersection[2] = (ray->to[2] - ray->from[2])*t + ray->from[2];
    return true;
  }

  t = (-b - sqrt(D)) / (2.0 * a);
  if (t > 0) {
    intersection[0] = (ray->to[0] - ray->from[0])*t + ray->from[0];
    intersection[1] = (ray->to[1] - ray->from[1])*t + ray->from[1];
    intersection[2] = (ray->to[2] - ray->from[2])*t + ray->from[2];
    return true;
  }

  return false;
}

// Draw a rectangle to show the head orientation.
void CompositeCameras::DrawHeadOrientation(float view_farf, int screen_width)
{
  GLfloat org_line_width;
  GLboolean org_depth_test;
  glm::mat4 detailed_view_translate;
  float detailed_view_leftf;
  float detailed_view_rightf;
  float detailed_view_topf;
  float detailed_view_bottomf;
  float detailed_view_nearf;

  {
    std::lock_guard<std::mutex> lock(g_overview_mutex);

    detailed_view_translate = g_detailed_view_translate;
    detailed_view_leftf = g_detailed_view_leftf;
    detailed_view_rightf = g_detailed_view_rightf;
    detailed_view_topf = g_detailed_view_topf;
    detailed_view_bottomf = g_detailed_view_bottomf;
    detailed_view_nearf = g_detailed_view_nearf;
  }

  if (!m_drawing_head_orientation_initialized) {
    // Create a color texture of which size is 1 x 1.
    glGenTextures(1, &m_head_orientation_colorTexture);
    glBindTexture(GL_TEXTURE_2D, m_head_orientation_colorTexture);
    // Set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    // Set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // Load image into the texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, m_colorTextureSrc);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Create a tone map texture of which size is 1.
    glGenTextures(1, &m_head_orientation_toneMapTexture);
    glBindTexture(GL_TEXTURE_1D, m_head_orientation_toneMapTexture);
    // Set the texture wrapping and filtering parameters
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // Load image into the texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, m_colorTextureSrc);
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, 1, 0, GL_RGB, GL_FLOAT, m_toneMapTextureSrc);
    glBindTexture(GL_TEXTURE_1D, 0);

    m_drawing_head_orientation_initialized = true;
  }

  //
  // Find intersection points of view frustum (or 4 corner vectors of view frustum) and a big sphere
  //
  float abs_detailed_view_leftf = fabs(detailed_view_leftf);
  float abs_detailed_view_rightf = fabs(detailed_view_rightf);
  float adjusted_detailed_view_leftf;
  float adjusted_detailed_view_rightf;

  // Check if the view frustum is symmetric with respect to the yz plane.
  // If it is not symmetric, assume that the detailed view is stereoscopically rendered for two eyes,
  // and adjust the left and right values of the view frustum,
  // so that it is symmetric (and as wide as covered using both eyes).
  if (abs_detailed_view_leftf >= abs_detailed_view_rightf) {
    adjusted_detailed_view_leftf = -abs_detailed_view_leftf;
    adjusted_detailed_view_rightf = abs_detailed_view_leftf;
  }
  else {
    adjusted_detailed_view_leftf = -abs_detailed_view_rightf;
    adjusted_detailed_view_rightf = abs_detailed_view_rightf;
  }

  // Calculate the rays of view frustum edges in the helicopter coorindate system (up: +Z)
  glm::mat4 inverse_detailed_view_translate = glm::inverse(detailed_view_translate);
  Ray tmp_ray = {glm::vec4(0, 0, 0, 1), glm::vec4(adjusted_detailed_view_leftf, detailed_view_bottomf, -detailed_view_nearf, 1)};
  Ray left_bottom_ray = { inverse_detailed_view_translate * tmp_ray.from, inverse_detailed_view_translate * tmp_ray.to };
  tmp_ray = { glm::vec4(0, 0, 0, 1), glm::vec4(adjusted_detailed_view_rightf, detailed_view_bottomf, -detailed_view_nearf, 1) };
  Ray right_bottom_ray = { inverse_detailed_view_translate * tmp_ray.from, inverse_detailed_view_translate * tmp_ray.to };
  tmp_ray = { glm::vec4(0, 0, 0, 1), glm::vec4(adjusted_detailed_view_leftf, detailed_view_topf, -detailed_view_nearf, 1) };
  Ray left_top_ray = { inverse_detailed_view_translate * tmp_ray.from, inverse_detailed_view_translate * tmp_ray.to };
  tmp_ray = { glm::vec4(0, 0, 0, 1), glm::vec4(adjusted_detailed_view_rightf, detailed_view_topf, -detailed_view_nearf, 1) };
  Ray right_top_ray = { inverse_detailed_view_translate * tmp_ray.from, inverse_detailed_view_translate * tmp_ray.to };

  double intersection_left_bottom_point[3];
  double intersection_right_bottom_point[3];
  double intersection_left_top_point[3];
  double intersection_right_top_point[3];
  bool intersection_left_bottom_found = false;
  bool intersection_right_bottom_found = false;
  bool intersection_left_top_found = false;
  bool intersection_right_top_found = false;

  // Find the intersection points between a big sphere enclosing all the camera image planes (i.e., radius = far clipping distance - 1) and the view frustum rays.
  intersection_left_bottom_found = FindRaySphereIntersection(&left_bottom_ray, view_farf-1.0, intersection_left_bottom_point);
  intersection_right_bottom_found = FindRaySphereIntersection(&right_bottom_ray, view_farf-1.0, intersection_right_bottom_point);
  intersection_left_top_found = FindRaySphereIntersection(&left_top_ray, view_farf-1.0, intersection_left_top_point);
  intersection_right_top_found = FindRaySphereIntersection(&right_top_ray, view_farf-1.0, intersection_right_top_point);

  // If all the intersection points are found, draw the rectangle formed by them.
  if (intersection_left_bottom_found && intersection_right_bottom_found
      && intersection_left_top_found && intersection_right_top_found) {
    // Bind the color and tone map textures to texture units 0 and 1, respectively.
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_head_orientation_colorTexture);
    glUniform1i(m_imageTextureId, 0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_1D, m_head_orientation_toneMapTexture);
    glUniform1i(m_toneMapTextureId, 1);

    // Save the current line thickness and set the line thickness to a new value.
    glGetFloatv(GL_LINE_WIDTH, &org_line_width);
    glLineWidth(screen_width/1000.0f);
    // Save the current depth test status and diable the depth test.
    glGetBooleanv(GL_DEPTH_TEST, &org_depth_test);
    glDisable(GL_DEPTH_TEST);

    glUniform1f(m_offsetUniformID, 0.0);
    glUniform1f(m_gainUniformID, 1.0);
    glUniform1f(m_depthScaleUniformID, -1.0);

    // Draw a rectangle
    //======================================
    // Revised by Sang Yoon to fix a bug in drawing a line where the cylindrical projection is used
    if (m_CP_enabled) {
      glm::mat4 inverse_view_translate = glm::inverse(g_overview_translate);
      double angle_left_bottom, angle_right_bottom, angle_left_top, angle_right_top;
      glm::vec4 view_intersection_left_bottom_point = g_overview_translate * glm::vec4(intersection_left_bottom_point[0], intersection_left_bottom_point[1], intersection_left_bottom_point[2], 1.0);
      glm::vec4 view_intersection_right_bottom_point = g_overview_translate * glm::vec4(intersection_right_bottom_point[0], intersection_right_bottom_point[1], intersection_right_bottom_point[2], 1.0);
      glm::vec4 view_intersection_left_top_point = g_overview_translate * glm::vec4(intersection_left_top_point[0], intersection_left_top_point[1], intersection_left_top_point[2], 1.0);
      glm::vec4 view_intersection_right_top_point = g_overview_translate * glm::vec4(intersection_right_top_point[0], intersection_right_top_point[1], intersection_right_top_point[2], 1.0);

      angle_left_bottom = atan2(view_intersection_left_bottom_point[0], -view_intersection_left_bottom_point[2]) * 180.0 / M_PI;
      angle_right_bottom = atan2(view_intersection_right_bottom_point[0], -view_intersection_right_bottom_point[2]) * 180.0 / M_PI;
      angle_left_top = atan2(view_intersection_left_top_point[0], -view_intersection_left_top_point[2]) * 180.0 / M_PI;
      angle_right_top = atan2(view_intersection_right_top_point[0], -view_intersection_right_top_point[2]) * 180.0 / M_PI;

      DrawLineCP(inverse_view_translate, view_intersection_left_bottom_point, angle_left_bottom, view_intersection_right_bottom_point, angle_right_bottom);
      DrawLineCP(inverse_view_translate, view_intersection_right_bottom_point, angle_right_bottom, view_intersection_right_top_point, angle_right_top);
      DrawLineCP(inverse_view_translate, view_intersection_right_top_point, angle_right_top, view_intersection_left_top_point, angle_left_top);
      DrawLineCP(inverse_view_translate, view_intersection_left_top_point, angle_left_top, view_intersection_left_bottom_point, angle_left_bottom);
    }
    else {
      glBegin(GL_LINE_LOOP);
      glVertex3f((GLfloat)intersection_left_bottom_point[0], (GLfloat)intersection_left_bottom_point[1], (GLfloat)intersection_left_bottom_point[2]);
      glVertex3f((GLfloat)intersection_right_bottom_point[0], (GLfloat)intersection_right_bottom_point[1], (GLfloat)intersection_right_bottom_point[2]);
      glVertex3f((GLfloat)intersection_right_top_point[0], (GLfloat)intersection_right_top_point[1], (GLfloat)intersection_right_top_point[2]);
      glVertex3f((GLfloat)intersection_left_top_point[0], (GLfloat)intersection_left_top_point[1], (GLfloat)intersection_left_top_point[2]);
      glEnd();
    }
    //======================================
    //

    // Unbind the color and tone map textures.
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_1D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Restore the previous line thickness.
    glLineWidth(org_line_width);
    // Restore the previous depth test status.
    if (org_depth_test)
      glEnable(GL_DEPTH_TEST);
  }
}
//======================================

//==================================================================================================
// Objects needed by the CompositeLineRawData class.

static const GLchar* lineRawDataVertexShader =
R"(#version 330 core

   layout (location = 0) in vec2 aPos;
   layout (location = 1) in float aTexCoord;

   out float TexCoord;

   void main()
   {
      gl_Position = vec4(aPos, 0.0, 1.0);
      TexCoord = aTexCoord;
   })";

static const GLchar* lineRawDataFragmentShader =
R"(#version 330 core
   out vec4 FragColor;
   in float TexCoord;

   uniform sampler1D textureID;
   void main()
   {
      FragColor = texture(textureID, TexCoord);
   })";

CompositeLineRawData::CompositeLineRawData(GLfloat x0, GLfloat y0, GLfloat x1, GLfloat y1,
  std::vector<uint8_t> const& valuesRGB)
  : Composite()
  , m_x0(x0)
  , m_y0(y0)
  , m_x1(x1)
  , m_y1(y1)
  , m_numPixels(valuesRGB.size() / 3)
  , m_programId(0)
  , m_texture(0)
  , m_textureId(0)
{
  // Check the input parameters
  if (valuesRGB.size() % 3 != 0) {
    m_numPixels = 0;
    throw std::runtime_error("CompositeLineRawData::CompositeLineRawData(): valuesRGB size must be a multiple of 3");
  }

  // We do not clear the buffers because we're an overlay.
  m_doClear = false;

  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  GLenum ret = glewInit();
  if (ret != GLEW_OK) {
    throw std::runtime_error("CompositeLineRawData::CompositeLineRawData(): Failed to initialize GLEW: " + ret);
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Create the 1D texture from the RGB values
  glGenTextures(1, &m_texture);
  if (m_texture == 0) {
    m_numPixels = 0;
    throw std::runtime_error("CompositeLineRawData::CompositeLineRawData(): glGenTextures failed");
  }
  glBindTexture(GL_TEXTURE_1D, m_texture);
  glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, m_numPixels, 0, GL_RGB, GL_UNSIGNED_BYTE, valuesRGB.data());
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_1D, 0);

  // Create the vertex buffer object for the line.
  glGenBuffers(1, &m_vertexBufferObject);
  if (m_vertexBufferObject == 0) {
    m_numPixels = 0;
    throw std::runtime_error("CompositeLineRawData::CompositeLineRawData(): glGenBuffers failed");
  }
}

bool CompositeLineRawData::SetupRendering()
{
  GLuint vertexShaderId = glCreateShader(GL_VERTEX_SHADER);
  GLuint fragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);

  try {
    // vertex shader
    glShaderSource(vertexShaderId, 1, &lineRawDataVertexShader, NULL);
    glCompileShader(vertexShaderId);
    checkShaderError(vertexShaderId, "Vertex shader compilation failed.");

    // fragment shader
    glShaderSource(fragmentShaderId, 1, &lineRawDataFragmentShader, NULL);
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
  }
  catch (std::runtime_error& e) {
    std::cerr << "CompositeLineRawData::SetupRendering(): " << e.what() << std::endl;
    return false;
  }

  // Get the IDs for all of the uniform parameters we will want to change.
  m_textureId = glGetUniformLocation(m_programId, "textureID");
  if (m_textureId == -1) {
    std::cerr << "CompositeCameras::SetupRendering(): Failed to get uniform texture ID" << std::endl;
    return false;
  }

  return true;
}

bool CompositeLineRawData::UpdateValues(std::vector<uint8_t> const& valuesRGB)
{
  // Verify that the number of pixels is the same as the original number of pixels.
  if (valuesRGB.size() != m_numPixels * 3) {
    std::cerr << "CompositeLineRawData::UpdateValues(): valuesRGB size must be the same as the original size" << std::endl;
    return false;
  }

  // Copy the new values into the image texture.
  glBindTexture(GL_TEXTURE_1D, m_texture);
  glTexSubImage1D(GL_TEXTURE_1D, 0, 0, m_numPixels, GL_RGB, GL_UNSIGNED_BYTE, valuesRGB.data());
  glBindTexture(GL_TEXTURE_1D, 0);

  return true;
}

CompositeLineRawData::~CompositeLineRawData()
{
  // Delete the buffer, texture and shader program (all calls ignore being called on an invalid ID).
  glDeleteBuffers(1, &m_vertexBufferObject);
  glDeleteTextures(1, &m_texture);
  glDeleteProgram(m_programId);
}

void CompositeLineRawData::ComputeVertexCoordinates(GLint width, GLint height, GLint px0, GLint py0,
  GLint px1, GLint py1, GLfloat& x0, GLfloat& y0, GLfloat& x1, GLfloat& y1)
{
  // Compute the normalized coordinates for the line, keeping in mind that the pixel centers are a half
  // pixel away from the edges.
  x0 = 2.0f * (px0 + 0.5f) / width - 1.0f;
  y0 = 1.0f - 2.0f * (py0 + 0.5f) / height;
  x1 = 2.0f * (px1 + 0.5f) / width - 1.0f;
  y1 = 1.0f - 2.0f * (py1 + 0.5f) / height;
}

void CompositeLineRawData::SetupRenderFrame(asdp::Time /* scanOutTime */)
{
  glUseProgram(m_programId);
  glDisable(GL_CULL_FACE);
  glPointSize(1.0f);
}

//======================================
// Revised by Sang Yoon to match the function declaration of the Composite class revised for the cylinderical projection
void CompositeLineRawData::RenderView(asdp::Time /* scanOutTime */, const float* /* viewProjection */, const float* /* modelViewMatrix */, const float /* lh_hFOVf */, const float /* rh_hFOVf */, const float /* bh_vFOVf */, const float /* th_vFOVf */, const float /* nearf */, const float /* farf */)
//======================================
{
  // Turn off depth testing, we always want to draw the line.
  glDisable(GL_DEPTH_TEST);

  // Bind the texture to texture unit 0.
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_1D, m_texture);
  glUniform1i(m_textureId, 0);

  // Compute the vertex coordinate data for the line.
  // The vertex coordinate are the start and end of the line, which is based on the constructor
  // parameters along with the viewport render information.

  // Fill in the vertex data.
  // There are two spatial coordinates and one texture coordinate per vertex.
  // The texture coordinates are the normalized position along the line.
  std::vector<GLfloat> vertices;
  vertices.push_back(m_x0);
  vertices.push_back(m_y0);
  vertices.push_back(0.0f);
  vertices.push_back(m_x1);
  vertices.push_back(m_y1);
  vertices.push_back(1.0f);

  // Unbind any currently bound vertex array object.
  // We cannot use vertex array objects because we're potentially going to be called
  // from multiple OpenGL contexts in different threads and VAOs are not shared between
  // contexts.
  glBindVertexArray(0);

  // Enable the vertex attribute arrays we are going to use
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);

  // Draw the line using its vertex buffer objects after specifying its layout.
  // Draw the final point on the line because OpenGL doesn't fill that point in.
  glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices[0]) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
  glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)(2 * sizeof(GLfloat)));
  glDrawArrays(GL_LINES, 0, vertices.size() / 3);
  glDrawArrays(GL_POINTS, 1, 1);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // Unbind the image from its texture unit
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_1D, 0);
}

void CompositeLineRawData::TearDownRenderFrame()
{
  glUseProgram(0);
}

//======================================
// Added by Sang Yoon for an override method inherited from the Composite class
// Note that this method is not used in the CompositeLineRawData class.
void CompositeLineRawData::DrawHeadOrientation(float view_farf, int screen_width)
{
    // Do nothing for CompositeLineRawData.
}
//======================================

//==================================================================================================
// Objects needed by the CompositePackXSightFrame class.

static const GLchar* packXSightFrameVertexShader =
R"(#version 330 core
   layout (location = 0) in vec2 aPos;
   layout (location = 1) in vec2 aTexCoord;

   uniform int displayWidth;

   out vec2 TexCoord1, TexCoord2;

   void main()
   {
      gl_Position = vec4(aPos, 0.0, 1.0);
      TexCoord1 = aTexCoord;
      TexCoord2 = aTexCoord;

      // The texture coordinates are those of the centers of the two pixels to be merged into
      // one on the output pixel.  This is a screen-aligned rectangle with a texture whose horizontal size
      // is twice the number of pixels in the image.  We compute the two coordinates that when interpolated
      // across the triangle will land on the left pixel and the right pixel, respectively.
      // We do this by truncating the first texture coordinate to the left pixel and adding half a pixel
      // to it to get the right pixel.
      // NOTE that texture coordinates 0 and 1 refer to the far corners of their respective texels, not
      // to their centers.  Also, the corners of the rectangle are at the edges of the pixels, not at their centers.
      // This means that we want to shift the texture coordinates by half a texel to the left and right, where
      // there are twice as many texels as pixels in X.
      int displayPixel = int(aTexCoord.x * float(displayWidth-1));
      float texturePixel1 = displayPixel * 2;
      float texturePixel2 = texturePixel1 + 1;
      TexCoord1.x -= 0.5f/(2*displayWidth);
      TexCoord2.x += 0.5f/(2*displayWidth);
   })";

static const GLchar* packXSightFrameFragmentShader =
R"(#version 330 core
   uniform sampler2D textureID;

   in vec2 TexCoord1, TexCoord2;
   out vec4 FragColor;

   void main()
   {
      // Read the two neighboring pixel values using the two texture coordinates.
      vec3 color1 = texture(textureID, TexCoord1).rgb;
      vec3 color2 = texture(textureID, TexCoord2).rgb;

      // Compute the average of the R, G, and B channels of each pixel.
      float avg1 = (color1.r + color1.g + color1.b) / 3.0;
      float avg2 = (color2.r + color2.g + color2.b) / 3.0;

      // Store the first pixel into the blue channel and the second into the green channel,
      // making red 0 and the alpha 1.
      FragColor.r = 0.0f;
      FragColor.g = avg2;
      FragColor.b = avg1;
      FragColor.a = 1.0f;
   })";

CompositePackXSightFrame::CompositePackXSightFrame(GLuint inputTexture, int displayWidth)
  : Composite()
  , m_inputTexture(inputTexture)
  , m_displayWidth(displayWidth)
  , m_programId(0)
  , m_displayWidthID(0)
  , m_numIndices(0)
{
  // Check the input parameters
  if (inputTexture == 0) {
    throw std::runtime_error("Zero texture ID");
  }
  if (displayWidth <= 0) {
    throw std::runtime_error("Invalid display width");
  }

  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  GLenum ret = glewInit();
  if (ret != GLEW_OK) {
    throw std::runtime_error("Failed to initialize GLEW: " + ret);
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Create the vertex buffer object for the line.
  glGenBuffers(1, &m_vertexBufferObject);
  if (m_vertexBufferObject == 0) {
    throw std::runtime_error("glGenBuffers failed");
  }

  // Create the index buffer object for the line.
  glGenBuffers(1, &m_indexBufferObject);
  if (m_indexBufferObject == 0) {
    throw std::runtime_error("glGenBuffers failed");
  }
}

bool CompositePackXSightFrame::SetupRendering()
{
  GLuint vertexShaderId = glCreateShader(GL_VERTEX_SHADER);
  GLuint fragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);

  try {
    // vertex shader
    glShaderSource(vertexShaderId, 1, &packXSightFrameVertexShader, NULL);
    glCompileShader(vertexShaderId);
    checkShaderError(vertexShaderId, "Vertex shader compilation failed.");

    // fragment shader
    glShaderSource(fragmentShaderId, 1, &packXSightFrameFragmentShader, NULL);
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
  }
  catch (std::runtime_error& e) {
    std::cerr << "Cannot construct shader program: " << e.what() << std::endl;
    return false;
  }

  // Get the IDs for all of the uniform parameters we will want to change.
  m_displayWidthID = glGetUniformLocation(m_programId, "displayWidth");
  if (m_displayWidthID == -1) {
    std::cerr << "Failed to get uniform display width ID" << std::endl;
    return false;
  }

  // Fill in the vertex data.
  // There are two spatial coordinates and two texture coordinate per vertex.
  // The texture coordinates are the normalized position along the line.
  std::vector<GLfloat> vertices = {
      -1.0f, -1.0f, 0.0f, 0.0f,
       1.0f, -1.0f, 1.0f, 0.0f,
       1.0f,  1.0f, 1.0f, 1.0f,
      -1.0f,  1.0f, 0.0f, 1.0f
  };

  // Index data to share position data
  std::vector<GLuint> indices = { 0, 1, 2, 0, 2, 3 };
  m_numIndices = static_cast<GLsizei>(indices.size());

  // Unbind any currently bound vertex array object.
  // We cannot use vertex array objects because we're potentially going to be called
  // from multiple OpenGL contexts in different threads and VAOs are not shared between
  // contexts.
  glBindVertexArray(0);

  // Enable the vertex attribute arrays we are going to use
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);

  // Draw the line using its vertex buffer objects after specifying its layout.
  // Draw the final point on the line because OpenGL doesn't fill that point in.
  glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices[0]) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBufferObject);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0]) * indices.size(), indices.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  return true;
}

CompositePackXSightFrame::~CompositePackXSightFrame()
{
  // Delete the buffers and shader program (all calls ignore being called on an invalid ID).
  glDeleteBuffers(1, &m_vertexBufferObject);
  glDeleteBuffers(1, &m_indexBufferObject);
  glDeleteProgram(m_programId);
}

void CompositePackXSightFrame::SetupRenderFrame(asdp::Time /* scanOutTime */)
{
  glUseProgram(m_programId);

  // Disable face culling, we always want to draw the quad.
  glDisable(GL_CULL_FACE);
}

void CompositePackXSightFrame::RenderView(asdp::Time /* scanOutTime */, const float* /* viewProjection */,
  const float* /* modelViewMatrix */, const float /* lh_hFOVf */, const float /* rh_hFOVf */,
  const float /* bh_vFOVf */, const float /* th_vFOVf */, const float /* nearf */, const float /* farf */)
{
  // Turn off depth testing, we always want to draw the quad.
  glDisable(GL_DEPTH_TEST);

  // Bind the texture to texture unit 0.
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, m_inputTexture);

  // Set the display-width uniform.
  glUniform1i(m_displayWidthID, m_displayWidth);

  // Unbind any currently bound vertex array object.
  // We cannot use vertex array objects because we're potentially going to be called
  // from multiple OpenGL contexts in different threads and VAOs are not shared between
  // contexts.
  glBindVertexArray(0);

  // Enable the vertex attribute arrays we are going to use
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);

  // Draw the quad using its vertex buffer objects after specifying its layout.
  glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBufferObject);
  glDrawElements(GL_TRIANGLES, m_numIndices, GL_UNSIGNED_INT, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  // Unbind the image from its texture unit
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void CompositePackXSightFrame::TearDownRenderFrame()
{
  glUseProgram(0);
}

//======================================
// Added by Sang Yoon for an override method inherited from the Composite class
// Note that this method is not used in the CompositeLineRawData class.
void CompositePackXSightFrame::DrawHeadOrientation(float view_farf, int screen_width)
{
  // Do nothing for CompositeLineRawData.
}
//======================================

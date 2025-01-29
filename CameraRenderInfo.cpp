/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "CameraRenderInfo.h"

using namespace asdp::render;

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
  double xHalfSpan = tan(glm::radians(m_fovDegrees[0]) * 0.5) * depth;
  double yHalfSpan = tan(glm::radians(m_fovDegrees[1]) * 0.5) * depth;

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
      vertex.vignetteGain = 1.0;
      if (m_vignette != nullptr) {
        // Compute the vignette gain at the point (xn, yn)
        vertex.vignetteGain = m_vignette->EvaluateAtPoint({ xn, yn });
      }
      vertices.push_back(vertex);
    }
  }

  m_mesh.nx = nx;
  m_mesh.ny = ny;
  m_mesh.vertexInfo = vertices;
}

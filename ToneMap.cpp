/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "ToneMap.h"
#include <GL/glew.h>

using namespace asdp::render;

ToneMap::ToneMap(std::vector<ToneMapEntry> mapping, size_t numEntries)
{
  // Default mapping from black to white.
  if (mapping.empty()) {
    mapping.push_back(ToneMapEntry(0.0f, 0.0f, 0.0f, 0.0f));
    mapping.push_back(ToneMapEntry(1.0f, 1.0f, 1.0f, 1.0f));
  }

  // Make sure we have at least two entries in the mapping, repeating the entry if there is only one.
  if (mapping.size() == 1) {
    mapping.push_back(mapping[0]);
  }

  // Fill in the mapping by doing linear interpolation between the values
  // in the mapping vector, computing their normalized location along the
  // intensity from 0-1.
  /// @todo Consider making this faster if needed by scan converting the mapping.
  m_mapping.resize(numEntries);
  for (size_t i = 0; i < numEntries; ++i) {
    float intensity = static_cast<float>(i) / static_cast<float>(numEntries - 1);
    size_t j = 0;
    while (j < mapping.size() && mapping[j].texCoord < intensity) {
      ++j;
    }
    if (j == 0) {
      ToneMapEntry &entry = mapping.front();
      m_mapping[i] = {entry.red, entry.green, entry.blue};
    } else if (j == mapping.size()) {
      ToneMapEntry &entry = mapping.back();
      m_mapping[i] = { entry.red, entry.green, entry.blue };
    } else {
      float t = (intensity - mapping[j - 1].texCoord) / (mapping[j].texCoord - mapping[j - 1].texCoord);
      m_mapping[i][0] = mapping[j - 1].red + t * (mapping[j].red - mapping[j - 1].red);
      m_mapping[i][1] = mapping[j - 1].green + t * (mapping[j].green - mapping[j - 1].green);
      m_mapping[i][2] = mapping[j - 1].blue + t * (mapping[j].blue - mapping[j - 1].blue);
    }
  }
}

uint32_t ToneMap::GenerateTexture()
{
  // Generate a 1D texture.
  GLuint texture = 0;
  glGenTextures(1, &texture);
  if (texture == 0) {
    return 0;
  }

  // Bind the texture.
  glBindTexture(GL_TEXTURE_1D, texture);

  // Set the texture parameters.
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // Fill in the texture.
  glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, static_cast<GLsizei>(m_mapping.size()), 0, GL_RGB, GL_FLOAT, m_mapping.data());

  // Unbind the texture.
  glBindTexture(GL_TEXTURE_1D, 0);

  return texture;
}

bool ToneMap::FillTexture(uint32_t textureID)
{
  // Bind the texture.
  glBindTexture(GL_TEXTURE_1D, textureID);

  // Fill in the texture.
  glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, static_cast<GLsizei>(m_mapping.size()), 0, GL_RGB, GL_FLOAT, m_mapping.data());

  // Unbind the texture.
  glBindTexture(GL_TEXTURE_1D, 0);

  return true;
}

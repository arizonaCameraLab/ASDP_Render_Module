/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <atomic>
#include <Composite.h>
#include <ASDP_Core_API.h>
#include <gfxwrapper_opengl.h>

/// @brief Make an image whose brightness varies from the top of the image to the bottom.
/// @details The image will be a gradient from the minimum value at the bottom to the
/// maximum value at the top.  The image order matches that of stored images, which is
/// different from OpenGL texture order; the first pixel is at the upper left of the image.
/// OpenGL must have been initialized before calling this function.
/// @param width Width of the image.
/// @param height Height of the image.
/// @param minVal Minimum value of the image (at the bottom of the image).
/// @param maxVal Maximum value of the image (at the top of the image).
/// @return An OpenGL texture ID.
GLuint MakeTexture(int width, int height, uint16_t minVal, uint16_t maxVal)
{
  std::vector<uint16_t> image(width * height);
  float range = static_cast<float>(maxVal - minVal);
  for (int j = 0; j < height; j++) {
    float normJ = static_cast<float>(j) / static_cast<float>(height - 1);
    for (int i = 0; i < width; i++) {
      image[i + j*width] = static_cast<uint16_t>(maxVal - normJ*range);
    }
  }

  unsigned int texture;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  // Set the texture wrapping parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  // Set texture filtering parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

   // Load image into texture
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image.data());
  glBindTexture(GL_TEXTURE_2D, 0);

  return texture;
}

// Vertex data for a full-screen quad with 3 spatial coordinates and 2 texture coordinates per vertex
GLfloat vertices[] = {
    -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
     1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
     1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
    -1.0f,  1.0f, 0.0f, 0.0f, 1.0f
};

// Index data to share position data
GLushort indices[] = { 0, 1, 2, 0, 2, 3 };

// Vertex Shader
const char* vertexShaderSource = "#version 330 core\n"
"layout (location = 0) in vec3 aPos;\n"
"layout (location = 1) in vec2 aTexCoord;\n"
"out vec2 TexCoord;\n"
"void main()\n"
"{\n"
"   gl_Position = vec4(aPos, 1.0);\n"
"   TexCoord = vec2(aTexCoord.x, aTexCoord.y);\n"
"}\n\0";

// Fragment Shader
const char* fragmentShaderSource = "#version 330 core\n"
"out vec4 FragColor;\n"
"in vec2 TexCoord;\n"
"uniform sampler2D texture1;\n"
"void main()\n"
"{\n"
"   FragColor = texture(texture1, TexCoord);\n"
"}\n\0";

int main()
{
  int windowSize = 640;
  int width = 1280;
  int height = 1024;

  // Create a windowed mode window and its OpenGL context
  ksDriverInstance driverInstance{};
  ksGpuQueueInfo queueInfo{};
  ksGpuSurfaceColorFormat colorFormat{ KS_GPU_SURFACE_COLOR_FORMAT_B8G8R8A8 };
  ksGpuSurfaceDepthFormat depthFormat{ KS_GPU_SURFACE_DEPTH_FORMAT_D24 };
  ksGpuSampleCount sampleCount{ KS_GPU_SAMPLE_COUNT_1 };
  ksGpuWindow m_window{};
  if (!ksGpuWindow_Create(&m_window, &driverInstance, &queueInfo, 0, colorFormat, depthFormat,
      sampleCount, windowSize, windowSize, false)) {
    std::cerr << "Failed to open window\n";
    return 1;
  }

  // Make the window's context current
  ksGpuContext_SetCurrent(&m_window.context);

  // Create a new shared context that we'll use to generate a texture into that
  // we'll use in the main context.
  ksGpuContext sharedContext;
  if (!ksGpuContext_CreateShared(&sharedContext, &m_window.context, 0)) {
    std::cerr << "Failed to create shared context\n";
    return 2;
  }

  // Create a new thread that switches to the new context and generates a texture
  // in that context.
  std::atomic<GLuint> texture{0};
  std::atomic_bool done{false};
  std::thread t([&sharedContext, width, height, &texture, &done]() {
    ksGpuContext_SetCurrent(&sharedContext);
    texture = MakeTexture(width, height, 0, 65535);
    glFinish();
    done = true;
  });

  // Wait until the thread is done and verify that the texture ID is valid.
  while (!done) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  t.join();
  if (texture == 0) {
    std::cerr << "Failed to generate texture\n";
    return 3;
  }

  // Make the window's context current
  ksGpuContext_SetCurrent(&m_window.context);

  // Generate and bind the vertex array
  GLuint vao;
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  // Generate and bind the vertex buffer object
  GLuint vbo;
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  // Generate and bind the element buffer object
  GLuint ebo;
  glGenBuffers(1, &ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

  // Create and compile the vertex shader
  GLuint vertexShader;
  vertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
  glCompileShader(vertexShader);

  // Create and compile the fragment shader
  GLuint fragmentShader;
  fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
  glCompileShader(fragmentShader);

  // Link the vertex and fragment shader into a shader program
  GLuint shaderProgram;
  shaderProgram = glCreateProgram();
  glAttachShader(shaderProgram, vertexShader);
  glAttachShader(shaderProgram, fragmentShader);
  glLinkProgram(shaderProgram);

  // Use the shader program
  glUseProgram(shaderProgram);

  // Specify the layout of the vertex data
  GLint posAttrib = glGetAttribLocation(shaderProgram, "aPos");
  glEnableVertexAttribArray(posAttrib);
  glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);

  GLint texAttrib = glGetAttribLocation(shaderProgram, "aTexCoord");
  glEnableVertexAttribArray(texAttrib);
  glVertexAttribPointer(texAttrib, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

  // Loop until the user closes the window.
  std::cout << "You should see a gradient red texture from the top of the image to the bottom." << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Close the window to exit." << std::endl;
  while (KS_GPU_WINDOW_EVENT_EXIT != ksGpuWindow_ProcessEvents(&m_window)) {

    // Draw a single rectangle that fills the window with the texture.
    glViewport(0, 0, windowSize, windowSize);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glBindTexture(GL_TEXTURE_2D, texture);

    // Draw the quad
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);

    glBindTexture(GL_TEXTURE_2D, 0);

    // Swap front and back buffers
    ksGpuWindow_SwapBuffers(&m_window);
  }

  // Done
  return 0;
}

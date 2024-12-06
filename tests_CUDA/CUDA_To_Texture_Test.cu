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
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Composite.h>
#include <ASDP_Core_API.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

/// @brief CUDA kernel to write an offset and scaled copy of a uint16 image to a surface.
/// @param surface The surface to write to.
/// @param buffer The buffer containing the uint16 data.
/// @param nx The width of the image.
/// @param ny The height of the image.
/// @param offset The offset to apply to the data (added to the data before scaling, should be negative, and
/// in the range 0 to -1).
/// @param scale The scale to apply to the data (multiplied by the data after offsetting, should be positive).
__global__ void myKernel(cudaSurfaceObject_t surface, uint16_t *buffer, int nx, int ny, float offset, float scale) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x < nx && y < ny) {
    // Convert the uint16 data, scaled to the range [0, 1] and then adjusted by the specified
    // offset and scale, to a float and clamp it to the range [0, 1].
    float data = scale * (offset + buffer[x + y*nx]/65535.0f);
    if (data < 0) { data = 0; }
    if (data > 1) { data = 1; }

    // Write the data to the surface. The x coordinate is in bytes, so we need to multiply by the
    // size of the data type.
    surf2Dwrite(data, surface, x * sizeof(float), y);
  }
}

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
  cudaError_t cudaStatus;

  // Fill a CPU buffer with the unsigned 16-bit image data.  Have it cover the whole
  // range of values from 0 to 63353.
  std::vector<uint16_t> image(width * height);
  size_t imageSizeBytes = width * height * sizeof(uint16_t);
  for (int j = 0; j < height; j++) {
    float normJ = static_cast<float>(j) / static_cast<float>(height - 1);
    for (int i = 0; i < width; i++) {
      image[i + j * width] = static_cast<uint16_t>(65535 - normJ * 65535);
    }
  }

  // Copy this data to an allocated GPU buffer
  uint16_t *cudaBuffer;
  cudaStatus = cudaMalloc(&cudaBuffer, imageSizeBytes);
  if (cudaStatus != cudaSuccess) {
    std::cerr << "Failed to allocate CUDA buffer: " << cudaGetErrorString(cudaStatus) << std::endl;
    return 0;
  }
  cudaStatus = cudaMemcpy(cudaBuffer, image.data(), imageSizeBytes, cudaMemcpyHostToDevice);
  if (cudaStatus != cudaSuccess) {
    std::cerr << "Failed to copy data to CUDA buffer: " << cudaGetErrorString(cudaStatus) << std::endl;
    cudaFree(cudaBuffer);
    return 0;
  }

  // Generate the texture, specifying its type and size but not filling it with data.
  GLuint textureID;
  glGenTextures(1, &textureID);
  glBindTexture(GL_TEXTURE_2D, textureID);
  // Set the texture wrapping parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  // Set texture filtering parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, nullptr);
  GLenum glErr = glGetError();
  if (glErr != GL_NO_ERROR) {
    std::cerr << "Failed to create texture: " << glErr << std::endl;
    glDeleteTextures(1, &textureID);
    cudaFree(cudaBuffer);
    return 0;
  }
  glBindTexture(GL_TEXTURE_2D, 0);

  // Register the OpenGL texture with CUDA
  cudaGraphicsResource* resource;
  cudaStatus = cudaGraphicsGLRegisterImage(&resource, textureID, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsSurfaceLoadStore);
  if (cudaStatus != cudaSuccess) {
    std::cerr << "Failed to register texture: " << cudaGetErrorString(cudaStatus) << std::endl;
    glDeleteTextures(1, &textureID);
    cudaFree(cudaBuffer);
    return 0;
  }

  // Map the texture for writing by CUDA
  cudaGraphicsMapResources(1, &resource, 0);
  cudaArray* textureData;
  cudaStatus = cudaGraphicsSubResourceGetMappedArray(&textureData, resource, 0, 0);
  if (cudaStatus != cudaSuccess) {
    std::cerr << "Failed to map texture: " << cudaGetErrorString(cudaStatus) << std::endl;
    cudaGraphicsUnregisterResource(resource);
    glDeleteTextures(1, &textureID);
    cudaFree(cudaBuffer);
    return 0;
  }

  // Create a 2D surface object
  cudaResourceDesc resDesc;
  memset(&resDesc, 0, sizeof(resDesc));
  resDesc.resType = cudaResourceTypeArray;
  resDesc.res.array.array = textureData;

  cudaSurfaceObject_t surfObj = 0;
  cudaStatus = cudaCreateSurfaceObject(&surfObj, &resDesc);
  if (cudaStatus != cudaSuccess) {
    std::cerr << "Failed to create surface object: " << cudaGetErrorString(cudaStatus) << std::endl;
    cudaGraphicsUnmapResources(1, &resource, 0);
    cudaGraphicsUnregisterResource(resource);
    glDeleteTextures(1, &textureID);
    cudaFree(cudaBuffer);
    return 0;
  }

  // Copy the buffer into a 2D floating-point texture, applying the offset and scale in the kernel.
  dim3 dimBlock(16, 16);
  dim3 dimGrid((width + dimBlock.x - 1) / dimBlock.x, (height + dimBlock.y - 1) / dimBlock.y);
  float offset = minVal / 65535.0f;
  float scale = (maxVal - minVal) / 65535.0f;
  myKernel<<<dimGrid, dimBlock>>>(surfObj, cudaBuffer, width, height, offset, scale);
  // Wait until the kernel finishes so we don't get partial results.
  cudaDeviceSynchronize();

  // Free up our resources
  cudaDestroySurfaceObject(surfObj);
  cudaGraphicsUnmapResources(1, &resource, 0);
  cudaGraphicsUnregisterResource(resource);
  cudaFree(cudaBuffer);

  return textureID;
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

  // Initialize the library
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return -1;
  }

  // Create a windowed mode window and its OpenGL context
  GLFWwindow* window = glfwCreateWindow(windowSize, windowSize, "CUDA_To_Texture_Test", NULL, NULL);
  if (!window) {
    std::cerr << "Failed to create main window\n";
    glfwTerminate();
    return -1;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  // Create a new shared context that we'll use to generate a texture into that
  // we'll use in the main context.  This will use a hidden window.
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  GLFWwindow* window2 = glfwCreateWindow(windowSize, windowSize, "Hidden", NULL, window);
  if (!window2) {
    std::cerr << "Failed to create hidden window\n";
    glfwTerminate();
    return -1;
  }

  // Create a new thread that switches to the new context and generates a texture
  // in that context.
  std::atomic<GLuint> texture{ 0 };
  std::atomic_bool done{ false };
  std::thread t([&window2, width, height, &texture, &done]() {
    glfwMakeContextCurrent(window2);
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
  glfwMakeContextCurrent(window);

  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    std::cerr << "Composite::Composite(): Failed to initialize GLEW" << std::endl;
    return 4;
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

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

  // Render both sides of each triangle to make sure we see it no matter which way it is facing.
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);

  // Loop until the user closes the window.
  std::cout << "You should see a gradient red texture from the top of the image to the bottom." << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Close the window to exit." << std::endl;
  while (!glfwWindowShouldClose(window)) {

    // Prepare for rendering this frame
    glViewport(0, 0, windowSize, windowSize);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Draw a single rectangle that fills the window.
    // Draw the quad using the texture
    glBindTexture(GL_TEXTURE_2D, texture);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Swap front and back buffers
    glfwSwapBuffers(window);

    // Poll for and process events
    glfwPollEvents();
  }

  // Clean up resources and exit
  glfwTerminate();
  return 0;
}

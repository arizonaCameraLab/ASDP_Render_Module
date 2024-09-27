/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <ASDP_Core_API.h>
#include <ASDP_ImageSource.h>
#include <CPUDataToTextureHandler.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Display.h>
#include <string.h>
#include <thread>
#include <iostream>
#include <chrono>

using namespace asdp;
using namespace asdp::render;
using namespace asdp::ImageSource;

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
"   float value = texture(texture1, TexCoord).r;\n"
"   FragColor = vec4(value, value, value, 0.1);\n"
"}\n\0";

void TextureThread(int width, int height, std::atomic_bool& done,
  GLFWwindow *window, std::shared_ptr<ImageQueue> imageQueue)
{
  cudaError_t ret;
  glfwMakeContextCurrent(window);

  // Allocate pinned memory for the CPU image buffer.
  unsigned char* cpuPinnedImageBuffer;
  ret = cudaMallocHost(&cpuPinnedImageBuffer, width * height * sizeof(uint16_t));
  if (ret != cudaSuccess) {
    std::cerr << "Failed to allocate pinned memory for the CPU image buffer: " << cudaGetErrorString(ret) << std::endl;
    return;
  }
  std::shared_ptr<unsigned char> cpuPinnedImageBufferSP(std::shared_ptr<unsigned char>(cpuPinnedImageBuffer,
    [](unsigned char* ptr) { cudaFreeHost(ptr); }
  ));

  // Allocate memory for the GPU image buffer.
  unsigned char* gpuImageBuffer;
  ret = cudaMalloc(&gpuImageBuffer, width * height * sizeof(uint16_t));
  if (ret != cudaSuccess) {
    std::cerr << "Failed to allocate memory for the GPU image buffer: " << cudaGetErrorString(ret) << std::endl;
    return;
  }
  std::shared_ptr<unsigned char> gpuImageBufferSP(std::shared_ptr<unsigned char>(gpuImageBuffer,
    [](unsigned char* ptr) { cudaFree(ptr); }
  ));

  // Make our CUDA stream
  cudaStream_t* streamPtr = new cudaStream_t;
  cudaStreamCreate(streamPtr);
  std::shared_ptr<cudaStream_t> stream(streamPtr,
    [](cudaStream_t* ptr) { cudaStreamDestroy(*ptr); delete ptr; }
  );

  // Construct a synthetic image source that will provide a loopable set of images.
  asdp::ImageSource::MovingBarsSource imageSource(width, height, 64, 20, 48, 2);

  // Map from Texture ID to cudaGraphicsResource* for the texture data.  This is used by the CPUDataToTextureHandler
  // objects to know which texture to use without having to repeatedly register and unregister it.
  std::shared_ptr< std::map<GLuint, cudaGraphicsResource*> > texturesToCUDAMap =
    std::make_shared< std::map<GLuint, cudaGraphicsResource*> >();

  // Update the texture in the image queue with new image data as fast as possible
  double period = 1.0 / 65.0;
  unsigned frame = 0;
  auto startedRendering = std::chrono::steady_clock::now();
  asdp::Time now, dt(0, 1);
  while (!done) {

    std::string ret;

    // Construct the data to send to the GPU and the CPUDataToTextureHandler object to send it
    std::shared_ptr<DataToSendToGPU> data = std::make_shared<DataToSendToGPU>();
    data->cpuImageBufferPtr = cpuPinnedImageBufferSP;
    data->gpuImageBufferPtr = gpuImageBufferSP;
    data->imageQueuePtr = imageQueue;
    data->streamPtr = stream;
    CPUDataToTextureHandler handler(texturesToCUDAMap, data, width, height, 16);

    // Get the next image from the image source and copy it into the pinned memory buffer
    // three lines at a time, then send to the GPU to be stored in the texture.
    std::shared_ptr<Image> image = imageSource.getNextImage();
    unsigned count = 0;
    for (uint16_t top = 0; top < height; top += 3) {
      size_t myHeight = 3;
      if (top + 3 > height) {
        myHeight = height - top;
      }
      memcpy(cpuPinnedImageBuffer + top * width*2, &(*image->getData())[top * width/2], width * myHeight * sizeof(uint16_t));
      ret = handler.ProcessImageSubset(0, top, width-1, top + myHeight - 1);
      if (ret.size() > 0) {
        std::cerr << "Error in CPUDataToTextureHandler::ProcessImageSubset(): " << ret << std::endl;
        return;
      }

      count++;
    }

    if (frame % 200 == 190) std::cout << "Frame " << frame << ", texture-write fps = " <<
      frame / std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - startedRendering).count() << std::endl;
    frame++;

    now += dt;
    ret = handler.SetCenterTime(now);
    if (ret.size() > 0) {
      std::cerr << "Error in CPUDataToTextureHandler::SetCenterTime(): " << ret << std::endl;
      return;
    }

    // Exit the loop, which will destroy the texture handler,
    // which will send the texture.
  }
}

int main()
{
  int width = 1280;
  int height = 1024;

  // Initialize the GLFW library
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return -1;
  }

  // Create a windowed mode window and its OpenGL context
  GLFWwindow* window = glfwCreateWindow(width, height, "CPUDataToTexture_Test", NULL, NULL);
  if (!window) {
    std::cerr << "Failed to create main window\n";
    glfwTerminate();
    return -1;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  // Make the image queue that will hold the textures to be filled in by the thread and
  // rendered by the main thread.  Initially fill all of the images with gray and time zero.
  std::shared_ptr<ImageQueue> imageQueue = std::make_shared<ImageQueue>();
  std::vector<uint16_t> image(width * height, 32767);
  for (size_t i = 0; i < 3; i++) {
    std::shared_ptr<ImageData> imageData = std::make_shared<ImageData>();

    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    // Set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    // Set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Load image into the texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image.data());
    glBindTexture(GL_TEXTURE_2D, 0);

    imageData->texture = texture;
    imageQueue->InsertImage(imageData);
  }

  // Create a new shared context that we'll use to generate a texture into that
  // we'll use in the main context.  This will use a hidden window.
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  GLFWwindow* window2 = glfwCreateWindow(width, height, "Hidden", NULL, window);
  if (!window2) {
    std::cerr << "Failed to create hidden window\n";
    glfwTerminate();
    return -1;
  }

  // Create a new thread that switches to the new context and generates a texture
  // in that context.
  std::atomic<GLuint> texture{ 0 };
  std::atomic_bool done{ false };
  std::thread t(TextureThread, width, height, std::ref(done), window2, imageQueue);

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
  std::cout << "You should see smooth rolling bars with no horizontal defects." << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Close the window to exit." << std::endl;
  while (!glfwWindowShouldClose(window)) {

    // Prepare for rendering this frame
    glViewport(0, 0, width, height);
    glClearColor(0.2f, 0.3f, 0.8f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Draw a single rectangle that fills the window.
    // Draw the quad using the newest texture
    std::shared_ptr<ImageData> image = imageQueue->GetNewestImages().front();
    glBindTexture(GL_TEXTURE_2D, image->texture);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Swap front and back buffers
    glfwSwapBuffers(window);

    // Done with the image (we just swapped our buffers), put it back in the queue.
    imageQueue->InsertImage(image);

    // Poll for and process events
    glfwPollEvents();
  }

  // Clean up resources and exit
  done = true;
  t.join();

  glfwTerminate();
  return 0;
}

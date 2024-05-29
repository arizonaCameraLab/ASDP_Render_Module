/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <ToneMap.h>

// Vertex data for a full-screen quad with 3 spatial coordinates and 2 texture coordinates per vertex
GLfloat vertices[] = {
    -1.0f, -1.0f, 0.0f, 0.0f,
     1.0f, -1.0f, 0.0f, 1.0f,
     1.0f,  1.0f, 0.0f, 1.0f,
    -1.0f,  1.0f, 0.0f, 0.0f
};

// Index data to share position data
GLushort indices[] = { 0, 1, 2, 0, 2, 3 };

// Vertex Shader
const char* vertexShaderSource =
R"(#version 330 core
   layout (location = 0) in vec3 aPos;
   layout (location = 1) in float aTexCoord;
   out float TexCoord;
   void main()
   {
      gl_Position = vec4(aPos, 1.0);
      TexCoord = aTexCoord;
   })";

// Fragment Shader
const char* fragmentShaderSource =
R"(#version 330 core
   out vec4 FragColor;
   in float TexCoord;
   uniform sampler1D texture1;
   void main()
   {
      FragColor = texture(texture1, TexCoord);
   })";

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
  GLFWwindow* window = glfwCreateWindow(windowSize, windowSize, "ToneMap_Test", NULL, NULL);
  if (!window) {
    std::cerr << "Failed to create main window\n";
    glfwTerminate();
    return -1;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  // Initialize GLEW in our context. It is okay to initialize it more than once.
  glewExperimental = true;
  if (glewInit() != GLEW_OK) {
    std::cerr << "Failed to initialize GLEW" << std::endl;
    return 4;
  }
  // Clear any GL error that Glew caused.  Apparently on Non-Windows
  // platforms, this can cause a spurious error 1280.
  glGetError();

  // Generate the texture for the tone map
  std::vector<asdp::render::ToneMapEntry> mapping;
  mapping.push_back(asdp::render::ToneMapEntry(0.00f, 0.0f, 0.0f, 0.0f));
  mapping.push_back(asdp::render::ToneMapEntry(0.33f, 0.3f, 0.0f, 0.0f));
  mapping.push_back(asdp::render::ToneMapEntry(0.67f, 0.6f, 0.5f, 0.0f));
  mapping.push_back(asdp::render::ToneMapEntry(1.00f, 1.0f, 1.0f, 1.0f));
  asdp::render::ToneMap toneMap(mapping);
  GLuint texture = toneMap.GenerateTexture();
  if (texture == 0) {
    std::cerr << "Failed to generate texture" << std::endl;
    return 5;
  }

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
  glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);

  GLint texAttrib = glGetAttribLocation(shaderProgram, "aTexCoord");
  glEnableVertexAttribArray(texAttrib);
  glVertexAttribPointer(texAttrib, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(3 * sizeof(float)));

  // Loop until the user closes the window.
  std::cout << "You should see a gradient black/red/yellow/white texture from the left of the image to the right." << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Close the window to exit." << std::endl;
  while (!glfwWindowShouldClose(window)) {

    // Draw a single rectangle that fills the window with the texture.
    glViewport(0, 0, windowSize, windowSize);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glBindTexture(GL_TEXTURE_1D, texture);

    // Draw the quad
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);

    glBindTexture(GL_TEXTURE_1D, 0);

    // Swap front and back buffers
    glfwSwapBuffers(window);

    // Poll for and process events
    glfwPollEvents();
  }

  // Clean up resources and exit
  glfwTerminate();
  return 0;
}

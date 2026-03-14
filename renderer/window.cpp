#include "window.hpp"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <chrono>

#include "../engine/world/physicsworld.hpp"
#include "drawbodies.hpp"

#include <iostream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);

void CreateWindow(PhysicsWorld& world) {
	// Initialize and configure GLFW
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// For macOS
#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	// Window creation
	GLFWwindow* window = glfwCreateWindow(800, 600, "Window", NULL, NULL);
	if (window == NULL) {
		std::cout << "Failed to create window" << std::endl;
		glfwTerminate();
		return;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	// GLAD OpenGL function pointers
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialise GLAD" << std::endl;
		return;
	}

	glViewport(0, 0, 800, 600);
	// Allow renderer to control point size
	glEnable(GL_PROGRAM_POINT_SIZE);

	// Initialize the body renderer
	initDrawBodies();

	const float dt = 1.0f / 60.0f;
	const int MAX_SUBSTEPS=8; 

	float simulation_time = 0.0f;
	float total_runtime = 3.0f; // we will run the simulation for 3 seconds in the startung testing phase

	auto last_time = std::chrono::high_resolution_clock::now();
	float accumulator = 0.0f;
	int frame = 0;

	// Loop to render frames
	while (!glfwWindowShouldClose(window)) {
		// Process incoming inputs
		processInput(window);

		auto current_time = std::chrono::high_resolution_clock::now();
		float frametime =
			std::chrono::duration<float>(current_time - last_time).count();
		frametime=std::min(frametime,0.25f);
		last_time = current_time;
		accumulator += frametime;

		int substeps=0;

		while (accumulator >= dt&&substeps<=MAX_SUBSTEPS)
		{
			world.step(dt);

			frame++;
			simulation_time += dt;
			accumulator -= dt;
		}

		if(substeps==MAX_SUBSTEPS){
			accumulator=dt;
		}

		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		RenderBodies(world);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
}

void processInput(GLFWwindow* window) {
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, true);
	}
}
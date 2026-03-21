#include "window.hpp"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <chrono>

#include "../engine/world/physicsworld.hpp"
#include "camera.hpp"
#include "drawbodies.hpp"
#include "bodymenu.hpp"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <iostream>

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void processInput(GLFWwindow *window, float deltaTime, Camera &camera);

void CreateWindow(PhysicsWorld &world)
{
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
	GLFWwindow *window = glfwCreateWindow(800, 600, "Window", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create window" << std::endl;
		glfwTerminate();
		return;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	// GLAD OpenGL function pointers
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialise GLAD" << std::endl;
		return;
	}

	glViewport(0, 0, 800, 600);
	// Allow renderer to control point size
	glEnable(GL_PROGRAM_POINT_SIZE);
	glEnable(GL_DEPTH_TEST);

	// Initialize the body renderer
	initDrawBodies();

	const float dt = 1.0f / 60.0f;
	const int MAX_SUBSTEPS = 8;
	bool isSimulationPaused = false;

	float simulation_time = 0.0f;
	float total_runtime = 3.0f; // we will run the simulation for 3 seconds in the startung testing phase

	auto last_time = std::chrono::high_resolution_clock::now();
	float accumulator = 0.0f;
	int frame = 0;
	Camera camera;

	// initialize ImGui once after OpenGL context creation.
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330 core");

	// Loop to render frames
	while (!glfwWindowShouldClose(window))
	{
		auto current_time = std::chrono::high_resolution_clock::now();
		float frametime =
			std::chrono::duration<float>(current_time - last_time).count();
		frametime = std::min(frametime, 0.25f);
		last_time = current_time;

		// Process incoming inputs with frame-rate independent movement.
		processInput(window, frametime, camera);

		if (!isSimulationPaused)
		{
			accumulator += frametime;
			int substeps = 0;

			while (accumulator >= dt && substeps < MAX_SUBSTEPS)
			{
				world.step(dt);
				++substeps;

				frame++;
				simulation_time += dt;
				accumulator -= dt;
			}

			if (substeps == MAX_SUBSTEPS)
			{
				accumulator = 0.0f;
			}
		}

		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		int framebufferWidth = 0;
		int framebufferHeight = 0;
		glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
		float aspectRatio = framebufferHeight > 0
								? static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight)
								: 1.0f;

		RenderBodies(world, camera, aspectRatio);

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		ImGui::Begin("Simulation Controls");
		if (ImGui::Button(isSimulationPaused ? "Resume Physics" : "Pause Physics"))
		{
			isSimulationPaused = !isSimulationPaused;
			if (isSimulationPaused)
			{
				accumulator = 0.0f;
			}
		}
		const ImGuiIO &io = ImGui::GetIO();
		ImGui::Text("Bodies: %zu", world.getBodies().size());
		ImGui::Text("Contacts: %zu", world.getContactCount());
		ImGui::Text("Frame: %d", frame);
		ImGui::Text("FPS: %.1f", io.Framerate);
		ImGui::Text("Frame time: %.3f ms", io.Framerate > 0.0f ? (1000.0f / io.Framerate) : 0.0f);
		ImGui::End();

		RenderBodyMenu(world);

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwTerminate();
	return;
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
	glViewport(0, 0, width, height);
}

void processInput(GLFWwindow *window, float deltaTime, Camera &camera)
{
	static bool isDraggingCamera = false;
	static double lastMouseX = 0.0;
	static double lastMouseY = 0.0;

	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, true);
	}

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
	{
		camera.moveForward(deltaTime);
	}
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
	{
		camera.moveBackward(deltaTime);
	}
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
	{
		camera.moveLeft(deltaTime);
	}
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
	{
		camera.moveRight(deltaTime);
	}

	const ImGuiIO &io = ImGui::GetIO();
	const bool canDragRotate = !io.WantCaptureMouse;
	const bool rightMouseDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

	if (canDragRotate && rightMouseDown)
	{
		double mouseX = 0.0;
		double mouseY = 0.0;
		glfwGetCursorPos(window, &mouseX, &mouseY);

		if (!isDraggingCamera)
		{
			isDraggingCamera = true;
			lastMouseX = mouseX;
			lastMouseY = mouseY;
		}
		else
		{
			float deltaX = static_cast<float>(mouseX - lastMouseX);
			float deltaY = static_cast<float>(mouseY - lastMouseY);
			camera.rotate(deltaX, -deltaY);
			lastMouseX = mouseX;
			lastMouseY = mouseY;
		}
	}
	else
	{
		isDraggingCamera = false;
	}
}
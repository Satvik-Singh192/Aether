#include "window.hpp"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <chrono>

#include "../engine/world/physicsworld.hpp"
#include "camera.hpp"
#include "drawbodies.hpp"
#include "bodymenu.hpp"
#include "aether_theme.hpp"
#include "../app/test_scenarios.hpp"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <iostream>

namespace
{
	enum class AppScreen
	{
		StartScreen,
		Running
	};

	constexpr TestCase kTestCases[] = {
		TestCase::SphereSphere,
		TestCase::BoxBox,
		TestCase::SphereBox,
		TestCase::BoxRamp,
		TestCase::SphereRamp,
		TestCase::BoxSphereRamp,
		TestCase::BoxStack,
		TestCase::PyramidStack,
		TestCase::ManySpheres,
		TestCase::ManyBoxes,
		TestCase::MixedPile,
		TestCase::BouncyBalls,
		TestCase::SlidingRampRow,
		TestCase::ChainCollide,
		TestCase::RandomScatter,
		TestCase::StressTestLarge,
		TestCase::RopeBasic,
		TestCase::RodBasic,
		TestCase::SpringBasic,
		TestCase::RopeChain,
		TestCase::RodChain,
		TestCase::SoftBody,
		TestCase::RopeCollision,
		TestCase::AngularTorque,
		TestCase::AngularImpact,
		TestCase::AngularStack,
		TestCase::OffCenterHit,
		TestCase::BoxCornerCollision,
		TestCase::StackTipping,
		TestCase::SphereRolling,
		TestCase::RampDropOnSphere,
		TestCase::RampRampStress,
		TestCase::Boxtopple};

	constexpr const char *kTestCaseNames[] = {
		"Sphere Sphere",
		"Box Box",
		"Sphere Box",
		"Box Ramp",
		"Sphere Ramp",
		"Box Sphere Ramp",
		"Box Stack",
		"Pyramid Stack",
		"Many Spheres",
		"Many Boxes",
		"Mixed Pile",
		"Bouncy Balls",
		"Sliding Ramp Row",
		"Chain Collide",
		"Random Scatter",
		"Stress Test Large",
		"Rope Basic",
		"Rod Basic",
		"Spring Basic",
		"Rope Chain",
		"Rod Chain",
		"Soft Body",
		"Rope Collision",
		"Angular Torque",
		"Angular Impact",
		"Angular Stack",
		"Off Center Hit",
		"Box Corner Collision",
		"Stack Tipping",
		"Sphere Rolling",
		"Ramp Drop On Sphere",
		"Ramp Ramp Stress",
		"Box Topple"};

	static_assert((sizeof(kTestCases) / sizeof(kTestCases[0])) == (sizeof(kTestCaseNames) / sizeof(kTestCaseNames[0])), "Test case arrays must stay aligned");
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void processInput(GLFWwindow *window, float deltaTime, Camera &camera, bool cameraEnabled);

static void ShowTooltip(const char *text)
{
	if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
	{
		ImGui::SetTooltip("%s", text);
	}
}

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
	GLFWwindow *window = glfwCreateWindow(1920, 1080, "Window", NULL, NULL);
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

	// Use the actual framebuffer size (important on Windows with DPI scaling).
	{
		int fbw = 0;
		int fbh = 0;
		glfwGetFramebufferSize(window, &fbw, &fbh);
		glViewport(0, 0, fbw, fbh);
	}
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
	AppScreen appScreen = AppScreen::StartScreen;
	bool showControlsHelp = false;
	bool showStartSettings = false;
	bool showBodyInspector = false;
	int selectedScenarioIndex = 31; // RampRampStress
	float startGravityY = world.getGravity().y;
	bool startWireframe = GetBodyDrawWireframeMode();
	float startTint[3] = {1.0f, 1.0f, 1.0f};
	GetBodyTint(startTint[0], startTint[1], startTint[2]);

	// initialize ImGui once after OpenGL context creation.
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	LoadAetherFont();
	ApplyAetherTheme();
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
		processInput(window, frametime, camera, appScreen == AppScreen::Running);
		if (appScreen == AppScreen::Running && !isSimulationPaused)
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
		glViewport(0, 0, framebufferWidth, framebufferHeight);
		float aspectRatio = framebufferHeight > 0
								? static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight)
								: 1.0f;

		RenderBodies(world, camera, aspectRatio);

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		auto toggleSimulationPause = [&]()
		{
			isSimulationPaused = !isSimulationPaused;
			if (isSimulationPaused)
			{
				accumulator = 0.0f;
			}
		};

		if (ImGui::BeginMainMenuBar())
		{
			if (ImGui::BeginMenu("App"))
			{
				if (ImGui::MenuItem("Home / Start Screen"))
				{
					appScreen = AppScreen::StartScreen;
					isSimulationPaused = true;
					accumulator = 0.0f;
				}
				if (ImGui::MenuItem("Controls Help"))
				{
					showControlsHelp = true;
				}
				if (ImGui::MenuItem("Quit", "Esc"))
				{
					glfwSetWindowShouldClose(window, true);
				}
				ImGui::EndMenu();
			}

			if (appScreen == AppScreen::Running)
			{
				if (ImGui::BeginMenu("Simulation"))
				{
					if (ImGui::MenuItem(isSimulationPaused ? "Resume Physics" : "Pause Physics"))
					{
						toggleSimulationPause();
					}
					ImGui::SeparatorText("Stats");
					const ImGuiIO &io = ImGui::GetIO();
					ImGui::Text("Bodies: %zu", world.getBodies().size());
					ImGui::Text("Contacts: %zu", world.getContactCount());
					ImGui::Text("Frame: %d", frame);
					ImGui::Text("FPS: %.1f", io.Framerate);
					ImGui::EndMenu();
				}

				if (ImGui::BeginMenu("Add Body"))
				{
					RenderAddBodyMenuContent(world);
					ImGui::EndMenu();
				}

				if (ImGui::BeginMenu("Constraints"))
				{
					RenderConstraintMenuContent(world);
					ImGui::EndMenu();
				}

				if (ImGui::BeginMenu("World"))
				{
					RenderWorldMenuContent(world);
					if (ImGui::MenuItem("Open Body Inspector"))
					{
						showBodyInspector = true;
					}
					ImGui::EndMenu();
				}

				if (ImGui::BeginMenu("View"))
				{
					bool wf = GetBodyDrawWireframeMode();
					if (ImGui::MenuItem("Wireframe mode", nullptr, &wf))
					{
						SetBodyDrawWireframeMode(wf);
						startWireframe = wf;
					}

					float br = 1.0f;
					float bg = 1.0f;
					float bb = 1.0f;
					GetBodyTint(br, bg, bb);
					float tint[3] = {br, bg, bb};
					ImGui::SeparatorText("Body Tint");
					if (ImGui::ColorEdit3("Body color tint", tint))
					{
						SetBodyTint(tint[0], tint[1], tint[2]);
						startTint[0] = tint[0];
						startTint[1] = tint[1];
						startTint[2] = tint[2];
					}
					ImGui::EndMenu();
				}
			}

			ImGui::EndMainMenuBar();
		}

		if (appScreen == AppScreen::StartScreen)
		{
			const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
			ImGui::SetNextWindowPos(ImVec2(displaySize.x * 0.5f, displaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
			ImGui::SetNextWindowSize(ImVec2(640.0f, 460.0f), ImGuiCond_Always);

			if (ImGui::Begin("Aether Studio - Start", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize))
			{
				auto centerText = [](const char *text, bool disabled)
				{
					const float textWidth = ImGui::CalcTextSize(text).x;
					const float centeredX = (ImGui::GetContentRegionAvail().x - textWidth) * 0.5f;
					if (centeredX > 0.0f)
					{
						ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centeredX);
					}
					if (disabled)
						ImGui::TextDisabled("%s", text);
					else
						ImGui::TextUnformatted(text);
				};

				auto centerButton = [](const char *label, const ImVec2 &size)
				{
					const float centeredX = (ImGui::GetContentRegionAvail().x - size.x) * 0.5f;
					if (centeredX > 0.0f)
					{
						ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centeredX);
					}
					return ImGui::Button(label, size);
				};

				ImGui::Dummy(ImVec2(0.0f, 16.0f));
				ImGui::SetWindowFontScale(1.18f);
				centerText("AETHER STUDIO", false);
				ImGui::SetWindowFontScale(1.0f);
				ImGui::Dummy(ImVec2(0.0f, 2.0f));
				centerText("Physics Simulation Engine", true);
				ImGui::Dummy(ImVec2(0.0f, 16.0f));
				ImGui::Separator();
				ImGui::Dummy(ImVec2(0.0f, 20.0f));

				if (centerButton("Start Simulation", ImVec2(300.0f, 44.0f)))
				{
					world = PhysicsWorld();
					LoadSingleTestScenario(world, kTestCases[selectedScenarioIndex]);
					Vec3 gravity = world.getGravity();
					world.setGravity(Vec3(gravity.x, startGravityY, gravity.z));
					SetBodyDrawWireframeMode(startWireframe);
					SetBodyTint(startTint[0], startTint[1], startTint[2]);

					appScreen = AppScreen::Running;
					isSimulationPaused = false;
					accumulator = 0.0f;
					simulation_time = 0.0f;
					frame = 0;
				}

				ImGui::Dummy(ImVec2(0.0f, 12.0f));
				if (centerButton("Settings", ImVec2(300.0f, 44.0f)))
				{
					showStartSettings = true;
				}

				ImGui::Dummy(ImVec2(0.0f, 12.0f));
				if (centerButton("Quit", ImVec2(300.0f, 44.0f)))
				{
					glfwSetWindowShouldClose(window, true);
				}

				ImGui::Dummy(ImVec2(0.0f, 20.0f));
				ImGui::Separator();
				ImGui::Dummy(ImVec2(0.0f, 4.0f));
				centerText("v1.0.0 | Built with OpenGL + ImGui", true);

				if (showStartSettings)
				{
					ImGui::OpenPopup("Start Settings");
					showStartSettings = false;
				}

				if (ImGui::BeginPopupModal("Start Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
				{
					ImGui::SeparatorText("Scenario");
					ImGui::Combo("Test Scenario", &selectedScenarioIndex, kTestCaseNames, static_cast<int>(sizeof(kTestCaseNames) / sizeof(kTestCaseNames[0])));
					ShowTooltip("Select one predefined test setup to load when starting.");

					ImGui::SeparatorText("Physics");
					if (ImGui::DragFloat("Gravity Y", &startGravityY, 0.1f, -100000.0f, 100000.0f))
					{
						Vec3 current = world.getGravity();
						world.setGravity(Vec3(current.x, startGravityY, current.z));
					}
					ShowTooltip("Applies after scenario load. Negative values pull downward.");

					ImGui::SeparatorText("Rendering");
					if (ImGui::Checkbox("Wireframe mode", &startWireframe))
					{
						SetBodyDrawWireframeMode(startWireframe);
					}
					if (ImGui::ColorEdit3("Body color tint", startTint))
					{
						SetBodyTint(startTint[0], startTint[1], startTint[2]);
					}

					if (ImGui::Button("Close", ImVec2(140.0f, 0.0f)))
					{
						ImGui::CloseCurrentPopup();
					}
					ImGui::EndPopup();
				}

				if (showControlsHelp)
				{
					ImGui::OpenPopup("Controls Help");
					showControlsHelp = false;
				}

				if (ImGui::BeginPopupModal("Controls Help", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
				{
					ImGui::Text("W / A / S / D : Move camera");
					ImGui::Text("Right Mouse Drag : Rotate camera");
					ImGui::Text("Esc : Quit application");
					ImGui::Text("Use top menu > App > Home / Start Screen to return here.");
					ImGui::Spacing();
					if (ImGui::Button("Close", ImVec2(140.0f, 0.0f)))
					{
						ImGui::CloseCurrentPopup();
					}
					ImGui::EndPopup();
				}
			}
			ImGui::End();
		}
		else
		{
			if (showBodyInspector)
			{
				ImGui::OpenPopup("Body Inspector");
				showBodyInspector = false;
			}

			const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
			const float menuBarHeight = ImGui::GetFrameHeight();
			const ImVec2 inspectorPos(8.0f, menuBarHeight + 8.0f);
			const ImVec2 inspectorSize(560.0f, displaySize.y - menuBarHeight - 16.0f);
			ImGui::SetNextWindowPos(inspectorPos, ImGuiCond_Appearing);
			ImGui::SetNextWindowSize(inspectorSize, ImGuiCond_Appearing);

			if (ImGui::BeginPopupModal("Body Inspector", nullptr, ImGuiWindowFlags_NoResize))
			{
				RenderBodyInspectorContent(world, true);
				ImGui::EndPopup();
			}
		}

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

void processInput(GLFWwindow *window, float deltaTime, Camera &camera, bool cameraEnabled)
{
	static bool isDraggingCamera = false;
	static double lastMouseX = 0.0;
	static double lastMouseY = 0.0;

	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, true);
	}

	if (!cameraEnabled)
	{
		isDraggingCamera = false;
		return;
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
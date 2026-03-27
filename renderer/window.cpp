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
		GuideScreen,
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

	constexpr const char *kGravityPresetNames[] = {
		"Mercury (3.70 m/s^2)",
		"Venus (8.87 m/s^2)",
		"Earth (9.81 m/s^2)",
		"Moon (1.62 m/s^2)",
		"Mars (3.71 m/s^2)",
		"Jupiter (24.79 m/s^2)",
		"Saturn (10.44 m/s^2)",
		"Uranus (8.69 m/s^2)",
		"Neptune (11.15 m/s^2)",
		"Pluto (0.62 m/s^2)"};

	constexpr float kGravityPresetValues[] = {
		3.70f,
		8.87f,
		9.81f,
		1.62f,
		3.71f,
		24.79f,
		10.44f,
		8.69f,
		11.15f,
		0.62f};

	static_assert((sizeof(kTestCases) / sizeof(kTestCases[0])) == (sizeof(kTestCaseNames) / sizeof(kTestCaseNames[0])), "Test case arrays must stay aligned");
	static_assert((sizeof(kGravityPresetNames) / sizeof(kGravityPresetNames[0])) == (sizeof(kGravityPresetValues) / sizeof(kGravityPresetValues[0])), "Gravity preset arrays must stay aligned");
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

	initDrawBodies();

	const float dt = 1.0f / 60.0f;
	const int MAX_SUBSTEPS = 8;
	bool isSimulationPaused = false;
	float simulation_time = 0.0f;

	auto last_time = std::chrono::high_resolution_clock::now();
	float accumulator = 0.0f;
	int frame = 0;
	Camera camera;
	AppScreen appScreen = AppScreen::StartScreen;
	bool guideShownForStart = false;
	bool skipGuideForSession = false;
	bool pendingSimulationStart = false;
	bool guideOpenedFromRunning = false;
	int selectedScenarioIndex = 30; // RampRampStress
	float startGravityY = world.getGravity().y;
	int selectedGravityPreset = 2; // Earth
	bool startWireframe = GetBodyDrawWireframeMode();
	float startTint[3] = {1.0f, 1.0f, 1.0f};
	GetBodyTint(startTint[0], startTint[1], startTint[2]);

	// State preservation
	int lastSimulationScenario = selectedScenarioIndex;
	float lastSimulationGravity = startGravityY;
	int lastSimulationGravityPreset = selectedGravityPreset;
	bool lastSimulationWireframe = startWireframe;
	float lastSimulationTint[3] = {startTint[0], startTint[1], startTint[2]};
	bool hasActiveSim = false;

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	LoadAetherFont();
	ApplyAetherTheme();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330 core");

	auto reloadSelectedScenario = [&]()
	{
		world = PhysicsWorld();
		LoadSingleTestScenario(world, kTestCases[selectedScenarioIndex]);
		Vec3 gravity = world.getGravity();
		world.setGravity(Vec3(gravity.x, startGravityY, gravity.z));
		SetBodyDrawWireframeMode(startWireframe);
		SetBodyTint(startTint[0], startTint[1], startTint[2]);
		isSimulationPaused = false;
		accumulator = 0.0f;
		simulation_time = 0.0f;
		frame = 0;
	};

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

		glClearColor(0.01f, 0.01f, 0.015f, 1.0f);
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

		auto openMenuScreen = [&]()
		{
			// Save current simulation state so start screen can resume with current settings.
			lastSimulationScenario = selectedScenarioIndex;
			lastSimulationGravity = startGravityY;
			lastSimulationGravityPreset = selectedGravityPreset;
			lastSimulationWireframe = startWireframe;
			lastSimulationTint[0] = startTint[0];
			lastSimulationTint[1] = startTint[1];
			lastSimulationTint[2] = startTint[2];
			hasActiveSim = true;

			appScreen = AppScreen::StartScreen;
			isSimulationPaused = true;
			accumulator = 0.0f;
		};

		auto startOrResumeSimulation = [&]()
		{
			if (!hasActiveSim)
			{
				reloadSelectedScenario();
			}
			else
			{
				// Restore saved start menu state.
				selectedScenarioIndex = lastSimulationScenario;
				startGravityY = lastSimulationGravity;
				selectedGravityPreset = lastSimulationGravityPreset;
				startWireframe = lastSimulationWireframe;
				startTint[0] = lastSimulationTint[0];
				startTint[1] = lastSimulationTint[1];
				startTint[2] = lastSimulationTint[2];
				SetBodyDrawWireframeMode(startWireframe);
				SetBodyTint(startTint[0], startTint[1], startTint[2]);
			}

			appScreen = AppScreen::Running;
			isSimulationPaused = false;
			accumulator = 0.0f;
			simulation_time = 0.0f;
			frame = 0;
			pendingSimulationStart = false;
			guideOpenedFromRunning = false;
		};

		if (ImGui::BeginMainMenuBar())
		{
			if (ImGui::BeginMenu("App"))
			{
				if (ImGui::MenuItem("Menu"))
				{
					if (appScreen == AppScreen::Running)
					{
						openMenuScreen();
					}
					else
					{
						appScreen = AppScreen::StartScreen;
					}
				}
				if (ImGui::MenuItem("Guide"))
				{
					pendingSimulationStart = false;
					guideOpenedFromRunning = (appScreen == AppScreen::Running);
					appScreen = AppScreen::GuideScreen;
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
					ImGui::EndMenu();
				}

				if (ImGui::BeginMenu("Settings"))
				{
					ImGui::SeparatorText("Scenario");
					ImGui::Combo("Test Scenario", &selectedScenarioIndex, kTestCaseNames, static_cast<int>(sizeof(kTestCaseNames) / sizeof(kTestCaseNames[0])));
					ShowTooltip("Choose a test case and use reload to apply it now.");

					if (ImGui::Button("Reload Selected Test Case", ImVec2(ImGui::GetContentRegionAvail().x, 0.0f)))
					{
						reloadSelectedScenario();
					}

					ImGui::SeparatorText("Physics");
					if (ImGui::Combo("Gravity Location", &selectedGravityPreset, kGravityPresetNames, static_cast<int>(sizeof(kGravityPresetNames) / sizeof(kGravityPresetNames[0]))))
					{
						startGravityY = -kGravityPresetValues[selectedGravityPreset];
						Vec3 current = world.getGravity();
						world.setGravity(Vec3(current.x, startGravityY, current.z));
					}
					ShowTooltip("Choose a place in the solar system. Gravity updates immediately.");

					ImGui::SeparatorText("Rendering");
					if (ImGui::Checkbox("Wireframe mode", &startWireframe))
					{
						SetBodyDrawWireframeMode(startWireframe);
					}
					if (ImGui::ColorEdit3("Body color tint", startTint))
					{
						SetBodyTint(startTint[0], startTint[1], startTint[2]);
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

			const char *startWindowTitle = hasActiveSim ? "Aether Studio - Menu" : "Aether Studio - Start";
			if (ImGui::Begin(startWindowTitle, nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize))
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

				if (centerButton(hasActiveSim ? "Resume Simulation" : "Start Simulation", ImVec2(300.0f, 44.0f)))
				{
					if (!guideShownForStart && !skipGuideForSession)
					{
						pendingSimulationStart = true;
						guideOpenedFromRunning = false;
						guideShownForStart = true;
						appScreen = AppScreen::GuideScreen;
					}
					else
					{
						startOrResumeSimulation();
					}
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
			}
			ImGui::End();
		}
		else if (appScreen == AppScreen::GuideScreen)
		{
			const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
			ImGui::SetNextWindowPos(ImVec2(displaySize.x * 0.5f, displaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
			ImGui::SetNextWindowSize(ImVec2(760.0f, 540.0f), ImGuiCond_Always);

			if (ImGui::Begin("Aether Studio - Guide", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize))
			{
				ImGui::SetWindowFontScale(1.14f);
				ImGui::TextUnformatted("Before You Start");
				ImGui::SetWindowFontScale(1.0f);
				ImGui::Separator();
				ImGui::Spacing();

				ImGui::TextWrapped("Aether Studio is an interactive rigid-body physics sandbox. You can build scenes, tune world settings, and inspect solver behavior in real time.");
				ImGui::Spacing();

				ImGui::SeparatorText("Feature Guide");
				ImGui::BulletText("Add Body: Spawn spheres, boxes, and ramps instantly.");
				ImGui::BulletText("Constraints: Link bodies with rods, ropes, and springs.");
				ImGui::BulletText("World: Tune gravity and global simulation parameters.");
				ImGui::BulletText("Settings: Change scenario, reload tests, and rendering style.");
				ImGui::BulletText("Simulation: Pause/resume physics and monitor contacts, frame count, and FPS.");
				ImGui::Spacing();

				ImGui::SeparatorText("Controls");
				ImGui::TextUnformatted("W / A / S / D : Move camera");
				ImGui::TextUnformatted("Right Mouse Drag : Rotate camera");
				ImGui::TextUnformatted("Esc : Quit application");
				ImGui::Spacing();

				bool skipGuideCheckbox = skipGuideForSession;
				if (ImGui::Checkbox("Skip guide next time (this app launch)", &skipGuideCheckbox))
				{
					skipGuideForSession = skipGuideCheckbox;
				}

				ImGui::Dummy(ImVec2(0.0f, 8.0f));
				const float buttonWidth = (ImGui::GetContentRegionAvail().x - 16.0f) * 0.5f;
				const char *primaryLabel = pendingSimulationStart
											   ? (hasActiveSim ? "Resume Simulation" : "Start Simulation")
											   : (guideOpenedFromRunning ? "Return To Simulation" : "Start Simulation");
				if (ImGui::Button(primaryLabel, ImVec2(buttonWidth, 40.0f)))
				{
					if (pendingSimulationStart)
					{
						startOrResumeSimulation();
					}
					else if (guideOpenedFromRunning)
					{
						appScreen = AppScreen::Running;
						guideOpenedFromRunning = false;
					}
					else
					{
						appScreen = AppScreen::StartScreen;
					}
				}

				ImGui::SameLine();
				if (ImGui::Button("Back To Start Menu", ImVec2(buttonWidth, 40.0f)))
				{
					pendingSimulationStart = false;
					if (guideOpenedFromRunning)
					{
						openMenuScreen();
					}
					else
					{
						appScreen = AppScreen::StartScreen;
					}
					guideOpenedFromRunning = false;
				}
			}
			ImGui::End();
		}
		else
		{
			const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
			const float menuBarHeight = ImGui::GetFrameHeight();
			float inspectorWidth = displaySize.x * 0.18f;
			if (inspectorWidth < 280.0f)
				inspectorWidth = 280.0f;
			if (inspectorWidth > 340.0f)
				inspectorWidth = 340.0f;
			const ImVec2 inspectorPos(8.0f, menuBarHeight + 8.0f);
			const float inspectorHeight = (displaySize.y - menuBarHeight - 16.0f) * 0.5f;
			const ImVec2 inspectorSize(inspectorWidth, inspectorHeight);
			ImGui::SetNextWindowPos(inspectorPos, ImGuiCond_Always);
			ImGui::SetNextWindowSize(inspectorSize, ImGuiCond_Always);
			ImGuiWindowFlags inspectorFlags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
			if (ImGui::Begin("Body Inspector", nullptr, inspectorFlags))
			{
				RenderBodyInspectorContent(world, false);
			}
			ImGui::End();
		}

		RenderEnginePopups();
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwTerminate();
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
	(void)window;
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
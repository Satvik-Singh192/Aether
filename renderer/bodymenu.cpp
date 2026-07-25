#include "bodymenu.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <imgui.h>
#include <string>
#include <vector>

#include "bodyselection.hpp"
#include "thermal_palette.hpp"

namespace
{
	static int shapeIndex = 0;
	static int linkBodyAIndex = 0;
	static int linkBodyBIndex = 1;
	static int linkKindIndex = 0;
	static float linkRestLength = 2.0f;
	static float linkStiffness = 5.0f;
	static float linkDamping = 2.0f;

	static float spawnPos[3] = {0.0f, 3.0f, 0.0f};
	static float spawnSpeed[3] = {0.0f, 0.0f, 0.0f};
	static float spawnForce[3] = {0.0f, 0.0f, 0.0f};
	static float spawnMass = 1.0f;
	static float spawnDensity = 1.0f;
	static float spawnVolume = 0.5235988f;

	static float sphereRadius = 0.5f;
	static float boxHalfSize[3] = {0.5f, 0.5f, 0.5f};
	static float rampSlope = 0.35f;
	static float rampLength = 8.0f;
	static float rampHalfWidthZ = 1.5f;

	static constexpr float kPi = 3.14159265358979323846f;

	static float sphereVolumeFromRadius(float radius)
	{
		radius = std::max(0.0f, radius);
		return (4.0f / 3.0f) * kPi * radius * radius * radius;
	}

	static float sphereRadiusFromVolume(float volume)
	{
		volume = std::max(0.0f, volume);
		if (volume <= 0.0f)
			return 0.0f;
		return std::cbrt((3.0f * volume) / (4.0f * kPi));
	}

	static void show_tooltip(const char *text)
	{
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
			ImGui::SetTooltip("%s", text);
	}

	static std::string g_toast_text;
	static double g_toast_until = 0.0;

	static void RequestEngineToast(const std::string &text)
	{
		if (text.empty())
			return;
		g_toast_text = text;
		g_toast_until = ImGui::GetTime() + 2.0;
	}

	static void RenderThermalLegend(const ThermalSettings &settings)
	{
		if (!settings.enabled)
			return;

		ImGui::SeparatorText("Heat Scale");
		float legendWidth = ImGui::GetContentRegionAvail().x;
		if (legendWidth <= 0.0f)
			legendWidth = 1.0f;
		const float barHeight = 18.0f;
		ImVec2 pos = ImGui::GetCursorScreenPos();
		ImDrawList *drawList = ImGui::GetWindowDrawList();
		const int segments = 64;
		for (int i = 0; i < segments; ++i)
		{
			float t0 = static_cast<float>(i) / static_cast<float>(segments);
			float t1 = static_cast<float>(i + 1) / static_cast<float>(segments);
			glm::vec3 c0 = SampleThermalGradient(t0);
			glm::vec3 c1 = SampleThermalGradient(t1);
			ImU32 col0 = ImColor(c0.r, c0.g, c0.b, 1.0f);
			ImU32 col1 = ImColor(c1.r, c1.g, c1.b, 1.0f);
			float x0 = pos.x + t0 * legendWidth;
			float x1 = pos.x + t1 * legendWidth;
			drawList->AddRectFilledMultiColor(ImVec2(x0, pos.y), ImVec2(x1, pos.y + barHeight), col0, col1, col1, col0);
		}
		drawList->AddRect(ImVec2(pos.x, pos.y), ImVec2(pos.x + legendWidth, pos.y + barHeight), ImGui::GetColorU32(ImGuiCol_Border));
		ImGui::Dummy(ImVec2(legendWidth, barHeight + 6.0f));

		auto formatLabel = [](const char *prefix, float value) {
			char buffer[32];
			std::snprintf(buffer, sizeof(buffer), "%s %.0fK", prefix, value);
			return std::string(buffer);
		};
		const std::string coldLabel = formatLabel("Cold", settings.minVisualTemperature);
		const std::string hotLabel = formatLabel("Hot", settings.maxVisualTemperature);
		const float startX = ImGui::GetCursorPosX();
		ImGui::TextUnformatted(coldLabel.c_str());
		ImGui::SameLine();
		float hotWidth = ImGui::CalcTextSize(hotLabel.c_str()).x;
		ImGui::SetCursorPosX(startX + legendWidth - hotWidth);
		ImGui::TextUnformatted(hotLabel.c_str());
		ImGui::Spacing();
	}

	static float approxRadius(const BodyState &body)
	{
		switch (body.meshType)
		{
		case MeshType::Sphere:
			return body.sphereRadius;
		case MeshType::Box:
			return std::sqrt(body.boxHalfSize.x * body.boxHalfSize.x + body.boxHalfSize.y * body.boxHalfSize.y + body.boxHalfSize.z * body.boxHalfSize.z);
		case MeshType::Ramp:
		{
			const float height = body.rampSlope * body.rampLength;
			const float halfLen = body.rampLength * 0.5f;
			const float halfH = height * 0.5f;
			return std::sqrt(halfLen * halfLen + halfH * halfH + body.rampHalfWidthZ * body.rampHalfWidthZ);
		}
		default:
			return 0.0f;
		}
	}

	static float approxNewShapeRadius()
	{
		if (shapeIndex == 0)
			return sphereRadius;
		if (shapeIndex == 1)
			return std::sqrt(boxHalfSize[0] * boxHalfSize[0] + boxHalfSize[1] * boxHalfSize[1] + boxHalfSize[2] * boxHalfSize[2]);
		const float height = rampSlope * rampLength;
		const float halfLen = rampLength * 0.5f;
		const float halfH = height * 0.5f;
		return std::sqrt(halfLen * halfLen + halfH * halfH + rampHalfWidthZ * rampHalfWidthZ);
	}

	static bool isBuoyancyHelperWall(const BuoyancySettings &settings, const BodyState &body)
	{
		if (body.meshType != MeshType::Box)
			return false;
		if (body.inverseMass != 0.0f)
			return false;

		const float halfSize = settings.beakerHalfSize;
		constexpr float wallThickness = 0.2f;
		constexpr float wallHalf = wallThickness / 2.0f;
		const float tol = 0.02f;
		const Vec3 hs = body.boxHalfSize;

		const bool matchesXWall = (std::abs(hs.x - wallHalf) < tol) && (std::abs(hs.y - halfSize) < tol) && (std::abs(hs.z - halfSize) < tol);
		const bool matchesZWall = (std::abs(hs.z - wallHalf) < tol) && (std::abs(hs.y - halfSize) < tol) && (std::abs(hs.x - halfSize) < tol);

		const float bottomHalfX = std::max(0.01f, halfSize - wallThickness);
		const float bottomHalfY = wallHalf;
		const float bottomHalfZ = std::max(0.01f, halfSize - wallThickness);
		const bool matchesBottom = (std::abs(hs.x - bottomHalfX) < tol) &&
						   (std::abs(hs.y - bottomHalfY) < tol) &&
						   (std::abs(hs.z - bottomHalfZ) < tol);

		return matchesXWall || matchesZWall || matchesBottom;
	}

	static void spawnBody(AetherAPI &api)
	{
		BodyState body;
		const BuoyancySettings buoyancy = api.getBuoyancySettings();
		const ThermalSpawnSettings thermalSpawn = api.getThermalSpawnSettings();

		int shapeChoice = shapeIndex;
		if (buoyancy.enabled)
		{
			if (shapeChoice < 0 || shapeChoice > 1)
				shapeChoice = 0;
			shapeIndex = shapeChoice;
			spawnMass = spawnDensity * spawnVolume;
			spawnSpeed[0] = spawnSpeed[1] = spawnSpeed[2] = 0.0f;
			spawnForce[0] = spawnForce[1] = spawnForce[2] = 0.0f;
			if (shapeChoice == 0)
				sphereRadius = sphereRadiusFromVolume(spawnVolume);
			else
			{
				const float h = std::cbrt(std::max(0.0f, spawnVolume) / 8.0f);
				boxHalfSize[0] = boxHalfSize[1] = boxHalfSize[2] = h;
			}
		}

		if (thermalSpawn.lockToBasicShapes && shapeChoice > 1)
		{
			shapeChoice = std::min(shapeChoice, 1);
			shapeIndex = shapeChoice;
		}

		body.position = Vec3(spawnPos[0], spawnPos[1], spawnPos[2]);
		body.velocity = Vec3(spawnSpeed[0], spawnSpeed[1], spawnSpeed[2]);
		body.forceAccum = Vec3(spawnForce[0], spawnForce[1], spawnForce[2]);
		body.mass = spawnMass;
		body.friction = PHYSICS_DEFAULT_FRICTION;
		body.restitution = PHYSICS_DEFAULT_RESTITUTION;

		BodyID createdId = 0;
		if (shapeChoice == 0)
		{
			body.meshType = MeshType::Sphere;
			body.sphereRadius = sphereRadius;
			createdId = api.createSphere(SphereSpawnInfo{body.position, body.velocity, body.forceAccum, body.sphereRadius, body.mass, body.friction, body.restitution, body.renderAlpha});
		}
		else if (shapeChoice == 1)
		{
			body.meshType = MeshType::Box;
			body.boxHalfSize = Vec3(boxHalfSize[0], boxHalfSize[1], boxHalfSize[2]);
			createdId = api.createBox(BoxSpawnInfo{body.position, body.velocity, body.forceAccum, body.boxHalfSize, body.mass, body.friction, body.restitution, body.renderAlpha});
		}
		else
		{
			body.meshType = MeshType::Ramp;
			body.rampSlope = rampSlope;
			body.rampLength = rampLength;
			body.rampHalfWidthZ = rampHalfWidthZ;
			createdId = api.createRamp(RampSpawnInfo{body.position, body.velocity, body.forceAccum, body.rampSlope, body.rampLength, body.rampHalfWidthZ, body.mass, body.friction, body.restitution, body.renderAlpha});
		}

		if (thermalSpawn.enabled)
		{
			if (auto existing = api.getBody(createdId))
			{
				BodyState edited = *existing;
				edited.thermalEnabled = true;
				edited.temperature = thermalSpawn.spawnTemperature;
				edited.heatCapacity = thermalSpawn.spawnHeatCapacity;
				edited.thermalConductivity = thermalSpawn.spawnConductivity;
				edited.thermalEmissivity = thermalSpawn.spawnEmissivity;
				api.updateBody(edited);
			}
		}

		SetSelectedBodyId(createdId);
	}
}

void RenderEnginePopups()
{
	if (g_toast_text.empty())
		return;
	if (ImGui::GetTime() > g_toast_until)
	{
		g_toast_text.clear();
		return;
	}

	ImGui::SetNextWindowBgAlpha(0.85f);
	ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 16.0f, ImGui::GetFrameHeight() + 24.0f), ImGuiCond_Always, ImVec2(1.0f, 0.0f));
	ImGui::Begin("##engine_toast", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing);
	ImGui::TextUnformatted(g_toast_text.c_str());
	ImGui::End();
}

void RenderAddBodyMenuContent(AetherAPI &api)
{
	const BuoyancySettings buoyancy = api.getBuoyancySettings();
	const ThermalSpawnSettings thermalSpawn = api.getThermalSpawnSettings();

	ImGui::SeparatorText("Spawn Body");
	if (buoyancy.enabled)
	{
		if (shapeIndex < 0 || shapeIndex > 1)
			shapeIndex = 0;
		const char *buoyShapes[] = {"Sphere", "Box"};
		ImGui::Combo("Add Shape", &shapeIndex, buoyShapes, 2);

		const Vec3 fluidCenter = buoyancy.beakerCenter;
		const float halfSize = buoyancy.beakerHalfSize;
		const float minX = fluidCenter.x - halfSize;
		const float maxX = fluidCenter.x + halfSize;
		const float minZ = fluidCenter.z - halfSize;
		const float maxZ = fluidCenter.z + halfSize;

		ImGui::DragFloat("X", &spawnPos[0], 0.1f);
		ImGui::DragFloat("Y", &spawnPos[1], 0.1f);
		ImGui::DragFloat("Z", &spawnPos[2], 0.1f);
		spawnPos[0] = std::max(minX, std::min(maxX, spawnPos[0]));
		spawnPos[2] = std::max(minZ, std::min(maxZ, spawnPos[2]));

		ImGui::DragFloat("Density", &spawnDensity, 0.05f, 0.01f, 100000.0f);
		const float prevVolume = spawnVolume;
		ImGui::DragFloat("Volume", &spawnVolume, 0.01f, 0.0001f, 100.0f);
		if (spawnVolume > 100.0f)
			spawnVolume = prevVolume;

		if (ImGui::Button("Add Body"))
		{
			static float last_add_time = -1000.0f;
			float now = ImGui::GetTime();
			if (now - last_add_time < 2.0f)
				RequestEngineToast("Too frequent requests");
			else
			{
				last_add_time = now;
				Vec3 pos(spawnPos[0], spawnPos[1], spawnPos[2]);
				if (std::abs(pos.x - fluidCenter.x) > halfSize || std::abs(pos.z - fluidCenter.z) > halfSize)
					RequestEngineToast("Body spawn must be inside beaker X/Z.");
				else
				{
					if (shapeIndex == 0)
						sphereRadius = sphereRadiusFromVolume(spawnVolume);
					else
					{
						const float h = std::cbrt(std::max(0.0f, spawnVolume) / 8.0f);
						boxHalfSize[0] = boxHalfSize[1] = boxHalfSize[2] = h;
					}
					const float newRad = approxNewShapeRadius();
					const auto bodies = api.getBodies();
					int overlap = 0;
					for (const auto &b : bodies)
					{
						if (buoyancy.enabled && b.inverseMass == 0.0f)
							continue;
						const float r = approxRadius(b);
						const float dx = b.position.x - pos.x;
						const float dy = b.position.y - pos.y;
						const float dz = b.position.z - pos.z;
						const float dist2 = dx * dx + dy * dy + dz * dz;
						const float rsum = newRad + r;
						const float thresh = rsum * 0.85f;
						if (dist2 < thresh * thresh)
							++overlap;
						if (overlap >= 4)
							break;
					}
					if (overlap >= 4)
						RequestEngineToast("area too clustered to add a body");
					else
					{
						spawnBody(api);
						RequestEngineToast("Body was added successfully");
					}
				}
			}
		}
		show_tooltip("Creates one rigid body with the current spawn parameters.");
		return;
	}

	const bool restrictShapes = thermalSpawn.lockToBasicShapes;
	const char *shapeNamesFull[] = {"Sphere", "Box", "Ramp"};
	const char *shapeNamesLimited[] = {"Sphere", "Box"};
	if (restrictShapes && shapeIndex > 1)
		shapeIndex = 0;
	const char **shapeNames = restrictShapes ? shapeNamesLimited : shapeNamesFull;
	const int shapeCount = restrictShapes ? 2 : 3;
	ImGui::Combo("Add Shape", &shapeIndex, shapeNames, shapeCount);
	show_tooltip("Choose which collider type to create for the next body.");

	ImGui::DragFloat3("Position", spawnPos, 0.1f);
	ImGui::DragFloat3("Speed", spawnSpeed, 0.1f);
	ImGui::DragFloat3("Force", spawnForce, 0.1f);
	ImGui::DragFloat("Mass", &spawnMass, 0.1f, 0.0f, 100000.0f);

	if (shapeIndex == 0)
		ImGui::DragFloat("Sphere Radius", &sphereRadius, 0.01f, 0.0f, 100000.0f);
	else if (shapeIndex == 1)
		ImGui::DragFloat3("Box Halfsize", boxHalfSize, 0.01f);
	else
	{
		ImGui::DragFloat("Ramp Slope", &rampSlope, 0.01f);
		ImGui::DragFloat("Ramp Length", &rampLength, 0.1f);
		ImGui::DragFloat("Ramp HalfWidthZ", &rampHalfWidthZ, 0.1f);
	}

	if (thermalSpawn.enabled)
	{
		ImGui::SeparatorText("Thermal Properties");
		ImGui::DragFloat("Spawn Temperature (K)", const_cast<float*>(&thermalSpawn.spawnTemperature), 1.0f, 100.0f, 1000.0f);
		ImGui::DragFloat("Spawn Heat Capacity", const_cast<float*>(&thermalSpawn.spawnHeatCapacity), 5.0f, 10.0f, 5000.0f);
		ImGui::DragFloat("Spawn Conductivity", const_cast<float*>(&thermalSpawn.spawnConductivity), 0.01f, 0.0f, 5.0f);
		ImGui::DragFloat("Spawn Emissivity", const_cast<float*>(&thermalSpawn.spawnEmissivity), 0.01f, 0.0f, 1.5f);
	}

	if (ImGui::Button("Add Body"))
	{
		static float last_add_time = -1000.0f;
		float now = ImGui::GetTime();
		if (now - last_add_time < 2.0f)
			RequestEngineToast("Too frequent requests");
		else
		{
			last_add_time = now;
			Vec3 pos(spawnPos[0], spawnPos[1], spawnPos[2]);
			const float newRad = approxNewShapeRadius();
			const auto bodies = api.getBodies();
			int overlap = 0;
			for (const auto &b : bodies)
			{
				if (buoyancy.enabled && b.inverseMass == 0.0f)
					continue;
				const float r = approxRadius(b);
				const float dx = b.position.x - pos.x;
				const float dy = b.position.y - pos.y;
				const float dz = b.position.z - pos.z;
				const float dist2 = dx * dx + dy * dy + dz * dz;
				const float rsum = newRad + r;
				const float thresh = rsum * 0.85f;
				if (dist2 < thresh * thresh)
					++overlap;
				if (overlap >= 3)
					break;
			}
			if (overlap >= 3)
				RequestEngineToast("area too clustered to add a body");
			else
			{
				spawnBody(api);
				RequestEngineToast("Body was added successfully");
			}
		}
	}
	show_tooltip("Creates one rigid body with the current spawn parameters.");
}

void RenderConstraintMenuContent(AetherAPI &api)
{
	ImGui::SeparatorText("Constraints");

	const auto bodies = api.getBodies();
	const int n = static_cast<int>(bodies.size());
	if (n < 2)
	{
		linkBodyAIndex = 0;
		linkBodyBIndex = 0;
	}
	else
	{
		if (linkBodyAIndex >= n)
			linkBodyAIndex = n - 1;
		if (linkBodyBIndex >= n)
			linkBodyBIndex = n - 1;
		if (linkBodyAIndex == linkBodyBIndex)
			linkBodyBIndex = (linkBodyAIndex + 1) % n;
	}

	ImGui::Text("Link two bodies");
	std::vector<std::string> linkLabels;
	linkLabels.reserve(bodies.size());
	for (const auto &b : bodies)
	{
		const char *t = "?";
		if (b.meshType == MeshType::Sphere)
			t = "Sphere";
		else if (b.meshType == MeshType::Box)
			t = "Box";
		else if (b.meshType == MeshType::Ramp)
			t = "Ramp";
		linkLabels.push_back("ID " + std::to_string(b.id) + " (" + t + ")");
	}
	std::vector<const char *> linkItems;
	linkItems.reserve(linkLabels.size());
	for (auto &s : linkLabels)
		linkItems.push_back(s.c_str());

	if (n >= 2)
	{
		ImGui::Combo("Body A", &linkBodyAIndex, linkItems.data(), n);
		ImGui::Combo("Body B", &linkBodyBIndex, linkItems.data(), n);
		const char *linkNames[] = {"Rope", "Rod", "Spring"};
		ImGui::Combo("Link type", &linkKindIndex, linkNames, 3);
		ImGui::DragFloat("Rest length", &linkRestLength, 0.05f, 0.01f, 1000.0f);
		ImGui::DragFloat("Stiffness", &linkStiffness, 0.05f, 0.0f, 1000.0f);
		ImGui::DragFloat("Damping", &linkDamping, 0.05f, 0.0f, 1000.0f);
		if (ImGui::Button("Add rope / rod / spring"))
		{
			if (linkBodyAIndex != linkBodyBIndex)
			{
				BodyID idA = bodies[static_cast<std::size_t>(linkBodyAIndex)].id;
				BodyID idB = bodies[static_cast<std::size_t>(linkBodyBIndex)].id;
				ConstraintType type = ConstraintType::Rope;
				if (linkKindIndex == 1)
					type = ConstraintType::Rod;
				else if (linkKindIndex == 2)
					type = ConstraintType::Spring;
				RequestEngineToast(api.createDistanceConstraint(idA, idB, linkRestLength, type, linkStiffness, linkDamping) ? "Constraint added successfully" : "Failed to add constraint");
			}
		}
	}
	else
		ImGui::TextDisabled("Need at least two bodies to add a link.");
}

void RenderWorldMenuContent(AetherAPI &api)
{
	ImGui::SeparatorText("World");
	Vec3 gravity = api.getGravity();
	float gravityY = gravity.y;
	if (ImGui::DragFloat("Gravity Y", &gravityY, 0.1f, -100000.0f, 100000.0f))
		api.setGravity(Vec3(gravity.x, gravityY, gravity.z));
}

void RenderBodyInspectorContent(AetherAPI &api, bool showCloseButton)
{
	if (showCloseButton && ImGui::Button("Back"))
	{
		ImGui::CloseCurrentPopup();
		return;
	}

	if (showCloseButton)
		ImGui::Separator();

	ImGui::SeparatorText("Bodies");
	RenderThermalLegend(api.getThermalSettings());

	float listHeight = ImGui::GetContentRegionAvail().y;
	if (showCloseButton)
		listHeight = (listHeight > 44.0f) ? (listHeight - 44.0f) : listHeight;
	ImGui::BeginChild("BodyList", ImVec2(0, listHeight), true);

	const auto bodies = api.getBodies();
	for (const auto &body : bodies)
	{
		ImGui::PushID(body.id);
		if (isBuoyancyHelperWall(api.getBuoyancySettings(), body))
		{
			ImGui::PopID();
			continue;
		}

		const bool isSelected = body.id == GetSelectedBodyId();
		const bool isLive = body.inverseMass != 0.0f;
		const char *typeStr = "Unknown";
		if (body.meshType == MeshType::Sphere)
			typeStr = "Sphere";
		else if (body.meshType == MeshType::Box)
			typeStr = "Box";
		else if (body.meshType == MeshType::Ramp)
			typeStr = "Ramp";

		std::string label = "Body " + std::to_string(body.id) + " (" + typeStr + ")";
		if (ImGui::Selectable(label.c_str(), isSelected))
			SetSelectedBodyId(body.id);

		ImGui::SameLine();
		ImGui::TextUnformatted(isLive ? "Live" : "Static");
		ImGui::Text("Pos: %.3f %.3f %.3f", body.position.x, body.position.y, body.position.z);
		ImGui::Text("Speed: %.3f %.3f %.3f", body.velocity.x, body.velocity.y, body.velocity.z);
		ImGui::Text("Force: %.3f %.3f %.3f", body.forceAccum.x, body.forceAccum.y, body.forceAccum.z);
		if (body.thermalEnabled)
		{
			ImGui::Text("Temp: %.1f K", body.temperature);
			ImGui::Text("k: %.2f | emiss: %.2f", body.thermalConductivity, body.thermalEmissivity);
		}

		if (isSelected)
		{
			BodyState edited = body;
			bool changed = false;
			if (body.meshType == MeshType::Sphere)
			{
				float currentVolume = sphereVolumeFromRadius(edited.sphereRadius);
				float currentDensity = (currentVolume > 1e-6f) ? (edited.mass / currentVolume) : 0.0f;
				float editVolume = currentVolume;
				float editDensity = currentDensity;
				changed |= ImGui::DragFloat("Volume", &editVolume, 0.01f, 0.0001f, 100.0f);
				changed |= ImGui::DragFloat("Density", &editDensity, 0.05f, 0.01f, 100000.0f);
				if (changed)
				{
					edited.sphereRadius = sphereRadiusFromVolume(editVolume);
					edited.mass = editDensity * editVolume;
				}
			}
			else if (body.meshType == MeshType::Box)
			{
				float currentVolume = (2.0f * edited.boxHalfSize.x) * (2.0f * edited.boxHalfSize.y) * (2.0f * edited.boxHalfSize.z);
				float currentDensity = (currentVolume > 1e-6f) ? (edited.mass / currentVolume) : 0.0f;
				float editVolume = currentVolume;
				float editDensity = currentDensity;
				changed |= ImGui::DragFloat("Volume", &editVolume, 0.01f, 0.0001f, 100.0f);
				changed |= ImGui::DragFloat("Density", &editDensity, 0.05f, 0.01f, 100000.0f);
				if (changed)
				{
					const float scale = std::cbrt(std::max(0.0001f, editVolume) / std::max(1e-6f, currentVolume));
					edited.boxHalfSize = Vec3(edited.boxHalfSize.x * scale, edited.boxHalfSize.y * scale, edited.boxHalfSize.z * scale);
					edited.mass = editDensity * editVolume;
				}
			}
			else if (body.meshType == MeshType::Ramp)
			{
				changed |= ImGui::DragFloat("Ramp Slope", &edited.rampSlope, 0.01f);
				changed |= ImGui::DragFloat("Ramp Length", &edited.rampLength, 0.1f);
				changed |= ImGui::DragFloat("Ramp HalfWidthZ", &edited.rampHalfWidthZ, 0.1f);
			}

			if (!api.getThermalSettings().enabled)
			{
				float editSpeed[3] = {edited.velocity.x, edited.velocity.y, edited.velocity.z};
				if (ImGui::DragFloat3("Edit Speed", editSpeed, 0.1f))
				{
					edited.velocity = Vec3(editSpeed[0], editSpeed[1], editSpeed[2]);
					changed = true;
				}

				float editForce[3] = {edited.forceAccum.x, edited.forceAccum.y, edited.forceAccum.z};
				if (ImGui::DragFloat3("Edit Force", editForce, 0.1f))
				{
					edited.forceAccum = Vec3(editForce[0], editForce[1], editForce[2]);
					changed = true;
				}
			}

			if (edited.thermalEnabled)
			{
				float editTemp = edited.temperature;
				if (ImGui::DragFloat("Edit Temperature", &editTemp, 0.5f, 100.0f, 1000.0f))
				{
					edited.temperature = editTemp;
					changed = true;
				}
			}

			if (changed)
				api.updateBody(edited);
		}

		ImGui::Separator();
		ImGui::PopID();
	}

	ImGui::EndChild();

	ImGui::SeparatorText("Selected Body Actions");
	const BodyID selectedId = GetSelectedBodyId();
	if (selectedId == 0)
		ImGui::BeginDisabled();

	if (ImGui::Button("Remove Selected Body"))
	{
		const bool removed = api.deleteBody(selectedId);
		RequestEngineToast(removed ? "Body was removed successfully" : "Failed to remove body");
		if (removed)
			SetSelectedBodyId(0);
	}

	if (selectedId == 0)
		ImGui::EndDisabled();
}

void RenderBodyMenu(AetherAPI &api)
{
	ImGui::Begin("Body Menu");
	RenderAddBodyMenuContent(api);
	if (!api.getBuoyancySettings().enabled)
		RenderConstraintMenuContent(api);
	RenderWorldMenuContent(api);
	RenderBodyInspectorContent(api, false);
	ImGui::End();
}
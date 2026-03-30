#include "bodymenu.hpp"

#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "../engine/core/box_collider.hpp"
#include "bodyselection.hpp"
#include "../engine/core/ramp_collider.hpp"
#include "../engine/core/rigidbody.hpp"
#include "../engine/core/sphere_collider.hpp"
#include "../engine/math/vec3.hpp"
#include "thermal_palette.hpp"

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

static float sphereRadius = 0.5f;
static float boxHalfSize[3] = {0.5f, 0.5f, 0.5f};
static float rampSlope = 0.35f;
static float rampLength = 8.0f;
static float rampHalfWidthZ = 1.5f;

static std::vector<std::unique_ptr<Collider>> ownedColliders;

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

static void RenderThermalLegend(const PhysicsWorld &world)
{
	if (!world.thermal_settings.enabled)
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
	const std::string coldLabel = formatLabel("Cold", world.thermal_settings.min_visual_temperature);
	const std::string hotLabel = formatLabel("Hot", world.thermal_settings.max_visual_temperature);
	const float startX = ImGui::GetCursorPosX();
	ImGui::TextUnformatted(coldLabel.c_str());
	ImGui::SameLine();
	float hotWidth = ImGui::CalcTextSize(hotLabel.c_str()).x;
	ImGui::SetCursorPosX(startX + legendWidth - hotWidth);
	ImGui::TextUnformatted(hotLabel.c_str());
	ImGui::Spacing();
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

static float approx_radius_from_collider(const Collider *c)
{
	if (!c)
		return 0.0f;
	if (c->type == ShapeType::Sphere)
		return static_cast<const SphereCollider *>(c)->radius;
	if (c->type == ShapeType::Box)
	{
		const Vec3 &hs = static_cast<const BoxCollider *>(c)->halfsize;
		return std::sqrt(hs.x * hs.x + hs.y * hs.y + hs.z * hs.z);
	}
	const RampCollider *rc = static_cast<const RampCollider *>(c);
	const float height = rc->getHeight();
	const float halfLen = rc->length * 0.5f;
	const float halfH = height * 0.5f;
	return std::sqrt(halfLen * halfLen + halfH * halfH + rc->half_width_z * rc->half_width_z);
}

static float approx_radius_for_new_shape()
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

static void spawn_body(PhysicsWorld &world)
{
	Collider *collider_ptr = nullptr;
	int shapeChoice = shapeIndex;
	if (world.thermal_spawn_controls.lock_to_basic_shapes && shapeChoice > 1)
	{
		shapeChoice = std::min(shapeChoice, 1);
		shapeIndex = shapeChoice;
	}

	if (shapeChoice == 0) // spawn sphere
	{
		auto c = std::make_unique<SphereCollider>(sphereRadius);
		collider_ptr = c.get();
		ownedColliders.push_back(std::move(c));
	}
	else if (shapeChoice == 1) // spawn box
	{
		auto c = std::make_unique<BoxCollider>(Vec3(boxHalfSize[0], boxHalfSize[1], boxHalfSize[2]));
		collider_ptr = c.get();
		ownedColliders.push_back(std::move(c));
	}
	else // spawn ramp
	{
		auto c = std::make_unique<RampCollider>(rampSlope, rampLength, rampHalfWidthZ);
		collider_ptr = c.get();
		ownedColliders.push_back(std::move(c));
	}

	// finally create the rigid body
	Rigidbody b(
		Vec3(spawnPos[0], spawnPos[1], spawnPos[2]),
		Vec3(spawnSpeed[0], spawnSpeed[1], spawnSpeed[2]),
		collider_ptr,
		spawnMass);
	b.force_accum = Vec3(spawnForce[0], spawnForce[1], spawnForce[2]);
	if (world.thermal_spawn_controls.enabled)
	{
		b.thermal_enabled = true;
		b.temperature = world.thermal_spawn_controls.spawn_temperature;
		b.heat_capacity = world.thermal_spawn_controls.spawn_heat_capacity;
		b.thermal_conductivity = world.thermal_spawn_controls.spawn_conductivity;
		b.thermal_emissivity = world.thermal_spawn_controls.spawn_emissivity;
	}
	SetSelectedBodyId(world.addBody(std::move(b)));
}

void RenderAddBodyMenuContent(PhysicsWorld &world)
{
	ImGui::SeparatorText("Spawn Body");
	const bool restrictShapes = world.thermal_spawn_controls.lock_to_basic_shapes;
	const char *shapeNamesFull[] = {"Sphere", "Box", "Ramp"};
	const char *shapeNamesLimited[] = {"Sphere", "Box"};
	if (restrictShapes && shapeIndex > 1)
	{
		shapeIndex = 0;
	}
	const char **shapeNames = restrictShapes ? shapeNamesLimited : shapeNamesFull;
	int shapeCount = restrictShapes ? 2 : 3;
	ImGui::Combo("Add Shape", &shapeIndex, shapeNames, shapeCount);
	show_tooltip("Choose which collider type to create for the next body.");
	if (restrictShapes)
	{
		ImGui::TextColored(ImVec4(0.95f, 0.65f, 0.25f, 1.0f), "Thermal scenario: only spheres and boxes are available.");
	}

	ImGui::DragFloat3("Position", spawnPos, 0.1f);
	show_tooltip("Initial world-space position for the new body.");
	ImGui::DragFloat3("Speed", spawnSpeed, 0.1f);
	show_tooltip("Initial linear velocity applied on spawn.");
	ImGui::DragFloat3("Force", spawnForce, 0.1f);
	show_tooltip("Initial accumulated force. Useful for immediate pushes.");
	ImGui::DragFloat("Mass", &spawnMass, 0.1f, 0.0f, 100000.0f);
	show_tooltip("Higher mass resists acceleration. 0.0 means static body.");

	if (shapeIndex == 0)
	{
		ImGui::DragFloat("Sphere Radius", &sphereRadius, 0.01f, 0.0f, 100000.0f);
		show_tooltip("Radius of the spawned sphere collider.");
	}
	else if (shapeIndex == 1)
	{
		ImGui::DragFloat3("Box Halfsize", boxHalfSize, 0.01f);
		show_tooltip("Half extents of the box along X, Y, and Z.");
	}
	else
	{
		ImGui::DragFloat("Ramp Slope", &rampSlope, 0.01f);
		show_tooltip("Vertical rise per unit horizontal run for the ramp.");
		ImGui::DragFloat("Ramp Length", &rampLength, 0.1f);
		show_tooltip("Ramp size along the forward axis.");
		ImGui::DragFloat("Ramp HalfWidthZ", &rampHalfWidthZ, 0.1f);
		show_tooltip("Half-width of the ramp across the Z axis.");
	}

	if (world.thermal_spawn_controls.enabled)
	{
		ImGui::SeparatorText("Thermal Properties");
		ImGui::DragFloat("Spawn Temperature (K)", &world.thermal_spawn_controls.spawn_temperature, 1.0f, 100.0f, 1000.0f);
		show_tooltip("Temperature assigned to newly spawned bodies inside the heat transfer lab.");
		ImGui::DragFloat("Spawn Heat Capacity", &world.thermal_spawn_controls.spawn_heat_capacity, 5.0f, 10.0f, 5000.0f);
		show_tooltip("Higher heat capacity slows down temperature changes.");
		ImGui::DragFloat("Spawn Conductivity", &world.thermal_spawn_controls.spawn_conductivity, 0.01f, 0.0f, 5.0f);
		show_tooltip("Controls how quickly bodies exchange heat via contacts.");
		ImGui::DragFloat("Spawn Emissivity", &world.thermal_spawn_controls.spawn_emissivity, 0.01f, 0.0f, 1.5f);
		show_tooltip("Higher emissivity radiates heat faster to nearby bodies.");
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
			const float newRad = approx_radius_for_new_shape();
			const auto &bodies = world.getBodies();
			int overlap = 0;
			for (auto &b : bodies)
			{
				if (!b.collider)
					continue;
				const float r = approx_radius_from_collider(b.collider);
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
				spawn_body(world);
				RequestEngineToast("Body was added successfully");
			}
		}
	}
	show_tooltip("Creates one rigid body with the current spawn parameters.");
}

void RenderConstraintMenuContent(PhysicsWorld &world)
{
	ImGui::SeparatorText("Constraints");

	{
		auto &bodies = world.getBodies();
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
		for (auto &b : bodies)
		{
			const char *t = "?";
			if (b.collider)
			{
				if (b.collider->type == ShapeType::Sphere)
					t = "Sphere";
				else if (b.collider->type == ShapeType::Box)
					t = "Box";
				else if (b.collider->type == ShapeType::Ramp)
					t = "Ramp";
			}
			linkLabels.push_back("ID " + std::to_string(b.id) + " (" + t + ")");
		}
		std::vector<const char *> linkItems;
		linkItems.reserve(linkLabels.size());
		for (auto &s : linkLabels)
			linkItems.push_back(s.c_str());

		if (n >= 2)
		{
			ImGui::Combo("Body A", &linkBodyAIndex, linkItems.data(), n);
			show_tooltip("First body in the distance constraint pair.");
			ImGui::Combo("Body B", &linkBodyBIndex, linkItems.data(), n);
			show_tooltip("Second body in the distance constraint pair.");
			const char *linkNames[] = {"Rope", "Rod", "Spring"};
			ImGui::Combo("Link type", &linkKindIndex, linkNames, 3);
			show_tooltip("Rope = max length, Rod = fixed length, Spring = elastic.");
			ImGui::DragFloat("Rest length", &linkRestLength, 0.05f, 0.01f, 1000.0f);
			show_tooltip("Target distance used by rope, rod, and spring links.");
			ImGui::DragFloat("Stiffness", &linkStiffness, 0.05f, 0.0f, 1000.0f);
			show_tooltip("How strongly a spring pulls bodies toward its rest length.");
			ImGui::DragFloat("Damping", &linkDamping, 0.05f, 0.0f, 1000.0f);
			show_tooltip("Reduces oscillation and jitter in spring-like motion.");
			if (ImGui::Button("Add rope / rod / spring"))
			{
				if (linkBodyAIndex != linkBodyBIndex)
				{
					std::uint32_t idA = bodies[static_cast<std::size_t>(linkBodyAIndex)].id;
					std::uint32_t idB = bodies[static_cast<std::size_t>(linkBodyBIndex)].id;
					DistanceConstraint::TYPE t = DistanceConstraint::ROPE;
					if (linkKindIndex == 1)
						t = DistanceConstraint::ROD;
					else if (linkKindIndex == 2)
						t = DistanceConstraint::SPRING;
					PhysicsResult res = world.addDistanceConstraints(idA, idB, linkRestLength, t, linkStiffness, linkDamping);
					RequestEngineToast(res.message);
				}
			}
		}
		else
			ImGui::TextDisabled("Need at least two bodies to add a link.");
	}
}

void RenderWorldMenuContent(PhysicsWorld &world)
{
	ImGui::SeparatorText("World");

	// gravity section
	{
		Vec3 gravity = world.getGravity();
		float gravityY = gravity.y;
		if (ImGui::DragFloat("Gravity Y", &gravityY, 0.1f, -100000.0f, 100000.0f))
			world.setGravity(Vec3(gravity.x, gravityY, gravity.z));
		show_tooltip("Negative values pull downward, positive values push upward.");
	}
}

void RenderBodyInspectorContent(PhysicsWorld &world, bool showCloseButton)
{
	if (showCloseButton && ImGui::Button("Back"))
	{
		ImGui::CloseCurrentPopup();
		return;
	}

	if (showCloseButton)
		ImGui::Separator();

	ImGui::SeparatorText("Bodies");
	if (world.thermal_settings.enabled)
	{
		RenderThermalLegend(world);
	}

	// show active bodies in the scene
	float listHeight = ImGui::GetContentRegionAvail().y;
	if (showCloseButton)
		listHeight = (listHeight > 44.0f) ? (listHeight - 44.0f) : listHeight;
	ImGui::BeginChild("BodyList", ImVec2(0, listHeight), true);

	auto &bodies = world.getBodies();
	for (auto &body : bodies)
	{
		ImGui::PushID(body.id);

		if (!body.collider)
		{
			ImGui::PopID();
			continue;
		}

		const bool isSelected = body.id == GetSelectedBodyId();
		const bool isLive = body.inverse_mass != 0.0f;

		const char *typeStr = "Unknown";
		if (body.collider->type == ShapeType::Sphere)
			typeStr = "Sphere";
		else if (body.collider->type == ShapeType::Box)
			typeStr = "Box";
		else if (body.collider->type == ShapeType::Ramp)
			typeStr = "Ramp";

		std::string label = "Body " + std::to_string(body.id) + " (" + typeStr + ")";

		if (ImGui::Selectable(label.c_str(), isSelected))
			SetSelectedBodyId(body.id);

		ImGui::SameLine();
		ImGui::TextUnformatted(isLive ? "Live" : "Static");

		ImGui::Text("Pos: %.3f %.3f %.3f", body.position.x, body.position.y, body.position.z);
		ImGui::Text("Speed: %.3f %.3f %.3f", body.velocity.x, body.velocity.y, body.velocity.z);
		ImGui::Text("Force: %.3f %.3f %.3f", body.force_accum.x, body.force_accum.y, body.force_accum.z);
		if (body.thermal_enabled)
		{
			ImGui::Text("Temp: %.1f K", body.temperature);
			ImGui::Text("k: %.2f | emiss: %.2f", body.thermal_conductivity, body.thermal_emissivity);
		}

		if (isSelected && isLive)
		{
			float editSpeed[3] = {body.velocity.x, body.velocity.y, body.velocity.z};
			if (ImGui::DragFloat3("Edit Speed", editSpeed, 0.1f))
				body.velocity = Vec3(editSpeed[0], editSpeed[1], editSpeed[2]);

			float editForce[3] = {body.force_accum.x, body.force_accum.y, body.force_accum.z};
			if (ImGui::DragFloat3("Edit Force", editForce, 0.1f))
				body.force_accum = Vec3(editForce[0], editForce[1], editForce[2]);
			if (body.thermal_enabled)
			{
				float editTemp = body.temperature;
				if (ImGui::DragFloat("Edit Temperature", &editTemp, 0.5f, 100.0f, 1000.0f))
				{
					body.temperature = editTemp;
				}
			}
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
		PhysicsResult res = world.deleteBody(selectedId);
		RequestEngineToast(res.message);
		if (res.success)
			SetSelectedBodyId(0);
	}
	show_tooltip("Deletes the currently selected body from the world.");

	if (selectedId == 0)
		ImGui::EndDisabled();
}

void RenderBodyMenu(PhysicsWorld &world)
{
	ImGui::Begin("Body Menu");
	RenderAddBodyMenuContent(world);
	RenderConstraintMenuContent(world);
	RenderWorldMenuContent(world);
	RenderBodyInspectorContent(world, false);
	ImGui::End();
}

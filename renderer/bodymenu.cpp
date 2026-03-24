#include "bodymenu.hpp"

#include <imgui.h>
#include <memory>
#include <string>
#include <vector>

#include "../engine/core/box_collider.hpp"
#include "bodyselection.hpp"
#include "../engine/core/ramp_collider.hpp"
#include "../engine/core/rigidbody.hpp"
#include "../engine/core/sphere_collider.hpp"

static int shapeIndex = 0;

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

static void spawn_body(PhysicsWorld &world)
{
	Collider *collider_ptr = nullptr;

	if (shapeIndex == 0) // spawn sphere
	{
		auto c = std::make_unique<SphereCollider>(sphereRadius);
		collider_ptr = c.get();
		ownedColliders.push_back(std::move(c));
	}
	else if (shapeIndex == 1) // spawn box
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
	SetSelectedBodyId(world.addBody(std::move(b)));
}

void RenderBodyMenu(PhysicsWorld &world)
{
	ImGui::SetNextWindowSize(ImVec2(420.0f, 760.0f), ImGuiCond_FirstUseEver);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 10.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_GrabRounding, 6.0f);
	ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f, 0.10f, 0.14f, 0.94f));
	ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.12f, 0.21f, 0.34f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.18f, 0.30f, 0.46f, 0.55f));
	ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.24f, 0.40f, 0.62f, 0.82f));
	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f, 0.36f, 0.60f, 0.72f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.48f, 0.75f, 0.92f));

	ImGui::Begin("Body Menu");

	ImGui::TextColored(ImVec4(0.75f, 0.90f, 1.0f, 1.0f), "Spawn and Edit Bodies");
	ImGui::Separator();

	// add shape section
	const char *shapeNames[] = {"Sphere", "Box", "Ramp"};
	ImGui::Combo("Shape", &shapeIndex, shapeNames, 3);

	ImGui::DragFloat3("Position", spawnPos, 0.1f);
	ImGui::DragFloat3("Speed", spawnSpeed, 0.1f);
	ImGui::DragFloat3("Force", spawnForce, 0.1f);
	ImGui::DragFloat("Mass", &spawnMass, 0.1f, 0.0f, 100000.0f);

	if (shapeIndex == 0)
		ImGui::DragFloat("Sphere Radius", &sphereRadius, 0.01f, 0.0f, 100000.0f);
	else if (shapeIndex == 1)
	{
		ImGui::DragFloat3("Box Halfsize", boxHalfSize, 0.01f);
	}
	else
	{
		ImGui::DragFloat("Ramp Slope", &rampSlope, 0.01f);
		ImGui::DragFloat("Ramp Length", &rampLength, 0.1f);
		ImGui::DragFloat("Ramp HalfWidthZ", &rampHalfWidthZ, 0.1f);
	}

	if (ImGui::Button("Add Body", ImVec2(-1.0f, 0.0f)))
		spawn_body(world);

	ImGui::Separator();

	// gravity section
	{
		Vec3 gravity = world.getGravity();
		float gravityY = gravity.y;
		if (ImGui::DragFloat("Gravity Y", &gravityY, 0.1f, -100000.0f, 100000.0f))
			world.setGravity(Vec3(gravity.x, gravityY, gravity.z));
	}

	ImGui::Separator();

	// show active bodies in the scene
	ImGui::TextColored(ImVec4(0.70f, 0.85f, 1.0f, 1.0f), "Active Bodies");
	ImGui::BeginChild("BodyList", ImVec2(0, 400), true);

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

		if (isSelected && isLive)
		{
			float editSpeed[3] = {body.velocity.x, body.velocity.y, body.velocity.z};
			if (ImGui::DragFloat3("Edit Speed", editSpeed, 0.1f))
				body.velocity = Vec3(editSpeed[0], editSpeed[1], editSpeed[2]);

			float editForce[3] = {body.force_accum.x, body.force_accum.y, body.force_accum.z};
			if (ImGui::DragFloat3("Edit Force", editForce, 0.1f))
				body.force_accum = Vec3(editForce[0], editForce[1], editForce[2]);
		}

		ImGui::Separator();
		ImGui::PopID();
	}

	ImGui::EndChild();
	ImGui::End();

	ImGui::PopStyleColor(6);
	ImGui::PopStyleVar(3);
}

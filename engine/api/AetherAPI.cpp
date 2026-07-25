#include "api/AetherAPI.hpp"

#include <cmath>
#include <utility>

#include "core/box_collider.hpp"
#include "core/ramp_collider.hpp"
#include "core/rigidbody.hpp"
#include "core/sphere_collider.hpp"
#include "world/physicsworld.hpp"

namespace
{
	MeshType toMeshType(const Collider* collider)
	{
		if (collider == nullptr)
		{
			return MeshType::Unknown;
		}

		switch (collider->type)
		{
		case ShapeType::Box:
			return MeshType::Box;
		case ShapeType::Sphere:
			return MeshType::Sphere;
		case ShapeType::Ramp:
			return MeshType::Ramp;
		default:
			return MeshType::Unknown;
		}
	}

	ConstraintType toConstraintType(DistanceConstraint::TYPE type)
	{
		switch (type)
		{
		case DistanceConstraint::ROPE:
			return ConstraintType::Rope;
		case DistanceConstraint::ROD:
			return ConstraintType::Rod;
		case DistanceConstraint::SPRING:
			return ConstraintType::Spring;
		default:
			return ConstraintType::Rope;
		}
	}

	DistanceConstraint::TYPE toDistanceConstraintType(ConstraintType type)
	{
		switch (type)
		{
		case ConstraintType::Rope:
			return DistanceConstraint::ROPE;
		case ConstraintType::Rod:
			return DistanceConstraint::ROD;
		case ConstraintType::Spring:
			return DistanceConstraint::SPRING;
		default:
			return DistanceConstraint::ROPE;
		}
	}

	float computeMass(const BodyState& state)
	{
		if (state.mass > 0.0f)
		{
			return state.mass;
		}
		if (state.inverseMass > 0.0f)
		{
			return 1.0f / state.inverseMass;
		}
		return 0.0f;
	}

	void applyMassAndInertia(Rigidbody& body, const BodyState& state)
	{
		const float mass = computeMass(state);
		const float minMass = PHYSICS_EPSILON;
		float effectiveMass = mass;

		if (mass <= 0.0f)
		{
			effectiveMass = 0.0f;
		}
		else if (mass < minMass)
		{
			effectiveMass = minMass;
		}

		body.inverse_mass = (effectiveMass > 0.0f) ? (1.0f / effectiveMass) : 0.0f;
		const float actualMass = (body.inverse_mass > 0.0f) ? (1.0f / body.inverse_mass) : 0.0f;

		if (!body.collider || actualMass <= 0.0f)
		{
			body.inverse_inertia_body = Mat3::identity() * 0.0f;
			body.updateworldinvinertia();
			return;
		}

		if (body.collider->type == ShapeType::Sphere)
		{
			auto* sphere = static_cast<SphereCollider*>(body.collider);
			const float I = 0.4f * actualMass * sphere->radius * sphere->radius;
			const float invI = (I > PHYSICS_EPSILON) ? (1.0f / I) : 0.0f;
			body.inverse_inertia_body = Mat3::diag(invI, invI, invI);
		}
		else if (body.collider->type == ShapeType::Box)
		{
			auto* box = static_cast<BoxCollider*>(body.collider);
			const Vec3 e = box->halfsize * 2.0f;
			const float Ix = (actualMass / 12.0f) * (e.y * e.y + e.z * e.z);
			const float Iy = (actualMass / 12.0f) * (e.x * e.x + e.z * e.z);
			const float Iz = (actualMass / 12.0f) * (e.x * e.x + e.y * e.y);
			body.inverse_inertia_body = Mat3::diag(
				Ix > PHYSICS_EPSILON ? (1.0f / Ix) : 0.0f,
				Iy > PHYSICS_EPSILON ? (1.0f / Iy) : 0.0f,
				Iz > PHYSICS_EPSILON ? (1.0f / Iz) : 0.0f);
		}
		else
		{
			body.inverse_inertia_body = Mat3::identity() * 0.0f;
		}

		body.updateworldinvinertia();
	}

	BodyState makeState(const Rigidbody& body)
	{
		BodyState state;
		state.id = body.id;
		state.position = body.position;
		state.velocity = body.velocity;
		state.forceAccum = body.force_accum;
		state.orientation = body.orientation;
		state.angularVelocity = body.angvel;
		state.renderAlpha = body.render_alpha;
		state.thermalEnabled = body.thermal_enabled;
		state.temperature = body.temperature;
		state.heatCapacity = body.heat_capacity;
		state.thermalConductivity = body.thermal_conductivity;
		state.thermalEmissivity = body.thermal_emissivity;
		state.friction = body.friction;
		state.restitution = body.restitution;
		state.inverseMass = body.inverse_mass;
		state.mass = body.inverse_mass > 0.0f ? (1.0f / body.inverse_mass) : 0.0f;

		if (body.collider == nullptr)
		{
			state.meshType = MeshType::Unknown;
			return state;
		}

		state.meshType = toMeshType(body.collider);
		switch (body.collider->type)
		{
		case ShapeType::Sphere:
			state.sphereRadius = static_cast<const SphereCollider*>(body.collider)->radius;
			break;
		case ShapeType::Box:
			state.boxHalfSize = static_cast<const BoxCollider*>(body.collider)->halfsize;
			break;
		case ShapeType::Ramp:
		{
			auto* ramp = static_cast<const RampCollider*>(body.collider);
			state.rampSlope = ramp->slope;
			state.rampLength = ramp->length;
			state.rampHalfWidthZ = ramp->half_width_z;
			break;
		}
		default:
			break;
		}

		return state;
	}

	void applyState(Rigidbody& body, const BodyState& state)
	{
		body.position = state.position;
		body.velocity = state.velocity;
		body.force_accum = state.forceAccum;
		body.orientation = state.orientation;
		body.angvel = state.angularVelocity;
		body.render_alpha = state.renderAlpha;
		body.thermal_enabled = state.thermalEnabled;
		body.temperature = state.temperature;
		body.heat_capacity = state.heatCapacity;
		body.thermal_conductivity = state.thermalConductivity;
		body.thermal_emissivity = state.thermalEmissivity;
		body.friction = state.friction;
		body.restitution = state.restitution;
		applyMassAndInertia(body, state);

		if (body.collider == nullptr)
		{
			return;
		}

		switch (body.collider->type)
		{
		case ShapeType::Sphere:
			static_cast<SphereCollider*>(body.collider)->radius = state.sphereRadius;
			break;
		case ShapeType::Box:
			static_cast<BoxCollider*>(body.collider)->halfsize = state.boxHalfSize;
			break;
		case ShapeType::Ramp:
		{
			auto* ramp = static_cast<RampCollider*>(body.collider);
			ramp->slope = state.rampSlope;
			ramp->length = state.rampLength;
			ramp->half_width_z = state.rampHalfWidthZ;
			break;
		}
		default:
			break;
		}
	}
}

struct AetherAPI::Impl
{
	PhysicsWorld world;
	std::unordered_map<BodyID, std::unique_ptr<Collider>> ownedColliders;

	void clearOwnedCollider(BodyID bodyId)
	{
		ownedColliders.erase(bodyId);
	}

	template <typename ColliderT, typename... Args>
	std::unique_ptr<ColliderT> createCollider(Args&&... args)
	{
		return std::make_unique<ColliderT>(std::forward<Args>(args)...);
	}
};

AetherAPI::AetherAPI()
	: impl(std::make_unique<Impl>())
{
}

AetherAPI::~AetherAPI() = default;

AetherAPI::AetherAPI(AetherAPI&&) noexcept = default;

AetherAPI& AetherAPI::operator=(AetherAPI&&) noexcept = default;

void AetherAPI::step(float dt)
{
	impl->world.step(dt);
}

void AetherAPI::reset()
{
	impl = std::make_unique<Impl>();
}

BodyID AetherAPI::createBox(const BoxSpawnInfo& info)
{
	auto collider = impl->createCollider<BoxCollider>(info.halfSize);
	Rigidbody body(info.position, info.velocity, collider.get(), info.mass, info.friction, info.restitution);
	body.force_accum = info.forceAccum;
	body.render_alpha = info.renderAlpha;
	BodyID id = impl->world.addBody(std::move(body));
	impl->ownedColliders.emplace(id, std::move(collider));
	return id;
}

BodyID AetherAPI::createSphere(const SphereSpawnInfo& info)
{
	auto collider = impl->createCollider<SphereCollider>(info.radius);
	Rigidbody body(info.position, info.velocity, collider.get(), info.mass, info.friction, info.restitution);
	body.force_accum = info.forceAccum;
	body.render_alpha = info.renderAlpha;
	BodyID id = impl->world.addBody(std::move(body));
	impl->ownedColliders.emplace(id, std::move(collider));
	return id;
}

BodyID AetherAPI::createRamp(const RampSpawnInfo& info)
{
	auto collider = impl->createCollider<RampCollider>(info.slope, info.length, info.halfWidthZ);
	Rigidbody body(info.position, info.velocity, collider.get(), info.mass, info.friction, info.restitution);
	body.force_accum = info.forceAccum;
	body.render_alpha = info.renderAlpha;
	BodyID id = impl->world.addBody(std::move(body));
	impl->ownedColliders.emplace(id, std::move(collider));
	return id;
}

bool AetherAPI::deleteBody(BodyID bodyId)
{
	const PhysicsResult result = impl->world.deleteBody(bodyId);
	if (result.success)
	{
		impl->clearOwnedCollider(bodyId);
		return true;
	}
	return false;
}

bool AetherAPI::applyForce(BodyID bodyId, const Vec3& force)
{
	Rigidbody* body = impl->world.getBodyByID(bodyId);
	if (body == nullptr)
	{
		return false;
	}
	body->applyForce(force);
	return true;
}

bool AetherAPI::applyImpulse(BodyID bodyId, const Vec3& impulse)
{
	Rigidbody* body = impl->world.getBodyByID(bodyId);
	if (body == nullptr || body->inverse_mass == 0.0f)
	{
		return false;
	}
	body->velocity += impulse * body->inverse_mass;
	return true;
}

bool AetherAPI::updateBody(const BodyState& state)
{
	Rigidbody* body = impl->world.getBodyByID(state.id);
	if (body == nullptr)
	{
		return false;
	}
	applyState(*body, state);
	return true;
}

std::vector<RenderBody> AetherAPI::getRenderBodies() const
{
	std::vector<RenderBody> renderBodies;
	const auto& bodies = impl->world.getBodies();
	renderBodies.reserve(bodies.size());

	for (const auto& body : bodies)
	{
		RenderBody renderBody;
		renderBody.id = body.id;
		renderBody.meshType = toMeshType(body.collider);
		renderBody.position = body.position;
		renderBody.orientation = body.orientation;
		renderBodies.push_back(renderBody);
	}

	return renderBodies;
}

std::vector<BodyState> AetherAPI::getBodies() const
{
	std::vector<BodyState> result;
	const auto& bodies = impl->world.getBodies();
	result.reserve(bodies.size());
	for (const auto& body : bodies)
	{
		result.push_back(makeState(body));
	}
	return result;
}

std::size_t AetherAPI::getBodyCount() const
{
	return impl->world.getBodies().size();
}

std::size_t AetherAPI::getContactCount() const
{
	return impl->world.getContactCount();
}

std::optional<BodyState> AetherAPI::getBody(BodyID bodyId) const
{
	const Rigidbody* body = impl->world.getBodyByID(bodyId);
	if (body == nullptr)
	{
		return std::nullopt;
	}
	return makeState(*body);
}

std::vector<DistanceConstraintState> AetherAPI::getDistanceConstraints() const
{
	std::vector<DistanceConstraintState> result;
	const auto& constraints = impl->world.getDistanceConstraints();
	result.reserve(constraints.size());
	for (const auto& constraint : constraints)
	{
		DistanceConstraintState state;
		state.firstBodyId = constraint.a_id;
		state.secondBodyId = constraint.b_id;
		state.type = toConstraintType(constraint.type);
		state.restLength = constraint.rest_length;
		state.stiffness = constraint.stiffness;
		state.damping = constraint.damping;
		result.push_back(state);
	}
	return result;
}

bool AetherAPI::createDistanceConstraint(BodyID firstBodyId, BodyID secondBodyId, float restLength, ConstraintType type, float stiffness, float damping)
{
	const PhysicsResult result = impl->world.addDistanceConstraints(firstBodyId, secondBodyId, restLength, toDistanceConstraintType(type), stiffness, damping);
	return result.success;
}

bool AetherAPI::deleteConstraint(BodyID firstBodyId, BodyID secondBodyId)
{
	return impl->world.deleteConstraint(firstBodyId, secondBodyId).success;
}

bool AetherAPI::deleteConstraint(BodyID bodyId)
{
	return impl->world.deleteConstraint(bodyId).success;
}

BuoyancySettings AetherAPI::getBuoyancySettings() const
{
	BuoyancySettings settings;
	settings.enabled = impl->world.enable_buoyancy;
	settings.beakerCenter = impl->world.water_fluid.beaker_center;
	settings.beakerHalfSize = impl->world.water_fluid.beaker_half_size;
	settings.waterHeight = impl->world.water_fluid.height;
	settings.fluidDensity = impl->world.water_fluid.density;
	settings.dragCoefficient = impl->world.water_fluid.drag_force;
	return settings;
}

void AetherAPI::setBuoyancySettings(const BuoyancySettings& settings)
{
	impl->world.enable_buoyancy = settings.enabled;
	impl->world.water_fluid.beaker_center = settings.beakerCenter;
	impl->world.water_fluid.beaker_half_size = settings.beakerHalfSize;
	impl->world.water_fluid.height = settings.waterHeight;
	impl->world.water_fluid.density = settings.fluidDensity;
	impl->world.water_fluid.drag_force = settings.dragCoefficient;
}

ThermalSettings AetherAPI::getThermalSettings() const
{
	ThermalSettings settings;
	settings.enabled = impl->world.thermal_settings.enabled;
	settings.conductionRate = impl->world.thermal_settings.conduction_rate;
	settings.radiationRate = impl->world.thermal_settings.radiation_rate;
	settings.ambientTemperature = impl->world.thermal_settings.ambient_temperature;
	settings.ambientCoupling = impl->world.thermal_settings.ambient_coupling;
	settings.radiationDistance = impl->world.thermal_settings.radiation_distance;
	settings.minVisualTemperature = impl->world.thermal_settings.min_visual_temperature;
	settings.maxVisualTemperature = impl->world.thermal_settings.max_visual_temperature;
	return settings;
}

void AetherAPI::setThermalSettings(const ThermalSettings& settings)
{
	impl->world.thermal_settings.enabled = settings.enabled;
	impl->world.thermal_settings.conduction_rate = settings.conductionRate;
	impl->world.thermal_settings.radiation_rate = settings.radiationRate;
	impl->world.thermal_settings.ambient_temperature = settings.ambientTemperature;
	impl->world.thermal_settings.ambient_coupling = settings.ambientCoupling;
	impl->world.thermal_settings.radiation_distance = settings.radiationDistance;
	impl->world.thermal_settings.min_visual_temperature = settings.minVisualTemperature;
	impl->world.thermal_settings.max_visual_temperature = settings.maxVisualTemperature;
}

ThermalSpawnSettings AetherAPI::getThermalSpawnSettings() const
{
	ThermalSpawnSettings settings;
	settings.enabled = impl->world.thermal_spawn_controls.enabled;
	settings.lockToBasicShapes = impl->world.thermal_spawn_controls.lock_to_basic_shapes;
	settings.spawnTemperature = impl->world.thermal_spawn_controls.spawn_temperature;
	settings.spawnHeatCapacity = impl->world.thermal_spawn_controls.spawn_heat_capacity;
	settings.spawnConductivity = impl->world.thermal_spawn_controls.spawn_conductivity;
	settings.spawnEmissivity = impl->world.thermal_spawn_controls.spawn_emissivity;
	return settings;
}

void AetherAPI::setThermalSpawnSettings(const ThermalSpawnSettings& settings)
{
	impl->world.thermal_spawn_controls.enabled = settings.enabled;
	impl->world.thermal_spawn_controls.lock_to_basic_shapes = settings.lockToBasicShapes;
	impl->world.thermal_spawn_controls.spawn_temperature = settings.spawnTemperature;
	impl->world.thermal_spawn_controls.spawn_heat_capacity = settings.spawnHeatCapacity;
	impl->world.thermal_spawn_controls.spawn_conductivity = settings.spawnConductivity;
	impl->world.thermal_spawn_controls.spawn_emissivity = settings.spawnEmissivity;
}

const Vec3& AetherAPI::getGravity() const
{
	return impl->world.getGravity();
}

void AetherAPI::setGravity(const Vec3& gravity)
{
	impl->world.setGravity(gravity);
}
#pragma once

#include <optional>
#include <memory>
#include <vector>

#include "engine_configs.hpp"
#include "math/vec3.hpp"
#include "api/RenderBody.hpp"

enum class ConstraintType
{
	Rope,
	Rod,
	Spring
};

struct BodyState
{
	BodyID id = 0;
	MeshType meshType = MeshType::Unknown;
	Vec3 position{};
	Vec3 velocity{};
	Vec3 forceAccum{};
	Quat orientation{};
	Vec3 angularVelocity{};
	float mass = 1.0f;
	float inverseMass = 1.0f;
	float friction = PHYSICS_DEFAULT_FRICTION;
	float restitution = PHYSICS_DEFAULT_RESTITUTION;
	float renderAlpha = 1.0f;
	bool thermalEnabled = false;
	float temperature = 293.15f;
	float heatCapacity = 900.0f;
	float thermalConductivity = 0.5f;
	float thermalEmissivity = 0.85f;
	Vec3 boxHalfSize{0.5f, 0.5f, 0.5f};
	float sphereRadius = 0.5f;
	float rampSlope = 0.35f;
	float rampLength = 8.0f;
	float rampHalfWidthZ = 1.5f;
};

struct DistanceConstraintState
{
	BodyID firstBodyId = 0;
	BodyID secondBodyId = 0;
	ConstraintType type = ConstraintType::Rope;
	float restLength = 0.0f;
	float stiffness = 0.0f;
	float damping = 0.0f;
};

struct BuoyancySettings
{
	bool enabled = false;
	Vec3 beakerCenter{0.0f, 0.0f, 0.0f};
	float beakerHalfSize = 4.0f;
	float waterHeight = 0.0f;
	float fluidDensity = 2.0f;
	float dragCoefficient = 0.3f;
};

struct ThermalSettings
{
	bool enabled = false;
	float conductionRate = 1.0f;
	float radiationRate = 0.02f;
	float ambientTemperature = 295.0f;
	float ambientCoupling = 0.05f;
	float radiationDistance = 3.5f;
	float minVisualTemperature = 250.0f;
	float maxVisualTemperature = 650.0f;
};

struct ThermalSpawnSettings
{
	bool enabled = false;
	bool lockToBasicShapes = false;
	float spawnTemperature = 320.0f;
	float spawnHeatCapacity = 900.0f;
	float spawnConductivity = 0.6f;
	float spawnEmissivity = 0.85f;
};

struct BoxSpawnInfo
{
	Vec3 position{};
	Vec3 velocity{};
	Vec3 forceAccum{};
	Vec3 halfSize{0.5f, 0.5f, 0.5f};
	float mass = 1.0f;
	float friction = PHYSICS_DEFAULT_FRICTION;
	float restitution = PHYSICS_DEFAULT_RESTITUTION;
	float renderAlpha = 1.0f;
};

struct SphereSpawnInfo
{
	Vec3 position{};
	Vec3 velocity{};
	Vec3 forceAccum{};
	float radius = 0.5f;
	float mass = 1.0f;
	float friction = PHYSICS_DEFAULT_FRICTION;
	float restitution = PHYSICS_DEFAULT_RESTITUTION;
	float renderAlpha = 1.0f;
};

struct RampSpawnInfo
{
	Vec3 position{};
	Vec3 velocity{};
	Vec3 forceAccum{};
	float slope = 0.35f;
	float length = 8.0f;
	float halfWidthZ = 1.5f;
	float mass = 1.0f;
	float friction = PHYSICS_DEFAULT_FRICTION;
	float restitution = PHYSICS_DEFAULT_RESTITUTION;
	float renderAlpha = 1.0f;
};

class AetherAPI
{
public:
	AetherAPI();
	~AetherAPI();

	AetherAPI(const AetherAPI&) = delete;
	AetherAPI& operator=(const AetherAPI&) = delete;
	AetherAPI(AetherAPI&&) noexcept;
	AetherAPI& operator=(AetherAPI&&) noexcept;

	void step(float dt);
	void reset();

	BodyID createBox(const BoxSpawnInfo& info);
	BodyID createSphere(const SphereSpawnInfo& info);
	BodyID createRamp(const RampSpawnInfo& info);
	bool deleteBody(BodyID bodyId);
	bool applyForce(BodyID bodyId, const Vec3& force);
	bool applyImpulse(BodyID bodyId, const Vec3& impulse);
	bool updateBody(const BodyState& state);

	std::vector<RenderBody> getRenderBodies() const;
	std::vector<BodyState> getBodies() const;
	std::size_t getBodyCount() const;
	std::size_t getContactCount() const;
	std::optional<BodyState> getBody(BodyID bodyId) const;
	std::vector<DistanceConstraintState> getDistanceConstraints() const;
	bool createDistanceConstraint(BodyID firstBodyId, BodyID secondBodyId, float restLength, ConstraintType type, float stiffness, float damping);
	bool deleteConstraint(BodyID firstBodyId, BodyID secondBodyId);
	bool deleteConstraint(BodyID bodyId);

	BuoyancySettings getBuoyancySettings() const;
	void setBuoyancySettings(const BuoyancySettings& settings);
	ThermalSettings getThermalSettings() const;
	void setThermalSettings(const ThermalSettings& settings);
	ThermalSpawnSettings getThermalSpawnSettings() const;
	void setThermalSpawnSettings(const ThermalSpawnSettings& settings);

	const Vec3& getGravity() const;
	void setGravity(const Vec3& gravity);

private:
	struct Impl;
	std::unique_ptr<Impl> impl;
};
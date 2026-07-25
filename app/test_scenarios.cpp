#include "test_scenarios.hpp"

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "api/AetherAPI.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::vector<std::string> chapters = {"Kinematics", "Laws of Motion", "Collision", "Rotation", "Fluids", "Thermal Properties", "Fun Tests"};
std::unordered_map<std::string, std::vector<TestCase>> testmap;

void InitializeTestMap()
{
	testmap["Kinematics"] = {TestCase::ProjectMotion, TestCase::RelativeVelocity, TestCase::InclinedPlane};
	testmap["Laws of Motion"] = {TestCase::NewtonThirdLaw, TestCase::MomentumTransfer};
	testmap["Collision"] = {TestCase::PerfectElasticCollision, TestCase::PerfectInelasticCollision, TestCase::Collision};
	testmap["Rotation"] = {TestCase::CenterOfMassTopple, TestCase::ConstraintPlayground, TestCase::AngularImpulse, TestCase::AngularStack, TestCase::CornerCollision, TestCase::RollingFriction, TestCase::BoxToppleOnRamp, TestCase::SphereToppleOnRamp, TestCase::CollisionCauseTopple, TestCase::CircularMotionRope, TestCase::CircularMotionSpring};
	testmap["Fluids"] = {TestCase::BuoyancyTest};
	testmap["Thermal Properties"] = {TestCase::HeatTransferDemo};
	testmap["Fun Tests"] = {TestCase::PyramidStack, TestCase::ManyBoxes, TestCase::ManySpheres, TestCase::RandomScatter};
}

namespace
{
	BodyID AddBox(AetherAPI &api, const Vec3 &position, const Vec3 &halfSize, float mass = 1.0f, float friction = PHYSICS_DEFAULT_FRICTION, float restitution = PHYSICS_DEFAULT_RESTITUTION, const Vec3 &velocity = Vec3())
	{
		BoxSpawnInfo info;
		info.position = position;
		info.velocity = velocity;
		info.halfSize = halfSize;
		info.mass = mass;
		info.friction = friction;
		info.restitution = restitution;
		return api.createBox(info);
	}

	BodyID AddSphere(AetherAPI &api, const Vec3 &position, float radius, float mass = 1.0f, float friction = PHYSICS_DEFAULT_FRICTION, float restitution = PHYSICS_DEFAULT_RESTITUTION, const Vec3 &velocity = Vec3())
	{
		SphereSpawnInfo info;
		info.position = position;
		info.velocity = velocity;
		info.radius = radius;
		info.mass = mass;
		info.friction = friction;
		info.restitution = restitution;
		return api.createSphere(info);
	}

	BodyID AddRamp(AetherAPI &api, const Vec3 &position, float slope, float length, float halfWidthZ, float mass = 0.0f)
	{
		RampSpawnInfo info;
		info.position = position;
		info.slope = slope;
		info.length = length;
		info.halfWidthZ = halfWidthZ;
		info.mass = mass;
		return api.createRamp(info);
	}

	Camera DefaultCamera(float x = 0.0f, float y = 5.0f, float z = 20.0f)
	{
		return Camera().setPosition(glm::vec3(x, y, z));
	}

	void AddGround(AetherAPI &api)
	{
		AddBox(api, Vec3(0.0f, -1.0f, 0.0f), Vec3(100.0f, 0.1f, 100.0f), 0.0f, 0.0f, 1.0f);
	}

	void AddSimpleWalls(AetherAPI &api)
	{
		AddBox(api, Vec3(-12.0f, 5.0f, 0.0f), Vec3(0.5f, 8.0f, 5.0f), 0.0f, 0.0f, 1.0f);
		AddBox(api, Vec3(12.0f, 5.0f, 0.0f), Vec3(0.5f, 8.0f, 5.0f), 0.0f, 0.0f, 1.0f);
	}

	void AddBuoyancyTank(AetherAPI &api)
	{
		BuoyancySettings settings;
		settings.enabled = true;
		settings.beakerCenter = Vec3(0.0f, 4.0f, 0.0f);
		settings.beakerHalfSize = 4.0f;
		settings.waterHeight = 5.0f;
		settings.fluidDensity = 2.0f;
		settings.dragCoefficient = 0.3f;
		api.setBuoyancySettings(settings);
		AddBox(api, Vec3(-3.8f, 4.0f, 0.0f), Vec3(0.1f, 4.0f, 4.0f), 0.0f, 0.0f, 1.0f);
		AddBox(api, Vec3(3.8f, 4.0f, 0.0f), Vec3(0.1f, 4.0f, 4.0f), 0.0f, 0.0f, 1.0f);
		AddBox(api, Vec3(0.0f, 4.0f, -3.8f), Vec3(4.0f, 4.0f, 0.1f), 0.0f, 0.0f, 1.0f);
		AddBox(api, Vec3(0.0f, 4.0f, 3.8f), Vec3(4.0f, 4.0f, 0.1f), 0.0f, 0.0f, 1.0f);
		AddBox(api, Vec3(0.0f, 0.0f, 0.0f), Vec3(3.5f, 0.1f, 3.5f), 0.0f, 0.0f, 1.0f);
	}

	void AddThermalDefaults(AetherAPI &api)
	{
		ThermalSettings thermal;
		thermal.enabled = true;
		thermal.conductionRate = 1.0f;
		thermal.radiationRate = 0.02f;
		thermal.ambientTemperature = 295.0f;
		thermal.ambientCoupling = 0.05f;
		thermal.radiationDistance = 3.5f;
		thermal.minVisualTemperature = 250.0f;
		thermal.maxVisualTemperature = 650.0f;
		api.setThermalSettings(thermal);

		ThermalSpawnSettings spawn;
		spawn.enabled = true;
		spawn.lockToBasicShapes = true;
		spawn.spawnTemperature = 320.0f;
		spawn.spawnHeatCapacity = 900.0f;
		spawn.spawnConductivity = 0.6f;
		spawn.spawnEmissivity = 0.85f;
		api.setThermalSpawnSettings(spawn);
	}
}

Camera LoadSingleTestScenario(AetherAPI &api, TestCase test_case)
{
	api.reset();
	api.setGravity(Vec3(0.0f, -9.81f, 0.0f));
	AddGround(api);

	switch (test_case)
	{
	case TestCase::ProjectMotion:
		AddSphere(api, Vec3(-10.0f, 0.5f, 0.0f), 0.5f, 1.0f, 0.2f, 0.7f, Vec3(15.0f, 12.0f, 0.0f));
		AddSphere(api, Vec3(-10.0f, 0.5f, 1.5f), 0.5f, 1.0f, 0.2f, 0.7f, Vec3(15.0f, 18.0f, 0.0f));
		return DefaultCamera(2.0f, 6.0f, 25.0f);
	case TestCase::PerfectElasticCollision:
		AddSphere(api, Vec3(-8.0f, 0.5f, 0.0f), 0.5f, 1.0f, 0.0f, 1.0f, Vec3(10.0f, 0.0f, 0.0f));
		AddSphere(api, Vec3(0.0f, 0.5f, 0.0f), 0.5f, 1.0f, 0.0f, 1.0f);
		return DefaultCamera();
	case TestCase::PerfectInelasticCollision:
		AddSphere(api, Vec3(-8.0f, 0.5f, 0.0f), 0.5f, 1.0f, 0.0f, 0.0f, Vec3(10.0f, 0.0f, 0.0f));
		AddSphere(api, Vec3(0.0f, 0.5f, 0.0f), 0.5f, 1.0f, 0.0f, 0.0f);
		return DefaultCamera();
	case TestCase::Collision:
		AddBox(api, Vec3(-4.0f, 0.5f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 1.0f, 0.1f, 0.6f, Vec3(8.0f, 0.0f, 0.0f));
		AddBox(api, Vec3(2.0f, 0.5f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 1.0f, 0.1f, 0.6f);
		return DefaultCamera();
	case TestCase::InclinedPlane:
		AddRamp(api, Vec3(-4.0f, 0.3f, 0.0f), 0.35f, 8.0f, 1.5f, 0.0f);
		AddSphere(api, Vec3(2.0f, 4.0f, 0.0f), 0.5f, 2.0f);
		return DefaultCamera();
	case TestCase::MomentumTransfer:
		for (int i = 0; i < 5; ++i)
			AddSphere(api, Vec3(-8.0f + i * 1.2f, 0.5f, 0.0f), 0.5f, 0.5f, 0.0f, 0.9f, Vec3(8.0f, 0.0f, 0.0f));
		return DefaultCamera();
	case TestCase::CenterOfMassTopple:
		AddBox(api, Vec3(0.0f, 0.5f, 0.0f), Vec3(0.6f, 0.6f, 0.6f), 1.0f);
		AddSphere(api, Vec3(-4.0f, 1.5f, 0.0f), 0.8f, 2.0f, 0.2f, 0.7f, Vec3(8.0f, 0.0f, 0.0f));
		return DefaultCamera();
	case TestCase::ConstraintPlayground:
		{
			BodyID a = AddSphere(api, Vec3(-9.0f, 7.0f, 0.0f), 0.5f, 1.0f, 0.2f, 0.8f, Vec3(0.4f, -0.2f, 0.0f));
			BodyID b = AddSphere(api, Vec3(-9.2f, 3.2f, 0.0f), 0.5f, 1.1f, 0.2f, 0.8f, Vec3(-0.2f, 0.0f, 0.0f));
			api.createDistanceConstraint(a, b, 4.0f, ConstraintType::Rope, 0.0f, 0.0f);
			BodyID c = AddBox(api, Vec3(0.0f, 5.0f, 0.0f), Vec3(0.45f, 0.45f, 0.45f), 1.0f, 0.2f, 0.6f);
			BodyID d = AddBox(api, Vec3(1.2f, 5.0f, 0.0f), Vec3(0.45f, 0.45f, 0.45f), 1.0f, 0.2f, 0.6f);
			api.createDistanceConstraint(c, d, 2.0f, ConstraintType::Rod, 0.6f, 0.6f);
		}
		return DefaultCamera();
	case TestCase::AngularImpulse:
		AddBox(api, Vec3(0.0f, 0.5f, 0.0f), Vec3(0.7f, 0.7f, 0.7f), 1.0f, 0.1f, 0.7f);
		AddSphere(api, Vec3(-4.0f, 1.5f, 0.0f), 0.8f, 1.0f, 0.2f, 0.8f, Vec3(10.0f, 0.0f, 0.0f));
		return DefaultCamera();
	case TestCase::AngularStack:
		for (int i = 0; i < 3; ++i)
			AddBox(api, Vec3(0.0f, 0.5f + i * 1.05f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
		AddSphere(api, Vec3(-2.5f, 1.55f, 0.0f), 0.5f, 0.5f, 0.2f, 0.7f, Vec3(9.0f, 0.0f, 0.0f));
		return DefaultCamera();
	case TestCase::CornerCollision:
		AddBox(api, Vec3(-5.0f, 2.0f, 0.0f), Vec3(0.7f, 0.7f, 0.7f), 1.0f, 0.2f, 0.6f, Vec3(6.0f, 0.0f, 0.0f));
		AddBox(api, Vec3(3.0f, 2.0f, 0.0f), Vec3(0.7f, 0.7f, 0.7f), 1.0f, 0.2f, 0.6f);
		return DefaultCamera();
	case TestCase::RollingFriction:
		AddRamp(api, Vec3(-2.0f, 0.0f, 0.0f), 0.35f, 8.0f, 1.5f, 0.0f);
		AddSphere(api, Vec3(2.0f, 4.0f, 0.0f), 0.8f, 1.0f, 0.8f, 0.1f);
		return DefaultCamera();
	case TestCase::BuoyancyTest:
		AddBuoyancyTank(api);
		AddSphere(api, Vec3(0.0f, 6.5f, 0.0f), 0.6f, 0.5f, 0.1f, 0.5f);
		AddBox(api, Vec3(2.0f, 6.0f, 0.0f), Vec3(0.4f, 0.4f, 0.4f), 0.9f, 0.1f, 0.5f);
		return DefaultCamera(0.0f, 8.0f, 16.0f);
	case TestCase::HeatTransferDemo:
		AddThermalDefaults(api);
		for (int i = 0; i < 6; ++i)
		{
			BodyID id = AddBox(api, Vec3(-4.0f + i * 1.6f, 2.0f + (i % 2) * 0.3f, 0.0f), Vec3(0.45f, 0.45f, 0.45f), 1.8f, 0.2f, 0.5f);
			(void)id;
		}
		return DefaultCamera(0.0f, 5.0f, 18.0f);
	case TestCase::RelativeVelocity:
		AddSphere(api, Vec3(-4.0f, 0.5f, 0.0f), 0.5f, 1.0f, 0.1f, 0.6f, Vec3(4.0f, 0.0f, 0.0f));
		AddSphere(api, Vec3(4.0f, 0.5f, 0.0f), 0.5f, 1.0f, 0.1f, 0.6f, Vec3(-4.0f, 0.0f, 0.0f));
		return DefaultCamera();
	case TestCase::NewtonThirdLaw:
		AddSphere(api, Vec3(-6.0f, 0.5f, 0.0f), 0.5f, 0.5f, 0.1f, 0.6f, Vec3(6.0f, 0.0f, 0.0f));
		AddSphere(api, Vec3(0.0f, 0.5f, 0.0f), 0.8f, 2.0f, 0.1f, 0.6f);
		return DefaultCamera();
	case TestCase::BoxToppleOnRamp:
		AddRamp(api, Vec3(-3.0f, 0.0f, 0.0f), 0.7f, 6.0f, 1.2f, 0.0f);
		AddBox(api, Vec3(3.5f, 4.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 1.5f);
		return DefaultCamera();
	case TestCase::SphereToppleOnRamp:
		AddRamp(api, Vec3(-3.0f, 0.0f, 0.0f), 0.7f, 6.0f, 1.2f, 0.0f);
		AddSphere(api, Vec3(3.5f, 4.0f, 0.0f), 0.8f, 1.2f);
		return DefaultCamera();
	case TestCase::CollisionCauseTopple:
		AddRamp(api, Vec3(-3.0f, 0.0f, 0.0f), 0.35f, 8.0f, 1.5f, 0.0f);
		AddBox(api, Vec3(-8.0f, 6.0f, 0.0f), Vec3(0.6f, 0.6f, 0.6f), 2.0f, 0.1f, 0.6f, Vec3(8.0f, 0.0f, 0.0f));
		AddSphere(api, Vec3(4.0f, 6.0f, 0.0f), 0.8f, 1.0f);
		return DefaultCamera();
	case TestCase::CircularMotionRope:
		{
			BodyID fixedBox = AddBox(api, Vec3(0.0f, 0.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 0.0f);
			BodyID orbit = AddSphere(api, Vec3(4.0f, 2.0f, 0.0f), 0.5f, 1.0f, 0.2f, 0.6f, Vec3(0.0f, 0.0f, 4.0f));
			api.createDistanceConstraint(fixedBox, orbit, 4.5f, ConstraintType::Rope, 0.0f, 0.0f);
		}
		return DefaultCamera(4.0f, 4.0f, 18.0f);
	case TestCase::CircularMotionSpring:
		{
			BodyID fixedBox = AddBox(api, Vec3(0.0f, 0.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 0.0f);
			BodyID orbit = AddSphere(api, Vec3(4.0f, 2.0f, 0.0f), 0.5f, 1.0f, 0.2f, 0.6f, Vec3(0.0f, 0.0f, 4.0f));
			api.createDistanceConstraint(fixedBox, orbit, 4.5f, ConstraintType::Spring, 2.0f, 0.7f);
		}
		return DefaultCamera(4.0f, 4.0f, 18.0f);
	case TestCase::PyramidStack:
		for (int y = 0; y < 5; ++y)
			for (int x = 0; x < 5 - y; ++x)
				AddBox(api, Vec3(-4.0f + x * 1.05f, 0.5f + y * 1.05f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
		return DefaultCamera();
	case TestCase::ManyBoxes:
		for (int i = 0; i < 80; ++i)
			AddBox(api, Vec3(-10.0f + (i % 10) * 2.0f, 1.0f + (i / 10) * 1.2f, 0.0f), Vec3(0.4f, 0.4f, 0.4f), 1.0f);
		return DefaultCamera();
	case TestCase::ManySpheres:
		for (int i = 0; i < 60; ++i)
			AddSphere(api, Vec3(-8.0f + (i % 10) * 1.8f, 1.0f + (i / 10) * 1.0f, 0.0f), 0.5f, 0.5f);
		return DefaultCamera();
	case TestCase::RandomScatter:
		for (int i = 0; i < 100; ++i)
		{
			const float x = -8.0f + (i % 10) * 1.8f;
			const float y = 1.0f + (i / 10) * 1.0f;
			if (i % 2 == 0)
				AddSphere(api, Vec3(x, y, 0.0f), 0.5f, 0.6f);
			else
				AddBox(api, Vec3(x, y, 0.0f), Vec3(0.4f, 0.4f, 0.4f), 0.9f);
		}
		return DefaultCamera();
	default:
		return DefaultCamera();
	}
}
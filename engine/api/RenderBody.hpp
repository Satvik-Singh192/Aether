#pragma once

#include <cstdint>

#include "math/quat.hpp"
#include "math/vec3.hpp"

using BodyID = std::uint32_t;

enum class MeshType
{
	Box,
	Sphere,
	Ramp,
	Unknown
};

struct RenderBody
{
	std::uint32_t id = 0;
	MeshType meshType = MeshType::Unknown;
	Vec3 position{};
	Quat orientation{};
};
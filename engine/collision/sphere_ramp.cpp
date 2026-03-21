#include "collision/collision.hpp"
#include "core/ramp_collider.hpp"
#include "core/sphere_collider.hpp"
#include <algorithm>
#include <cmath>

namespace
{
    float clampValue(float value, float minValue, float maxValue)
    {
        return std::max(minValue, std::min(value, maxValue));
    }
}

bool buildRampSphereContact(Rigidbody &ramp_body, Rigidbody &sphere_body, Contact &outContact)
{
    auto *ramp = static_cast<RampCollider *>(ramp_body.collider);
    auto *sphere = static_cast<SphereCollider *>(sphere_body.collider);

    if (!ramp || !sphere)
        return false;

    const float radius = sphere->radius;
    const float length = ramp->getLength();
    const float height = ramp->getHeight();
    const float halfWidthZ = ramp->getHalfWidthZ();

    if (radius <= PHYSICS_EPSILON)
        return false;

    const float dirLengthSq = length * length + height * height;
    if (dirLengthSq <= PHYSICS_EPSILON)
        return false;

    const Vec3 localCenter = sphere_body.position - ramp_body.position;

    const float rampMinX = std::min(0.0f, length);
    const float rampMaxX = std::max(0.0f, length);
    const float rampMinY = std::min(0.0f, height);
    const float rampMaxY = std::max(0.0f, height);

    if (localCenter.x < rampMinX - radius || localCenter.x > rampMaxX + radius ||
        localCenter.y < rampMinY - radius || localCenter.y > rampMaxY + radius ||
        localCenter.z < -halfWidthZ - radius || localCenter.z > halfWidthZ + radius)
    {
        return false;
    }

    const float tUnclamped = (localCenter.x * length + localCenter.y * height) / dirLengthSq;
    const float t = clampValue(tUnclamped, 0.0f, 1.0f);

    const Vec3 closestOnSlope(length * t, height * t, clampValue(localCenter.z, -halfWidthZ, halfWidthZ));
    const Vec3 delta = localCenter - closestOnSlope;

    const float distSq = delta.dot(delta);
    if (distSq > radius * radius)
        return false;

    const float dist = std::sqrt(std::max(0.0f, distSq));
    Vec3 normal;

    if (dist <= PHYSICS_EPSILON)
    {
        const float mag = std::sqrt(dirLengthSq);
        normal = Vec3(-height / mag, length / mag, 0.0f);
    }
    else
    {
        normal = delta * (1.0f / dist);
    }

    outContact = Contact{};
    outContact.a = &ramp_body;
    outContact.b = &sphere_body;
    outContact.a_id = ramp_body.id;
    outContact.b_id = sphere_body.id;
    outContact.normal = normal;
    outContact.penetration = radius - dist;
    outContact.contact_point = ramp_body.position + closestOnSlope;
    outContact.restitution = (ramp_body.restitution+ sphere_body.restitution)*0.5f;
    outContact.friction_coeff = std::sqrt(ramp_body.friction * sphere_body.friction);

    return true;
}

bool buildSphereRampContact(Rigidbody &sphere_body, Rigidbody &ramp_body, Contact &outContact)
{
    if (!buildRampSphereContact(ramp_body, sphere_body, outContact))
        return false;

    std::swap(outContact.a, outContact.b);
    std::swap(outContact.a_id, outContact.b_id);
    outContact.normal = outContact.normal * -1.0f;
    return true;
}

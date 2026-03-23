#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/ramp_collider.hpp"
#include "core/sphere_collider.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
    float clampValue(float value, float minValue, float maxValue)
    {
        return std::max(minValue, std::min(value, maxValue));
    }

    bool isValidFloat(float v)
    {
        return std::isfinite(v);
    }

    bool isValidVec3(const Vec3 &v)
    {
        return isValidFloat(v.x) && isValidFloat(v.y) && isValidFloat(v.z);
    }
}

// A is Ramp, B is sphere
bool buildRampSphereManifold(Rigidbody &A, Rigidbody &B, ContactManifold &manifold)
{
    auto *ramp = static_cast<RampCollider *>(A.collider);
    auto *sphere = static_cast<SphereCollider *>(B.collider);

    if (!ramp || !sphere)
        return false;

    const float radius = sphere->radius;
    if (radius <= PHYSICS_EPSILON)
        return false;

    const float length = ramp->getLength();
    const float height = ramp->getHeight();

    // Validate ramp has reasonable dimensions
    if (std::abs(length) <= PHYSICS_EPSILON || std::abs(height) <= PHYSICS_EPSILON)
        return false;

    const float halfWidthZ = ramp->getHalfWidthZ();
    const float dirLengthSq = length * length + height * height;
    if (dirLengthSq <= PHYSICS_EPSILON)
        return false;

    // Broad-phase AABB rejection
    const Vec3 localCenter = B.position - A.position;
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

    // Project sphere center onto ramp slope line (clamped to segment)
    const float tUnclamped = (localCenter.x * length + localCenter.y * height) / dirLengthSq;
    const float t = clampValue(tUnclamped, 0.0f, 1.0f);

    const Vec3 closestOnSlope(length * t, height * t, clampValue(localCenter.z, -halfWidthZ, halfWidthZ));
    const Vec3 delta = localCenter - closestOnSlope;
    const float distSq = delta.dot(delta);

    // Check if sphere actually intersects the ramp
    if (distSq > radius * radius)
        return false;

    // Compute distance to surface
    const float dist = std::sqrt(std::max(0.0f, distSq));
    const float penetration = radius - dist;

    if (penetration <= PHYSICS_EPSILON)
        return false;

    // Compute contact normal
    Vec3 normal;
    if (dist <= PHYSICS_EPSILON)
    {
        // Sphere center is very close to slope, use slope normal
        const float mag = std::sqrt(dirLengthSq);
        if (mag <= PHYSICS_EPSILON)
            return false;
        normal = Vec3(-height / mag, length / mag, 0.0f);
    }
    else
    {
        // Normal points from closest point toward sphere center
        normal = delta * (1.0f / dist);
    }

    // Ensure normal points toward sphere (away from ramp)
    {
        const float mag = std::sqrt(dirLengthSq);
        if (mag <= PHYSICS_EPSILON)
            return false;
        const Vec3 slopeNormal(-height / mag, length / mag, 0.0f);
        if (normal.dot(slopeNormal) < 0.0f)
            normal = normal * -1.0f;
    }

    if (!isValidVec3(normal) || normal.length() <= PHYSICS_EPSILON)
        return false;
    normal = normal.normalized();

    // Build single contact
    Contact c;
    c.a = &A;
    c.b = &B;
    c.normal = normal;
    c.penetration = penetration;
    c.contact_point = A.position + closestOnSlope;
    c.restitution = (A.restitution + B.restitution) * 0.5f;
    c.friction_coeff = std::sqrt(A.friction * B.friction);

    if (!isValidVec3(c.normal) || !isValidVec3(c.contact_point) ||
        !isValidFloat(c.penetration) || c.penetration <= PHYSICS_EPSILON)
    {
        return false;
    }

    std::cout << "RampSphere Penetration: " << c.penetration << "\n";
    std::cout << "RampSphere Normal: " << c.normal << "\n";

    // Build manifold
    manifold.a = &A;
    manifold.b = &B;
    manifold.a_id = A.id;
    manifold.b_id = B.id;
    manifold.normal = c.normal;
    manifold.contact_count = 1;
    c.a = manifold.a;
    c.b = manifold.b;
    c.a_id = manifold.a_id;
    c.b_id = manifold.b_id;
    manifold.contacts[0] = c;

    return true;
}

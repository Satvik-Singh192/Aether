#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/ramp_collider.hpp"
#include "core/sphere_collider.hpp"
#include <algorithm>
#include <cmath>

namespace
{
    float clampRange(float value, float a, float b)
    {
        const float lo = std::min(a, b);
        const float hi = std::max(a, b);
        return std::max(lo, std::min(value, hi));
    }

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

    struct CandidateContact
    {
        Vec3 contact_point;
        float penetration;
        Vec3 normal;
    };

    CandidateContact best_candidate = {};
    best_candidate.penetration = -1.0f;

    {

        const float tUnclamped = (localCenter.x * length + localCenter.y * height) / dirLengthSq;
        const float t = clampValue(tUnclamped, 0.0f, 1.0f);

        const Vec3 closestOnSlope(length * t, height * t, clampRange(localCenter.z, -halfWidthZ, halfWidthZ));
        const Vec3 delta = localCenter - closestOnSlope;
        const float distSq = delta.dot(delta);

        if (distSq <= radius * radius)
        {
            const float dist = std::sqrt(std::max(0.0f, distSq));
            const float penetration = radius - dist;

            if (penetration > PHYSICS_EPSILON)
            {
                CandidateContact slope_candidate;
                slope_candidate.contact_point = A.position + closestOnSlope;

                if (dist <= PHYSICS_EPSILON)
                {
                    const float mag = std::sqrt(dirLengthSq);
                    slope_candidate.normal = Vec3(-height / mag, length / mag, 0.0f);
                }
                else
                {
                    slope_candidate.normal = delta * (1.0f / dist);
                }

                {
                    const float mag = std::sqrt(dirLengthSq);
                    const Vec3 slopeNormal(-height / mag, length / mag, 0.0f);
                    if (slope_candidate.normal.dot(slopeNormal) < 0.0f)
                        slope_candidate.normal = slope_candidate.normal * -1.0f;
                }

                slope_candidate.penetration = penetration;

                if (slope_candidate.penetration > best_candidate.penetration)
                    best_candidate = slope_candidate;
            }
        }
    }

    {
        const Vec3 bottomClosest(
            clampRange(localCenter.x, 0.0f, length),
            0.0f,
            clampRange(localCenter.z, -halfWidthZ, halfWidthZ));
        const Vec3 delta = localCenter - bottomClosest;
        const float distSq = delta.dot(delta);

        if (distSq <= radius * radius)
        {
            const float dist = std::sqrt(std::max(0.0f, distSq));
            const float penetration = radius - dist;

            if (penetration > PHYSICS_EPSILON)
            {
                CandidateContact bottom_candidate;
                bottom_candidate.contact_point = A.position + bottomClosest;

                if (dist <= PHYSICS_EPSILON)
                {
                    bottom_candidate.normal = Vec3(0.0f, -1.0f, 0.0f);
                }
                else
                {
                    bottom_candidate.normal = delta * (1.0f / dist);
                    if (bottom_candidate.normal.y > 0.0f)
                        bottom_candidate.normal = bottom_candidate.normal * -1.0f;
                }

                bottom_candidate.penetration = penetration;

                if (bottom_candidate.penetration > best_candidate.penetration)
                    best_candidate = bottom_candidate;
            }
        }
    }

    {
        const Vec3 backClosest(
            length,
            clampRange(localCenter.y, 0.0f, height),
            clampRange(localCenter.z, -halfWidthZ, halfWidthZ));
        const Vec3 delta = localCenter - backClosest;
        const float distSq = delta.dot(delta);

        if (distSq <= radius * radius)
        {
            const float dist = std::sqrt(std::max(0.0f, distSq));
            const float penetration = radius - dist;

            if (penetration > PHYSICS_EPSILON)
            {
                CandidateContact back_candidate;
                back_candidate.contact_point = A.position + backClosest;

                if (dist <= PHYSICS_EPSILON)
                {
                    back_candidate.normal = Vec3(1.0f, 0.0f, 0.0f);
                }
                else
                {
                    back_candidate.normal = delta * (1.0f / dist);
                    if (back_candidate.normal.x < 0.0f)
                        back_candidate.normal = back_candidate.normal * -1.0f;
                }

                back_candidate.penetration = penetration;

                if (back_candidate.penetration > best_candidate.penetration)
                    best_candidate = back_candidate;
            }
        }
    }

    if (best_candidate.penetration <= PHYSICS_EPSILON)
        return false;

    // Validate best contact
    if (!isValidVec3(best_candidate.normal) || best_candidate.normal.length() <= PHYSICS_EPSILON)
        return false;
    best_candidate.normal = best_candidate.normal.normalized();

    if (!isValidVec3(best_candidate.contact_point) || !isValidFloat(best_candidate.penetration))
        return false;

    Contact c;
    c.a = &A;
    c.b = &B;
    c.normal = best_candidate.normal;
    c.penetration = best_candidate.penetration;
    c.contact_point = best_candidate.contact_point;
    c.restitution = (A.restitution + B.restitution) * 0.5f;
    c.friction_coeff = std::sqrt(A.friction * B.friction);

    if (!isValidVec3(c.normal) || !isValidVec3(c.contact_point) ||
        !isValidFloat(c.penetration) || c.penetration <= PHYSICS_EPSILON)
    {
        return false;
    }

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

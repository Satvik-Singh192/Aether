#include "collision/collision.hpp"
#include "core/ramp_collider.hpp"
#include <algorithm>
#include <cmath>

namespace
{
    bool overlaps1D(float minA, float maxA, float minB, float maxB)
    {
        return maxA >= minB && maxB >= minA;
    }

    Vec3 buildRampNormal(const RampCollider &ramp)
    {
        const float height = ramp.getHeight();
        const float length = ramp.getLength();
        const float mag = std::sqrt(height * height + length * length);
        if (mag <= PHYSICS_EPSILON)
            return Vec3();

        return Vec3(-height / mag, length / mag, 0.0f);
    }
}

bool buildRampRampContact(Rigidbody &a, Rigidbody &b, Contact &outContact)
{
    auto *rampA = static_cast<RampCollider *>(a.collider);
    auto *rampB = static_cast<RampCollider *>(b.collider);

    if (!rampA || !rampB)
        return false;

    const float lengthA = rampA->getLength();
    const float lengthB = rampB->getLength();
    if (std::abs(lengthA) <= PHYSICS_EPSILON || std::abs(lengthB) <= PHYSICS_EPSILON)
        return false;

    const float heightA = rampA->getHeight();
    const float heightB = rampB->getHeight();

    float aMinX = std::min(a.position.x, a.position.x + lengthA);
    float aMaxX = std::max(a.position.x, a.position.x + lengthA);
    float aMinY = std::min(a.position.y, a.position.y + heightA);
    float aMaxY = std::max(a.position.y, a.position.y + heightA);
    const float aMinZ = a.position.z - rampA->getHalfWidthZ();
    const float aMaxZ = a.position.z + rampA->getHalfWidthZ();

    float bMinX = std::min(b.position.x, b.position.x + lengthB);
    float bMaxX = std::max(b.position.x, b.position.x + lengthB);
    float bMinY = std::min(b.position.y, b.position.y + heightB);
    float bMaxY = std::max(b.position.y, b.position.y + heightB);
    const float bMinZ = b.position.z - rampB->getHalfWidthZ();
    const float bMaxZ = b.position.z + rampB->getHalfWidthZ();

    if (!overlaps1D(aMinX, aMaxX, bMinX, bMaxX) ||
        !overlaps1D(aMinY, aMaxY, bMinY, bMaxY) ||
        !overlaps1D(aMinZ, aMaxZ, bMinZ, bMaxZ))
    {
        return false;
    }

    const Vec3 centerA((aMinX + aMaxX) * 0.5f, (aMinY + aMaxY) * 0.5f, (aMinZ + aMaxZ) * 0.5f);
    const Vec3 centerB((bMinX + bMaxX) * 0.5f, (bMinY + bMaxY) * 0.5f, (bMinZ + bMaxZ) * 0.5f);

    const Vec3 halfA((aMaxX - aMinX) * 0.5f, (aMaxY - aMinY) * 0.5f, (aMaxZ - aMinZ) * 0.5f);
    const Vec3 halfB((bMaxX - bMinX) * 0.5f, (bMaxY - bMinY) * 0.5f, (bMaxZ - bMinZ) * 0.5f);

    const Vec3 normalA = buildRampNormal(*rampA);
    const Vec3 normalB = buildRampNormal(*rampB);
    if (normalA.length() <= PHYSICS_EPSILON || normalB.length() <= PHYSICS_EPSILON)
        return false;

    const float distBToAPlane = (centerB - a.position).dot(normalA);
    const float projBOnA =
        std::abs(normalA.x) * halfB.x +
        std::abs(normalA.y) * halfB.y +
        std::abs(normalA.z) * halfB.z;
    const float penetrationA = projBOnA - distBToAPlane;

    const float distAToBPlane = (centerA - b.position).dot(normalB);
    const float projAOnB =
        std::abs(normalB.x) * halfA.x +
        std::abs(normalB.y) * halfA.y +
        std::abs(normalB.z) * halfA.z;
    const float penetrationB = projAOnB - distAToBPlane;

    if (penetrationA <= 0.0f || penetrationB <= 0.0f)
        return false;

    Vec3 normal = normalA;
    float penetration = penetrationA;
    Vec3 contactPoint = centerB - normalA * distBToAPlane;

    if (penetrationB < penetrationA)
    {
        normal = normalB * -1.0f;
        penetration = penetrationB;
        contactPoint = centerA + normalB * distAToBPlane;
    }

    if ((centerB - centerA).dot(normal) < 0.0f)
        normal = normal * -1.0f;

    outContact = Contact{};
    outContact.a = &a;
    outContact.b = &b;
    outContact.normal = normal;
    outContact.penetration = penetration;
    outContact.contact_point = contactPoint;
    outContact.restitution = (a.restitution + b.restitution) * 0.5f;
    outContact.friction_coeff = std::sqrt(a.friction * b.friction);

    return true;
}

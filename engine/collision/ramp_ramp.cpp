#include "collision/collision.hpp"
#include "core/ramp_collider.hpp"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>

namespace
{
    bool overlaps1D(float minA, float maxA, float minB, float maxB)
    {
        return maxA >= minB && maxB >= minA;
    }

    bool isValidFloat(float v)
    {
        return std::isfinite(v);
    }

    bool isValidVec3(const Vec3 &v)
    {
        return isValidFloat(v.x) && isValidFloat(v.y) && isValidFloat(v.z);
    }

    float computeAabbOverlapOnAxis(const Vec3 &axisN,
                                   const Vec3 &centerA,
                                   const Vec3 &halfA,
                                   const Vec3 &centerB,
                                   const Vec3 &halfB)
    {
        const float distance = std::abs((centerB - centerA).dot(axisN));
        const float radiusA =
            std::abs(axisN.x) * halfA.x +
            std::abs(axisN.y) * halfA.y +
            std::abs(axisN.z) * halfA.z;
        const float radiusB =
            std::abs(axisN.x) * halfB.x +
            std::abs(axisN.y) * halfB.y +
            std::abs(axisN.z) * halfB.z;
        return radiusA + radiusB - distance;
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

    float minOverlap = FLT_MAX;
    Vec3 bestAxis;
    const Vec3 axes[] = {
        normalA,
        normalB,
        Vec3(1.0f, 0.0f, 0.0f),
        Vec3(0.0f, 1.0f, 0.0f),
        Vec3(0.0f, 0.0f, 1.0f)};

    for (const Vec3 &rawAxis : axes)
    {
        const float axisLen = rawAxis.length();
        if (axisLen <= PHYSICS_EPSILON)
            return false;

        const Vec3 axis = rawAxis * (1.0f / axisLen);
        const float overlap = computeAabbOverlapOnAxis(axis, centerA, halfA, centerB, halfB);

        if (!isValidFloat(overlap) || overlap <= PHYSICS_EPSILON)
            return false;

        if (overlap < minOverlap)
        {
            minOverlap = overlap;
            bestAxis = axis;
        }
    }

    if (minOverlap == FLT_MAX)
        return false;

    if (!isValidVec3(bestAxis) || bestAxis.length() <= PHYSICS_EPSILON)
        return false;

    Vec3 normal = bestAxis.normalized();
    if (!isValidVec3(normal) || normal.length() <= PHYSICS_EPSILON)
        return false;

    if (normal.dot(b.position - a.position) < 0.0f)
        normal = normal * -1.0f;

    const float penetration = minOverlap;
    if (!isValidFloat(penetration) || penetration <= PHYSICS_EPSILON)
        return false;

    const Vec3 contactPoint = (centerA + centerB) * 0.5f;

    outContact = Contact{};
    outContact.a = &a;
    outContact.b = &b;
    outContact.normal = normal;
    outContact.penetration = penetration;
    outContact.contact_point = contactPoint;
    outContact.restitution = (a.restitution + b.restitution) * 0.5f;
    outContact.friction_coeff = std::sqrt(a.friction * b.friction);

    if (!isValidVec3(outContact.normal) || !isValidVec3(outContact.contact_point) ||
        !isValidFloat(outContact.penetration) || outContact.penetration <= PHYSICS_EPSILON)
    {
        return false;
    }

    std::cout << "RampRamp Contact Penetration: " << outContact.penetration << "\n";
    std::cout << "RampRamp Contact Normal: " << outContact.normal << "\n";

    return true;
}

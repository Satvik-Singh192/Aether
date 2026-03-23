#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/ramp_collider.hpp"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{
    bool isValidFloat(float v)
    {
        return std::isfinite(v);
    }

    bool isValidVec3(const Vec3 &v)
    {
        return isValidFloat(v.x) && isValidFloat(v.y) && isValidFloat(v.z);
    }

    bool overlaps1D(float minA, float maxA, float minB, float maxB)
    {
        return maxA >= minB && maxB >= minA;
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

    Vec3 buildSlopeNormal(const RampCollider &ramp)
    {
        const float length = ramp.getLength();
        const float height = ramp.getHeight();
        const float mag = std::sqrt(length * length + height * height);
        if (mag <= PHYSICS_EPSILON)
            return Vec3();
        return Vec3(-height / mag, length / mag, 0.0f);
    }

    bool isPointOnRampSlope(const Rigidbody &rampBody, const RampCollider &ramp, const Vec3 &worldPoint)
    {
        const Vec3 local = worldPoint - rampBody.position;
        const float length = ramp.getLength();
        const float height = ramp.getHeight();
        const float dirLenSq = length * length + height * height;

        if (dirLenSq <= PHYSICS_EPSILON)
            return false;

        const float t = (local.x * length + local.y * height) / dirLenSq;
        if (t < -0.05f || t > 1.05f)
            return false;

        return std::abs(local.z) <= ramp.getHalfWidthZ() + 0.05f;
    }

    void pushUniqueContact(std::vector<Contact> &contacts, const Contact &c)
    {
        constexpr float kMinDistSq = 0.0004f;
        for (const Contact &existing : contacts)
        {
            const Vec3 d = existing.contact_point - c.contact_point;
            if (d.dot(d) <= kMinDistSq)
                return;
        }
        contacts.push_back(c);
    }
}

// A and B are ramps.
bool buildRampRampManifold(Rigidbody &A, Rigidbody &B, ContactManifold &manifold)
{
    auto *rampA = static_cast<RampCollider *>(A.collider);
    auto *rampB = static_cast<RampCollider *>(B.collider);

    if (!rampA || !rampB)
        return false;

    const float lengthA = rampA->getLength();
    const float lengthB = rampB->getLength();
    const float heightA = rampA->getHeight();
    const float heightB = rampB->getHeight();

    // Validate both ramps have reasonable dimensions
    if (std::abs(lengthA) <= PHYSICS_EPSILON || std::abs(lengthB) <= PHYSICS_EPSILON ||
        std::abs(heightA) <= PHYSICS_EPSILON || std::abs(heightB) <= PHYSICS_EPSILON)
        return false;

    float aMinX = std::min(A.position.x, A.position.x + lengthA);
    float aMaxX = std::max(A.position.x, A.position.x + lengthA);
    float aMinY = std::min(A.position.y, A.position.y + heightA);
    float aMaxY = std::max(A.position.y, A.position.y + heightA);
    const float aMinZ = A.position.z - rampA->getHalfWidthZ();
    const float aMaxZ = A.position.z + rampA->getHalfWidthZ();

    float bMinX = std::min(B.position.x, B.position.x + lengthB);
    float bMaxX = std::max(B.position.x, B.position.x + lengthB);
    float bMinY = std::min(B.position.y, B.position.y + heightB);
    float bMaxY = std::max(B.position.y, B.position.y + heightB);
    const float bMinZ = B.position.z - rampB->getHalfWidthZ();
    const float bMaxZ = B.position.z + rampB->getHalfWidthZ();

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

    const Vec3 normalA = buildSlopeNormal(*rampA);
    const Vec3 normalB = buildSlopeNormal(*rampB);
    if (normalA.length() <= PHYSICS_EPSILON || normalB.length() <= PHYSICS_EPSILON)
        return false;

    float minOverlap = FLT_MAX;
    Vec3 bestAxis;

    const std::vector<Vec3> axes = {
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

    Vec3 normal = bestAxis;
    if (!isValidVec3(normal) || normal.length() <= PHYSICS_EPSILON)
        return false;

    if (normal.dot(B.position - A.position) < 0.0f)
        normal = normal * -1.0f;

    normal = normal.normalized();
    if (!isValidVec3(normal) || normal.length() <= PHYSICS_EPSILON)
        return false;

    const float penetration = minOverlap;
    if (!isValidFloat(penetration) || penetration <= PHYSICS_EPSILON)
        return false;

    const Vec3 contactPoint = (centerA + centerB) * 0.5f;

    // Create validated contact
    Contact baseContact;
    baseContact.a = &A;
    baseContact.b = &B;
    baseContact.normal = normal;
    baseContact.penetration = penetration;
    baseContact.contact_point = contactPoint;
    baseContact.restitution = (A.restitution + B.restitution) * 0.5f;
    baseContact.friction_coeff = std::sqrt(A.friction * B.friction);

    if (!isValidVec3(baseContact.normal) || !isValidVec3(baseContact.contact_point) ||
        !isValidFloat(baseContact.penetration) || baseContact.penetration <= 0.0f)
    {
        return false;
    }

    std::cout << "RampRamp Penetration: " << baseContact.penetration << "\n";
    std::cout << "RampRamp Normal: " << baseContact.normal << "\n";

    std::vector<Contact> candidates;
    pushUniqueContact(candidates, baseContact);

    {
        const float z = rampB->getHalfWidthZ();
        const Vec3 p0 = B.position;
        const Vec3 p1 = B.position + Vec3(rampB->getLength(), rampB->getHeight(), 0.0f);

        const Vec3 bCorners[] = {
            p0 + Vec3(0.0f, 0.0f, z),
            p0 + Vec3(0.0f, 0.0f, -z),
            p1 + Vec3(0.0f, 0.0f, z),
            p1 + Vec3(0.0f, 0.0f, -z)};

        for (const Vec3 &corner : bCorners)
        {
            const float signedDistance = (corner - A.position).dot(normalA);
            if (signedDistance > 0.0f)
                continue;

            const Vec3 projected = corner - normalA * signedDistance;
            if (!isPointOnRampSlope(A, *rampA, projected))
                continue;

            Contact c = baseContact;
            c.contact_point = projected;
            pushUniqueContact(candidates, c);
        }
    }

    {
        const float z = rampA->getHalfWidthZ();
        const Vec3 p0 = A.position;
        const Vec3 p1 = A.position + Vec3(rampA->getLength(), rampA->getHeight(), 0.0f);

        const Vec3 aCorners[] = {
            p0 + Vec3(0.0f, 0.0f, z),
            p0 + Vec3(0.0f, 0.0f, -z),
            p1 + Vec3(0.0f, 0.0f, z),
            p1 + Vec3(0.0f, 0.0f, -z)};

        for (const Vec3 &corner : aCorners)
        {
            const float signedDistance = (corner - B.position).dot(normalB);
            if (signedDistance > 0.0f)
                continue;

            const Vec3 projected = corner - normalB * signedDistance;
            if (!isPointOnRampSlope(B, *rampB, projected))
                continue;

            Contact c = baseContact;
            c.contact_point = projected;
            pushUniqueContact(candidates, c);
        }
    }

    if (candidates.empty())
        return false;

    manifold.a = &A;
    manifold.b = &B;
    manifold.a_id = A.id;
    manifold.b_id = B.id;
    manifold.normal = baseContact.normal;

    manifold.contact_count = 0;
    manifold.contacts[manifold.contact_count++] = candidates[0];

    if (candidates.size() > 1)
    {
        float bestDistSq = -1.0f;
        int bestIdx = -1;
        for (int i = 1; i < static_cast<int>(candidates.size()); ++i)
        {
            const Vec3 d = candidates[i].contact_point - manifold.contacts[0].contact_point;
            const float distSq = d.dot(d);
            if (distSq > bestDistSq)
            {
                bestDistSq = distSq;
                bestIdx = i;
            }
        }

        if (bestIdx >= 0 && bestDistSq > 0.0004f)
        {
            manifold.contacts[manifold.contact_count++] = candidates[bestIdx];
        }
    }

    for (int i = 0; i < manifold.contact_count; ++i)
    {
        manifold.contacts[i].a = manifold.a;
        manifold.contacts[i].b = manifold.b;
        manifold.contacts[i].a_id = manifold.a_id;
        manifold.contacts[i].b_id = manifold.b_id;
        manifold.contacts[i].normal = manifold.normal;
        manifold.contacts[i].penetration = baseContact.penetration;
    }

    return manifold.contact_count > 0;
}

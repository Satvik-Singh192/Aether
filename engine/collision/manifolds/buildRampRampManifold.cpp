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

    std::vector<Vec3> getRampVertices(const Rigidbody &body, const RampCollider &ramp)
    {
        std::vector<Vec3> verts;
        const float L = ramp.getLength();
        const float H = ramp.getHeight();
        const float W = ramp.getHalfWidthZ();
        const Vec3 p = body.position;

        verts.push_back(p + Vec3(0.0f, 0.0f, -W));
        verts.push_back(p + Vec3(L, 0.0f, -W));
        verts.push_back(p + Vec3(0.0f, 0.0f, W));
        verts.push_back(p + Vec3(L, 0.0f, W));

        verts.push_back(p + Vec3(L, H, -W));
        verts.push_back(p + Vec3(L, H, W));

        return verts;
    }

    void projectVertices(const std::vector<Vec3> &verts, const Vec3 &axis, float &minProj, float &maxProj)
    {
        minProj = FLT_MAX;
        maxProj = -FLT_MAX;

        for (const Vec3 &v : verts)
        {
            const float proj = v.dot(axis);
            minProj = std::min(minProj, proj);
            maxProj = std::max(maxProj, proj);
        }
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

    std::vector<Vec3> axes;
    axes.push_back(normalA);
    axes.push_back(normalB);
    axes.push_back(Vec3(1.0f, 0.0f, 0.0f));
    axes.push_back(Vec3(0.0f, 1.0f, 0.0f));
    axes.push_back(Vec3(0.0f, 0.0f, 1.0f));

    const Vec3 edgesA[] = {Vec3(lengthA, 0.0f, 0.0f), Vec3(0.0f, heightA, 0.0f), Vec3(0.0f, 0.0f, 1.0f)};
    const Vec3 edgesB[] = {Vec3(lengthB, 0.0f, 0.0f), Vec3(0.0f, heightB, 0.0f), Vec3(0.0f, 0.0f, 1.0f)};

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            const Vec3 crossAxis = edgesA[i].cross(edgesB[j]);
            const float crossLen = crossAxis.length();
            if (crossLen > PHYSICS_EPSILON)
                axes.push_back(crossAxis.normalized());
        }
    }

    std::cout << "Axes count: " << axes.size() << "\n";

    const std::vector<Vec3> vertsA = getRampVertices(A, *rampA);
    const std::vector<Vec3> vertsB = getRampVertices(B, *rampB);

    for (const Vec3 &rawAxis : axes)
    {
        const float axisLen = rawAxis.length();
        if (axisLen <= PHYSICS_EPSILON)
            return false;

        const Vec3 axis = rawAxis * (1.0f / axisLen);
        float minA, maxA, minB, maxB;
        projectVertices(vertsA, axis, minA, maxA);
        projectVertices(vertsB, axis, minB, maxB);

        const float overlap = std::min(maxA, maxB) - std::max(minA, minB);

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

    float secondMinOverlap = FLT_MAX;
    for (const Vec3 &rawAxis : axes)
    {
        const float axisLen = rawAxis.length();
        if (axisLen <= PHYSICS_EPSILON)
            continue;
        const Vec3 axis = rawAxis * (1.0f / axisLen);
        float minA, maxA, minB, maxB;
        projectVertices(vertsA, axis, minA, maxA);
        projectVertices(vertsB, axis, minB, maxB);
        const float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (isValidFloat(overlap) && overlap > PHYSICS_EPSILON && overlap < secondMinOverlap &&
            !(axis.dot(bestAxis) > 0.99f))
        {
            secondMinOverlap = overlap;
        }
    }

    if (isValidFloat(secondMinOverlap) && (secondMinOverlap - minOverlap) / minOverlap < 0.05f)
    {
        if ((centerB - centerA).length() < 0.01f)
        {
            if (A.id > B.id)
                normal = normal * -1.0f;
        }
    }

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

    const Vec3 contactPoint = (A.position + B.position) * 0.5f;

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

    manifold.a = &A;
    manifold.b = &B;
    manifold.a_id = A.id;
    manifold.b_id = B.id;
    manifold.normal = normal;
    manifold.contact_count = 1;
    manifold.contacts[0] = baseContact;

    return true;
}

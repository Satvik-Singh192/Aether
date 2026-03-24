#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/box_collider.hpp"
#include "core/ramp_collider.hpp"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{
    float clampValue(float v, float lo, float hi)
    {
        return std::max(lo, std::min(v, hi));
    }

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

    void projectVerts(const std::vector<Vec3> &verts, const Vec3 &axis, float &min, float &max)
    {
        min = max = verts[0].dot(axis);
        for (int i = 1; i < verts.size(); i++)
        {
            float p = verts[i].dot(axis);
            min = std::min(min, p);
            max = std::max(max, p);
        }
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

// A is ramp, B is box.
bool buildRampBoxManifold(Rigidbody &A, Rigidbody &B, ContactManifold &manifold)
{
    auto *ramp = static_cast<RampCollider *>(A.collider);
    auto *box = static_cast<BoxCollider *>(B.collider);

    if (!ramp || !box)
        return false;

    const float length = ramp->getLength();
    const float height = ramp->getHeight();

    // Validate ramp has reasonable dimensions
    if (std::abs(length) <= PHYSICS_EPSILON || std::abs(height) <= PHYSICS_EPSILON)
        return false;

    const Vec3 half = box->getHalfExtents();
    const Vec3 rampHalf(
        std::abs(length) * 0.5f,
        std::abs(height) * 0.5f,
        ramp->getHalfWidthZ());
    const Vec3 boxMin = B.position - half;
    const Vec3 boxMax = B.position + half;

    float rampMinX = A.position.x;
    float rampMaxX = A.position.x + length;
    if (rampMinX > rampMaxX)
        std::swap(rampMinX, rampMaxX);

    float rampMinY = A.position.y;
    float rampMaxY = A.position.y + height;
    if (rampMinY > rampMaxY)
        std::swap(rampMinY, rampMaxY);

    float rampMinZ = A.position.z - ramp->getHalfWidthZ();
    float rampMaxZ = A.position.z + ramp->getHalfWidthZ();

    // Broad phase AABB test
    if (!overlaps1D(boxMin.x, boxMax.x, rampMinX, rampMaxX) ||
        !overlaps1D(boxMin.y, boxMax.y, rampMinY, rampMaxY) ||
        !overlaps1D(boxMin.z, boxMax.z, rampMinZ, rampMaxZ))
    {
        return false;
    }

    const float mag = std::sqrt(height * height + length * length);
    if (mag <= PHYSICS_EPSILON)
        return false;

    Vec3 rampNormal(-height / mag, length / mag, 0.0f);
    if (!isValidVec3(rampNormal) || rampNormal.length() <= PHYSICS_EPSILON)
        return false;

    // Build ramp vertices (6 total: base quad + top slope edge)
    std::vector<Vec3> rampVerts;
    rampVerts.push_back(A.position + Vec3(0, 0, -ramp->getHalfWidthZ()));
    rampVerts.push_back(A.position + Vec3(length, 0, -ramp->getHalfWidthZ()));
    rampVerts.push_back(A.position + Vec3(0, 0, ramp->getHalfWidthZ()));
    rampVerts.push_back(A.position + Vec3(length, 0, ramp->getHalfWidthZ()));
    rampVerts.push_back(A.position + Vec3(length, height, -ramp->getHalfWidthZ()));
    rampVerts.push_back(A.position + Vec3(length, height, ramp->getHalfWidthZ()));

    // Build box vertices (8 total)
    std::vector<Vec3> boxVerts;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                boxVerts.push_back(B.position + Vec3(sx * half.x, sy * half.y, sz * half.z));

    // Build SAT axes: ramp normal + cardinals + edge cross products
    std::vector<Vec3> axes;
    axes.push_back(rampNormal);
    axes.push_back(Vec3(1.0f, 0.0f, 0.0f));
    axes.push_back(Vec3(0.0f, 1.0f, 0.0f));
    axes.push_back(Vec3(0.0f, 0.0f, 1.0f));

    // Add edge cross products: ramp slope edge × box edges
    Vec3 rampEdge(length, height, 0.0f);
    Vec3 boxEdges[3] = {
        Vec3(1.0f, 0.0f, 0.0f),
        Vec3(0.0f, 1.0f, 0.0f),
        Vec3(0.0f, 0.0f, 1.0f)};

    for (int i = 0; i < 3; i++)
    {
        Vec3 axis = rampEdge.cross(boxEdges[i]);
        if (axis.length() > 1e-6f)
            axes.push_back(axis.normalized());
    }

    // SAT: project all vertices onto each axis
    float minOverlap = FLT_MAX;
    Vec3 bestAxis;

    for (const Vec3 &rawAxis : axes)
    {
        const float axisLen = rawAxis.length();
        if (axisLen <= PHYSICS_EPSILON)
            continue;

        const Vec3 axis = rawAxis * (1.0f / axisLen);

        float minA, maxA, minB, maxB;
        projectVerts(rampVerts, axis, minA, maxA);
        projectVerts(boxVerts, axis, minB, maxB);

        float overlap = std::min(maxA, maxB) - std::max(minA, minB);

        if (overlap <= PHYSICS_EPSILON)
            return false;

        if (overlap < minOverlap)
        {
            minOverlap = overlap;
            bestAxis = axis;
        }
    }

    if (minOverlap == FLT_MAX || !isValidVec3(bestAxis) || bestAxis.length() <= PHYSICS_EPSILON)
        return false;

    Vec3 normal = bestAxis;
    if (normal.dot(B.position - A.position) < 0.0f)
        normal = normal * -1.0f;

    const float penetration = minOverlap;
    if (!isValidFloat(penetration) || penetration <= PHYSICS_EPSILON)
        return false;

    // Create base contact
    Contact baseContact;
    baseContact.a = &A;
    baseContact.b = &B;
    baseContact.normal = normal;
    baseContact.penetration = penetration;
    baseContact.contact_point = Vec3(
        clampValue(B.position.x, rampMinX, rampMaxX),
        clampValue(B.position.y, rampMinY, rampMaxY),
        clampValue(B.position.z, rampMinZ, rampMaxZ));
    baseContact.restitution = (A.restitution + B.restitution) * 0.5f;
    baseContact.friction_coeff = std::sqrt(A.friction * B.friction);

    if (!isValidVec3(baseContact.normal) || !isValidVec3(baseContact.contact_point) ||
        !isValidFloat(baseContact.penetration) || baseContact.penetration <= PHYSICS_EPSILON)
    {
        return false;
    }

    std::vector<Contact> candidates;
    pushUniqueContact(candidates, baseContact);

    for (int sx = -1; sx <= 1; sx += 2)
    {
        for (int sy = -1; sy <= 1; sy += 2)
        {
            for (int sz = -1; sz <= 1; sz += 2)
            {
                const Vec3 corner = B.position + Vec3(sx * half.x, sy * half.y, sz * half.z);
                const float cornerDist = (corner - A.position).dot(normal);

                // only project if corner is on penetrating side
                if (cornerDist >= 0.0f)
                    continue;

                const Vec3 projected = corner - normal * cornerDist;

                // validate projected point is on ramp slope surface
                if (!isPointOnRampSlope(A, *ramp, projected))
                    continue;

                Contact c = baseContact;
                c.contact_point = projected;
                c.penetration = std::min(-cornerDist, penetration);
                if (isValidVec3(c.contact_point) && isValidFloat(c.penetration) && c.penetration > PHYSICS_EPSILON)
                    pushUniqueContact(candidates, c);
            }
        }
    }

    if (candidates.empty())
        return false;

    // build manifold from candidates
    manifold.a = &A;
    manifold.b = &B;
    manifold.a_id = A.id;
    manifold.b_id = B.id;
    manifold.normal = normal;
    if (!isValidVec3(manifold.normal) || manifold.normal.length() <= PHYSICS_EPSILON)
        return false;

    std::cout << "RampBox Penetration: " << baseContact.penetration << "\n";
    std::cout << "RampBox Normal: " << manifold.normal << "\n";
    manifold.contact_count = 0;

    // keep first and best separated contact
    manifold.contacts[manifold.contact_count++] = candidates[0];

    if (candidates.size() > 1)
    {
        float bestDistSq = -1.0f;
        int bestIdx = -1;
        for (size_t i = 1; i < candidates.size(); ++i)
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
    }

    return manifold.contact_count > 0;
}

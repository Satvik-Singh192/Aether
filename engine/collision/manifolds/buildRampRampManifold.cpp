#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/ramp_collider.hpp"
#include <algorithm>
#include <cfloat>
#include <cmath>
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

    std::vector<Vec3> getRampVertices(const Rigidbody &body, const RampCollider &ramp)
    {
        const float length = ramp.getLength();
        const float height = ramp.getHeight();
        const float halfWidthZ = ramp.getHalfWidthZ();
        const Vec3 comLocal = ramp.getLocalCenterOfMassOffset();

        const Mat3 rot = body.orientation.toMat3();
        std::vector<Vec3> verts;
        verts.reserve(6);
        verts.push_back(body.position + rot * (Vec3(0.0f, 0.0f, -halfWidthZ) - comLocal));
        verts.push_back(body.position + rot * (Vec3(0.0f, 0.0f, halfWidthZ) - comLocal));
        verts.push_back(body.position + rot * (Vec3(length, 0.0f, -halfWidthZ) - comLocal));
        verts.push_back(body.position + rot * (Vec3(length, 0.0f, halfWidthZ) - comLocal));
        verts.push_back(body.position + rot * (Vec3(length, height, -halfWidthZ) - comLocal));
        verts.push_back(body.position + rot * (Vec3(length, height, halfWidthZ) - comLocal));
        return verts;
    }

    bool pointInTriangle2D(float px, float py, float ax, float ay, float bx, float by, float cx, float cy)
    {
        const auto edgeSign = [](float x1, float y1, float x2, float y2, float x3, float y3)
        {
            return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
        };

        const float d1 = edgeSign(px, py, ax, ay, bx, by);
        const float d2 = edgeSign(px, py, bx, by, cx, cy);
        const float d3 = edgeSign(px, py, cx, cy, ax, ay);

        const bool hasNeg = (d1 < -PHYSICS_EPSILON) || (d2 < -PHYSICS_EPSILON) || (d3 < -PHYSICS_EPSILON);
        const bool hasPos = (d1 > PHYSICS_EPSILON) || (d2 > PHYSICS_EPSILON) || (d3 > PHYSICS_EPSILON);
        return !(hasNeg && hasPos);
    }

    bool pointInsideRamp(const Rigidbody &body, const RampCollider &ramp, const Vec3 &worldPoint)
    {
        const Mat3 rotT = body.orientation.toMat3().transpose();
        const Vec3 local = ramp.getLocalCenterOfMassOffset() + (rotT * (worldPoint - body.position));

        if (std::abs(local.z) > ramp.getHalfWidthZ() + 0.02f)
            return false;

        const float length = ramp.getLength();
        const float height = ramp.getHeight();
        return pointInTriangle2D(local.x, local.y,
                                 0.0f, 0.0f,
                                 length, 0.0f,
                                 length, height);
    }

    Vec3 supportPointOnVerts(const std::vector<Vec3> &verts, const Vec3 &dir)
    {
        Vec3 best = verts[0];
        float bestProj = best.dot(dir);
        for (size_t i = 1; i < verts.size(); ++i)
        {
            const float p = verts[i].dot(dir);
            if (p > bestProj)
            {
                bestProj = p;
                best = verts[i];
            }
        }
        return best;
    }

    void pushUniqueContact(std::vector<Contact> &contacts, const Contact &candidate)
    {
        constexpr float kMinDistSq = 0.0004f;
        for (const Contact &existing : contacts)
        {
            const Vec3 d = existing.contact_point - candidate.contact_point;
            if (d.dot(d) <= kMinDistSq)
                return;
        }
        contacts.push_back(candidate);
    }

    void addUniqueAxis(std::vector<Vec3> &axes, const Vec3 &rawAxis)
    {
        const float len = rawAxis.length();
        if (len <= PHYSICS_EPSILON)
            return;

        Vec3 axis = rawAxis * (1.0f / len);
        for (const Vec3 &existing : axes)
        {
            if (std::abs(existing.dot(axis)) > 0.999f)
                return;
        }
        axes.push_back(axis);
    }

    void getAabb(const std::vector<Vec3> &verts, float &minX, float &maxX, float &minY, float &maxY, float &minZ, float &maxZ)
    {
        minX = minY = minZ = FLT_MAX;
        maxX = maxY = maxZ = -FLT_MAX;
        for (const Vec3 &v : verts)
        {
            minX = std::min(minX, v.x);
            maxX = std::max(maxX, v.x);
            minY = std::min(minY, v.y);
            maxY = std::max(maxY, v.y);
            minZ = std::min(minZ, v.z);
            maxZ = std::max(maxZ, v.z);
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
    const float heightA = rampA->getHeight();
    const float widthA = rampA->getHalfWidthZ();
    const float lengthB = rampB->getLength();
    const float heightB = rampB->getHeight();
    const float widthB = rampB->getHalfWidthZ();

    if (std::abs(lengthA) <= PHYSICS_EPSILON || std::abs(heightA) <= PHYSICS_EPSILON || widthA <= PHYSICS_EPSILON ||
        std::abs(lengthB) <= PHYSICS_EPSILON || std::abs(heightB) <= PHYSICS_EPSILON || widthB <= PHYSICS_EPSILON)
    {
        return false;
    }

    const std::vector<Vec3> vertsA = getRampVertices(A, *rampA);
    const std::vector<Vec3> vertsB = getRampVertices(B, *rampB);

    float aMinX, aMaxX, aMinY, aMaxY, aMinZ, aMaxZ;
    float bMinX, bMaxX, bMinY, bMaxY, bMinZ, bMaxZ;
    getAabb(vertsA, aMinX, aMaxX, aMinY, aMaxY, aMinZ, aMaxZ);
    getAabb(vertsB, bMinX, bMaxX, bMinY, bMaxY, bMinZ, bMaxZ);

    if (!overlaps1D(aMinX, aMaxX, bMinX, bMaxX) ||
        !overlaps1D(aMinY, aMaxY, bMinY, bMaxY) ||
        !overlaps1D(aMinZ, aMaxZ, bMinZ, bMaxZ))
    {
        return false;
    }

    const Mat3 rotA = A.orientation.toMat3();
    const Mat3 rotB = B.orientation.toMat3();

    const float slopeLenA = std::sqrt(lengthA * lengthA + heightA * heightA);
    const float slopeLenB = std::sqrt(lengthB * lengthB + heightB * heightB);
    if (slopeLenA <= PHYSICS_EPSILON || slopeLenB <= PHYSICS_EPSILON)
        return false;

    std::vector<Vec3> axes;
    axes.reserve(24);

    addUniqueAxis(axes, rotA * Vec3(-heightA / slopeLenA, lengthA / slopeLenA, 0.0f));
    addUniqueAxis(axes, rotB * Vec3(-heightB / slopeLenB, lengthB / slopeLenB, 0.0f));
    addUniqueAxis(axes, rotA * Vec3(0.0f, -1.0f, 0.0f));
    addUniqueAxis(axes, rotB * Vec3(0.0f, -1.0f, 0.0f));
    addUniqueAxis(axes, rotA * Vec3((lengthA >= 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f));
    addUniqueAxis(axes, rotB * Vec3((lengthB >= 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f));
    addUniqueAxis(axes, rotA * Vec3(0.0f, 0.0f, 1.0f));
    addUniqueAxis(axes, rotB * Vec3(0.0f, 0.0f, 1.0f));

    const Vec3 edgeDirsA[] = {
        rotA * Vec3(1.0f, 0.0f, 0.0f),
        rotA * Vec3(0.0f, 1.0f, 0.0f),
        rotA * Vec3(0.0f, 0.0f, 1.0f),
        rotA * Vec3(lengthA, heightA, 0.0f)};
    const Vec3 edgeDirsB[] = {
        rotB * Vec3(1.0f, 0.0f, 0.0f),
        rotB * Vec3(0.0f, 1.0f, 0.0f),
        rotB * Vec3(0.0f, 0.0f, 1.0f),
        rotB * Vec3(lengthB, heightB, 0.0f)};

    for (const Vec3 &ea : edgeDirsA)
    {
        for (const Vec3 &eb : edgeDirsB)
            addUniqueAxis(axes, ea.cross(eb));
    }

    if (axes.empty())
        return false;

    float minOverlap = FLT_MAX;
    Vec3 bestAxis;

    for (const Vec3 &axis : axes)
    {
        float minA = 0.0f;
        float maxA = 0.0f;
        float minB = 0.0f;
        float maxB = 0.0f;
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

    if (!isValidVec3(bestAxis) || bestAxis.length() <= PHYSICS_EPSILON)
        return false;

    Vec3 normal = bestAxis.normalized();
    if ((B.position - A.position).dot(normal) < 0.0f)
        normal = normal * -1.0f;

    if (!isValidVec3(normal) || minOverlap <= PHYSICS_EPSILON || !isValidFloat(minOverlap))
        return false;

    Contact base;
    base.a = &A;
    base.b = &B;
    base.a_id = A.id;
    base.b_id = B.id;
    base.normal = normal;
    base.penetration = minOverlap;
    base.restitution = (A.restitution + B.restitution) * 0.5f;
    base.friction_coeff = std::sqrt(A.friction * B.friction);

    const Vec3 supportA = supportPointOnVerts(vertsA, normal);
    const Vec3 supportB = supportPointOnVerts(vertsB, normal * -1.0f);
    base.contact_point = (supportA + supportB) * 0.5f;

    std::vector<Contact> candidates;
    candidates.reserve(8);
    if (isValidVec3(base.contact_point))
        pushUniqueContact(candidates, base);

    for (const Vec3 &v : vertsA)
    {
        if (!pointInsideRamp(B, *rampB, v))
            continue;
        Contact c = base;
        c.contact_point = v;
        pushUniqueContact(candidates, c);
    }

    for (const Vec3 &v : vertsB)
    {
        if (!pointInsideRamp(A, *rampA, v))
            continue;
        Contact c = base;
        c.contact_point = v;
        pushUniqueContact(candidates, c);
    }

    if (candidates.empty())
        return false;

    manifold.a = &A;
    manifold.b = &B;
    manifold.a_id = A.id;
    manifold.b_id = B.id;
    manifold.normal = normal;
    manifold.contact_count = 0;

    manifold.contacts[manifold.contact_count++] = candidates[0];

    if (candidates.size() > 1)
    {
        float bestDistSq = -1.0f;
        int bestIdx = -1;
        for (size_t i = 1; i < candidates.size(); ++i)
        {
            const Vec3 d = candidates[i].contact_point - candidates[0].contact_point;
            const float distSq = d.dot(d);
            if (distSq > bestDistSq)
            {
                bestDistSq = distSq;
                bestIdx = static_cast<int>(i);
            }
        }

        if (bestIdx >= 0 && bestDistSq > 0.0004f)
            manifold.contacts[manifold.contact_count++] = candidates[bestIdx];
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

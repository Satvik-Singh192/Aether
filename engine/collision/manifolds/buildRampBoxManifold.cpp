#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/box_collider.hpp"
#include "core/ramp_collider.hpp"
#include "collision/obb.hpp"
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

    void projectVerts(const std::vector<Vec3> &verts, const Vec3 &axis, float &minV, float &maxV)
    {
        minV = maxV = verts[0].dot(axis);
        for (size_t i = 1; i < verts.size(); ++i)
        {
            const float p = verts[i].dot(axis);
            minV = std::min(minV, p);
            maxV = std::max(maxV, p);
        }
    }

    std::vector<Vec3> getOBBCorners(const OBB &obb)
    {
        std::vector<Vec3> corners;
        corners.reserve(8);

        for (int sx = -1; sx <= 1; sx += 2)
        {
            for (int sy = -1; sy <= 1; sy += 2)
            {
                for (int sz = -1; sz <= 1; sz += 2)
                {
                    Vec3 corner = obb.center;
                    corner += obb.axis[0] * (obb.halfsize.x * static_cast<float>(sx));
                    corner += obb.axis[1] * (obb.halfsize.y * static_cast<float>(sy));
                    corner += obb.axis[2] * (obb.halfsize.z * static_cast<float>(sz));
                    corners.push_back(corner);
                }
            }
        }

        return corners;
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

    bool pointInsideRamp(const Rigidbody &rampBody, const RampCollider &ramp, const Vec3 &worldPoint)
    {
        const Mat3 rot = rampBody.orientation.toMat3();
        const Mat3 rotT = rot.transpose();
        const Vec3 comLocal = ramp.getLocalCenterOfMassOffset();
        const Vec3 local = comLocal + (rotT * (worldPoint - rampBody.position));
        if (std::abs(local.z) > ramp.getHalfWidthZ() + 0.01f)
            return false;

        const float length = ramp.getLength();
        const float height = ramp.getHeight();
        return pointInTriangle2D(local.x, local.y,
                                 0.0f, 0.0f,
                                 length, 0.0f,
                                 length, height);
    }

    Vec3 supportPoint(const OBB &obb, const Vec3 &dir)
    {
        Vec3 p = obb.center;
        p += obb.axis[0] * (obb.axis[0].dot(dir) >= 0.0f ? obb.halfsize.x : -obb.halfsize.x);
        p += obb.axis[1] * (obb.axis[1].dot(dir) >= 0.0f ? obb.halfsize.y : -obb.halfsize.y);
        p += obb.axis[2] * (obb.axis[2].dot(dir) >= 0.0f ? obb.halfsize.z : -obb.halfsize.z);
        return p;
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
}

// A is ramp, B is box.
bool buildRampBoxManifold(Rigidbody &A, Rigidbody &B, ContactManifold &m)
{
    auto *ramp = static_cast<RampCollider *>(A.collider);
    auto *box = static_cast<BoxCollider *>(B.collider);
    if (!ramp || !box)
        return false;

    const float length = ramp->getLength();
    const float height = ramp->getHeight();
    const float halfWidthZ = ramp->getHalfWidthZ();

    if (std::abs(length) <= PHYSICS_EPSILON || std::abs(height) <= PHYSICS_EPSILON || halfWidthZ <= PHYSICS_EPSILON)
        return false;

    const Mat3 rampRot = A.orientation.toMat3();
    const Vec3 rampComLocal = ramp->getLocalCenterOfMassOffset();

    const OBB boxObb = makeOBB(B, box);
    const std::vector<Vec3> boxVerts = getOBBCorners(boxObb);

    std::vector<Vec3> rampVerts;
    rampVerts.reserve(6);
    rampVerts.push_back(A.position + (rampRot * (Vec3(0.0f, 0.0f, -halfWidthZ) - rampComLocal)));
    rampVerts.push_back(A.position + (rampRot * (Vec3(0.0f, 0.0f, halfWidthZ) - rampComLocal)));
    rampVerts.push_back(A.position + (rampRot * (Vec3(length, 0.0f, -halfWidthZ) - rampComLocal)));
    rampVerts.push_back(A.position + (rampRot * (Vec3(length, 0.0f, halfWidthZ) - rampComLocal)));
    rampVerts.push_back(A.position + (rampRot * (Vec3(length, height, -halfWidthZ) - rampComLocal)));
    rampVerts.push_back(A.position + (rampRot * (Vec3(length, height, halfWidthZ) - rampComLocal)));

    float rampMinX = FLT_MAX;
    float rampMaxX = -FLT_MAX;
    float rampMinY = FLT_MAX;
    float rampMaxY = -FLT_MAX;
    float rampMinZ = FLT_MAX;
    float rampMaxZ = -FLT_MAX;
    for (const Vec3 &v : rampVerts)
    {
        rampMinX = std::min(rampMinX, v.x);
        rampMaxX = std::max(rampMaxX, v.x);
        rampMinY = std::min(rampMinY, v.y);
        rampMaxY = std::max(rampMaxY, v.y);
        rampMinZ = std::min(rampMinZ, v.z);
        rampMaxZ = std::max(rampMaxZ, v.z);
    }

    float boxMinX = FLT_MAX;
    float boxMaxX = -FLT_MAX;
    float boxMinY = FLT_MAX;
    float boxMaxY = -FLT_MAX;
    float boxMinZ = FLT_MAX;
    float boxMaxZ = -FLT_MAX;
    for (const Vec3 &v : boxVerts)
    {
        boxMinX = std::min(boxMinX, v.x);
        boxMaxX = std::max(boxMaxX, v.x);
        boxMinY = std::min(boxMinY, v.y);
        boxMaxY = std::max(boxMaxY, v.y);
        boxMinZ = std::min(boxMinZ, v.z);
        boxMaxZ = std::max(boxMaxZ, v.z);
    }

    if (!overlaps1D(boxMinX, boxMaxX, rampMinX, rampMaxX) ||
        !overlaps1D(boxMinY, boxMaxY, rampMinY, rampMaxY) ||
        !overlaps1D(boxMinZ, boxMaxZ, rampMinZ, rampMaxZ))
    {
        return false;
    }

    const float slopeLen = std::sqrt(length * length + height * height);
    if (slopeLen <= PHYSICS_EPSILON)
        return false;

    const Vec3 slopeNormal = rampRot * Vec3(-height / slopeLen, length / slopeLen, 0.0f);
    const Vec3 baseNormal = rampRot * Vec3(0.0f, -1.0f, 0.0f);
    const Vec3 backNormal = rampRot * Vec3((length >= 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f);
    const Vec3 sideNormal = rampRot * Vec3(0.0f, 0.0f, 1.0f);

    const Vec3 rampDirs[] = {
        rampRot * Vec3(1.0f, 0.0f, 0.0f),
        rampRot * Vec3(0.0f, 1.0f, 0.0f),
        rampRot * Vec3(0.0f, 0.0f, 1.0f),
        rampRot * Vec3(length, height, 0.0f)};

    std::vector<Vec3> satAxes;
    satAxes.reserve(18);
    satAxes.push_back(slopeNormal);
    satAxes.push_back(baseNormal);
    satAxes.push_back(backNormal);
    satAxes.push_back(sideNormal);
    satAxes.push_back(boxObb.axis[0]);
    satAxes.push_back(boxObb.axis[1]);
    satAxes.push_back(boxObb.axis[2]);

    for (const Vec3 &rampDir : rampDirs)
    {
        for (int i = 0; i < 3; ++i)
        {
            const Vec3 axis = rampDir.cross(boxObb.axis[i]);
            if (axis.length() > 1e-6f)
                satAxes.push_back(axis);
        }
    }

    float minOverlap = FLT_MAX;
    Vec3 bestAxis;

    for (const Vec3 &rawAxis : satAxes)
    {
        const float axisLen = rawAxis.length();
        if (axisLen <= PHYSICS_EPSILON)
            continue;

        const Vec3 axis = rawAxis * (1.0f / axisLen);

        float minRamp = 0.0f;
        float maxRamp = 0.0f;
        float minBox = 0.0f;
        float maxBox = 0.0f;
        projectVerts(rampVerts, axis, minRamp, maxRamp);
        projectVerts(boxVerts, axis, minBox, maxBox);

        const float overlap = std::min(maxRamp, maxBox) - std::max(minRamp, minBox);
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

    Vec3 normal = bestAxis.normalized();

    {
        const Vec3 upAxis(0.0f, 1.0f, 0.0f);
        float minRampY = 0.0f;
        float maxRampY = 0.0f;
        float minBoxY = 0.0f;
        float maxBoxY = 0.0f;
        projectVerts(rampVerts, upAxis, minRampY, maxRampY);
        projectVerts(boxVerts, upAxis, minBoxY, maxBoxY);
        const float overlapY = std::min(maxRampY, maxBoxY) - std::max(minRampY, minBoxY);

        if (overlapY > PHYSICS_EPSILON && overlapY <= (minOverlap * 1.35f))
        {
            const Vec3 centerDelta = B.position - A.position;
            const bool stronglyVertical = std::abs(centerDelta.y) >= std::max(std::abs(centerDelta.x), std::abs(centerDelta.z));
            if (stronglyVertical)
            {
                minOverlap = overlapY;
                normal = upAxis;
            }
        }
    }

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

    const Vec3 boxSupport = supportPoint(boxObb, normal * -1.0f);
    const Vec3 rampSupport = supportPointOnVerts(rampVerts, normal);
    base.contact_point = (boxSupport + rampSupport) * 0.5f;

    std::vector<Contact> candidates;
    candidates.reserve(6);
    if (isValidVec3(base.contact_point) && isValidFloat(base.penetration))
        pushUniqueContact(candidates, base);

    for (const Vec3 &corner : boxVerts)
    {
        if (!pointInsideRamp(A, *ramp, corner))
            continue;

        Contact c = base;
        c.contact_point = corner;
        c.penetration = minOverlap;
        if (isValidVec3(c.contact_point) && isValidFloat(c.penetration) && c.penetration > PHYSICS_EPSILON)
            pushUniqueContact(candidates, c);
    }

    for (const Vec3 &rv : rampVerts)
    {
        if (!pointInsideOBB(rv, boxObb))
            continue;

        Contact c = base;
        c.contact_point = rv;
        c.penetration = minOverlap;
        if (isValidVec3(c.contact_point) && isValidFloat(c.penetration) && c.penetration > PHYSICS_EPSILON)
            pushUniqueContact(candidates, c);
    }

    if (candidates.empty())
        return false;

    m.a = &A;
    m.b = &B;
    m.a_id = A.id;
    m.b_id = B.id;
    m.normal = normal;
    m.contact_count = 0;

    m.contacts[m.contact_count++] = candidates[0];

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
            m.contacts[m.contact_count++] = candidates[bestIdx];
    }

    for (int i = 0; i < m.contact_count; ++i)
    {
        m.contacts[i].a = m.a;
        m.contacts[i].b = m.b;
        m.contacts[i].a_id = m.a_id;
        m.contacts[i].b_id = m.b_id;
        m.contacts[i].normal = m.normal;
    }

    return m.contact_count > 0;
}

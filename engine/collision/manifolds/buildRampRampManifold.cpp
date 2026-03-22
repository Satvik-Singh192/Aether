#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/ramp_collider.hpp"
#include <algorithm>
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

    std::vector<Vec3> getSlopeCorners(const Rigidbody &body, const RampCollider &ramp)
    {
        const float z = ramp.getHalfWidthZ();
        const Vec3 p0 = body.position;
        const Vec3 p1 = body.position + Vec3(ramp.getLength(), ramp.getHeight(), 0.0f);

        std::vector<Vec3> corners;
        corners.reserve(4);
        corners.push_back(p0 + Vec3(0.0f, 0.0f, z));
        corners.push_back(p0 + Vec3(0.0f, 0.0f, -z));
        corners.push_back(p1 + Vec3(0.0f, 0.0f, z));
        corners.push_back(p1 + Vec3(0.0f, 0.0f, -z));
        return corners;
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

    void addProjectedCandidates(const Rigidbody &srcBody, const RampCollider &srcRamp,
                                const Rigidbody &dstBody, const RampCollider &dstRamp,
                                Vec3 planeNormal, const Contact &baseContact,
                                std::vector<Contact> &out)
    {
        if (planeNormal.length() <= PHYSICS_EPSILON)
            return;

        for (const Vec3 &corner : getSlopeCorners(srcBody, srcRamp))
        {
            const float signedDistance = (corner - dstBody.position).dot(planeNormal);
            if (signedDistance > 0.0f)
                continue;

            const Vec3 projected = corner - planeNormal * signedDistance;
            if (!isPointOnRampSlope(dstBody, dstRamp, projected))
                continue;

            Contact c = baseContact;
            c.contact_point = projected;
            pushUniqueContact(out, c);
        }
    }
}

// A and B are ramps.
bool buildRampRampManifold(Rigidbody &A, Rigidbody &B, ContactManifold &manifold)
{
    Contact baseContact;
    if (!buildRampRampContact(A, B, baseContact))
    {
        return false;
    }

    if (!isValidVec3(baseContact.normal) || !isValidFloat(baseContact.penetration) ||
        baseContact.penetration <= 0.0f || baseContact.penetration > 100.0f)
    {
        return false;
    }

    auto *rampA = static_cast<RampCollider *>(A.collider);
    auto *rampB = static_cast<RampCollider *>(B.collider);
    if (!rampA || !rampB)
        return false;

    Vec3 nA = buildSlopeNormal(*rampA);
    Vec3 nB = buildSlopeNormal(*rampB);
    if (nA.length() <= PHYSICS_EPSILON || nB.length() <= PHYSICS_EPSILON)
        return false;

    if (nA.dot(baseContact.normal) < 0.0f)
        nA = nA * -1.0f;

    // For B plane, orient normal toward A.
    if (nB.dot(baseContact.normal) > 0.0f)
        nB = nB * -1.0f;

    std::vector<Contact> candidates;
    pushUniqueContact(candidates, baseContact);

    addProjectedCandidates(B, *rampB, A, *rampA, nA, baseContact, candidates);
    addProjectedCandidates(A, *rampA, B, *rampB, nB, baseContact, candidates);

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

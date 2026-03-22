#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/box_collider.hpp"
#include "core/ramp_collider.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
    float clampValue(float v, float lo, float hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    bool isValidFloat(float v)
    {
        return std::isfinite(v);
    }

    bool isValidVec3(const Vec3 &v)
    {
        return isValidFloat(v.x) && isValidFloat(v.y) && isValidFloat(v.z);
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
    Contact baseContact;
    if (!buildRampBoxContact(A, B, baseContact))
    {
        return false;
    }

    if (!isValidVec3(baseContact.normal) || !isValidFloat(baseContact.penetration) ||
        baseContact.penetration <= 0.0f || baseContact.penetration > 100.0f)
    {
        return false;
    }

    auto *ramp = static_cast<RampCollider *>(A.collider);
    auto *box = static_cast<BoxCollider *>(B.collider);
    if (!ramp || !box)
        return false;

    std::vector<Contact> candidates;
    pushUniqueContact(candidates, baseContact);

    const Vec3 normal = baseContact.normal;
    const Vec3 half = box->getHalfExtents();

    // Use penetrating box corners and project them to ramp slope plane.
    for (int sx = -1; sx <= 1; sx += 2)
    {
        for (int sy = -1; sy <= 1; sy += 2)
        {
            for (int sz = -1; sz <= 1; sz += 2)
            {
                const Vec3 corner = B.position + Vec3(sx * half.x, sy * half.y, sz * half.z);
                const float signedDistance = (corner - A.position).dot(normal);
                if (signedDistance > 0.0f)
                    continue;

                const Vec3 projected = corner - normal * signedDistance;
                if (!isPointOnRampSlope(A, *ramp, projected))
                    continue;

                Contact c = baseContact;
                c.contact_point = projected;
                c.normal = normal;
                c.penetration = baseContact.penetration;
                pushUniqueContact(candidates, c);
            }
        }
    }

    // Add center projection as fallback spatial anchor.
    {
        const float d = (B.position - A.position).dot(normal);
        const Vec3 projectedCenter = B.position - normal * d;
        if (isPointOnRampSlope(A, *ramp, projectedCenter))
        {
            Contact c = baseContact;
            c.contact_point = projectedCenter;
            c.normal = normal;
            c.penetration = baseContact.penetration;
            pushUniqueContact(candidates, c);
        }
    }

    if (candidates.empty())
        return false;

    manifold.a = baseContact.a;
    manifold.b = baseContact.b;
    manifold.a_id = A.id;
    manifold.b_id = B.id;
    manifold.normal = normal;

    // Keep ramp manifolds small and stable: up to 2 spread contacts.
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
        manifold.contacts[i].normal = manifold.normal;
        manifold.contacts[i].penetration = clampValue(baseContact.penetration, 0.0f, 100.0f);
        manifold.contacts[i].a = manifold.a;
        manifold.contacts[i].b = manifold.b;
        manifold.contacts[i].a_id = manifold.a_id;
        manifold.contacts[i].b_id = manifold.b_id;
    }

    return manifold.contact_count > 0;
}

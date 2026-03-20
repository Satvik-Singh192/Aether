#include "collision/collision.hpp"
#include "core/box_collider.hpp"
#include "core/ramp_collider.hpp"
#include <algorithm>
#include <cmath>

namespace
{
    bool overlaps1D(float minA, float maxA, float minB, float maxB)
    {
        return maxA >= minB && maxB >= minA;
    }
}

bool buildRampBoxContact(Rigidbody &ramp_body, Rigidbody &box_body, Contact &outContact)
{
    auto *ramp = static_cast<RampCollider *>(ramp_body.collider);
    auto *box = static_cast<BoxCollider *>(box_body.collider);

    if (!ramp || !box)
        return false;
    if (std::abs(ramp->getLength()) <= PHYSICS_EPSILON)
        return false;

    const Vec3 half = box->getHalfExtents();
    const Vec3 boxMin = box_body.position - half;
    const Vec3 boxMax = box_body.position + half;

    float rampMinX = ramp_body.position.x;
    float rampMaxX = ramp_body.position.x + ramp->getLength();
    if (rampMinX > rampMaxX)
        std::swap(rampMinX, rampMaxX);

    float rampMinY = ramp_body.position.y;
    float rampMaxY = ramp_body.position.y + ramp->getHeight();
    if (rampMinY > rampMaxY)
        std::swap(rampMinY, rampMaxY);

    float rampMinZ = ramp_body.position.z - ramp->getHalfWidthZ();
    float rampMaxZ = ramp_body.position.z + ramp->getHalfWidthZ();

    if (!overlaps1D(boxMin.x, boxMax.x, rampMinX, rampMaxX) ||
        !overlaps1D(boxMin.y, boxMax.y, rampMinY, rampMaxY) ||
        !overlaps1D(boxMin.z, boxMax.z, rampMinZ, rampMaxZ))
    {
        return false;
    }

    const float height = ramp->getHeight();
    const float length = ramp->getLength();
    const float mag = std::sqrt(height * height + length * length);
    if (mag <= PHYSICS_EPSILON)
        return false;

    Vec3 rampNormal(-height / mag, length / mag, 0.0f);
    const Vec3 toBoxCenter = box_body.position - ramp_body.position;

    float signedDistance = toBoxCenter.dot(rampNormal);
    if (signedDistance < 0.0f)
    {
        rampNormal = rampNormal * -1.0f;
        signedDistance = -signedDistance;
    }

    const float distance = std::abs(signedDistance);
    const float projected_half_size =
        std::abs(rampNormal.x) * half.x +
        std::abs(rampNormal.y) * half.y +
        std::abs(rampNormal.z) * half.z;

    const float penetration = projected_half_size - distance;
    if (penetration <= 0.0f)
        return false;

    outContact = Contact{};
    outContact.a = &ramp_body;
    outContact.b = &box_body;
    outContact.normal = rampNormal;
    outContact.penetration = penetration;
    outContact.contact_point = box_body.position - rampNormal * distance;
    outContact.restitution = (ramp_body.restitution + box_body.restitution) * 0.5f;
    outContact.friction_coeff = std::sqrt(ramp_body.friction * box_body.friction);

    return true;
}

bool buildBoxRampContact(Rigidbody &box_body, Rigidbody &ramp_body, Contact &outContact)
{
    auto *box = static_cast<BoxCollider *>(box_body.collider);
    auto *ramp = static_cast<RampCollider *>(ramp_body.collider);

    // Stabilize ramp resting on the large static floor box.
    if (box && ramp &&
        box_body.inverse_mass == 0.0f &&
        box->halfsize.x >= 20.0f && box->halfsize.z >= 20.0f && box->halfsize.y <= 1.0f)
    {
        const Vec3 boxMin = box_body.position - box->getHalfExtents();
        const Vec3 boxMax = box_body.position + box->getHalfExtents();

        const float rampMinX = std::min(ramp_body.position.x, ramp_body.position.x + ramp->getLength());
        const float rampMaxX = std::max(ramp_body.position.x, ramp_body.position.x + ramp->getLength());
        const float rampMinY = std::min(ramp_body.position.y, ramp_body.position.y + ramp->getHeight());
        const float rampMaxY = std::max(ramp_body.position.y, ramp_body.position.y + ramp->getHeight());
        const float rampMinZ = ramp_body.position.z - ramp->getHalfWidthZ();
        const float rampMaxZ = ramp_body.position.z + ramp->getHalfWidthZ();

        if (!overlaps1D(boxMin.x, boxMax.x, rampMinX, rampMaxX) ||
            !overlaps1D(boxMin.z, boxMax.z, rampMinZ, rampMaxZ))
        {
            return false;
        }

        if (rampMaxY < boxMin.y || rampMinY > boxMax.y)
            return false;

        const float penetration = boxMax.y - rampMinY;
        if (penetration <= 0.0f)
            return false;

        outContact = Contact{};
        outContact.a = &box_body;
        outContact.b = &ramp_body;
        outContact.normal = Vec3(0.0f, 1.0f, 0.0f);
        outContact.penetration = penetration;
        outContact.contact_point = Vec3(ramp_body.position.x, boxMax.y, ramp_body.position.z);
        outContact.restitution = (box_body.restitution + ramp_body.restitution) * 0.5f;
        outContact.friction_coeff = std::sqrt(box_body.friction * ramp_body.friction);
        return true;
    }

    if (!buildRampBoxContact(ramp_body, box_body, outContact))
        return false;

    std::swap(outContact.a, outContact.b);
    outContact.normal = outContact.normal * -1.0f;
    return true;
}

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

    const Vec3 rampNormal(-height / mag, length / mag, 0.0f);
    const Vec3 toBoxCenter = box_body.position - ramp_body.position;

    const float distance = toBoxCenter.dot(rampNormal);
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
    if (!buildRampBoxContact(ramp_body, box_body, outContact))
        return false;

    std::swap(outContact.a, outContact.b);
    outContact.normal = outContact.normal * -1.0f;
    return true;
}

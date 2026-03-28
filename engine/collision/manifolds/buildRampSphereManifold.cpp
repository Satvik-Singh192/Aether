#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/ramp_collider.hpp"
#include "core/sphere_collider.hpp"
#include <algorithm>
#include <cmath>

namespace
{
    float clampRange(float value, float a, float b)
    {
        const float lo = std::min(a, b);
        const float hi = std::max(a, b);
        return std::max(lo, std::min(value, hi));
    }

    bool isValidFloat(float v)
    {
        return std::isfinite(v);
    }

    bool isValidVec3(const Vec3 &v)
    {
        return isValidFloat(v.x) && isValidFloat(v.y) && isValidFloat(v.z);
    }

    Vec3 closestPointOnTriangle2D(const Vec3 &p, const Vec3 &a, const Vec3 &b, const Vec3 &c)
    {
        const Vec3 ab = b - a;
        const Vec3 ac = c - a;
        const Vec3 ap = p - a;
        const float d1 = ab.dot(ap);
        const float d2 = ac.dot(ap);
        if (d1 <= 0.0f && d2 <= 0.0f)
            return a;

        const Vec3 bp = p - b;
        const float d3 = ab.dot(bp);
        const float d4 = ac.dot(bp);
        if (d3 >= 0.0f && d4 <= d3)
            return b;

        const float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        {
            const float v = d1 / (d1 - d3);
            return a + ab * v;
        }

        const Vec3 cp = p - c;
        const float d5 = ab.dot(cp);
        const float d6 = ac.dot(cp);
        if (d6 >= 0.0f && d5 <= d6)
            return c;

        const float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        {
            const float w = d2 / (d2 - d6);
            return a + ac * w;
        }

        const float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        {
            const Vec3 bc = c - b;
            const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + bc * w;
        }

        const float denom = 1.0f / (va + vb + vc);
        const float v = vb * denom;
        const float w = vc * denom;
        return a + ab * v + ac * w;
    }

    Vec3 localFallbackNormal(float length, float height)
    {
        const float slopeLen = std::sqrt(length * length + height * height);
        if (slopeLen <= PHYSICS_EPSILON)
            return Vec3(0.0f, 1.0f, 0.0f);
        return Vec3(-height / slopeLen, length / slopeLen, 0.0f);
    }
}

// A is ramp, B is sphere.
bool buildRampSphereManifold(Rigidbody &A, Rigidbody &B, ContactManifold &manifold)
{
    auto *ramp = static_cast<RampCollider *>(A.collider);
    auto *sphere = static_cast<SphereCollider *>(B.collider);
    if (!ramp || !sphere)
        return false;

    const float radius = sphere->radius;
    if (radius <= PHYSICS_EPSILON)
        return false;

    const float length = ramp->getLength();
    const float height = ramp->getHeight();
    const float halfWidthZ = ramp->getHalfWidthZ();
    if (std::abs(length) <= PHYSICS_EPSILON || std::abs(height) <= PHYSICS_EPSILON || halfWidthZ <= PHYSICS_EPSILON)
        return false;

    const Mat3 rampRot = A.orientation.toMat3();
    const Mat3 rampRotT = rampRot.transpose();
    const Vec3 rampComLocal = ramp->getLocalCenterOfMassOffset();

    const Vec3 sphereCenterLocal = rampComLocal + (rampRotT * (B.position - A.position));

    const float minX = std::min(0.0f, length);
    const float maxX = std::max(0.0f, length);
    const float minY = std::min(0.0f, height);
    const float maxY = std::max(0.0f, height);
    if (sphereCenterLocal.x < minX - radius || sphereCenterLocal.x > maxX + radius ||
        sphereCenterLocal.y < minY - radius || sphereCenterLocal.y > maxY + radius ||
        sphereCenterLocal.z < -halfWidthZ - radius || sphereCenterLocal.z > halfWidthZ + radius)
    {
        return false;
    }

    const Vec3 triA(0.0f, 0.0f, 0.0f);
    const Vec3 triB(length, 0.0f, 0.0f);
    const Vec3 triC(length, height, 0.0f);
    const Vec3 point2(sphereCenterLocal.x, sphereCenterLocal.y, 0.0f);

    const Vec3 closest2 = closestPointOnTriangle2D(point2, triA, triB, triC);
    const Vec3 closestLocal(closest2.x, closest2.y, clampRange(sphereCenterLocal.z, -halfWidthZ, halfWidthZ));

    const Vec3 localDelta = sphereCenterLocal - closestLocal;
    const float distSq = localDelta.dot(localDelta);
    if (!isValidFloat(distSq) || distSq > radius * radius)
        return false;

    float dist = 0.0f;
    Vec3 localNormal;
    if (distSq > PHYSICS_EPSILON)
    {
        dist = std::sqrt(distSq);
        localNormal = localDelta * (1.0f / dist);
    }
    else
    {
        localNormal = localFallbackNormal(length, height);
    }

    Vec3 normal = rampRot * localNormal;
    if (!isValidVec3(normal) || normal.length() <= PHYSICS_EPSILON)
        return false;
    normal = normal.normalized();

    if ((B.position - A.position).dot(normal) < 0.0f)
        normal = normal * -1.0f;

    const float penetration = radius - dist;
    if (!isValidFloat(penetration) || penetration <= PHYSICS_EPSILON)
        return false;

    const Vec3 contactPoint = A.position + (rampRot * (closestLocal - rampComLocal));
    if (!isValidVec3(contactPoint))
        return false;

    Contact c;
    c.a = &A;
    c.b = &B;
    c.a_id = A.id;
    c.b_id = B.id;
    c.normal = normal;
    c.penetration = penetration;
    c.contact_point = contactPoint;
    c.restitution = (A.restitution + B.restitution) * 0.5f;
    c.friction_coeff = std::sqrt(A.friction * B.friction);

    manifold.a = &A;
    manifold.b = &B;
    manifold.a_id = A.id;
    manifold.b_id = B.id;
    manifold.normal = normal;
    manifold.contact_count = 1;
    manifold.contacts[0] = c;

    return true;
}

#pragma once
#include "math/vec3.hpp"
#include "math/mat3.hpp"
#include "core/rigidbody.hpp"
#include "core/box_collider.hpp"
#include <cmath>

struct OBB {
    Vec3 center;
    Vec3 axis[3];      // world-space axes
    Vec3 halfsize;     // local half-extents
};

inline OBB makeOBB(const Rigidbody& body, const BoxCollider* box) {
    OBB obb;
    obb.center = body.position;
    obb.halfsize = box->halfsize;
    
    Mat3 R = body.orientation.toMat3();
    obb.axis[0] = R * Vec3(1, 0, 0);
    obb.axis[1] = R * Vec3(0, 1, 0);
    obb.axis[2] = R * Vec3(0, 0, 1);
    
    return obb;
}

inline bool overlapOnAxis(const OBB& A, const OBB& B, const Vec3& axis, float& penetration) {
    if (axis.length() < PHYSICS_EPSILON) return true;
    
    Vec3 n = axis.normalized();
    
    float projA = A.halfsize.x * std::abs(n.dot(A.axis[0])) +
                  A.halfsize.y * std::abs(n.dot(A.axis[1])) +
                  A.halfsize.z * std::abs(n.dot(A.axis[2]));
                  
    float projB = B.halfsize.x * std::abs(n.dot(B.axis[0])) +
                  B.halfsize.y * std::abs(n.dot(B.axis[1])) +
                  B.halfsize.z * std::abs(n.dot(B.axis[2]));
                  
    float dist = std::abs((B.center - A.center).dot(n));
    float overlap = projA + projB - dist;
    
    if (overlap < 0) return false;
    
    penetration = overlap;
    return true;
}

inline bool pointInsideOBB(const Vec3& p, const OBB& obb) {
    Vec3 local = p - obb.center;
    
    float dx = std::abs(local.dot(obb.axis[0]));
    float dy = std::abs(local.dot(obb.axis[1]));
    float dz = std::abs(local.dot(obb.axis[2]));
    
    return (dx <= obb.halfsize.x && 
            dy <= obb.halfsize.y && 
            dz <= obb.halfsize.z);
}

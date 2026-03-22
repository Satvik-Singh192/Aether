#include "quat.hpp"
#include <cmath>

Quat::Quat() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
Quat:: Quat (float w,float x, float y, float z)
    : w(w),x(x),y(y),z(z) {}

Quat Quat::identity() {
    return Quat(1.0f,0.0f, 0.0f, 0.0f);
}

Quat Quat::normalised() const {
    float mag = std::sqrt(w*w + x*x + y*y +z*z);
    if(mag==0.0f){
        return Quat(1.0f,0.0f,0.0f,0.0f);

    }
    return Quat(w/mag, x/mag, y/mag, z/mag);
}

void Quat::normalise() {
    float mag=std::sqrt(w*w + x*x + y*y+z*z);
    if(mag==0.0f){
        w=1.0f; x=y=z=0.0f;
        return;

    }
    w/=mag;
    x /= mag;
    y /= mag;
    z /= mag;
}
Quat Quat::operator*(const Quat& rhs) const {
    return Quat(
        w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
        w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
        w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
        w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
    );
}
Quat Quat::operator+(const Quat& rhs) const {
    return Quat(
        w + rhs.w,
        x + rhs.x,
        y + rhs.y,
        z + rhs.z
    );
}
Quat Quat::operator*(float scalar) const {
    return Quat(
        w * scalar,
        x * scalar,
        y * scalar,
        z * scalar
    );
}
Quat Quat::fromAngularVelocity(const Vec3& omega){
    return Quat(0.0f,omega.x,omega.y, omega.z);
}
Mat3 Quat::toMat3() const {
    float xx = x * x;
    float yy = y * y;
    float zz = z * z;

    float xy = x * y;
    float xz = x * z;
    float yz = y * z;

    float wx = w * x;
    float wy = w * y;
    float wz = w * z;

    Mat3 m;

    m.m[0][0] = 1.0f - 2.0f * (yy + zz);
    m.m[0][1] = 2.0f * (xy - wz);
    m.m[0][2] = 2.0f * (xz + wy);

    m.m[1][0] = 2.0f * (xy + wz);
    m.m[1][1] = 1.0f - 2.0f * (xx + zz);
    m.m[1][2] = 2.0f * (yz - wx);

    m.m[2][0] = 2.0f * (xz - wy);
    m.m[2][1] = 2.0f * (yz + wx);
    m.m[2][2] = 1.0f - 2.0f * (xx + yy);

    return m;
}


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
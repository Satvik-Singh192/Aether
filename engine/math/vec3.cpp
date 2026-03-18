#include "vec3.hpp"

Vec3::Vec3():x(0.0f),y(0.0f),z(0.0f){}

Vec3::Vec3(float x, float y, float z):x(x),y(y),z(z){}

Vec3 Vec3::operator+(const Vec3& other) const {
    return Vec3(x+other.x, y+other.y, z+other.z);
}

Vec3& Vec3::operator+=(const Vec3& other) {
    x+=other.x;
    y+=other.y;
    z+=other.z;
    return *this; 
}

Vec3& Vec3::operator-=(const Vec3& other) {
    x-=other.x;
    y-=other.y;
    z-=other.z;
    return *this; 
}

Vec3 Vec3::operator-(const Vec3& other) const {
    return Vec3(x-other.x, y-other.y,z-other.z);
}

Vec3 Vec3::operator*(float scalar) const {
    return Vec3(x * scalar, y * scalar, z * scalar);
}

float Vec3::dot(const Vec3& other) const {
    return x * other.x + y * other.y + z * other.z;
}

float Vec3::length() const {
    return std::sqrt(dot(*this));
}

Vec3 Vec3::normalized() const {
    float len = length();
    if (len == 0.0f) return Vec3();
    return (*this) * (1.0f / len);
}

std::ostream& operator<<(std::ostream& os,const Vec3& vector3){
    os<<'('<<vector3.x<<vector3.y<<vector3.z<<')';
    return os;
}

bool is_corrupt(const Vec3& vector3){
    return (!isfinite(vector3.x)||!isfinite(vector3.y)||!isfinite(vector3.z));
}
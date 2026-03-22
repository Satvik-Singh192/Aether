#pragma once
#include "vec3.hpp"

class Mat3 {
    public: 
        float m[3][3];

        Mat3();
        static Mat3 identity();
        Mat3 transpose() const;
        Vec3 operator*(const Vec3& v) const;
        Mat3 operator*(const Mat3& rhs) const;
        Mat3 operator*(float scalar) const;
        static Mat3 diag(float x, float y, float z);
};
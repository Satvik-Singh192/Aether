#pragma once
#include "vec3.hpp"
#include "mat3.hpp"
class Quat {
    public  : 
        float w,x,y,z;
        Quat();
        Quat(float w,float x, float y, float z);

        static Quat identity();
        Quat normalised() const;
        void normalise();

        Quat operator*(const Quat& rhs) const;
        Quat operator+(const Quat& rhs) const;
        Quat operator*(float scalar) const;
        static Quat fromAxisAngle(const Vec3& axis, float angle);
        Mat3 toMat3() const;
        static Quat fromAngularVelocity(const Vec3& omega);

};
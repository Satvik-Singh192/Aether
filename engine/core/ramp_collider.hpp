#pragma once
#include "collider.hpp"
#include "common_header.hpp"

class RampCollider : public Collider
{
public:
    float slope;
    float length;
    float half_width_z;

    RampCollider(float slope_value, float length_value, float half_width_z_value = 1.0f)
        : slope(slope_value), length(length_value), half_width_z(half_width_z_value)
    {
        type = ShapeType::Ramp;
    }

    float getSlope() const
    {
        return slope;
    }

    float getLength() const
    {
        return length;
    }

    float getHalfWidthZ() const
    {
        return half_width_z;
    }

    float getHeight() const
    {
        return slope * length;
    }

    Vec3 getLocalCenterOfMassOffset() const
    {
        return Vec3((2.0f * length) / 3.0f, getHeight() / 3.0f, 0.0f);
    }
};

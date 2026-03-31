#pragma once
enum class ShapeType
{
    Box,
    Sphere,
    Ramp
};
class Collider
{
public:
    ShapeType type = ShapeType::Box;

    // tells the compiler to clean up child;s rss first before cleaning this class(parents) rss.......prevents memory leaks
    virtual ~Collider() = default;
};
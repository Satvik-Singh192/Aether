#pragma once
enum class ShapeType{
    Box,
    Sphere
};
class Collider{
public:
    ShapeType type;

    // tells the compiler to clean up child;s rss first before cleaning this class(parents) rss.......prevents memory leaks
    virtual ~Collider()=default;
};
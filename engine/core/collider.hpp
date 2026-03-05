#pragma once
enum class ShapeType{
    Sphere,
    Box
};
class Collider{
public:
    ShapeType type;

    // tells the compiler to clean up child;s rss first before cleaning this class(parents) rss.......prevents memory leaks
    virtual ~Collider()=default;
};
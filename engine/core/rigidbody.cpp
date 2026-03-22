#include "core/rigidbody.hpp"

Rigidbody::Rigidbody(
    
    const Vec3& position,
    const Vec3& velocity,
    Collider* col,
    float mass,
    float fric,
    float res
)
    : position(position),velocity(velocity),force_accum(0,0,0),inverse_mass(0.0f),friction(fric),restitution(res),collider(col)
{
    const float MIN_MASS = PHYSICS_EPSILON;
    float effective_mass = mass;

    if (mass <= 0.0f) {
        std::cerr << "tried to create a rigidbody with non-positive mass, treating as static body\n";
        effective_mass = 0.0f;
    }
    else if (mass < MIN_MASS) {
        std::cerr<<"tried to create a rigidbody with too small mass, clamping to min mass: "<<MIN_MASS<<'\n';
        effective_mass = MIN_MASS;
    }

    inverse_mass = (effective_mass > 0.0f) ? 1.0f / effective_mass : 0.0f;

    friction = std::max(0.0f, friction);
    restitution = std::max(0.0f, restitution);
    restitution = std::min(1.0f, restitution);
    
}

void Rigidbody::applyForce(const Vec3& force) {
    force_accum=force_accum+force;
}

void Rigidbody::clearAccum() {
    force_accum = Vec3();
}
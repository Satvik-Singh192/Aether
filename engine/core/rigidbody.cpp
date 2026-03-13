#include "core/rigidbody.hpp"

Rigidbody::Rigidbody(const Vec3& pos,
                     const Vec3& vel,
                     Collider* col,
                     float mass
                    )
    : position(pos), velocity(vel),collider(col), force_accum(0,0,0), friction(0.5f)
{
    inverse_mass=(mass>0.0f)?1.0f/mass:0.0f;
}

void Rigidbody::applyForce(const Vec3& force) {
    force_accum=force_accum+force;
}

void Rigidbody::clearForces() {
    force_accum = Vec3();
}
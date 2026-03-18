#include "core/rigidbody.hpp"

Rigidbody::Rigidbody(const Vec3& pos,
                     const Vec3& vel,
                     Collider* col,
                     float mass
                    )
    : position(pos), velocity(vel),collider(col), force_accum(0,0,0), friction(PHYSICS_DEFAULT_FRICTION)
{
    const float MIN_MASS=PHYSICS_EPSILON;
    if(mass<0){
        std::cerr<<"tried to create a rigidbody with negative, clamping to 0\n";
    }
    else if(mass<MIN_MASS){
        std::cerr<<"tried to create a rigidbody with too small mass, clamping to min mass: "<<MIN_MASS<<'\n';
    }
    if(mass==0.0f){
        inverse_mass=0.0f;
    }
    else{
        inverse_mass=1.0f/mass;
    }
    inverse_mass=(mass>0.0f)?1.0f/mass:0.0f;
    
}

void Rigidbody::applyForce(const Vec3& force) {
    force_accum=force_accum+force;
}

void Rigidbody::clearForces() {
    force_accum = Vec3();
}
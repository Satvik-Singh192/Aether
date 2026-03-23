#include "core/rigidbody.hpp"
#include "core/sphere_collider.hpp"
#include "box_collider.hpp"


static Mat3 makeInverseInertiabody(const Collider* col, float mass) {
    if(mass<=0.0f || !col) {
        return Mat3::identity() * 0.0f;
    }
    if(col->type==ShapeType::Sphere){
        auto* s=static_cast<const SphereCollider*>(col);
        float I=0.4f*mass*s->radius*s->radius;
        float invI;
        if (I>PHYSICS_EPSILON) {
            invI = 1.0f/I;}
             else{
            invI = 0.0f;
        }
        return Mat3::diag(invI,invI,invI);
    }
    if (col->type == ShapeType::Box) {
        auto* b = static_cast<const BoxCollider*>(col);
        Vec3 e = b->halfsize * 2.0f;

        float Ix=(mass/12.0f)*(e.y*e.y +e.z*e.z);
        float Iy=(mass/12.0f)*(e.x*e.x+e.z*e.z);
        float Iz=(mass/12.0f)*(e.x*e.x+e.y*e.y);

        return Mat3::diag(
            Ix > PHYSICS_EPSILON ? 1.0f / Ix : 0.0f,
            Iy > PHYSICS_EPSILON ? 1.0f / Iy : 0.0f,
            Iz > PHYSICS_EPSILON ? 1.0f / Iz : 0.0f
        );
    }
   return Mat3::identity()*0.0f;
}

Rigidbody::Rigidbody(
    
    const Vec3& position,
    const Vec3& velocity,
    Collider* col,
    float mass,
    float fric,
    float res
)
    : position(position),velocity(velocity),force_accum(0,0,0),inverse_mass(0.0f),friction(fric),restitution(res),collider(col), orientation(Quat()),angvel(Vec3(0,0,0)), acctork(Vec3(0,0,0))
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
    float actualmass=(inverse_mass>0.0f)?(1.0f/inverse_mass):0.0f;
    inverse_inertia_body = makeInverseInertiabody(collider, actualmass);
    updateworldinvinertia();

    friction = std::max(0.0f, friction);
    restitution = std::max(0.0f, restitution);
    restitution = std::min(1.0f, restitution);
    
}
 

void Rigidbody::applyForce(const Vec3& force) {
    force_accum=force_accum+force;
}
void Rigidbody::applyTorque(const Vec3& torque) {
    acctork = acctork + torque;
}
void Rigidbody::updateworldinvinertia() {
    Mat3 R = orientation.toMat3();
    inverse_inertia_world = R * inverse_inertia_body * R.transpose();
}

void Rigidbody::clearForces() {
    force_accum = Vec3();
    
}
void Rigidbody::clearAccum() {
    force_accum = Vec3();
    acctork=Vec3();
    
}


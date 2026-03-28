#include "math/vec3.hpp"
#include "core/rigidbody.hpp"
#include "core/sphere_collider.hpp"
#include "core/box_collider.hpp"
#include "buoyancy.hpp"
#include "../engine_configs.hpp"
#include <algorithm>
#include <glm/glm.hpp>
void ApplyBuoyancyToSphere(Rigidbody& body, const Fluid&fluid, float gravity) {
    if(body.inverse_mass==0.0f) return;
    SphereCollider* sphere =dynamic_cast<SphereCollider*> (body.collider);
    if(!sphere) return;
    float radius =sphere->radius;
    float depth =fluid.height -body.position.y;
    float submergedvol=0.0f;
    float fullvol = (4.0f/3.0f)*PI*radius*radius*radius;
    if(depth<=-radius){
        submergedvol =0.0f;
    }
    else if(depth>=radius){
        submergedvol = fullvol;

    }
    else {
        float h=glm::clamp(depth+radius, 0.0f,2.0f*radius);
        submergedvol=(PI*h*h*((3.0f*radius) -h))/3.0f;
    }
    float absg=fabs(gravity);
    float fb = fluid.density*submergedvol*absg;
    body.applyForce(Vec3(0.0f, fb, 0.0f));
    float submersionratio ;
    if(fullvol>0.0f) {
        submersionratio=submergedvol/fullvol;
    }
    else submersionratio=0.0f;
    Vec3 damping = -Vec3(0.0f,body.velocity.y, 0.0f ) *fluid.drag_force*submersionratio;
    body.applyForce(damping);


} 
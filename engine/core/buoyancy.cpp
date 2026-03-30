#include "math/vec3.hpp"
#include "core/rigidbody.hpp"
#include "core/sphere_collider.hpp"
#include "core/box_collider.hpp"
#include "core/buoyancy.hpp"
#include "../engine_configs.hpp"
#include <algorithm>
#include <glm/glm.hpp>

bool IsBodyInBeaker(const Rigidbody& body, const Fluid& fluid) {
    
    Vec3 diff = body.position - fluid.beaker_center;
    if (body.collider && body.collider->type == ShapeType::Sphere) {
        SphereCollider* sphere = static_cast<SphereCollider*>(body.collider);
        if (glm::abs(diff.x) <= fluid.beaker_half_size && 
            glm::abs(diff.y) <= fluid.beaker_half_size && 
            glm::abs(diff.z) <= fluid.beaker_half_size) {
            return true;
        }
    }
    else {
        if (glm::abs(diff.x) <= fluid.beaker_half_size && 
            glm::abs(diff.y) <= fluid.beaker_half_size && 
            glm::abs(diff.z) <= fluid.beaker_half_size) {
            return true;
        }
    }
    
    return false;
}

void ApplyBuoyancyToSphere(Rigidbody& body, const Fluid&fluid, float gravity) {
    if(body.inverse_mass==0.0f) return;
    if(body.collider->type != ShapeType::Sphere) return;
    if (!IsBodyInBeaker(body, fluid)) return;
    
    SphereCollider* sphere = static_cast<SphereCollider*>(body.collider);
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
        Vec3 damping = Vec3(0.0f,body.velocity.y, 0.0f ) *fluid.drag_force*submersionratio * -1.0f;
        body.applyForce(damping);
    }
    
    void ApplyBuoyancyToBox(Rigidbody& body, const Fluid& fluid, float gravity){
        if(body.inverse_mass==0.0f) {
            return;
        }
        if(body.collider->type != ShapeType::Box) return;
        if (!IsBodyInBeaker(body, fluid)) return;
        
        BoxCollider* box = static_cast<BoxCollider*>(body.collider);

        Vec3 half_size=box->halfsize;
        float bottom=body.position.y -half_size.y;
        float top=body.position.y+ half_size.y;
        float fullvol= (2.0f*half_size.x ) *(2.0f*half_size.y)*(2.0f*half_size.z);
        float submergedheight = glm::clamp(fluid.height-bottom ,0.0f, 2.0f*half_size.y);
        float submergedvol=(2.0f * half_size.x)*(2.0f*half_size.z)*submergedheight ;
        float abs_gravity = fabs(gravity);
        float fb = fluid.density * submergedvol * abs_gravity;
        body.applyForce(Vec3(0.0f, fb, 0.0f));
        
        float submersionratio;
        if(fullvol > 0.0f) {
            submersionratio = submergedvol / fullvol;
        }
        else {
            submersionratio = 0.0f;
        }
        Vec3 damping = Vec3(0.0f, body.velocity.y, 0.0f) * fluid.drag_force * submersionratio * -1.0f;
        body.applyForce(damping);

    }
    void ApplyBuoyancy(Rigidbody& body, const Fluid& fluid, float gravity){
        if(!body.collider) return;
        switch(body.collider->type){
            case ShapeType::Sphere:
                ApplyBuoyancyToSphere(body,fluid, gravity);
                break;
            case ShapeType::Box:
                ApplyBuoyancyToBox(body,fluid,gravity);
                break;
            default:   
                break;
        }
    }



#include "world/physicsworld.hpp"
#include "collision/collision.hpp"
#include<vector>

PhysicsWorld::PhysicsWorld():gravity(0.0f,-9.8f,0.0f){}

void PhysicsWorld::addBody(const Rigidbody& body) {
	bodies.push_back(body);
	bodies_size++;
}

std::vector<Rigidbody>& PhysicsWorld::getBodies(){
	return bodies;
}

void PhysicsWorld::step(float dt) {
	for (auto& body:bodies) {
		if(body.inverse_mass==0.0f)continue;

		Vec3 GravityForce=gravity*(1.0f/body.inverse_mass);
		body.applyForce(GravityForce);
	}

	for (auto& body:bodies) {
		if(body.inverse_mass==0.0f)continue;

		Vec3 acceleration=body.force_accum*body.inverse_mass;
		body.velocity+=acceleration*dt;
		body.position+=body.velocity*dt;

		body.clearForces();
	}

	for(int i=0;i<bodies_size-1;i++){
		for(int j=i+1;j<bodies_size;j++){
			Collider* ca=bodies[i].collider;
			Collider* cb=bodies[j].collider;
			if(!ca||!cb)continue;
			if(ca->type==ShapeType::Sphere&&cb->type==ShapeType::Sphere)
				resolveSphereSphere(bodies[i],bodies[j]);
			else if(ca->type==ShapeType::Box&&cb->type==ShapeType::Box)
				resolveBoxBox(bodies[i],bodies[j]);
			else if(((ca->type==ShapeType::Sphere&&cb->type==ShapeType::Box)))
				resolveSphereBox(bodies[i],bodies[j]);
		}
	}


}

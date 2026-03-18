#include "world/physicsworld.hpp"
#include "collision/collision.hpp"
#include <unordered_map>
#include <utility>
#include <vector>

using CollisionKey = std::pair<ShapeType, ShapeType>;
using CollisionResolver = void(*)(Rigidbody&, Rigidbody&);

struct CollisionKeyHash {
	size_t operator()(const CollisionKey& key) const {
		return (static_cast<size_t>(key.first) << 8) ^ static_cast<size_t>(key.second);
	}
};

void resolveBoxSphere(Rigidbody& box_body, Rigidbody& sphere_body) {
	resolveSphereBox(sphere_body, box_body);
}

const std::unordered_map<CollisionKey, CollisionResolver, CollisionKeyHash> collision_resolvers = {
	{{ShapeType::Box, ShapeType::Box}, resolveBoxBox},
	{{ShapeType::Box, ShapeType::Sphere}, resolveBoxSphere},
	{{ShapeType::Sphere, ShapeType::Sphere}, resolveSphereSphere},
};

PhysicsWorld::PhysicsWorld():gravity(0.0f,PHYSICS_GRAVITY,0.0f){}

void PhysicsWorld::addBody(const Rigidbody& body) {
	bodies.push_back(body);
}

std::vector<Rigidbody>& PhysicsWorld::getBodies(){
	return bodies;
}

void PhysicsWorld::validate_body(Rigidbody& body){
	if(is_corrupt(body.position)){
		std::cout << "body position corrupted: " <<body.position<<'\n';
		body.position=Vec3();
	}
	if(is_corrupt(body.velocity)){
		std::cout << "body velocity corrupted: " <<body.velocity<<'\n';
		body.velocity=Vec3();
	}
	if(is_corrupt(body.force_accum)){
		std::cout << "body force accumulator corrupted: " <<body.force_accum<<'\n';
		body.force_accum=Vec3();
	}
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

	for(int i=0;i<bodies.size()-1;i++){
		for(int j=i+1;j<bodies.size();j++){
			Collider* ca=bodies[i].collider;
			Collider* cb=bodies[j].collider;
			if(!ca||!cb)continue;

			Rigidbody* first_body=&bodies[i];
			Rigidbody* second_body=&bodies[j];
			ShapeType first_type=ca->type;
			ShapeType second_type=cb->type;

			if(static_cast<int>(first_type)>static_cast<int>(second_type)){
				std::swap(first_type, second_type);
				std::swap(first_body, second_body);
			}

			auto resolver_it=collision_resolvers.find({first_type, second_type});
			if(resolver_it!=collision_resolvers.end()){
				resolver_it->second(*first_body, *second_body);
			}
		}
	}
	for(auto& body:bodies){
		validate_body(body);
	}


}

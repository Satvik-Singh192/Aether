#include "world/physicsworld.hpp"
#include <utility>
#include <vector>

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

	clear_contacts();
	generate_contacts();
	solve_contacts();
	//std::cout<<"Contacts: "<<contacts.size()<<'\n';
	for(auto& body:bodies){
		validate_body(body);
	}


}

void PhysicsWorld::generate_contacts(){
	for(int i=0;i<bodies.size()-1;i++){
		for(int j=i+1;j<bodies.size();j++){
			Rigidbody& a=bodies[i];
			Rigidbody& b=bodies[j];

			if(!a.collider||!b.collider)continue;
			Contact c;
			if (a.collider->type == ShapeType::Sphere &&
                b.collider->type == ShapeType::Sphere)
            {
                if (buildSphereSphereContact(a, b, c))
                    contacts.push_back(c);
            }
            else if (a.collider->type == ShapeType::Sphere &&
                     b.collider->type == ShapeType::Box)
            {
                if (buildSphereBoxContact(a, b, c))
                    contacts.push_back(c);
            }
            else if (a.collider->type == ShapeType::Box &&
                     b.collider->type == ShapeType::Sphere)
            {
                if (buildSphereBoxContact(b, a, c))
                {
                    std::swap(c.a, c.b);
                    c.normal = c.normal * -1.0f;
                    contacts.push_back(c);
                }
            }
            else if (a.collider->type == ShapeType::Box &&
                     b.collider->type == ShapeType::Box)
            {
                if (buildBoxBoxContact(a, b, c))
                    contacts.push_back(c);
            }
		}
	}
}
void PhysicsWorld::clear_contacts(){
	contacts.clear();
}

void PhysicsWorld::solve_contacts(){
	for(auto&c:contacts){
		Rigidbody a=*c.a;
		Rigidbody b=*c.b;

		float total_invmass=a.inverse_mass+b.inverse_mass;
		if(total_invmass==0.0f)continue; //contact bw 2 imovable objects must be ignoreeded

		Vec3 rel_vel=b.velocity-a.velocity; //following the A to B convention
		float relvel_along_normal=rel_vel.dot(c.normal);

		if(relvel_along_normal>-PHYSICS_EPSILON)continue; //skip already seprating thingies

		float e=c.restituion;
		float j=-(1.0+e)*relvel_along_normal;
		j/=total_invmass;

		Vec3 impulse=c.normal*j;

        a.velocity-=impulse*a.inverse_mass;
        b.velocity+=impulse*b.inverse_mass;
	}
}
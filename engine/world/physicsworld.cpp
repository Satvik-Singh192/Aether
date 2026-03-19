#include "world/physicsworld.hpp"
#include <utility>
#include <vector>

PhysicsWorld::PhysicsWorld():gravity(0.0f,PHYSICS_GRAVITY,0.0f),next_body_id(1){}

void PhysicsWorld::addBody(const Rigidbody& body) {
	Rigidbody copy = body;
	copy.id = next_body_id++;
	bodies.push_back(copy);
}

void PhysicsWorld::addBody(Rigidbody&& body) {
	body.id = next_body_id++;
	bodies.push_back(std::move(body));
}

std::vector<Rigidbody>& PhysicsWorld::getBodies(){

	return bodies;
}

std::size_t PhysicsWorld::getContactCount() const {
	return contacts.size();
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
	solve_position();
	//std::cout<<"Contacts: "<<contacts.size()<<'\n';
	for(auto& body:bodies){
		validate_body(body);
	}


}

void PhysicsWorld::generate_contacts(){
	for (std::size_t i = 0; i + 1 < bodies.size(); ++i) {
		for (std::size_t j = i + 1; j < bodies.size(); ++j) {
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
			else if (a.collider->type == ShapeType::Box &&
					 b.collider->type == ShapeType::Ramp)
			{
				if (buildBoxRampContact(a, b, c))
					contacts.push_back(c);
			}
			else if (a.collider->type == ShapeType::Ramp &&
					 b.collider->type == ShapeType::Box)
			{
				if (buildRampBoxContact(a, b, c))
					contacts.push_back(c);
			}
		}
	}
}
void PhysicsWorld::clear_contacts(){
	contacts.clear();
}

void PhysicsWorld::solve_contacts(){
	const int iterations=PHYSICS_VEL_SOLVER_ITERATION;
	for(int i=0;i<iterations;i++){
	for(auto&c:contacts){
		Rigidbody& a=*c.a;
		Rigidbody& b=*c.b;

		float total_invmass=a.inverse_mass+b.inverse_mass;
		if(total_invmass==0.0f)continue; //contact bw 2 imovable objects must be ignoreeded

		Vec3 rel_vel=b.velocity-a.velocity; //following the A to B convention
		float relvel_along_normal=rel_vel.dot(c.normal);

		if(relvel_along_normal>-PHYSICS_EPSILON)continue; //skip already seprating thingies

		float e=c.restitution;
		float j=-(1.0+e)*relvel_along_normal;
		j/=total_invmass;

		Vec3 impulse=c.normal*j;

        a.velocity-=impulse*a.inverse_mass;
        b.velocity+=impulse*b.inverse_mass;


		// lets handle friction now
		rel_vel=b.velocity-a.velocity; //recompute cuz it was updated during normal resolution
		Vec3 tangent=rel_vel-c.normal*rel_vel.dot(c.normal);

		float tangent_length=tangent.length();
		if(tangent_length<=PHYSICS_EPSILON) continue;
		tangent=tangent*(1.0f/tangent_length); //converting to unit vec

		float jt=-rel_vel.dot(tangent);
		jt/=total_invmass;

		float mu=c.friction_coeff;
		float maxfriction=mu*j; //j is the normal impulse 
		jt = std::max(-maxfriction, std::min(jt, maxfriction));   

		Vec3 friction_impulse=tangent*jt;
		a.velocity-=friction_impulse*a.inverse_mass;
		b.velocity+=friction_impulse*b.inverse_mass;

	}}
}

void PhysicsWorld::solve_position(){
	const int iterations=PHYSICS_POS_SOLVER_ITERATION;
	for(int i=0;i<iterations;i++){
	for(auto&c:contacts){
		Rigidbody&a=*c.a;
		Rigidbody&b=*c.b;

		float total_invmass=a.inverse_mass+b.inverse_mass;
		if(total_invmass==0.0f)continue; //contact bw 2 imovable objects must be ignoreeded

		float slop=PHYSICS_PENETRATION_SLOP;
		float percent=PHYSICS_CORRECTION_PERCENT;
		
		float penetration=c.penetration;

		float correction_mag=std::max(penetration-slop,0.0f);
		correction_mag=(correction_mag/total_invmass)*percent;

		Vec3 correction=c.normal*correction_mag;

		a.position-=correction * a.inverse_mass;
        b.position+=correction * b.inverse_mass;
	}
	}
}
#include "world/physicsworld.hpp"
#include "collision/manifolds/manifold_builders.hpp"
#include <utility>
#include <vector>
#include <cfloat>
#include <algorithm>

PhysicsWorld::PhysicsWorld() : gravity(0.0f, PHYSICS_GRAVITY, 0.0f), next_body_id(1) {}

std::uint32_t PhysicsWorld::addBody(const Rigidbody &body)
{
	Rigidbody copy = body;
	copy.id = next_body_id++;
	bodies.push_back(copy);
	syncAllPointers();
	return copy.id;
}

std::uint32_t PhysicsWorld::addBody(Rigidbody &&body)
{
	body.id = next_body_id++;
	bodies.push_back(std::move(body));
	syncAllPointers();
	return body.id;
}
Rigidbody* PhysicsWorld::getBodyByID(uint32_t body_id){
	auto it=lookup_map.find(body_id);
    if (it!=lookup_map.end()) {
        return it->second;
    }
    return nullptr;
}
void PhysicsWorld::rebuildLookupTable() {
    lookup_map.clear();
    for (auto& body:bodies) {
        lookup_map[body.id]=&body;
    }
}
PhysicsResult PhysicsWorld::deleteBody(uint32_t body_id){
	// we dont free up the space used by the collider pointer, cuz multiple rigibodies reuse the same collider
	auto it=std::remove_if(bodies.begin(),bodies.end(),[body_id](const Rigidbody&b){
		return b.id==body_id;
	});
	if(it==bodies.end()){
		return {false,PhysicsError::IDNotFound,"Body id: "+std::to_string(body_id)+" not found"};
	}
	else{
		deleteConstraint(body_id);
		
		bodies.erase(it,bodies.end());
		syncAllPointers();

		//delete all the manifolds and prev_manifolds where the target body is a member to prevent ghost collision
		auto it=std::remove_if(manifolds.begin(),manifolds.end(),[body_id](const ContactManifold&manifold){
			return (body_id==manifold.a_id||body_id==manifold.b_id);
		});
		if(it!=manifolds.end()){
			#ifdef DEBUG_MODE
				size_t count=std::distance(it,manifolds.end());
				std::cout<<count<<" current manifolds deleted\n";
			#endif
			manifolds.erase(it,manifolds.end());
		}
		it=std::remove_if(prev_manifolds.begin(),prev_manifolds.end(),[body_id](const ContactManifold&manifold){
			return (body_id==manifold.a_id||body_id==manifold.b_id);
		});
		if(it!=prev_manifolds.end()){
			#ifdef DEBUG_MODE
				size_t count=std::distance(it,prev_manifolds.end());
				std::cout<<count<<" previous manifolds deleted\n";
			#endif
			prev_manifolds.erase(it,prev_manifolds.end());
		}
		return {true,PhysicsError::None,"Body id: "+std::to_string(body_id)+" deleted successfully"};
	}
}

void PhysicsWorld::syncAllPointers(){
	rebuildLookupTable();
	for (auto& c:constraints) {
        c.a=lookup_map[c.a_id];
        c.b=lookup_map[c.b_id];
    }
	for (auto& m:manifolds) {
		m.a=lookup_map[m.a_id];
		m.b=lookup_map[m.b_id];
	}
	for (auto& m:prev_manifolds) {
		m.a=lookup_map[m.a_id];
		m.b=lookup_map[m.b_id];
	}
}

void PhysicsWorld::addforce(const Vec3 &force, std::uint32_t id)
{
	for (auto &it : bodies)
	{
		if (it.id == id)
		{
			it.applyForce(force);
			return;
		}
	}
}
std::vector<Rigidbody> &PhysicsWorld::getBodies()
{

	return bodies;
}

const Vec3 &PhysicsWorld::getGravity() const
{
	return gravity;
}

void PhysicsWorld::setGravity(const Vec3 &new_gravity)
{
	gravity = new_gravity;
}

const std::vector<DistanceConstraint> &PhysicsWorld::getDistanceConstraints() const
{
	return constraints;
}

std::size_t PhysicsWorld::getContactCount() const
{
	std::size_t total = 0;
	for (const auto &m : manifolds)
	{
		total += m.contact_count;
	}
	return total;
}

void PhysicsWorld::validate_body(Rigidbody &body)
{

	if (is_corrupt(body.position))
	{
		std::cout << "body position corrupted: " << body.position << '\n';
		body.position = Vec3();
	}
	if (is_corrupt(body.velocity))
	{
		std::cout << "body velocity corrupted: " << body.velocity << '\n';
		body.velocity = Vec3();
	}
	if (is_corrupt(body.force_accum))
	{
		std::cout << "body force accumulator corrupted: " << body.force_accum << '\n';
		body.force_accum = Vec3();
	}
}

void PhysicsWorld::step(float dt)
{
	// Make sure constraint endpoints are in sync with the bodies array layout
	syncAllPointers();
	for (auto &body : bodies)
	{
		if (body.inverse_mass == 0.0f)
			continue;

		Vec3 GravityForce = gravity * (1.0f / body.inverse_mass);
		body.applyForce(GravityForce);
		if(body.position.y<=-15)bodies_to_delete.push_back(body.id);
	}

	for (auto &body : bodies)
	{
		if (body.inverse_mass == 0.0f)
			continue;

		Vec3 acceleration = body.force_accum * body.inverse_mass;
		body.velocity += acceleration * dt;
		body.position += body.velocity * dt;
	}
	for(auto &body :bodies){
		if(body.inverse_mass==0.0f)
		continue;
		body.updateworldinvinertia();
		Vec3 angAcc =body.inverse_inertia_world*body.acctork;
		body.angvel+=angAcc*dt;
		Quat omega(0.0f, body.angvel.x, body.angvel.y, body.angvel.z);
		Quat qdot=(omega*body.orientation)*0.5f;
		body.orientation=body.orientation+qdot*dt;
		body.orientation=body.orientation.normalised();
		body.angvel=body.angvel*ANGULAR_DAMPING;
		const float ANG_SLEEP = 0.01f;
        if (body.angvel.dot(body.angvel) < ANG_SLEEP * ANG_SLEEP) {
            body.angvel = Vec3();
        }


	}
	for(auto &body:bodies){
		if(body.inverse_mass==0.0f)
		continue;
		body.clearAccum();
	}

	if(bodies_to_delete.size()!=0){
		for(std::uint32_t id: bodies_to_delete){
			deleteBody(id);
		}
		bodies_to_delete.clear();
	}

	prev_manifolds = manifolds;
	clear_manifolds();
	generate_manifolds();
	match_manifolds();
	for (auto &m : manifolds)
	{
		for (int i = 0; i < m.contact_count; i++)
		{
			Contact &c = m.contacts[i];
			c.rA=c.contact_point - c.a->position;
			c.rB=c.contact_point - c.b->position;
			//vel k liye linear + r x w
			Vec3 vA = c.a->velocity+c.a->angvel.cross(c.rA);
			Vec3 vB = c.b->velocity + c.b->angvel.cross(c.rB);
			c.pre_solve_normal_velocity = (vB-vA).dot(c.normal);
		}
	}
	warm_start_manifolds();
	warm_start_constraints();
	const int iterations = PHYSICS_VEL_SOLVER_ITERATION;

	for (int i = 0; i < iterations; i++)
	{
		solve_manifolds_vel_iteration();
		solve_constraints_vel(dt);
	}
	solve_manifolds_pos();

	// Apply angular damping once per frame after all solver iterations
	const float CONTACT_ANG_DAMP = 0.99f;
	for (auto &body : bodies)
	{
		body.angvel = body.angvel * CONTACT_ANG_DAMP;
		if (body.inverse_mass == 0.0f)
			continue;
		body.velocity = body.velocity * PHYSICS_LINEAR_DAMPING;
		if (std::abs(body.velocity.x) < 0.01f) body.velocity.x = 0;
	if (std::abs(body.velocity.z) < 0.01f) body.velocity.z = 0;
		float tangent_speed_sq = body.velocity.x * body.velocity.x + body.velocity.z * body.velocity.z;
		if (tangent_speed_sq < PHYSICS_RESTING_TANGENT_SLEEP_THRESHOLD * PHYSICS_RESTING_TANGENT_SLEEP_THRESHOLD)
		{
			body.velocity.x = 0.0f;
			body.velocity.z = 0.0f;
		}
		if (std::abs(body.velocity.y) < PHYSICS_RESTING_NORMAL_SLEEP_THRESHOLD)
		{
			body.velocity.y = 0.0f;
		}
		float vel_mag_sq = body.velocity.dot(body.velocity);
		if (vel_mag_sq < PHYSICS_SLEEP_VELOCITY_THRESHOLD * PHYSICS_SLEEP_VELOCITY_THRESHOLD)
		{
			body.velocity = Vec3();
		}
	}
}

void PhysicsWorld::generate_manifolds()
{
	for (std::size_t i = 0; i + 1 < bodies.size(); ++i)
	{
		for (std::size_t j = i + 1; j < bodies.size(); ++j)
		{
			Rigidbody &a = bodies[i];
			Rigidbody &b = bodies[j];

			if (!a.collider || !b.collider)
				continue;
			ContactManifold m;

			if (a.collider->type == ShapeType::Sphere &&
				b.collider->type == ShapeType::Sphere)
			{
				if (buildSphereSphereManifold(a, b, m))
				{
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Sphere &&
					 b.collider->type == ShapeType::Box)
			{
				if (buildBoxSphereManifold(b, a, m))
				{
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Box &&
					 b.collider->type == ShapeType::Sphere)
			{
				if (buildBoxSphereManifold(a, b, m))
				{
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Box &&
					 b.collider->type == ShapeType::Box)
			{
				if (buildBoxBoxManifold(a, b, m))
				{
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Box &&
					 b.collider->type == ShapeType::Ramp)
			{
				if (buildRampBoxManifold(b, a, m))
				{
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Ramp &&
					 b.collider->type == ShapeType::Box)
			{
				if (buildRampBoxManifold(a, b, m))
				{
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Sphere &&
					 b.collider->type == ShapeType::Ramp)
			{
				if (buildRampSphereManifold(b, a, m))
				{
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Ramp &&
					 b.collider->type == ShapeType::Sphere)
			{
				if (buildRampSphereManifold(a, b, m))
				{
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Ramp &&
					 b.collider->type == ShapeType::Ramp)
			{
				if (buildRampRampManifold(a, b, m))
				{
					manifolds.push_back(m);
				}
			}
		}
	}
}
void PhysicsWorld::clear_manifolds()
{
	manifolds.clear();
}

void PhysicsWorld::match_manifolds()
{
	for (auto &m : manifolds)
	{
		// Find matching old manifold
		for (const auto &old_m : prev_manifolds)
		{
			if (m.a_id == old_m.a_id && m.b_id == old_m.b_id)
			{
				// Found matching manifold pair, now match individual contacts
				for (int i = 0; i < m.contact_count; i++)
				{
					Contact &new_c = m.contacts[i];

					for (int j = 0; j < old_m.contact_count; j++)
					{
						const Contact &old_c = old_m.contacts[j];

						float dist = (old_c.contact_point - new_c.contact_point).length();
						float normal_alignment = new_c.normal.dot(old_c.normal);

						if (dist <= PHSYICS_CONTACT_SLOP && normal_alignment > 0.95f)
						{
							new_c.accumulated_normal_impulse = old_c.accumulated_normal_impulse;
							new_c.accumulated_tangent_impulse = old_c.accumulated_tangent_impulse;
							new_c.tangent = old_c.tangent;
							break;
						}
					}
				}
				break; // Found matching manifold, move to next new manifold
			}
		}
	}
}
void PhysicsWorld::warm_start_manifolds()
{
	for (auto &m : manifolds)
	{
		Rigidbody &a = *m.a;
		Rigidbody &b = *m.b;

		for (int i = 0; i < m.contact_count; i++)
		{
			Contact &c = m.contacts[i];

			Vec3 pn = c.normal * c.accumulated_normal_impulse;
			Vec3 pt = Vec3();

			if (c.tangent.length() > PHYSICS_EPSILON)
			{
				pt = c.tangent * c.accumulated_tangent_impulse;
			}

			Vec3 impulse = pn + pt;
			a.velocity -= impulse * a.inverse_mass;
			b.velocity += impulse * b.inverse_mass;
			a.angvel -= a.inverse_inertia_world * c.rA.cross(impulse);
			b.angvel += b.inverse_inertia_world * c.rB.cross(impulse);
		}
	}
}

void PhysicsWorld::solve_manifolds_vel_iteration()
{ //now it solves only 1 iteration of manifold , we move the iteration outside this function
		for (auto &m : manifolds)
		{
			Rigidbody &a = *m.a;
			Rigidbody &b = *m.b;

			for (int j = 0; j < m.contact_count; j++)
			{
				Contact &c = m.contacts[j];

				float total_invmass = a.inverse_mass + b.inverse_mass;
				if (total_invmass == 0.0f)
					continue; // contact bw 2 imovable objects must be ignoreeded
				Vec3 vA = a.velocity + a.angvel.cross(c.rA);
				Vec3 vB = b.velocity + b.angvel.cross(c.rB);
				Vec3 rel_vel = vB-vA; // following the A to B convention
				float relvel_along_normal = rel_vel.dot(c.normal);

				float target_post_normal_velocity = 0.0f;
				if (c.pre_solve_normal_velocity < -PHYSICS_RESTITUTION_VELOCITY_THRESHOLD)
				{
					target_post_normal_velocity = -c.restitution * c.pre_solve_normal_velocity;
				}
				Vec3 raXn = c.rA.cross(c.normal);
				Vec3 rbXn = c.rB.cross(c.normal);

				Vec3 angA = a.inverse_inertia_world * raXn;
				Vec3 angB = b.inverse_inertia_world * rbXn;

				float kNormal = a.inverse_mass + b.inverse_mass + c.normal.dot(angA.cross(c.rA) + angB.cross(c.rB));
				if (kNormal <= PHYSICS_EPSILON) continue;
				float jn = (target_post_normal_velocity - relvel_along_normal) / kNormal;

				float prev_normal_impulse = c.accumulated_normal_impulse;
				c.accumulated_normal_impulse = std::max(prev_normal_impulse + jn, 0.0f);

				float delta_impulse = c.accumulated_normal_impulse - prev_normal_impulse;

				Vec3 impulse = c.normal * delta_impulse;
				float maxAngular=10.0;

				a.velocity -= impulse * a.inverse_mass;
				b.velocity += impulse * b.inverse_mass;

				Vec3 torqueA=c.rA.cross(impulse);
				if (torqueA.length() > maxAngular) {
					torqueA = torqueA.normalized() * maxAngular;
				}
				Vec3 torqueB=c.rB.cross(impulse);
				if (torqueB.length() > maxAngular) {
					torqueB = torqueB.normalized() * maxAngular;
				}
				a.angvel-=a.inverse_inertia_world*torqueA;
				b.angvel+=b.inverse_inertia_world*torqueB;

				// lets handle friction now
				// Recompute velocity at contact point with NEW velocities after normal impulse

				vA = a.velocity + a.angvel.cross(c.rA);
				vB = b.velocity + b.angvel.cross(c.rB);
				rel_vel = vB - vA;
				float vn = rel_vel.dot(c.normal);
				Vec3 tangent = rel_vel - c.normal * vn;

				float tangent_length = tangent.length();
				if (tangent_length > PHYSICS_EPSILON) {
					tangent = tangent * (1.0f / tangent_length);
					c.tangent = tangent;
				} else {
					float cached_len = c.tangent.length();
					if (cached_len <= PHYSICS_EPSILON)
						continue;
					tangent = c.tangent * (1.0f / cached_len);
					tangent_length = cached_len;
				}

				float vt = rel_vel.dot(tangent);

				const float REST_VN = 0.01f;
				const float REST_VT = 0.02f;

				bool isResting = (std::abs(vn) < REST_VN && std::abs(vt) < REST_VT);

				Vec3 raXt = c.rA.cross(tangent);
				Vec3 rbXt = c.rB.cross(tangent);

				Vec3 angTA = a.inverse_inertia_world * raXt;
				Vec3 angTB = b.inverse_inertia_world * rbXt;

				float kTangent = a.inverse_mass + b.inverse_mass +
					tangent.dot(angTA.cross(c.rA) + angTB.cross(c.rB));

				if (kTangent <= PHYSICS_EPSILON) continue;
				float jt;
				if (isResting) {
					jt = -vt / kTangent;
				jt *= 0.1f; //dampening to the friction
			} else {
				jt = -vt / kTangent;
			}

			float prev_tangent_impulse = c.accumulated_tangent_impulse;
			float mu = c.friction_coeff;
			float maxfriction = mu * c.accumulated_normal_impulse;
			float new_tangent_impulse = prev_tangent_impulse + jt;
			c.accumulated_tangent_impulse = std::max(-maxfriction, std::min(new_tangent_impulse, maxfriction));
			
			float delta_tangent = c.accumulated_tangent_impulse - prev_tangent_impulse;
			Vec3 friction_impulse = tangent * delta_tangent;

			a.velocity -= friction_impulse * a.inverse_mass;
			b.velocity += friction_impulse * b.inverse_mass;

			a.angvel -= a.inverse_inertia_world * c.rA.cross(friction_impulse);
			b.angvel += b.inverse_inertia_world * c.rB.cross(friction_impulse);
		}
	}
}

void PhysicsWorld::solve_manifolds_pos()
{
	const int iterations = PHYSICS_POS_SOLVER_ITERATION;
	for (int i = 0; i < iterations; i++)
	{
		for (auto &m : manifolds)
		{
			Rigidbody &a = *m.a;
			Rigidbody &b = *m.b;

			for (int j = 0; j < m.contact_count; j++)
			{
				Contact &c = m.contacts[j];

				float total_invmass = a.inverse_mass + b.inverse_mass;
				if (total_invmass == 0.0f)
					continue; // contact bw 2 imovable objects must be ignoreeded

				float slop = PHYSICS_PENETRATION_SLOP;
				float percent = PHYSICS_CORRECTION_PERCENT;

				float penetration = c.penetration / static_cast<float>(std::max(1, m.contact_count));

				float correction_mag = std::max(penetration - slop, 0.0f);
				correction_mag = (correction_mag / total_invmass) * percent;

				Vec3 correction = c.normal * correction_mag;

				a.position -= correction * a.inverse_mass;
				b.position += correction * b.inverse_mass;
			}
		}
	}
}


//methods for constraints
PhysicsResult PhysicsWorld::addDistanceConstraints(
		std::uint32_t a_id,
		std::uint32_t b_id,
		float rest_length,
		DistanceConstraint::TYPE type,
		float stiffness,
		float damping){
		Rigidbody* a=nullptr;
		Rigidbody* b=nullptr;

		for(auto& body:bodies){
			if(body.id==a_id)a=&body;
			if(body.id==b_id)b=&body;
		}
		if(!a || !b){
			return {false,PhysicsError::IDNotFound,"Invalid Body ID passed to Constraint creator"};
		}
		if(a->collider->type==ShapeType::Ramp||b->collider->type==ShapeType::Ramp){
			return {false,PhysicsError::InvalidRequest,"Distance Constraints can't be applied to Ramps"};
		}
		constraints.push_back({
		a_id,
		b_id,
		a,
		b,
		rest_length,
		0.0f,
		stiffness,
		damping,
		type
	});
	return {true,PhysicsError::None,"Added "+std::to_string(type)+" constraint successfully"};

}

PhysicsResult PhysicsWorld::deleteConstraint(uint32_t first_body_id,uint32_t second_body_id){
	auto it=std::remove_if(constraints.begin(),constraints.end(),[first_body_id,second_body_id](const DistanceConstraint& constraint){
		return (first_body_id==constraint.a_id&&second_body_id==constraint.b_id)||(first_body_id==constraint.b_id&&second_body_id==constraint.a_id);
	});
	if(it==constraints.end()){
		return {false,PhysicsError::IDNotFound,"No distance constraint exists between "+std::to_string(first_body_id)+" and "+std::to_string(second_body_id)};
	}
	else{
		constraints.erase(it,constraints.end());
		return {true,PhysicsError::None,"Constraint deleted successfully"};
	}
}
PhysicsResult PhysicsWorld::deleteConstraint(uint32_t body_id){
	auto it=std::remove_if(constraints.begin(),constraints.end(),[body_id](const DistanceConstraint& constraint){
		return (body_id==constraint.a_id||body_id==constraint.b_id);
	});
	if(it==constraints.end()){
		return {false,PhysicsError::IDNotFound,"No distance constraint exists between "+std::to_string(body_id)+" and any other body"};
	}
	else{
		constraints.erase(it,constraints.end());
		return {true,PhysicsError::None,"Constraint deleted successfully"};
	}
}

static void solve_1D_constraint(
	Rigidbody& a,
    Rigidbody& b,
    const Vec3& normal,
    float& accumulated_impulse,
    float bias,
    float min_impulse,
    float max_impulse,
	const Vec3& rA,
    const Vec3& rB){

	/*
	this function is used to solve hard constrain (rod and rope). rope must be handeled sepratel
	*/
	Vec3 vA = a.velocity + a.angvel.cross(rA);
	Vec3 vB = b.velocity + b.angvel.cross(rB);
	Vec3 relvel = vB - vA;
	float rel = relvel.dot(normal);

	Vec3 raXn = rA.cross(normal);
	Vec3 rbXn = rB.cross(normal);

	Vec3 angA = a.inverse_inertia_world * raXn;
	Vec3 angB = b.inverse_inertia_world * rbXn;

	float inv_mass = a.inverse_mass + b.inverse_mass + normal.dot(angA.cross(rA) + angB.cross(rB));
	if(inv_mass <= PHYSICS_EPSILON) return;

	float j=-(rel+bias)/inv_mass;

	float prev=accumulated_impulse;
	accumulated_impulse = std::max(min_impulse, std::min(prev + j, max_impulse));
	j=accumulated_impulse-prev;

	Vec3 impulse=normal*j;

	a.velocity -= impulse * a.inverse_mass;
	b.velocity += impulse * b.inverse_mass;
	a.angvel -= a.inverse_inertia_world * rA.cross(impulse);
	b.angvel += b.inverse_inertia_world * rB.cross(impulse);
}

void PhysicsWorld::solve_constraints_vel(float dt){
	for(auto&c:constraints){
		if(!c.a||!c.b)continue;

		Vec3 delta=c.b->position-c.a->position;
		float dist=delta.length();
		if(dist<PHYSICS_EPSILON)continue;

		Vec3 n=delta*(1.0f/dist);
		float error=dist-c.rest_length; //positive: too far, negative: too close

		if(c.type==DistanceConstraint::ROPE&&error<0.0f){
			c.accumulated_impulse = 0.0f;
			continue;
		}

		if(c.type==DistanceConstraint::SPRING){
			//special handling for spring cuz no force is applied differently in rod and rope
			Vec3 relvel=c.b->velocity-c.a->velocity;
			float rel=relvel.dot(n);

			float force = -c.stiffness * error - c.damping * rel;
			float j = force * dt;
			float maxJ = 10.0f; //clamp to prevent exploion
			j = std::max(-maxJ, std::min(j, maxJ));
			Vec3 impulse=n*j;
			c.a->velocity-=impulse*c.a->inverse_mass;
			c.b->velocity+=impulse*c.b->inverse_mass;

			continue;
		}

		// Baumgarte stabilization: bias helps correct constraint drift
		float beta=0.2f;
		float bias=beta*error/dt;

		float min_impulse, max_impulse;
		if(c.type==DistanceConstraint::ROPE) {
			min_impulse = -FLT_MAX;  // Allow pulling
			max_impulse = 0.0f;      // Prevent pushing
		} else {
			min_impulse = -FLT_MAX;
			max_impulse = FLT_MAX;
		}

		Vec3 rA = delta * (-0.5f);
		Vec3 rB = delta * 0.5f;

		solve_1D_constraint(
			*c.a,
			*c.b,
			n,
			c.accumulated_impulse,
			bias,
			min_impulse,
			max_impulse,
			rA,
			rB
		);
	}
}

void PhysicsWorld::warm_start_constraints(){
	for (auto& c:constraints){
		if(!c.a||!c.b)continue;
		Vec3 delta=c.b->position-c.a->position;
		float dist=delta.length();
		if (dist<PHYSICS_EPSILON) continue;

		float error=dist-c.rest_length;

		// Skip warm start for slack ropes - they cannot pull
		if(c.type==DistanceConstraint::ROPE&&error<0.0f){
			continue;
		}

		Vec3 n=delta*(1.0f/dist);
		Vec3 impulse=n*c.accumulated_impulse;

		Vec3 rA = delta * (-0.5f);
		Vec3 rB = delta * 0.5f;

		c.a->velocity -= impulse * c.a->inverse_mass;
		c.b->velocity += impulse * c.b->inverse_mass;
		c.a->angvel -= c.a->inverse_inertia_world * rA.cross(impulse);
		c.b->angvel += c.b->inverse_inertia_world * rB.cross(impulse);
	}
}
#include "world/physicsworld.hpp"
#include <utility>
#include <vector>

PhysicsWorld::PhysicsWorld() : gravity(0.0f, PHYSICS_GRAVITY, 0.0f), next_body_id(1) {}

std::uint32_t PhysicsWorld::addBody(const Rigidbody &body)
{
	Rigidbody copy = body;
	copy.id = next_body_id++;
	bodies.push_back(copy);
	return copy.id;
}

std::uint32_t PhysicsWorld::addBody(Rigidbody &&body)
{
	body.id = next_body_id++;
	bodies.push_back(std::move(body));
	return body.id;
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

const Vec3& PhysicsWorld::getGravity() const
{
	return gravity;
}

void PhysicsWorld::setGravity(const Vec3& new_gravity)
{
	gravity = new_gravity;
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
	for (auto &body : bodies)
	{
		if (body.inverse_mass == 0.0f)
			continue;

		Vec3 GravityForce = gravity * (1.0f / body.inverse_mass);
		body.applyForce(GravityForce);
	}

	for (auto &body : bodies)
	{
		if (body.inverse_mass == 0.0f)
			continue;

		Vec3 acceleration = body.force_accum * body.inverse_mass;
		body.velocity += acceleration * dt;
		body.position += body.velocity * dt;

		body.clearForces();
	}
	prev_manifolds = manifolds;
	clear_contacts();
	generate_contacts();
	match_contacts();
	for (auto &m : manifolds)
	{
		for (int i = 0; i < m.contact_count; i++)
		{
			Contact &c = m.contacts[i];
			c.pre_solve_normal_velocity = (c.b->velocity - c.a->velocity).dot(c.normal);
		}
	}
	warm_start_contacts();
	solve_contacts();
	solve_position();

	const float SLEEP_VELOCITY_THRESHOLD = 0.001f;
	for (auto &body : bodies)
	{
		if (body.inverse_mass == 0.0f)
			continue;

		float vel_mag_sq = body.velocity.dot(body.velocity);
		if (vel_mag_sq < SLEEP_VELOCITY_THRESHOLD * SLEEP_VELOCITY_THRESHOLD)
		{
			body.velocity = Vec3();
		}
	}

	for (auto &body : bodies)
	{
		validate_body(body);
	}
}

void PhysicsWorld::generate_contacts()
{
	for (std::size_t i = 0; i + 1 < bodies.size(); ++i)
	{
		for (std::size_t j = i + 1; j < bodies.size(); ++j)
		{
			Rigidbody &a = bodies[i];
			Rigidbody &b = bodies[j];

			if (!a.collider || !b.collider)
				continue;

			Contact c;
			ContactManifold m;
			m.a = &a;
			m.b = &b;
			m.a_id = a.id;
			m.b_id = b.id;

			if (a.collider->type == ShapeType::Sphere &&
				b.collider->type == ShapeType::Sphere)
			{
				if (buildSphereSphereContact(a, b, c))
				{
					m.contacts[0] = c;
					m.contact_count = 1;
					m.normal = c.normal;
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Sphere &&
					 b.collider->type == ShapeType::Box)
			{
				if (buildSphereBoxContact(a, b, c))
				{
					m.contacts[0] = c;
					m.contact_count = 1;
					m.normal = c.normal;
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Box &&
					 b.collider->type == ShapeType::Sphere)
			{
				if (buildSphereBoxContact(b, a, c))
				{
					std::swap(c.a, c.b);
					std::swap(c.a_id, c.b_id);
					c.normal = c.normal * -1.0f;
					m.contacts[0] = c;
					m.contact_count = 1;
					m.normal = c.normal;
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Box &&
					 b.collider->type == ShapeType::Box)
			{
				if (buildBoxBoxContact(a, b, c))
				{
					m.contacts[0] = c;
					m.contact_count = 1;
					m.normal = c.normal;
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Box &&
					 b.collider->type == ShapeType::Ramp)
			{
				if (buildBoxRampContact(a, b, c))
				{
					m.contacts[0] = c;
					m.contact_count = 1;
					m.normal = c.normal;
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Ramp &&
					 b.collider->type == ShapeType::Box)
			{
				if (buildRampBoxContact(a, b, c))
				{
					m.contacts[0] = c;
					m.contact_count = 1;
					m.normal = c.normal;
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Sphere &&
					 b.collider->type == ShapeType::Ramp)
			{
				if (buildSphereRampContact(a, b, c))
				{
					m.contacts[0] = c;
					m.contact_count = 1;
					m.normal = c.normal;
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Ramp &&
					 b.collider->type == ShapeType::Sphere)
			{
				if (buildRampSphereContact(a, b, c))
				{
					m.contacts[0] = c;
					m.contact_count = 1;
					m.normal = c.normal;
					manifolds.push_back(m);
				}
			}
			else if (a.collider->type == ShapeType::Ramp &&
					 b.collider->type == ShapeType::Ramp)
			{
				if (buildRampRampContact(a, b, c))
				{
					m.contacts[0] = c;
					m.contact_count = 1;
					m.normal = c.normal;
					manifolds.push_back(m);
				}
			}
		}
	}
}
void PhysicsWorld::clear_contacts()
{
	manifolds.clear();
}

void PhysicsWorld::match_contacts()
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
void PhysicsWorld::warm_start_contacts()
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
		}
	}
}

void PhysicsWorld::solve_contacts()
{
	const int iterations = PHYSICS_VEL_SOLVER_ITERATION;
	const float RESTITUTION_VELOCITY_THRESHOLD = 0.1f;
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

				Vec3 rel_vel = b.velocity - a.velocity; // following the A to B convention
				float relvel_along_normal = rel_vel.dot(c.normal);

				float target_post_normal_velocity = 0.0f;
				if (c.pre_solve_normal_velocity < -RESTITUTION_VELOCITY_THRESHOLD)
				{
					target_post_normal_velocity = -c.restitution * c.pre_solve_normal_velocity;
				}

				float jn = (target_post_normal_velocity - relvel_along_normal) / total_invmass;

				float prev_normal_impulse = c.accumulated_normal_impulse;
				c.accumulated_normal_impulse = std::max(prev_normal_impulse + jn, 0.0f);

				float delta_impulse = c.accumulated_normal_impulse - prev_normal_impulse;

				Vec3 impulse = c.normal * delta_impulse;

				a.velocity -= impulse * a.inverse_mass;
				b.velocity += impulse * b.inverse_mass;

				// lets handle friction now
				rel_vel = b.velocity - a.velocity; // recompute cuz it was updated during normal resolution
				Vec3 tangent = rel_vel - c.normal * rel_vel.dot(c.normal);
				float tangent_length = tangent.length();
				if (tangent_length > PHYSICS_EPSILON)
				{
					tangent = tangent * (1.0f / tangent_length);
					c.tangent = tangent;
				}
				else
				{
					float cached_len = c.tangent.length();
					if (cached_len <= PHYSICS_EPSILON)
						continue;
					tangent = c.tangent * (1.0f / cached_len);
				}

				float jt = -rel_vel.dot(tangent);
				jt /= total_invmass;

				float prev_tangent_impulse = c.accumulated_tangent_impulse;

				float mu = c.friction_coeff;
				float maxfriction = mu * c.accumulated_normal_impulse;
				float new_tangent_impulse = prev_tangent_impulse + jt;
				c.accumulated_tangent_impulse = std::max(-maxfriction, std::min(new_tangent_impulse, maxfriction));

				float delta_tangent = c.accumulated_tangent_impulse - prev_tangent_impulse;

				Vec3 friction_impulse = tangent * delta_tangent;
				a.velocity -= friction_impulse * a.inverse_mass;
				b.velocity += friction_impulse * b.inverse_mass;
			}
		}
	}
}

void PhysicsWorld::solve_position()
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

				float penetration = c.penetration;

				float correction_mag = std::max(penetration - slop, 0.0f);
				correction_mag = (correction_mag / total_invmass) * percent;

				Vec3 correction = c.normal * correction_mag;

				a.position -= correction * a.inverse_mass;
				b.position += correction * b.inverse_mass;
			}
		}
	}
}
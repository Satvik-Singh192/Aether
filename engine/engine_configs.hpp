#pragma once

constexpr float PHYSICS_FIXED_DTv=1.0f/60.0f;
constexpr int PHYSICS_MAX_SUBSTEP=8; //how many max frame backlogs are allowed (if we dont limit this , we will spiral into infinte laggggg)

constexpr float PHYSICS_EPSILON=1e-4f; //to check if 2 floats are equal if their diff is lesst than this

constexpr float PHYSICS_GRAVITY=-9.8f;

constexpr float PHYSICS_DEFAULT_RESTITUTION=0.5f;
constexpr float PHYSICS_DEFAULT_FRICTION=0.3f;

constexpr float PHSYICS_CONTACT_SLOP=0.02f; //if contacts difference less than this then new contact will be treated the same as old contact, called in file physics world::match_contacts()
constexpr float PHYSICS_PENETRATION_SLOP=0.01f; //small tolerance for overlap so engine doesnt jitter the object resting on a surface to solve mircroscopic overlapp
constexpr float PHYSICS_CORRECTION_PERCENT=0.12f; //percent of overlap resolved in 1 frame, if we resolved fully in 1 frame ....it looks like crazy teleport

constexpr int PHYSICS_VEL_SOLVER_ITERATION=8; // how many times a contact is solved for velocity
constexpr int PHYSICS_POS_SOLVER_ITERATION=3; // how many times a contact is solved for pos

// Multiplies linear velocity every step after solver. Values < 1 reduce residual jitter energy.
constexpr float PHYSICS_LINEAR_DAMPING=0.995f;

// If a body is currently in contact manifolds and horizontal speed (x/z plane) is below this,
// snap x/z velocity to zero to stop tiny friction-driven sliding drift.
constexpr float PHYSICS_RESTING_TANGENT_SLEEP_THRESHOLD=0.05f;

// If a body is currently in contact manifolds and |y velocity| is below this,
// snap y velocity to zero to stop tiny vertical bouncing/oscillation on resting contacts.
constexpr float PHYSICS_RESTING_NORMAL_SLEEP_THRESHOLD=0.04f;

// Global full-speed sleep threshold. If total speed magnitude is below this,
// velocity is fully zeroed regardless of contact state.
constexpr float PHYSICS_SLEEP_VELOCITY_THRESHOLD=0.005f;
//ROTATION//
constexpr float ANGULAR_DAMPING = 0.995f;
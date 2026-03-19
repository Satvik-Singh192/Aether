#pragma once

constexpr float PHYSICS_FIXED_DTv=1.0f/60.0f;
constexpr int PHYSICS_MAX_SUBSTEP=8; //how many max frame backlogs are allowed (if we dont limit this , we will spiral into infinte laggggg)

constexpr float PHYSICS_EPSILON=1e-4f; //to check if 2 floats are equal if their diff is lesst than this

constexpr float PHYSICS_GRAVITY=-9.8f;

constexpr float PHYSICS_DEFAULT_RESTITUTION=0.5f;
constexpr float PHYSICS_DEFAULT_FRICTION=0.5f;

constexpr float PHYSICS_PENETRATION_SLOP=0.03f; //small tolerance for overlap so engine doesnt jitter the object resting on a surface to solve mircroscopic overlapp
constexpr float PHYSICS_CORRECTION_PERCENT=0.7f; //percent of overlap resolved in 1 frame, if we resolved fully in 1 frame ....it looks like crazy teleport

constexpr int PHYSICS_VEL_SOLVER_ITERATION=8; // how many times a contact is solved for velocity
constexpr int PHYSICS_POS_SOLVER_ITERATION=3; // how many times a contact is solved for pos

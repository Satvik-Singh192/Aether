#pragma once

constexpr float PHYSICS_FIXED_DTv=1.0f/60.0f;
constexpr int PHYSICS_MAX_SUBSTEP=8; //how many max frame backlogs are allowed (if we dont limit this , we will spiral into infinte laggggg)

constexpr float PHYSICS_EPSILON=1e-4f; //to check if 2 floats are equal if their diff is lesst than this

constexpr float PHYSICS_GRAVITY=-9.8f;

constexpr float PHYSICS_DEFAULT_RESTITUION=0.5f;
constexpr float PHYSICS_DEFAULT_FRICTION=0.5f;


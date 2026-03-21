#pragma once

#include "collision/contactmanifold.hpp"

bool buildBoxBoxManifold(Rigidbody& A, Rigidbody& B, ContactManifold& manifold);
bool buildBoxSphereManifold(Rigidbody& A, Rigidbody& B, ContactManifold& manifold);
bool buildSphereSphereManifold(Rigidbody& A, Rigidbody& B, ContactManifold& manifold);

bool buildRampBoxManifold(Rigidbody& A, Rigidbody& B, ContactManifold& manifold);
bool buildRampSphereManifold(Rigidbody& A, Rigidbody& B, ContactManifold& manifold);
bool buildRampRampManifold(Rigidbody& A, Rigidbody& B, ContactManifold& manifold);

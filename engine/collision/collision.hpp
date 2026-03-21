#pragma once
#include "core/rigidbody.hpp"
#include "contact.hpp"

bool buildSphereSphereContact(Rigidbody &a, Rigidbody &b, Contact &outContact);
bool buildBoxBoxContact(Rigidbody &a, Rigidbody &b, Contact &outContact);
bool buildSphereBoxContact(Rigidbody &sphere_body, Rigidbody &box_body, Contact &outContact);
bool buildBoxRampContact(Rigidbody &box_body, Rigidbody &ramp_body, Contact &outContact);
bool buildRampBoxContact(Rigidbody &ramp_body, Rigidbody &box_body, Contact &outContact);
bool buildSphereRampContact(Rigidbody &sphere_body, Rigidbody &ramp_body, Contact &outContact);
bool buildRampSphereContact(Rigidbody &ramp_body, Rigidbody &sphere_body, Contact &outContact);
bool buildRampRampContact(Rigidbody &a, Rigidbody &b, Contact &outContact);

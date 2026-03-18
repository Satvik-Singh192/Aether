#pragma once
#include "core/rigidbody.hpp"
#include "collision/contact.hpp"

bool buildSphereSphereContact(Rigidbody& a, Rigidbody& b, Contact& outContact);
bool buildBoxBoxContact(Rigidbody& a, Rigidbody& b, Contact& outContact);
bool buildSphereBoxContact(Rigidbody& sphere_body, Rigidbody& box_body, Contact& outContact);
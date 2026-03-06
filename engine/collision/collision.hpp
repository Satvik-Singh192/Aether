#pragma once
#include "core/rigidbody.hpp"

void resolveSphereSphere(Rigidbody&a,Rigidbody&b);
void resolveBoxBox(Rigidbody&a, Rigidbody&b);
void resolveSphereBox(Rigidbody&sphere_body,Rigidbody&box_body);
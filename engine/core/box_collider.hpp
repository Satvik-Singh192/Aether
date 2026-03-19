#pragma once
#include "collider.hpp"
#include "common_header.hpp"
class BoxCollider : public Collider {
    public : 
        Vec3 halfsize;
        BoxCollider(const Vec3& hs){
            halfsize =hs;// teeno axis ka half dist according to dimension, x/2,y/2,z/2 lena h yha pe, just like radius , aur jab collision hoga , teeno axis simultaneously check hongi, aur jo andar ghusne wali race condition hai wo bhi teeno axis me check krni hogi.
            type = ShapeType::Box;
        }

        const Vec3& getHalfExtents() const {
            return halfsize;// for case suppose if want to mamke half size private
        }
};
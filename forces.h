#pragma once
#include "bodies.h"
#include "common.h"

class Force {
    public: 
        virtual void apply(Part& part) = 0;
        virtual ~Force() = default;
};


class Gravity : public Force { 
    public:
        void apply(Part& part) override { part.acc += Vec2(0, -Physics::gravAcc); };
};


class Spring : public Force {
    public: 
        Vec2 pos;
        double L0;
        double k;

        Spring(Vec2 pos, double L0, double k) : pos(pos), L0(L0), k(k) {}

        void apply(Part& part) override {
            double d = sqrt((pos.x - part.pos.x)*(pos.x - part.pos.x) + (pos.y - part.pos.y)*(pos.y-part.pos.y));
            Vec2 u = (pos - part.pos).unit();
            part.acc += k*d*u/part.mass();
        }
};


class Damp : public Force { 
    public: 
        double c;

        Damp(double c) : c(c) {}

        void apply(Part& part) override {
            part.acc += -part.vel*c;
        }
};
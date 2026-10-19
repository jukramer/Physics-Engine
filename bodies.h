#pragma once
#include <iostream>
#include "common.h"

class Part {
    private:
        double mass_;
        double r_;

    public: 
        Vec2 pos;
        Vec2 vel;
        Vec2 acc;
        const int id;

        Part(double mass, double r, const Vec2& pos, const Vec2& vel, const Vec2& acc, int id = std::rand()) : mass_(mass), r_(r), pos(pos), vel(vel), acc(acc), id(id) {}

        double mass() const { return mass_; }
        double r() const { return r_; }
        Rect rect() const { return Rect(pos, 2*r_, 2*r_); }

        void setMass(double mass) { mass_ = mass; }
        void setr(double r) { r_ = r; }

        void printInfo() const { 
            std::cout << "=====[Particle " << id << "]=====\nMass: " << mass_ << "\n";
            std::cout << "Position: (" << pos.x << ", " << pos.y <<")\n";
            std::cout << "Velocity: (" << vel.x << ", " << vel.y <<")\n";
            std::cout << "Acceleration: (" << acc.x << ", " << acc.y <<")\n";
        }
};


class RigidBody {
    public:
        RigidBody() {}
};

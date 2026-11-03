#pragma once
#include <iostream>
#include <memory>
#include "collision.h"
#include "common.h"


enum class BodyType {
    RigidBody,
    Particle
};


class Body {
    public:
        Vec2 pos;
        Vec2 vel;
        Vec2 acc;

        const BodyType type;
        const int id;

        Body(const Vec2& pos, const Vec2& vel, const Vec2& acc, const BodyType type, const int id) : pos(pos), vel(vel), acc(acc), type(type), id(id) {}
        virtual ~Body() = default;

        virtual const std::vector<std::unique_ptr<Collider>>& colliders() const = 0;
};


class Part : public Body {
    private:
        double mass_;
        double r_;
        std::vector<std::unique_ptr<Collider>> collider_;

    public: 
        Part(double mass, double r, const Vec2& pos, const Vec2& vel, const Vec2& acc, int id = std::rand()) : Body(pos, vel, acc, BodyType::Particle, id), mass_(mass), r_(r) {
            collider_.push_back(std::make_unique<PartCollider>(pos, 2*r, 2*r));
        }

        double mass() const { return mass_; }
        double r() const { return r_; }
        const std::vector<std::unique_ptr<Collider>>& colliders() const override { return collider_;  }

        void setMass(double mass) { mass_ = mass; }
        void setr(double r) { r_ = r; }

        void printInfo() const { 
            std::cout << "=====[Particle " << id << "]=====\nMass: " << mass_ << "\n";
            std::cout << "Position: (" << pos.x << ", " << pos.y <<")\n";
            std::cout << "Velocity: (" << vel.x << ", " << vel.y <<")\n";
            std::cout << "Acceleration: (" << acc.x << ", " << acc.y <<")\n";
        }
};


class RigidBody : public Body {
    private:
        double mass_;
        double invMass_;
        std::vector<std::unique_ptr<Collider>> colliders_;

    public:
        RigidBody(const double mass, const Vec2& pos, const Vec2& vel, const Vec2& acc, const int id) : Body(pos, vel, acc, BodyType::RigidBody, id), mass_(mass), invMass_(1/mass) {}

        double mass() const { return mass_; };
        double invMass() const { return invMass_; };
        const std::vector<std::unique_ptr<Collider>>& colliders() const override { return colliders_; }

        void addCollider(std::unique_ptr<Collider> collider) { colliders_.push_back(std::move(collider)); }
};

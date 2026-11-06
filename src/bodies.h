#pragma once
#include <iostream>
#include <memory>
#include "common.h"


class Collider;
class PolygonCollider;
class PartCollider;


enum class BodyType {
    RigidBody,
    Particle
};


class Body {
    public:
        Body(const Vec2& pos, const Vec2& vel, const Vec2& acc, const BodyType type, const int id) : pos(pos), vel(vel), acc(acc), type(type), id(id) {}
        virtual ~Body() = default;

        Vec2 pos;
        Vec2 vel;
        Vec2 acc;

        const BodyType type;
        const int id;

        virtual const std::vector<std::unique_ptr<Collider>>& colliders() const = 0;

        virtual void printInfo() const = 0;
};


class Part : public Body {
    public:
        Part(double mass, double r, const Vec2& pos, const Vec2& vel, const Vec2& acc, const int id);

        double mass() const { return mass_; }
        const std::vector<std::unique_ptr<Collider>>& colliders() const override { return collider_;  }

        void setMass(double mass) { mass_ = mass; }

        void printInfo() const override;

    private:
        double mass_;
        std::vector<std::unique_ptr<Collider>> collider_;

};


class RigidBody : public Body {
    public:
        RigidBody(const double mass, const Vec2& pos, const Vec2& vel, const Vec2& acc, const double theta, const double omega, const double alpha, const int id) : Body(pos, vel, acc, BodyType::RigidBody, id), theta(theta), omega(omega), alpha(alpha), mass_(mass), invMass_(1/mass) {}

        double mass() const { return mass_; }
        double invMass() const { return invMass_; }
        const std::vector<std::unique_ptr<Collider>>& colliders() const override { return colliders_; }

        void addCollider(std::unique_ptr<Collider> collider) { colliders_.push_back(std::move(collider)); }
        void printInfo() const override;

        double theta;
        double omega;
        double alpha;

    private:
        double mass_;
        double invMass_;
        double MOI_;
        std::vector<std::unique_ptr<Collider>> colliders_;
};

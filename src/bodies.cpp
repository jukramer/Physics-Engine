#include <memory>
#include "bodies.h"
#include "collider.h"

inline Part::Part(const double mass, const double r, const Vec2& pos, const Vec2& vel, const Vec2& acc, const int id = std::rand()) : Body(pos, vel, acc, BodyType::Particle, id), mass_(mass) {
    collider_.push_back(std::make_unique<PartCollider>(pos, r));
}

void Part::printInfo() const {
    std::cout << "=====[Particle " << id << "]=====\nMass: " << mass_ << "\n";
    std::cout << "Position: (" << pos.x << ", " << pos.y <<")\n";
    std::cout << "Velocity: (" << vel.x << ", " << vel.y <<")\n";
    std::cout << "Acceleration: (" << acc.x << ", " << acc.y <<")\n";
}

void RigidBody::printInfo() const {
    std::cout << "=====[Member " << id << "]=====\nMass: " << mass_ << "\n";
    std::cout << "Position: (" << pos.x << ", " << pos.y <<")\n";
    std::cout << "Velocity: (" << vel.x << ", " << vel.y <<")\n";
    std::cout << "Acceleration: (" << acc.x << ", " << acc.y <<")\n";
}

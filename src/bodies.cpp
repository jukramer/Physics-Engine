#include <memory>
#include "bodies.h"
#include "collider.h"


Part::Part(double mass, double r, const Vec2& pos, const Vec2& vel, const Vec2& acc, const int id = std::rand()) : Body(pos, vel, acc, BodyType::Particle, id), mass_(mass), r_(r) {
    collider_.push_back(std::make_unique<PartCollider>(pos, r));
}
// #pragma once
// #include "bodies.h"
// #include "common.h"
//
// enum ForceType {
//     Gravity,
//     Spring,
//     Damp,
//     Electromagnetic,
//     Drag
// };
//
//
// struct ForceDef {
//     ForceType type;
// };
//
//
// class Force {
//     protected:
//         ForceType type_;
//     public:
//         virtual void apply(Body& body) = 0;
//         virtual ~Force() = default;
//         ForceType type() const { return type_; }
// };
//
//
// class Gravity : public Force {
//     public:
//         void apply(Body& body) override { body.acc += Vec2(0, -Physics::gravAcc); };
// };
//
//
// class Spring : public Force {
//     public:
//         Vec2 pos;
//         double L0;
//         double k;
//
//         Spring(Vec2 pos, double L0, double k) : pos(pos), L0(L0), k(k) {}
//
//         void apply(Body& body) override {
//             double d = sqrt((pos.x - body.pos.x)*(pos.x - body.pos.x) + (pos.y - body.pos.y)*(pos.y-body.pos.y));
//             Vec2 u = (pos - body.pos).unit();
//             // body.acc += k*d*u/body.mass;
//         }
// };
//
//
// class Damp : public Force {
//     public:
//         double c;
//
//         Damp(double c) : c(c) {}
//
//         void apply(Body& body) override {
//             body.acc += -body.vel*c;
//         }
// };
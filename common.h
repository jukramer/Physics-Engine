#pragma once
#include <cmath>
#include "forces.h"


namespace Physics {
    constexpr double dt = 1e-3;
    constexpr double elementalCharge = 1.602e-19;
    constexpr double gravAcc = 9.80665;
    constexpr double gravConst = 6.674e-11;
}


// class Collider {
//     private:
//         Vec2 pos_;
//         std::vector<Vec2> verts_;
//         double xmin_;
//         double xmax_;
//         double ymin_;
//         double ymax_;

//     public:
//         Collider(Vec2 pos, std::vector<Vec2> verts) :
//                 pos_(pos), 
//                 verts_(verts), 
//                 xmin_(std::min_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.x < b.x; })->x), 
//                 xmax_(std::max_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.x < b.x; })->x),
//                 ymin_(std::min_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.y < b.y; })->y),
//                 ymax_(std::max_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.y < b.y; })->y)
//         {} 
// };
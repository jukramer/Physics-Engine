#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "bodies.h"
#include "collision.h"
#include "common.h"
#include "forces.h"
#include "world.h"


int main() {
    // World world;
    Collision c;

    auto verts1 = std::vector<Vec2>{Vec2(-1, 1), Vec2(1,1), Vec2(1,-1), Vec2(-1,-1)};
    auto verts2 = std::vector<Vec2>{Vec2(-1, 1), Vec2(1,1), Vec2(1,-1), Vec2(-1,-1)};

    PolygonCollider c1(Vec2(-5,-5), Vec2(0,0), verts1);
    PolygonCollider c2(Vec2(5,5), Vec2(0,0), verts2);
    std::vector<Collider*> colliders{&c1, &c2};

    for (double n = 0; n < 5; n += 0.01) {
        auto res = c.USGCollision(colliders, -20, 20, -20, 20, 40, 40);
        std::vector<std::pair<PolygonCollider*, PolygonCollider*>> res1;

        std::cout << n << ",1," << c1.pos.x << "," << c1.pos.y << "\n";
        std::cout << n << ",2," << c2.pos.x << "," << c2.pos.y << "\n";


        for (auto& pair : res) {
            res1.push_back(std::make_pair(dynamic_cast<PolygonCollider*>(pair.first), dynamic_cast<PolygonCollider*>(pair.second)));
        }

        auto res2 = c.SATCollision(res1);

        // for (auto& result : res2) {
        //
        //     // if (result.colliding) {
        //     //     std::cout << "Colliding!\n";
        //     // }
        // }

        c1.pos += Vec2( 0.02,  0.02);
        c2.pos += Vec2(-0.02, -0.02);
    }

    // world.addMember(Part(1, 1, Vec2(-5, 0), Vec2(4, 0), Vec2(0, 0), 1));
    // world.addMember(Part(1, 1, Vec2(5, 0), Vec2(-4, 0), Vec2(0, 0), 2));
    // world.addMember(Part(1, 1, Vec2(0, 5), Vec2(0, -4), Vec2(0, 0), 3));
    // world.addMember(Part(1, 1, Vec2(0, -5), Vec2(0, 4), Vec2(0, 0), 4));
    //
    // for (double t = 0; t < 5; t += Physics::dt) {
    //     c.resolveCollision(world.members);
    //     world.update(Physics::dt);
    //     world.renderOut(std::vector{1, 2, 3, 4});
    // }

    // std::unique_ptr<Gravity> grav = std::make_unique<Gravity>();
    // std::unique_ptr<Spring> spring = std::make_unique<Spring>(Vec2(0,0), 1, 5);
    // std::unique_ptr<Damp> damp = std::make_unique<Damp>(3);

    // World world;

    // world.addMember(part1);
    // world.addMember(part2);
    // world.addForce(std::move(grav));
    // world.addForce(std::move(spring));
    // world.addForce(std::move(damp));

    // for (double t = 0; t < 2; t += Physics::dt) {
    //     world.update(Physics::dt);
    // }

    // world.printInfo(std::vector<int>{1,2});
}
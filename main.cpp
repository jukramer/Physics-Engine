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
    World world;
    Collision c;

    Part part1(1, 1, Vec2(-5, 0), Vec2(4, 0), Vec2(0, 0), 1);
    Part part2(1, 1, Vec2(5, 0), Vec2(-4, 0), Vec2(0, 0), 2);
    Part part3(1, 1, Vec2(0, 5), Vec2(0, -4), Vec2(0, 0), 3);
    Part part4(1, 1, Vec2(0, -5), Vec2(0, 4), Vec2(0, 0), 4);

    world.addMember(part1);
    world.addMember(part2);
    world.addMember(part3);
    world.addMember(part4);

    for (double t = 0; t < 5; t += Physics::dt) {
        c.resolveCollision(world.members);
        world.update(Physics::dt);
        world.renderOut(std::vector{1, 2, 3, 4});
    }

    // Part part1(1, 1, Vec2(0, 5), Vec2(0, 0), Vec2(0, 0), 1);
    // Part part2(1, 1, Vec2(0, 0), Vec2(3, 3), Vec2(0, 0), 2);

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
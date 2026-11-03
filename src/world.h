#pragma once
#include <vector>
#include "bodies.h"

class World { 
    private:

    public: 
        double t;
        std::vector<Part> members;
        std::vector<std::unique_ptr<Force>> forces;

        World() : t(0) {}

        void addMember(Part member) { members.push_back(std::move(member)); }
        void addForce(std::unique_ptr<Force> force) { forces.push_back(std::move(force)); }

        void update(double dt) {
            for (auto& member : members) {
                member.acc = Vec2(0,0);
                for (auto& force : forces) {
                    force->apply(member);
                }
                member.vel += member.acc*dt;
                member.pos += member.vel*dt;
            }
            t += dt;
        }

        void printInfo(std::vector<int> ids) {
            for (auto& id : ids) {
                for (auto& member : members) {
                    if (member.id == id) {
                        member.printInfo();
                    }
                }
            }
        }

        void renderOut(std::vector<int> ids) {
            for (auto& id : ids) {
                for (auto& member : members) {
                    if (member.id == id) {
                        std::cout << t << ',' << id << ',' << member.pos.x << ',' << member.pos.y << ',' << member.r() << '\n';
                    }
                }
            }
        }
};



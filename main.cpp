#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include "common.h"


class Part {
    private:
        Vec2 pos_;
        double r_;
        Rect rect_;

    public: 
        double mass;
        Vec2 vel;
        Vec2 acc;
        const int id;

        Part(double mass, double r, const Vec2& pos, const Vec2& vel, const Vec2& acc, double id = std::rand()) : mass(mass), r_(r), pos_(pos), vel(vel), acc(acc), id(id), rect_(pos, r/2, r/2){}
        
        void setPos(Vec2 pos) { 
            pos_ = pos; 
            rect_.setPos(pos_);
        }

        void incPos(Vec2 pos) {
            pos_ += pos;
            rect_.setPos(pos_);
        }

        void setr(double r) {
            r_ = r;
            rect_.seth(r/2);
            rect_.setw(r/2);
        }

        Vec2 pos() const { return pos_; }
        Rect rect() const { return rect_; }
        double r() const { return r_; }

        void printInfo() const { 
            std::cout << "=====[Particle " << id << "]=====\nMass: " << mass << "\n";
            std::cout << "Position: (" << pos_.x << ", " << pos_.y <<")\n";
            std::cout << "Velocity: (" << vel.x << ", " << vel.y <<")\n";
            std::cout << "Acceleration: (" << acc.x << ", " << acc.y <<")\n";
        }
};


class Force {
    public: 
        virtual void apply(Part& part) = 0;
        virtual ~Force() = default;
};


class Gravity : public Force { 
    public:
        void apply(Part& part) override { part.acc += Vec2(0, -Physics::g); };
};


class Spring : public Force {
    public: 
        Vec2 pos;
        double L0;
        double k;

        Spring(Vec2 pos, double L0, double k) : pos(pos), L0(L0), k(k) {}

        void apply(Part& part) override {
            double d = sqrt((pos.x - part.pos().x)*(pos.x - part.pos().x) + (pos.y - part.pos().y)*(pos.y-part.pos().y));
            Vec2 u = (pos - part.pos()).unit();
            part.acc += k*d*u/part.mass;
        }
};


class Damp : public Force { 
    public: 
        double c;

        Damp(double c) : c(c) {}

        void apply(Part& part) override {
            part.acc += -part.vel*c;
        }
};


struct CollisionResult {
    std::pair<Part*, Part*> pair;
    bool colliding;
    Vec2 collisionPoint;
    Vec2 sepDir;
    double depth;

    CollisionResult(std::pair<Part*, Part*> pair, bool colliding = false, Vec2 collisionPoint = Vec2(0,0), Vec2 sepDir = Vec2(0,0), double depth = 0) : pair(pair), colliding(colliding), collisionPoint(collisionPoint), sepDir(sepDir), depth(depth) {}
};


class Collision {
    public:
        Collision() {}

        // Global collision
        void resolveCollision(std::vector<Part>& members) {
            // Broad phase collision
            auto [partXmin, partXmax] = std::minmax_element(members.begin(), members.end(), 
                                                    [] (const auto& a, const auto& b) {
                                                        return a.pos().x < b.pos().x;
                                                    });

            auto [partYmin, partYmax] = std::minmax_element(members.begin(), members.end(), 
                                                    [] (const auto& a, const auto& b) {
                                                        return a.pos().y < b.pos().y;
                                                    });

            double xmin = std::floor(partXmin->rect().xmin()), xmax = std::floor(partXmax->rect().xmax());
            double ymin = std::floor(partYmin->rect().ymin()), ymax = std::floor(partYmax->rect().ymax());
            int m = xmax - xmin;
            int n = ymax - ymin;

            auto pairs = USGCollision(members, xmin, xmax, ymin, ymax, m, n);

            // Narrow phase collision
            auto collisions = circCollision(pairs);

            // Resolve collision, update velocities
            for (auto& collision : collisions) {
                Vec2 u1 = collision.pair.first->vel;
                Vec2 u2 = collision.pair.second->vel;
                double m1 = collision.pair.first->mass;
                double m2 = collision.pair.second->mass;
                Vec2 n = collision.sepDir.normal();

                Vec2 v1 = u1 - 2*m2/(m1+m2)*((u1-u2).dot(n))*n;
                Vec2 v2 = u2 + 2*m1/(m1+m2)*((u1-u2).dot(n))*n;

                collision.pair.first->vel = v2;
                collision.pair.second->vel = v1;
            }
        }

    private:
        // Broad Phase Collision
        std::vector<std::pair<Part*, Part*>> USGCollision(std::vector<Part>& members, double xmin, double xmax, double ymin, double ymax, int m, int n) const {
            std::vector<std::pair<Part*, std::pair<int,int>>> partCells;

            // Find cells covered by each particle
            for (auto& member : members) {
                // Min and max cell indices covered by particle bounding box
                int mmin = static_cast<int>((member.rect().xmin() - xmin)*m/(xmax-xmin));
                int nmin = static_cast<int>((member.rect().ymin() - ymin)*m/(ymax-ymin));
                int mmax = static_cast<int>(std::ceil((member.rect().xmax() - xmin)*m/(xmax-xmin)));
                int nmax = static_cast<int>(std::ceil((member.rect().ymax() - ymin)*m/(ymax-ymin)));

                // Store each cell covered by particle bounding box
                for (int i = mmin; i < mmax+1; i++) {
                    for (int j = nmin; j < nmax+1; j++) {
                        partCells.push_back(std::make_pair(&member, std::make_pair(i, j)));
                    }
                }
            }

            // Sort by cell indices (adjacent cells together)
            std::sort(partCells.begin(), partCells.end(), 
                      [] (const auto& a, const auto& b) { return a.second < b.second; });
            
            // Group particles of same cell into vectors, store in outside vector
            std::vector<std::vector<Part*>> partsGrouped;
            std::vector<Part*> group;
            bool firstPart = true; // prevent groups of only 1 particle

            for (int i=0; i < partCells.size()-1; i++) {
                if (partCells[i].second == partCells[i+1].second) {
                    group.push_back(partCells[i].first);
                    firstPart = false;
                } else if ((partCells[i].second != partCells[i+1].second) && !firstPart) {
                    group.push_back(partCells[i].first);
                    partsGrouped.push_back(group);
                    group.clear();
                    firstPart = true;
                }
            }

            // Generate pairs of particles per group
            std::vector<std::pair<Part*, Part*>> partsPaired;
            
            for (auto& group : partsGrouped) {
                std::vector<bool> mask(group.size());
                fill(mask.begin(), mask.begin() + 2, true);
                do {
                    std::vector<Part*> pairVec;
                    for (int i=0; i<mask.size(); i++) {
                        if (mask[i]) {
                            pairVec.push_back(group[i]);
                        }
                    } 
                    sort(pairVec.begin(), pairVec.end(), [] (const auto& a, const auto& b) { return a->id < b->id; });
                    std::pair<Part*, Part*> pair(pairVec[0], pairVec[1]);
                    partsPaired.push_back(pair);
                } while (std::next_permutation(mask.begin(), mask.end()));
            }

            // Remove duplicates
            sort(partsPaired.begin(), partsPaired.end(), 
                [] (const auto& a, const auto& b) { 
                    if (a.first->id != b.first->id) {
                        return a.first->id < b.first->id;
                    }
                    return a.second->id < b.second->id;
                }
            );
            
            auto last = std::unique(partsPaired.begin(), partsPaired.end());
            partsPaired.erase(last, partsPaired.end());

            return partsPaired;
        }

        // Narrow Phase Collision
        std::vector<CollisionResult> circCollision(std::vector<std::pair<Part*, Part*>>& pairs) const {
            std::vector<CollisionResult> result;

            for (auto& pair : pairs) {
                double d = sqrt(pow(pair.first->pos().x - pair.second->pos().x, 2) + pow(pair.first->pos().y + pair.second->pos().y, 2));

                if (d < pair.first->r() + pair.second->r()) {
                    Vec2 collisionPoint = (pair.second->pos() - pair.first->pos())/2 + pair.first->pos();
                    double depth = pair.first->r() + pair.second->r() - d;
                    Vec2 sepDir = (pair.second->pos() - pair.first->pos()).unit();

                    result.push_back(CollisionResult(pair, true, collisionPoint, sepDir, depth));
                }
            }
            

            return result;
        }

        std::vector<CollisionResult> SATCollision(std::vector<std::pair<Part*, Part*>>& pairs) const {
            
        }

};


class World { 
    private:

    public: 
        double t;
        std::vector<Part> members;
        std::vector<std::unique_ptr<Force>> forces;

        World() {}

        void addMember(Part member) { members.push_back(std::move(member)); }
        void addForce(std::unique_ptr<Force> force) { forces.push_back(std::move(force)); }

        void update(double dt) {
            for (auto& member : members) {
                member.acc = Vec2(0,0);
                for (auto& force : forces) {
                    force->apply(member);
                }
                member.vel += member.acc*dt;
                member.setPos(member.pos() + member.vel*dt);
                t += dt;
            }
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
                        std::cout << t << ',' << id << ',' << member.pos().x << ',' << member.pos().y << ',' << member.r() << '\n';
                    }
                }
            }
        }
};


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

    for (double t; t < 5; t += Physics::dt) {
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
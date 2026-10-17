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


class Collision {
    public: 
        Collision() {}

        std::vector<std::pair<Part*, Part*>> USGCollision(std::vector<Part>& members, double xmin, double xmax, double ymin, double ymax, int m, int n) const {
            std::vector<std::pair<Part*, std::pair<int,int>>> partCells;

            // Find cells covered by each particle
            for (auto& member : members) {
                // Min and max cell indices covered by particle bounding box
                int mmin = static_cast<int>((member.rect().xmin() - xmin)*m/(xmax-xmin));
                int nmin = static_cast<int>((member.rect().ymin() - ymin)*m/(ymax-ymin));
                int mmax = static_cast<int>(std::ceil((member.rect().xmax() - xmin)*m/(xmax-xmin)));
                int nmax = static_cast<int>(std::ceil((member.rect().ymax() - ymin)*m/(ymax-ymin)));
                // std::cout << "For particle with id " << member.id << ": \n";
                // std::cout << "mmin: " << mmin <<" mmax: " << mmax << '\n';
                // std::cout << "nmin: " << nmin <<" nmax: " << nmax << '\n';

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

            // for (auto& cells : partCells) {
            //     std::cout << "Member with id: " << cells.first->id << " is in cell: " << cells.second.first << " " << cells.second.second << '\n';
            // }
            
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

            // for (auto& group : partsGrouped) {
            //     std::cout << "This group contains particles with id: \n";
            //     for (auto& member : group) {
            //         std::cout << member->id << ", ";
            //     }
            //     std::cout << "\n";
            // }

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

        std::vector<std::pair<Part*, Part*>> 
};


class World { 
    private:
        std::vector<Part> members;
        std::vector<std::unique_ptr<Force>> forces;

    public: 
        double t;

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
};


int main() {
    Collision c;

    Part part1(1, 1, Vec2(0, -1), Vec2(0, 0), Vec2(0, 0), 1);
    Part part2(1, 1, Vec2(0.5, -1), Vec2(0, 0), Vec2(0, 0), 2);
    Part part3(1, 1, Vec2(0, 0), Vec2(0, 0), Vec2(0, 0), 3);
    Part part4(1, 1, Vec2(0.5, 2), Vec2(0, 0), Vec2(0, 0), 4);

    std::vector<Part> members{part1, part2, part3, part4};
    std::vector<std::pair<Part*, Part*>> pairs = c.USGCollision(members, -3, 3, -3, 3, 6, 6);

    for (auto& pair : pairs) {
        std::cout << pair.first->id << ' ' << pair.second->id << '\n';
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
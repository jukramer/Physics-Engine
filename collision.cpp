#include "collision.h"

void Collision::resolveCollision(std::vector<Part>& members) const {
    // Broad phase collision
    auto [partXmin, partXmax] = std::minmax_element(members.begin(), members.end(), 
                                            [] (const auto& a, const auto& b) {
                                                return a.pos.x < b.pos.x;
                                            });

    auto [partYmin, partYmax] = std::minmax_element(members.begin(), members.end(), 
                                            [] (const auto& a, const auto& b) {
                                                return a.pos.y < b.pos.y;
                                            });

    double xmin = std::floor(partXmin->rect().xmin()), xmax = std::floor(partXmax->rect().xmax());
    double ymin = std::floor(partYmin->rect().ymin()), ymax = std::floor(partYmax->rect().ymax());
    int m = xmax - xmin;
    int n = ymax - ymin;

    auto pairs = Collision::USGCollision(members, xmin, xmax, ymin, ymax, m, n);

    // Narrow phase collision
    auto collisions = Collision::circCollision(pairs);

    // Resolve collision, update velocities
    for (auto& collision : collisions) {
        Vec2 u1 = collision.pair.first->vel;
        Vec2 u2 = collision.pair.second->vel;
        double m1 = collision.pair.first->mass();
        double m2 = collision.pair.second->mass();
        Vec2 n = collision.sepDir.normal();

        Vec2 v1 = u1 - 2*m2/(m1+m2)*((u1-u2).dot(n))*n;
        Vec2 v2 = u2 + 2*m1/(m1+m2)*((u1-u2).dot(n))*n;

        collision.pair.first->vel = v2;
        collision.pair.second->vel = v1;
    }
}


std::vector<std::pair<Part*, Part*>> Collision::USGCollision(std::vector<Part>& members, double xmin, double xmax, double ymin, double ymax, int m, int n) const {
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


std::vector<CollisionResult> Collision::circCollision(std::vector<std::pair<Part*, Part*>>& pairs) const {
    std::vector<CollisionResult> result;

    for (auto& pair : pairs) {
        double d = sqrt(pow(pair.first->pos.x - pair.second->pos.x, 2) + pow(pair.first->pos.y + pair.second->pos.y, 2));

        if (d < pair.first->r() + pair.second->r()) {
            Vec2 collisionPoint = (pair.second->pos - pair.first->pos)/2 + pair.first->pos;
            double depth = pair.first->r() + pair.second->r() - d;
            Vec2 sepDir = (pair.second->pos - pair.first->pos).unit();

            result.push_back(CollisionResult(pair, true, collisionPoint, sepDir, depth));
        }
    }

    return result;
}

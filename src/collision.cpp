#include <algorithm>
#include "collision.h"


void Collision::resolveCollision(std::vector<Body>& members) const {
    // Unpack colliders
    std::vector<Collider> colliders;
    for (auto& member : members) {
        for (auto& collider : member.colliders()) {
            colliders.push_back(*collider);
        }
    }

    // Broad phase collision
    auto [partXmin, partXmax] = std::minmax_element(colliders.begin(), colliders.end(), 
                                            [] (const auto& a, const auto& b) {
                                                return a.pos.x < b.pos.x;
                                            });

    auto [partYmin, partYmax] = std::minmax_element(colliders.begin(), colliders.end(), 
                                            [] (const auto& a, const auto& b) {
                                                return a.pos.y < b.pos.y;
                                            });

    double xmin = std::floor(partXmin->xmin()), xmax = std::floor(partXmax->xmax());
    double ymin = std::floor(partYmin->ymin()), ymax = std::floor(partYmax->ymax());
    int m = xmax - xmin;
    int n = ymax - ymin;

    auto pairs = Collision::USGCollision(colliders, xmin, xmax, ymin, ymax, m, n);

    // TODO: Logic for different collider types
    // Narrow phase collision 

    // TODO: move to separate function
    // Resolve collision, update velocities
}


std::vector<std::pair<Collider*, Collider*>> Collision::USGCollision(std::vector<Collider>& colliders, double xmin, double xmax, double ymin, double ymax, int m, int n) const {
    std::vector<std::pair<Collider*, std::pair<int,int>>> partCells;
    partCells.reserve(colliders.size());

    // Find cells covered by each particle
    for (auto& collider : colliders) {
        // Min and max cell indices covered by particle bounding box
        int mmin = static_cast<int>((collider.xmin() - xmin)*m/(xmax-xmin));
        int nmin = static_cast<int>((collider.ymin() - ymin)*m/(ymax-ymin));
        int mmax = static_cast<int>(std::ceil((collider.xmax() - xmin)*m/(xmax-xmin)));
        int nmax = static_cast<int>(std::ceil((collider.ymax() - ymin)*m/(ymax-ymin)));

        // Store each cell covered by particle bounding box
        for (int i = mmin; i < mmax+1; i++) {
            for (int j = nmin; j < nmax+1; j++) {
                partCells.push_back(std::make_pair(&collider, std::make_pair(i, j)));
            }
        }
    }

    // Sort by cell indices (adjacent cells together)
    std::sort(partCells.begin(), partCells.end(), [] (const auto& a, const auto& b) { return a.second < b.second; });
    
    // Group particles of same cell into vectors, store in outside vector
    std::vector<std::vector<Collider*>> partsGrouped;
    std::vector<Collider*> group;
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
    std::vector<std::pair<Collider*, Collider*>> partsPaired;
    
    for (auto& group : partsGrouped) {
        std::vector<bool> mask(group.size());
        fill(mask.begin(), mask.begin() + 2, true);
        do {
            std::vector<Collider*> pairVec;
            for (int i=0; i<mask.size(); i++) {
                if (mask[i]) {
                    pairVec.push_back(group[i]);
                }
            } 
            sort(pairVec.begin(), pairVec.end(), [] (const auto& a, const auto& b) { return a->id < b->id; });
            std::pair<Collider*, Collider*> pair(pairVec[0], pairVec[1]);
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


std::vector<CollisionResult> Collision::circCollision(std::vector<std::pair<PartCollider*, PartCollider*>>& pairs) const {
    std::vector<CollisionResult> result;

    for (auto& pair : pairs) {
        double d = sqrt(pow(pair.first->pos.x - pair.second->pos.x, 2) + pow(pair.first->pos.y - pair.second->pos.y, 2));

        if (d < pair.first->r() + pair.second.owner()->r()) {
            Vec2 collisionPoint = (pair.second->pos - pair.first->pos)/2 + pair.first->pos;
            double depth = pair.first->r() + pair.second->r() - d;
            Vec2 sepDir = (pair.second->pos - pair.first->pos).unit();

            result.push_back(CollisionResult(pair, true, collisionPoint, sepDir, depth));
        }
    }

    return result;
}


void Collision::updateMembers(std::vector<CollisionResult>& collisions) {

}
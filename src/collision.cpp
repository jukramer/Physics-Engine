#include <algorithm>
#include <limits>
#include "collision.h"


void Collision::resolveCollision(const std::vector<Body>& members) const {
    // Unpack colliders
    std::vector<Collider*> colliders;
    for (auto& member : members) {
        for (auto& collider : member.colliders()) {
            colliders.push_back(collider.get());
        }
    }

    // Broad phase collision
    auto [partXmin, partXmax] = std::minmax_element(colliders.begin(), colliders.end(), 
                                            [] (const auto& a, const auto& b) {
                                                return a->pos.x < b->pos.x;
                                            });

    auto [partYmin, partYmax] = std::minmax_element(colliders.begin(), colliders.end(), 
                                            [] (const auto& a, const auto& b) {
                                                return a->pos.y < b->pos.y;
                                            });

    const double xmin = std::floor((*partXmin)->xmin()), xmax = std::floor((*partXmax)->xmax());
    const double ymin = std::floor((*partYmin)->ymin()), ymax = std::floor((*partYmax)->ymax());
    const int m = xmax - xmin;
    const int n = ymax - ymin;

    auto pairs = Collision::USGCollision(colliders, xmin, xmax, ymin, ymax, m, n);

    // TODO: Logic for different collider types
    // Narrow phase collision

    // TODO: move to separate function
    // Resolve collision, update velocities
}

std::vector<std::pair<Collider*, Collider*>> Collision::USGCollision(std::vector<Collider*>& colliders, double xmin, double xmax, double ymin, double ymax, int m, int n) const {
    std::vector<std::pair<Collider*, std::pair<int,int>>> partCells;
    partCells.reserve(colliders.size());

    // Find cells covered by each particle
    for (auto& collider : colliders) {
        // Min and max cell indices covered by particle bounding box
        const int mmin = static_cast<int>((collider->xmin() - xmin)*m/(xmax-xmin));
        const int nmin = static_cast<int>((collider->ymin() - ymin)*n/(ymax-ymin));
        const int mmax = static_cast<int>(std::ceil((collider->xmax() - xmin)*m/(xmax-xmin)));
        const int nmax = static_cast<int>(std::ceil((collider->ymax() - ymin)*n/(ymax-ymin)));

        // Store each cell covered by particle bounding box
        for (int i = mmin; i < mmax+1; i++) {
            for (int j = nmin; j < nmax+1; j++) {
                partCells.push_back(std::make_pair(collider, std::make_pair(i, j)));
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
        std::fill(mask.begin(), mask.begin() + 2, true);
        do {
            std::vector<Collider*> pairVec;
            for (int i=0; i<mask.size(); i++) {
                if (mask[i]) {
                    pairVec.push_back(group[i]);
                }
            } 
            std::sort(pairVec.begin(), pairVec.end(), [] (const auto& a, const auto& b) { return a->id < b->id; });
            std::pair<Collider*, Collider*> pair(pairVec[0], pairVec[1]);
            partsPaired.push_back(pair);
        } while (std::next_permutation(mask.begin(), mask.end()));
    }

    // Remove duplicates
    std::sort(partsPaired.begin(), partsPaired.end(),
        [] (const auto& a, const auto& b) { 
            if (a.first->id != b.first->id) {
                return a.first->id < b.first->id;
            }
            return a.second->id < b.second->id;
        }
    );
    
    const auto last = std::unique(partsPaired.begin(), partsPaired.end());
    partsPaired.erase(last, partsPaired.end());

    return partsPaired;
}

std::vector<CollisionResult> Collision::circCollision(const std::vector<std::pair<PartCollider*, PartCollider*>>& pairs) const {
    std::vector<CollisionResult> result;

    for (auto& pair : pairs) {
        const double d = sqrt(pow(pair.first->pos.x - pair.second->pos.x, 2) + pow(pair.first->pos.y - pair.second->pos.y, 2));

        if (d < pair.first->r + pair.second->r) {
            const Vec2 collisionPoint = (pair.second->pos - pair.first->pos)/2 + pair.first->pos;
            const double depth = pair.first->r + pair.second->r - d;
            const Vec2 sepDir = (pair.second->pos - pair.first->pos).unit();

            result.push_back(CollisionResult(pair, true, std::vector<Vec2>{collisionPoint}, sepDir, depth));
        }
    }

    return result;
}

std::vector<CollisionResult> Collision::SATCollision(const std::vector<std::pair<PolygonCollider*, PolygonCollider*>>& pairs) const {
    std::vector<CollisionResult> result;
    // constexpr auto inf = std::numeric_limits<double>::infinity();

    for (const auto& pair : pairs) {
        const std::vector<Vec2> axes1 = pair.first->edgeNormals();
        const std::vector<Vec2> axes2 = pair.second->edgeNormals();

        std::vector<Vec2> axes = axes1;
        axes.insert(axes.end(), axes2.begin(), axes2.end());

        double dmin = inf; // Minimum penetration distance
        Vec2 sepDir = Vec2(inf, inf);

        for (const auto& axis : axes) {
            // Projection intervals
            double amin = inf, amax = -inf;
            double bmin = inf, bmax = -inf;

            for (const auto& vert : pair.first->worldVerts()) {
                const double p = vert.dot(axis);
                if (p < amin) {
                    amin = p;
                }
                if (p > amax) {
                    amax = p;
                }
            }

            for (const auto& vert : pair.second->worldVerts()) {
                const double p = vert.dot(axis);
                if (p < bmin) {
                    bmin = p;
                }
                if (p > bmax) {
                    bmax = p;
                }
            }

            const double d = std::min(amax, bmax) - std::max(amin, bmin);
            if (d < dmin) {
                dmin = d;
                sepDir = axis;
            }
        }

        // Colliding if dmin >= 0
        // TODO: Placeholder for collisionpoint, potentially not needed
        if (dmin >= 0) {
            result.push_back(CollisionResult(pair, true, std::vector<Vec2>{Vec2(0,0)}, sepDir, dmin));
        }
    }

    return result;
}

std::vector<CollisionResult> Collision::findContactManifolds(std::vector<CollisionResult>& results) const {
    auto resultsCopy = results;
    for (auto& result : resultsCopy) {
        if (!(dynamic_cast<PolygonCollider*>(result.pair.first) && dynamic_cast<PolygonCollider*>(result.pair.second))) {
            throw std::logic_error("Function can be used only for type PolygonCollider.");
        }

        auto first = dynamic_cast<PolygonCollider*>(result.pair.first);
        auto second = dynamic_cast<PolygonCollider*>(result.pair.second);

        // Align collision normal with A->B
        Vec2 dirAB = second->pos - first->pos;
        Vec2 collisionNormal = result.sepDir.normal();

        if (collisionNormal.dot(dirAB) < 0) {
            collisionNormal = -collisionNormal;
        }

        // Find ref and incident edges
        auto firstWorldVerts = first->verts();
        auto secondWorldVerts = second->verts();
        Vec2 v1, v2; // Reference edge endpoints
        Vec2 u1(0, 0), u2(0, 0); // Incident edge endpoints
        Vec2 refNormal = -collisionNormal; // init values for finding argmax/argmin(nr/nc.n)
        Vec2 incNormal = collisionNormal;

        // Ref
        for (int i = 0; i < firstWorldVerts.size(); i++) {
            const Vec2 edge = firstWorldVerts[(i+1) % firstWorldVerts.size()] - firstWorldVerts[i];
            if (edge.normal().dot(collisionNormal) >= refNormal.dot(collisionNormal)) {
                refNormal = edge.normal();
                v1 = firstWorldVerts[i];
                v2 = firstWorldVerts[(i+1) % firstWorldVerts.size()];
            }
        }

        // Incident
        for (int i = 0; i < secondWorldVerts.size(); i++) {
            const Vec2 edge = secondWorldVerts[(i+1) % secondWorldVerts.size()] - secondWorldVerts[i];
            if (edge.normal().dot(collisionNormal) <= incNormal.dot(collisionNormal)) {
                incNormal = edge.normal();
                u1 = secondWorldVerts[i];
                u2 = secondWorldVerts[(i+1) % secondWorldVerts.size()];
            }
        }

        // Clip incident edge
        // TODO: revisit
        Vec2 t = (v2-v1)/(v2-v1).norm(); // ref edge unit tangent
        double lambda1 = (v1-u1).dot(t)/(u2-u1).dot(t);
        double lambda2 = (v2-u1).dot(-t)/(u2-u1).dot(-t);

        double s1 = (u1 - v1).dot(t);
        double s2 = (u2 - v2).dot(-t);

        if (s1 < 0) {
            u1 = u1 + lambda1*(u2-u1);
        }
        if (s2 < 0) {
            u2 = u2 + lambda2*(u2-u1);
        }

        // Separation
        double sep1 = (u1 - v1).dot(t.normal());
        double sep2 = (u2 - v2).dot(t.normal());

        if (sep1 <= 0) {
            result.collisionPoints.push_back(u1);
        }

        if (sep2 <= 0) {
            result.collisionPoints.push_back(u2);
        }
    }

    return resultsCopy;
}

void Collision::updateMembers(std::vector<CollisionResult>& collisions) {

}
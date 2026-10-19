#pragma once
#include <utility>
#include <vector>
#include "bodies.h"

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
        void resolveCollision(std::vector<Part>& members) const;

    private:
        // Broad Phase Collision
        std::vector<std::pair<Part*, Part*>> USGCollision(std::vector<Part>& members, double xmin, double xmax, double ymin, double ymax, int m, int n) const;
            
        // Narrow Phase Collision
        std::vector<CollisionResult> circCollision(std::vector<std::pair<Part*, Part*>>& pairs) const;

        // std::vector<CollisionResult> SATCollision(std::vector<std::pair<Part*, Part*>>& pairs) const {

        // }
};
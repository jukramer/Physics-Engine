#pragma once
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>
#include "collider.h"
#include "common.h"


struct CollisionResult {
    std::pair<Collider*, Collider*> pair;
    bool colliding;
    Vec2 collisionPoint;
    Vec2 sepDir;
    double depth;

    CollisionResult(const std::pair<Collider*, Collider*>& pair, const bool colliding = false, const Vec2 collisionPoint = Vec2(0,0), const Vec2 sepDir = Vec2(0,0), const double depth = 0) : pair(pair), colliding(colliding), collisionPoint(collisionPoint), sepDir(sepDir), depth(depth) {}
    CollisionResult(const std::pair<PartCollider*, PartCollider*>& pair, const bool colliding = false, const Vec2 collisionPoint = Vec2(0,0), const Vec2 sepDir = Vec2(0,0), const double depth = 0) : pair(pair), colliding(colliding), collisionPoint(collisionPoint), sepDir(sepDir), depth(depth) {}
    CollisionResult(const std::pair<PolygonCollider*, PolygonCollider*>& pair, const bool colliding = false, const Vec2 collisionPoint = Vec2(0,0), const Vec2 sepDir = Vec2(0,0), const double depth = 0) : pair(pair), colliding(colliding), collisionPoint(collisionPoint), sepDir(sepDir), depth(depth) {}
};


// COLLISION ENGINE
class Collision {
    public:
        Collision() = default;

        // Global collision
        void resolveCollision(const std::vector<Body>& members) const;

        // Broad Phase Collision
        std::vector<std::pair<Collider*, Collider*>> USGCollision(std::vector<Collider*>& colliders, double xmin, double xmax, double ymin, double ymax, int m, int n) const;
            
        // Narrow Phase Collision
        std::vector<CollisionResult> circCollision(const std::vector<std::pair<PartCollider*, PartCollider*>>& pairs) const;
        std::vector<CollisionResult> SATCollision(const std::vector<std::pair<PolygonCollider*, PolygonCollider*>>& pairs) const;

        void updateMembers(std::vector<CollisionResult>& collisions); 
};
#pragma once
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>
#include "collider.h"
#include "common.h"


// COLLISION ENGINE
class Collision {
    public:
        Collision() = default;

        // Global collision
        void resolveCollision(const std::vector<Body>& members) const;

    private:
        // Broad Phase Collision
        std::vector<std::pair<Collider*, Collider*>> USGCollision(std::vector<Collider*>& colliders, double xmin, double xmax, double ymin, double ymax, int m, int n) const;
            
        // Narrow Phase Collision
        std::vector<CollisionResult> circCollision(std::vector<std::pair<PartCollider*, PartCollider*>>& pairs) const;

        // std::vector<CollisionResult> SATCollision(std::vector<std::pair<Part*, Part*>>& pairs) const {

        // }

        void updateMembers(std::vector<CollisionResult>& collisions); 
};
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


class Collider {
    private:
        Vec2 pos_;
        std::vector<Vec2> verts_;
        double xmin_;
        double xmax_;
        double ymin_;
        double ymax_;

    public:
        Collider(Vec2 pos, std::vector<Vec2> verts) :
                pos_(pos), 
                verts_(verts), 
                xmin_(std::min_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.x < b.x; })->x), 
                xmax_(std::max_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.x < b.x; })->x),
                ymin_(std::min_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.y < b.y; })->y),
                ymax_(std::max_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.y < b.y; })->y)
        {} 
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
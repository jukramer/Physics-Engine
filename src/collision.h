#pragma once
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>
#include "common.h"

// TYPES
enum ShapeType {
    Circle,
    Polygon,
    Rectangle
};


class Collider;
class Body;
class Part;


struct CollisionResult {
    std::pair<Collider*, Collider*> pair;
    bool colliding;
    Vec2 collisionPoint;
    Vec2 sepDir;
    double depth;

    CollisionResult(std::pair<Collider*, Collider*>& pair, bool colliding = false, Vec2 collisionPoint = Vec2(0,0), Vec2 sepDir = Vec2(0,0), double depth = 0) : pair(pair), colliding(colliding), collisionPoint(collisionPoint), sepDir(sepDir), depth(depth) {}
};


// COLLIDERS
class Collider {
    protected:
        bool ownerAdded;

    public:
        const ShapeType shape;
        Vec2 pos;
        Body* owner_;
        const int id;

        Collider(const ShapeType shape, const Vec2& pos) : ownerAdded(false), shape(shape), pos(pos), owner_(nullptr), id(std::rand()) {}
        virtual ~Collider() = default;

        Body* owner() const {
            if (!ownerAdded) {
                throw std::runtime_error("Must add owning rigid body to collider.");
            }
            return owner_;
        }

        virtual double xmin() const = 0;
        virtual double xmax() const = 0;
        virtual double ymin() const = 0;
        virtual double ymax() const = 0;

        void setOwner(Body* owner) {
            owner_ = owner;
            ownerAdded = true;
        };
};


class PartCollider : public Collider {
    public:
        double w, h;

        PartCollider(const Vec2& pos, double w, double h) : Collider(Rectangle, pos), w(w), h(h) {};
        double xmin() const override { return pos.x - w/2; }
        double xmax() const override { return pos.x + w/2; }
        double ymin() const override { return pos.y - h/2; }
        double ymax() const override { return pos.y + h/2; }
};


class PolygonCollider : public Collider {
    private:
        std::vector<Vec2> verts_; // Relative to centroid

    public:
        Vec2 offset; // Offset to owner rigid body centroid

        PolygonCollider(const Vec2& pos, const Vec2& offset, std::vector<Vec2>& verts) : Collider(Polygon, pos), verts_(initVerts(verts)), ownerAdded(false), offset(offset) {}


        double xmin() const override { return std::min_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.x < b.x; })->x; }
        double xmax() const override { return std::max_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.x < b.x; })->x; }
        double ymin() const override { return std::min_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.y < b.y; })->y; }
        double ymax() const override { return std::max_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.y < b.y; })->y; }
        std::vector<Vec2> verts() const { return verts_; }

        std::vector<Vec2> worldVerts() const {
            std::vector<Vec2> result = verts_;
            result.reserve(verts_.size());

            for (auto& vert : result) {
                vert += pos;
            }

            return result;
        }

        Vec2 calcCentroid(std::vector<Vec2>& verts) const { return std::accumulate(verts.begin(), verts.end(), Vec2(0,0)) / verts.size(); }

        std::vector<Vec2> initVerts(std::vector<Vec2>& verts) {
            Vec2 centroid = calcCentroid(verts);
            for (auto& vert : verts) {
                vert -= centroid;
            }

            return verts;
        }
};

// COLLISION ENGINE
class Collision {
    public:
        Collision() = default;

        // Global collision
        void resolveCollision(std::vector<Body>& members) const;

    private:
        // Broad Phase Collision
        std::vector<std::pair<Collider*, Collider*>> USGCollision(std::vector<Collider>& colliders, double xmin, double xmax, double ymin, double ymax, int m, int n) const;
            
        // Narrow Phase Collision
        std::vector<CollisionResult> circCollision(std::vector<std::pair<PartCollider*, PartCollider*>>& pairs) const;

        // std::vector<CollisionResult> SATCollision(std::vector<std::pair<Part*, Part*>>& pairs) const {

        // }

        void updateMembers(std::vector<CollisionResult>& collisions); 
};
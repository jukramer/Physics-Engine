#pragma once
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include "bodies.h"
#include "common.h"


// TYPES
enum ShapeType {
    Circle,
    Polygon,
    Rectangle
};


class Collider;
class Body;
class RigidBody;
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
    public:
        Collider(const ShapeType shape, const Vec2& pos) : shape(shape), pos(pos), id(std::rand()), ownerAdded(false) {}
        virtual ~Collider() = default;

        const ShapeType shape;
        Vec2 pos;
        const int id;

        virtual Body* owner() const = 0;
        virtual double xmin() const = 0;
        virtual double xmax() const = 0;
        virtual double ymin() const = 0;
        virtual double ymax() const = 0;

        // virtual void setOwner(Body* owner) const = 0;

    protected:
        bool ownerAdded;
};


class PartCollider : public Collider {
    public:
        PartCollider(const Vec2& pos, const double r) : Collider(Rectangle, pos), r(r), owner_(nullptr) {};

        double r;

        double xmin() const override { return pos.x - r; }
        double xmax() const override { return pos.x + r; }
        double ymin() const override { return pos.y - r; }
        double ymax() const override { return pos.y + r; }

        Part* owner() const override {
            if (!ownerAdded) {
                throw std::runtime_error("Must add owning rigid body to collider.");
            }
            return owner_;
        }

        void setOwner(Part* owner) { owner_ = owner; }

    private:
        Part* owner_;
};


class PolygonCollider : public Collider {
    public:
        using vecVec2 = std::vector<Vec2>;
        PolygonCollider(const Vec2& pos, const Vec2& offset, vecVec2& verts) : Collider(Polygon, pos), offset(offset), verts_(initVerts(verts)), owner_(nullptr) {}

        double xmin() const override { return std::min_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.x < b.x; })->x; }
        double xmax() const override { return std::max_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.x < b.x; })->x; }
        double ymin() const override { return std::min_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.y < b.y; })->y; }
        double ymax() const override { return std::max_element(verts_.begin(), verts_.end(), [] (const auto& a, const auto& b) { return a.y < b.y; })->y; }
        std::vector<Vec2> verts() const { return verts_; }

        Body* owner() const override { return owner_; }
        void setOwner(RigidBody* owner) { owner_ = owner; }

        Vec2 offset; // Offset to owner rigid body centroid

        vecVec2 worldVerts() const {
            vecVec2 result = verts_;
            result.reserve(verts_.size());

            for (auto& vert : result) {
                vert += pos;
            }

            return result;
        }

    private:
        vecVec2 verts_; // Relative to centroid
        RigidBody* owner_;

        Vec2 calcCentroid(vecVec2& verts) const { return std::accumulate(verts.begin(), verts.end(), Vec2(0,0)) / verts.size(); }

        vecVec2 initVerts(vecVec2& verts) {
            Vec2 centroid = calcCentroid(verts);
            for (auto& vert : verts) {
                vert -= centroid;
            }

            return verts;
        }
};




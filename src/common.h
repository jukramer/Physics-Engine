#pragma once
#include <cmath>

namespace Physics {
    constexpr double dt = 1e-3;
    constexpr double elementalCharge = 1.602e-19;
    constexpr double gravAcc = 9.80665;
    constexpr double gravConst = 6.674e-11;
}


struct Vec2 {
    public: 
        Vec2(const double x, const double y) : x(x), y(y) {}
        Vec2() : x(0), y(0) {}

        Vec2 normal() const {
            // Always returns right hand normal
            return Vec2(y, -x);
        }

        Vec2 unit() const { 
            const double mag = norm();
        
            if (mag == 0) {
                return Vec2(0, 0);
            }

            return Vec2(x/mag, y/mag);
        }

        // Angle in radians, CCW+
        Vec2 rotate(const double angle) const { return Vec2(x*std::cos(angle) - y*std::sin(angle), x*std::sin(angle) + y*std::cos(angle)); }

        Vec2 reflect(const Vec2& other) const;
        double norm() const { return std::sqrt(x*x + y*y); }
        double dot(const Vec2& other) const { return x*other.x + y*other.y; }

        // Vec-vec operators
        Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
        Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
        Vec2& operator+=(const Vec2& other) { x += other.x; y += other.y; return *this; }
        Vec2& operator-=(const Vec2& other) { x -= other.x; y -= other.y; return *this; }
        bool operator==(const Vec2& other) const { return (x == other.x) && (y == other.y); }

        // Vec-scalar operators
        Vec2 operator+(const double s) const { return Vec2(x + s, y + s); }
        Vec2 operator-(const double s) const { return Vec2(x - s, y - s); }
        Vec2& operator+=(const double s) { x += s; y += s; return *this; }
        Vec2& operator-=(const double s) { x -= s; y -= s; return *this; }
        Vec2 operator*(const double s) const { return Vec2(s*x, s*y); }
        Vec2 operator/(const double s) const { return Vec2(x/s, y/s); }
        Vec2& operator*=(const double s) { x *= s; y *= s; return *this; }
        Vec2& operator/=(const double s) { x /= s; y /= s; return *this; }

        // Unary operators
        Vec2 operator-() const { return Vec2(-x, -y); }

        double x;
        double y;
};

// Additional vector operators
inline Vec2 operator*(const double s, const Vec2& vec) { return Vec2(s*vec.x, s*vec.y); }

inline Vec2 Vec2::reflect(const Vec2& other) const {
    const Vec2 proj = dot(other)/(other.norm()*other.norm())*other;
    return 2*proj - *this;
}
#pragma once
#include <cmath>


namespace Physics {
    constexpr double dt = 1e-2;
    constexpr double e = 1.602e-19;
    constexpr double g = 9.80665;
    constexpr double G = 6.674e-11;
}


class Vec2 {
    public: 
        double x;
        double y;

        Vec2(double x, double y) : x(x), y(y) {}

        double norm() const { return sqrt(x*x + y*y); }

        Vec2 unit() const { 
            double mag = norm();
        
            if (mag == 0) {
                return Vec2(0, 0);
            }

            return Vec2(x/mag, y/mag);
        }   

        // Vec-vec operators
        Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
        Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
        void operator+=(const Vec2& other) { x += other.x; y += other.y;}
        void operator-=(const Vec2& other) { x += other.x; y += other.y;}
        bool operator==(const Vec2& other) const { return (x == other.x) && (y == other.y); }

        // Vec-scalar operators
        Vec2 operator+(const double s) const { return Vec2(x + s, y + s); }
        Vec2 operator-(const double s) const { return Vec2(x - s, y - s); }
        void operator+=(const double s) { x += s; y += s;}
        void operator-=(const double s) { x += s; y += s;}
        Vec2 operator*(const double s) const { return Vec2(s*x, s*y); }
        Vec2 operator/(const double s) const { return Vec2(x/s, y/s); }
        void operator*=(const double s) { x *= s; y *= s;}
        void operator/=(const double s) { x /= s; y /= s;}

        // Unary operators
        Vec2 operator-() const { return Vec2(-x, -y); }
};

// Additional vector operators
inline Vec2 operator*(double s, const Vec2& vec) { return Vec2(s*vec.x, s*vec.y); }
inline Vec2 operator/(double s, const Vec2& vec) { return Vec2(vec.x/s, vec.y/s); }


class Rect {
    private: 
        Vec2 pos_;
        double w, h;
        double xmin_, xmax_;
        double ymin_, ymax_;
    
    public:
        Rect(Vec2 pos, double w, double h) : pos_(pos), w(w), h(h), xmin_(pos.x-w/2), xmax_(pos.x+w/2), ymin_(pos.y-h/2), ymax_(pos.y+h/2) {};
        
        void setPos(Vec2 pos) {
            pos_ = pos;
            xmin_ = pos.x - w/2;
            xmax_ = pos.x + w/2;
            ymin_ = pos.y - h/2;
            ymax_ = pos.y + h/2;
        }

        void setw(double w) {
            xmin_ = pos_.x - w/2;
            xmax_ = pos_.x + w/2;
        }

        void seth(double h) {
            ymin_ = pos_.y - h/2;
            ymax_ = pos_.y + h/2;
        }

        Vec2 pos() const { return pos_; }
        double xmin() const { return xmin_; }
        double xmax() const { return xmax_; }
        double ymin() const { return ymin_; }
        double ymax() const { return ymax_; }
};


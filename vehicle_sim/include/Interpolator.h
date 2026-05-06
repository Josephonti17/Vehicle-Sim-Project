// Interpolator.h
// Generic linear interpolator over a sorted sequence of (x, y) points.
// Used for engine torque curves and potentially other tabulated data.
//
// Template parameter T is the numeric type (double by default).
// Points are stored internally as std::vector and kept sorted by x.
#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <utility>

template <typename T = double>
class Interpolator {
public:
    Interpolator() = default;

    // Add a single (x, y) data point. Keeps the internal vector sorted by x.
    void addPoint(T x, T y) {
        Point p{x, y};
        // Insert in sorted position using std::lower_bound (generic algorithm).
        auto it = std::lower_bound(points_.begin(), points_.end(), p,
            [](const Point& a, const Point& b) { return a.x < b.x; });
        points_.insert(it, p);
    }

    // Evaluate the interpolated y value at x.
    // Extrapolates flat beyond the first/last point (no unrealistic overshoot).
    T evaluate(T x) const {
        if (points_.empty()) {
            throw std::runtime_error("Interpolator: no data points");
        }
        if (points_.size() == 1) return points_.front().y;
        if (x <= points_.front().x) return points_.front().y;
        if (x >= points_.back().x)  return points_.back().y;

        // Binary search for the first point with x >= query.
        Point key{x, T{}};
        auto it = std::lower_bound(points_.begin(), points_.end(), key,
            [](const Point& a, const Point& b) { return a.x < b.x; });

        const Point& hi = *it;
        const Point& lo = *(it - 1);
        T t = (x - lo.x) / (hi.x - lo.x);
        return lo.y + t * (hi.y - lo.y);
    }

    std::size_t size() const { return points_.size(); }
    bool empty() const { return points_.empty(); }

    // Find the x at which y is maximized (useful for torque peak).
    T argMax() const {
        if (points_.empty()) throw std::runtime_error("Interpolator: empty");
        auto it = std::max_element(points_.begin(), points_.end(),
            [](const Point& a, const Point& b) { return a.y < b.y; });
        return it->x;
    }

    T maxValue() const {
        if (points_.empty()) throw std::runtime_error("Interpolator: empty");
        auto it = std::max_element(points_.begin(), points_.end(),
            [](const Point& a, const Point& b) { return a.y < b.y; });
        return it->y;
    }

private:
    struct Point {
        T x;
        T y;
    };
    std::vector<Point> points_;
};

#endif // INTERPOLATOR_H

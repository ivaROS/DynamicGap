#pragma once
#include <Eigen/Core>
#include <vector>
#include <cmath>

// Quadratic Bézier interpolation with Eigen vectors
inline Eigen::Vector2f bezier(const Eigen::Vector2f& p0,
                              const Eigen::Vector2f& p1,
                              const Eigen::Vector2f& p2,
                              float t) {
    float u = 1.0f - t;
    return u*u*p0 + 2*u*t*p1 + t*t*p2;
}

// Composite Bézier: two quadratics joined at p2
inline std::vector<Eigen::Vector2f> compositeBezier(const Eigen::Vector2f& p0,
                                                    const Eigen::Vector2f& p2,
                                                    const Eigen::Vector2f& p4,
                                                    float min_scan_dist,
                                                    const Eigen::Vector2f& v_dir,
                                                    int num_samples = 100) {
    std::vector<Eigen::Vector2f> curve;

    Eigen::Vector2f p1 = p0 + (min_scan_dist / 2.0f) * v_dir;

    // First quadratic: p0 -> p1 -> p2
    for (int i = 0; i <= num_samples / 2; i++) {
        float t = static_cast<float>(i) / (num_samples / 2);
        curve.push_back(bezier(p0, p1, p2, t));
    }

    // Second quadratic: p2 -> p3 -> p4
    Eigen::Vector2f p3 = p2 + ((p4 - p2).norm() / 2.0f) * ((p2 - p1).normalized());
    for (int i = 1; i <= num_samples / 2; i++) {
        float t = static_cast<float>(i) / (num_samples / 2);
        curve.push_back(bezier(p2, p3, p4, t));
    }

    return curve;
}

// Project a goal point onto a circle 
inline Eigen::Vector2f projectOntoCircle(const Eigen::Vector2f& goal, float radius) {
    if (goal.norm() < 1e-6)
        return Eigen::Vector2f(radius, 0.0f);

    return radius * goal.normalized();
}

/**
 * @file vector3.hpp
 * @brief Simple 3D vector math.
 */

#ifndef RTK_ENGINE_VECTOR3_HPP
#define RTK_ENGINE_VECTOR3_HPP

#include <cmath>

namespace rtk {

/**
 * @brief Representation of a 3D vector (x, y, z).
 */
struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vector3() = default;
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3& o) const { return Vector3(x + o.x, y + o.y, z + o.z); }
    Vector3 operator-(const Vector3& o) const { return Vector3(x - o.x, y - o.y, z - o.z); }
    Vector3 operator*(double s) const { return Vector3(x * s, y * s, z * s); }
    
    /** @brief Vector dot product. */
    double dot(const Vector3& o) const { return x * o.x + y * o.y + z * o.z; }
    
    /** @brief Euclidean norm (magnitude). */
    double norm() const { return std::sqrt(x * x + y * y + z * z); }
    
    /** @brief Returns a unit-length version of the vector. */
    Vector3 normalized() const {
        double n = norm();
        if (n < 1e-12) return Vector3(0, 0, 0);
        return Vector3(x / n, y / n, z / n);
    }
};

} // namespace rtk

#endif // RTK_ENGINE_VECTOR3_HPP

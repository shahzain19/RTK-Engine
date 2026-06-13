/**
 * @file matrix.hpp
 * @brief Fixed-size matrix math templates.
 */

#ifndef RTK_ENGINE_MATRIX_HPP
#define RTK_ENGINE_MATRIX_HPP

#include "rtk_engine/math/vector3.hpp"
#include <cmath>
#include <algorithm>

namespace rtk {

/**
 * @brief Optimized 3x3 matrix for coordinate transformations.
 */
struct Matrix3x3 {
    double m[3][3] = {{0.0}};

    Matrix3x3() = default;

    /** @brief Matrix-vector multiplication. */
    Vector3 operator*(const Vector3& v) const {
        return Vector3(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }

    /** @brief Direct inversion using Cramer's rule. */
    Matrix3x3 inverse() const {
        double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                     m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                     m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        
        Matrix3x3 inv;
        if (std::abs(det) < 1e-15) {
            return inv; // Singularity
        }

        double inv_det = 1.0 / det;
        inv.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
        inv.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
        inv.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;

        inv.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
        inv.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
        inv.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;

        inv.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
        inv.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
        inv.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;

        return inv;
    }
};

/**
 * @brief Generic fixed-size matrix template.
 * @tparam R Number of rows.
 * @tparam C Number of columns.
 */
template<int R, int C>
struct Matrix {
    double data[R][C] = {{0.0}};

    Matrix() = default;

    /** @brief Matrix transposition. */
    Matrix<C, R> transpose() const {
        Matrix<C, R> out;
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                out.data[j][i] = data[i][j];
            }
        }
        return out;
    }

    /** @brief Matrix-matrix multiplication. */
    template<int C2>
    Matrix<R, C2> operator*(const Matrix<C, C2>& o) const {
        Matrix<R, C2> out;
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C2; ++j) {
                double sum = 0.0;
                for (int k = 0; k < C; ++k) {
                    sum += data[i][k] * o.data[k][j];
                }
                out.data[i][j] = sum;
            }
        }
        return out;
    }
};

/**
 * @brief Helper for matrix inversion using Gauss-Jordan elimination.
 */
template<int N>
struct MatrixInv {
    /** @brief Inverts a square matrix. */
    static Matrix<N, N> invert(const Matrix<N, N>& m) {
        Matrix<N, N> inv;
        double temp[N][2 * N];
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                temp[i][j] = m.data[i][j];
                temp[i][j + N] = (i == j) ? 1.0 : 0.0;
            }
        }

        // Gauss-Jordan elimination with partial pivoting
        for (int i = 0; i < N; ++i) {
            double max_val = std::abs(temp[i][i]);
            int pivot_row = i;
            for (int k = i + 1; k < N; ++k) {
                if (std::abs(temp[k][i]) > max_val) {
                    max_val = std::abs(temp[k][i]);
                    pivot_row = k;
                }
            }

            if (max_val < 1e-15) {
                return inv; // Singularity
            }

            if (pivot_row != i) {
                for (int j = 0; j < 2 * N; ++j) {
                    std::swap(temp[i][j], temp[pivot_row][j]);
                }
            }

            double pivot = temp[i][i];
            for (int j = i; j < 2 * N; ++j) {
                temp[i][j] /= pivot;
            }

            for (int k = 0; k < N; ++k) {
                if (k != i) {
                    double factor = temp[k][i];
                    for (int j = i; j < 2 * N; ++j) {
                        temp[k][j] -= factor * temp[i][j];
                    }
                }
            }
        }

        // Extract right side (the inverse)
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                inv.data[i][j] = temp[i][j + N];
            }
        }
        return inv;
    }
};

} // namespace rtk

#endif // RTK_ENGINE_MATRIX_HPP

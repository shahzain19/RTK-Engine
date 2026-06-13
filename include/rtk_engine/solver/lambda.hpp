/**
 * @file lambda.hpp
 * @brief LAMBDA (Least-squares AMBiguity Decorrelation Adjustment) implementation.
 */

#ifndef RTK_ENGINE_LAMBDA_HPP
#define RTK_ENGINE_LAMBDA_HPP

#include "rtk_engine/common.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace rtk {

/**
 * @brief Algorithms for integer ambiguity resolution.
 * @details Implements decorrelation and discrete search in the ambiguity space.
 */
class Lambda {
public:
    /** @brief Performs LDLt decomposition on a square symmetric matrix. */
    static void ldltDecompose(const std::vector<std::vector<double>>& Q, std::vector<std::vector<double>>& L, std::vector<double>& D) {
        int n = Q.size();
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j <= i; ++j) {
                double sum = 0.0;
                for (int k = 0; k < j; ++k) sum += L[i][k] * L[j][k] * D[k];
                if (i == j) { D[i] = Q[i][i] - sum; L[i][i] = 1.0; } else { L[i][j] = (Q[i][j] - sum) / D[j]; }
            }
        }
    }

    /**
     * @brief Transforms float ambiguities into a decorrelated space.
     * @param Qa Covariance matrix of float ambiguities.
     * @param float_a Float ambiguity vector.
     * @param Z Transformation matrix.
     * @param float_z Transformed float ambiguities.
     * @param L Output L from LDLt of transformed covariance.
     * @param D Output D from LDLt of transformed covariance.
     */
    static void lambdaDecorrelate(const std::vector<std::vector<double>>& Qa, const std::vector<double>& float_a,
                                  std::vector<std::vector<double>>& Z, std::vector<double>& float_z,
                                  std::vector<std::vector<double>>& L, std::vector<double>& D) {
        int n = Qa.size();
        Z.assign(n, std::vector<double>(n, 0.0));
        for (int i = 0; i < n; ++i) Z[i][i] = 1.0;
        ldltDecompose(Qa, L, D);
        int i = 1;
        while (i < n) {
            int k = i - 1;
            for (int j = k; j >= 0; --j) {
                if (std::abs(L[i][j]) > 0.5) {
                    double mu = std::round(L[i][j]);
                    for (int m = 0; m <= j; ++m) L[i][m] -= mu * L[j][m];
                    for (int m = 0; m < n; ++m) Z[m][i] -= mu * Z[m][j];
                }
            }
            double delta = D[i] + L[i][k] * L[i][k] * D[k];
            if (delta < D[k] - 1e-9) {
                double d_k = D[k], d_i = D[i], eta = L[i][k];
                D[k] = delta; D[i] = (d_k * d_i) / delta; L[i][k] = (eta * d_k) / delta;
                for (int j = 0; j < k; ++j) { double t1 = L[k][j], t2 = L[i][j]; L[k][j] = t2 - eta * t1; L[i][j] = L[i][k] * t2 + (d_i / delta) * t1; }
                for (int j = i + 1; j < n; ++j) { double t1 = L[j][k], t2 = L[j][i]; L[j][k] = t1 * L[i][k] + t2 * (d_i / delta); L[j][i] = t1 - eta * t2; }
                for (int j = 0; j < n; ++j) std::swap(Z[j][k], Z[j][i]);
                i = (i > 1) ? i - 1 : 1;
            } else i++;
        }
        float_z.assign(n, 0.0);
        for (int r = 0; r < n; ++r) { for (int c = 0; c < n; ++c) float_z[r] += Z[c][r] * float_a[c]; }
    }

    /**
     * @brief Recursively searches the ambiguity ellipsoid for the two best integer candidates.
     */
    static void searchSphere(int level, double S_prev, std::vector<double>& v,
                             std::vector<int>& current_z, const std::vector<std::vector<double>>& L, const std::vector<double>& D,
                             const std::vector<double>& z_float, double& best_S, std::vector<int>& best_z,
                             double& second_best_S, std::vector<int>& second_best_z) {
        int n = L.size();
        if (level == n) {
            double S = S_prev;
            if (S < best_S) { second_best_S = best_S; second_best_z = best_z; best_S = S; best_z = current_z; }
            else if (S < second_best_S) { second_best_S = S; second_best_z = current_z; }
            return;
        }
        double center = z_float[level];
        for (int j = 0; j < level; ++j) center -= L[level][j] * v[j];
        double diff = second_best_S - S_prev;
        if (diff < 0.0) diff = 0.0;
        double limit = std::sqrt(D[level] * diff);
        int z_min = static_cast<int>(std::ceil(center - limit)), z_max = static_cast<int>(std::floor(center + limit));
        struct Candidate { int val; double dist; };
        std::vector<Candidate> candidates;
        for (int zi = z_min; zi <= z_max; ++zi) candidates.push_back({zi, std::abs(zi - center)});
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) { return a.dist < b.dist; });
        for (const auto& cand : candidates) {
            double vi = center - cand.val;
            double S_i = S_prev + (vi * vi) / D[level];
            if (S_i >= second_best_S) continue;
            v[level] = vi; current_z[level] = cand.val;
            searchSphere(level + 1, S_i, v, current_z, L, D, z_float, best_S, best_z, second_best_S, second_best_z);
        }
    }
};

} // namespace rtk

#endif // RTK_ENGINE_LAMBDA_HPP

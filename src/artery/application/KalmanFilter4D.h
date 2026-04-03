#ifndef ARTERY_KALMANFILTER4D_H_
#define ARTERY_KALMANFILTER4D_H_

#include <vector>
#include <cmath>

/**
 * UPGRADED: 4D Kalman Filter implementation for Constant Velocity (CV) model.
 * Tracks 4 states: [pos_x, pos_y, vel_x, vel_y]
 * Measurements are now FULL 4D: [pos_x, pos_y, vel_x, vel_y]
 * Uses optimized Sequential Scalar Update to bypass 4x4 Matrix Inversion.
 */
class KalmanFilter4D {
public:
    KalmanFilter4D(double q_noise = 20) {
        // State Transition Matrix (F) - 4x4
        F_ = std::vector<double>(16, 0.0);
        for(int i = 0; i < 4; i++) F_[i * 4 + i] = 1.0;

        // Initial State Vector (x) - 4x1
        x_ = std::vector<double>(4, 0.0);

        // State Covariance Matrix (P) - 4x4
        P_ = std::vector<double>(16, 0.0);
        for(int i = 0; i < 4; i++) P_[i * 4 + i] = 1000.0; // High initial uncertainty

        // Process Noise Covariance (Q) - 4x4
        Q_ = std::vector<double>(16, 0.0);

        is_initialized_ = false;
        q_factor_ = q_noise;
    }

    void init(const std::vector<double>& x0) {
        if (x0.size() == 4) {
            x_ = x0;
            is_initialized_ = true;
        }
    }

    void predict(double dt) {
        if (!is_initialized_) return;

        updateDt(dt);

        // State Prediction: x = F * x
        std::vector<double> x_new(4, 0.0);
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                x_new[i] += F_[i * 4 + j] * x_[j];
            }
        }
        x_ = x_new;

        // Covariance Prediction: P = F * P * F^T + Q
        std::vector<double> P_temp(16, 0.0);
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                for(int k = 0; k < 4; k++) {
                    P_temp[i * 4 + j] += F_[i * 4 + k] * P_[k * 4 + j];
                }
            }
        }

        std::vector<double> P_new(16, 0.0);
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                for(int k = 0; k < 4; k++) {
                    P_new[i * 4 + j] += P_temp[i * 4 + k] * F_[j * 4 + k];
                }
                P_new[i * 4 + j] += Q_[i * 4 + j];
            }
        }
        P_ = P_new;
    }

    /**
     * UPGRADED: Sequential Scalar Update for 4D measurement [x, y, vx, vy]
     * Eliminates the need for H matrix and costly inversions.
     */
    void update(const std::vector<double>& z) {
        if (!is_initialized_ || z.size() < 4) return;

        for (size_t m = 0; m < z.size(); m++) {
            // Measurement Noise Variance (R)
            double R_m = (m == 0 || m == 1) ? 0.5 : 1.0; 

            // Measurement residual
            double y_res = z[m] - x_[m];

            // Innovation Covariance
            double S = P_[m * 4 + m] + R_m;
            if (std::abs(S) < 1e-9) continue;

            // Kalman Gain
            std::vector<double> K(4, 0.0);
            for (int i = 0; i < 4; i++) {
                K[i] = P_[i * 4 + m] / S;
            }

            // Update State
            for (int i = 0; i < 4; i++) {
                x_[i] += K[i] * y_res;
            }

            // Update Covariance
            std::vector<double> P_new(16, 0.0);
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    P_new[i * 4 + j] = P_[i * 4 + j] - K[i] * P_[m * 4 + j];
                }
            }
            P_ = P_new;
        }
    }

    std::vector<double> getState() const { return x_; }

private:
    void updateDt(double dt) {
        // Kinematic Velocity Transition
        F_[0 * 4 + 2] = dt;            // x = x + vx*dt
        F_[1 * 4 + 3] = dt;            // y = y + vy*dt

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;

        std::fill(Q_.begin(), Q_.end(), 0.0);

        // Kinematic Process Noise Covariance (Q) for Constant Velocity
        double q = q_factor_;
        Q_[0 * 4 + 0] = 0.25 * dt4 * q;
        Q_[0 * 4 + 2] = 0.5 * dt3 * q;
        Q_[2 * 4 + 0] = 0.5 * dt3 * q;
        Q_[2 * 4 + 2] = dt2 * q;

        Q_[1 * 4 + 1] = 0.25 * dt4 * q;
        Q_[1 * 4 + 3] = 0.5 * dt3 * q;
        Q_[3 * 4 + 1] = 0.5 * dt3 * q;
        Q_[3 * 4 + 3] = dt2 * q;
    }

    std::vector<double> x_, F_, P_, Q_;
    bool is_initialized_;
    double q_factor_;
};

#endif
#ifndef ARTERY_KALMANFILTER6D_H_
#define ARTERY_KALMANFILTER6D_H_

#include <vector>
#include <cmath>

/**
 * 6D Kalman Filter implementation for Constant Acceleration (CA) model.
 * Tracks 6 states: [pos_x, pos_y, vel_x, vel_y, acc_x, acc_y]
 * Measurements are 6D: [pos_x, pos_y, vel_x, vel_y, acc_x, acc_y]
 * Uses optimized Sequential Scalar Update to bypass 6x6 Matrix Inversion.
 */
class KalmanFilter6D {
public:
    KalmanFilter6D(double q_noise = 20.0) {
        // State Transition Matrix (F) - 6x6
        F_ = std::vector<double>(36, 0.0);
        for(int i = 0; i < 6; i++) F_[i * 6 + i] = 1.0;

        // Initial State Vector (x) - 6x1
        x_ = std::vector<double>(6, 0.0);

        // State Covariance Matrix (P) - 6x6
        P_ = std::vector<double>(36, 0.0);
        for(int i = 0; i < 6; i++) P_[i * 6 + i] = 1000.0; // High initial uncertainty

        // Process Noise Covariance (Q) - 6x6
        Q_ = std::vector<double>(36, 0.0);

        is_initialized_ = false;
        q_factor_ = q_noise;
    }

    void init(const std::vector<double>& x0) {
        if (x0.size() == 6) {
            x_ = x0;
            is_initialized_ = true;
        }
    }

    void predict(double dt) {
        if (!is_initialized_) return;

        updateDt(dt);

        // State Prediction: x = F * x
        std::vector<double> x_new(6, 0.0);
        for(int i = 0; i < 6; i++) {
            for(int j = 0; j < 6; j++) {
                x_new[i] += F_[i * 6 + j] * x_[j];
            }
        }
        x_ = x_new;

        // Covariance Prediction: P = F * P * F^T + Q
        std::vector<double> P_temp(36, 0.0);
        for(int i = 0; i < 6; i++) {
            for(int j = 0; j < 6; j++) {
                for(int k = 0; k < 6; k++) {
                    P_temp[i * 6 + j] += F_[i * 6 + k] * P_[k * 6 + j];
                }
            }
        }

        std::vector<double> P_new(36, 0.0);
        for(int i = 0; i < 6; i++) {
            for(int j = 0; j < 6; j++) {
                for(int k = 0; k < 6; k++) {
                    P_new[i * 6 + j] += P_temp[i * 6 + k] * F_[j * 6 + k];
                }
                P_new[i * 6 + j] += Q_[i * 6 + j];
            }
        }
        P_ = P_new;
    }

    /**
     * Sequential Scalar Update for 6D measurement [x, y, vx, vy, ax, ay]
     * Elegantly avoids 6x6 matrix inversion by processing independent sensors one by one.
     */
    void update(const std::vector<double>& z) {
        if (!is_initialized_ || z.size() < 6) return;

        for (size_t m = 0; m < z.size(); m++) {
            // Assign dynamic Measurement Noise Variance (R)
            double R_m = 1.0;
            if (m == 0 || m == 1) R_m = 0.5;      // Position noise variance
            else if (m == 2 || m == 3) R_m = 1.0; // Velocity noise variance
            else if (m == 4 || m == 5) R_m = 5.0; // Acceleration noise variance

            // Measurement residual
            double y_res = z[m] - x_[m];

            // Innovation Covariance (Scalar): S = H_m * P * H_m^T + R_m
            // Since H_m is a 1-to-1 mapping, H_m * P * H_m^T is simply the diagonal element P[m, m]
            double S = P_[m * 6 + m] + R_m;
            if (std::abs(S) < 1e-9) continue;

            // Kalman Gain: K = P * H_m^T / S
            std::vector<double> K(6, 0.0);
            for (int i = 0; i < 6; i++) {
                K[i] = P_[i * 6 + m] / S;
            }

            // Update State: x = x + K * y_res
            for (int i = 0; i < 6; i++) {
                x_[i] += K[i] * y_res;
            }

            // Update Covariance: P = P - K * (H_m * P)
            std::vector<double> P_new(36, 0.0);
            for (int i = 0; i < 6; i++) {
                for (int j = 0; j < 6; j++) {
                    P_new[i * 6 + j] = P_[i * 6 + j] - K[i] * P_[m * 6 + j];
                }
            }
            P_ = P_new;
        }
    }

    std::vector<double> getState() const { return x_; }

private:
    void updateDt(double dt) {
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;

        // Kinematic Parabolic Transition Matrix (F)
        // X-Axis
        F_[0 * 6 + 2] = dt;            // x = x + vx*dt
        F_[0 * 6 + 4] = 0.5 * dt2;     // x = x + ... + 0.5*ax*dt^2
        F_[2 * 6 + 4] = dt;            // vx = vx + ax*dt
        // Y-Axis
        F_[1 * 6 + 3] = dt;            // y = y + vy*dt
        F_[1 * 6 + 5] = 0.5 * dt2;     // y = y + ... + 0.5*ay*dt^2
        F_[3 * 6 + 5] = dt;            // vy = vy + ay*dt

        std::fill(Q_.begin(), Q_.end(), 0.0);

        // Kinematic Process Noise Covariance (Q) for Constant Acceleration
        double q = q_factor_;
        
        // X-Axis Block
        Q_[0 * 6 + 0] = (dt5 / 20.0) * q;  // var(x)
        Q_[0 * 6 + 2] = (dt4 / 8.0) * q;   // cov(x, vx)
        Q_[0 * 6 + 4] = (dt3 / 6.0) * q;   // cov(x, ax)
        Q_[2 * 6 + 0] = Q_[0 * 6 + 2];
        Q_[2 * 6 + 2] = (dt3 / 3.0) * q;   // var(vx)
        Q_[2 * 6 + 4] = (dt2 / 2.0) * q;   // cov(vx, ax)
        Q_[4 * 6 + 0] = Q_[0 * 6 + 4];
        Q_[4 * 6 + 2] = Q_[2 * 6 + 4];
        Q_[4 * 6 + 4] = dt * q;            // var(ax)

        // Y-Axis Block
        Q_[1 * 6 + 1] = (dt5 / 20.0) * q;  // var(y)
        Q_[1 * 6 + 3] = (dt4 / 8.0) * q;   // cov(y, vy)
        Q_[1 * 6 + 5] = (dt3 / 6.0) * q;   // cov(y, ay)
        Q_[3 * 6 + 1] = Q_[1 * 6 + 3];
        Q_[3 * 6 + 3] = (dt3 / 3.0) * q;   // var(vy)
        Q_[3 * 6 + 5] = (dt2 / 2.0) * q;   // cov(vy, ay)
        Q_[5 * 6 + 1] = Q_[1 * 6 + 5];
        Q_[5 * 6 + 3] = Q_[3 * 6 + 5];
        Q_[5 * 6 + 5] = dt * q;            // var(ay)
    }

    std::vector<double> x_, F_, P_, Q_;
    bool is_initialized_;
    double q_factor_;
};

#endif
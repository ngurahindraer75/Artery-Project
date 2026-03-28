#ifndef ARTERY_KALMANFILTER4D_H_
#define ARTERY_KALMANFILTER4D_H_

#include <vector>
#include <cmath>

/*
 * Implementasi Kalman Filter 4D (Constant Velocity) FULL MATRIX.
 * Melacak 4 state: [x, y, vx, vy]
 */
class KalmanFilter4D {
public:
    KalmanFilter4D() {
        F_ = std::vector<double>(16, 0.0);
        for(int i=0; i<4; i++) F_[i*4 + i] = 1.0;

        // H matriks ukuran 2x4
        H_ = std::vector<double>(8, 0.0);
        H_[ 0 ] = 1.0;
        H_[ 5 ] = 1.0;

        x_ = std::vector<double>(4, 0.0);

        P_ = std::vector<double>(16, 0.0);
        P_[ 0 ] = 1000.0; P_[ 5 ] = 1000.0;
        P_[ 10 ] = 1000.0; P_[ 15 ] = 1000.0;

        Q_ = std::vector<double>(16, 0.0);

        R_ = std::vector<double>(4, 0.0);
        R_[ 0 ] = 0.1;
        R_[ 3 ] = 0.1;

        is_initialized_ = false;
        last_timestamp_ = 0.0;
    }

    void init(double x, double y, double timestamp) {
        x_[ 0 ] = x;
        x_[ 1 ] = y;
        x_[ 2 ] = 0; // vx awal
        x_[ 3 ] = 0; // vy awal
        last_timestamp_ = timestamp;
        is_initialized_ = true;
    }

    bool isInitialized() const {
        return is_initialized_;
    }

    std::vector<double> getState() const {
        return x_;
    }

    void update(double x, double y, double timestamp) {
        if (!is_initialized_) {
            init(x, y, timestamp);
            return;
        }

        double dt = timestamp - last_timestamp_;
        last_timestamp_ = timestamp;

        if (dt <= 0) dt = 0.01;

        updateDt(dt);
        predict();

        std::vector<double> z = {x, y};
        std::vector<double> innovation = {z[ 0 ] - x_[ 0 ], z[ 1 ] - x_[ 1 ]};

        // S = H * P * H^T + R
        std::vector<double> S(4, 0.0);
        S[ 0 ] = P_[ 0 ] + R_[ 0 ];
        S[ 1 ] = P_[ 1 ];
        S[ 2 ] = P_[ 4 ];
        S[ 3 ] = P_[ 5 ] + R_[ 3 ];

        double det_S = S[ 0 ] * S[ 3 ] - S[ 1 ] * S[ 2 ];
        if (std::abs(det_S) < 1e-9) det_S = 1e-9;
        std::vector<double> S_inv = {S[ 3 ] / det_S, -S[ 1 ] / det_S, -S[ 2 ] / det_S, S[ 0 ] / det_S};

        // K = P * H^T * S_inv
        std::vector<double> K(8, 0.0);
        for(int i = 0; i < 4; i++) {
            K[i*2 + 0] = P_[i*4 + 0] * S_inv[ 0 ] + P_[i*4 + 1] * S_inv[ 2 ];
            K[i*2 + 1] = P_[i*4 + 0] * S_inv[ 1 ] + P_[i*4 + 1] * S_inv[ 3 ];
        }

        // x = x + K * innovation
        for(int i = 0; i < 4; i++) {
            x_[i] += K[i*2 + 0] * innovation[ 0 ] + K[i*2 + 1] * innovation[ 1 ];
        }

        // I_KH = I - K * H
        std::vector<double> I_KH(16, 0.0);
        for(int i = 0; i < 4; i++) {
            I_KH[i*4 + i] = 1.0;
            for(int j = 0; j < 4; j++) {
                double kh = 0.0;
                for(int k = 0; k < 2; k++) {
                    kh += K[i*2 + k] * H_[k*4 + j];
                }
                I_KH[i*4 + j] -= kh;
            }
        }

        // P = (I - K * H) * P
        std::vector<double> P_old = P_;
        std::vector<double> P_new(16, 0.0);
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                for(int k = 0; k < 4; k++) {
                    P_new[i*4 + j] += I_KH[i*4 + k] * P_old[k*4 + j];
                }
            }
        }
        P_ = P_new;
    }

    // --- PERBAIKAN FUNGSI PREDIKSI KE MASA DEPAN ---
    std::pair<double, double> predict(double delta_time) {
        // Ekstrapolasi: Posisi_Baru = Posisi_Sekarang + (Kecepatan * Waktu)
        double pred_x = x_[ 0 ] + (x_[ 2 ] * delta_time);
        double pred_y = x_[ 1 ] + (x_[ 3 ] * delta_time);
        return {pred_x, pred_y};
    }
    // -----------------------------------------------

private:
    std::vector<double> x_;
    std::vector<double> F_;
    std::vector<double> P_;
    std::vector<double> Q_;
    std::vector<double> H_;
    std::vector<double> R_;
    bool is_initialized_;
    double last_timestamp_;

    void predict() {
        // x = F * x
        std::vector<double> x_new(4, 0.0);
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                x_new[i] += F_[i*4 + j] * x_[j];
            }
        }
        x_ = x_new;

        // P = F * P * F^T + Q
        std::vector<double> P_temp(16, 0.0);
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                for(int k = 0; k < 4; k++) {
                    P_temp[i*4 + j] += F_[i*4 + k] * P_[k*4 + j];
                }
            }
        }

        std::vector<double> P_new(16, 0.0);
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                for(int k = 0; k < 4; k++) {
                    P_new[i*4 + j] += P_temp[i*4 + k] * F_[j*4 + k];
                }
                P_new[i*4 + j] += Q_[i*4 + j];
            }
        }
        P_ = P_new;
    }

    void updateDt(double dt) {
        F_[ 2 ] = dt;
        F_[ 7 ] = dt;

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;

        double noise_ax = 2.0; // Disamakan dengan KF 6D
        double noise_ay = 2.0;

        Q_[ 0 ] = (dt4 / 4.0) * noise_ax;
        Q_[ 2 ] = (dt3 / 2.0) * noise_ax;
        Q_[ 5 ] = (dt4 / 4.0) * noise_ay;
        Q_[ 7 ] = (dt3 / 2.0) * noise_ay;
        Q_[ 8 ] = (dt3 / 2.0) * noise_ax;
        Q_[ 10 ] = dt2 * noise_ax;
        Q_[ 13 ] = (dt3 / 2.0) * noise_ay;
        Q_[ 15 ] = dt2 * noise_ay;
    }
};

#endif /* ARTERY_KALMANFILTER4D_H_ */
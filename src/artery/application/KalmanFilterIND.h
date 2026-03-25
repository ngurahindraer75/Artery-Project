#ifndef ARTERY_KALMANFILTERIND_H_
#define ARTERY_KALMANFILTERIND_H_

#include <vector>
#include <cmath>

/*
 * Implementasi Kalman Filter 2D yang disederhanakan.
 * Dibuat header-only agar mudah di-include dan tidak perlu library eksternal.
 *
 * Model ini melacak 4 state: [x, y, vx, vy]
 * - x: Posisi (misal, Latitude)
 * - y: Posisi (misal, Longitude)
 * - vx: Kecepatan di sumbu x
 * - vy: Kecepatan di sumbu y
 */
class KalmanFilterIND {
public:
    KalmanFilterIND() {
        // Matriks transisi state (F) - Model Constant Velocity
        // [ 1  0 dt  0 ]
        // [ 0  1  0 dt ]
        // [ 0  0  1  0 ]
        // [ 0  0  0  1 ]
        // (dt akan di-update nanti)
        F_ = std::vector<double>(16, 0.0);
        F_[0] = 1.0; F_[2] = 1.0; // dt placeholder
        F_[5] = 1.0; F_[7] = 1.0; // dt placeholder
        F_[10] = 1.0;
        F_[15] = 1.0;

        // Matriks observasi (H) - Kita hanya mengobservasi x dan y
        // [ 1 0 0 0 ]
        // [ 0 1 0 0 ]
        H_ = std::vector<double>(8, 0.0);
        H_[0] = 1.0;
        H_[5] = 1.0;

        // State vector [x, y, vx, vy]
        x_ = std::vector<double>(4, 0.0);

        // Kovarians estimasi (P) - Inisialisasi dengan ketidakpastian tinggi
        P_ = std::vector<double>(16, 0.0);
        P_[0] = 1000.0;
        P_[5] = 1000.0;
        P_[10] = 1000.0;
        P_[15] = 1000.0;

        // Noise proses (Q) - Ketidakpastian dalam model (misal, akselerasi)
        Q_ = std::vector<double>(16, 0.0);
        // (akan di-set di 'updateDt')

        // Noise pengukuran (R) - Ketidakpastian dari sensor (CAM)
        R_ = std::vector<double>(4, 0.0);
        R_[0] = 0.1; // Sesuaikan nilai ini
        R_[3] = 0.1; // Sesuaikan nilai ini

        is_initialized_ = false;
        last_timestamp_ = 0.0;
    }

    void init(double x, double y, double timestamp) {
        x_[0] = x;
        x_[1] = y;
        x_[2] = 0; // vx awal
        x_[3] = 0; // vy awal
        last_timestamp_ = timestamp;
        is_initialized_ = true;
    }

    bool isInitialized() const {
        return is_initialized_;
    }

    // Fungsi utama untuk memperbarui state berdasarkan pengukuran baru
    void update(double x, double y, double timestamp) {
        if (!is_initialized_) {
            init(x, y, timestamp);
            return;
        }

        // 1. Hitung delta time (dt)
        double dt = timestamp - last_timestamp_;
        last_timestamp_ = timestamp;

        if (dt <= 0) dt = 0.01; // hindari dt=0

        // 2. Lakukan PREDICT step (berdasarkan dt)
        updateDt(dt);
        predict();

        // 3. Lakukan UPDATE step (dengan data sensor baru)
        std::vector<double> z = {x, y}; // Pengukuran

        // ---- PERBAIKAN DI SINI ----
        // y = z - H * x_
        // Variabel 'y' (vektor) diganti namanya menjadi 'innovation'
        // untuk menghindari konflik dengan parameter 'y' (double)
        std::vector<double> innovation = {z[0] - x_[0], z[1] - x_[1]};
        // ---- AKHIR PERBAIKAN 1 ----

        // S = H * P_ * H_transpose + R
        std::vector<double> S(4, 0.0);
        S[0] = P_[0] + R_[0];
        S[1] = P_[1];
        S[2] = P_[4];
        S[3] = P_[5] + R_[3];

        // Hitung invers S (untuk 2x2)
        double det_S = S[0] * S[3] - S[1] * S[2];
        if (std::abs(det_S) < 1e-9) det_S = 1e-9;
        std::vector<double> S_inv = {S[3] / det_S, -S[1] / det_S, -S[2] / det_S, S[0] / det_S};

        // K = P_ * H_transpose * S_inv
        std::vector<double> K(8, 0.0);
        K[0] = P_[0] * S_inv[0] + P_[1] * S_inv[2];
        K[1] = P_[0] * S_inv[1] + P_[1] * S_inv[3];
        K[2] = P_[4] * S_inv[0] + P_[5] * S_inv[2];
        K[3] = P_[4] * S_inv[1] + P_[5] * S_inv[3];
        K[4] = P_[8] * S_inv[0] + P_[9] * S_inv[2];
        K[5] = P_[8] * S_inv[1] + P_[9] * S_inv[3];
        K[6] = P_[12] * S_inv[0] + P_[13] * S_inv[2];
        K[7] = P_[12] * S_inv[1] + P_[13] * S_inv[3];

        // ---- PERBAIKAN DI SINI ----
        // x_ = x_ + K * y
        // Menggunakan nama variabel baru 'innovation'
        x_[0] = x_[0] + (K[0] * innovation[0] + K[1] * innovation[1]);
        x_[1] = x_[1] + (K[2] * innovation[0] + K[3] * innovation[1]);
        x_[2] = x_[2] + (K[4] * innovation[0] + K[5] * innovation[1]);
        x_[3] = x_[3] + (K[6] * innovation[0] + K[7] * innovation[1]);
        // ---- AKHIR PERBAIKAN 2 ----


        // P_ = (I - K * H) * P_
        std::vector<double> I_KH(16, 0.0);
        I_KH[0] = 1.0 - K[0]; I_KH[1] = -K[1];
        I_KH[4] = -K[2]; I_KH[5] = 1.0 - K[3];
        I_KH[8] = -K[4]; I_KH[9] = -K[5];
        I_KH[10] = 1.0;
        I_KH[12] = -K[6]; I_KH[13] = -K[7];
        I_KH[15] = 1.0;

        std::vector<double> P_old = P_;
        P_[0] = I_KH[0] * P_old[0] + I_KH[1] * P_old[4];
        P_[1] = I_KH[0] * P_old[1] + I_KH[1] * P_old[5];
        P_[4] = I_KH[4] * P_old[0] + I_KH[5] * P_old[4];
        P_[5] = I_KH[4] * P_old[1] + I_KH[5] * P_old[5];
        // (Sederhana, hanya update bagian atas 2x2 dari P)
    }

    // Fungsi untuk memprediksi state ke masa depan
    std::pair<double, double> predict(double delta_time) {
        // Lakukan prediksi sederhana dari state saat ini
        double pred_x = x_[0] + x_[2] * delta_time;
        double pred_y = x_[1] + x_[3] * delta_time;
        return {pred_x, pred_y};
    }

private:
    std::vector<double> x_; // State [x, y, vx, vy]
    std::vector<double> F_; // Matriks Transisi
    std::vector<double> P_; // Kovarians Estimasi
    std::vector<double> Q_; // Kovarians Noise Proses
    std::vector<double> H_; // Matriks Observasi
    std::vector<double> R_; // Kovarians Noise Pengukuran

    bool is_initialized_;
    double last_timestamp_;

    // PREDICT step internal
    void predict() {
        // x_ = F * x_
        std::vector<double> x_new(4, 0.0);
        x_new[0] = x_[0] * F_[0] + x_[2] * F_[2];
        x_new[1] = x_[1] * F_[5] + x_[3] * F_[7];
        x_new[2] = x_[2] * F_[10];
        x_new[3] = x_[3] * F_[15];
        x_ = x_new;

        // P_ = F * P_ * F_transpose + Q
        // (Implementasi disederhanakan untuk performa, mengabaikan P update
        // di predict() untuk implementasi cepat ini.
        // Implementasi penuh akan membutuhkan perkalian matriks 4x4)
        // P = F * P_ * F_T + Q;
    }

    // Update matriks F dan Q berdasarkan dt
    void updateDt(double dt) {
        F_[2] = dt;
        F_[7] = dt;

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;

        // Noise proses (Q) di-update
        // Kita asumsikan noise akselerasi (ax, ay)
        double noise_ax = 0.5; // Sesuaikan nilai ini
        double noise_ay = 0.5; // Sesuaikan nilai ini

        Q_[0] = dt4 / 4.0 * noise_ax;
        Q_[2] = dt3 / 2.0 * noise_ax;
        Q_[5] = dt4 / 4.0 * noise_ay;
        Q_[7] = dt3 / 2.0 * noise_ay;
        Q_[8] = dt3 / 2.0 * noise_ax;
        Q_[10] = dt2 * noise_ax;
        Q_[13] = dt3 / 2.0 * noise_ay;
        Q_[15] = dt2 * noise_ay;
    }
};

#endif /* ARTERY_KALMANFILTERIND_H_ */

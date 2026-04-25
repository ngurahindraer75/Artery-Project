#ifndef ARTERY_TRAJEKTORIAPPKF_H_
#define ARTERY_TRAJEKTORIAPPKF_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include <omnetpp/simtime.h>
#include <omnetpp/cmessage.h>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <deque>
#include <cmath>

namespace artery
{

// =========================================================================
// STRUKTUR MATEMATIS DECOUPLED 2D KALMAN FILTER
// =========================================================================
struct KalmanFilter1D {
    double x[2];         // State Vector -> [0]: Posisi, [1]: Kecepatan
    double P[2][2];    // Covariance Matrix
    double Q[2][2];    // Process Noise Matrix
    double R[2][2];    // Measurement Noise Matrix

    // Inisialisasi Matriks
    void init(double pos, double vel) {
        x[0] = pos; 
        x[1] = vel;

        P[0][0] = 1.0; P[0][1] = 0.0;
        P[1][0] = 0.0; P[1][1] = 1.0;

        Q[0][0] = 2.0; Q[0][1] = 0.0;
        Q[1][0] = 0.0; Q[1][1] = 2.0;

        R[0][0] = 0.25; R[0][1] = 0.0;
        R[1][0] = 0.0; R[1][1] = 0.04;
    }

    // Langkah 1: PREDIKSI (Dilengkapi dengan B_k u_k -> Akselerasi)
    void predict(double dt, double a) {
        // B_k * u_k = 0.5 * a * dt^2 (untuk posisi) dan a * dt (untuk kecepatan)
        double x_new_0 = x[0] + (dt * x[1]) + (0.5 * a * dt * dt); 
        double x_new_1 = x[1] + (a * dt);

        double P_new_0_0 = P[0][0] + dt * P[1][0] + dt * P[0][1] + dt * dt * P[1][1] + Q[0][0];
        double P_new_0_1 = P[0][1] + dt * P[1][1] + Q[0][1];
        double P_new_1_0 = P[1][0] + dt * P[1][1] + Q[1][0];
        double P_new_1_1 = P[1][1] + Q[1][1];

        x[0] = x_new_0;
        x[1] = x_new_1;
        P[0][0] = P_new_0_0;
        P[0][1] = P_new_0_1;
        P[1][0] = P_new_1_0;
        P[1][1] = P_new_1_1;
    }

    // Langkah 2: KOREKSI BERDASARKAN DATA CAM BARU
    void update(double meas_pos, double meas_vel) {
        double y_0 = meas_pos - x[0];
        double y_1 = meas_vel - x[1];

        double S_0_0 = P[0][0] + R[0][0];
        double S_0_1 = P[0][1] + R[0][1];
        double S_1_0 = P[1][0] + R[1][0];
        double S_1_1 = P[1][1] + R[1][1];

        double det = S_0_0 * S_1_1 - S_0_1 * S_1_0;
        if (std::abs(det) < 1e-9) return; 

        double Sinv_0_0 = S_1_1 / det;
        double Sinv_0_1 = -S_0_1 / det;
        double Sinv_1_0 = -S_1_0 / det;
        double Sinv_1_1 = S_0_0 / det;

        double K_0_0 = P[0][0] * Sinv_0_0 + P[0][1] * Sinv_1_0;
        double K_0_1 = P[0][0] * Sinv_0_1 + P[0][1] * Sinv_1_1;
        double K_1_0 = P[1][0] * Sinv_0_0 + P[1][1] * Sinv_1_0;
        double K_1_1 = P[1][0] * Sinv_0_1 + P[1][1] * Sinv_1_1;

        x[0] = x[0] + K_0_0 * y_0 + K_0_1 * y_1;
        x[1] = x[1] + K_1_0 * y_0 + K_1_1 * y_1;

        double P_new_0_0 = P[0][0] - (K_0_0 * P[0][0] + K_0_1 * P[1][0]);
        double P_new_0_1 = P[0][1] - (K_0_0 * P[0][1] + K_0_1 * P[1][1]);
        double P_new_1_0 = P[1][0] - (K_1_0 * P[0][0] + K_1_1 * P[1][0]);
        double P_new_1_1 = P[1][1] - (K_1_0 * P[0][1] + K_1_1 * P[1][1]);

        P[0][0] = P_new_0_0;
        P[0][1] = P_new_0_1;
        P[1][0] = P_new_1_0;
        P[1][1] = P_new_1_1;
    }
};

struct MovementDataKF {
    double time_send_absolut;
    double time_receive;
    long lat_raw;
    long lon_raw;
};

struct PendingPredictionKF {
    double processing_time;
    double latest_cam_time;
    double kf_x_pos;
    double kf_x_vel;
    double kf_y_pos;
    double kf_y_vel;
};

struct AgentHistoryKF {
    bool is_initialized = false;
    double last_update_time = 0.0;
    
    KalmanFilter1D kf_X; 
    KalmanFilter1D kf_Y; 

    std::deque<MovementDataKF> ground_truth_history;
    std::vector<PendingPredictionKF> pending_1s;
    std::vector<PendingPredictionKF> pending_2s;
    std::vector<PendingPredictionKF> pending_3s;
};

class TrajektoriAppKF : public ItsG5BaseService
{
public:
    void initialize() override;
    void finish() override;
    void handleMessage(omnetpp::cMessage* msg) override;

protected:
    void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID,
                       omnetpp::cObject* obj, omnetpp::cObject* details) override;

private:
    std::string getNodeType();
    void runKalmanPrediction();

    omnetpp::simsignal_t mCamReceivedSignal;
    omnetpp::simsignal_t mCamSentSignal; 
    omnetpp::cMessage* mPredictionTimer = nullptr;

    std::ofstream mLogFile_1s_car;
    std::ofstream mLogFile_2s_car;
    std::ofstream mLogFile_3s_car;
    std::ofstream mLogFile_1s_person;
    std::ofstream mLogFile_2s_person;
    std::ofstream mLogFile_3s_person;

    const artery::VehicleDataProvider* mVehicleDataProvider = nullptr;
    std::map<long, AgentHistoryKF> mOtherNodes;
};

} // namespace artery

#endif /* ARTERY_TRAJEKTORIAPPKF_H_ */

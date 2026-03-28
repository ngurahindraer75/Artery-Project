#ifndef ARTERY_TRAJEKTORIAPPKF4D_H_
#define ARTERY_TRAJEKTORIAPPKF4D_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include "KalmanFilter4D.h" 
#include <omnetpp/simtime.h>
#include <deque>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <memory> 

namespace artery
{

struct MovementData {
    long gen_delta_time_raw;
    double cam_received_time;
    double calculated_delay;
    omnetpp::simtime_t timestamp;
    double latitude;
    double longitude;
    double speed_mps;
};

// STRUKTUR ANTREAN PREDIKSI MURNI KALMAN FILTER
struct PendingPrediction {
    double target_time;
    double creation_time;
    double kf_pos_lat; // Posisi x dari State KF
    double kf_pos_lon; // Posisi y dari State KF
    double kf_vel_lat; // Kecepatan vx dari State KF
    double kf_vel_lon; // Kecepatan vy dari State KF
    double pred_lat;
    double pred_lon;
};

struct AgentHistory {
    std::deque<MovementData> history; 
    omnetpp::simtime_t lastReceptionTime;
    bool hasNewData = false;

    std::unique_ptr<KalmanFilter4D> kf_state;
    std::vector<PendingPrediction> pending_predictions; 
};

class TrajektoriAppKF4D : public ItsG5BaseService
{
public:
    void initialize() override;
    void finish() override;

protected:
    void handleMessage(omnetpp::cMessage* msg) override;
    void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID,
                       omnetpp::cObject* obj, omnetpp::cObject* details) override;

private:
    void logTrajectory();
    std::string getNodeType();
    void runPredictions();

    omnetpp::cMessage* mLogTimer = nullptr;
    omnetpp::simsignal_t mCamReceivedSignal;
    omnetpp::SimTime mLogInterval;

    std::ofstream mLogFile;
    std::map<long, AgentHistory> mOtherNodes;
    const artery::VehicleDataProvider* mVehicleDataProvider = nullptr;

    omnetpp::cMessage* mPredictionTimer = nullptr;
    std::ofstream mPredictionLogFile;

    double mSumAE = 0.0;
    long mCountAE = 0;
};

} // namespace artery

#endif /* ARTERY_TRAJEKTORIAPPKF4D_H_ */
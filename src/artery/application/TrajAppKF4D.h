/**
 * @file V4_TrajAppKF4D.h
 * @brief Header file for Trajectory Prediction Application using Kalman Filter 4D.
 * @details UPGRADED TO V4: Omnidirectional Tracking. All observers track all targets.
 *          Output files are strictly branched based on TARGET type.
 *          MAE accumulators are removed to keep CSV purely tabular for ML post-processing.
 */

#ifndef ARTERY_TRAJAPPKF4D_H_
#define ARTERY_TRAJAPPKF4D_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/CaObject.h"
#include "artery/application/KalmanFilter4D.h"
#include <omnetpp.h>
#include <omnetpp/simtime.h>
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <memory>
#include <cmath>
#include <vector>

namespace artery {

struct MovementDataKF4D {
    long gen_delta_time_raw;
    double cam_received_time;
    double calculated_delay;
    double timestamp;
    double lat_raw;
    double lon_raw;
    double local_x;
    double local_y;
    double vel_x;
    double vel_y;
};

struct PendingPredictionKF4D {
    double processing_time;
    double latest_cam_time;
    double base_cam_lat;
    double base_cam_lon;
    
    // Extracted Cartesian Matrices [X, Y, Vx, Vy]
    double state_x;
    double state_y;
    double vel_x;
    double vel_y;
    
    bool eval_1s_done = false;
    bool eval_2s_done = false;
    bool eval_3s_done = false;
};

struct AgentHistoryKF4D {
    std::string target_type; // Identity marker for branched evaluation
    std::deque<MovementDataKF4D> history;
    std::deque<PendingPredictionKF4D> pending_queue;
    std::unique_ptr<KalmanFilter4D> kf_state;
    
    double last_prediction_time = -1.0;
    double last_reception_time = -1.0;
    double ref_lat_raw = 0.0;
    double ref_lon_raw = 0.0;
    bool is_ref_set = false;
};

class TrajAppKF4D : public ItsG5BaseService {
public:
    virtual ~TrajAppKF4D() override;

protected:
    virtual void initialize() override;
    virtual void finish() override;
    virtual void handleMessage(omnetpp::cMessage* msg) override;
    virtual void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;

private:
    std::string getNodeType();
    void latLonToCartesian(double lat_raw, double lon_raw, double ref_lat_raw, double ref_lon_raw, double& x, double& y);
    void cartesianToLatLon(double x, double y, double ref_lat_raw, double ref_lon_raw, double& lat_raw, double& lon_raw);
    void takeKfSnapshot();
    void evaluatePendingPredictionsKF4D(long targetId, AgentHistoryKF4D& hist_struct);

    std::map<long, AgentHistoryKF4D> mOtherNodes;
    
    // Dual Output Streams
    std::ofstream mCamLogVeh, mCamLogPed;
    std::ofstream mPredLog1sVeh, mPredLog1sPed;
    std::ofstream mPredLog2sVeh, mPredLog2sPed;
    std::ofstream mPredLog3sVeh, mPredLog3sPed;

    omnetpp::simsignal_t mCamReceivedSignal;
    omnetpp::cMessage* mLogTimer = nullptr;
    omnetpp::cMessage* mPredictionTimer = nullptr;
    omnetpp::SimTime mLogInterval;
};

} // namespace artery
#endif

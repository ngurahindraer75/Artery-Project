#ifndef ARTERY_TRAJEKTORIAPPKF6D_H_
#define ARTERY_TRAJEKTORIAPPKF6D_H_

#include "artery/application/ItsG5BaseService.h"
#include "KalmanFilter6D.h"
#include <vector>
#include <deque>
#include <map>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <memory>

namespace artery {

struct MovementData {
    long gen_delta_time_raw;
    double cam_received_time;
    double calculated_delay;
    omnetpp::simtime_t timestamp;
    double latitude;
    double longitude;
    double speed_mps;
    double heading_degree;
    double acceleration_mps2; 
    
    // Extracted Cartesian Metrics
    double local_x;
    double local_y;
    double vel_x;
    double vel_y;
    double acc_x;
    double acc_y;
};

struct PendingPrediction {
    double processing_time;
    double latest_cam_time;
    double base_cam_lat;
    double base_cam_lon;
    std::vector<double> kf_state_snapshot; // Now holds 6 elements

    bool eval_1s_done = false;
    bool eval_2s_done = false;
    bool eval_3s_done = false;
};

struct AgentHistory {
    std::deque<MovementData> history;
    std::deque<PendingPrediction> pending_queue;

    omnetpp::simtime_t lastReceptionTime;
    bool hasNewData = false;
    
    std::unique_ptr<KalmanFilter6D> kf_state;

    double ref_lat = 0.0;
    double ref_lon = 0.0;
    bool is_ref_set = false;
};

class TrajektoriAppKF6D : public ItsG5BaseService {
public:
    virtual ~TrajektoriAppKF6D();
    virtual void initialize() override;
    virtual void finish() override;

protected:
    virtual void handleMessage(omnetpp::cMessage* msg) override;
    virtual void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;

private:
    void logTrajectory();
    void takeKfSnapshot();
    void evaluatePendingPredictions(long targetId, AgentHistory& hist_struct);
    std::string getNodeType();

    void latLonToCartesian(double lat_micro, double lon_micro, double ref_lat_micro, double ref_lon_micro, double& x, double& y);
    void cartesianToLatLon(double x, double y, double ref_lat_micro, double ref_lon_micro, double& lat_micro, double& lon_micro);

    omnetpp::cMessage* mLogTimer = nullptr;
    omnetpp::cMessage* mPredictionTimer = nullptr;
    omnetpp::simsignal_t mCamReceivedSignal;
    omnetpp::SimTime mLogInterval;

    std::ofstream mCamLogFile;
    std::ofstream mPredLog1s;
    std::ofstream mPredLog2s;
    std::ofstream mPredLog3s;

    double mSumAe1s = 0.0, mSumAe2s = 0.0, mSumAe3s = 0.0;
    long mCountAe1s = 0, mCountAe2s = 0, mCountAe3s = 0;

    std::map<long, AgentHistory> mOtherNodes;
};

} // namespace artery

#endif
#ifndef ARTERY_TRAJEKTORIAPPKFV3_H_
#define ARTERY_TRAJEKTORIAPPKFV3_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include "KalmanFilter4D.h"
#include <omnetpp/simtime.h>
#include <deque>
#include <vector>
#include <fstream>
#include <string>
#include <map>
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
    double local_x; 
    double local_y; 
};

// On-the-fly snapshot waiting in the queue for future CAMs
struct PendingPrediction {
    double processing_time;
    double latest_cam_time;
    double base_cam_lat; 
    double base_cam_lon; 
    std::vector<double> kf_state_snapshot;
    
    // Multi-stage evaluation flags
    bool eval_1s_done = false;
    bool eval_2s_done = false;
    bool eval_3s_done = false;
};

struct AgentHistory {
    // Deque is used to easily pop old data and prevent memory leaks
    std::deque<MovementData> history; 
    std::deque<PendingPrediction> pending_queue; 
    
    omnetpp::simtime_t lastReceptionTime;
    bool hasNewData = false;
    std::unique_ptr<KalmanFilter4D> kf_state;
    
    double ref_lat = 0.0;
    double ref_lon = 0.0;
    bool is_ref_set = false;
};

class TrajektoriAppKFv3 : public ItsG5BaseService {
public:
    virtual ~TrajektoriAppKFv3(); 
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
    
    void latLonToCartesian(double lat, double lon, double ref_lat, double ref_lon, double& x, double& y);
    void cartesianToLatLon(double x, double y, double ref_lat, double ref_lon, double& lat, double& lon);

    omnetpp::cMessage* mLogTimer = nullptr;
    omnetpp::cMessage* mPredictionTimer = nullptr;
    omnetpp::simsignal_t mCamReceivedSignal;
    omnetpp::SimTime mLogInterval;

    // Separate file streams for multi-horizon logging
    std::ofstream mCamLogFile;
    std::ofstream mPredLog1s;
    std::ofstream mPredLog2s;
    std::ofstream mPredLog3s;

    // Accumulators for Mean Absolute Error (MAE)
    double mSumAe1s = 0.0, mSumAe2s = 0.0, mSumAe3s = 0.0;
    long mCountAe1s = 0, mCountAe2s = 0, mCountAe3s = 0;
    
    std::map<long, AgentHistory> mOtherNodes;
};

} // namespace artery

#endif
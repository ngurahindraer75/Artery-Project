#ifndef ARTERY_TRAJEKTORIAPPKFV2_H_
#define ARTERY_TRAJEKTORIAPPKFV2_H_

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
};

struct PendingPrediction {
    double processing_time;
    double latest_cam_time;
    std::vector<double> kf_state_snapshot;
};

struct AgentHistory {
    std::deque<MovementData> history;
    omnetpp::simtime_t lastReceptionTime;
    bool hasNewData = false;
    std::deque<PendingPrediction> pending_queue;
    std::unique_ptr<KalmanFilter4D> kf_state;
};

class TrajektoriAppKFv2 : public ItsG5BaseService {
public:
    virtual ~TrajektoriAppKFv2(); 
    virtual void initialize() override;
    virtual void finish() override;

protected:
    virtual void handleMessage(omnetpp::cMessage* msg) override;
    virtual void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;

private:
    void logTrajectory();
    void runPredictions();
    std::string getNodeType();

    omnetpp::cMessage* mLogTimer = nullptr;
    omnetpp::cMessage* mPredictionTimer = nullptr;
    omnetpp::simsignal_t mCamReceivedSignal;
    omnetpp::SimTime mLogInterval;

    std::ofstream mLogFile;
    std::ofstream mPredictionLogFile;
    
    std::map<long, AgentHistory> mOtherNodes;

    double mPredictionHorizon = 1.0; 

    // Variabel Safety
    long mMyStationId = -1; 
    long long mTaiOffsetMod = -1;
};

} // namespace artery

#endif
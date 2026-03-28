#ifndef ARTERY_TRAJEKTORIAPPKFV1_H_
#define ARTERY_TRAJEKTORIAPPKFV1_H_

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

// --- USER LOGIC STEP 1 & 4: BUFFER FOR PENDING PREDICTION ---
// Struct to hold the 4D KF State Snapshot while waiting for the future
struct PendingPrediction {
    double processing_time;
    double latest_cam_time;
    std::vector<double> kf_state_snapshot; 
    bool is_active = false;
};

struct AgentHistory {
    std::deque<MovementData> history;
    omnetpp::simtime_t lastReceptionTime;
    bool hasNewData = false;

    // The asynchronous prediction waiting to be validated in the next cycle
    PendingPrediction pending; 
    std::unique_ptr<KalmanFilter4D> kf_state;
};

class TrajektoriAppKFv1 : public ItsG5BaseService {
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
    
    // Core function to execute Kalman Filter with Asynchronous Pipeline
    void runPredictions();

    omnetpp::cMessage* mLogTimer = nullptr;
    omnetpp::simsignal_t mCamReceivedSignal;
    omnetpp::SimTime mLogInterval;

    std::ofstream mLogFile;
    std::map<long, AgentHistory> mOtherNodes;

    omnetpp::cMessage* mPredictionTimer = nullptr;
    std::ofstream mPredictionLogFile;
    
    // Variables to accumulate Mean Absolute Error (MAE)
    double mSumAE = 0.0;
    long mCountAE = 0;
};

} // namespace artery

#endif /* ARTERY_TRAJEKTORIAPPKFV1_H_ */
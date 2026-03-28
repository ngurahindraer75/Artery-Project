#ifndef ARTERY_TRAJEKTORIAPP_H_
#define ARTERY_TRAJEKTORIAPP_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include <omnetpp/simtime.h>
#include <deque>
#include <fstream>
#include <string>
#include <map>
#include <vector>

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

// --- USER LOGIC STEP 1 & 3: BUFFER FOR PENDING PREDICTION ---
// Struct to hold the "Snapshot" and Training Data while waiting for the future
struct PendingPrediction {
    double processing_time;
    double latest_cam_time;
    std::vector<MovementData> train_data; 
    bool is_active = false;
};

struct AgentHistory {
    std::deque<MovementData> history;
    omnetpp::simtime_t lastReceptionTime;
    bool hasNewData = false;
    
    // The asynchronous prediction waiting to be validated in the next cycle
    PendingPrediction pending; 
};

class TrajektoriApp : public ItsG5BaseService {
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
    
    // Core function to execute Linear Regression with Asynchronous Pipeline
    void runLinearRegression();

    omnetpp::cMessage* mLogTimer = nullptr;
    omnetpp::simsignal_t mCamReceivedSignal;
    omnetpp::SimTime mLogInterval;
    
    std::ofstream mLogFile;
    std::map<long, AgentHistory> mOtherNodes;

    omnetpp::cMessage* mPredictionTimer = nullptr;
    std::ofstream mCoefficientLogFile;

    // Variables to accumulate Mean Absolute Error (MAE)
    double mSumAE = 0.0;
    long mCountAE = 0;
};

} // namespace artery

#endif /* ARTERY_TRAJEKTORIAPP_H_ */
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

// Structure to record each CAM data point received by the observer
struct MovementData {
    long gen_delta_time_raw;       // Raw generationDeltaTime from ETSI satellite
    double cam_received_time;      // Local time when CAM was received (seconds)
    double calculated_delay;       // Network delay extracted from payload (seconds)
    omnetpp::simtime_t timestamp;  // Absolute time reconstructed (CAM_Generation_Time)
    double latitude;
    double longitude;
    double speed_mps;
    double heading_degree;         // Added to ensure data format consistency
};

// Structure to buffer the movement history without past prediction queues
struct AgentHistory {
    std::deque<MovementData> history;
    omnetpp::simtime_t lastReceptionTime;
    bool hasNewData = false;

    // Safety marker to prevent redundant extrapolation for the same latest CAM data
    double last_used_absolute_time = -1.0; 

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
    
    // Core function to execute instant prediction printing
    void runPredictions();

    omnetpp::cMessage* mLogTimer = nullptr;
    omnetpp::simsignal_t mCamReceivedSignal;
    omnetpp::SimTime mLogInterval;

    std::ofstream mLogFile;
    std::map<long, AgentHistory> mOtherNodes;

    omnetpp::cMessage* mPredictionTimer = nullptr;
    std::ofstream mPredictionLogFile;
};

} // namespace artery

#endif /* ARTERY_TRAJEKTORIAPPKFV1_H_ */
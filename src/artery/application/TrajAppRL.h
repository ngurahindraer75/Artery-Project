/**
 * @file V4_TrajAppRL.h
 * @brief Header file for Trajectory Prediction Application using Linear Regression.
 * @details UPGRADED TO V4: Omnidirectional Tracking. All observers track all targets.
 *          Output files and MAE accumulators are strictly branched based on TARGET type.
 */

#ifndef ARTERY_TRAJAPPRL_H_
#define ARTERY_TRAJAPPRL_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/CaObject.h"
#include <omnetpp.h>
#include <omnetpp/simtime.h>
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <map>

namespace artery {

struct MovementDataRL {
    long gen_delta_time_raw;
    double cam_received_time;
    double calculated_delay;
    double timestamp; 
    double lat;
    double lon;
};

struct PendingPredictionRL {
    double processing_time;
    double latest_cam_time;
    double base_cam_lat;
    double base_cam_lon;
    double slope_lat;
    double intercept_lat;
    double slope_lon;
    double intercept_lon;
    bool eval_1s_done = false;
    bool eval_2s_done = false;
    bool eval_3s_done = false;
};

struct AgentHistoryRL {
    std::string target_type; // NEW: Identity marker for branched evaluation
    std::deque<MovementDataRL> history;
    std::deque<PendingPredictionRL> pending_queue;
    double last_prediction_time = -1.0; 
};

class TrajAppRL : public ItsG5BaseService {
public:
    virtual ~TrajAppRL() override;

protected:
    virtual void initialize() override;
    virtual void finish() override;
    virtual void handleMessage(omnetpp::cMessage* msg) override;
    virtual void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;

private:
    std::string getNodeType();
    void takeRlSnapshot();
    void evaluatePendingPredictionsRL(long targetId, AgentHistoryRL& hist_struct);

    std::map<long, AgentHistoryRL> mOtherNodes;
    
    // NEW: Dual Output Streams
    std::ofstream mCamLogVeh, mCamLogPed;
    std::ofstream mPredLog1sVeh, mPredLog1sPed;
    std::ofstream mPredLog2sVeh, mPredLog2sPed;
    std::ofstream mPredLog3sVeh, mPredLog3sPed;
    
    omnetpp::simsignal_t mCamReceivedSignal;
    omnetpp::cMessage* mLogTimer = nullptr;
    omnetpp::cMessage* mPredictionTimer = nullptr;
    omnetpp::SimTime mLogInterval;

    // NEW: Dual MAE Accumulators
    double mSumAe1sVeh = 0.0, mSumAe2sVeh = 0.0, mSumAe3sVeh = 0.0;
    long mCountAe1sVeh = 0, mCountAe2sVeh = 0, mCountAe3sVeh = 0;
    
    double mSumAe1sPed = 0.0, mSumAe2sPed = 0.0, mSumAe3sPed = 0.0;
    long mCountAe1sPed = 0, mCountAe2sPed = 0, mCountAe3sPed = 0;
};

} // namespace artery
#endif
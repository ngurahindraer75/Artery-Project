/**
 * @file TrajAppRL.h
 * @brief Header file for Trajectory Prediction Application using Linear Regression.
 * @details Evaluates ETSI ITS-G5 CAMs using Ordinary Least Squares (OLS) regression.
 *          Implements a cyclic prediction timer and strict ISO/ETSI-compliant payload 
 *          extraction to prevent data leakage and race conditions.
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

/**
 * @struct MovementDataRL
 * @brief Stores historical trajectory points for Linear Regression analysis.
 */
struct MovementDataRL {
    double timestamp;
    double lat;
    double lon;
};

/**
 * @struct PendingPredictionRL
 * @brief Holds calculated LR coefficients to evaluate future Absolute Error.
 */
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

/**
 * @struct AgentHistoryRL
 * @brief Maintains tracking memory queue and cyclic timer for the target node.
 */
struct AgentHistoryRL {
    std::deque<MovementDataRL> history;
    std::deque<PendingPredictionRL> pending_queue;
    double last_prediction_time = -1.0; // Cyclic timer initialization
};

/**
 * @class TrajAppRL
 * @brief OMNeT++ V2X Application for Trajectory Prediction via Linear Regression.
 */
class TrajAppRL : public ItsG5BaseService {
public:
    virtual ~TrajAppRL() override;

protected:
    virtual void initialize() override;
    virtual void finish() override;
    virtual void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;

private:
    std::string getNodeType();
    
    // Global tracking memory for surrounding entities
    std::map<long, AgentHistoryRL> mOtherNodes;
    
    // Output File Streams
    std::ofstream mCamLogFile;
    std::ofstream mPredLog1s;
    std::ofstream mPredLog2s;
    std::ofstream mPredLog3s;
    
    // CAM reception signal identifier
    omnetpp::simsignal_t mCamReceivedSignal;
};

} // namespace artery

#endif
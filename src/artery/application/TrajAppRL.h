/**
 * @file TrajAppRL.h
 * @brief Header file for Trajectory Prediction Application using Linear Regression.
 * @details Defines the necessary data structures and class interfaces for tracking
 * target nodes and predicting future coordinates using Ordinary Least Squares (OLS).
 */

#ifndef ARTERY_TrajAppRL_H_
#define ARTERY_TrajAppRL_H_

#include "artery/application/ItsG5BaseService.h"
#include <omnetpp/simtime.h>
#include <deque>
#include <fstream>
#include <string>
#include <map>

namespace artery {

/**
 * @struct MovementDataRL
 * @brief Stores extracted and synchronized properties of a single received CAM.
 */
struct MovementDataRL {
    long gen_delta_time_raw;        ///< Raw GenerationDeltaTime from ASN.1 payload
    double cam_received_time;       ///< Exact simulation time when the packet was received
    double calculated_delay;        ///< Network transmission delay (Age of Information)
    omnetpp::simtime_t timestamp;   ///< Absolute creation time of the packet in OMNeT++
    double latitude;                ///< Absolute Latitude in microdegrees
    double longitude;               ///< Absolute Longitude in microdegrees
    double speed_mps;               ///< Speed in m/s extracted from basic vehicle container
    double heading_degree;          ///< Heading in degrees
};

/**
 * @struct PendingPredictionRL
 * @brief Holds the calculated Linear Regression coefficients waiting for future CAMs to evaluate Mean Absolute Error (MAE).
 */
struct PendingPredictionRL {
    double processing_time;         ///< The exact time the snapshot/prediction was taken
    double latest_cam_time;         ///< The timestamp of the most recent CAM used for prediction
    double base_cam_lat;            ///< Latitude at the time of prediction
    double base_cam_lon;            ///< Longitude at the time of prediction
    
    // Linear Regression Coefficients (Y = mX + C)
    double slope_lat;               ///< Slope (m) for Latitude progression
    double intercept_lat;           ///< Intercept (C) for Latitude progression
    double slope_lon;               ///< Slope (m) for Longitude progression
    double intercept_lon;           ///< Intercept (C) for Longitude progression
    
    // Multi-stage evaluation flags to ensure each horizon is evaluated exactly once
    bool eval_1s_done = false;
    bool eval_2s_done = false;
    bool eval_3s_done = false;
};

/**
 * @struct AgentHistoryRL
 * @brief Maintains the sliding window history and prediction queues for a specific target node.
 */
struct AgentHistoryRL {
    std::deque<MovementDataRL> history;           ///< Sliding window of recent CAMs (used for OLS fitting)
    std::deque<PendingPredictionRL> pending_queue;///< Queue of snapshots awaiting future CAMs for evaluation
    
    omnetpp::simtime_t lastReceptionTime;       ///< Tracks the last time a message was received from this target
    bool hasNewData = false;                    ///< Flag to trigger trajectory logging
};

/**
 * @class TrajAppRL
 * @brief OMNeT++ simple module class for ITS-G5 based trajectory prediction via Linear Regression.
 */
class TrajAppRL : public ItsG5BaseService {
public:
    virtual ~TrajAppRL(); 
    virtual void initialize() override;
    virtual void finish() override;

protected:
    virtual void handleMessage(omnetpp::cMessage* msg) override;
    virtual void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;

private:
    /**
     * @brief Logs the raw CAM data to the CSV file.
     */
    void logTrajectory();

    /**
     * @brief Captures the current trajectory history and calculates OLS coefficients.
     */
    void takeRlSnapshot();

    /**
     * @brief Evaluates the accuracy of pending predictions against newly arrived CAMs.
     * @param targetId The ID of the node being tracked.
     * @param hist_struct The history and pending queue structure for the target.
     */
    void evaluatePendingPredictionsRL(long targetId, AgentHistoryRL& hist_struct);

    /**
     * @brief Determines whether the hosting node is a Vehicle or a Person.
     * @return String literal "Vehicle", "Person", or "Unknown".
     */
    std::string getNodeType();

    omnetpp::cMessage* mLogTimer = nullptr;
    omnetpp::cMessage* mPredictionTimer = nullptr;
    omnetpp::simsignal_t mCamReceivedSignal;
    omnetpp::SimTime mLogInterval;

    // File streams for data logging
    std::ofstream mCamLogFile;
    std::ofstream mPredLog1s;
    std::ofstream mPredLog2s;
    std::ofstream mPredLog3s;

    // Accumulators for Mean Absolute Error (MAE) calculation
    double mSumAe1s = 0.0, mSumAe2s = 0.0, mSumAe3s = 0.0;
    long mCountAe1s = 0, mCountAe2s = 0, mCountAe3s = 0;
    
    // Hash map to track multiple targets simultaneously
    std::map<long, AgentHistoryRL> mOtherNodes;
};

} // namespace artery

#endif // ARTERY_TrajAppRL_H_
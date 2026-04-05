/**
 * @file TrajAppKF4D.h
 * @brief Header file for Trajectory Prediction Application using Kalman Filter 4D.
 * @details Evaluates ETSI ITS-G5 CAMs using a recursive 4D Kalman Filter (X, Y, Vx, Vy).
 *          Implements a cyclic prediction timer, equirectangular spatial projection, 
 *          nearest-neighbor time alignment, and strict cross-entity isolation.
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

/**
 * @struct MovementDataKF4D
 * @brief Stores historical trajectory points for Nearest Neighbor AE evaluation.
 */
struct MovementDataKF4D {
    double timestamp;
    double lat_raw;
    double lon_raw;
    double local_x;
    double local_y;
    double vel_x;
    double vel_y;
};

/**
 * @struct PendingPredictionKF4D
 * @brief Holds a snapshot of the KF State to evaluate future Absolute Error.
 */
struct PendingPredictionKF4D {
    double processing_time;
    double latest_cam_time;
    double base_cam_lat;
    double base_cam_lon;
    
    // PERBAIKAN: Dikembalikan ke tipe data 'double' (bukan std::vector)
    double state_x;
    double state_y;
    double vel_x;
    double vel_y;
    
    bool eval_1s_done = false;
    bool eval_2s_done = false;
    bool eval_3s_done = false;
};

/**
 * @struct AgentHistoryKF4D
 * @brief Maintains KF state, geographical origin, and tracking queues per entity.
 */
struct AgentHistoryKF4D {
    std::deque<MovementDataKF4D> history;
    std::deque<PendingPredictionKF4D> pending_queue;
    
    // Matriks State KF individual untuk setiap kendaraan/pejalan kaki
    std::unique_ptr<KalmanFilter4D> kf_state;
    
    double last_prediction_time = -1.0;
    double last_reception_time = -1.0;
    
    // Geographical Origin (Reference point for Cartesian projection)
    double ref_lat_raw = 0.0;
    double ref_lon_raw = 0.0;
    bool is_ref_set = false;
};

/**
 * @class TrajAppKF4D
 * @brief OMNeT++ V2X Application for Trajectory Prediction via Kalman Filter 4D.
 */
class TrajAppKF4D : public ItsG5BaseService {
public:
    virtual ~TrajAppKF4D() override;

protected:
    virtual void initialize() override;
    virtual void finish() override;
    virtual void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;

private:
    std::string getNodeType();
    
    // Spatial Coordinate Converters
    void latLonToCartesian(double lat_raw, double lon_raw, double ref_lat_raw, double ref_lon_raw, double& x, double& y);
    void cartesianToLatLon(double x, double y, double ref_lat_raw, double ref_lon_raw, double& lat_raw, double& lon_raw);
    
    // Global tracking memory mapped by Target Station ID
    std::map<long, AgentHistoryKF4D> mOtherNodes;
    
    // Output File Streams
    std::ofstream mCamLogFile;
    std::ofstream mPredLog1s;
    std::ofstream mPredLog2s;
    std::ofstream mPredLog3s;
    
    omnetpp::simsignal_t mCamReceivedSignal;
};

} // namespace artery

#endif
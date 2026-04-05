/**
 * @file TrajAppKF6D.cc
 * @brief Trajectory Prediction Application using Kalman Filter 6D.
 * @details Extracts precise ETSI ITS-G5 high-frequency kinematics (including acceleration). 
 *          Uses Cartesian Equirectangular projection and quadratic Newtonian physics 
 *          equations (m, m/s, m/s^2) for extrapolation. Maintains a 4-second memory 
 *          queue for Nearest-Neighbor time alignment against cyclic evaluation horizons.
 */

#include "artery/application/TrajAppKF6D.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <iostream>

using namespace omnetpp;

namespace artery {

// REQUIRED FOR OMNET++ TO RECOGNIZE THE C++ MODULE
Define_Module(TrajAppKF6D);

TrajAppKF6D::~TrajAppKF6D() {}

std::string TrajAppKF6D::getNodeType() {
    cModule* myNode = getParentModule()->getParentModule();
    if (myNode) {
        std::string type = myNode->getNedTypeName();
        if (type.find("Person") != std::string::npos || type.find("Pedestrian") != std::string::npos) {
            return "Pedestrian";
        }
    }
    return "Vehicle"; 
}

// -----------------------------------------------------------------------------------------
// SPATIAL CONVERTERS (0.1 Microdegree ETSI format <-> Local Cartesian Meters)
// -----------------------------------------------------------------------------------------
void TrajAppKF6D::latLonToCartesian(double lat_raw, double lon_raw, double ref_lat_raw, double ref_lon_raw, double& x, double& y) {
    const double R = 6371000.0; // Earth's radius in meters
    double lat_deg = lat_raw * 1e-7;
    double lon_deg = lon_raw * 1e-7;
    double ref_lat_deg = ref_lat_raw * 1e-7;
    double ref_lon_deg = ref_lon_raw * 1e-7;
    
    x = R * (lon_deg - ref_lon_deg) * (M_PI / 180.0) * std::cos(ref_lat_deg * M_PI / 180.0);
    y = R * (lat_deg - ref_lat_deg) * (M_PI / 180.0);
}

void TrajAppKF6D::cartesianToLatLon(double x, double y, double ref_lat_raw, double ref_lon_raw, double& lat_raw, double& lon_raw) {
    const double R = 6371000.0;
    double ref_lat_deg = ref_lat_raw * 1e-7;
    double ref_lon_deg = ref_lon_raw * 1e-7;
    
    double lat_deg = ref_lat_deg + (y / R) * (180.0 / M_PI);
    double lon_deg = ref_lon_deg + (x / (R * std::cos(ref_lat_deg * M_PI / 180.0))) * (180.0 / M_PI);
    
    lat_raw = lat_deg * 1e7;
    lon_raw = lon_deg * 1e7;
}

void TrajAppKF6D::initialize() {
    ItsG5BaseService::initialize();

    std::string nodeType = getNodeType();
    std::string targetLabel = (nodeType == "Vehicle") ? "pedestrian_prediction" : "vehicle_prediction";

    // 1. Raw CAM Data Log File Initialization (Added Acc_mps2)
    mCamLogFile.open("results/KF6D_CAM_data_" + targetLabel + ".csv", std::ios_base::out | std::ios_base::app);
    mCamLogFile.seekp(0, std::ios::end); 
    if (mCamLogFile.tellp() <= 0) {
        mCamLogFile << "GenDeltaTime_Raw;CAM_Received_Time;Calculated_Delay;CAM_Generation_Time;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Speed_mps;Heading;Acc_mps2\n";
        mCamLogFile.flush(); 
    }

    // 2. Horizon Prediction Log Initialization (Added Acc_X, Acc_Y)
    auto initPredLog = [&](std::ofstream& stream, const std::string& horizonStr) {
        stream.open("results/KF6D_" + horizonStr + "_coefficient_log_" + targetLabel + ".csv", std::ios_base::out | std::ios_base::app);
        stream.seekp(0, std::ios::end); 
        if (stream.tellp() <= 0) {
            stream << "Processing_Time(s);Latest_CAM_Time(s);Observer_ID;Base_CAM_Lat;Base_CAM_Lon;Target_Prediction_Time(s);Target_ID;Actual_Lat;Actual_Lon;State_X;State_Y;Vel_X;Vel_Y;Acc_X;Acc_Y;Pred_Lat;Pred_Lon;Lat_AE;Lon_AE;Total_AE\n";
            stream.flush(); 
        }
    };

    initPredLog(mPredLog1s, "1s");
    initPredLog(mPredLog2s, "2s");
    initPredLog(mPredLog3s, "3s");

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);
}

void TrajAppKF6D::finish() {
    if (mCamLogFile.is_open()) mCamLogFile.close();
    if (mPredLog1s.is_open()) mPredLog1s.close();
    if (mPredLog2s.is_open()) mPredLog2s.close();
    if (mPredLog3s.is_open()) mPredLog3s.close();
    ItsG5BaseService::finish();
}

void TrajAppKF6D::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    if (signalID == mCamReceivedSignal) {
        if (auto ca_obj = dynamic_cast<CaObject*>(obj)) {
            const vanetza::asn1::Cam& cam_wrapper = ca_obj->asn1();
            const CAM_t& cam_struct = *cam_wrapper;
            
            long observer_id = getParentModule()->getParentModule()->getId();
            long target_id = cam_struct.header.stationID;
            double current_time = simTime().dbl();
            
            // =========================================================================
            // DUAL-LAYER CROSS-ENTITY ISOLATION
            // =========================================================================
            const auto& basic = cam_struct.cam.camParameters.basicContainer;
            long target_station_type = basic.stationType;
            std::string target_type = "Vehicle"; 
            
            if (target_station_type == 1) {
                target_type = "Pedestrian";
            } else if (target_station_type == 0 || target_station_type == 5) {
                cModule* targetNode = getSimulation()->getModule(target_id);
                if (targetNode) {
                    std::string nedType = targetNode->getNedTypeName();
                    if (nedType.find("Person") != std::string::npos || nedType.find("Pedestrian") != std::string::npos) {
                        target_type = "Pedestrian";
                    }
                }
            }

            std::string observer_type = getNodeType();

            if (observer_type == target_type) {
                return; // Prevent data leakage between same entity types
            }
            
            double current_lat_raw = static_cast<double>(basic.referencePosition.latitude); 
            double current_lon_raw = static_cast<double>(basic.referencePosition.longitude);
            
            // =========================================================================
            // ETSI ITS-G5 HIGH-FREQUENCY PAYLOAD EXTRACTION (Speed, Heading, Acceleration)
            // =========================================================================
            double speed_mps = 0.0;
            double heading_deg = 0.0;
            double acc_mps2 = 0.0;

            const auto& hf_container = cam_struct.cam.camParameters.highFrequencyContainer;
            if (hf_container.present == HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
                const auto& bvhf = hf_container.choice.basicVehicleContainerHighFrequency;
                
                // Speed: 0.01 m/s, Heading: 0.1 degree, Longitudinal Acceleration: 0.1 m/s^2
                speed_mps = static_cast<double>(bvhf.speed.speedValue) * 0.01;
                heading_deg = static_cast<double>(bvhf.heading.headingValue) * 0.1;
                acc_mps2 = static_cast<double>(bvhf.longitudinalAcceleration.longitudinalAccelerationValue) * 0.1;
            }
            
            // Vector Decomposition based on Heading Angle
            double heading_rad = heading_deg * M_PI / 180.0;
            double vel_x = speed_mps * std::sin(heading_rad);
            double vel_y = speed_mps * std::cos(heading_rad);
            double acc_x = acc_mps2 * std::sin(heading_rad);
            double acc_y = acc_mps2 * std::cos(heading_rad);
            
            // ATOMIC LOGGING FOR RAW CAM DATA
            std::stringstream cam_ss;
            cam_ss << std::fixed << std::setprecision(12) 
                   << cam_struct.cam.generationDeltaTime << ";" 
                   << current_time << ";" << 0.0 << ";" << current_time << ";" 
                   << observer_id << ";" << target_id << ";"
                   << current_lat_raw << ";" << current_lon_raw << ";" 
                   << speed_mps << ";" << heading_deg << ";" << acc_mps2 << "\n";
            mCamLogFile << cam_ss.str();
            mCamLogFile.flush();

            // =========================================================================
            // KALMAN FILTER 6D INSTANTIATION & PREDICT-CORRECT CYCLE
            // =========================================================================
            AgentHistoryKF6D& agent = mOtherNodes[target_id];
            
            // Define geographical origin for Cartesian projection upon first sight
            if (!agent.is_ref_set) {
                agent.ref_lat_raw = current_lat_raw;
                agent.ref_lon_raw = current_lon_raw;
                agent.is_ref_set = true;
                
                agent.kf_state = std::make_unique<KalmanFilter6D>(); 
                // Initialize 6D State Vector [X, Y, Vx, Vy, Ax, Ay]
                agent.kf_state->init({0.0, 0.0, vel_x, vel_y, acc_x, acc_y}); 
            }
            
            double local_x = 0.0, local_y = 0.0;
            latLonToCartesian(current_lat_raw, current_lon_raw, agent.ref_lat_raw, agent.ref_lon_raw, local_x, local_y);

            double dt = (agent.last_reception_time > 0.0) ? (current_time - agent.last_reception_time) : 0.1;
            if (dt > 0.0 && agent.kf_state) {
                agent.kf_state->predict(dt);
                // Measurement update 6D Vector [X, Y, Vx, Vy, Ax, Ay]
                agent.kf_state->update({local_x, local_y, vel_x, vel_y, acc_x, acc_y}); 
            }
            agent.last_reception_time = current_time;

            // NEAREST NEIGHBOR EVALUATION QUEUE (Sliding Window: 4 Seconds)
            agent.history.push_back({current_time, current_lat_raw, current_lon_raw, local_x, local_y, vel_x, vel_y, acc_x, acc_y});
            double evaluation_window = 4.0; 
            while (agent.history.size() > 1 && (current_time - agent.history.front().timestamp > evaluation_window)) {
                agent.history.pop_front();
            }

            // =========================================================================
            // CYCLIC PREDICTION TIMER (1.0s Horizon Snapshots)
            // =========================================================================
            double prediction_interval = 1.0; 
            if (agent.last_prediction_time < 0.0 || (current_time - agent.last_prediction_time) >= prediction_interval) {
                if (agent.kf_state) {
                    PendingPredictionKF6D snap;
                    snap.processing_time = current_time;
                    snap.latest_cam_time = current_time;
                    snap.base_cam_lat = current_lat_raw;
                    snap.base_cam_lon = current_lon_raw;

                    // Retrieving State Vector as std::vector<double> [X, Y, Vx, Vy, Ax, Ay]
                    std::vector<double> current_state = agent.kf_state->getState();
                    if(current_state.size() >= 6) {
                        snap.state_x = current_state[0];
                        snap.state_y = current_state[1];
                        snap.vel_x = current_state[2];
                        snap.vel_y = current_state[3];
                        snap.acc_x = current_state[4];
                        snap.acc_y = current_state[5];
                    }

                    agent.pending_queue.push_back(snap);
                    agent.last_prediction_time = current_time;
                }
            }
            
            // =========================================================================
            // ABSOLUTE ERROR EVALUATION (Nearest Neighbor Time Alignment & GLBB Extrapolation)
            // =========================================================================
            auto it = agent.pending_queue.begin();
            while (it != agent.pending_queue.end()) {
                bool all_done = true;

                auto evaluate_horizon = [&](double horizon, bool& is_done, std::ofstream& logFile) {
                    if (is_done) return;
                    
                    double ideal_target_time = it->latest_cam_time + horizon;

                    if (current_time >= ideal_target_time) {
                        const MovementDataKF6D* best_match = &agent.history.back(); 

                        // Find the nearest CAM neighbor to the ideal prediction time
                        if (current_time > ideal_target_time && agent.history.size() > 1) {
                            const MovementDataKF6D& prev_cam = agent.history[agent.history.size() - 2];
                            double diff_after = current_time - ideal_target_time;
                            double diff_before = ideal_target_time - prev_cam.timestamp;
                            if (diff_before <= diff_after) {
                                best_match = &prev_cam;
                            }
                        }

                        // 1. Cartesian Extrapolation using Constant Acceleration Kinematic Equation
                        // Pos = Pos_0 + (V * t) + (0.5 * a * t^2)
                        double dt_pred = best_match->timestamp - it->processing_time;
                        double dt_squared = dt_pred * dt_pred;
                        
                        double pred_x = it->state_x + (it->vel_x * dt_pred) + (0.5 * it->acc_x * dt_squared);
                        double pred_y = it->state_y + (it->vel_y * dt_pred) + (0.5 * it->acc_y * dt_squared);
                        
                        // 2. Conversion back to Geospatial standard
                        double pred_lat_raw = 0.0, pred_lon_raw = 0.0;
                        cartesianToLatLon(pred_x, pred_y, agent.ref_lat_raw, agent.ref_lon_raw, pred_lat_raw, pred_lon_raw);
                        
                        // 3. Error Computation
                        double lat_ae = std::abs(best_match->lat_raw - pred_lat_raw);
                        double lon_ae = std::abs(best_match->lon_raw - pred_lon_raw);
                        double total_ae = std::sqrt((lat_ae * lat_ae) + (lon_ae * lon_ae));
                        
                        std::stringstream ss;
                        ss << std::fixed << std::setprecision(12) 
                           << it->processing_time << ";" << it->latest_cam_time << ";" 
                           << observer_id << ";" << it->base_cam_lat << ";" << it->base_cam_lon << ";" 
                           << best_match->timestamp << ";" << target_id << ";" 
                           << best_match->lat_raw << ";" << best_match->lon_raw << ";" 
                           << it->state_x << ";" << it->state_y << ";" << it->vel_x << ";" << it->vel_y << ";" << it->acc_x << ";" << it->acc_y << ";"
                           << pred_lat_raw << ";" << pred_lon_raw << ";" 
                           << lat_ae << ";" << lon_ae << ";" << total_ae << "\n";
                        
                        logFile << ss.str(); 
                        logFile.flush();
                        is_done = true;
                    }
                };

                evaluate_horizon(1.0, it->eval_1s_done, mPredLog1s);
                evaluate_horizon(2.0, it->eval_2s_done, mPredLog2s);
                evaluate_horizon(3.0, it->eval_3s_done, mPredLog3s);

                if (!it->eval_1s_done || !it->eval_2s_done || !it->eval_3s_done) all_done = false;
                if (current_time > it->latest_cam_time + 4.5) all_done = true; // Drop dead targets

                if (all_done) {
                    it = agent.pending_queue.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }
}

} // namespace artery
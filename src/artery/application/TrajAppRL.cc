/**
 * @file TrajAppRL.cc
 * @brief Trajectory Prediction Application using Linear Regression.
 * @details Implements a 1-second cyclic prediction interval matching the baseline thesis. 
 *          Restores the Nearest-Neighbor Time Alignment logic to precisely match the 
 *          "Ideal Target Time" with the closest Actual CAM reception time.
 */

#include "artery/application/TrajAppRL.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace omnetpp;

namespace artery {

// REQUIRED FOR OMNET++ TO RECOGNIZE THE C++ MODULE
Define_Module(TrajAppRL);

TrajAppRL::~TrajAppRL() {}

std::string TrajAppRL::getNodeType() {
    cModule* myNode = getParentModule()->getParentModule();
    if (myNode) {
        std::string type = myNode->getNedTypeName();
        if (type.find("Person") != std::string::npos || type.find("Pedestrian") != std::string::npos) {
            return "Pedestrian";
        }
    }
    return "Vehicle"; 
}

void TrajAppRL::initialize() {
    ItsG5BaseService::initialize();

    std::string nodeType = getNodeType();
    std::string targetLabel = (nodeType == "Vehicle") ? "pedestrian_prediction" : "vehicle_prediction";

    // Initialize Raw CAM Data Log File
    mCamLogFile.open("results/RL_CAM_data_" + targetLabel + ".csv", std::ios_base::out | std::ios_base::app);
    mCamLogFile.seekp(0, std::ios::end); 
    if (mCamLogFile.tellp() <= 0) {
        mCamLogFile << "GenDeltaTime_Raw;CAM_Received_Time;Calculated_Delay;CAM_Generation_Time;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Speed_mps;Heading\n";
        mCamLogFile.flush(); 
    }

    // Initialize the 3 Separate Prediction Horizon Files
    auto initPredLog = [&](std::ofstream& stream, const std::string& horizonStr) {
        stream.open("results/RL_" + horizonStr + "_coefficient_log_" + targetLabel + ".csv", std::ios_base::out | std::ios_base::app);
        stream.seekp(0, std::ios::end); 
        if (stream.tellp() <= 0) {
            stream << "Processing_Time(s);Latest_CAM_Time(s);Observer_ID;Base_CAM_Lat;Base_CAM_Lon;Target_Prediction_Time(s);Target_ID;Actual_Lat;Actual_Lon;Slope_Lat;Intercept_Lat;Pred_Lat;Slope_Lon;Intercept_Lon;Pred_Lon;Lat_AE;Lon_AE;Total_AE\n";
            stream.flush(); 
        }
    };

    initPredLog(mPredLog1s, "1s");
    initPredLog(mPredLog2s, "2s");
    initPredLog(mPredLog3s, "3s");

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);
}

void TrajAppRL::finish() {
    if (mCamLogFile.is_open()) mCamLogFile.close();
    if (mPredLog1s.is_open()) mPredLog1s.close();
    if (mPredLog2s.is_open()) mPredLog2s.close();
    if (mPredLog3s.is_open()) mPredLog3s.close();
    ItsG5BaseService::finish();
}

void TrajAppRL::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    if (signalID == mCamReceivedSignal) {
        if (auto ca_obj = dynamic_cast<CaObject*>(obj)) {
            const vanetza::asn1::Cam& cam_wrapper = ca_obj->asn1();
            const CAM_t& cam_struct = *cam_wrapper;
            
            long observer_id = getParentModule()->getParentModule()->getId();
            long target_id = cam_struct.header.stationID;
            double current_time = simTime().dbl();
            
            // STRICT ETSI TS 102 894-2 STATION TYPE CLASSIFICATION
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

            // CROSS-ENTITY TARGET FILTER
            if (observer_type == target_type) {
                return; // Drops CAMs from identical entity types
            }
            
            double current_lat = static_cast<double>(basic.referencePosition.latitude); 
            double current_lon = static_cast<double>(basic.referencePosition.longitude);
            
            // ETSI ITS-G5 HIGH-FREQUENCY PAYLOAD EXTRACTION
            double speed_raw = 0.0;
            double heading_raw = 0.0;

            const auto& hf_container = cam_struct.cam.camParameters.highFrequencyContainer;
            if (hf_container.present == HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
                const auto& bvhf = hf_container.choice.basicVehicleContainerHighFrequency;
                speed_raw = static_cast<double>(bvhf.speed.speedValue) * 0.01;
                heading_raw = static_cast<double>(bvhf.heading.headingValue) * 0.1;
            }
            
            // ATOMIC LOGGING FOR RAW CAM DATA
            std::stringstream cam_ss;
            cam_ss << std::fixed << std::setprecision(12) 
                   << cam_struct.cam.generationDeltaTime << ";" 
                   << current_time << ";" 
                   << 0.0 << ";" 
                   << current_time << ";" 
                   << observer_id << ";" << target_id << ";"
                   << current_lat << ";" << current_lon << ";" 
                   << speed_raw << ";" << heading_raw << "\n";
            
            mCamLogFile << cam_ss.str();
            mCamLogFile.flush();

            // QUEUE MANAGEMENT
            AgentHistoryRL& agent = mOtherNodes[target_id];
            agent.history.push_back({current_time, current_lat, current_lon});
            
            double sliding_window_time = 1.0; 
            while (agent.history.size() > 2 && (current_time - agent.history.front().timestamp > sliding_window_time)) {
                agent.history.pop_front();
            }

            // =========================================================================
            // CYCLIC PREDICTION TIMER (Baseline Flowchart Timer)
            // =========================================================================
            double prediction_interval = 1.0; 

            if (agent.last_prediction_time < 0.0 || (current_time - agent.last_prediction_time) >= prediction_interval) {
                if (agent.history.size() >= 2) {
                    double sum_x = 0, sum_y_lat = 0, sum_y_lon = 0;
                    double sum_xy_lat = 0, sum_xy_lon = 0, sum_x_squared = 0;
                    double n = static_cast<double>(agent.history.size());

                    for (const auto& point : agent.history) {
                        sum_x += point.timestamp;
                        sum_y_lat += point.lat;
                        sum_y_lon += point.lon;
                        sum_xy_lat += point.timestamp * point.lat;
                        sum_xy_lon += point.timestamp * point.lon;
                        sum_x_squared += point.timestamp * point.timestamp;
                    }

                    double denominator = (n * sum_x_squared) - (sum_x * sum_x);

                    if (denominator != 0.0) {
                        PendingPredictionRL snap;
                        snap.processing_time = current_time;
                        snap.latest_cam_time = agent.history.back().timestamp;
                        snap.base_cam_lat = current_lat;
                        snap.base_cam_lon = current_lon;

                        snap.slope_lat = ((n * sum_xy_lat) - (sum_x * sum_y_lat)) / denominator;
                        snap.intercept_lat = (sum_y_lat - (snap.slope_lat * sum_x)) / n;
                        
                        snap.slope_lon = ((n * sum_xy_lon) - (sum_x * sum_y_lon)) / denominator;
                        snap.intercept_lon = (sum_y_lon - (snap.slope_lon * sum_x)) / n;

                        agent.pending_queue.push_back(snap);
                        agent.last_prediction_time = current_time;
                    }
                }
            }
            
            // =========================================================================
            // ABSOLUTE ERROR EVALUATION (Nearest Neighbor Time Alignment)
            // =========================================================================
            auto it = agent.pending_queue.begin();
            while (it != agent.pending_queue.end()) {
                bool all_done = true;

                // Inline lambda helper to evaluate specific prediction horizons cleanly
                auto evaluate_horizon = [&](double horizon, bool& is_done, std::ofstream& logFile) {
                    if (is_done) return;
                    
                    // Priority 1: Define Ideal Target Time
                    double ideal_target_time = it->latest_cam_time + horizon;

                    // Priority 2: Execute precisely when CAM time has passed the Ideal Time
                    if (current_time >= ideal_target_time) {
                        const MovementDataRL* best_match = &agent.history.back(); // Current CAM

                        // Select the nearest neighbor CAM to the ideal target time
                        if (current_time > ideal_target_time && agent.history.size() > 1) {
                            const MovementDataRL& prev_cam = agent.history[agent.history.size() - 2];
                            
                            double diff_after = current_time - ideal_target_time;
                            double diff_before = ideal_target_time - prev_cam.timestamp;
                            
                            // If the previous CAM was closer in time, use it instead!
                            if (diff_before <= diff_after) {
                                best_match = &prev_cam;
                            }
                        }

                        double target_prediction_time = best_match->timestamp;
                        double actual_lat = best_match->lat;
                        double actual_lon = best_match->lon;

                        // Extrapolate predicted coordinates dynamically based on the exact Target Prediction Time
                        double pred_lat = it->intercept_lat + (it->slope_lat * target_prediction_time);
                        double pred_lon = it->intercept_lon + (it->slope_lon * target_prediction_time);
                        
                        double lat_ae = std::abs(actual_lat - pred_lat);
                        double lon_ae = std::abs(actual_lon - pred_lon);
                        double total_ae = std::sqrt((lat_ae * lat_ae) + (lon_ae * lon_ae));
                        
                        // Atomic logging
                        std::stringstream ss;
                        ss << std::fixed << std::setprecision(12) 
                           << it->processing_time << ";" << it->latest_cam_time << ";" 
                           << observer_id << ";" << it->base_cam_lat << ";" << it->base_cam_lon << ";" 
                           << target_prediction_time << ";" << target_id << ";" 
                           << actual_lat << ";" << actual_lon << ";" 
                           << it->slope_lat << ";" << it->intercept_lat << ";" << pred_lat << ";" 
                           << it->slope_lon << ";" << it->intercept_lon << ";" << pred_lon << ";" 
                           << lat_ae << ";" << lon_ae << ";" << total_ae << "\n";
                        
                        logFile << ss.str(); 
                        logFile.flush();
                        
                        is_done = true;
                    }
                };

                // Execute parallel evaluation for 1s, 2s, and 3s horizons
                evaluate_horizon(1.0, it->eval_1s_done, mPredLog1s);
                evaluate_horizon(2.0, it->eval_2s_done, mPredLog2s);
                evaluate_horizon(3.0, it->eval_3s_done, mPredLog3s);

                if (!it->eval_1s_done || !it->eval_2s_done || !it->eval_3s_done) {
                    all_done = false;
                }

                // Failsafe timeout: Drop snapshots if Target stops transmitting CAMs
                if (current_time > it->latest_cam_time + 4.5) {
                    all_done = true;
                }

                // Erase from RAM when evaluated
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
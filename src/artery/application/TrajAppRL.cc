/**
 * @file V4_TrajAppRL.cc
 * @brief Trajectory Prediction Application using Linear Regression.
 * @details UPGRADED TO V4: Omnidirectional Tracking. Removes Cross-Entity Isolation 
 *          to allow unrestricted P2P, V2V, P2V, and V2P tracking simultaneously. 
 *          Incorporates absolute time sync (Modulus 65536). Output is target-branched.
 */

#include "artery/application/TrajAppRL.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include "artery/application/Middleware.h"
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <omnetpp/cpacket.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

namespace artery {
using namespace omnetpp;

Define_Module(TrajAppRL);

TrajAppRL::~TrajAppRL() {
    if (mCamLogVeh.is_open()) mCamLogVeh.close();
    if (mCamLogPed.is_open()) mCamLogPed.close();
    if (mPredLog1sVeh.is_open()) mPredLog1sVeh.close();
    if (mPredLog1sPed.is_open()) mPredLog1sPed.close();
    if (mPredLog2sVeh.is_open()) mPredLog2sVeh.close();
    if (mPredLog2sPed.is_open()) mPredLog2sPed.close();
    if (mPredLog3sVeh.is_open()) mPredLog3sVeh.close();
    if (mPredLog3sPed.is_open()) mPredLog3sPed.close();

    if (mLogTimer) { cancelAndDelete(mLogTimer); mLogTimer = nullptr; }
    if (mPredictionTimer) { cancelAndDelete(mPredictionTimer); mPredictionTimer = nullptr; }
}

std::string TrajAppRL::getNodeType() {
    cModule* myNode = getParentModule()->getParentModule();
    if (myNode) {
        std::string type = myNode->getNedTypeName();
        if (type.find("Person") != std::string::npos || type.find("Pedestrian") != std::string::npos) return "Pedestrian";
    }
    return "Vehicle";
}

void TrajAppRL::initialize() {
    ItsG5BaseService::initialize();
    mLogInterval = par("logInterval");

    auto initCamLog = [&](std::ofstream& stream, const std::string& targetLabel) {
        stream.open("results/V4_RL_CAM_Target_" + targetLabel + ".csv", std::ios_base::out | std::ios_base::app);
        
        stream.seekp(0, std::ios::end); 
        
        if (stream.tellp() <= 0) {
            stream << "GenDeltaTime_Raw;CAM_Received_Time;Calculated_Delay;CAM_Generation_Time;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Speed_mps;Heading\n";
            stream.flush(); // Immediately save to disk
        }
    };
    initCamLog(mCamLogVeh, "Vehicle");
    initCamLog(mCamLogPed, "Pedestrian");

    auto initPredLog = [&](std::ofstream& stream, const std::string& targetLabel, const std::string& horizonStr) {
        stream.open("results/V4_RL_" + horizonStr + "s_coefficient_log_Target_" + targetLabel + ".csv", std::ios_base::out | std::ios_base::app);
        
        stream.seekp(0, std::ios::end); 
        
        if (stream.tellp() <= 0) {
            stream << "Processing_Time(s);Latest_CAM_Time(s);Observer_ID;Base_CAM_Lat;Base_CAM_Lon;Target_Prediction_Time(s);Target_ID;Actual_Lat;Actual_Lon;Slope_Lat;Intercept_Lat;Pred_Lat;Slope_Lon;Intercept_Lon;Pred_Lon;Lat_AE;Lon_AE;Total_AE\n";
            stream.flush(); // Immediately save to disk
        }
    };
    
    initPredLog(mPredLog1sVeh, "Vehicle", "1");
    initPredLog(mPredLog2sVeh, "Vehicle", "2");
    initPredLog(mPredLog3sVeh, "Vehicle", "3");
    
    initPredLog(mPredLog1sPed, "Pedestrian", "1");
    initPredLog(mPredLog2sPed, "Pedestrian", "2");
    initPredLog(mPredLog3sPed, "Pedestrian", "3");

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);

    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);
    mPredictionTimer = new cMessage("predictionTimer");
    scheduleAt(simTime() + 1.0, mPredictionTimer);
}

void TrajAppRL::handleMessage(cMessage* msg) {
    if (msg == mLogTimer) {
        scheduleAt(simTime() + mLogInterval, mLogTimer);
        return;
    }
    if (msg == mPredictionTimer) {
        takeRlSnapshot();
        scheduleAt(simTime() + 1.0, mPredictionTimer);
        return;
    }
    delete msg;
}

void TrajAppRL::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    if (signalID != mCamReceivedSignal) return;

    auto ca_obj = dynamic_cast<CaObject*>(obj);
    if (!ca_obj) return;

    const vanetza::asn1::Cam& cam_wrapper = ca_obj->asn1();
    const CAM_t& cam_struct = *cam_wrapper;

    long observer_id = getParentModule()->getParentModule()->getId();
    long target_id = cam_struct.header.stationID;
    
    if (observer_id == target_id) return; // Prevent self-reception

    double current_time = simTime().dbl();

    // IDENTIFY TARGET TYPE
    const auto& basic = cam_struct.cam.camParameters.basicContainer;
    long target_station_type = basic.stationType;
    std::string target_type = "Vehicle";

    if (target_station_type == 1) {
        target_type = "Pedestrian";
    } else if (target_station_type == 0 || target_station_type == 5) {
        cModule* targetNode = getSimulation()->getModule(target_id);
        if (targetNode) {
            std::string nedType = targetNode->getNedTypeName();
            if (nedType.find("Person") != std::string::npos || nedType.find("Pedestrian") != std::string::npos) target_type = "Pedestrian";
        }
    }

    double current_lat = static_cast<double>(basic.referencePosition.latitude);
    double current_lon = static_cast<double>(basic.referencePosition.longitude);

    // KINEMATICS EXTRACTION
    double speed_mps = 0.0, heading_deg = 0.0;
    const auto& hf_container = cam_struct.cam.camParameters.highFrequencyContainer;
    if (hf_container.present == HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
        const auto& bvhf = hf_container.choice.basicVehicleContainerHighFrequency;
        speed_mps = static_cast<double>(bvhf.speed.speedValue) * 0.01;
        heading_deg = static_cast<double>(bvhf.heading.headingValue) * 0.1;
    }

    // ETSI TIME SYNCHRONIZATION (Modulus 65536)
    long genDeltaTime_ms = cam_struct.cam.generationDeltaTime;
    simtime_t time_receive = simTime();
    long long current_time_ms = time_receive.inUnit(SIMTIME_MS);

    static long long tai_offset_mod = -1;
    if (tai_offset_mod == -1) {
        const cPacket* packet = dynamic_cast<const cPacket*>(details);
        long long true_creation_ms = (packet ? packet->getCreationTime() : simTime()).inUnit(SIMTIME_MS);
        tai_offset_mod = (genDeltaTime_ms - true_creation_ms) % 65536;
        if (tai_offset_mod < 0) tai_offset_mod += 65536;
    }

    long current_mod = (current_time_ms + tai_offset_mod) % 65536;
    long delay_ms = current_mod - genDeltaTime_ms;

    if (delay_ms < -32768) delay_ms += 65536; 
    else if (delay_ms > 32768) delay_ms -= 65536; 
    if (delay_ms < 0) delay_ms = 0; 

    double calculated_delay = delay_ms / 1000.0;
    double time_send_absolut = (current_time_ms - delay_ms) / 1000.0;

    // ATOMIC LOGGING (Branched)
    std::stringstream cam_ss;
    cam_ss << std::fixed << std::setprecision(12)
           << genDeltaTime_ms << ";" << current_time << ";" << calculated_delay << ";"
           << time_send_absolut << ";" << observer_id << ";" << target_id << ";"
           << current_lat << ";" << current_lon << ";"
           << speed_mps << ";" << heading_deg << "\n";
           
    if (target_type == "Vehicle") {
        mCamLogVeh << cam_ss.str(); mCamLogVeh.flush();
    } else {
        mCamLogPed << cam_ss.str(); mCamLogPed.flush();
    }

    // MEMORY INSTANTIATION
    AgentHistoryRL& agent = mOtherNodes[target_id];
    agent.target_type = target_type;

    MovementDataRL data;
    data.gen_delta_time_raw = genDeltaTime_ms;
    data.cam_received_time = current_time;
    data.calculated_delay = calculated_delay;
    data.timestamp = time_send_absolut;
    data.lat = current_lat;
    data.lon = current_lon;

    agent.history.push_back(data);

    if (agent.history.size() > 40) {
        agent.history.pop_front();
    }

    evaluatePendingPredictionsRL(target_id, agent);
}

void TrajAppRL::evaluatePendingPredictionsRL(long targetId, AgentHistoryRL& hist_struct) {
    if (hist_struct.history.empty()) return;

    const MovementDataRL& current_cam = hist_struct.history.back();
    double current_time = current_cam.timestamp; 
    long observer_id = getParentModule()->getParentModule()->getId();

    auto it = hist_struct.pending_queue.begin();
    while (it != hist_struct.pending_queue.end()) {
        bool all_done = true;

        auto evaluate_horizon = [&](double horizon, bool& is_done, 
                                    std::ofstream& logVeh, std::ofstream& logPed,
                                    double& sumVeh, long& countVeh, 
                                    double& sumPed, long& countPed) {
            if (is_done) return;

            double ideal_target_time = it->latest_cam_time + horizon;

            if (current_time >= ideal_target_time) {
                const MovementDataRL* best_match = &current_cam;

                if (current_time > ideal_target_time && hist_struct.history.size() > 1) {
                    const MovementDataRL& prev_cam = hist_struct.history[hist_struct.history.size() - 2];
                    double diff_after = current_time - ideal_target_time;
                    double diff_before = ideal_target_time - prev_cam.timestamp;
                    if (diff_before <= diff_after) best_match = &prev_cam;
                }

                double target_prediction_time = best_match->timestamp;
                double dt_predict = target_prediction_time - it->latest_cam_time;
                
                double pred_lat = (it->slope_lat * dt_predict) + it->intercept_lat;
                double pred_lon = (it->slope_lon * dt_predict) + it->intercept_lon;

                double lat_ae = std::abs(pred_lat - best_match->lat);
                double lon_ae = std::abs(pred_lon - best_match->lon);
                double total_ae = std::sqrt((lat_ae * lat_ae) + (lon_ae * lon_ae));

                std::stringstream ss;
                ss << std::fixed << std::setprecision(12)
                   << it->processing_time << ";" << it->latest_cam_time << ";"
                   << observer_id << ";" << it->base_cam_lat << ";" << it->base_cam_lon << ";"
                   << target_prediction_time << ";" << targetId << ";"
                   << best_match->lat << ";" << best_match->lon << ";"
                   << it->slope_lat << ";" << it->intercept_lat << ";" << pred_lat << ";"
                   << it->slope_lon << ";" << it->intercept_lon << ";" << pred_lon << ";"
                   << lat_ae << ";" << lon_ae << ";" << total_ae << "\n";

                if (hist_struct.target_type == "Vehicle") {
                    sumVeh += total_ae; countVeh++;
                    if (logVeh.is_open()) { logVeh << ss.str(); logVeh.flush(); }
                } else {
                    sumPed += total_ae; countPed++;
                    if (logPed.is_open()) { logPed << ss.str(); logPed.flush(); }
                }
                
                is_done = true;
            } else {
                all_done = false;
            }
        };

        evaluate_horizon(1.0, it->eval_1s_done, mPredLog1sVeh, mPredLog1sPed, mSumAe1sVeh, mCountAe1sVeh, mSumAe1sPed, mCountAe1sPed);
        evaluate_horizon(2.0, it->eval_2s_done, mPredLog2sVeh, mPredLog2sPed, mSumAe2sVeh, mCountAe2sVeh, mSumAe2sPed, mCountAe2sPed);
        evaluate_horizon(3.0, it->eval_3s_done, mPredLog3sVeh, mPredLog3sPed, mSumAe3sVeh, mCountAe3sVeh, mSumAe3sPed, mCountAe3sPed);

        if (current_time > it->latest_cam_time + 4.5) all_done = true;

        if (all_done) it = hist_struct.pending_queue.erase(it);
        else ++it;
    }
}

void TrajAppRL::takeRlSnapshot() {
    double t_sim = simTime().dbl();

    for (auto& pair : mOtherNodes) {
        auto& hist_struct = pair.second;
        if (hist_struct.history.empty()) continue;

        MovementDataRL current_latest_data = hist_struct.history.back();
        double current_latest_cam_time = current_latest_data.timestamp;

        if (t_sim - current_latest_cam_time > 1.5) continue;

        std::vector<MovementDataRL> window_data;
        for (auto it = hist_struct.history.rbegin(); it != hist_struct.history.rend(); ++it) {
            if (current_latest_cam_time - it->timestamp <= 1.2) {
                window_data.insert(window_data.begin(), *it);
            } else {
                break;
            }
        }

        if (window_data.size() < 2) continue;

        int n = window_data.size();
        double sum_x = 0.0, sum_y_lat = 0.0, sum_y_lon = 0.0;
        double sum_xy_lat = 0.0, sum_xy_lon = 0.0, sum_x2 = 0.0;

        for (const auto& d : window_data) {
            double x = d.timestamp - current_latest_cam_time;
            double y_lat = d.lat;
            double y_lon = d.lon;

            sum_x += x;
            sum_y_lat += y_lat;
            sum_y_lon += y_lon;
            sum_xy_lat += (x * y_lat);
            sum_xy_lon += (x * y_lon);
            sum_x2 += (x * x);
        }

        double denominator = (n * sum_x2) - (sum_x * sum_x);
        if (std::abs(denominator) < 1e-9) continue;

        PendingPredictionRL snap;
        snap.processing_time = t_sim;
        snap.latest_cam_time = current_latest_cam_time;
        snap.base_cam_lat = current_latest_data.lat;
        snap.base_cam_lon = current_latest_data.lon;

        snap.slope_lat = ((n * sum_xy_lat) - (sum_x * sum_y_lat)) / denominator;
        snap.intercept_lat = (sum_y_lat - (snap.slope_lat * sum_x)) / n;

        snap.slope_lon = ((n * sum_xy_lon) - (sum_x * sum_y_lon)) / denominator;
        snap.intercept_lon = (sum_y_lon - (snap.slope_lon * sum_x)) / n;

        hist_struct.pending_queue.push_back(snap);
    }
}

void TrajAppRL::finish() {
    ItsG5BaseService::finish();
}

} // namespace artery
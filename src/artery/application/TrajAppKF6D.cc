/**
 * @file V4_TrajAppKF6D.cc
 * @brief Trajectory Prediction Application using Kalman Filter 6D.
 * @details UPGRADED TO V4: Omnidirectional Tracking. Removes Cross-Entity Isolation 
 *          to allow V2V, V2P, P2V, and P2P tracking. Incorporates dynamic Q-tuning 
 *          and Modulus 65536 time sync. Output is branched by target identity.
 *          FIXED: I/O duplicate header prevention and pure CSV raw data generation.
 */

#include "artery/application/TrajAppKF6D.h"
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

Define_Module(TrajAppKF6D);

TrajAppKF6D::~TrajAppKF6D() {
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

std::string TrajAppKF6D::getNodeType() {
    cModule* myNode = getParentModule()->getParentModule();
    if (myNode) {
        std::string type = myNode->getNedTypeName();
        if (type.find("Person") != std::string::npos || type.find("Pedestrian") != std::string::npos) return "Pedestrian";
    }
    return "Vehicle";
}

void TrajAppKF6D::latLonToCartesian(double lat_raw, double lon_raw, double ref_lat_raw, double ref_lon_raw, double& x, double& y) {
    const double R = 6371000.0;
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
    mLogInterval = par("logInterval");

    // Open Dual Streams independently of Observer's Node Type
    auto initCamLog = [&](std::ofstream& stream, const std::string& targetLabel) {
        stream.open("results/V4_KF6D_CAM_Target_" + targetLabel + ".csv", std::ios_base::out | std::ios_base::app);
        
        // FIX: Sync pointer to prevent duplicate headers during multi-node instantiation
        stream.seekp(0, std::ios::end); 
        if (stream.tellp() <= 0) {
            stream << "GenDeltaTime_Raw;CAM_Received_Time;Calculated_Delay;CAM_Generation_Time;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Speed_mps;Heading;Acc_mps2\n";
            stream.flush();
        }
    };
    initCamLog(mCamLogVeh, "Vehicle");
    initCamLog(mCamLogPed, "Pedestrian");

    auto initPredLog = [&](std::ofstream& stream, const std::string& targetLabel, const std::string& horizonStr) {
        stream.open("results/V4_KF6D_" + horizonStr + "s_coefficient_log_Target_" + targetLabel + ".csv", std::ios_base::out | std::ios_base::app);
        
        // FIX: Sync pointer to prevent duplicate headers
        stream.seekp(0, std::ios::end); 
        if (stream.tellp() <= 0) {
            stream << "Processing_Time(s);Latest_CAM_Time(s);Observer_ID;Base_CAM_Lat;Base_CAM_Lon;Target_Prediction_Time(s);Target_ID;Actual_Lat;Actual_Lon;State_X;State_Y;Vel_X;Vel_Y;Acc_X;Acc_Y;Pred_Lat;Pred_Lon;Lat_AE;Lon_AE;Total_AE\n";
            stream.flush();
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

void TrajAppKF6D::handleMessage(cMessage* msg) {
    if (msg == mLogTimer) {
        scheduleAt(simTime() + mLogInterval, mLogTimer);
        return;
    }
    if (msg == mPredictionTimer) {
        takeKfSnapshot();
        scheduleAt(simTime() + 1.0, mPredictionTimer);
        return;
    }
    delete msg;
}

void TrajAppKF6D::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    if (signalID != mCamReceivedSignal) return;

    auto ca_obj = dynamic_cast<CaObject*>(obj);
    if (!ca_obj) return;

    // FIX: Resolves the double dereference operator error cleanly
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

    double current_lat_raw = static_cast<double>(basic.referencePosition.latitude);
    double current_lon_raw = static_cast<double>(basic.referencePosition.longitude);

    // KINEMATICS EXTRACTION (6D includes Acceleration)
    double speed_mps = 0.0, heading_deg = 0.0, acc_mps2 = 0.0;
    const auto& hf_container = cam_struct.cam.camParameters.highFrequencyContainer;
    if (hf_container.present == HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
        const auto& bvhf = hf_container.choice.basicVehicleContainerHighFrequency;
        speed_mps = static_cast<double>(bvhf.speed.speedValue) * 0.01;
        heading_deg = static_cast<double>(bvhf.heading.headingValue) * 0.1;
        long acc_val = bvhf.longitudinalAcceleration.longitudinalAccelerationValue;
        if (acc_val != 161) acc_mps2 = static_cast<double>(acc_val) * 0.1; // 161 means unavailable
    }

    double heading_rad = heading_deg * M_PI / 180.0;
    double vel_x = speed_mps * std::sin(heading_rad);
    double vel_y = speed_mps * std::cos(heading_rad);
    double acc_x = acc_mps2 * std::sin(heading_rad);
    double acc_y = acc_mps2 * std::cos(heading_rad);

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
           << current_lat_raw << ";" << current_lon_raw << ";"
           << speed_mps << ";" << heading_deg << ";" << acc_mps2 << "\n";
           
    if (target_type == "Vehicle") {
        mCamLogVeh << cam_ss.str(); mCamLogVeh.flush();
    } else {
        mCamLogPed << cam_ss.str(); mCamLogPed.flush();
    }

    // MEMORY INSTANTIATION & DYNAMIC Q-TUNING
    AgentHistoryKF6D& agent = mOtherNodes[target_id];
    agent.target_type = target_type;

    if (!agent.is_ref_set) {
        agent.ref_lat_raw = current_lat_raw;
        agent.ref_lon_raw = current_lon_raw;
        agent.is_ref_set = true;
        // Dynamic Q: Vehicle=2.0 (Linear), Pedestrian=20.0 (Stochastic)
        double q_tuning_factor = (target_type == "Vehicle") ? 10.0 : 20.0;
        agent.kf_state = std::make_unique<KalmanFilter6D>(q_tuning_factor);
        agent.kf_state->init({0.0, 0.0, vel_x, vel_y, acc_x, acc_y});
    }

    double local_x = 0.0, local_y = 0.0;
    latLonToCartesian(current_lat_raw, current_lon_raw, agent.ref_lat_raw, agent.ref_lon_raw, local_x, local_y);

    double dt = 0.1;
    if (!agent.history.empty()) {
        dt = time_send_absolut - agent.history.back().timestamp;
    }

    // PREDICT-CORRECT CYCLE (6D Vector Update)
    if (dt > 0.0 && agent.kf_state) {
        agent.kf_state->predict(dt);
        agent.kf_state->update({local_x, local_y, vel_x, vel_y, acc_x, acc_y});
    }

    MovementDataKF6D data;
    data.gen_delta_time_raw = genDeltaTime_ms;
    data.cam_received_time = current_time;
    data.calculated_delay = calculated_delay;
    data.timestamp = time_send_absolut;
    data.lat_raw = current_lat_raw;
    data.lon_raw = current_lon_raw;
    data.local_x = local_x;
    data.local_y = local_y;
    data.vel_x = vel_x;
    data.vel_y = vel_y;
    data.acc_x = acc_x;
    data.acc_y = acc_y;

    agent.history.push_back(data);
    agent.last_reception_time = current_time;

    double evaluation_window = 4.0;
    while (agent.history.size() > 1 && (time_send_absolut - agent.history.front().timestamp > evaluation_window)) {
        agent.history.pop_front();
    }

    evaluatePendingPredictionsKF6D(target_id, agent);
}

void TrajAppKF6D::evaluatePendingPredictionsKF6D(long targetId, AgentHistoryKF6D& hist_struct) {
    if (hist_struct.history.empty()) return;

    const MovementDataKF6D& current_cam = hist_struct.history.back();
    double current_time = current_cam.timestamp;
    long observer_id = getParentModule()->getParentModule()->getId();

    auto it = hist_struct.pending_queue.begin();
    while (it != hist_struct.pending_queue.end()) {
        bool all_done = true;

        auto evaluate_horizon = [&](double horizon, bool& is_done, std::ofstream& logVeh, std::ofstream& logPed) {
            if (is_done) return;

            double ideal_target_time = it->latest_cam_time + horizon;

            if (current_time >= ideal_target_time) {
                const MovementDataKF6D* best_match = &current_cam;

                if (current_time > ideal_target_time && hist_struct.history.size() > 1) {
                    const MovementDataKF6D& prev_cam = hist_struct.history[hist_struct.history.size() - 2];
                    double diff_after = current_time - ideal_target_time;
                    double diff_before = ideal_target_time - prev_cam.timestamp;
                    if (diff_before <= diff_after) best_match = &prev_cam;
                }

                double dt_pred = best_match->timestamp - it->latest_cam_time; 
                double dt_squared = dt_pred * dt_pred;
                
                // Extrapolation using Constant Acceleration Kinematics
                double pred_x = it->state_x + (it->vel_x * dt_pred) + (0.5 * it->acc_x * dt_squared);
                double pred_y = it->state_y + (it->vel_y * dt_pred) + (0.5 * it->acc_y * dt_squared);

                double pred_lat_raw = 0.0, pred_lon_raw = 0.0;
                cartesianToLatLon(pred_x, pred_y, hist_struct.ref_lat_raw, hist_struct.ref_lon_raw, pred_lat_raw, pred_lon_raw);

                double lat_ae = std::abs(best_match->lat_raw - pred_lat_raw);
                double lon_ae = std::abs(best_match->lon_raw - pred_lon_raw);
                double total_ae = std::sqrt((lat_ae * lat_ae) + (lon_ae * lon_ae));

                std::stringstream ss;
                ss << std::fixed << std::setprecision(12)
                   << it->processing_time << ";" << it->latest_cam_time << ";"
                   << observer_id << ";" << it->base_cam_lat << ";" << it->base_cam_lon << ";"
                   << best_match->timestamp << ";" << targetId << ";"
                   << best_match->lat_raw << ";" << best_match->lon_raw << ";"
                   << it->state_x << ";" << it->state_y << ";"
                   << it->vel_x << ";" << it->vel_y << ";" << it->acc_x << ";" << it->acc_y << ";"
                   << pred_lat_raw << ";" << pred_lon_raw << ";"
                   << lat_ae << ";" << lon_ae << ";" << total_ae << "\n";

                if (hist_struct.target_type == "Vehicle") {
                    if (logVeh.is_open()) { logVeh << ss.str(); logVeh.flush(); }
                } else {
                    if (logPed.is_open()) { logPed << ss.str(); logPed.flush(); }
                }
                
                is_done = true;
            } else {
                all_done = false;
            }
        };

        evaluate_horizon(1.0, it->eval_1s_done, mPredLog1sVeh, mPredLog1sPed);
        evaluate_horizon(2.0, it->eval_2s_done, mPredLog2sVeh, mPredLog2sPed);
        evaluate_horizon(3.0, it->eval_3s_done, mPredLog3sVeh, mPredLog3sPed);

        if (current_time > it->latest_cam_time + 4.5) all_done = true;

        if (all_done) it = hist_struct.pending_queue.erase(it);
        else ++it;
    }
}

void TrajAppKF6D::takeKfSnapshot() {
    double t_sim = simTime().dbl();

    for (auto& pair: mOtherNodes) {
        auto& hist_struct = pair.second;
        if (hist_struct.history.empty() || !hist_struct.kf_state) continue;

        MovementDataKF6D current_latest_data = hist_struct.history.back();
        double current_latest_cam_time = current_latest_data.timestamp;

        if (t_sim - current_latest_cam_time > 1.5) continue;

        PendingPredictionKF6D snap;
        snap.processing_time = t_sim;
        snap.latest_cam_time = current_latest_cam_time;
        snap.base_cam_lat = current_latest_data.lat_raw;
        snap.base_cam_lon = current_latest_data.lon_raw;

        std::vector<double> state = hist_struct.kf_state->getState();
        // FIX: Defensive programming bound check (Size must be >= 6 for KF6D)
        if (state.size() >= 6) { 
            snap.state_x = state[0]; snap.state_y = state[1];
            snap.vel_x = state[2]; snap.vel_y = state[3];
            snap.acc_x = state[4]; snap.acc_y = state[5];
        }
        hist_struct.pending_queue.push_back(snap);
    }
}

void TrajAppKF6D::finish() {
    // MAE Summary printing has been removed to keep the generated CSV files
    // purely tabular, providing a clean Ground Truth Dataset for the LSTM pipeline.
    ItsG5BaseService::finish();
}

} // namespace artery
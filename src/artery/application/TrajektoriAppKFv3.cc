#include "artery/application/TrajektoriAppKFv3.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include "artery/application/Middleware.h"
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <cmath>
#include <iomanip>
#include <limits>

namespace artery {
using namespace omnetpp;

Define_Module(TrajektoriAppKFv3);

TrajektoriAppKFv3::~TrajektoriAppKFv3() {
    // Safely close all logs to prevent data corruption on SUMO crash
    if (mCamLogFile.is_open()) mCamLogFile.close();
    if (mPredLog1s.is_open()) mPredLog1s.close();
    if (mPredLog2s.is_open()) mPredLog2s.close();
    if (mPredLog3s.is_open()) mPredLog3s.close();

    if (mLogTimer) {
        cancelAndDelete(mLogTimer);
        mLogTimer = nullptr;
    }
    if (mPredictionTimer) {
        cancelAndDelete(mPredictionTimer);
        mPredictionTimer = nullptr;
    }
}

std::string TrajektoriAppKFv3::getNodeType() {
    std::string type = getParentModule()->getNedTypeName();
    if (type.find("Vehicle") != std::string::npos) return "Vehicle";
    if (type.find("Person") != std::string::npos) return "Person";
    return "Unknown";
}

void TrajektoriAppKFv3::initialize() {
    ItsG5BaseService::initialize();
    mLogInterval = par("logInterval");

    std::string nodeType = getNodeType();
    std::string targetLabel = (nodeType == "Vehicle") ? "pedestrian_prediction" : "vehicle_prediction";

    // 1. Initialize Raw CAM Data Log
    mCamLogFile.open("results/KF_CAM_data_" + targetLabel + "_v3.csv", std::ios::out | std::ios::app);
    if (mCamLogFile.tellp() == 0) {
        mCamLogFile << "GenDeltaTime_Raw;CAM_Received_Time;Calculated_Delay;CAM_Generation_Time;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Target_Speed_Raw;Target_Heading_Raw\n";
    }

    // 2. Initialize Prediction Logs for all 3 Horizons
    auto initPredLog = [&](std::ofstream& stream, const std::string& horizonStr) {
        stream.open("results/KF_" + horizonStr + "s_coefficient_log_" + targetLabel + "_v3.csv", std::ios::out);
        stream << "Processing_Time(s);Latest_CAM_Time(s);Base_CAM_Lat;Base_CAM_Lon;Target_Prediction_Time(s);Node_Target;Actual_Lat;Actual_Lon;KF_PosX;KF_PosY;KF_VelX;KF_VelY;Pred_Lat;Pred_Lon;Lat_AE;Lon_AE;AE\n";
    };

    initPredLog(mPredLog1s, "1");
    initPredLog(mPredLog2s, "2");
    initPredLog(mPredLog3s, "3");

    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);

    // Schedule snapshot generation
    mPredictionTimer = new cMessage("predictionTimer");
    scheduleAt(simTime() + 1.0, mPredictionTimer);
}

void TrajektoriAppKFv3::handleMessage(cMessage* msg) {
    if (msg == mLogTimer) {
        logTrajectory();
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

void TrajektoriAppKFv3::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    if (signalID != mCamReceivedSignal) return;

    auto ca_obj = dynamic_cast<CaObject*>(obj);
    
    // SAFETY GUARD: Prevent Segfault when node is destroyed
    if (!ca_obj) return;

    const auto& cam = *ca_obj->asn1();
    long targetId = cam.header.stationID;

    const auto& basic = cam.cam.camParameters.basicContainer;
    const auto& hfc = cam.cam.camParameters.highFrequencyContainer;
    
    if (hfc.present != HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) return;
    const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;

    long genDeltaTime_ms = cam.cam.generationDeltaTime;
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
    if (delay_ms < 0) delay_ms += 65536;

    double time_send_absolut = (current_time_ms - delay_ms) / 1000.0;

    MovementData data;
    data.gen_delta_time_raw = genDeltaTime_ms;
    data.cam_received_time = current_time_ms / 1000.0;
    data.calculated_delay = delay_ms / 1000.0;
    data.timestamp = time_send_absolut;
    
    // Values extracted here are in MICRODEGREES (e.g., 106512997.0)
    data.latitude = static_cast<double>(basic.referencePosition.latitude) / 10.0;
    data.longitude = static_cast<double>(basic.referencePosition.longitude) / 10.0;
    data.speed_mps = static_cast<double>(bvc.speed.speedValue);
    data.heading_degree = static_cast<double>(bvc.heading.headingValue) / 10.0;

    AgentHistory& history = mOtherNodes[targetId];

    if (!history.is_ref_set) {
        history.ref_lat = data.latitude;
        history.ref_lon = data.longitude;
        history.is_ref_set = true;
        
        history.kf_state = std::make_unique<KalmanFilter4D>(0.1); 
        history.kf_state->init({0.0, 0.0, 0.0, 0.0}); 
    }

    double local_x, local_y;
    latLonToCartesian(data.latitude, data.longitude, history.ref_lat, history.ref_lon, local_x, local_y);
    data.local_x = local_x;
    data.local_y = local_y;

    double dt = 0.1;
    if (!history.history.empty()) {
        dt = data.timestamp.dbl() - history.history.back().timestamp.dbl();
    }
    
    if (dt > 0.0 && history.kf_state) {
        history.kf_state->predict(dt); 
    }
    
    if (history.kf_state) {
        std::vector<double> measurement = {local_x, local_y};
        history.kf_state->update(measurement);
    }

    history.history.push_back(data);
    history.lastReceptionTime = time_receive;
    history.hasNewData = true;

    // MEMORY LEAK PREVENTION: Bound the history size.
    if (history.history.size() > 20) {
        history.history.pop_front();
    }

    // ON-THE-FLY EVALUATION TRIGGER
    evaluatePendingPredictions(targetId, history);
}

void TrajektoriAppKFv3::evaluatePendingPredictions(long targetId, AgentHistory& hist_struct) {
    if (hist_struct.history.empty()) return;
    
    const MovementData& current_cam = hist_struct.history.back();
    double current_time = current_cam.timestamp.dbl();
    
    auto it = hist_struct.pending_queue.begin();
    while (it != hist_struct.pending_queue.end()) {
        bool all_done = true;
        
        // Inline helper to evaluate specific horizon cleanly
        auto evaluate_horizon = [&](double horizon, bool& is_done, std::ofstream& logFile, double& sumAE, long& countAE) {
            if (is_done) return;
            
            double ideal_target_time = it->latest_cam_time + horizon;
            
            // Priority 1 & Priority 2 Alignment executed at the exact moment of CAM arrival
            if (current_time >= ideal_target_time) {
                const MovementData* best_match = &current_cam;
                
                if (current_time > ideal_target_time && hist_struct.history.size() > 1) {
                    const MovementData& prev_cam = hist_struct.history[hist_struct.history.size() - 2];
                    double diff_after = current_time - ideal_target_time;
                    double diff_before = ideal_target_time - prev_cam.timestamp.dbl();
                    if (diff_before <= diff_after) {
                        best_match = &prev_cam;
                    }
                }
                
                double target_prediction_time = best_match->timestamp.dbl();
                double dt_predict = target_prediction_time - it->latest_cam_time;
                
                // Extract KF state (Cartesian coordinates in meters and m/s)
                double state_x = it->kf_state_snapshot[0];
                double state_y = it->kf_state_snapshot[1];
                double vel_x = it->kf_state_snapshot[2];
                double vel_y = it->kf_state_snapshot[3];
                
                // Cartesian Extrapolation
                double pred_local_x = state_x + (vel_x * dt_predict);
                double pred_local_y = state_y + (vel_y * dt_predict);
                
                double pred_lat, pred_lon;
                cartesianToLatLon(pred_local_x, pred_local_y, hist_struct.ref_lat, hist_struct.ref_lon, pred_lat, pred_lon);
                
                // AE Calculation (Done directly in microdegrees for consistency with RLv2)
                double lat_ae = std::abs(pred_lat - best_match->latitude);
                double lon_ae = std::abs(pred_lon - best_match->longitude);
                double ae = std::sqrt((lat_ae * lat_ae) + (lon_ae * lon_ae));
                
                sumAE += ae;
                countAE++;
                
                if (logFile.is_open()) {
                    logFile << std::fixed << std::setprecision(12)
                            << it->processing_time << ";"
                            << it->latest_cam_time << ";"
                            << it->base_cam_lat << ";" << it->base_cam_lon << ";" 
                            << target_prediction_time << ";" 
                            << targetId << ";"
                            << best_match->latitude << ";" << best_match->longitude << ";"
                            << state_x << ";" << state_y << ";" << vel_x << ";" << vel_y << ";"
                            << pred_lat << ";" << pred_lon << ";"
                            << lat_ae << ";" << lon_ae << ";" << ae << "\n";
                }
                is_done = true; // Mark this horizon as completed
            } else {
                all_done = false; // Still waiting for future CAM
            }
        };
        
        evaluate_horizon(1.0, it->eval_1s_done, mPredLog1s, mSumAe1s, mCountAe1s);
        evaluate_horizon(2.0, it->eval_2s_done, mPredLog2s, mSumAe2s, mCountAe2s);
        evaluate_horizon(3.0, it->eval_3s_done, mPredLog3s, mSumAe3s, mCountAe3s);
        
        // Timeout Cleanup: Drop snapshots if no CAM arrives for a long time
        if (current_time > it->latest_cam_time + 4.5) {
            all_done = true;
        }
        
        // Destroy snapshot from RAM immediately when all horizons are evaluated
        if (all_done) {
            it = hist_struct.pending_queue.erase(it);
        } else {
            ++it;
        }
    }
}

void TrajektoriAppKFv3::logTrajectory() {
    try {
        auto facilities = getFacilities();
        auto& vdp = facilities.get_const<VehicleDataProvider>();
        long myId = vdp.station_id();
        
        if (!mCamLogFile.is_open()) return;

        for (auto& pair : mOtherNodes) {
            long targetId = pair.first;
            auto& targetHist = pair.second;
            
            if (targetHist.hasNewData && !targetHist.history.empty()) {
                MovementData latest = targetHist.history.back();
                mCamLogFile << std::fixed << std::setprecision(12)
                         << latest.gen_delta_time_raw << ";"
                         << latest.cam_received_time << ";"
                         << latest.calculated_delay << ";"
                         << latest.timestamp.dbl() << ";"
                         << myId << ";" << targetId << ";"
                         << latest.latitude << ";" << latest.longitude << ";"
                         << latest.speed_mps << ";" << latest.heading_degree << "\n";
                targetHist.hasNewData = false;
            }
        }
        mCamLogFile.flush();
    } catch (...) { return; }
}

void TrajektoriAppKFv3::takeKfSnapshot() {
    double t_sim = simTime().dbl();

    for (auto& pair : mOtherNodes) {
        auto& hist_struct = pair.second;

        if (hist_struct.history.empty() || !hist_struct.kf_state) continue;

        MovementData current_latest_data = hist_struct.history.back();
        double current_latest_cam_time = current_latest_data.timestamp.dbl();

        if (t_sim - current_latest_cam_time > 1.5) continue;

        PendingPrediction snap;
        snap.processing_time = t_sim;
        snap.latest_cam_time = current_latest_cam_time;
        snap.base_cam_lat = current_latest_data.latitude;   
        snap.base_cam_lon = current_latest_data.longitude;  
        snap.kf_state_snapshot = hist_struct.kf_state->getState();

        // Push to waitlist. It will be evaluated in the next CAM arrivals
        hist_struct.pending_queue.push_back(snap);
    }
}

void TrajektoriAppKFv3::finish() {
    auto printMaeSummary = [](std::ofstream& stream, double sumAe, long countAe) {
        if (stream.is_open() && countAe > 0) {
            double mae_microdegree = sumAe / countAe;
            // 0.11132 roughly converts microdegrees to meters
            double mae_meter = mae_microdegree * 0.11132;
            stream << "\n;;;;;;;;;;;;;;;;MAE (microdegree);" << std::fixed << std::setprecision(12) << mae_microdegree << "\n";
            stream << ";;;;;;;;;;;;;;;;MAE (meter);" << mae_meter << "\n";
        }
    };

    printMaeSummary(mPredLog1s, mSumAe1s, mCountAe1s);
    printMaeSummary(mPredLog2s, mSumAe2s, mCountAe2s);
    printMaeSummary(mPredLog3s, mSumAe3s, mCountAe3s);

    ItsG5BaseService::finish();
}

// FIX APPLIED: Converts microdegrees to decimal degrees before spatial projection
void TrajektoriAppKFv3::latLonToCartesian(double lat_micro, double lon_micro, double ref_lat_micro, double ref_lon_micro, double& x, double& y) {
    const double R = 6371000.0; // Earth radius in meters
    
    double lat = lat_micro / 1000000.0;
    double lon = lon_micro / 1000000.0;
    double ref_lat = ref_lat_micro / 1000000.0;
    double ref_lon = ref_lon_micro / 1000000.0;

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double ref_lat_rad = ref_lat * M_PI / 180.0;
    double ref_lon_rad = ref_lon * M_PI / 180.0;

    x = R * (lon_rad - ref_lon_rad) * cos(ref_lat_rad);
    y = R * (lat_rad - ref_lat_rad);
}

// FIX APPLIED: Converts projected decimal degrees back to microdegrees
void TrajektoriAppKFv3::cartesianToLatLon(double x, double y, double ref_lat_micro, double ref_lon_micro, double& lat_micro, double& lon_micro) {
    const double R = 6371000.0;
    
    double ref_lat = ref_lat_micro / 1000000.0;
    double ref_lon = ref_lon_micro / 1000000.0;

    double ref_lat_rad = ref_lat * M_PI / 180.0;
    double ref_lon_rad = ref_lon * M_PI / 180.0;

    double lat_rad = (y / R) + ref_lat_rad;
    double lon_rad = (x / (R * cos(ref_lat_rad))) + ref_lon_rad;

    double lat = lat_rad * 180.0 / M_PI;
    double lon = lon_rad * 180.0 / M_PI;
    
    lat_micro = lat * 1000000.0;
    lon_micro = lon * 1000000.0;
}

} // namespace artery
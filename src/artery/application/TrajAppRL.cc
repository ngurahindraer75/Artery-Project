/**
 * @file TrajAppRL.cc
 * @brief Trajectory Prediction Application using Linear Regression (Sliding Window).
 * @details Implements ETSI ITS-G5 CAM reception, shortest-path modulus time synchronization,
 * and Ordinary Least Squares (OLS) regression for trajectory extrapolation.
 */

#include "artery/application/TrajAppRL.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include "artery/application/Middleware.h"
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <cmath>
#include <iomanip>
#include <limits>
#include <vector>

namespace artery {
using namespace omnetpp;

Define_Module(TrajAppRL);

TrajAppRL::~TrajAppRL() {
    // Safely close all file streams to prevent data corruption upon simulation termination
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

std::string TrajAppRL::getNodeType() {
    std::string type = getParentModule()->getNedTypeName();
    if (type.find("Vehicle") != std::string::npos) return "Vehicle";
    if (type.find("Person") != std::string::npos) return "Person";
    return "Unknown";
}

void TrajAppRL::initialize() {
    ItsG5BaseService::initialize();
    mLogInterval = par("logInterval");

    std::string nodeType = getNodeType();
    std::string targetLabel = (nodeType == "Vehicle") ? "pedestrian_prediction" : "vehicle_prediction";

    // 1. Initialize Raw CAM Data Log
    mCamLogFile.open("results/RL_CAM_data_" + targetLabel + "_v2.csv", std::ios::out | std::ios::app);
    if (mCamLogFile.tellp() == 0) {
        mCamLogFile << "GenDeltaTime_Raw;CAM_Received_Time;Calculated_Delay;CAM_Generation_Time;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Target_Speed_Raw;Target_Heading_Raw\n";
    }

    // 2. Initialize Prediction Logs for all 3 Horizons
    auto initPredLog = [&](std::ofstream& stream, const std::string& horizonStr) {
        stream.open("results/RL_" + horizonStr + "s_coefficient_log_" + targetLabel + "_v2.csv", std::ios::out);
        stream << "Processing_Time(s);Latest_CAM_Time(s);Base_CAM_Lat;Base_CAM_Lon;Target_Prediction_Time(s);Node_Target;Actual_Lat;Actual_Lon;Slope_Lat;Intercept_Lat;Pred_Lat;Slope_Lon;Intercept_Lon;Pred_Lon;Lat_AE;Lon_AE;AE\n";
    };

    initPredLog(mPredLog1s, "1");
    initPredLog(mPredLog2s, "2");
    initPredLog(mPredLog3s, "3");

    // Initialize timers
    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);

    // Schedule the first snapshot generation trigger
    mPredictionTimer = new cMessage("predictionTimer");
    scheduleAt(simTime() + 1.0, mPredictionTimer);
}

void TrajAppRL::handleMessage(cMessage* msg) {
    if (msg == mLogTimer) {
        logTrajectory();
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
    
    // Safety guard against segmentation faults during node destruction
    if (!ca_obj) return;

    const auto& cam = *ca_obj->asn1();
    long targetId = cam.header.stationID;

    const auto& basic = cam.cam.camParameters.basicContainer;
    const auto& hfc = cam.cam.camParameters.highFrequencyContainer;
    
    // Ensure we are processing Basic Vehicle Container (High Frequency)
    if (hfc.present != HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) return;
    const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;

    // --- BASELINE TIME SYNCHRONIZATION LOGIC ---
    long genDeltaTime_ms = cam.cam.generationDeltaTime;
    simtime_t time_receive = simTime();
    long long current_time_ms = time_receive.inUnit(SIMTIME_MS);

    // Calculate TAI offset modulation (Executed only once per observer)
    static long long tai_offset_mod = -1;
    if (tai_offset_mod == -1) {
        const cPacket* packet = dynamic_cast<const cPacket*>(details);
        long long true_creation_ms = (packet ? packet->getCreationTime() : simTime()).inUnit(SIMTIME_MS);
        tai_offset_mod = (genDeltaTime_ms - true_creation_ms) % 65536;
        if (tai_offset_mod < 0) tai_offset_mod += 65536;
    }

    long current_mod = (current_time_ms + tai_offset_mod) % 65536;
    long delay_ms = current_mod - genDeltaTime_ms;
    
    // BUGFIX: Shortest path modulus correction for 65s wrap-around anomaly
    // Handles scenarios where standard 16-bit ETSI timestamps reset to 0
    if (delay_ms < -32768) {
        delay_ms += 65536; // Target clock just wrapped around
    } else if (delay_ms > 32768) {
        delay_ms -= 65536; // Observer clock just wrapped around
    }
    
    // Failsafe for minor floating-point or OMNeT++ scheduling inaccuracies
    if (delay_ms < 0) {
        delay_ms = 0; 
    }

    double time_send_absolut = (current_time_ms - delay_ms) / 1000.0;
    // --------------------------------------------------------

    MovementData data;
    data.gen_delta_time_raw = genDeltaTime_ms;
    data.cam_received_time = current_time_ms / 1000.0;
    data.calculated_delay = delay_ms / 1000.0;
    data.timestamp = time_send_absolut;
    
    // Maintain absolute microdegrees for Linear Regression
    data.latitude = static_cast<double>(basic.referencePosition.latitude) / 10.0;
    data.longitude = static_cast<double>(basic.referencePosition.longitude) / 10.0;
    data.speed_mps = static_cast<double>(bvc.speed.speedValue);
    data.heading_degree = static_cast<double>(bvc.heading.headingValue) / 10.0;

    AgentHistory& history = mOtherNodes[targetId];

    history.history.push_back(data);
    history.lastReceptionTime = time_receive;
    history.hasNewData = true;

    // MEMORY MANAGEMENT: Bound the tracking history 
    // Kept slightly larger (e.g., 40) to accommodate the 1.2s sliding window even at high Hz
    if (history.history.size() > 40) {
        history.history.pop_front();
    }

    // ON-THE-FLY EVALUATION TRIGGER
    evaluatePendingPredictions(targetId, history);
}

void TrajAppRL::evaluatePendingPredictions(long targetId, AgentHistory& hist_struct) {
    if (hist_struct.history.empty()) return;
    
    const MovementData& current_cam = hist_struct.history.back();
    double current_time = current_cam.timestamp.dbl();
    
    auto it = hist_struct.pending_queue.begin();
    while (it != hist_struct.pending_queue.end()) {
        bool all_done = true;
        
        // Inline lambda helper to evaluate specific prediction horizons cleanly
        auto evaluate_horizon = [&](double horizon, bool& is_done, std::ofstream& logFile, double& sumAE, long& countAE) {
            if (is_done) return;
            
            double ideal_target_time = it->latest_cam_time + horizon;
            
            // Priority 1 & Priority 2 Time Alignment: Executed precisely upon CAM arrival
            if (current_time >= ideal_target_time) {
                const MovementData* best_match = &current_cam;
                
                // Select the nearest neighbor CAM to the ideal target time
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
                
                // Linear Regression Prediction Equation: Y = m*X + C
                // X is the relative time difference (dt_predict)
                double pred_lat = (it->slope_lat * dt_predict) + it->intercept_lat;
                double pred_lon = (it->slope_lon * dt_predict) + it->intercept_lon;
                
                // Absolute Error (AE) Calculation utilizing Euclidean distance
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
                            << it->slope_lat << ";" << it->intercept_lat << ";" << pred_lat << ";"
                            << it->slope_lon << ";" << it->intercept_lon << ";" << pred_lon << ";"
                            << lat_ae << ";" << lon_ae << ";" << ae << "\n";
                }
                is_done = true; 
            } else {
                all_done = false; 
            }
        };
        
        // Execute evaluation for 1s, 2s, and 3s horizons
        evaluate_horizon(1.0, it->eval_1s_done, mPredLog1s, mSumAe1s, mCountAe1s);
        evaluate_horizon(2.0, it->eval_2s_done, mPredLog2s, mSumAe2s, mCountAe2s);
        evaluate_horizon(3.0, it->eval_3s_done, mPredLog3s, mSumAe3s, mCountAe3s);
        
        // Timeout Cleanup: Drop snapshots if Target stops transmitting CAMs
        if (current_time > it->latest_cam_time + 4.5) {
            all_done = true;
        }
        
        // Destroy snapshot from RAM immediately when all configured horizons are evaluated
        if (all_done) {
            it = hist_struct.pending_queue.erase(it);
        } else {
            ++it;
        }
    }
}

void TrajAppRL::logTrajectory() {
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

void TrajAppRL::takeRlSnapshot() {
    double t_sim = simTime().dbl();

    for (auto& pair : mOtherNodes) {
        auto& hist_struct = pair.second;

        if (hist_struct.history.empty()) continue;

        MovementData current_latest_data = hist_struct.history.back();
        double current_latest_cam_time = current_latest_data.timestamp.dbl();

        // Prevent generating snapshots for stale targets (disconnected nodes)
        if (t_sim - current_latest_cam_time > 1.5) continue;

        // --- LINEAR REGRESSION (OLS) CALCULATION ---
        // Utilizing a 1.2-second sliding window history to compute the slope and intercept
        std::vector<MovementData> window_data;
        for (auto it = hist_struct.history.rbegin(); it != hist_struct.history.rend(); ++it) {
            if (current_latest_cam_time - it->timestamp.dbl() <= 1.2) {
                window_data.insert(window_data.begin(), *it);
            } else {
                break;
            }
        }

        if (window_data.size() < 2) continue; // Requires at least 2 points to draw a line

        int n = window_data.size();
        double sum_x = 0.0, sum_y_lat = 0.0, sum_y_lon = 0.0;
        double sum_xy_lat = 0.0, sum_xy_lon = 0.0, sum_x2 = 0.0;

        for (const auto& d : window_data) {
            // Using relative time (delta T from the latest CAM) to stabilize intercept values
            double x = d.timestamp.dbl() - current_latest_cam_time; 
            double y_lat = d.latitude;
            double y_lon = d.longitude;

            sum_x += x;
            sum_y_lat += y_lat;
            sum_y_lon += y_lon;
            sum_xy_lat += (x * y_lat);
            sum_xy_lon += (x * y_lon);
            sum_x2 += (x * x);
        }

        double denominator = (n * sum_x2) - (sum_x * sum_x);
        if (std::abs(denominator) < 1e-9) continue; // Prevent division by zero

        PendingPrediction snap;
        snap.processing_time = t_sim;
        snap.latest_cam_time = current_latest_cam_time;
        snap.base_cam_lat = current_latest_data.latitude;
        snap.base_cam_lon = current_latest_data.longitude;

        // OLS Coefficients Formula
        snap.slope_lat = ((n * sum_xy_lat) - (sum_x * sum_y_lat)) / denominator;
        snap.intercept_lat = (sum_y_lat - (snap.slope_lat * sum_x)) / n;

        snap.slope_lon = ((n * sum_xy_lon) - (sum_x * sum_y_lon)) / denominator;
        snap.intercept_lon = (sum_y_lon - (snap.slope_lon * sum_x)) / n;

        // Push to waitlist. It will be evaluated asynchronously upon future CAM arrivals
        hist_struct.pending_queue.push_back(snap);
    }
}

void TrajAppRL::finish() {
    // Helper lambda to append MAE summary at the bottom of the log file
    auto printMaeSummary = [](std::ofstream& stream, double sumAe, long countAe) {
        if (stream.is_open() && countAe > 0) {
            double mae_microdegree = sumAe / countAe;
            // Constant 0.11132 roughly converts microdegrees to Cartesian meters near the equator
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

} // namespace artery
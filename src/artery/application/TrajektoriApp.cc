#include "artery/application/TrajektoriApp.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include "artery/application/Middleware.h"
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <cmath>
#include <iomanip>
#include <vector>

namespace artery {
using namespace omnetpp;

Define_Module(TrajektoriApp);

std::string TrajektoriApp::getNodeType() {
    std::string type = getParentModule()->getNedTypeName();
    if (type.find("Vehicle") != std::string::npos) return "Vehicle";
    if (type.find("Person") != std::string::npos) return "Person";
    return "Unknown";
}

void TrajektoriApp::initialize() {
    ItsG5BaseService::initialize();

    mLogInterval = par("logInterval");
    std::string nodeType = getNodeType();

    // 1. Setup Raw CAM Log File
    std::string logFilename;
    if (nodeType == "Vehicle") {
        logFilename = "results/CAM_data_from_car_RL.csv";
    } else if (nodeType == "Person") {
        logFilename = "results/CAM_data_from_person_RL.csv";
    } else {
        logFilename = "results/CAM_data_unknown.csv";
    }

    mLogFile.open(logFilename, std::ios::out | std::ios::app);
    if (mLogFile.tellp() == 0) {
        mLogFile << "GenDeltaTime_Raw;CAM_Received_Time;Calculated_Delay;CAM_Generation_Time;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Target_Speed_Raw;Target_Heading_Raw" << std::endl;
    }

    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);

    // 2. Setup Prediction Log File (Using 3 Real-Time Time Pillars)
    std::string coefficientLogFilename;
    if (nodeType == "Vehicle") {
        coefficientLogFilename = "results/coefficient_log_from_car_RL.csv";
    } else { 
        coefficientLogFilename = "results/coefficient_log_from_person_RL.csv";
    }

    mCoefficientLogFile.open(coefficientLogFilename, std::ios::out | std::ios::app);
    if (mCoefficientLogFile.tellp() == 0) {
        // --- CSV HEADER UPDATED (3 Standardized Time Pillars) ---
        mCoefficientLogFile << "Processing_Time(s);Latest_CAM_Time(s);Node_Target;Actual_Lat;Actual_Lon;Target_Prediction_Time(s);Slope_Lat;Intercept_Lat;Pred_Lat;Slope_Lon;Intercept_Lon;Pred_Lon" << std::endl;
    }

    mPredictionTimer = new cMessage("predictionTimer");
    scheduleAt(simTime() + 1.0, mPredictionTimer);
}

void TrajektoriApp::handleMessage(cMessage* msg) {
    if (msg == mLogTimer) {
        logTrajectory();
        scheduleAt(simTime() + mLogInterval, mLogTimer);
        return; 
    }
    if (msg == mPredictionTimer) {
        runLinearRegression(); 
        scheduleAt(simTime() + 1.0, mPredictionTimer); 
        return; 
    }
    delete msg;
}

void TrajektoriApp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    if (signalID != mCamReceivedSignal) return;

    auto ca_obj = dynamic_cast<CaObject*>(obj);
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
    
    data.latitude = static_cast<double>(basic.referencePosition.latitude) / 10.0;
    data.longitude = static_cast<double>(basic.referencePosition.longitude) / 10.0;
    data.speed_mps = static_cast<double>(bvc.speed.speedValue);
    data.heading_degree = static_cast<double>(bvc.heading.headingValue) / 10.0;

    AgentHistory& history = mOtherNodes[targetId];
    history.history.push_back(data);
    history.lastReceptionTime = time_receive;
    history.hasNewData = true;

    while (!history.history.empty() && (time_receive.dbl() - history.history.front().timestamp.dbl() > 5.0)) {
        history.history.pop_front();
    }
}

void TrajektoriApp::logTrajectory() {
    auto& vdp = getFacilities().get_const<VehicleDataProvider>();
    long myId = vdp.station_id();

    if (!mLogFile.is_open()) return;

    for (auto& [targetId, targetHist] : mOtherNodes) {
        if (targetHist.hasNewData) {
            if (targetHist.history.empty()) continue;

            MovementData latest = targetHist.history.back();
            mLogFile << std::fixed << std::setprecision(12)
                     << latest.gen_delta_time_raw << ";"  
                     << latest.cam_received_time << ";"
                     << latest.calculated_delay << ";"
                     << latest.timestamp.dbl() << ";" 
                     << myId << ";"
                     << targetId << ";"
                     << latest.latitude << ";"
                     << latest.longitude << ";"
                     << latest.speed_mps << ";"
                     << latest.heading_degree << std::endl;

            targetHist.hasNewData = false;
        }
    }
    mLogFile.flush();
}

void TrajektoriApp::runLinearRegression() {
    double t_sim = simTime().dbl();

    for (auto& pair : mOtherNodes) {
        long targetId = pair.first;
        auto& hist_struct = pair.second;
        auto& hist = hist_struct.history;

        if (hist.empty()) continue;

        // Retrieve the latest observation (CAM) data in memory
        MovementData latest_data = hist.back();
        double last_time = latest_data.timestamp.dbl(); // This is Latest_CAM_Time(s)

        // SAFETY LOGIC: Prevent redundant predictions for the same latest data
        if (last_time == hist_struct.last_used_absolute_time) {
            continue;
        }

        // Ensure target is still active / has not left the map (2 Seconds Timeout)
        if (t_sim - last_time <= 2.0) {
            
            // --- REAL-TIME PROJECTION PARADIGM ---
            // Project the guess exactly 1 second into the future from the last CAM data
            double target_prediction_time = last_time + 1.0;

            // Define 1-Second Training Window (exactly 1 second before the latest data)
            double t_start = last_time - 1.0; 
            double t_end = last_time;

            std::vector<MovementData> train_data;
            MovementData fallback_point;
            bool has_fallback = false;

            for (auto& pt : hist) {
                if (pt.timestamp.dbl() >= t_start && pt.timestamp.dbl() <= t_end) {
                    train_data.push_back(pt);
                }
                if (pt.timestamp.dbl() < t_start) {
                    fallback_point = pt;
                    has_fallback = true;
                }
            }

            // If the node has just appeared and lacks data, use the oldest 1 data as an anchor
            if (train_data.size() < 2 && has_fallback) {
                train_data.insert(train_data.begin(), fallback_point);
            }

            // Absolute requirement for Linear Regression: Needs at least 2 points to draw a line
            if (train_data.size() < 2) continue;

            // Calculate Linear Regression (Least Squares Method)
            double n = train_data.size();
            double sum_x = 0, sum_y_lat = 0, sum_y_lon = 0;
            double sum_xx = 0, sum_xy_lat = 0, sum_xy_lon = 0;

            for (auto& pt : train_data) {
                double x = pt.timestamp.dbl();
                double y_lat = pt.latitude;
                double y_lon = pt.longitude;

                sum_x += x;
                sum_y_lat += y_lat;
                sum_y_lon += y_lon;
                sum_xx += x * x;
                sum_xy_lat += x * y_lat;
                sum_xy_lon += x * y_lon;
            }

            double denominator = n * sum_xx - sum_x * sum_x;
            if (std::abs(denominator) < 1e-9) continue;

            double b_lat = (n * sum_xy_lat - sum_x * sum_y_lat) / denominator;
            double a_lat = (sum_y_lat - b_lat * sum_x) / n;
            
            double b_lon = (n * sum_xy_lon - sum_x * sum_y_lon) / denominator;
            double a_lon = (sum_y_lon - b_lon * sum_x) / n;

            // Perform the Final Prediction to target_prediction_time
            double pred_lat = a_lat + b_lat * target_prediction_time;
            double pred_lon = a_lon + b_lon * target_prediction_time;

            // Save the output to the CSV file
            if (mCoefficientLogFile.is_open()) {
                mCoefficientLogFile << std::fixed << std::setprecision(12)
                                    << t_sim << ";"                     // Processing_Time(s)
                                    << last_time << ";"                 // Latest_CAM_Time(s)
                                    << targetId << ";"                  // Node_Target
                                    << latest_data.latitude << ";"      // Actual_Lat (Ground Truth)
                                    << latest_data.longitude << ";"     // Actual_Lon (Ground Truth)
                                    << target_prediction_time << ";"    // Target_Prediction_Time(s)
                                    << b_lat << ";"                     // Slope_Lat
                                    << a_lat << ";"                     // Intercept_Lat
                                    << pred_lat << ";"                  // Pred_Lat
                                    << b_lon << ";"                     // Slope_Lon
                                    << a_lon << ";"                     // Intercept_Lon
                                    << pred_lon << std::endl;           // Pred_Lon
            }
            
            // Record this observation time so the system waits for new data in the next cycle
            hist_struct.last_used_absolute_time = last_time; 
        }
    }
    mCoefficientLogFile.flush();
}

void TrajektoriApp::finish() {
    if (mLogFile.is_open()) mLogFile.close();
    cancelAndDelete(mLogTimer);

    if (mCoefficientLogFile.is_open()) mCoefficientLogFile.close();
    cancelAndDelete(mPredictionTimer);

    ItsG5BaseService::finish();
}

} // namespace artery
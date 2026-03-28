#include "artery/application/TrajektoriAppKFv1.h"
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

Define_Module(TrajektoriAppKFv1);

std::string TrajektoriAppKFv1::getNodeType() {
    std::string type = getParentModule()->getNedTypeName();
    if (type.find("Vehicle") != std::string::npos) return "Vehicle";
    if (type.find("Person") != std::string::npos) return "Person";
    return "Unknown";
}

void TrajektoriAppKFv1::initialize() {
    ItsG5BaseService::initialize();
    mLogInterval = par("logInterval");
    std::string nodeType = getNodeType();

    // 1. Setup Raw CAM Log File
    std::string logFilename;
    if (nodeType == "Vehicle") {
        logFilename = "results/CAM_data_from_car_KFv1.csv";
    } else {
        logFilename = "results/CAM_data_from_person_KFv1.csv";
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
    std::string kfLogFilename;
    if (nodeType == "Vehicle") {
        kfLogFilename = "results/kf_prediction_log_from_car_KFv1.csv";
    } else {
        kfLogFilename = "results/kf_prediction_log_from_person_KFv1.csv";
    }

    mPredictionLogFile.open(kfLogFilename, std::ios::out | std::ios::app);
    if (mPredictionLogFile.tellp() == 0) {
        // --- CSV HEADER UPDATED (3 Standardized Time Pillars to match Linear Regression) ---
        mPredictionLogFile << "Processing_Time(s);Latest_CAM_Time(s);Node_Target;Actual_Lat;Actual_Lon;Target_Prediction_Time(s);KF_Vel_Lat;KF_Vel_Lon;Pred_Lat;Pred_Lon" << std::endl;
    }

    mPredictionTimer = new cMessage("predictionTimer");
    scheduleAt(simTime() + 1.0, mPredictionTimer);
}

void TrajektoriAppKFv1::handleMessage(cMessage* msg) {
    if (msg == mLogTimer) {
        logTrajectory();
        scheduleAt(simTime() + mLogInterval, mLogTimer);
        return;
    }
    if (msg == mPredictionTimer) {
        runPredictions();
        scheduleAt(simTime() + 1.0, mPredictionTimer);
        return;
    }
    delete msg;
}

void TrajektoriAppKFv1::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    if (signalID != mCamReceivedSignal) return;

    auto ca_obj = dynamic_cast<CaObject*>(obj);
    if (!ca_obj) return;

    const auto& cam = *ca_obj->asn1();
    long targetId = cam.header.stationID;

    const auto& basic = cam.cam.camParameters.basicContainer;
    const auto& hfc = cam.cam.camParameters.highFrequencyContainer;
    if (hfc.present != HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) return;
    
    const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;

    // ========================================================================
    // HYBRID TIME RECONSTRUCTION (GPS/TAI CALIBRATION + DELAY EXTRACTION)
    // ========================================================================
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

    // UPDATE 4D KALMAN FILTER STATE MATRIX
    if (!history.kf_state) {
        history.kf_state = std::make_unique<KalmanFilter4D>();
        history.kf_state->init(data.latitude, data.longitude, data.timestamp.dbl());
    } else {
        history.kf_state->update(data.latitude, data.longitude, data.timestamp.dbl());
    }

    // Clean up data older than 5 seconds from memory to prevent overflow
    while (!history.history.empty() && (simTime().dbl() - history.history.front().timestamp.dbl() > 5.0)) {
        history.history.pop_front();
    }
}

void TrajektoriAppKFv1::logTrajectory() {
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

void TrajektoriAppKFv1::runPredictions() {
    double t_sim = simTime().dbl();

    for (auto& pair : mOtherNodes) {
        long targetId = pair.first;
        auto& hist_struct = pair.second;

        // INSTANT PREDICTION AND PRINTING PROCESS
        if (hist_struct.kf_state && hist_struct.kf_state->isInitialized() && !hist_struct.history.empty()) {
            
            // Retrieve the latest observation (CAM) data in memory
            MovementData latest_data = hist_struct.history.back();
            double last_time = latest_data.timestamp.dbl(); // This is Latest_CAM_Time(s)

            // SAFETY LOGIC 1: Prevent redundant predictions for the same latest data
            if (last_time == hist_struct.last_used_absolute_time) {
                continue; 
            }

            // SAFETY LOGIC 2: Ensure target is still active / has not left the map (2 Seconds Timeout)
            if (t_sim - last_time <= 2.0) {
                
                // --- REAL-TIME PROJECTION PARADIGM ---
                // Project the guess exactly 1 second into the future from the last CAM data
                double target_prediction_time = last_time + 1.0;
                
                // The Kalman Filter delta_t is the distance from the last known state to the target
                double delta_t = target_prediction_time - last_time; 
                if (delta_t < 0.01) delta_t = 0.01; // Safety fallback

                // 1. Extrapolate Prediction into the Future
                auto [pred_lat, pred_lon] = hist_struct.kf_state->predict(delta_t);
                
                // 2. Retrieve 4D State parameters to access Current Velocities
                std::vector<double> state = hist_struct.kf_state->getState();

                // 3. Print to CSV with Standardized Columns
                if (mPredictionLogFile.is_open()) {
                    mPredictionLogFile << std::fixed << std::setprecision(12)
                                       << t_sim << ";"                     // Processing_Time(s)
                                       << last_time << ";"                 // Latest_CAM_Time(s)
                                       << targetId << ";"                  // Node_Target
                                       << latest_data.latitude << ";"      // Actual_Lat (Ground Truth)
                                       << latest_data.longitude << ";"     // Actual_Lon (Ground Truth)
                                       << target_prediction_time << ";"    // Target_Prediction_Time(s)
                                       << state[1] << ";"                  // KF_Vel_Lat
                                       << state[2] << ";"                  // KF_Vel_Lon
                                       << pred_lat << ";"                  // Pred_Lat
                                       << pred_lon << std::endl;           // Pred_Lon
                }

                // Record this observation time so the system waits for new data in the next cycle
                hist_struct.last_used_absolute_time = last_time; 
            }
        }
    }
    mPredictionLogFile.flush();
}

void TrajektoriAppKFv1::finish() {
    if (mLogFile.is_open()) mLogFile.close();
    cancelAndDelete(mLogTimer);

    if (mPredictionLogFile.is_open()) mPredictionLogFile.close();
    cancelAndDelete(mPredictionTimer);

    ItsG5BaseService::finish();
}

} // namespace artery
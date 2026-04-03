#include "artery/application/TrajektoriAppKFv2.h"
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

Define_Module(TrajektoriAppKFv2);

TrajektoriAppKFv2::~TrajektoriAppKFv2() {
    if (mLogFile.is_open()) mLogFile.close();
    if (mPredictionLogFile.is_open()) mPredictionLogFile.close();

    try {
        if (mLogTimer) { cancelAndDelete(mLogTimer); mLogTimer = nullptr; }
        if (mPredictionTimer) { cancelAndDelete(mPredictionTimer); mPredictionTimer = nullptr; }
    } catch (...) {}
}

std::string TrajektoriAppKFv2::getNodeType() {
    std::string type = getParentModule()->getNedTypeName();
    if (type.find("Vehicle") != std::string::npos) return "Vehicle";
    if (type.find("Person") != std::string::npos) return "Person";
    return "Unknown";
}

void TrajektoriAppKFv2::initialize() {
    ItsG5BaseService::initialize();
    mLogInterval = par("logInterval");
    mPredictionHorizon = par("predictionHorizon").doubleValue();
    std::string nodeType = getNodeType();

    auto& vdp = getFacilities().get_const<VehicleDataProvider>();
    mMyStationId = vdp.station_id();

    std::string horizonPrefix = std::to_string(static_cast<int>(mPredictionHorizon)) + "s_";
    std::string logFilename = (nodeType == "Vehicle") ?
        "results/" + horizonPrefix + "CAM_data_from_car_kfv2.csv" :
        "results/" + horizonPrefix + "CAM_data_from_person_kfv2.csv";

    mLogFile.open(logFilename, std::ios::out | std::ios::app);
    if (mLogFile.tellp() == 0) {
        mLogFile << "GenDeltaTime_Raw;CAM_Received_Time;Calculated_Delay;CAM_Generation_Time;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Target_Speed_Raw;Target_Heading_Raw" << std::endl;
    }

    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);

    std::string kfLogFilename = (nodeType == "Vehicle") ?
        "results/" + horizonPrefix + "kf_prediction_log_from_car_v2.csv" :
        "results/" + horizonPrefix + "kf_prediction_log_from_person_v2.csv";

    mPredictionLogFile.open(kfLogFilename, std::ios::out | std::ios::app);
    if (mPredictionLogFile.tellp() == 0) {
        // HEADER DISEDERHANAKAN: Tanpa AE dan Lat_AE / Lon_AE
        mPredictionLogFile << "Processing_Time(s);Latest_CAM_Time(s);Target_Prediction_Time(s);Node_Target;Actual_Lat;Actual_Lon;KF_Vel_Lat;KF_Vel_Lon;Pred_Lat;Pred_Lon" << std::endl;
    }

    mPredictionTimer = new cMessage("predictionTimer");
    scheduleAt(simTime() + 1.0, mPredictionTimer);
}

void TrajektoriAppKFv2::handleMessage(cMessage* msg) {
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
    ItsG5BaseService::handleMessage(msg); 
}

void TrajektoriAppKFv2::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    if (signalID != mCamReceivedSignal) return;

    auto ca_obj = dynamic_cast<CaObject*>(obj);
    if (!ca_obj) return; 

    const auto& cam = *ca_obj->asn1();
    long targetId = cam.header.stationID;

    if (targetId == mMyStationId) return; 

    const auto& basic = cam.cam.camParameters.basicContainer;
    const auto& hfc = cam.cam.camParameters.highFrequencyContainer;
    if (hfc.present != HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) return;

    const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;

    long genDeltaTime_ms = cam.cam.generationDeltaTime;
    simtime_t time_receive = simTime();
    long long current_time_ms = time_receive.inUnit(SIMTIME_MS);

    if (mTaiOffsetMod == -1) {
        const cPacket* packet = dynamic_cast<const cPacket*>(details);
        long long true_creation_ms = (packet ? packet->getCreationTime() : simTime()).inUnit(SIMTIME_MS);
        mTaiOffsetMod = (genDeltaTime_ms - true_creation_ms) % 65536;
        if (mTaiOffsetMod < 0) mTaiOffsetMod += 65536;
    }

    long current_mod = (current_time_ms + mTaiOffsetMod) % 65536;
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

    if (!history.kf_state) {
        history.kf_state = std::make_unique<KalmanFilter4D>();
        history.kf_state->init(data.latitude, data.longitude, data.timestamp.dbl());
    } else {
        history.kf_state->update(data.latitude, data.longitude, data.timestamp.dbl());
    }

    history.history.push_back(data);
    history.lastReceptionTime = time_receive;
    history.hasNewData = true;

    while (!history.history.empty() && (simTime().dbl() - history.history.front().timestamp.dbl() > (mPredictionHorizon + 5.0))) {
        history.history.pop_front();
    }
}

void TrajektoriAppKFv2::logTrajectory() {
    long myId = mMyStationId; 
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
                     << myId << ";" << targetId << ";"
                     << latest.latitude << ";" << latest.longitude << ";"
                     << latest.speed_mps << ";" << latest.heading_degree << std::endl;
            targetHist.hasNewData = false;
        }
    }
    mLogFile.flush();
}

void TrajektoriAppKFv2::runPredictions() {
    double t_sim = simTime().dbl();

    for (auto& pair : mOtherNodes) {
        long targetId = pair.first;
        auto& hist_struct = pair.second;

        if (hist_struct.history.empty() || !hist_struct.kf_state || !hist_struct.kf_state->isInitialized()) continue;

        MovementData current_latest_data = hist_struct.history.back();
        double current_latest_cam_time = current_latest_data.timestamp.dbl();

        auto it = hist_struct.pending_queue.begin();
        while (it != hist_struct.pending_queue.end()) {
            double target_prediction_time = current_latest_cam_time;
            double actual_horizon_gap = target_prediction_time - it->latest_cam_time;

            if (actual_horizon_gap >= (mPredictionHorizon - 0.2)) {
                std::vector<double> state = it->kf_state_snapshot;
                
                if (state.size() >= 4) { 
                    double delta_t = target_prediction_time - it->latest_cam_time;
                    double pred_lat = state[0] + (state[2] * delta_t);
                    double pred_lon = state[1] + (state[3] * delta_t);

                    if (mPredictionLogFile.is_open()) {
                        mPredictionLogFile << std::fixed << std::setprecision(12)
                                           << it->processing_time << ";"
                                           << it->latest_cam_time << ";"
                                           << target_prediction_time << ";"
                                           << targetId << ";"
                                           << current_latest_data.latitude << ";"
                                           << current_latest_data.longitude << ";"
                                           << state[2] << ";" << state[3] << ";"
                                           << pred_lat << ";" << pred_lon << std::endl;
                    }
                }
                it = hist_struct.pending_queue.erase(it);
            }
            else if (t_sim - it->latest_cam_time > mPredictionHorizon + 2.0) {
                it = hist_struct.pending_queue.erase(it);
            }
            else {
                ++it;
            }
        }

        if (t_sim - current_latest_cam_time <= 1.5) {
            PendingPrediction new_pending;
            new_pending.processing_time = t_sim;
            new_pending.latest_cam_time = current_latest_cam_time;
            new_pending.kf_state_snapshot = hist_struct.kf_state->getState();
            hist_struct.pending_queue.push_back(new_pending);
        }
    }
    
    if (mPredictionLogFile.is_open()) {
        mPredictionLogFile.flush();
    }
}

void TrajektoriAppKFv2::finish() {
    // Fungsi cetak MAE telah dihapus sepenuhnya
    if (mLogFile.is_open()) mLogFile.close();
    if (mPredictionLogFile.is_open()) mPredictionLogFile.close();

    if (mLogTimer) { cancelAndDelete(mLogTimer); mLogTimer = nullptr; }
    if (mPredictionTimer) { cancelAndDelete(mPredictionTimer); mPredictionTimer = nullptr; }

    ItsG5BaseService::finish();
}

} // namespace artery
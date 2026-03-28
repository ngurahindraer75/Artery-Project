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

    std::string logFilename = (nodeType == "Vehicle") ? "results/CAM_data_from_car_kfv1.csv" : "results/CAM_data_from_person_kfv1.csv";
    mLogFile.open(logFilename, std::ios::out | std::ios::app);
    if (mLogFile.tellp() == 0) {
        mLogFile << "GenDeltaTime_Raw;CAM_Received_Time;Calculated_Delay;CAM_Generation_Time;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Target_Speed_Raw;Target_Heading_Raw" << std::endl;
    }

    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);

    std::string kfLogFilename = (nodeType == "Vehicle") ? "results/kf_prediction_log_from_car_v1.csv" : "results/kf_prediction_log_from_person_v1.csv";
    mPredictionLogFile.open(kfLogFilename, std::ios::out | std::ios::app);
    if (mPredictionLogFile.tellp() == 0) {
        mPredictionLogFile << "Processing_Time(s);Latest_CAM_Time(s);Target_Prediction_Time(s);Node_Target;Actual_Lat;Actual_Lon;KF_Vel_Lat;KF_Vel_Lon;Pred_Lat;Pred_Lon;Lat_AE;Lon_AE;AE" << std::endl;
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

    // REGULAR KF STATE UPDATE FOR TRACKING
    if (!history.kf_state) {
        history.kf_state = std::make_unique<KalmanFilter4D>();
        history.kf_state->init(data.latitude, data.longitude, data.timestamp.dbl());
    } else {
        history.kf_state->update(data.latitude, data.longitude, data.timestamp.dbl());
    }

    history.history.push_back(data);
    history.lastReceptionTime = time_receive;
    history.hasNewData = true;

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
                     << myId << ";" << targetId << ";"
                     << latest.latitude << ";" << latest.longitude << ";"
                     << latest.speed_mps << ";" << latest.heading_degree << std::endl;
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

        if (hist_struct.history.empty() || !hist_struct.kf_state || !hist_struct.kf_state->isInitialized()) continue;
        
        // Fetch the absolute most recent CAM data in memory
        MovementData current_latest_data = hist_struct.history.back();
        double current_latest_cam_time = current_latest_data.timestamp.dbl(); 

        // ===================================================================================
        // USER LOGIC STEP 2, 4, 5, 6: VALIDATE PREVIOUS CYCLE PENDING PREDICTION
        // ===================================================================================
        if (hist_struct.pending.is_active) {
            
            // The newest CAM Time becomes the Target Prediction Time & Actual Ground Truth
            double target_prediction_time = current_latest_cam_time; 

            // Ensure simulation actually moved forward approx 1 second (min 0.5s safeguard)
            if (target_prediction_time - hist_struct.pending.latest_cam_time >= 0.5) {

                // Step 4: Extract the KF State Snapshot saved from the PREVIOUS cycle
                std::vector<double> state = hist_struct.pending.kf_state_snapshot;
                
                // Calculate the exact time gap delta_t from the saved snapshot to current ground truth
                double delta_t = target_prediction_time - hist_struct.pending.latest_cam_time; 
                
                // Extrapolate Prediction into the Target Time using Constant Velocity model logic
                // Pred = current_pos + (current_vel * delta_t)
                // state[0] = Lat, state[1] = Lon, state[2] = Vel_Lat, state[3] = Vel_Lon
                double pred_lat = state[0] + (state[2] * delta_t);
                double pred_lon = state[1] + (state[3] * delta_t);

                // Step 5: Calculate Absolute Error (AE) directly with current Real Ground Truth
                double lat_ae = std::abs(pred_lat - current_latest_data.latitude);
                double lon_ae = std::abs(pred_lon - current_latest_data.longitude);
                double ae = std::sqrt((lat_ae * lat_ae) + (lon_ae * lon_ae));

                mSumAE += ae;
                mCountAE++;

                // Print to CSV. Processing_Time and Latest_CAM_Time use the PENDING values!
                if (mPredictionLogFile.is_open()) {
                    mPredictionLogFile << std::fixed << std::setprecision(12)
                                       << hist_struct.pending.processing_time << ";"                     
                                       << hist_struct.pending.latest_cam_time << ";"                 
                                       << target_prediction_time << ";"    
                                       << targetId << ";"                  
                                       << current_latest_data.latitude << ";"      
                                       << current_latest_data.longitude << ";"     
                                       << state[2] << ";" << state[3] << ";"  // Recorded KF_Vel_Lat & Lon               
                                       << pred_lat << ";" << pred_lon << ";"
                                       << lat_ae << ";" << lon_ae << ";" << ae << std::endl;           
                }
            }
            // Deactivate the pending task after it is fulfilled
            hist_struct.pending.is_active = false; 
        }

        // ===================================================================================
        // USER LOGIC STEP 1 & 4: RECORD PROCESSING TIME, BASE TIME, AND SNAPSHOT KF STATE
        // ===================================================================================
        hist_struct.pending.processing_time = t_sim; // E.g., 1.1
        hist_struct.pending.latest_cam_time = current_latest_cam_time; // E.g., 0.838
        
        // Take a Snapshot of the 4D KF state at this exact moment and store it in the Pending Buffer
        hist_struct.pending.kf_state_snapshot = hist_struct.kf_state->getState();
        
        // Mark the pending prediction as active, waiting for the next timer cycle to evaluate it
        hist_struct.pending.is_active = true;
    }
    mPredictionLogFile.flush();
}

void TrajektoriAppKFv1::finish() {
    if (mLogFile.is_open()) mLogFile.close();
    cancelAndDelete(mLogTimer);

    // Calculate and Print final MAE
    if (mCountAE > 0 && mPredictionLogFile.is_open()) {
        double mae_microdegree = mSumAE / mCountAE;
        double mae_meter = mae_microdegree * 0.11132;
        mPredictionLogFile << "\n;;;;;;;;;;;;MAE (microdegree);" << std::fixed << std::setprecision(12) << mae_microdegree << std::endl;
        mPredictionLogFile << ";;;;;;;;;;;;MAE (meter);" << mae_meter << std::endl;
    }

    if (mPredictionLogFile.is_open()) mPredictionLogFile.close();
    cancelAndDelete(mPredictionTimer);

    ItsG5BaseService::finish();
}

} // namespace artery
#include "artery/application/TrajektoriAppKF4D.h"
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

Define_Module(TrajektoriAppKF4D);

std::string TrajektoriAppKF4D::getNodeType() {
    std::string type = getParentModule()->getNedTypeName();
    if (type.find("Vehicle") != std::string::npos) return "Vehicle";
    if (type.find("Person") != std::string::npos) return "Person";
    return "Unknown";
}

void TrajektoriAppKF4D::initialize() {
    ItsG5BaseService::initialize();
    mLogInterval = par("logInterval");
    std::string nodeType = getNodeType();

    std::string logFilename;
    if (nodeType == "Vehicle") {
        logFilename = "results/CAM_data_from_car_kf.csv";
    } else {
        logFilename = "results/CAM_data_from_person_kf.csv";
    }

    mLogFile.open(logFilename, std::ios::out | std::ios::app);
    if (mLogFile.tellp() == 0) {
        mLogFile << "GenDeltaTime_Raw;CAM_Received_Time;Calculated_Delay;CAM_Generation_Time;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Target_Speed_Raw" << std::endl;
    }

    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);

    std::string kfLogFilename;
    if (nodeType == "Vehicle") {
        kfLogFilename = "results/kf_prediction_log_from_car.csv";
    } else {
        kfLogFilename = "results/kf_prediction_log_from_person.csv";
    }

    mPredictionLogFile.open(kfLogFilename, std::ios::out | std::ios::app);
    if (mPredictionLogFile.tellp() == 0) {
        // HEADER CSV DIREVISI MENJADI MURNI PARAMETER KALMAN FILTER
        mPredictionLogFile << "Time_Eval(s);Time_Creation_Absolut(s);Node_Target;GT_Lat;GT_Lon;KF_Pos_Lat;KF_Pos_Lon;KF_Vel_Lat;KF_Vel_Lon;Pred_Lat;Pred_Lon;Lat_AE;Lon_AE;AE" << std::endl;
    }

    mPredictionTimer = new cMessage("predictionTimer");
    scheduleAt(simTime() + 1.0, mPredictionTimer);
}

void TrajektoriAppKF4D::handleMessage(cMessage* msg) {
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

void TrajektoriAppKF4D::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
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

    AgentHistory& history = mOtherNodes[targetId];
    history.history.push_back(data);
    history.lastReceptionTime = time_receive;
    history.hasNewData = true;

    if (!history.kf_state) {
        history.kf_state = std::make_unique<KalmanFilter4D>();
        history.kf_state->init(data.latitude, data.longitude, data.timestamp.dbl());
    } else {
        history.kf_state->update(data.latitude, data.longitude, data.timestamp.dbl());
    }

    while (!history.history.empty() && (simTime() - history.history.front().timestamp > 5.0)) {
        history.history.pop_front();
    }
}

void TrajektoriAppKF4D::logTrajectory() {
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
                     << latest.speed_mps << std::endl;

            targetHist.hasNewData = false;
        }
    }
    mLogFile.flush();
}

void TrajektoriAppKF4D::runPredictions() {
    double t_sim = simTime().dbl();
    double current_floor = std::floor(t_sim);
    double future_target = current_floor + 1.0;

    for (auto& pair : mOtherNodes) {
        long targetId = pair.first;
        auto& hist = pair.second;

        // 1. EVALUASI PREDIKSI MASA LALU (GROUND TRUTH)
        // PASTIKAN TIDAK ADA "++it" DI DALAM KURUNG FOR INI!
        for (auto it = hist.pending_predictions.begin(); it != hist.pending_predictions.end(); ) {
            if (t_sim >= it->target_time) {
                MovementData* gt_point = nullptr;
                double min_diff = 9999.0;
                for (auto& pt : hist.history) {
                    double diff = std::abs(pt.timestamp.dbl() - it->target_time);
                    if (diff < min_diff) {
                        min_diff = diff;
                        gt_point = &pt;
                    }
                }

                if (gt_point) {
                    // LOGIKA PENGAMANAN 1: Validasi Data Ground Truth
                    if (min_diff <= 1.0) {
                        // Hitung Absolute Error (AE)
                        double lat_ae = std::abs(it->pred_lat - gt_point->latitude);
                        double lon_ae = std::abs(it->pred_lon - gt_point->longitude);
                        double ae = std::sqrt((lat_ae * lat_ae) + (lon_ae * lon_ae));

                        mSumAE += ae;
                        mCountAE++;

                        if (mPredictionLogFile.is_open()) {
                            mPredictionLogFile << std::fixed << std::setprecision(12)
                                               << t_sim << ";"                     // Time_Eval(s)
                                               << it->creation_time << ";"         // Time_Creation_Absolut(s)
                                               << targetId << ";"                  // Node_Target
                                               << gt_point->latitude << ";"        // GT_Lat
                                               << gt_point->longitude << ";"       // GT_Lon
                                               << it->kf_pos_lat << ";"            // KF_Pos_Lat (x)
                                               << it->kf_pos_lon << ";"            // KF_Pos_Lon (y)
                                               << it->kf_vel_lat << ";"            // KF_Vel_Lat (vx)
                                               << it->kf_vel_lon << ";"            // KF_Vel_Lon (vy)
                                               << it->pred_lat << ";"              // Pred_Lat
                                               << it->pred_lon << ";"              // Pred_Lon
                                               << lat_ae << ";"                    // Lat_AE
                                               << lon_ae << ";"                    // Lon_AE
                                               << ae << std::endl;                 // AE
                        }
                    }
                }
                // SANGAT KRUSIAL: Hapus elemen dan otomatis tangkap posisi baru
                it = hist.pending_predictions.erase(it); 
            } else {
                // Maju ke elemen berikutnya HANYA JIKA belum waktunya dievaluasi
                ++it; 
            }
        } // BATAS AKHIR LOOP EVALUASI

        // 2. PEMBANGKITAN PREDIKSI BARU UNTUK MASA DEPAN
        if (hist.kf_state && hist.kf_state->isInitialized() && !hist.history.empty()) {
            double last_absolute_time = hist.history.back().timestamp.dbl();

            // LOGIKA PENGAMANAN 2: Validasi Koneksi (Timeout 2 detik)
            if (t_sim - last_absolute_time <= 2.0) {
                double delta_t = future_target - last_absolute_time;
                if (delta_t < 0.01) delta_t = 0.01;

                auto [pred_lat, pred_lon] = hist.kf_state->predict(delta_t);

                std::vector<double> state = hist.kf_state->getState();

                PendingPrediction p;
                p.target_time = future_target;
                p.creation_time = last_absolute_time;
                p.kf_pos_lat = state[0];
                p.kf_pos_lon = state[1];
                p.kf_vel_lat = state[2];
                p.kf_vel_lon = state[3];
                p.pred_lat = pred_lat;
                p.pred_lon = pred_lon;

                hist.pending_predictions.push_back(p);
            }
        }
    }
    mPredictionLogFile.flush();
}

void TrajektoriAppKF4D::finish() {
    if (mLogFile.is_open()) mLogFile.close();
    cancelAndDelete(mLogTimer);

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
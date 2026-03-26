#include "TrajektoriAppVerIND.h"
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

Define_Module(TrajektoriAppVerIND);

std::string TrajektoriAppVerIND::getNodeType()
{
    std::string type = getParentModule()->getNedTypeName();
    if (type.find("Vehicle") != std::string::npos) return "Vehicle";
    if (type.find("Person") != std::string::npos) return "Person";
    return "Unknown";
}

void TrajektoriAppVerIND::initialize()
{
    ItsG5BaseService::initialize();
    mLogInterval = par("logInterval");
    std::string nodeType = getNodeType();

    std::string logFilename;
    if (nodeType == "Vehicle") {
        logFilename = "results/CAM_data_from_car_v2.csv"; 
    } else {
        logFilename = "results/CAM_data_from_person_v2.csv"; 
    }
    mLogFile.open(logFilename, std::ios::out | std::ios::app);
    if (mLogFile.tellp() == 0) {
        mLogFile << "Time_s;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Target_Speed_Raw" << std::endl;
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
        // Header log baru disamakan 100% dengan hasil RL
        mPredictionLogFile << "Time_prediction(s);Time_absolut_s;Node_Target;Actual_Lat;Actual_Lon;Slope_Lat;Intercept_Lat;Pred_Lat;Slope_Lon;Intercept_Lon;Pred_Lon;Lat_AE;Lon_AE;AE" << std::endl;
    }
    mPredictionTimer = new cMessage("predictionTimer");
    scheduleAt(simTime() + 1.0, mPredictionTimer); 
}

void TrajektoriAppVerIND::handleMessage(cMessage* msg)
{
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

void TrajektoriAppVerIND::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    if (signalID != mCamReceivedSignal) return;
    auto ca_obj = dynamic_cast<CaObject*>(obj);
    if (!ca_obj) return;
    const cPacket* packet = dynamic_cast<const cPacket*>(details);
    simtime_t creationTime = packet ? packet->getCreationTime() : simTime();
    const auto& cam = *ca_obj->asn1();
    long stationId = cam.header.stationID;
    const auto& basic = cam.cam.camParameters.basicContainer;
    const auto& hfc = cam.cam.camParameters.highFrequencyContainer;
    if (hfc.present != HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) return;
    
    const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;
    MovementData data;
    data.timestamp = creationTime;
    data.latitude = static_cast<double>(basic.referencePosition.latitude)/10;
    data.longitude = static_cast<double>(basic.referencePosition.longitude)/10;
    data.speed_mps = static_cast<double>(bvc.speed.speedValue);

    AgentHistory& history = mOtherNodes[stationId];
    history.history.push_back(data);
    history.lastReceptionTime = simTime();
    history.hasNewData = true;

    if (!history.kf_state) {
        history.kf_state = std::make_unique<KalmanFilterIND>();
        history.kf_state->init(data.latitude, data.longitude, data.timestamp.dbl());
    } else {
        history.kf_state->update(data.latitude, data.longitude, data.timestamp.dbl());
    }

    while (!history.history.empty() && (simTime() - history.history.front().timestamp > 5.0)) {
        history.history.pop_front();
    }
}

void TrajektoriAppVerIND::logTrajectory()
{
    auto& vdp = getFacilities().get_const<VehicleDataProvider>();
    long myId = vdp.station_id();
    if (!mLogFile.is_open()) return;

    for (auto& [targetId, targetHist] : mOtherNodes) {
        if (targetHist.hasNewData) {
            if (targetHist.history.empty()) continue;
            MovementData latest = targetHist.history.back();
            mLogFile << std::fixed << std::setprecision(12)
                     << latest.timestamp.dbl() << ";" << myId << ";" << targetId << ";"
                     << latest.latitude << ";" << latest.longitude << ";" << latest.speed_mps << std::endl;
            targetHist.hasNewData = false;
        }
    }
    mLogFile.flush();
}

void TrajektoriAppVerIND::runPredictions()
{
    simtime_t now = simTime();
    double horizon_s = 1.0; 
    double target_time = now.dbl() + horizon_s; 

    for (auto& [targetId, targetHist] : mOtherNodes)
    {
        // 1. CEK TIMEOUT (Data Stale)
        if ((now - targetHist.lastReceptionTime).dbl() > 2.0) {
            targetHist.pending_predictions.clear(); 
            continue;
        }

        // 2. AMBIL STATE KF & TRANSLASI KE INTERCEPT ABSOLUT
        if (targetHist.kf_state && targetHist.kf_state->isInitialized())
        {
            std::vector<double> state = targetHist.kf_state->getState();
            
            // Indeks State KF 6D: 0=Lat, 1=Lon, 2=vLat, 3=vLon, 4=aLat, 5=aLon
            
            double slope_lat = state[ 2 ]; // Ambil Kecepatan Lat
            double intercept_lat = state[ 0 ] - (slope_lat * now.dbl()); // Ambil Posisi Lat

            double slope_lon = state[ 3 ]; // Ambil Kecepatan Lon
            double intercept_lon = state[ 1 ] - (slope_lon * now.dbl()); // Ambil Posisi Lon

            PendingPrediction p;
            p.target_time = target_time;
            p.creation_time = now.dbl();
            p.slope_lat = slope_lat;
            p.intercept_lat = intercept_lat;
            p.slope_lon = slope_lon;
            p.intercept_lon = intercept_lon;
            
            targetHist.pending_predictions.push_back(p);
        }

        // 3. DELAYED EVALUATION (Evaluasi Prediksi Masa Lalu)
        auto it = targetHist.pending_predictions.begin();
        while (it != targetHist.pending_predictions.end()) {
            if (it->target_time <= now.dbl()) {
                
                MovementData closest_actual;
                double min_time_diff = 9999.0;
                bool found_actual = false;

                for (const auto& point : targetHist.history) {
                    double time_diff = std::abs(point.timestamp.dbl() - it->target_time);
                    if (time_diff < min_time_diff) {
                        min_time_diff = time_diff;
                        closest_actual = point;
                        found_actual = true;
                    }
                }

                // --- PERBAIKAN LOGIKA DI SINI ---
                // Hanya hitung dan catat MAE jika selisih waktu aktual dan target tidak lebih dari 0.5 detik.
                // Ini mencegah Phantom Evaluation (pasangan prediksi dengan Ground Truth yang sudah basi).
                if (found_actual && min_time_diff <= 0.5) {
                    
                    // Ekstrak Time_absolut dari Ground Truth
                    double time_absolut = closest_actual.timestamp.dbl();
                    
                    // PERUBAHAN: Gunakan time_absolut sebagai pengali (bukan target_time)
                    double pred_lat = it->intercept_lat + (it->slope_lat * time_absolut);
                    double pred_lon = it->intercept_lon + (it->slope_lon * time_absolut);
                    
                    double lat_ae = std::abs(closest_actual.latitude - pred_lat);
                    double lon_ae = std::abs(closest_actual.longitude - pred_lon);
                    double ae = std::sqrt((lat_ae * lat_ae) + (lon_ae * lon_ae)); 
                    
                    mTotalAE += ae;
                    mCountAE++;
                    
                    mPredictionLogFile << std::fixed << std::setprecision(12)
                                       << it->target_time << ";"
                                       << time_absolut << ";" // Log Time_absolut
                                       << targetId << ";"
                                       << closest_actual.latitude << ";"
                                       << closest_actual.longitude << ";"
                                       << it->slope_lat << ";"
                                       << it->intercept_lat << ";"
                                       << pred_lat << ";"
                                       << it->slope_lon << ";"
                                       << it->intercept_lon << ";"
                                       << pred_lon << ";"
                                       << lat_ae << ";"
                                       << lon_ae << ";"
                                       << ae << std::endl;
                }
                
                // Terlepas dari dicatat atau tidak, prediksi ini tetap harus dihapus dari antrean
                it = targetHist.pending_predictions.erase(it);
            } else {
                ++it;
            }
        }
    }
    mPredictionLogFile.flush();
}

void TrajektoriAppVerIND::finish()
{
    if (mLogFile.is_open()) mLogFile.close();
    cancelAndDelete(mLogTimer);

    if (mPredictionLogFile.is_open()) {
        // Cetak kalkulasi MAE otomatis di baris bawah
        if (mCountAE > 0) {
            double mae_microdegree = mTotalAE / mCountAE;
            double mae_meter = mae_microdegree * 0.11132; // Konversi spasial
            
            mPredictionLogFile << std::endl; 
            mPredictionLogFile << ";;;;;;;;;;;;MAE (microdegree);" << std::fixed << std::setprecision(12) << mae_microdegree << std::endl;
            mPredictionLogFile << ";;;;;;;;;;;;MAE (meter);" << mae_meter << std::endl;
        }
        mPredictionLogFile.close();
    }
    cancelAndDelete(mPredictionTimer);
    
    ItsG5BaseService::finish();
}

} // namespace artery

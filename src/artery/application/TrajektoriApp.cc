#include "TrajektoriApp.h"
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

std::string TrajektoriApp::getNodeType()
{
    std::string type = getParentModule()->getNedTypeName();
    if (type.find("Vehicle") != std::string::npos) return "Vehicle";
    if (type.find("Person") != std::string::npos) return "Person";
    return "Unknown";
}

void TrajektoriApp::initialize()
{
    ItsG5BaseService::initialize();
    mLogInterval = par("logInterval");
    std::string nodeType = getNodeType();

    std::string logFilename;
    if (nodeType == "Vehicle") {
        logFilename = "results/CAM_data_from_car.csv";
    } else if (nodeType == "Person") {
        logFilename = "results/CAM_data_from_person.csv";
    } else {
        logFilename = "results/CAM_data_unknown.csv";
    }
    
    mLogFile.open(logFilename, std::ios::out | std::ios::app);
    if (mLogFile.tellp() == 0) {
        mLogFile << "Time_s;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Target_Speed_Raw" << std::endl;
    }
    
    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);
    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);

    std::string coefficientLogFilename;
    if (nodeType == "Vehicle") {
        coefficientLogFilename = "results/coefficient_log_from_car.csv";
    } else {
        coefficientLogFilename = "results/coefficient_log_from_person.csv";
    }
    
    mCoefficientLogFile.open(coefficientLogFilename, std::ios::out | std::ios::app);
    if (mCoefficientLogFile.tellp() == 0) {
        // Header disamakan persis dengan format komparasi
        mCoefficientLogFile << "Time_prediction(s);Time_absolut_s;Node_Target;Actual_Lat;Actual_Lon;Slope_Lat;Intercept_Lat;Pred_Lat;Slope_Lon;Intercept_Lon;Pred_Lon;Lat_AE;Lon_AE;AE" << std::endl;
    }
    
    mPredictionTimer = new cMessage("predictionTimer");
    scheduleAt(simTime() + 1.0, mPredictionTimer);
}

void TrajektoriApp::handleMessage(cMessage* msg)
{
    if (msg == mLogTimer) {
        logTrajectory();
        scheduleAt(simTime() + mLogInterval, mLogTimer);
        return;
    }
    if (msg == mPredictionTimer) {
        logCoefficients(); 
        scheduleAt(simTime() + 1.0, mPredictionTimer);
        return;
    }
    delete msg;
}

void TrajektoriApp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
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

    while (!history.history.empty() && (simTime() - history.history.front().timestamp > 5.0)) {
        history.history.pop_front();
    }
}

void TrajektoriApp::logTrajectory()
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

void TrajektoriApp::logCoefficients()
{
    simtime_t now = simTime();
    double horizon_s = 1.0; 
    double target_time = now.dbl() + horizon_s;

    for (auto& [targetId, targetHist] : mOtherNodes)
    {
        // 1. CEK TIMEOUT
        if ((now - targetHist.lastReceptionTime).dbl() > 2.0) {
            targetHist.pending_predictions.clear(); 
            continue;
        }

        // 2. HITUNG KOEFISIEN RL & SIMPAN SEBAGAI PENDING PREDICTION
        std::vector<MovementData> points_1s;
        for (const auto& point : targetHist.history) {
            if (now - point.timestamp <= 1.0 && now > point.timestamp) { 
                points_1s.push_back(point);
            }
        }
        if (points_1s.size() < 2 && targetHist.history.size() >= 2) {
            points_1s.clear();
            points_1s.push_back(targetHist.history[targetHist.history.size() - 2]);
            points_1s.push_back(targetHist.history.back());
        }

        if (points_1s.size() >= 2) {
            RegressionCoefficients coeffs = calculateCoefficients(points_1s);
            if (coeffs.valid) {
                PendingPrediction p;
                p.target_time = target_time;
                p.creation_time = now.dbl();
                p.slope_lat = coeffs.b_lat;
                p.intercept_lat = coeffs.a_lat;
                p.slope_lon = coeffs.b_lon;
                p.intercept_lon = coeffs.a_lon;
                
                targetHist.pending_predictions.push_back(p);
            }
        }

        // 3. DELAYED EVALUATION (Mengevaluasi Prediksi Masa Lalu dengan Time_absolut)
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

                // Proteksi dari Phantom Evaluation (0.5s toleransi)
                if (found_actual && min_time_diff <= 0.5) {
                    double time_absolut = closest_actual.timestamp.dbl();
                    
                    // PERUBAHAN: Pengali menggunakan time_absolut (Persis seperti KF)
                    double pred_lat = it->intercept_lat + (it->slope_lat * time_absolut);
                    double pred_lon = it->intercept_lon + (it->slope_lon * time_absolut);
                    
                    double lat_ae = std::abs(closest_actual.latitude - pred_lat);
                    double lon_ae = std::abs(closest_actual.longitude - pred_lon);
                    double ae = std::sqrt((lat_ae * lat_ae) + (lon_ae * lon_ae)); 
                    
                    mTotalAE += ae;
                    mCountAE++;
                    
                    mCoefficientLogFile << std::fixed << std::setprecision(12)
                                        << it->target_time << ";"
                                        << time_absolut << ";" 
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
                it = targetHist.pending_predictions.erase(it);
            } else {
                ++it;
            }
        }
    }
    mCoefficientLogFile.flush();
}

RegressionCoefficients TrajektoriApp::calculateCoefficients(const std::vector<MovementData>& points)
{
    RegressionCoefficients result;
    if (points.size() < 2) return result;

    double n = points.size();
    double sum_t = 0, sum_lat = 0, sum_lon = 0;
    double sum_t_sq = 0, sum_t_lat = 0, sum_t_lon = 0;

    for (const auto& p : points) {
        double t = p.timestamp.dbl();
        sum_t += t;
        sum_lat += p.latitude;
        sum_lon += p.longitude;
        sum_t_sq += t * t;
        sum_t_lat += t * p.latitude;
        sum_t_lon += t * p.longitude;
    }

    double denominator = n * sum_t_sq - sum_t * sum_t;
    if (std::abs(denominator) < 1e-9) return result;

    result.b_lat = (n * sum_t_lat - sum_t * sum_lat) / denominator;
    result.a_lat = (sum_lat - result.b_lat * sum_t) / n;
    result.b_lon = (n * sum_t_lon - sum_t * sum_lon) / denominator;
    result.a_lon = (sum_lon - result.b_lon * sum_t) / n;
    
    result.valid = true;
    return result;
}

void TrajektoriApp::finish()
{
    if (mLogFile.is_open()) mLogFile.close();
    cancelAndDelete(mLogTimer);

    if (mCoefficientLogFile.is_open()) {
        // Cetak kalkulasi MAE otomatis di baris bawah (Sama seperti KF)
        if (mCountAE > 0) {
            double mae_microdegree = mTotalAE / mCountAE;
            double mae_meter = mae_microdegree * 0.11132;
            
            mCoefficientLogFile << std::endl; 
            mCoefficientLogFile << ";;;;;;;;;;;;MAE (microdegree);" << std::fixed << std::setprecision(12) << mae_microdegree << std::endl;
            mCoefficientLogFile << ";;;;;;;;;;;;MAE (meter);" << mae_meter << std::endl;
        }
        mCoefficientLogFile.close();
    }
    cancelAndDelete(mPredictionTimer);
    
    ItsG5BaseService::finish();
}
} // namespace artery

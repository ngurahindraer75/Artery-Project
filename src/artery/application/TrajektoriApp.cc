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
#include <chrono>

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
    // --- Original Initialization Logic (Untouched) ---
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
        mLogFile << "GenDeltaTime;Time_send_s;Time_received_s;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Target_Speed_Raw" << std::endl;
    }

    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);
    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);
    // --- End of Original Initialization Logic ---

    // --- ADDITION: New Logic for Coefficient Prediction ---
    std::string coefficientLogFilename;
    if (nodeType == "Vehicle") {
        coefficientLogFilename = "results/coefficient_log_from_car.csv";
    } else { // Also for Person and Unknown
        coefficientLogFilename = "results/coefficient_log_from_person.csv";
    }
    mCoefficientLogFile.open(coefficientLogFilename, std::ios::out | std::ios::app);
    if (mCoefficientLogFile.tellp() == 0) {
        mCoefficientLogFile << "Time_prediction(s);Time_absolut_s;Node_Target;Actual_Lat;Actual_Lon;Slope_Lat;Intercept_Lat;Pred_Lat;Slope_Lon;Intercept_Lon;Pred_Lon;Lat_AE;Lon_AE;AE" << std::endl;
    }
    mPredictionTimer = new cMessage("predictionTimer");
    // Schedule the new timer to run every 1 second
    scheduleAt(simTime() + 1.0, mPredictionTimer);
    // --- END OF ADDITION ---
}

void TrajektoriApp::handleMessage(cMessage* msg)
{
    // --- Original Timer Handling (Untouched) ---
    if (msg == mLogTimer) {
        logTrajectory();
        scheduleAt(simTime() + mLogInterval, mLogTimer);
        return; // Return to avoid deleting the message
    }
    // --- End of Original Timer Handling ---

    // --- ADDITION: Handling for the new prediction timer ---
    if (msg == mPredictionTimer) {
        logCoefficients(); // Call the new function
        scheduleAt(simTime() + 1.0, mPredictionTimer); // Reschedule for the next second
        return; // Return to avoid deleting the message
    }
    // --- END OF ADDITION ---
    
    // Default case to clean up other messages
    delete msg;
}

void TrajektoriApp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    if (signalID != mCamReceivedSignal) {
        return;
    }
    auto ca_obj = dynamic_cast<CaObject*>(obj);
    if (!ca_obj) {
        return;
    }
    const auto& cam = *ca_obj->asn1();
    long stationId = cam.header.stationID;
    const auto& basic = cam.cam.camParameters.basicContainer;
    const auto& hfc = cam.cam.camParameters.highFrequencyContainer;

    if (hfc.present != HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
        return;
    }
    const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;

    MovementData data;
        // --- TAMBAHAN BARU: Simpan nilai aslinya tanpa modifikasi ---
    data.genDeltaTime = static_cast<uint16_t>(cam.cam.generationDeltaTime);

    // --- 1. ABSOLUTE RECEPTION TIME (Time_received) ---
    omnetpp::simtime_t rxTime = simTime(); 
    data.receptionTime = rxTime; 

    // --- 2. ACCESSING TAI ETSI TIME ZONE ---
    // Akses fasilitas Timer Artery untuk sinkronisasi waktu TAI
    const artery::Timer& timer = getFacilities().get_const<artery::Timer>();
    auto rx_tai = timer.getTimeFor(rxTime);
    
    // Waktu TAI penerimaan dalam format milidetik
    int64_t rx_tai_ms = std::chrono::duration_cast<std::chrono::milliseconds>(rx_tai.time_since_epoch()).count();

    // --- 3. REKONSTRUKSI LATENSI DARI MODULO 65536 ---
    // Waktu pengiriman TAI termodulo dari pesan CAM (0 - 65535)
    uint16_t tx_tai_mod = static_cast<uint16_t>(cam.cam.generationDeltaTime);
    
    // Modulokan juga waktu penerimaan TAI dengan 65536
    uint16_t rx_tai_mod = static_cast<uint16_t>(rx_tai_ms % 65536);

    // Hitung Latensi Transmisi Jaringan (dalam milidetik)
    int64_t latency_ms;
    if (rx_tai_mod >= tx_tai_mod) {
        latency_ms = rx_tai_mod - tx_tai_mod;
    } else {
        // Terjadi wrap-around (waktu melewati batas 65.536 milidetik / ~65,5 detik)
        latency_ms = (rx_tai_mod + 65536) - tx_tai_mod;
    }

    // --- 4. WAKTU PENGIRIMAN ABSOLUT (Time_send) ---
    // Waktu pengiriman = Waktu penerimaan dikurangi latensi
    data.timestamp = rxTime - omnetpp::SimTime(latency_ms, omnetpp::SIMTIME_MS);
    
    // Continue with coordinate extraction
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
            if (targetHist.history.empty()) {
                continue;
            }
            MovementData latest = targetHist.history.back();
            mLogFile    << latest.genDeltaTime << ";"
                        << std::fixed << std::setprecision(12)
                        << latest.timestamp.dbl() << ";" // print timestamp when the CAM was Created
                        << latest.receptionTime.dbl() << ";" // print timestamp when the CAM was received
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

// This function is called every 1 second by the new timer
void TrajektoriApp::logCoefficients()
{
    simtime_t now = simTime();
    
    // --- PARAMETER SKENARIO PREDIKSI ---
    // Ubah horizon_s menjadi 2.0 untuk pengujian "Mid-Term Prediction"
    double horizon_s = 1.0; 
    double window_s = 1.0;  // Jendela data histori (1 detik)
    
    // Target waktu yang ingin diprediksi dan dievaluasi
    double target_time = now.dbl() - horizon_s; 
    
    // Batas akhir waktu data yang boleh dipakai untuk meregresi prediksi ini
    double prediction_creation_time = target_time - horizon_s; 

    for (auto const& [targetId, targetHist] : mOtherNodes)
    {
        // --- Cek Timeout (Data Stale) ---
        if ((now - targetHist.lastReceptionTime).dbl() > 2.0) {
            continue; // Lewati perhitungan regresi untuk node ini
        }
        
        std::vector<MovementData> regression_points;
        MovementData closest_actual_data;
        double min_time_diff = 9999.0;
        bool found_actual = false;
        
        for (const auto& point : targetHist.history) {
            double t = point.timestamp.dbl();
            
            // 1. Kumpulkan data Regresi di jendela masa lalu
            if (t <= prediction_creation_time && t > (prediction_creation_time - window_s)) {
                regression_points.push_back(point);
            }
            
            // 2. Cari data Ground Truth riil yang PALING MENDEKATI target_time
            double time_diff = std::abs(t - target_time);
            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_actual_data = point;
                found_actual = true;
            }
        }
        
        // 3. Fallback logic: Jika data regresi di jendela tersebut kurang dari 2 titik
        if (regression_points.size() < 2) {
            regression_points.clear();
            std::vector<MovementData> past_points;
            for (const auto& point : targetHist.history) {
                if (point.timestamp.dbl() <= prediction_creation_time) {
                    past_points.push_back(point);
                }
            }
            // Ambil 2 titik data terakhir yang tersedia
            if (past_points.size() >= 2) {
                regression_points.push_back(past_points[past_points.size() - 2]);
                regression_points.push_back(past_points.back());
            }
        }
        
        // 4. Kalkulasi Prediksi dan Cetak Log Terpadu
        if (regression_points.size() >= 2 && found_actual) {
            RegressionCoefficients coeffs = calculateCoefficients(regression_points);
            if (coeffs.valid) {
                double pred_lat = coeffs.a_lat + (coeffs.b_lat * target_time);
                double pred_lon = coeffs.a_lon + (coeffs.b_lon * target_time);
                
                // --- Perhitungan Absolute Error (AE) ---
                double lat_ae = std::abs(closest_actual_data.latitude - pred_lat);
                double lon_ae = std::abs(closest_actual_data.longitude - pred_lon);
                // Jarak Euclidean 2D (Spatial Error)
                double ae = std::sqrt((lat_ae * lat_ae) + (lon_ae * lon_ae)); 
                
                // Akumulasi untuk perhitungan MAE akhir
                mTotalAE += ae;
                mCountAE++;
                // ------------------------------------------------------
                
                mCoefficientLogFile << std::fixed << std::setprecision(12)
                                    << target_time << ";"
                                    << closest_actual_data.timestamp.dbl() << ";"
                                    << targetId << ";"
                                    << closest_actual_data.latitude << ";"
                                    << closest_actual_data.longitude << ";"
                                    << coeffs.b_lat << ";"
                                    << coeffs.a_lat << ";"
                                    << pred_lat << ";"
                                    << coeffs.b_lon << ";"
                                    << coeffs.a_lon << ";"
                                    << pred_lon << ";"       
                                    << lat_ae << ";"         // Kolom Lat_AE
                                    << lon_ae << ";"         // Kolom Lon_AE
                                    << ae << std::endl;      // Kolom AE

                }
            }                   
    }    
    mCoefficientLogFile.flush();
}

// This helper function calculates the coefficients
RegressionCoefficients TrajektoriApp::calculateCoefficients(const std::vector<MovementData>& points)
{
    RegressionCoefficients result;
    if (points.size() < 2) {
        return result; 
    }
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
    if (std::abs(denominator) < 1e-9) {
        return result;
    }
    result.b_lat = (n * sum_t_lat - sum_t * sum_lat) / denominator;
    result.a_lat = (sum_lat - result.b_lat * sum_t) / n;
    result.b_lon = (n * sum_t_lon - sum_t * sum_lon) / denominator;
    result.a_lon = (sum_lon - result.b_lon * sum_t) / n;
    result.valid = true;
    return result;
}

// --- END OF ADDITION ---


void TrajektoriApp::finish()
{
    // --- Original Finish Logic (Untouched) ---
    if (mLogFile.is_open()) mLogFile.close();
    cancelAndDelete(mLogTimer);

    // --- ADDITION: Clean up new resources & CETAK MAE ---
    if (mCoefficientLogFile.is_open()) {
        
        // Cetak kalkulasi MAE di bagian paling bawah jika ada data
        if (mCountAE > 0) {
            double mae_microdegree = mTotalAE / mCountAE;
            double mae_meter = mae_microdegree * 0.11132; // Konversi spasial
            
            // Memberikan sela 1 baris kosong
            mCoefficientLogFile << std::endl; 
            
            // Mencetak dengan 12 titik koma agar labelnya jatuh sejajar di kolom Lon_AE dan nilainya persis di bawah kolom AE
            mCoefficientLogFile << ";;;;;;;;;;;;MAE (microdegree);" << std::fixed << std::setprecision(12) << mae_microdegree << std::endl;
            mCoefficientLogFile << ";;;;;;;;;;;;MAE (meter);" << mae_meter << std::endl;
        }
        
        mCoefficientLogFile.close();
    }
    cancelAndDelete(mPredictionTimer);
    // --- END OF ADDITION ---

    ItsG5BaseService::finish();
}

} // namespace artery

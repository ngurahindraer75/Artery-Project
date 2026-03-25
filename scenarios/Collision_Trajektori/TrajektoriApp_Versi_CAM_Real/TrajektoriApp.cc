/#include "artery/application/TrajektoriApp.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include "artery/application/Middleware.h"
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <cmath>

namespace artery {
using namespace omnetpp;

Define_Module(TrajektoriApp);

// Inisialisasi modul, membuka file log, dan menjadwalkan timer
void TrajektoriApp::initialize()
{
    ItsG5BaseService::initialize();

    mLogInterval = par("logInterval");
    mStaticSpeedThreshold = par("staticSpeedThreshold");

    // Menentukan nama file log berdasarkan tipe node (Vehicle, Person, dll.)
    std::string nodeType = getNodeType();
    std::string logFilename;
    if (nodeType == "Vehicle") {
        logFilename = "results/trajektori_log_from_car.csv";
        mPredictionFile.open("results/trajektori_from_car.csv");
    } else if (nodeType == "Person") {
        logFilename = "results/trajektori_log_from_person.csv";
        mPredictionFile.open("results/trajektori_from_person.csv");
    } else {
        logFilename = "results/trajektori_log_unknown.csv";
        mPredictionFile.open("results/trajektori_from_unknown.csv");
    }

    mLogFile.open(logFilename, std::ios::out | std::ios::app);

    // Menulis header ke file log jika file baru dibuat
    if (mLogFile.tellp() == 0) {
        mLogFile << "Time;Observer_ID;Observer_Lat;Observer_Lon;Observer_Speed;Observer_Status;"
                 << "Target_ID;Target_Lat;Target_Lon;Target_Speed;Target_Status;Comm_Status" << std::endl;
    }

    // Menulis header ke file prediksi
    mPredictionFile << "Time;Observer_ID;Observer_Lat;Observer_Lon;Observer_Speed;"
                    << "Target_ID;Prediction_Lat;Prediction_Lon;Target_Speed" << std::endl;

    // Inisialisasi dan penjadwalan timer untuk logging dan prediksi
    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);

    mPredictionTimer = new cMessage("predictionTimer");
    scheduleAt(simTime() + 1.0, mPredictionTimer);

    // Mendaftarkan sinyal untuk menerima CAM
    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);
}

// Handler untuk pesan timer
void TrajektoriApp::handleMessage(cMessage* msg)
{
    if (msg == mLogTimer) {
        logTrajectory();
        scheduleAt(simTime() + mLogInterval, mLogTimer);
    } else if (msg == mPredictionTimer) {
        predictTrajectory();
        scheduleAt(simTime() + 1.0, mPredictionTimer);
    } else {
        delete msg;
    }
}

// Handler untuk sinyal penerimaan CAM
void TrajektoriApp::receiveSignal(cComponent*, simsignal_t signalID, cObject* obj, cObject*)
{
    if (signalID != mCamReceivedSignal) return;

    auto ca_obj = dynamic_cast<CaObject*>(obj);
    if (!ca_obj) return;

    const auto& cam = *ca_obj->asn1();
    long stationId = cam.header.stationID;

    // Ekstraksi data dari CAM
    const auto& basic = cam.cam.camParameters.basicContainer;
    const auto& hfc = cam.cam.camParameters.highFrequencyContainer;
    const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;

    MovementData data;
    data.timestamp = simTime();
    data.latitude = static_cast<double>(basic.referencePosition.latitude) * 1e-7;
    data.longitude = static_cast<double>(basic.referencePosition.longitude) * 1e-7;
    data.speed_mps = static_cast<double>(bvc.speed.speedValue) * 0.01;

    // Menyimpan data pergerakan ke dalam histori node terkait
    AgentHistory& history = mOtherNodes[stationId];
    history.history.push_back(data);
    history.lastReceptionTime = simTime();

    // Menghapus data histori yang lebih tua dari 5 detik
    while (!history.history.empty() && simTime() - history.history.front().timestamp > 5.0) {
        history.history.pop_front();
    }
}

// Mengklasifikasikan status agen (bergerak atau diam)
AgentState TrajektoriApp::classifyState(const AgentHistory& history)
{
    if (history.history.empty()) return AgentState::STATIC;

    double total = 0.0;
    for (const auto& data : history.history) {
        total += data.speed_mps;
    }
    double avgSpeed = total / history.history.size();
    return avgSpeed < mStaticSpeedThreshold ? AgentState::STATIC : AgentState::DYNAMIC;
}

// Mengonversi enum AgentState menjadi string
std::string TrajektoriApp::stateToString(AgentState state)
{
    return (state == AgentState::STATIC) ? "STATIC" : "DYNAMIC";
}

// Mencatat data trajektori ke file log
void TrajektoriApp::logTrajectory()
{
    auto& vdp = getFacilities().get_const<VehicleDataProvider>();
    long myId = vdp.station_id();
    double myLat = vdp.latitude().value();
    double myLon = vdp.longitude().value();
    double mySpeed = vdp.speed().value();

    // Memperbarui histori diri sendiri
    mSelfHistory.history.push_back({simTime(), myLat, myLon, mySpeed});
    while (!mSelfHistory.history.empty() && simTime() - mSelfHistory.history.front().timestamp > 5.0) {
        mSelfHistory.history.pop_front();
    }

    AgentState myState = classifyState(mSelfHistory);
    double avgMySpeed = computeAverageSpeed(mSelfHistory);

    if (!mLogFile.is_open()) return;

    // Mencatat data untuk setiap node lain yang terdeteksi
    for (auto& [targetId, targetHist] : mOtherNodes) {
        if (targetHist.history.empty()) {
            continue;
        }

        MovementData latest = targetHist.history.back();
        std::string commStatus = (simTime() - targetHist.lastReceptionTime > 1.0) ? "OUT_OF_RANGE" : "IN_RANGE";

        // Logika untuk menentukan status target
        AgentState predictedTargetState;
        if (commStatus == "IN_RANGE") {
            predictedTargetState = (latest.speed_mps > mStaticSpeedThreshold) ? AgentState::DYNAMIC : AgentState::STATIC;
        } else {
            predictedTargetState = AgentState::STATIC;
        }

        double latestTargetSpeed = latest.speed_mps;

        // Menulis baris log
        mLogFile << simTime().dbl() << ";"
                 << myId << ";" << myLat << ";" << myLon << ";" << avgMySpeed << ";" << stateToString(myState) << ";"
                 << targetId << ";" << latest.latitude << ";" << latest.longitude << ";" << latestTargetSpeed << ";" << stateToString(predictedTargetState) << ";"
                 << commStatus << std::endl;
    }
    mLogFile.flush();
}

// --- FUNGSI PREDIKSI YANG TELAH DIKONFIRMASI ---
// Memprediksi trajektori masa depan berdasarkan data historis
void TrajektoriApp::predictTrajectory()
{
    auto& vdp = getFacilities().get_const<VehicleDataProvider>();
    long myId = vdp.station_id();
    double myLat = vdp.latitude().value();
    double myLon = vdp.longitude().value();
    double avgMySpeed = computeAverageSpeed(mSelfHistory);

    if (!mPredictionFile.is_open()) return;

    for (auto& [targetId, hist] : mOtherNodes) {
        // =========================================================================
        // POIN #2: Mengumpulkan data dari jendela waktu 1 detik terakhir
        // =========================================================================
        std::deque<MovementData> window;
        // Iterasi melalui semua riwayat yang disimpan untuk node target
        for (const auto& d : hist.history) {
            // Ambil data jika timestamp-nya berada dalam 1.0 detik dari waktu simulasi saat ini
            if (simTime() - d.timestamp <= 1.0) {
                window.push_back(d);
            }
        }

        std::deque<MovementData> prediction_points;

        // =========================================================================
        // POIN #3: Logika untuk menangani jumlah titik data yang berbeda
        // =========================================================================
        if (window.size() == 1) {
            // KASUS: Hanya 1 titik data dalam jendela 1 detik.
            // Cari titik data valid terakhir sebelum jendela ini.
            MovementData last_known_point;
            bool found_last_known = false;
            
            // Cari mundur dari data terbaru di seluruh riwayat
            for (auto it = hist.history.rbegin(); it != hist.history.rend(); ++it) {
                if (it->timestamp < (simTime() - 1.0)) {
                    last_known_point = *it;
                    found_last_known = true;
                    break; // Titik ditemukan, keluar dari loop
                }
            }

            if (found_last_known) {
                // Jika ditemukan, gunakan titik historis dan titik saat ini untuk prediksi
                prediction_points.push_back(last_known_point);
                prediction_points.push_back(window.front());
            }
        } else if (window.size() >= 2) {
            // KASUS: 2 atau lebih titik data. Gunakan semua titik dalam jendela.
            prediction_points = window;
        }

        // =========================================================================
        // Lakukan prediksi jika ada cukup titik data (minimal 2)
        // =========================================================================
        if (prediction_points.size() >= 2) {
            auto [baseLat, slopeLat] = linearRegression(prediction_points, true);  // Regresi untuk Latitude
            auto [baseLon, slopeLon] = linearRegression(prediction_points, false); // Regresi untuk Longitude

            // Waktu awal (t0) adalah timestamp dari titik pertama yang digunakan untuk regresi
            double t0 = prediction_points.front().timestamp.dbl();
            // Waktu prediksi adalah 1 detik di masa depan dari waktu simulasi saat ini
            double future_time_relative = (simTime().dbl() + 1.0) - t0;

            // Hitung posisi yang diprediksi
            double predLat = baseLat + slopeLat * future_time_relative;
            double predLon = baseLon + slopeLon * future_time_relative;
            double avgTargetSpeed = computeAverageSpeed(hist); // Gunakan kecepatan rata-rata target

            // Tulis hasil prediksi ke file
            mPredictionFile << simTime().dbl() << ";" << myId << ";" << myLat << ";" << myLon << ";" << avgMySpeed << ";"
                            << targetId << ";" << predLat << ";" << predLon << ";" << avgTargetSpeed << std::endl;
        }
    }
    mPredictionFile.flush();
}


// Melakukan regresi linear pada data historis
std::pair<double, double> TrajektoriApp::linearRegression(const std::deque<MovementData>& history, bool useLat)
{
    int n = history.size();
    if (n < 2) return {0, 0}; // Regresi tidak mungkin dengan kurang dari 2 titik

    double t0 = history.front().timestamp.dbl(); // Waktu referensi awal
    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

    for (const auto& d : history) {
        double x = d.timestamp.dbl() - t0; // Waktu relatif
        double y = useLat ? d.latitude : d.longitude;
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    // Menghindari pembagian dengan nol
    double denominator = (n * sumX2 - sumX * sumX);
    if (std::abs(denominator) < 1e-9) return {0, 0};

    double slope = (n * sumXY - sumX * sumY) / denominator;
    double intercept = (sumY - slope * sumX) / n;
    return {intercept, slope};
}

// Mendapatkan tipe node dari nama modul NED
std::string TrajektoriApp::getNodeType()
{
    std::string type = getParentModule()->getNedTypeName();
    if (type.find("Vehicle") != std::string::npos) return "Vehicle";
    if (type.find("Person") != std::string::npos) return "Person";
    return "Unknown";
}

// Menghitung kecepatan rata-rata dari histori
double TrajektoriApp::computeAverageSpeed(const AgentHistory& history)
{
    if (history.history.empty()) return 0.0;

    double total = 0.0;
    for (const auto& data : history.history) {
        total += data.speed_mps;
    }
    return total / history.history.size();
}

// Membersihkan sumber daya saat simulasi berakhir
void TrajektoriApp::finish()
{
    if (mLogFile.is_open()) mLogFile.close();
    if (mPredictionFile.is_open()) mPredictionFile.close();
    cancelAndDelete(mLogTimer);
    cancelAndDelete(mPredictionTimer);
    ItsG5BaseService::finish();
}

} // namespace artery

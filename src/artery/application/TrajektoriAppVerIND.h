#ifndef ARTERY_TRAJEKTORIAPPVERIND_H_
#define ARTERY_TRAJEKTORIAPPVERIND_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include "KalmanFilterIND.h" // <- PENTING: Include file KF kita
#include <omnetpp/simtime.h>
#include <deque>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <memory> // Untuk std::unique_ptr

namespace artery
{

// Struct data mentah (Sama seperti v1)
struct MovementData {
    omnetpp::simtime_t timestamp;
    double latitude;
    double longitude;
    double speed_mps;
};

// Struct history (Dimodifikasi untuk KF)
struct AgentHistory {
    std::deque<MovementData> history; // Masih berguna untuk log mentah
    omnetpp::simtime_t lastReceptionTime;
    bool hasNewData = false;

    // --- PERUBAHAN UTAMA ---
    // Setiap agen sekarang memiliki instance Kalman Filter-nya sendiri
    std::unique_ptr<KalmanFilterIND> kf_state;
    // ----------------------
};

// Ganti nama class
class TrajektoriAppVerIND : public ItsG5BaseService
{
    public:
        void initialize() override;
        void finish() override;

    protected:
        void handleMessage(omnetpp::cMessage* msg) override;
        void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;

    private:
        // Fungsi lama untuk log mentah (tidak berubah)
        void logTrajectory();
        std::string getNodeType();

        // --- PERUBAHAN UTAMA ---
        // Ganti nama dari logCoefficients menjadi runPredictions
        void runPredictions();
        // ----------------------

        // Variabel lama (tidak berubah)
        omnetpp::cMessage* mLogTimer = nullptr;
        omnetpp::simsignal_t mCamReceivedSignal;
        omnetpp::SimTime mLogInterval;
        std::ofstream mLogFile;
        std::map<long, AgentHistory> mOtherNodes;
        const artery::VehicleDataProvider* mVehicleDataProvider = nullptr;
        
        // --- PERUBAHAN UTAMA ---
        // Ganti nama timer dan file log
        omnetpp::cMessage* mPredictionTimer = nullptr;
        std::ofstream mPredictionLogFile;
        // ----------------------
};

} // namespace artery

#endif /* ARTERY_TRAJEKTORIAPPVERIND_H_ */

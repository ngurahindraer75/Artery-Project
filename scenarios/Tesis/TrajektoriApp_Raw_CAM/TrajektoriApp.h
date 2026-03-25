#ifndef ARTERY_TRAJEKTORIAPP_H_
#define ARTERY_TRAJEKTORIAPP_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include <omnetpp/simtime.h>
#include <deque>
#include <fstream>
#include <string>
#include <map>

namespace artery
{

// Struct disesuaikan untuk menyimpan data mentah
struct MovementData {
    omnetpp::simtime_t timestamp; // Waktu penerimaan (untuk manajemen histori)
    double latitude;              // Nilai mentah Lintang
    double longitude;             // Nilai mentah Bujur
    double speed_mps;             // Nilai mentah Kecepatan
};

// Struct untuk menyimpan histori pergerakan dari setiap node yang terdeteksi
struct AgentHistory {
    std::deque<MovementData> history;
    omnetpp::simtime_t lastReceptionTime;
    // PERUBAHAN: Bendera untuk menandai adanya data baru
    bool hasNewData = false;
};

class TrajektoriApp : public ItsG5BaseService
{
    public:
        void initialize() override;
        void finish() override;

    protected:
        void handleMessage(omnetpp::cMessage* msg) override;
        void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;

    private:
        void logTrajectory();
        std::string getNodeType();

        // Deklarasi variabel anggota yang digunakan di file .cc
        omnetpp::cMessage* mLogTimer = nullptr;
        omnetpp::simsignal_t mCamReceivedSignal;
        omnetpp::SimTime mLogInterval;
        std::ofstream mLogFile;
        std::map<long, AgentHistory> mOtherNodes;
        const artery::VehicleDataProvider* mVehicleDataProvider = nullptr;
};

} // namespace artery

#endif /* ARTERY_TRAJEKTORIAPP_H_ */

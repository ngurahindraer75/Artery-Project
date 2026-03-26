#ifndef ARTERY_TRAJEKTORIAPPVERIND_H_
#define ARTERY_TRAJEKTORIAPPVERIND_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include "KalmanFilterIND.h" 
#include <omnetpp/simtime.h>
#include <deque>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <memory> 

namespace artery
{
    struct MovementData {
        omnetpp::simtime_t timestamp;
        double latitude;
        double longitude;
        double speed_mps;
    };

    // --- STRUKTUR BARU: Buffer Prediksi Gantung ---
    struct PendingPrediction {
        double target_time;
        double creation_time;
        double slope_lat;
        double intercept_lat;
        double slope_lon;
        double intercept_lon;
    };

    struct AgentHistory {
        std::deque<MovementData> history; 
        omnetpp::simtime_t lastReceptionTime;
        bool hasNewData = false;
        
        std::unique_ptr<KalmanFilterIND> kf_state;
        std::vector<PendingPrediction> pending_predictions; // Memori prediksi gantung
    };

    class TrajektoriAppVerIND : public ItsG5BaseService
    {
    public:
        void initialize() override;
        void finish() override;

    protected:
        void handleMessage(omnetpp::cMessage* msg) override;
        void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID,
                           omnetpp::cObject* obj, omnetpp::cObject* details) override;

    private:
        void logTrajectory();
        std::string getNodeType();
        void runPredictions();

        omnetpp::cMessage* mLogTimer = nullptr;
        omnetpp::simsignal_t mCamReceivedSignal;
        omnetpp::SimTime mLogInterval;
        std::ofstream mLogFile;
        std::map<long, AgentHistory> mOtherNodes;
        const artery::VehicleDataProvider* mVehicleDataProvider = nullptr;

        omnetpp::cMessage* mPredictionTimer = nullptr;
        std::ofstream mPredictionLogFile;
        
        // --- VARIABEL PELACAK MAE ---
        double mTotalAE = 0.0;
        int mCountAE = 0;
    };
} // namespace artery

#endif /* ARTERY_TRAJEKTORIAPPVERIND_H_ */

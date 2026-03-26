#ifndef ARTERY_TRAJEKTORIAPP_H_
#define ARTERY_TRAJEKTORIAPP_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include <omnetpp/simtime.h>
#include <deque>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <cmath>

namespace artery
{
    struct MovementData {
        omnetpp::simtime_t timestamp;
        double latitude;
        double longitude;
        double speed_mps;
    };

    // --- STRUKTUR BARU: Buffer Prediksi Gantung untuk RL ---
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
        
        // Memori prediksi gantung untuk evaluasi tertunda
        std::vector<PendingPrediction> pending_predictions; 
    };

    struct RegressionCoefficients {
        bool valid = false;
        double a_lat = 0.0, b_lat = 0.0; // Intercept & Slope Lat
        double a_lon = 0.0, b_lon = 0.0; // Intercept & Slope Lon
    };

    class TrajektoriApp : public ItsG5BaseService
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
        void logCoefficients(); // Fungsi eksekusi prediksi RL
        RegressionCoefficients calculateCoefficients(const std::vector<MovementData>& points);

        omnetpp::cMessage* mLogTimer = nullptr;
        omnetpp::simsignal_t mCamReceivedSignal;
        omnetpp::SimTime mLogInterval;
        std::ofstream mLogFile;
        std::map<long, AgentHistory> mOtherNodes;
        const artery::VehicleDataProvider* mVehicleDataProvider = nullptr;

        omnetpp::cMessage* mPredictionTimer = nullptr;
        std::ofstream mCoefficientLogFile;

        // --- VARIABEL PELACAK MAE ---
        double mTotalAE = 0.0;
        int mCountAE = 0;
    };
} // namespace artery

#endif /* ARTERY_TRAJEKTORIAPP_H_ */

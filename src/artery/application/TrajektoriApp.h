#ifndef ARTERY_TRAJEKTORIAPP_H_
#define ARTERY_TRAJEKTORIAPP_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include <omnetpp/simtime.h>
#include <deque>
#include <fstream>
#include <string>
#include <map>
#include <vector> // Added for easier data handling

namespace artery
{

// Struct to hold raw movement data from CAMs
struct MovementData {
    uint16_t genDeltaTime;              // <-- TAMBAHAN BARU: Nilai mentah 
    omnetpp::simtime_t timestamp;       // Time when the CAM was sent
    omnetpp::simtime_t receptionTime;   // Time when the CAM was received
    double latitude;
    double longitude;
    double speed_mps;
};

// Struct to hold the history of a detected node
struct AgentHistory {
    std::deque<MovementData> history;
    omnetpp::simtime_t lastReceptionTime;
    bool hasNewData = false;
};

// ADDITION: New struct to hold only the regression coefficients
struct RegressionCoefficients
{
    bool valid = false;
    double a_lat = 0.0, b_lat = 0.0; // Intercept and Slope for Latitude
    double a_lon = 0.0, b_lon = 0.0; // Intercept and Slope for Longitude
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
        // Original functions
        void logTrajectory();
        std::string getNodeType();

        // ADDITION: New functions for coefficient calculation
        void logCoefficients();
        RegressionCoefficients calculateCoefficients(const std::vector<MovementData>& points);

        // --- TAMBAHAN BARU: Variabel pelacak MAE ---
        double mTotalAE = 0.0;
        int mCountAE = 0;

        // Original member variables
        omnetpp::cMessage* mLogTimer = nullptr;
        omnetpp::simsignal_t mCamReceivedSignal;
        omnetpp::SimTime mLogInterval;
        std::ofstream mLogFile;
        std::map<long, AgentHistory> mOtherNodes;
        const artery::VehicleDataProvider* mVehicleDataProvider = nullptr;
        
        // ADDITION: New member variables for the new logic
        omnetpp::cMessage* mPredictionTimer = nullptr;
        std::ofstream mCoefficientLogFile;
};

} // namespace artery

#endif /* ARTERY_TRAJEKTORIAPP_H_ */

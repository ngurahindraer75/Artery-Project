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
        mLogFile << "Time_s;Observer_ID;Target_ID;Target_Lat_Raw;Target_Lon_Raw;Target_Speed_Raw" << std::endl;
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
        mCoefficientLogFile << "Time_prediction(s);Node_Target;Slope_Lat;Intercept_Lat;Slope_Lon;Intercept_Lon" << std::endl;
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
    // This entire function remains untouched as its job is only to collect data.
    if (signalID != mCamReceivedSignal) {
        return;
    }
    auto ca_obj = dynamic_cast<CaObject*>(obj);
    if (!ca_obj) {
        return;
    }
    const cPacket* packet = dynamic_cast<const cPacket*>(details);
    simtime_t creationTime = packet ? packet->getCreationTime() : simTime();
    const auto& cam = *ca_obj->asn1();
    long stationId = cam.header.stationID;
    const auto& basic = cam.cam.camParameters.basicContainer;
    const auto& hfc = cam.cam.camParameters.highFrequencyContainer;
    if (hfc.present != HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
        return;
    }
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

// --- Original logTrajectory function (Untouched) ---
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
            mLogFile << std::fixed << std::setprecision(12)
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
// --- End of Original logTrajectory function ---


// --- ADDITION: New functions to implement the requested logic ---

// This function is called every 1 second by the new timer
void TrajektoriApp::logCoefficients()
{
    simtime_t now = simTime();
    for (auto const& [targetId, targetHist] : mOtherNodes)
    {
        std::vector<MovementData> points_1s;
        // 1. Collect data points within the ideal 1-second window
        for (const auto& point : targetHist.history) {
            if (now - point.timestamp <= 1.0 && now > point.timestamp) { // ensure we are using past data
                points_1s.push_back(point);
            }
        }

        // 2. Apply fallback logic if needed
        if (points_1s.size() < 2 && targetHist.history.size() >= 2) {
            points_1s.clear();
            points_1s.push_back(targetHist.history[targetHist.history.size() - 2]);
            points_1s.push_back(targetHist.history.back());
        }

        // 3. Calculate coefficients if we have enough points
        if (points_1s.size() >= 2) {
            RegressionCoefficients coeffs = calculateCoefficients(points_1s);
            if (coeffs.valid) {
                // 4. Log the results to the new file
                mCoefficientLogFile << std::fixed << std::setprecision(12)
                                  << (now + 1.0).dbl() << ";" // Time prediction is 1s from now
                                  << targetId << ";"
                                  << coeffs.b_lat << ";" << coeffs.a_lat << ";"
                                  << coeffs.b_lon << ";" << coeffs.a_lon << std::endl;
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
    // --- End of Original Finish Logic ---

    // --- ADDITION: Clean up new resources ---
    if (mCoefficientLogFile.is_open()) mCoefficientLogFile.close();
    cancelAndDelete(mPredictionTimer);
    // --- END OF ADDITION ---

    ItsG5BaseService::finish();
}

} // namespace artery

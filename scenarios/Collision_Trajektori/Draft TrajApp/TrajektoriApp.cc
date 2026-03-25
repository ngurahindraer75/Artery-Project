#include "artery/application/TrajektoriApp.h"
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

void TrajektoriApp::initialize()
{
    ItsG5BaseService::initialize();

    mLogInterval = par("logInterval");
    mStaticSpeedThreshold = par("staticSpeedThreshold");
    
    const char* filename = par("logFileName").stringValue();
    mLogFile.open(filename, std::ios::out | std::ios::app);

    if (mLogFile.tellp() == 0) {
        mLogFile << "Time;Observer_ID;Observer_Lat;Observer_Lon;Observer_Speed;Observer_Status;"
                 << "Target_ID;Target_Lat;Target_Lon;Target_Speed;Target_Status;Comm_Status" << std::endl;
    }
    
    std::string nodeType = getNodeType();
    if (nodeType == "Vehicle") {
        mPredictionFile.open("results/trajektori_from car.csv");
    } else {
        mPredictionFile.open("results/trajektori_from person.csv");
    }
    mPredictionFile << "Time;Observer_ID;Observer_Lat;Observer_Lon;Observer_Speed;"
                    << "Target_ID;Prediction_Lat;Prediction_Lon;Target_Speed" << std::endl;

    
    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);
    
    mPredictionTimer = new cMessage("predictionTimer");
	scheduleAt(simTime() + 1.0, mPredictionTimer);

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);

    EV_INFO << "[TrajektoriApp] Initialized and subscribed to CamReceived signal." << std::endl;
}

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

void TrajektoriApp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject*)
{
    if (signalID == mCamReceivedSignal) {
        auto ca_obj = dynamic_cast<CaObject*>(obj);
        if (!ca_obj) return;

        const vanetza::asn1::Cam& cam_wrapper = ca_obj->asn1();
        const CAM_t& cam = *cam_wrapper;
        long stationId = cam.header.stationID;

        const auto& basic = cam.cam.camParameters.basicContainer;
        const auto& hfc = cam.cam.camParameters.highFrequencyContainer;
        const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;

        MovementData data;
        data.timestamp = simTime();
        data.latitude = static_cast<double>(basic.referencePosition.latitude) * 1e-7;
        data.longitude = static_cast<double>(basic.referencePosition.longitude) * 1e-7;
        data.speed_mps = static_cast<double>(bvc.speed.speedValue) * 0.01;
		
		AgentHistory& history = mOtherNodes[stationId];
        history.history.push_back(data);
        history.lastReceptionTime = simTime();

        // Hapus data lama > 5 detik
        auto& hist = history.history;
        while (!hist.empty() && (simTime() - hist.front().timestamp > 5.0)) {
            hist.pop_front();
        }
    }
}

AgentState TrajektoriApp::classifyState(const AgentHistory& history)
{
    double avgSpeed = computeAverageSpeed(history);
    return avgSpeed < mStaticSpeedThreshold ? AgentState::STATIC : AgentState::DYNAMIC;
}

double TrajektoriApp::computeAverageSpeed(const AgentHistory& history)
{
    if (history.history.empty()) return 0.0;

    double total = 0.0;
    for (const auto& data : history.history) {
        total += data.speed_mps;
    }
    return total / history.history.size();
}

std::string TrajektoriApp::stateToString(AgentState state)
{
    return (state == AgentState::STATIC) ? "STATIC" : "DYNAMIC";
}


void TrajektoriApp::logTrajectory()
{
    Enter_Method("logTrajectory");

    auto& vdp = getFacilities().get_const<VehicleDataProvider>();
    long myId = vdp.station_id();
    double myLat = vdp.latitude().value();
    double myLon = vdp.longitude().value();
    double mySpeed = vdp.speed().value();

    mSelfHistory.history.push_back({simTime(), myLat, myLon, mySpeed});
    while (!mSelfHistory.history.empty() && (simTime() - mSelfHistory.history.front().timestamp > 5.0)) {
        mSelfHistory.history.pop_front();
    }
	AgentState myStatus = classifyState(mSelfHistory);
    double avgMySpeed = computeAverageSpeed(mSelfHistory);
    
    if (mLogFile.is_open()) {
        for (const auto& pair : mOtherNodes) {
            long targetId = pair.first;
            const AgentHistory& targetHist = pair.second;

            if (!targetHist.history.empty()) {
                const auto& last = targetHist.history.back();
                AgentState targetStatus = classifyState(targetHist);
                double avgTargetSpeed = computeAverageSpeed(targetHist);

                std::string commStatus = "IN_RANGE";
                if (simTime() - targetHist.lastReceptionTime > 1.0) {
                    commStatus = "OUT_OF_RANGE";
                }

                mLogFile << simTime().dbl() << ";"
                         << myId << ";" << myLat << ";" << myLon << ";" << avgMySpeed << ";" << stateToString(myStatus) << ";"
                         << targetId << ";" << last.latitude << ";" << last.longitude << ";" << avgTargetSpeed << ";" << stateToString(targetStatus) << ";"
                         << commStatus << std::endl;
            }
        }
        mLogFile.flush();
    }
}

void TrajektoriApp::predictTrajectory()
{
    auto& vdp = getFacilities().get_const<VehicleDataProvider>();
    long myId = vdp.station_id();
    double myLat = vdp.latitude().value();
    double myLon = vdp.longitude().value();
    double mySpeed = vdp.speed().value();
    double avgMySpeed = computeAverageSpeed(mSelfHistory);

    for (auto& pair : mOtherNodes) {
        long targetId = pair.first;
        auto& hist = pair.second.history;

        // Ambil hanya data 1 detik terakhir
        std::deque<MovementData> window;
        for (const auto& entry : hist) {
            if (simTime() - entry.timestamp <= 1.0) {
                window.push_back(entry);
            }
        }

        if (window.size() >= 2) {
            auto [predLat, latSlope] = linearRegression(window, true);
			auto [predLon, lonSlope] = linearRegression(window, false);
            double predTime = simTime().dbl() + 1.0;

            double predictedLat = predLat + latSlope * 1.0;
            double predictedLon = predLon + lonSlope * 1.0;

            double targetSpeed = computeAverageSpeed(pair.second);

            mPredictionFile << simTime().dbl() << ";" << myId << ";" << myLat << ";" << myLon << ";" << avgMySpeed << ";"
                            << targetId << ";" << predictedLat << ";" << predictedLon << ";" << targetSpeed << std::endl;
        }
    }
    mPredictionFile.flush();
}

std::pair<double, double> TrajektoriApp::linearRegression(const std::deque<MovementData>& history, bool useLatitude)
{
    double n = history.size();
    if (n < 2) return {0, 0};

    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    double t0 = history.front().timestamp.dbl();

    for (const auto& d : history) {
        double x = d.timestamp.dbl() - t0;
        double y = useLatitude ? d.latitude : d.longitude;
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX + 1e-9);
    double intercept = (sumY - slope * sumX) / n;
    return {intercept, slope};
}

std::string TrajektoriApp::getNodeType()
{
    std::string type = getParentModule()->getNedTypeName();
    if (type.find("Vehicle") != std::string::npos) return "Vehicle";
    if (type.find("Person") != std::string::npos) return "Person";
    return "Unknown";
}

void TrajektoriApp::finish()
{
    if (mLogFile.is_open()) mLogFile.close();
    if (mPredictionFile.is_open()) mPredictionFile.close();
    cancelAndDelete(mLogTimer);
    ItsG5BaseService::finish();
}

} // namespace artery

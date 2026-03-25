#include "artery/application/TrajektoriApp.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>

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

    mLogTimer = new cMessage("logTimer");
    scheduleAt(simTime() + mLogInterval, mLogTimer);

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);

    EV_INFO << "[TrajektoriApp] Initialized and subscribed to CamReceived signal." << std::endl;
}

void TrajektoriApp::handleMessage(cMessage* msg)
{
    if (msg == mLogTimer) {
        logTrajectory();
        scheduleAt(simTime() + mLogInterval, mLogTimer);
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

                std::string commStatus = "OUT_OF_RANGE";
                if (targetHist.lastReceptionTime >= simTime() - mLogInterval - 0.001) {
                    commStatus = "IN_RANGE";
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

void TrajektoriApp::finish()
{
    if (mLogFile.is_open()) {
        mLogFile.close();
    }
    cancelAndDelete(mLogTimer);
    ItsG5BaseService::finish();
}

} // namespace artery

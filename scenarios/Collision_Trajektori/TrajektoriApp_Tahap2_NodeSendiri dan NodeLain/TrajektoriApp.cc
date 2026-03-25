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
    const char* filename = par("logFileName").stringValue();
    mLogFile.open(filename, std::ios::out | std::ios::app);

    if (mLogFile.tellp() == 0) {
        mLogFile << "Time;Observer_ID;Observer_Lat;Observer_Lon;Observer_Speed;"
                 << "Target_ID;Target_Lat;Target_Lon;Target_Speed" << std::endl;
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

        mOtherNodes[stationId].history.push_back(data);

        // Hapus data lama > 5 detik
        auto& hist = mOtherNodes[stationId].history;
        while (!hist.empty() && (simTime() - hist.front().timestamp > 5.0)) {
            hist.pop_front();
        }
    }
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

    if (mLogFile.is_open()) {
        for (const auto& pair : mOtherNodes) {
            long targetId = pair.first;
            const auto& targetHist = pair.second.history;
            if (!targetHist.empty()) {
                const auto& last = targetHist.back();
                mLogFile << simTime().dbl() << ";"
                         << myId << ";" << myLat << ";" << myLon << ";" << mySpeed << ";"
                         << targetId << ";" << last.latitude << ";" << last.longitude << ";" << last.speed_mps << std::endl;
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

#include "artery/application/TrajektoriApp.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include "artery/application/Middleware.h"
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <cmath>
#include <iomanip>


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

void TrajektoriApp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    if (signalID != mCamReceivedSignal) {
        return;
    }
    auto ca_obj = dynamic_cast<CaObject*>(obj);
    if (!ca_obj) {
        return;
    }

    // Mengambil timestamp dari parameter 'details' yang berisi cPacket asli
    const cPacket* packet = dynamic_cast<const cPacket*>(details);
    simtime_t creationTime;

    if (packet) {
        // Ini adalah cara yang benar dan seharusnya berhasil
        creationTime = packet->getCreationTime();
    } else {
        // Fallback jika 'details' bukan packet, untuk mencegah crash
        EV_WARN << "Could not cast signal details to cPacket to get creation time. Using current simTime as fallback." << endl;
        creationTime = simTime();
    }


    const auto& cam = *ca_obj->asn1();
    long stationId = cam.header.stationID;
    const auto& basic = cam.cam.camParameters.basicContainer;
    const auto& hfc = cam.cam.camParameters.highFrequencyContainer;
    if (hfc.present != HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
        return;
    }

    const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;

    MovementData data;
    data.timestamp = creationTime; // Menggunakan timestamp yang sudah didapat
    data.latitude = static_cast<double>(basic.referencePosition.latitude);
    data.longitude = static_cast<double>(basic.referencePosition.longitude);
    data.speed_mps = static_cast<double>(bvc.speed.speedValue);
    
    AgentHistory& history = mOtherNodes[stationId];
    history.history.push_back(data);
    history.lastReceptionTime = simTime();
    history.hasNewData = true;

    while (!history.history.empty() && (simTime() - history.history.front().timestamp > 5.0)) {
        history.history.pop_front();
    }
}

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


void TrajektoriApp::finish()
{
    if (mLogFile.is_open()) mLogFile.close();
    cancelAndDelete(mLogTimer);
    ItsG5BaseService::finish();
}
} // namespace artery 

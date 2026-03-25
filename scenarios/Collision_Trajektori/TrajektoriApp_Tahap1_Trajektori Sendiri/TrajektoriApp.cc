#include "artery/application/TrajektoriApp.h"
#include "artery/application/VehicleDataProvider.h"
#include <omnetpp.h>

namespace artery {
using namespace omnetpp;

Define_Module(TrajektoriApp);

void TrajektoriApp::initialize()
{
    ItsG5BaseService::initialize();

    // Ambil parameter dari NED
    mLogInterval = par("logInterval");

    // Buka file log
    const char* filename = par("logFileName").stringValue();
    mLogFile.open(filename, std::ios::out | std::ios::app);

    // Tulis header CSV jika file baru
    if (mLogFile.tellp() == 0) {
        mLogFile << "Time(s);StationID;Latitude;Longitude;Speed(m/s)" << std::endl;
    }

    // Jadwalkan timer logging
    mLogTimer = new omnetpp::cMessage("logTimer");
    scheduleAt(omnetpp::simTime() + mLogInterval, mLogTimer);

    EV_INFO << "[TrajektoriApp] Self logging initialized." << std::endl;
}

void TrajektoriApp::handleMessage(omnetpp::cMessage* msg)
{
    if (msg == mLogTimer) {
        logSelfTrajectory();
        scheduleAt(omnetpp::simTime() + mLogInterval, mLogTimer);
    } else {
        delete msg;
    }
}

void TrajektoriApp::logSelfTrajectory()
{
    Enter_Method("logSelfTrajectory");

    auto& vdp = getFacilities().get_const<VehicleDataProvider>();
    long stationId = vdp.station_id();
    double lat = vdp.latitude().value();      // WGS84 latitude in degrees
    double lon = vdp.longitude().value();     // WGS84 longitude in degrees
    double speed = vdp.speed().value();       // m/s
    
    if (mLogFile.is_open()) {
		mLogFile << omnetpp::simTime().dbl() << ";"
                 << stationId << ";"
                 << lat << ";"
                 << lon << ";"
                 << speed << std::endl;
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

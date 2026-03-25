#include "artery/application/TrajektoriApp.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include "artery/application/Middleware.h"
#include <boost/units/systems/si/prefixes.hpp>
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <cmath>
#include <iomanip>


namespace artery {
using namespace omnetpp;

Define_Module(TrajektoriApp);

namespace {

// FUNGSI HELPER UNTUK PEMBULATAN
template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
    boost::units::quantity<U> v { q };
    return std::round(v.value());
}

// FUNGSI HELPER UNTUK KONVERSI KECEPATAN
SpeedValue_t buildSpeedValue(const vanetza::units::Velocity& v)
{
    static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
    static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };
    auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

    SpeedValue_t speed = SpeedValue_unavailable;
    if (v >= upper) {
        speed = 16382;
    } else if (v >= lower) {
        speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
    }
    return speed;
}

} // akhir dari anonymous namespace

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
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();

    std::string nodeType = getNodeType();
    std::string logFilename;
    if (nodeType == "Vehicle") {
        logFilename = "results/sender_log_from_car.csv";
    } else if (nodeType == "Person") {
        logFilename = "results/sender_log_from_person.csv";
    } else {
        logFilename = "results/sender_log_unknown.csv";
    }

    mLogFile.open(logFilename, std::ios::out | std::ios::app);

    // Header diubah untuk mencerminkan data mentah
    if (mLogFile.tellp() == 0) {
        mLogFile << "Time_s;Node_ID;Lat_microdeg;Lon_microdeg;Speed_cmps" << std::endl;
    }

    mCamSentSignal = registerSignal("CamSent");
    getParentModule()->subscribe(mCamSentSignal, this);
}

void TrajektoriApp::receiveSignal(cComponent*, simsignal_t signalID, cObject*, cObject*)
{
    if (signalID == mCamSentSignal) {
        simtime_t now = simTime();
        long myId = mVehicleDataProvider->station_id();

        // Meniru proses konversi untuk mendapatkan nilai mentah
        auto microdegree = vanetza::units::degree * boost::units::si::micro;
        long latitude_raw = round(mVehicleDataProvider->latitude(), microdegree);
        long longitude_raw = round(mVehicleDataProvider->longitude(), microdegree);
        long speed_raw = buildSpeedValue(mVehicleDataProvider->speed());

        // Menulis data mentah ke file log
        mLogFile << std::fixed << std::setprecision(12) << now.dbl() << ";"
                 << myId << ";"
                 << latitude_raw << ";"
                 << longitude_raw << ";"
                 << speed_raw << std::endl;
    }
}

void TrajektoriApp::finish()
{
    if (mLogFile.is_open()) {
        mLogFile.close();
    }
    ItsG5BaseService::finish();
}

} // namespace artery


#include "artery/application/CamLogger.h"
#include "artery/application/Middleware.h"
#include "artery/application/VehicleDataProvider.h" 
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <string>
#include <cmath> 

using namespace omnetpp;

namespace artery
{

std::string stationTypeToString(long type_code)
{
    switch (type_code) {
        case 1: return "Pedestrian"; 
        case 5: return "Car";
        default: return "Unknown";
    }
}
std::string simplifyNedTypeName(const std::string& nedTypeName)
{
    if (nedTypeName.find("Car") != std::string::npos || nedTypeName.find("Vehicle") != std::string::npos) {
        return "Car";
    } else if (nedTypeName.find("Person") != std::string::npos) {
        return "Person";
    }
    return "Unknown";
}

std::string extractNodeName(const std::string& fullPath)
{
    size_t first_dot = fullPath.find('.');
    if (first_dot == std::string::npos) return fullPath;

    size_t second_dot = fullPath.find('.', first_dot + 1);
    if (second_dot == std::string::npos) return fullPath;

    return fullPath.substr(first_dot + 1, second_dot - (first_dot + 1));
}

Define_Module(CamLogger);

void CamLogger::initialize()
{
    const char* logFileName = par("logFileName").stringValue();
    mLogFile.open(logFileName, std::ios_base::app);
    if (mLogFile.tellp() == 0) {
        mLogFile << "Time(s);Sender_Node;Sender_Type;Sender_Latitude;Sender_Longitude;Sender_Speed(m/s);Sender_Direction(deg);"
                 <<"Receiver_Node;Receiver_Type;Receiver_Latitude;Receiver_Longitude;Receiver_Speed(m/s);Receiver_Direction(deg)" << std::endl;
    }

    mCamSentSignal = registerSignal("CamSent");
    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamSentSignal, this);
    getParentModule()->subscribe(mCamReceivedSignal, this);
    EV_INFO << "CamLogger diinisialisasi dan berlangganan sinyal 'CamSent' dan 'CamReceived'." << std::endl;
}

void CamLogger::finish()
{
    if (mLogFile.is_open()) {
        mLogFile.close();
    }
}

void CamLogger::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    Enter_Method("receiveSignal");

	if (signalID == mCamSentSignal) {
		if (auto ca_obj = dynamic_cast<CaObject*>(obj)) {
            const vanetza::asn1::Cam& cam_wrapper = ca_obj->asn1();
            const CAM_t& cam_struct = *cam_wrapper;
            long sender_station_id = cam_struct.header.stationID;
            cModule* sender_node = source->getParentModule()->getParentModule();
            mStationIdToNode[sender_station_id] = sender_node;
        }
    }
    else if (signalID == mCamReceivedSignal) {
        if (auto ca_obj = dynamic_cast<CaObject*>(obj)) {
            const vanetza::asn1::Cam& cam_wrapper = ca_obj->asn1();
            const CAM_t& cam_struct = *cam_wrapper;
            long sender_station_id = cam_struct.header.stationID;

            cModule* receiver_node = source->getParentModule()->getParentModule();
            std::string receiver_name = extractNodeName(receiver_node->getFullPath());
            std::string receiver_type = simplifyNedTypeName(receiver_node->getNedTypeName());

            std::string sender_name = "Unknown";
            std::string sender_type = "Unknown";
            auto it = mStationIdToNode.find(sender_station_id);
            if (it != mStationIdToNode.end()) {
                cModule* sender_node_from_map = it->second;
                sender_name = extractNodeName(sender_node_from_map->getFullPath());
                sender_type = simplifyNedTypeName(sender_node_from_map->getNedTypeName());
            }
			
			auto middleware = check_and_cast<Middleware*>(receiver_node->getSubmodule("middleware"));
            auto& vdp_receiver = middleware->getFacilities().get_const<VehicleDataProvider>();
            const auto& hfc = cam_struct.cam.camParameters.highFrequencyContainer;
            const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;
            const auto& basic = cam_struct.cam.camParameters.basicContainer;
			double receiver_speed_ms = vdp_receiver.speed().value();
            double receiver_heading_rad = vdp_receiver.heading().value();
			double receiver_heading_deg = receiver_heading_rad * 180.0 / M_PI;
            if (mLogFile.is_open()) {
                mLogFile << omnetpp::simTime().dbl() << ";"
                         << sender_name << ";"
                         << sender_type << ";"
                         << static_cast<double>(basic.referencePosition.latitude) * 1e-7 << ";"
                         << static_cast<double>(basic.referencePosition.longitude) * 1e-7 << ";"
                         << static_cast<double>(bvc.speed.speedValue) * 0.01 << ";"
                         << static_cast<double>(bvc.heading.headingValue) * 0.1 << ";"
                         << receiver_name << ";"
                         << receiver_type << ";"
                         << vdp_receiver.latitude().value() << ";"
                         << vdp_receiver.longitude().value() << ";"
                         << receiver_speed_ms << ";"
                         << receiver_heading_deg << std::endl;
                         
            }
        }
    }
}


void CamLogger::receiveSignal(cComponent*, simsignal_t, bool, cObject*) {}
void CamLogger::receiveSignal(cComponent*, simsignal_t, long, cObject*) {}
void CamLogger::receiveSignal(cComponent*, simsignal_t, unsigned long, cObject*) {}
void CamLogger::receiveSignal(cComponent*, simsignal_t, double, cObject*) {}
void CamLogger::receiveSignal(cComponent*, simsignal_t, const SimTime&, cObject*) {}
void CamLogger::receiveSignal(cComponent*, simsignal_t, const char*, cObject*) {}


}

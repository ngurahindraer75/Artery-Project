#include "artery/application/CamLogger.h"
#include "artery/application/Middleware.h"
#include "artery/application/VehicleDataProvider.h"
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <string>
#include <cmath>

using namespace omnetpp;

namespace artery {

/**
 * @brief Translates ETSI ITS-G5 Station Type codes into standardized string labels.
 * @param type_code Numeric code from the ASN.1 CAM payload.
 * @return Standardized entity string ("Vehicle" or "Pedestrian").
 */
std::string stationTypeToString(long type_code) {
    switch (type_code) {
        case 1: return "Pedestrian";
        case 5: return "Vehicle"; // Standardized from "Car" to "Vehicle" [1]
        default: return "Unknown";
    }
}

/**
 * @brief Simplifies raw OMNeT++ NED type directories into standardized string labels.
 * @param nedTypeName Raw OMNeT++ module path (e.g., "artery.inet.Car").
 * @return Standardized entity string ("Vehicle" or "Pedestrian").
 */
std::string simplifyNedTypeName(const std::string& nedTypeName) {
    if (nedTypeName.find("Car") != std::string::npos || nedTypeName.find("Vehicle") != std::string::npos) {
        return "Vehicle"; // Standardized from "Car" to "Vehicle" [1]
    } else if (nedTypeName.find("Person") != std::string::npos) {
        return "Pedestrian"; // Standardized from "Person" to "Pedestrian" [1]
    }
    return "Unknown";
}

/**
 * @brief Extracts the short node name from a full OMNeT++ hierarchical path.
 */
std::string extractNodeName(const std::string& fullPath) {
    size_t first_dot = fullPath.find('.');
    if (first_dot == std::string::npos) return fullPath;

    size_t second_dot = fullPath.find('.', first_dot + 1);
    if (second_dot == std::string::npos) return fullPath;

    return fullPath.substr(first_dot + 1, second_dot - (first_dot + 1));
}

Define_Module(CamLogger);

// Allocate memory for the global static registry before simulation starts
std::map<long, CamLogger::NodeIdentity> CamLogger::mStationRegistry;

void CamLogger::initialize() {
    // Get the real name of this specific node (e.g., "node", "node[2]")
    cModule* myNode = getParentModule()->getParentModule();
    std::string nodeName = extractNodeName(myNode->getFullPath());

    // PREVENT RACE CONDITION: Each node creates its own unique CSV file 
    // to avoid I/O bottlenecks when multiple vehicles operate simultaneously.
    std::string fileName = "results/cam_log_" + nodeName + ".csv";

    // Open the file with output and append modes
    mLogFile.open(fileName, std::ios_base::out | std::ios_base::app);

    // Write CSV headers if the file is newly created
    if (mLogFile.tellp() == 0) {
        mLogFile << "Time(s);Sender_Node;Sender_Type;Sender_Latitude;Sender_Longitude;Sender_Speed(m/s);Sender_Direction(deg);"
                 << "Receiver_Node;Receiver_Type;Receiver_Latitude;Receiver_Longitude;Receiver_Speed(m/s);Receiver_Direction(deg)\n";
    }

    // Register and subscribe to CAM transmission and reception signals
    mCamSentSignal = registerSignal("CamSent");
    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamSentSignal, this);
    getParentModule()->subscribe(mCamReceivedSignal, this);

    EV_INFO << "CamLogger initialized for " << nodeName << " and subscribed to 'CamSent' and 'CamReceived'." << std::endl;
}

void CamLogger::finish() {
    // Gracefully close the file stream at the end of the simulation
    if (mLogFile.is_open()) {
        mLogFile.close();
    }
}

void CamLogger::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method("receiveSignal");

    if (signalID == mCamSentSignal) {
        if (auto ca_obj = dynamic_cast<CaObject*>(obj)) {
            const vanetza::asn1::Cam& cam_wrapper = ca_obj->asn1();
            const CAM_t& cam_struct = *cam_wrapper;
            
            long sender_station_id = cam_struct.header.stationID;
            cModule* my_node = source->getParentModule()->getParentModule();

            // LOGGING UTILITY: Register this node's identity into the global dictionary 
            // when it transmits its first CAM to the air.
            mStationRegistry[sender_station_id] = {
                extractNodeName(my_node->getFullPath()),
                simplifyNedTypeName(my_node->getNedTypeName())
            };
        }
    }
    else if (signalID == mCamReceivedSignal) {
        if (auto ca_obj = dynamic_cast<CaObject*>(obj)) {
            const vanetza::asn1::Cam& cam_wrapper = ca_obj->asn1();
            const CAM_t& cam_struct = *cam_wrapper;
            
            // Extract Sender ID from the ASN.1 ETSI payload
            long sender_station_id = cam_struct.header.stationID;

            // Extract Receiver's Physical Data
            cModule* receiver_node = source->getParentModule()->getParentModule();
            std::string receiver_name = extractNodeName(receiver_node->getFullPath());
            std::string receiver_type = simplifyNedTypeName(receiver_node->getNedTypeName());

            std::string sender_name = "Unknown";
            std::string sender_type = "Unknown";

            // LOOKUP LOGGING IDENTITY: Ask the global registry for the string name of the sender ID
            auto it = mStationRegistry.find(sender_station_id);
            if (it != mStationRegistry.end()) {
                sender_name = it->second.node_name;
                sender_type = it->second.node_type;
            }

            // Obtain receiver's current physical state via Middleware -> VehicleDataProvider
            auto middleware = check_and_cast<Middleware*>(receiver_node->getSubmodule("middleware"));
            auto& vdp_receiver = middleware->getFacilities().get_const<VehicleDataProvider>();
            
            const auto& hfc = cam_struct.cam.camParameters.highFrequencyContainer;
            const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;
            const auto& basic = cam_struct.cam.camParameters.basicContainer;
            
            double receiver_speed_ms = vdp_receiver.speed().value();
            double receiver_heading_rad = vdp_receiver.heading().value();
            double receiver_heading_deg = receiver_heading_rad * 180.0 / M_PI;

            if (mLogFile.is_open()) {
                // Print real-world denormalized physical data
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
                         << receiver_heading_deg << "\n";
                
                // PREVENT DATA LOSS: Flush buffer to disk immediately (Safe against SUMO crash)
                mLogFile.flush(); 
            }
        }
    }
}

} // namespace artery
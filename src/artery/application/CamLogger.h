#ifndef ARTERY_CAMLOGGER_H_
#define ARTERY_CAMLOGGER_H_

#include "artery/application/CaObject.h"
#include <omnetpp.h>
#include <fstream>
#include <map>
#include <string>

namespace artery {

/**
 * @class CamLogger
 * @brief An eavesdropping observer module designed to log V2X Cooperative Awareness Messages (CAM).
 * 
 * This module listens to ETSI ITS-G5 CAM transmissions and receptions in the air. 
 * It extracts physical and geographical states of both the Sender and the Receiver 
 * at the exact simulation time to provide Ground Truth data for Collision Warning evaluations.
 */
class CamLogger : public omnetpp::cSimpleModule, public omnetpp::cIListener {
protected:
    virtual void initialize() override;
    virtual void finish() override;
    
    // Main signal receiver for CAM transmissions and receptions
    virtual void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;
    
    // Empty overrides for unused OMNeT++ signal types (Best practice to keep them inline)
    virtual void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, bool, omnetpp::cObject*) override {}
    virtual void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, long, omnetpp::cObject*) override {}
    virtual void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, unsigned long, omnetpp::cObject*) override {}
    virtual void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, double, omnetpp::cObject*) override {}
    virtual void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, const omnetpp::SimTime&, omnetpp::cObject*) override {}
    virtual void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, const char*, omnetpp::cObject*) override {}

private:
    std::ofstream mLogFile;
    omnetpp::simsignal_t mCamSentSignal;
    omnetpp::simsignal_t mCamReceivedSignal;

    /**
     * @struct NodeIdentity
     * @brief Structure to store the string identities of Nodes for CSV LOGGING purposes.
     */
    struct NodeIdentity {
        std::string node_name;
        std::string node_type;
    };
    
    /**
     * @brief Global static registry to act as an OMNeT++ logging utility (V2N Abstraction).
     * 
     * This shared memory allows the receiving node's CamLogger to instantly resolve 
     * the real OMNeT++ string name of the sender using only its ETSI Station ID.
     */
    static std::map<long, NodeIdentity> mStationRegistry;
};

} // namespace artery
#endif
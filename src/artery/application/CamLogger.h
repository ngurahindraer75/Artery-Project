#ifndef ARTERY_CAMLOGGER_H_
#define ARTERY_CAMLOGGER_H_

#include "artery/application/CaObject.h"
#include <omnetpp.h>
#include <fstream>
#include <map>

namespace artery
{

class CamLogger : public omnetpp::cSimpleModule, public omnetpp::cIListener
{
protected:
    void initialize() override;
    void finish() override;
    void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;
    
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, bool, omnetpp::cObject*) override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, long, omnetpp::cObject*) override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, unsigned long, omnetpp::cObject*) override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, double, omnetpp::cObject*) override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, const omnetpp::SimTime&, omnetpp::cObject*) override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, const char*, omnetpp::cObject*) override;

private:
    std::ofstream mLogFile;
    omnetpp::simsignal_t mCamSentSignal;
    omnetpp::simsignal_t mCamReceivedSignal;
    std::map<long, omnetpp::cModule*> mStationIdToNode;
};

} 
#endif 

#ifndef ARTERY_TRAJEKTORIAPP_H_
#define ARTERY_TRAJEKTORIAPP_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/CaService.h"
#include <fstream>
#include <map>
#include <deque>
#include <omnetpp.h>

namespace artery {

struct MovementData {
    omnetpp::simtime_t timestamp;
    double latitude;
    double longitude;
    double speed_mps;
};

struct AgentHistory {
    std::deque<MovementData> history;
    omnetpp::simtime_t lastReceptionTime = -1;
};

class TrajektoriApp : public ItsG5BaseService
{
  protected:
    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage* msg) override;
    virtual void finish() override;
    virtual void receiveSignal(omnetpp::cComponent* source, omnetpp::simsignal_t signalID, omnetpp::cObject* obj, omnetpp::cObject* details) override;

  private:
    void logTrajectory();
    double mLogInterval;
    omnetpp::cMessage* mLogTimer = nullptr;
    std::ofstream mLogFile;

    AgentHistory mSelfHistory;
    std::map<long, AgentHistory> mOtherNodes;

    omnetpp::simsignal_t mCamReceivedSignal;
};

} // namespace artery

#endif

#ifndef ARTERY_TRAJEKTORIAPP_H_
#define ARTERY_TRAJEKTORIAPP_H_

#include "artery/application/ItsG5BaseService.h"
#include <fstream>
#include <omnetpp.h>

namespace artery {

class TrajektoriApp : public ItsG5BaseService
{
  protected:
    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage* msg) override;
    virtual void finish() override;

  private:
    void logSelfTrajectory();

    std::ofstream mLogFile;
    omnetpp::cMessage* mLogTimer = nullptr;
    omnetpp::simtime_t mLogInterval;
};

} // namespace artery

#endif

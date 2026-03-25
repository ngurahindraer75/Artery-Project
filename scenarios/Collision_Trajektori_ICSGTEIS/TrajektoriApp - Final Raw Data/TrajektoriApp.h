#ifndef ARTERY_TRAJEKTORIAPP_H_
#define ARTERY_TRAJEKTORIAPP_H_

#include "artery/application/ItsG5BaseService.h"
#include <omnetpp/simtime.h>
#include <vanetza/units/velocity.hpp>
#include <deque>
#include <fstream>
#include <string>
#include <map>

namespace artery
{

// Forward declaration untuk menghindari include header penuh, ini adalah praktik yang baik
class VehicleDataProvider;

class TrajektoriApp : public ItsG5BaseService
{
    public:
        void initialize() override;
        void finish() override;

    protected:
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;

    private:
        // Deklarasi fungsi privat untuk mendapatkan tipe node
        std::string getNodeType();

        // Deklarasi variabel anggota yang digunakan di file .cc
        const VehicleDataProvider* mVehicleDataProvider = nullptr;
        omnetpp::simsignal_t mCamSentSignal;
        std::ofstream mLogFile;
};

} // namespace artery

#endif /* ARTERY_SENDERLOGAPP_H_ */

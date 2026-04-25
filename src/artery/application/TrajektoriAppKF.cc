#include "artery/application/TrajektoriAppKF.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/CaService.h"
#include "artery/application/Middleware.h"
#include <vanetza/asn1/its/CAM.h>
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <omnetpp/cpacket.h>
#include <cmath>
#include <iomanip>

namespace artery {

using namespace omnetpp;

Define_Module(TrajektoriAppKF);

static std::map<long, std::string> g_stationIdToNodeType_KF;

std::string TrajektoriAppKF::getNodeType()
{
    std::string type = getParentModule()->getNedTypeName();
    if (type.find("Vehicle") != std::string::npos || type.find("Car") != std::string::npos) return "Vehicle";
    if (type.find("Person") != std::string::npos) return "Person";
    return "Unknown";
}

void TrajektoriAppKF::initialize()
{
    ItsG5BaseService::initialize();

    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();

    mLogFile_1s_car.open("results/KF_1s_car_prediction.csv", std::ios::out | std::ios::app);
    mLogFile_2s_car.open("results/KF_2s_car_prediction.csv", std::ios::out | std::ios::app);
    mLogFile_3s_car.open("results/KF_3s_car_prediction.csv", std::ios::out | std::ios::app);
    mLogFile_1s_person.open("results/KF_1s_person_prediction.csv", std::ios::out | std::ios::app);
    mLogFile_2s_person.open("results/KF_2s_person_prediction.csv", std::ios::out | std::ios::app);
    mLogFile_3s_person.open("results/KF_3s_person_prediction.csv", std::ios::out | std::ios::app);

    std::string header = "Processing_Time (s);Latest_CAM_Time (s);Time_Absolut_Prediction (s);Observer_ID;Target_ID;Target_Real_Lat (microdegree);KF_State_Y (meter);KF_State_Vy (mps);Pred_Lat (microdegree);Target_Real_Lon (microdegree);KF_State_X (meter);KF_State_Vx (mps);Pred_Lon (microdegree)";

    if (mLogFile_1s_car.tellp() == 0) mLogFile_1s_car << header << std::endl;
    if (mLogFile_2s_car.tellp() == 0) mLogFile_2s_car << header << std::endl;
    if (mLogFile_3s_car.tellp() == 0) mLogFile_3s_car << header << std::endl;
    if (mLogFile_1s_person.tellp() == 0) mLogFile_1s_person << header << std::endl;
    if (mLogFile_2s_person.tellp() == 0) mLogFile_2s_person << header << std::endl;
    if (mLogFile_3s_person.tellp() == 0) mLogFile_3s_person << header << std::endl;

    mCamReceivedSignal = registerSignal("CamReceived");
    getParentModule()->subscribe(mCamReceivedSignal, this);

    mCamSentSignal = registerSignal("CamSent");
    getParentModule()->subscribe(mCamSentSignal, this);

    mPredictionTimer = new cMessage("predictionTimerKF");
    scheduleAt(simTime() + 1.0, mPredictionTimer);
}

void TrajektoriAppKF::handleMessage(cMessage* msg)
{
    if (msg == mPredictionTimer) {
        runKalmanPrediction();
        scheduleAt(simTime() + 1.0, mPredictionTimer);
        return;
    }
    delete msg;
}

void TrajektoriAppKF::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    if (signalID == mCamSentSignal) {
        auto ca_obj = dynamic_cast<CaObject*>(obj);
        if (ca_obj) {
            long myStationId = ca_obj->asn1()->header.stationID;
            g_stationIdToNodeType_KF[myStationId] = getNodeType();
        }
        return; 
    }

    if (signalID != mCamReceivedSignal) return;

    auto ca_obj = dynamic_cast<CaObject*>(obj);
    if (!ca_obj) return;

    const auto& cam = *ca_obj->asn1();
    long targetId = cam.header.stationID;

    long genDeltaTime_ms = cam.cam.generationDeltaTime;
    simtime_t time_receive = simTime();
    long long current_time_ms = time_receive.inUnit(SIMTIME_MS);

    static long long tai_offset_mod = -1;
    if (tai_offset_mod == -1) {
        const cPacket* packet = dynamic_cast<const cPacket*>(details);
        long long true_creation_ms = (packet ? packet->getCreationTime() : simTime()).inUnit(SIMTIME_MS);
        tai_offset_mod = (genDeltaTime_ms - true_creation_ms) % 65536;
        if (tai_offset_mod < 0) tai_offset_mod += 65536;
    }

    long current_mod = (current_time_ms + tai_offset_mod) % 65536;
    long delay_ms = current_mod - genDeltaTime_ms;
    if (delay_ms < 0) delay_ms += 65536;

    double absolute_send_time_s = (current_time_ms - delay_ms) / 1000.0;

    const auto& basic = cam.cam.camParameters.basicContainer;
    const auto& hfc = cam.cam.camParameters.highFrequencyContainer;
    if (hfc.present != HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) return;
    const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;

    long lat_raw = basic.referencePosition.latitude / 10;
    long lon_raw = basic.referencePosition.longitude / 10;
    long speed_raw = bvc.speed.speedValue;
    long heading_raw = bvc.heading.headingValue;

    double R_earth = 6371000.0;
    double lat_rad = (lat_raw / 1000000.0) * M_PI / 180.0;
    double lon_rad = (lon_raw / 1000000.0) * M_PI / 180.0;

    double target_lat_m = R_earth * lat_rad; 
    double target_lon_m = R_earth * lon_rad * std::cos(lat_rad); 

    double target_speed_mps = speed_raw / 100.0;     
    double target_heading_deg = heading_raw / 10.0;
    double heading_rad = target_heading_deg * M_PI / 180.0;

    double v_x = target_speed_mps * std::sin(heading_rad);
    double v_y = target_speed_mps * std::cos(heading_rad);

    // =========================================================================
    // EKSTRAKSI AKSELERASI SEBAGAI VEKTOR KONTROL (B_k * u_k)
    // =========================================================================
    double target_accel_mps2 = 0.0;
    long accel_raw = bvc.longitudinalAcceleration.longitudinalAccelerationValue;
    
    // Nilai 161 (161.0) dalam standar ETSI adalah "Unavailable" (Tidak Tersedia)
    if (accel_raw != 161) {
        target_accel_mps2 = accel_raw / 10.0; // Ubah resolusi 0.1 m/s^2 menjadi m/s^2 murni
    }

    // Pecah akselerasi menjadi komponen vektor X dan Y
    double a_x = target_accel_mps2 * std::sin(heading_rad);
    double a_y = target_accel_mps2 * std::cos(heading_rad);

    AgentHistoryKF& hist = mOtherNodes[targetId];

    if (!hist.is_initialized) {
        hist.kf_X.init(target_lon_m, v_x);
        hist.kf_Y.init(target_lat_m, v_y);
        hist.is_initialized = true;
        hist.last_update_time = absolute_send_time_s;
    } else {
        double dt = absolute_send_time_s - hist.last_update_time;
        if (dt > 0.0) {
            // SUNTIKKAN AKSELERASI 'a_x' DAN 'a_y' KE DALAM FUNGSI PREDIKSI
            hist.kf_X.predict(dt, a_x);
            hist.kf_Y.predict(dt, a_y);

            hist.kf_X.update(target_lon_m, v_x);
            hist.kf_Y.update(target_lat_m, v_y);

            hist.last_update_time = absolute_send_time_s;
        }
    }

    hist.ground_truth_history.push_back({absolute_send_time_s, time_receive.dbl(), lat_raw, lon_raw});

    while (!hist.ground_truth_history.empty() && (simTime().dbl() - hist.ground_truth_history.front().time_receive > 5.0)) {
        hist.ground_truth_history.pop_front();
    }
}

void TrajektoriAppKF::runKalmanPrediction()
{
    double t_sim = simTime().dbl();
    long observerId = mVehicleDataProvider ? mVehicleDataProvider->station_id() : -1;
    double R_earth = 6371000.0;

    for (auto& pair : mOtherNodes) {
        long targetId = pair.first;
        auto& hist_struct = pair.second;

        if (!hist_struct.is_initialized || hist_struct.ground_truth_history.empty()) continue;

        std::ofstream* out_1s = &mLogFile_1s_car;
        std::ofstream* out_2s = &mLogFile_2s_car;
        std::ofstream* out_3s = &mLogFile_3s_car;

        auto it_type = g_stationIdToNodeType_KF.find(targetId);
        if (it_type != g_stationIdToNodeType_KF.end() && it_type->second == "Person") {
            out_1s = &mLogFile_1s_person;
            out_2s = &mLogFile_2s_person;
            out_3s = &mLogFile_3s_person;
        }

        auto evaluate_queue = [&](std::vector<PendingPredictionKF>& pending_queue, double target_offset, std::ofstream& outFile) {
            for (auto it = pending_queue.begin(); it != pending_queue.end(); ) {
                double target_time = it->processing_time + target_offset;

                if (hist_struct.last_update_time >= target_time - 0.2) {
                    MovementDataKF* gt_point = nullptr;
                    double min_diff = 9999.0;

                    for (auto& pt : hist_struct.ground_truth_history) {
                        double diff = std::abs(pt.time_send_absolut - target_time);
                        if (diff < min_diff) {
                            min_diff = diff;
                            gt_point = &pt;
                        }
                    }

                    if (gt_point) {
                        double target_prediction_time = gt_point->time_send_absolut;

                        double pred_x_m = it->kf_x_pos + it->kf_x_vel * target_offset;
                        double pred_y_m = it->kf_y_pos + it->kf_y_vel * target_offset;

                        double pred_lat_rad = pred_y_m / R_earth;
                        double pred_lon_rad = pred_x_m / (R_earth * std::cos(pred_lat_rad));

                        long pred_lat_raw = static_cast<long>(std::round(pred_lat_rad * 180.0 / M_PI * 1000000.0));
                        long pred_lon_raw = static_cast<long>(std::round(pred_lon_rad * 180.0 / M_PI * 1000000.0));

                        if (outFile.is_open()) {
                            outFile << std::fixed << std::setprecision(12)
                                    << it->processing_time << ";"
                                    << it->latest_cam_time << ";"
                                    << target_prediction_time << ";"
                                    << observerId << ";"
                                    << targetId << ";"
                                    << gt_point->lat_raw << ";"
                                    << it->kf_y_pos << ";" << it->kf_y_vel << ";" << pred_lat_raw << ";"
                                    << gt_point->lon_raw << ";"
                                    << it->kf_x_pos << ";" << it->kf_x_vel << ";" << pred_lon_raw << std::endl;
                        }
                    }
                    it = pending_queue.erase(it);
                } else {
                    ++it;
                }
            }
        };

        evaluate_queue(hist_struct.pending_1s, 0.9, *out_1s);
        evaluate_queue(hist_struct.pending_2s, 1.9, *out_2s);
        evaluate_queue(hist_struct.pending_3s, 2.9, *out_3s);

        PendingPredictionKF new_task;
        new_task.processing_time = t_sim;
        new_task.latest_cam_time = hist_struct.last_update_time;
        new_task.kf_x_pos = hist_struct.kf_X.x[0];
        new_task.kf_x_vel = hist_struct.kf_X.x[1];
        new_task.kf_y_pos = hist_struct.kf_Y.x[0];
        new_task.kf_y_vel = hist_struct.kf_Y.x[1];

        hist_struct.pending_1s.push_back(new_task);
        hist_struct.pending_2s.push_back(new_task);
        hist_struct.pending_3s.push_back(new_task);
    }

    if (mLogFile_1s_car.is_open()) mLogFile_1s_car.flush();
    if (mLogFile_2s_car.is_open()) mLogFile_2s_car.flush();
    if (mLogFile_3s_car.is_open()) mLogFile_3s_car.flush();
    if (mLogFile_1s_person.is_open()) mLogFile_1s_person.flush();
    if (mLogFile_2s_person.is_open()) mLogFile_2s_person.flush();
    if (mLogFile_3s_person.is_open()) mLogFile_3s_person.flush();
}

void TrajektoriAppKF::finish()
{
    if (mLogFile_1s_car.is_open()) mLogFile_1s_car.close();
    if (mLogFile_2s_car.is_open()) mLogFile_2s_car.close();
    if (mLogFile_3s_car.is_open()) mLogFile_3s_car.close();
    if (mLogFile_1s_person.is_open()) mLogFile_1s_person.close();
    if (mLogFile_2s_person.is_open()) mLogFile_2s_person.close();
    if (mLogFile_3s_person.is_open()) mLogFile_3s_person.close();

    cancelAndDelete(mPredictionTimer);
    ItsG5BaseService::finish();
}

} // namespace arter

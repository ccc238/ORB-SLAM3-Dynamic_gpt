#ifndef ORB_SLAM3_FRONTEND_DIAGNOSTIC_LOG_H
#define ORB_SLAM3_FRONTEND_DIAGNOSTIC_LOG_H
#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <vector>
namespace ORB_SLAM3 {
// Null destination means no clock reads. Timers never drive SLAM decisions.
class DiagnosticTimer {
    double* destination;
    std::chrono::steady_clock::time_point start;
public:
    explicit DiagnosticTimer(double* value) : destination(value) {
        if (destination) start=std::chrono::steady_clock::now();
    }
    void Stop() {
        if (!destination) return;
        *destination=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        destination=nullptr;
    }
    ~DiagnosticTimer() { Stop(); }
};
struct FrontendStageMetrics {
    double coarseMs=-1, b2Ms=-1, b3Ms=-1, b4Ms=-1, fineMs=-1;
    int coarseBefore=-1,coarseRejected=-1,coarseAfter=-1,coarseInliers=-1,coarseAttempts=0;
    bool coarseApplied=false;
    int fineBefore=-1,fineP=-1,fineD=-1,fineBoundary=-1,fineUnknown=-1,fineAfter=-1,fineInliers=-1;
};
using DiagnosticRow=std::map<std::string,std::string>;
class FrontendDiagnosticLog {
    std::string prefix;
    std::ofstream online;
    std::vector<DiagnosticRow> rows;
    bool finalized=false;
public:
    explicit FrontendDiagnosticLog(const std::string& parent);
    void Append(const DiagnosticRow& row);
    // Called only AFTER mapping/loop threads join. Counts belong to the SOURCE
    // KeyFrame frame, not the input frame concurrent with the mapping thread.
    void Finalize(const std::string& mapCsv, bool mapEnabled);
    static const std::vector<std::string>& Columns();
};
}
#endif

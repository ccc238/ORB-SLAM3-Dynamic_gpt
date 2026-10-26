#ifndef ORB_SLAM3_MAP_ADMISSION_H
#define ORB_SLAM3_MAP_ADMISSION_H
#include <cstddef>
#include <string>
namespace ORB_SLAM3 {
class Frame;
class KeyFrame;
class Atlas;
void PrepareFrameMapAdmission(Frame& frame, const std::string& logPath);
// Scoped, aggregated indexed checks; no global MapPoint::AddObservation hook.
class MapAdmissionBatch {
public:
    MapAdmissionBatch(const KeyFrame* keyframe, const char* stage);
    ~MapAdmissionBatch();
    bool Check(size_t index);
    void Created(size_t index);
private:
    const KeyFrame* kf;
    const char* stage;
    int candidates=0, pending=0, dynamic=0, boundary=0, unknown=0, created=0;
};
// Call only after backend worker shutdown. Checks BOTH observation directions.
void SaveMapAdmissionAudit(Atlas& atlas, const std::string& filename);
}
#endif

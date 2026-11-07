#ifndef ORB_SLAM3_SEMANTIC_CLASS_CONFIG_H
#define ORB_SLAM3_SEMANTIC_CLASS_CONFIG_H
#include <algorithm>
#include <string>
#include <vector>

namespace ORB_SLAM3 {
// Selected model classes are potentially movable, NOT observed motion truth.
// Names are display metadata; identity and policy use integer model IDs.
struct SemanticClassConfig {
    bool multiClassEnable = false;
    std::vector<int> classIds = {0};
    std::vector<std::string> classNames;
    bool Contains(int id) const {
        return std::find(classIds.begin(), classIds.end(), id) != classIds.end();
    }
    bool Managed(int id) const {
        return multiClassEnable ? Contains(id) : id == 0;
    }
    std::string Name(int id) const {
        const auto found = std::find(classIds.begin(), classIds.end(), id);
        if (found != classIds.end() && classNames.size() == classIds.size())
            return classNames[static_cast<size_t>(found - classIds.begin())];
        return id == 0 ? "person" : "class_" + std::to_string(id);
    }
};
}
#endif

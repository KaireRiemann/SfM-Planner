#pragma once

#include <mission/mission_types.hpp>
#include <string>

namespace mission {

class MissionTargetStore {
public:
    explicit MissionTargetStore(std::string path);

    const std::string &path() const {
        return path_;
    }

    bool load(MissionTarget &target) const;

    bool saveAtomic(const MissionTarget &target) const;

    bool exists() const;

private:
    std::string path_;
};

}  // namespace mission

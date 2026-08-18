#include <mission/mission_target_store.hpp>

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace mission {
namespace {

Eigen::Vector3d readVec3(const YAML::Node &node, const Eigen::Vector3d &fallback) {
    if (!node || !node.IsSequence() || node.size() < 3) {
        return fallback;
    }
    return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

void writeVec3(YAML::Emitter &out, const Eigen::Vector3d &v) {
    out << YAML::Flow << YAML::BeginSeq << v.x() << v.y() << v.z() << YAML::EndSeq;
}

}  // namespace

MissionTargetStore::MissionTargetStore(std::string path) : path_(std::move(path)) {}

bool MissionTargetStore::exists() const {
    std::error_code ec;
    return std::filesystem::exists(path_, ec);
}

bool MissionTargetStore::load(MissionTarget &target) const {
    try {
        if (!exists()) {
            return false;
        }
        const YAML::Node root = YAML::LoadFile(path_);
        target = MissionTarget{};
        target.scene_id = root["scene_id"] ? root["scene_id"].as<std::string>() : "";
        target.version = root["target_version"] ? root["target_version"].as<uint32_t>() : 0;
        target.map_version = root["map_version"] ? root["map_version"].as<std::string>() : "";
        target.face_center = readVec3(root["face_center"], Eigen::Vector3d::Zero());
        target.face_normal = readVec3(root["face_normal"], -Eigen::Vector3d::UnitX());
        if (target.face_normal.norm() > 1e-6) {
            target.face_normal.normalize();
        }
        target.nav_goal = readVec3(root["nav_goal"], target.face_center);
        target.goal_yaw = root["goal_yaw"] ? root["goal_yaw"].as<double>() : 0.0;
        target.confidence = root["confidence"] ? root["confidence"].as<double>() : 0.0;

        if (root["change_region"]) {
            const YAML::Node cr = root["change_region"];
            target.previous_face_region.center =
                    readVec3(cr["center"], target.face_center);
            target.previous_face_region.normal =
                    readVec3(cr["normal"], target.face_normal);
            if (target.previous_face_region.normal.norm() > 1e-6) {
                target.previous_face_region.normal.normalize();
            }
            target.previous_face_region.width =
                    cr["width"] ? cr["width"].as<double>() : 0.0;
            target.previous_face_region.height =
                    cr["height"] ? cr["height"].as<double>() : 0.0;
            target.previous_face_region.thickness =
                    cr["thickness"] ? cr["thickness"].as<double>() : 1.0;
            target.previous_face_region.valid =
                    target.previous_face_region.width > 0.0 &&
                    target.previous_face_region.height > 0.0 &&
                    target.previous_face_region.thickness > 0.0;
        }
        // Existing target files did not carry this field.  Treat a valid
        // stored change region as a verified legacy face, while a manual
        // anchor can explicitly set face_prior_valid: false.
        target.face_prior_valid = root["face_prior_valid"]
                                          ? root["face_prior_valid"].as<bool>()
                                          : target.previous_face_region.valid;
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[MissionTargetStore] load failed: " << e.what() << std::endl;
        return false;
    }
}

bool MissionTargetStore::saveAtomic(const MissionTarget &target) const {
    try {
        const std::filesystem::path path(path_);
        if (path.has_parent_path()) {
            std::filesystem::create_directories(path.parent_path());
        }
        const std::string tmp_path = path_ + ".tmp";

        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "scene_id" << YAML::Value << target.scene_id;
        out << YAML::Key << "target_version" << YAML::Value << target.version;
        out << YAML::Key << "map_version" << YAML::Value << target.map_version;
        out << YAML::Key << "face_center" << YAML::Value;
        writeVec3(out, target.face_center);
        out << YAML::Key << "face_normal" << YAML::Value;
        writeVec3(out, target.face_normal);
        out << YAML::Key << "face_prior_valid" << YAML::Value
            << target.face_prior_valid;
        out << YAML::Key << "nav_goal" << YAML::Value;
        writeVec3(out, target.nav_goal);
        out << YAML::Key << "goal_yaw" << YAML::Value << target.goal_yaw;
        out << YAML::Key << "confidence" << YAML::Value << target.confidence;

        out << YAML::Key << "change_region" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "center" << YAML::Value;
        writeVec3(out, target.previous_face_region.center);
        out << YAML::Key << "normal" << YAML::Value;
        writeVec3(out, target.previous_face_region.normal);
        out << YAML::Key << "width" << YAML::Value << target.previous_face_region.width;
        out << YAML::Key << "height" << YAML::Value << target.previous_face_region.height;
        out << YAML::Key << "thickness" << YAML::Value
            << target.previous_face_region.thickness;
        out << YAML::EndMap;
        out << YAML::EndMap;

        {
            std::ofstream ofs(tmp_path, std::ios::trunc);
            if (!ofs.is_open()) {
                return false;
            }
            ofs << out.c_str() << '\n';
            ofs.flush();
            if (!ofs.good()) {
                return false;
            }
        }

        // Validate temp file before rename.
        MissionTarget probe;
        MissionTargetStore tmp_store(tmp_path);
        if (!tmp_store.load(probe)) {
            std::remove(tmp_path.c_str());
            return false;
        }

        std::error_code ec;
        std::filesystem::rename(tmp_path, path_, ec);
        if (ec) {
            std::remove(tmp_path.c_str());
            return false;
        }
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[MissionTargetStore] save failed: " << e.what() << std::endl;
        return false;
    }
}

}  // namespace mission

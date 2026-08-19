/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef GENERAL_FSM_CONFIG_HPP
#define GENERAL_FSM_CONFIG_HPP


#include <general_core/config.hpp>
#include <general_core/planning_semantics.hpp>
#include <mission/mission_types.hpp>
#include <algorithm>
#include <cctype>
#include <vector>
#include <cstring>
#include <utils/header/yaml_loader.hpp>

namespace fsm {
    using namespace traj_opt;
    using namespace general_planner;
    static constexpr int MPC_PVAJ_MODE = 1;
    static constexpr int MPC_POLYTRAJ_MODE = 2;

    enum class TaskMode {
        STATE_TO_STATE = 0,
        TRACKING = 1,
        PERCHING = 2,
        EXPLORATION = 3,
        DYNAMIC_TAKEOFF = 4,
        TRACKING_PERCHING = 5
    };

    inline general_planner::architecture::TaskType taskTypeFromTaskMode(const TaskMode mode) {
        using general_planner::architecture::TaskType;
        switch (mode) {
            case TaskMode::TRACKING:
            case TaskMode::TRACKING_PERCHING:
                return TaskType::TRACKING;
            case TaskMode::PERCHING:
                return TaskType::PERCHING;
            case TaskMode::EXPLORATION:
                return TaskType::EXPLORATION;
            case TaskMode::DYNAMIC_TAKEOFF:
                return TaskType::TAKEOFF;
            case TaskMode::STATE_TO_STATE:
            default:
                return TaskType::STATE_TO_STATE;
        }
    }

    inline general_planner::architecture::MissionMode missionModeFromTaskMode(const TaskMode mode) {
        using general_planner::architecture::MissionMode;
        if (mode == TaskMode::TRACKING_PERCHING) {
            return MissionMode::PERCHING_MISSION;
        }
        return general_planner::architecture::missionModeForTask(taskTypeFromTaskMode(mode));
    }

    inline std::string normalizeTaskMode(std::string mode) {
        std::transform(mode.begin(), mode.end(), mode.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (mode == "state_to_state" || mode == "state-2-state" || mode == "state2state" ||
            mode == "s2s" || mode == "corridor" || mode == "esdf" || mode == "plain" ||
            mode == "se3" || mode == "se3_aggressive" || mode == "aggressive" ||
            mode == "racing") {
            return "state2state";
        }
        if (mode == "track" || mode == "tracking") {
            return "tracking";
        }
        if (mode == "tracking_perching" || mode == "tracking-perching" ||
            mode == "track_perch" || mode == "track-perch") {
            return "tracking_perching";
        }
        if (mode == "perch" || mode == "perching") {
            return "perching";
        }
        if (mode == "takeoff" || mode == "dynamic_takeoff" ||
            mode == "dynamic-takeoff" || mode == "unperching") {
            return "dynamic_takeoff";
        }
        if (mode == "explore" || mode == "exploration") {
            return "exploration";
        }
        return "state2state";
    }

    inline TaskMode taskModeFromString(const std::string &mode) {
        const std::string normalized = normalizeTaskMode(mode);
        if (normalized == "tracking") {
            return TaskMode::TRACKING;
        }
        if (normalized == "tracking_perching") {
            return TaskMode::TRACKING_PERCHING;
        }
        if (normalized == "perching") {
            return TaskMode::PERCHING;
        }
        if (normalized == "dynamic_takeoff") {
            return TaskMode::DYNAMIC_TAKEOFF;
        }
        if (normalized == "exploration") {
            return TaskMode::EXPLORATION;
        }
        return TaskMode::STATE_TO_STATE;
    }

    class Config {
    public:
        bool timer_en{true};

        // Fsm Params
        bool click_goal_en{}, click_goal_3d_en{}, visualization_en{};
        bool auto_start{false};
        double replan_rate{}, resolution{};
        double click_height{};

        bool click_yaw_en{};
        string cmd_topic, mpc_cmd_topic, so3_cmd_topic, click_goal_topic, click_goal_3d_topic;
        bool publish_so3_cmd{false};
        vector<double> so3_kR{0.0, 0.0, 0.0};
        vector<double> so3_kOm{0.0, 0.0, 0.0};
        double flatness_mass{1.0};
        double flatness_dh{0.7};
        double flatness_dv{0.8};
        double flatness_grav{9.81};
        double flatness_cp{0.01};
        double flatness_v_eps{0.0001};
        string task_mode_str{"state2state"};
        TaskMode task_mode{TaskMode::STATE_TO_STATE};
        string planning_backend_str{"auto"};
        general_planner::architecture::TaskType task_type{
                general_planner::architecture::TaskType::STATE_TO_STATE};
        general_planner::architecture::MissionMode mission_mode{
                general_planner::architecture::MissionMode::SINGLE_TASK};
        general_planner::architecture::BackendType backend_type{
                general_planner::architecture::BackendType::AUTO};
        bool task_planner_en{false};
        string task_mode_topic{"/planning/task_mode"};
        string tracking_target_odom_topic{"/tracking/target_odom"};
        string tracking_target_prediction_topic{"/tracking/target_prediction"};
        bool tracking_use_target_prediction_path{true};
        string perching_surface_odom_topic{"/perching/surface_odom"};
        double dynamic_takeoff_start_delay{0.0};
        bool tracking_perching_enable{false};
        double tracking_prediction_horizon{4.0};
        double tracking_prediction_dt{0.25};
        bool tracking_prediction_use_kinodynamic{true};
        double tracking_prediction_accel{3.0};
        double tracking_prediction_vmax{4.0};
        double tracking_prediction_rho_accel{1.0};
        double tracking_prediction_max_time{0.03};
        double tracking_static_position_epsilon{0.05};
        double tracking_static_velocity_epsilon{0.05};
        double tracking_static_yaw_epsilon{0.05};
        double tracking_static_replan_remaining_time{0.8};
        double tracking_static_task_position_epsilon{0.12};
        double tracking_static_task_velocity_epsilon{0.10};
        double tracking_static_prediction_filter_velocity_epsilon{0.08};
        double tracking_static_safety_check_horizon{1.5};
        double tracking_static_safety_check_dt{0.12};
        double tracking_static_replan_log_period{1.0};
        int tracking_plan_from_rest_max_failures{4};
        double tracking_plan_from_rest_failure_backoff{0.5};
        double tracking_plan_from_rest_limited_backoff{2.0};
        bool tracking_static_finish_on_plan_failure{false};
        bool perception_replan_check_en{false};
        double perception_replan_check_rate{30.0};
        double perception_replan_check_horizon{2.0};
        double perception_replan_check_dt{0.08};
        double perception_replan_min_interval{0.08};
        int perception_replan_consecutive_hits{2};
        bool perception_replan_unknown_as_occupied{false};
        double perception_replan_emergency_horizon{0.45};
        double perception_replan_log_period{0.5};
        bool dynamic_obstacle_layer_enable{false};
        string dynamic_obstacle_layer_cloud_topic{"/cloud_registered"};
        int dynamic_obstacle_layer_cloud_queue_size{3};
        double dynamic_obstacle_layer_odom_timeout{0.2};
        double task_timeout{0.6};
        int state2state_plan_from_rest_max_failures{0};
        double state2state_plan_from_rest_max_failure_sec{0.0};
        // Inspection approach/return can legitimately wait for map evidence,
        // while a capture-viewpoint retry must be bounded and rate-limited.
        double state2state_inspection_capture_max_failure_sec{0.0};
        double state2state_inspection_retry_backoff_initial_sec{0.15};
        double state2state_inspection_retry_backoff_max_sec{1.0};
        // Capture viewpoints are camera poses, not high-speed transit goals.
        // Non-positive values retain the capture_traj profile's own limits.
        double state2state_inspection_capture_max_vel{0.0};
        double state2state_inspection_capture_max_acc{0.0};
        double state2state_inspection_capture_max_jerk{0.0};
        bool state2state_clear_goal_on_plan_failure{false};
        double yaw_dot_max{};
        bool diagnostic_log_en{true};
        string diagnostic_event_topic{"/planning/diagnostics/events"};
        bool swarm_enable{false};
        int swarm_drone_id{-1};
        double swarm_des_clearance{0.75};
        bool swarm_broadcast_enable{true};
        string swarm_traj_broadcast_topic{"/swarm/trajectory"};
        string swarm_state_broadcast_topic{"/swarm/state"};
        bool swarm_formation_reference_enable{false};
        string swarm_formation_reference_topic{"/swarm/formation_reference"};
        vector<string> swarm_traj_topics;
        vector<int> swarm_traj_ids;

        mission::InspectionMissionConfig inspection_mission;

        Config() = default;

        Config(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            vector<double> tem_gain;
            loader.LoadParam("fsm/timer_en", timer_en, false);
            loader.LoadParam("fsm/auto_start", auto_start, false);
            loader.LoadParam("fsm/click_goal_en", click_goal_en, false);
            loader.LoadParam("fsm/click_goal_3d_en", click_goal_3d_en, false);
            loader.LoadParam("fsm/click_yaw_en", click_yaw_en, false);
            loader.LoadParam("fsm/replan_rate", replan_rate, 10.0);
            loader.LoadParam("fsm/click_height", click_height, 1.5);
            loader.LoadParam("fsm/cmd_topic", cmd_topic, string("/planning/pos_cmd"));
            loader.LoadParam("fsm/mpc_cmd_topic", mpc_cmd_topic, string("/planning_cmd/mpc"));
            loader.LoadParam("fsm/publish_so3_cmd", publish_so3_cmd, false);
            loader.LoadParam("fsm/so3_cmd_topic", so3_cmd_topic, string("/planning/so3_cmd"));
            loader.LoadParam("fsm/so3_kR", so3_kR, vector<double>{0.0, 0.0, 0.0});
            loader.LoadParam("fsm/so3_kOm", so3_kOm, vector<double>{0.0, 0.0, 0.0});
            loader.LoadParam("traj_opt/flatness/mass", flatness_mass, 1.0);
            loader.LoadParam("traj_opt/flatness/dh", flatness_dh, 0.7);
            loader.LoadParam("traj_opt/flatness/dv", flatness_dv, 0.8);
            loader.LoadParam("traj_opt/flatness/grav", flatness_grav, 9.81);
            loader.LoadParam("traj_opt/flatness/cp", flatness_cp, 0.01);
            loader.LoadParam("traj_opt/flatness/v_eps", flatness_v_eps, 0.0001);
            if (so3_kR.size() != 3) {
                so3_kR = vector<double>{0.0, 0.0, 0.0};
            }
            if (so3_kOm.size() != 3) {
                so3_kOm = vector<double>{0.0, 0.0, 0.0};
            }
            loader.LoadParam("fsm/click_goal_topic", click_goal_topic, string("/planning/click_goal_topic"));
            loader.LoadParam("fsm/click_goal_3d_topic", click_goal_3d_topic, string("/goal_3d"));
            loader.LoadParam("fsm/task_mode", task_mode_str, string("state2state"));
            const string raw_task_mode_str = task_mode_str;
            task_mode_str = normalizeTaskMode(task_mode_str);
            task_mode = taskModeFromString(task_mode_str);
            task_type = taskTypeFromTaskMode(task_mode);
            mission_mode = missionModeFromTaskMode(task_mode);
            loader.LoadParam("fsm/planning_backend", planning_backend_str, string(""));
            if (planning_backend_str.empty()) {
                loader.LoadParam("general_planner/backend", planning_backend_str, string(""));
            }
            bool planner_plain_traj_en{false};
            bool planner_esdf_traj_en{false};
            loader.LoadParam("general_planner/plain_traj_en", planner_plain_traj_en, false);
            loader.LoadParam("general_planner/esdf_traj_en", planner_esdf_traj_en, false);
            const auto legacy_backend =
                    general_planner::architecture::backendTypeFromLegacyMode(raw_task_mode_str);
            if (planning_backend_str.empty() && legacy_backend.has_value()) {
                backend_type = *legacy_backend;
            } else if (!planning_backend_str.empty()) {
                backend_type = general_planner::architecture::backendTypeFromString(planning_backend_str);
            } else if (task_type == general_planner::architecture::TaskType::STATE_TO_STATE) {
                if (planner_plain_traj_en) {
                    backend_type = general_planner::architecture::BackendType::PLAIN;
                } else if (planner_esdf_traj_en) {
                    backend_type = general_planner::architecture::BackendType::ESDF;
                } else {
                    backend_type = general_planner::architecture::BackendType::CORRIDOR;
                }
            } else if (task_type == general_planner::architecture::TaskType::TRACKING) {
                bool tracking_use_snap{false};
                loader.LoadParam("general_planner/tracking/use_snap", tracking_use_snap, false);
                backend_type = tracking_use_snap
                                   ? general_planner::architecture::BackendType::SNAP_TRACKING
                                   : general_planner::architecture::BackendType::JERK_TRACKING;
            } else {
                backend_type = general_planner::architecture::defaultBackendForTask(task_type);
            }
            planning_backend_str = general_planner::architecture::toString(backend_type);
            loader.LoadParam("fsm/task_planner_en", task_planner_en, false);
            loader.LoadParam("fsm/task_mode_topic", task_mode_topic, string("/planning/task_mode"));
            loader.LoadParam("fsm/tracking_target_odom_topic", tracking_target_odom_topic,
                             string("/tracking/target_odom"));
            loader.LoadParam("fsm/tracking_target_prediction_topic", tracking_target_prediction_topic,
                             string("/tracking/target_prediction"));
            loader.LoadParam("fsm/tracking_use_target_prediction_path", tracking_use_target_prediction_path, true);
            loader.LoadParam("fsm/perching_surface_odom_topic", perching_surface_odom_topic,
                             string("/perching/surface_odom"));
            loader.LoadParam("fsm/dynamic_takeoff_start_delay", dynamic_takeoff_start_delay, 0.0);
            loader.LoadParam("general_planner/tracking_perching/enable", tracking_perching_enable, false);
            loader.LoadParam("fsm/tracking_prediction_horizon", tracking_prediction_horizon, 4.0);
            loader.LoadParam("fsm/tracking_prediction_dt", tracking_prediction_dt, 0.25);
            loader.LoadParam("fsm/tracking_prediction_use_kinodynamic", tracking_prediction_use_kinodynamic, true);
            loader.LoadParam("fsm/tracking_prediction_accel", tracking_prediction_accel, 3.0);
            loader.LoadParam("fsm/tracking_prediction_vmax", tracking_prediction_vmax, 4.0);
            loader.LoadParam("fsm/tracking_prediction_rho_accel", tracking_prediction_rho_accel, 1.0);
            loader.LoadParam("fsm/tracking_prediction_max_time", tracking_prediction_max_time, 0.03);
            loader.LoadParam("fsm/tracking_static_position_epsilon", tracking_static_position_epsilon, 0.05);
            loader.LoadParam("fsm/tracking_static_velocity_epsilon", tracking_static_velocity_epsilon, 0.05);
            loader.LoadParam("fsm/tracking_static_yaw_epsilon", tracking_static_yaw_epsilon, 0.05);
            loader.LoadParam("fsm/tracking_static_replan_remaining_time", tracking_static_replan_remaining_time, 0.8);
            loader.LoadParam("fsm/tracking_static_task_position_epsilon", tracking_static_task_position_epsilon, 0.12);
            loader.LoadParam("fsm/tracking_static_task_velocity_epsilon", tracking_static_task_velocity_epsilon, 0.10);
            loader.LoadParam("fsm/tracking_static_prediction_filter_velocity_epsilon",
                             tracking_static_prediction_filter_velocity_epsilon, 0.08);
            loader.LoadParam("fsm/tracking_static_safety_check_horizon", tracking_static_safety_check_horizon, 1.5);
            loader.LoadParam("fsm/tracking_static_safety_check_dt", tracking_static_safety_check_dt, 0.12);
            loader.LoadParam("fsm/tracking_static_replan_log_period", tracking_static_replan_log_period, 1.0);
            loader.LoadParam("fsm/tracking_plan_from_rest_max_failures",
                             tracking_plan_from_rest_max_failures,
                             4);
            loader.LoadParam("fsm/tracking_plan_from_rest_failure_backoff",
                             tracking_plan_from_rest_failure_backoff,
                             0.5);
            loader.LoadParam("fsm/tracking_plan_from_rest_limited_backoff",
                             tracking_plan_from_rest_limited_backoff,
                             2.0);
            loader.LoadParam("fsm/tracking_static_finish_on_plan_failure",
                             tracking_static_finish_on_plan_failure,
                             false);
            loader.LoadParam("fsm/perception_replan_check_en", perception_replan_check_en, false);
            loader.LoadParam("fsm/perception_replan_check_rate", perception_replan_check_rate, 30.0);
            loader.LoadParam("fsm/perception_replan_check_horizon", perception_replan_check_horizon, 2.0);
            loader.LoadParam("fsm/perception_replan_check_dt", perception_replan_check_dt, 0.08);
            loader.LoadParam("fsm/perception_replan_min_interval", perception_replan_min_interval, 0.08);
            loader.LoadParam("fsm/perception_replan_consecutive_hits", perception_replan_consecutive_hits, 2);
            loader.LoadParam("fsm/perception_replan_unknown_as_occupied", perception_replan_unknown_as_occupied, false);
            loader.LoadParam("fsm/perception_replan_emergency_horizon", perception_replan_emergency_horizon, 0.45);
            loader.LoadParam("fsm/perception_replan_log_period", perception_replan_log_period, 0.5);
            loader.LoadParam("general_planner/dynamic_obstacle_layer/enable",
                             dynamic_obstacle_layer_enable, false);
            loader.LoadParam("general_planner/dynamic_obstacle_layer/cloud_topic",
                             dynamic_obstacle_layer_cloud_topic, string("/cloud_registered"));
            loader.LoadParam("general_planner/dynamic_obstacle_layer/cloud_queue_size",
                             dynamic_obstacle_layer_cloud_queue_size, 3);
            loader.LoadParam("general_planner/dynamic_obstacle_layer/odom_timeout",
                             dynamic_obstacle_layer_odom_timeout, 0.2);
            loader.LoadParam("fsm/task_timeout", task_timeout, 0.6);
            loader.LoadParam("fsm/state2state_plan_from_rest_max_failures",
                             state2state_plan_from_rest_max_failures,
                             0);
            loader.LoadParam("fsm/state2state_plan_from_rest_max_failure_sec",
                             state2state_plan_from_rest_max_failure_sec,
                             0.0);
            loader.LoadParam("fsm/state2state_inspection_capture_max_failure_sec",
                             state2state_inspection_capture_max_failure_sec,
                             0.0);
            loader.LoadParam("fsm/state2state_inspection_retry_backoff_initial_sec",
                             state2state_inspection_retry_backoff_initial_sec,
                             0.15);
            loader.LoadParam("fsm/state2state_inspection_retry_backoff_max_sec",
                             state2state_inspection_retry_backoff_max_sec,
                             1.0);
            loader.LoadParam("fsm/state2state_inspection_capture_max_vel",
                             state2state_inspection_capture_max_vel,
                             0.0);
            loader.LoadParam("fsm/state2state_inspection_capture_max_acc",
                             state2state_inspection_capture_max_acc,
                             0.0);
            loader.LoadParam("fsm/state2state_inspection_capture_max_jerk",
                             state2state_inspection_capture_max_jerk,
                             0.0);
            loader.LoadParam("fsm/state2state_clear_goal_on_plan_failure",
                             state2state_clear_goal_on_plan_failure,
                             false);
            loader.LoadParam("fsm/diagnostic_log_en", diagnostic_log_en, true);
            loader.LoadParam("fsm/diagnostic_event_topic", diagnostic_event_topic,
                             string("/planning/diagnostics/events"));
            loader.LoadParam("general_planner/swarm/enable", swarm_enable, false);
            loader.LoadParam("general_planner/swarm/drone_id", swarm_drone_id, -1);
            loader.LoadParam("general_planner/swarm/des_clearance", swarm_des_clearance, 0.75);
            loader.LoadParam("general_planner/swarm/broadcast_enable", swarm_broadcast_enable, true);
            loader.LoadParam("general_planner/swarm/traj_broadcast_topic", swarm_traj_broadcast_topic,
                             string("/swarm/trajectory"));
            loader.LoadParam("general_planner/swarm/state_broadcast_topic", swarm_state_broadcast_topic,
                             string("/swarm/state"));
            loader.LoadParam("general_planner/swarm/formation/reference_enable",
                             swarm_formation_reference_enable, false);
            loader.LoadParam("general_planner/swarm/formation/reference_topic",
                             swarm_formation_reference_topic, string("/swarm/formation_reference"));
            loader.LoadParam("general_planner/swarm/traj_topics", swarm_traj_topics, vector<string>{});
            loader.LoadParam("general_planner/swarm/traj_ids", swarm_traj_ids, vector<int>{});


            loader.LoadParam("general_planner/yaw_dot_max", yaw_dot_max, 1.0, true);
            loader.LoadParam("general_planner/visualization_en", visualization_en, false, true);
            loader.LoadParam("rog_map/resolution", resolution, 0.1, true);

            loader.LoadParam("inspection_mission/enable", inspection_mission.enable, false);
            loader.LoadParam("inspection_mission/navigation_only",
                             inspection_mission.navigation_only, false);
            loader.LoadParam("inspection_mission/auto_start",
                             inspection_mission.auto_start, false);
            loader.LoadParam("inspection_mission/trigger_from_2d_goal",
                             inspection_mission.trigger_from_2d_goal, false);
            loader.LoadParam("inspection_mission/skip_face_detection",
                             inspection_mission.skip_face_detection, false);
            loader.LoadParam("inspection_mission/use_target_nav_goal_directly",
                             inspection_mission.use_target_nav_goal_directly, true);
            loader.LoadParam("inspection_mission/allow_mock_coverage_fallback",
                             inspection_mission.allow_mock_coverage_fallback, false);
            loader.LoadParam("inspection_mission/mock_external", inspection_mission.mock_external, true);
            loader.LoadParam("inspection_mission/mock_face_detection",
                             inspection_mission.mock_face_detection,
                             inspection_mission.mock_external);
            loader.LoadParam("inspection_mission/mock_capture",
                             inspection_mission.mock_capture,
                             inspection_mission.mock_external);
            loader.LoadParam("inspection_mission/use_internal_detector",
                             inspection_mission.use_internal_detector, false);
            loader.LoadParam("inspection_mission/apply_change_region_mask",
                             inspection_mission.apply_change_region_mask, true);
            loader.LoadParam("inspection_mission/target_file", inspection_mission.target_file,
                             string("config/mission_target.yaml"));
            loader.LoadParam("inspection_mission/start_service", inspection_mission.start_service,
                             string("/inspection/start"));
            loader.LoadParam("inspection_mission/face_request_topic",
                             inspection_mission.face_request_topic,
                             string("/inspection/face/request"));
            loader.LoadParam("inspection_mission/face_result_topic",
                             inspection_mission.face_result_topic,
                             string("/inspection/face/result"));
            loader.LoadParam("inspection_mission/face_debug_topic",
                             inspection_mission.face_debug_topic,
                             string("/inspection/face/debug"));
            loader.LoadParam("inspection_mission/capture_request_topic",
                             inspection_mission.capture_request_topic,
                             string("/inspection/capture/request"));
            loader.LoadParam("inspection_mission/capture_result_topic",
                             inspection_mission.capture_result_topic,
                             string("/inspection/capture/result"));
            loader.LoadParam("inspection_mission/status_topic", inspection_mission.status_topic,
                             string("/inspection/status"));
            loader.LoadParam("inspection_mission/cloud_topic", inspection_mission.cloud_topic,
                             string("/cloud_registered"));
            loader.LoadParam("inspection_mission/home_mode", inspection_mission.home_mode,
                             string("capture_on_trigger"));
            loader.LoadParam("inspection_mission/approach_distance_min",
                             inspection_mission.approach_distance_min, 2.0);
            loader.LoadParam("inspection_mission/approach_distance_max",
                             inspection_mission.approach_distance_max, 4.0);
            loader.LoadParam("inspection_mission/approach_distance_step",
                             inspection_mission.approach_distance_step, 0.2);
            loader.LoadParam("inspection_mission/safe_radius", inspection_mission.safe_radius, 0.6);
            loader.LoadParam("inspection_mission/flight_height_min",
                             inspection_mission.flight_height_min, 0.5);
            loader.LoadParam("inspection_mission/flight_height_max",
                             inspection_mission.flight_height_max, 5.0);
            loader.LoadParam("inspection_mission/face/forward_min",
                             inspection_mission.face_forward_min, 1.0);
            loader.LoadParam("inspection_mission/face/forward_max",
                             inspection_mission.face_forward_max, 12.0);
            loader.LoadParam("inspection_mission/face/min_confidence",
                             inspection_mission.face_min_confidence, 0.75);
            loader.LoadParam("inspection_mission/face/min_area",
                             inspection_mission.face_min_area, 4.0);
            loader.LoadParam("inspection_mission/face/min_points",
                             inspection_mission.face_min_points, 300);
            loader.LoadParam("inspection_mission/face/normal_alignment_min",
                             inspection_mission.face_normal_alignment_min, 0.8);
            loader.LoadParam("inspection_mission/face/voxel_leaf",
                             inspection_mission.face_voxel_leaf, 0.1);
            loader.LoadParam("inspection_mission/face/stability_frames",
                             inspection_mission.face_stability_frames, 3);
            loader.LoadParam("inspection_mission/face/stability_center_tol",
                             inspection_mission.face_stability_center_tol, 0.35);
            loader.LoadParam("inspection_mission/face/stability_normal_tol",
                             inspection_mission.face_stability_normal_tol, 0.15);
            loader.LoadParam("inspection_mission/face/cluster_tolerance",
                             inspection_mission.face_cluster_tolerance, 0.35);
            loader.LoadParam("inspection_mission/face/cluster_min_size",
                             inspection_mission.face_cluster_min_size, 200);
            loader.LoadParam("inspection_mission/face/ransac_dist",
                             inspection_mission.face_ransac_dist, 0.08);
            loader.LoadParam("inspection_mission/face/prior_center_tolerance",
                             inspection_mission.face_prior_center_tolerance, 3.0);
            loader.LoadParam("inspection_mission/face/prior_normal_alignment_min",
                             inspection_mission.face_prior_normal_alignment_min, 0.9);
            loader.LoadParam("inspection_mission/coverage/camera_hfov_deg",
                             inspection_mission.camera_hfov_deg, 70.0);
            loader.LoadParam("inspection_mission/coverage/camera_vfov_deg",
                             inspection_mission.camera_vfov_deg, 50.0);
            loader.LoadParam("inspection_mission/coverage/capture_distance",
                             inspection_mission.capture_distance, 4.0);
            loader.LoadParam("inspection_mission/coverage/image_overlap",
                             inspection_mission.image_overlap, 0.7);
            loader.LoadParam("inspection_mission/coverage/min_observation_count",
                             inspection_mission.min_observation_count, 2);
            loader.LoadParam("inspection_mission/coverage/min_baseline_angle_deg",
                             inspection_mission.min_baseline_angle_deg, 8.0);
            loader.LoadParam("inspection_mission/coverage/max_incidence_angle_deg",
                             inspection_mission.max_incidence_angle_deg, 60.0);
            loader.LoadParam("inspection_mission/coverage/surface_sample_resolution",
                             inspection_mission.surface_sample_resolution, 0.4);
            loader.LoadParam("inspection_mission/coverage/visibility_unknown_as_occupied",
                             inspection_mission.visibility_unknown_as_occupied, false);
            loader.LoadParam("inspection_mission/coverage/min_predicted_coverage",
                             inspection_mission.min_predicted_coverage, 0.95);
            loader.LoadParam("inspection_mission/coverage/max_viewpoints",
                             inspection_mission.max_viewpoints, 60);
            loader.LoadParam("inspection_mission/coverage/capture_settle_time_sec",
                             inspection_mission.capture_settle_time_sec, 0.5);
            loader.LoadParam("inspection_mission/change_region_thickness",
                             inspection_mission.change_region_thickness, 1.0);
            loader.LoadParam("inspection_mission/fail_retry_count",
                             inspection_mission.fail_retry_count, 1);
            loader.LoadParam("inspection_mission/face_result_timeout_sec",
                             inspection_mission.face_result_timeout_sec, 15.0);
            loader.LoadParam("inspection_mission/capture_result_timeout_sec",
                             inspection_mission.capture_result_timeout_sec, 10.0);
            loader.LoadParam("inspection_mission/arrival_yaw_tolerance_rad",
                             inspection_mission.arrival_yaw_tolerance_rad, 0.15);

        }
    };
}

#endif // GENERAL_FSM_CONFIG_H

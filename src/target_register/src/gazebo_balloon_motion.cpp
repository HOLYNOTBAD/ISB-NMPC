#include <Eigen/Dense>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/FsmState.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <swarm_msgs/InterceptionEvent.h>
#include <swarm_msgs/MassPoint.h>
#include <swarm_msgs/MassPoints.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

struct BalloonState {
    int64_t id = 0;
    // Nominal motion is kept separate so measured wind/detection-like jitter
    // can be superimposed without drifting or feeding back into keyboard,
    // random-walk, or preset-trajectory dynamics.
    Eigen::Vector3d nominal_position;
    Eigen::Vector3d nominal_velocity;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d initial_position;
    bool intercepted = false;
};

struct BalloonJitterPhases {
    std::array<Eigen::Vector3d, 2> low;
    std::array<Eigen::Vector3d, 2> high;
};

struct PresetTrajectoryPoint {
    double time = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
};

class GazeboBalloonNode {
public:
    static constexpr std::size_t kKeyPosX = 0;
    static constexpr std::size_t kKeyNegX = 1;
    static constexpr std::size_t kKeyPosY = 2;
    static constexpr std::size_t kKeyNegY = 3;
    static constexpr std::size_t kKeyPosZ = 4;
    static constexpr std::size_t kKeyNegZ = 5;

    GazeboBalloonNode()
        : nh_("~"),
          global_nh_(),
          has_mav_pose_(false),
          has_gazebo_mav_pose_(false),
          gazebo_mav_yaw_(0.0),
          motion_start_min_uav_z_(0.0),
          balloon_radius_override_(0.0),
          impact_radius_(1.50),
          uav_collision_radius_(0.34),
          interception_check_enabled_(true),
          target_active_(true),
          spawn_on_start_(true),
          auto_advance_on_idle_(false),
          relative_place_min_uav_z_(0.0),
          relative_place_use_abs_uav_z_(false),
          relative_follow_before_min_z_(false),
          relative_initialized_(false),
          balloon_count_(0),
          mav_yaw_(0.0),
          motion_started_(false),
          motion_trigger_received_(false),
          motion_start_use_abs_uav_z_(false),
          preset_elapsed_time_(0.0),
          preset_duration_(0.0),
          preset_trajectory_valid_(false),
          motion_mode_("velocity"),
          motion_mode_configured_(false),
          keyboard_control_enabled_(false),
          keyboard_acceleration_(1.0),
          keyboard_max_speed_(3.0),
          keyboard_key_timeout_sec_(0.25),
          stop_keyboard_thread_(false),
          keyboard_raw_mode_active_(false) {
        mass_points_pub_ = nh_.advertise<swarm_msgs::MassPoints>("/balloons/masspoint", 10);
        interception_status_pub_ = nh_.advertise<std_msgs::Bool>("/balloons/intercepted", 1, true);
        interception_event_pub_ =
            nh_.advertise<swarm_msgs::InterceptionEvent>("/balloons/event", 10);
        impact_marker_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("/balloons/impact_markers", 10);
        id_marker_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("/balloons/id_markers", 10);
        trajectory_marker_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("/balloons/trajectory_markers", 10);
        local_pos_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &GazeboBalloonNode::localPosCb, this);
        gazebo_model_states_sub_ = global_nh_.subscribe(
            "/gazebo/model_states", 10, &GazeboBalloonNode::gazeboModelStatesCb,
            this);
        intercept_closest_srv_ =
            global_nh_.advertiseService("/balloons/intercept_closest",
                                        &GazeboBalloonNode::interceptClosestCb,
                                        this);
        reset_trial_srv_ =
            global_nh_.advertiseService("/balloons/reset_trial",
                                        &GazeboBalloonNode::resetTrialCb,
                                        this);
        clear_trial_srv_ =
            global_nh_.advertiseService("/balloons/clear_trial",
                                        &GazeboBalloonNode::clearTrialCb,
                                        this);

        set_model_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        spawn_model_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
        delete_model_client_ = nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
        world_properties_client_ = nh_.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

        nh_.param<std::string>("model_prefix", model_prefix_, std::string("balloon_"));
        nh_.param<std::string>("world_frame", world_frame_, std::string("world"));
        nh_.param<std::string>("uav_model_name", uav_model_name_,
                               std::string("holydrone"));
        nh_.param<std::string>("motion_trigger_topic", motion_trigger_topic_,
                               std::string("/fsm/motion_trigger"));
        nh_.param<double>("motion_start_delay_sec", motion_start_delay_sec_, 0.0);
        nh_.param<double>("motion_start_min_uav_z", motion_start_min_uav_z_, 0.0);
        nh_.param<bool>("motion_start_use_abs_uav_z", motion_start_use_abs_uav_z_, false);
        motion_trigger_sub_ = global_nh_.subscribe(
            motion_trigger_topic_, 10, &GazeboBalloonNode::motionTriggerCb, this);
        nh_.param<double>("balloon_radius", balloon_radius_override_, 0.0);
        if (!std::isfinite(balloon_radius_override_) ||
            balloon_radius_override_ < 0.0) {
            ROS_FATAL_STREAM("Invalid balloon_radius=" << balloon_radius_override_
                             << "; expected zero (YAML fallback) or a positive value.");
            ros::shutdown();
            return;
        }
        // Approximate the 0.47 x 0.47 x 0.11 m airframe by a bounding sphere.
        nh_.param<double>("uav_collision_radius", uav_collision_radius_, 0.34);
        uav_collision_radius_ = std::max(0.0, uav_collision_radius_);
        nh_.param<double>("rviz_marker_alpha", rviz_marker_alpha_, 0.8);
        rviz_marker_alpha_ = std::clamp(rviz_marker_alpha_, 0.0, 1.0);
        nh_.param<double>("id_marker_text_height", id_marker_text_height_, 2.0);
        nh_.param<double>("hide_z", hide_z_, -5.0);
        nh_.param<bool>("interception_check_enabled", interception_check_enabled_, true);
        nh_.param<bool>("force_disable_interception_check",
                        force_disable_interception_check_, false);
        nh_.param<bool>("spawn_in_gazebo", spawn_in_gazebo_, true);
        nh_.param<bool>("spawn_on_start", spawn_on_start_, true);
        nh_.param<bool>("respawn_models", respawn_models_, true);
        nh_.param<bool>("auto_advance_on_idle", auto_advance_on_idle_, false);
        nh_.param<std::string>("fsm_state_topic", fsm_state_topic_,
                               std::string("/fsm/fsm_state"));
        nh_.param<double>("color_r", color_r_, 1.0);
        nh_.param<double>("color_g", color_g_, 0.0);
        nh_.param<double>("color_b", color_b_, 0.0);
        nh_.param<double>("color_a", color_a_, 1.0);
        nh_.param<double>("relative_place_min_uav_z", relative_place_min_uav_z_, 0.0);
        nh_.param<bool>("relative_place_use_abs_uav_z", relative_place_use_abs_uav_z_, false);
        nh_.param<bool>("relative_follow_before_min_z", relative_follow_before_min_z_, false);
        nh_.param<bool>("spawn_relative_to_mav", spawn_relative_to_mav_, true);
        nh_.param<int>("balloon_count", balloon_count_, 0);
        nh_.param<std::string>("motion_mode", motion_mode_, std::string("velocity"));
        nh_.param<bool>("keyboard_control", keyboard_control_enabled_, false);
        nh_.param<double>("keyboard_acceleration", keyboard_acceleration_, 1.0);
        nh_.param<double>("keyboard_max_speed", keyboard_max_speed_, 3.0);
        nh_.param<double>("keyboard_key_timeout_sec", keyboard_key_timeout_sec_, 0.25);
        nh_.param<int>("trajectory_max_points", trajectory_max_points_, 1000);
        nh_.param<double>("trajectory_min_distance", trajectory_min_distance_, 0.05);

        nh_.param<std::string>("balloon_motion_file", motion_file_,
                               ros::package::getPath("target_register") +
                                   "/config/balloon_motion/balloon_motion_preset_traj_sine_5ms.yaml");
        nh_.param<std::string>("trial_sequence_file", trial_sequence_file_, "");

        if (!loadTrialSequence(trial_sequence_file_)) {
            ros::shutdown();
            return;
        }
        if (!trial_config_files_.empty()) {
            motion_file_ = trial_config_files_.front();
        }

        if (!loadBalloonConfig(motion_file_)) {
            ROS_ERROR_STREAM("Failed to load balloon motion file: " << motion_file_);
            ros::shutdown();
            return;
        }
        if (force_disable_interception_check_) {
            interception_check_enabled_ = false;
        }
        applyMotionModeConfig();
        target_active_ = spawn_on_start_;

        if (spawn_in_gazebo_) {
            waitForGazeboServices();
            if (spawn_on_start_) {
                spawnModels();
            } else {
                deleteModels();
            }
        }

        if (motion_mode_ == "keyboard") {
            motion_trigger_received_ = true;
            startKeyboardThread();
        }

        if (auto_advance_on_idle_) {
            fsm_state_sub_ = global_nh_.subscribe(
                fsm_state_topic_, 10, &GazeboBalloonNode::fsmStateCb, this);
        }
    }

    ~GazeboBalloonNode() {
        stopKeyboardThread();
    }

    void spin() {
        ros::Rate loop_rate(30.0);
        const ros::Time start_time = ros::Time::now();
        ros::Time last_time = start_time;

        while (ros::ok()) {
            ros::spinOnce();

            const ros::Time now = ros::Time::now();
            double dt = (now - last_time).toSec();
            if (dt < 0.0) {
                dt = 0.0;
            }
            last_time = now;

            if (target_active_ && spawn_relative_to_mav_) {
                maybeInitializeRelativeSpawn();
            }

            const bool motion_delay_ready =
                (now - start_time).toSec() >= motion_start_delay_sec_;
            const double mav_height_for_motion_gate =
                motion_start_use_abs_uav_z_ ? std::abs(mav_pos_.z()) : mav_pos_.z();
            const bool motion_uav_ready =
                motion_start_min_uav_z_ <= 0.0 ||
                (has_mav_pose_ && mav_height_for_motion_gate >= motion_start_min_uav_z_);
            if (target_active_ && !motion_started_ && motion_trigger_received_ &&
                motion_delay_ready && motion_uav_ready) {
                motion_started_ = true;
                if (motion_mode_ == "preset_traj") {
                    preset_elapsed_time_ = 0.0;
                    for (auto& balloon : balloons_) {
                        balloon.initial_position = balloon.nominal_position;
                    }
                }
                ROS_INFO_STREAM("Balloon motion started by trigger topic "
                                << motion_trigger_topic_
                                << ": delay_sec=" << motion_start_delay_sec_
                                << " min_uav_z=" << motion_start_min_uav_z_
                                << " use_abs_uav_z=" << motion_start_use_abs_uav_z_
                                << " current_uav_z="
                                << (has_mav_pose_ ? mav_pos_.z() : std::numeric_limits<double>::quiet_NaN())
                                << " gate_height="
                                << (has_mav_pose_ ? mav_height_for_motion_gate
                                                  : std::numeric_limits<double>::quiet_NaN()));
            }

            if (target_active_ &&
                (!spawn_relative_to_mav_ || relative_initialized_)) {
                updateBalloonMotion(dt, motion_started_);
            }

            if (target_active_) {
                updateInterceptionMask();
            }
            publishInterceptionStatus();
            publishMassPoints();
            publishImpactMarkers();
            publishIdMarkers();
            publishTrajectoryMarkers();
            if (target_active_ && spawn_in_gazebo_) {
                syncModelsToGazebo();
            }

            loop_rate.sleep();
        }
    }

private:
    static std::string resolvePackagePath(const std::string& path) {
        if (path.empty() || path.front() == '/') {
            return path;
        }
        return ros::package::getPath("target_register") + "/" + path;
    }

    bool loadJitterVector(const YAML::Node& node, const char* key,
                          Eigen::Vector3d* value) const {
        if (!node[key]) {
            return true;
        }
        const YAML::Node values = node[key];
        if (!values.IsSequence() || values.size() != 3) {
            ROS_ERROR_STREAM(key << " must contain [forward, right, up].");
            return false;
        }
        *value = Eigen::Vector3d(values[0].as<double>(), values[1].as<double>(),
                                 values[2].as<double>());
        if (!value->allFinite() || (value->array() < 0.0).any()) {
            ROS_ERROR_STREAM(key << " must contain finite non-negative values.");
            return false;
        }
        return true;
    }

    bool loadJitterFrequencies(const YAML::Node& node, const char* key,
                               std::array<double, 2>* value) const {
        if (!node[key]) {
            return true;
        }
        const YAML::Node values = node[key];
        if (!values.IsSequence() || values.size() != value->size()) {
            ROS_ERROR_STREAM(key << " must contain exactly two frequencies.");
            return false;
        }
        for (std::size_t i = 0; i < value->size(); ++i) {
            (*value)[i] = values[i].as<double>();
            if (!std::isfinite((*value)[i]) || (*value)[i] <= 0.0 ||
                (*value)[i] >= 15.0) {
                ROS_ERROR_STREAM(key << " values must be finite and in (0, 15) Hz.");
                return false;
            }
        }
        return true;
    }

    void initializeJitter() {
        jitter_elapsed_time_ = 0.0;
        jitter_frame_yaw_ = 0.0;
        jitter_phases_.clear();
        jitter_phases_.resize(balloons_.size());
        const int64_t effective_seed =
            static_cast<int64_t>(jitter_seed_) +
            static_cast<int64_t>(jitter_seed_stride_) *
                static_cast<int64_t>(jitter_generation_++);
        jitter_effective_seed_ = static_cast<int>(effective_seed);
        std::mt19937 rng(
            static_cast<std::mt19937::result_type>(jitter_effective_seed_));
        std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * M_PI);
        for (auto& phases : jitter_phases_) {
            for (std::size_t component = 0; component < 2; ++component) {
                for (int axis = 0; axis < 3; ++axis) {
                    phases.low[component](axis) = phase_dist(rng);
                    phases.high[component](axis) = phase_dist(rng);
                }
            }
        }
    }

    bool loadTrialSequence(const std::string& path) {
        trial_config_files_.clear();
        trial_index_ = 0;
        if (path.empty()) {
            return true;
        }

        YAML::Node node;
        try {
            node = YAML::LoadFile(path);
        } catch (const YAML::Exception& e) {
            ROS_ERROR_STREAM("Trial sequence YAML load failed: " << e.what());
            return false;
        }
        if (!node["trials"] || !node["trials"].IsSequence() ||
            node["trials"].size() == 0) {
            ROS_ERROR_STREAM("Trial sequence must contain a non-empty 'trials' list: " << path);
            return false;
        }
        sequence_policy_ = node["sequence_policy"]
                               ? node["sequence_policy"].as<std::string>()
                               : "loop";
        if (sequence_policy_ != "loop" && sequence_policy_ != "stop" &&
            sequence_policy_ != "hold_last") {
            ROS_ERROR_STREAM("Unknown trial sequence_policy='" << sequence_policy_
                             << "'. Expected loop, stop, or hold_last.");
            return false;
        }
        for (const auto& trial : node["trials"]) {
            if (!trial.IsScalar() && !trial["config"]) {
                ROS_ERROR("Each trial sequence entry must be a path or contain 'config'.");
                return false;
            }
            const std::string config = trial.IsScalar()
                                           ? trial.as<std::string>()
                                           : trial["config"].as<std::string>();
            trial_config_files_.push_back(resolvePackagePath(config));
        }
        ROS_INFO_STREAM("Loaded trial sequence with " << trial_config_files_.size()
                        << " entries, policy=" << sequence_policy_ << ".");
        return true;
    }

    bool selectNextTrial() {
        if (trial_config_files_.empty()) {
            return true;
        }
        if (trial_index_ + 1 < trial_config_files_.size()) {
            ++trial_index_;
        } else if (sequence_policy_ == "loop") {
            trial_index_ = 0;
        } else if (sequence_policy_ == "stop") {
            return false;
        }
        motion_file_ = trial_config_files_[trial_index_];
        return true;
    }

    void fsmStateCb(const quadrotor_msgs::FsmState::ConstPtr& msg) {
        const bool completed_return =
            last_fsm_state_ == "RETURN_TO_START" && msg->state_name == "IDLE";
        last_fsm_state_ = msg->state_name;
        if (!completed_return) {
            return;
        }

        std::string result;
        if (!resetTrial(true, &result)) {
            ROS_ERROR_STREAM("Automatic balloon trial advance failed: " << result);
        }
    }

    void motionTriggerCb(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data && !motion_trigger_received_) {
            ROS_INFO_STREAM("Balloon motion trigger received on "
                            << motion_trigger_topic_ << ".");
        }
        motion_trigger_received_ = motion_trigger_received_ || msg->data;
    }

    bool loadBalloonConfig(const std::string& path) {
        YAML::Node node;
        try {
            node = YAML::LoadFile(path);
        } catch (const YAML::Exception& e) {
            ROS_ERROR_STREAM("YAML load failed: " << e.what());
            return false;
        }

        if (!node["balloons"]) {
            ROS_ERROR("No 'balloons' field in motion config.");
            return false;
        }

        balloons_.clear();
        motion_mode_configured_ = false;
        preset_trajectory_valid_ = false;
        preset_trajectory_points_.clear();
        preset_traj_file_.clear();

        if (balloon_count_ < -1) {
            ROS_ERROR_STREAM("Invalid balloon_count=" << balloon_count_
                             << ". Use 0 for YAML target_num, a positive number, or -1 for all balloons.");
            return false;
        }

        const std::size_t configured_count = node["balloons"].size();
        int effective_balloon_count = balloon_count_;
        if (effective_balloon_count == 0 && node["target_num"]) {
            effective_balloon_count = node["target_num"].as<int>();
        }
        if (effective_balloon_count == 0) {
            effective_balloon_count = -1;
        }
        if (effective_balloon_count < -1) {
            ROS_ERROR_STREAM("Invalid target_num=" << effective_balloon_count
                             << " in " << path
                             << ". Use a positive number, or -1 for all balloons.");
            return false;
        }
        const std::size_t requested_count =
            effective_balloon_count < 0
                ? configured_count
                : std::min(static_cast<std::size_t>(effective_balloon_count),
                           configured_count);

        int64_t fallback_id = 0;
        for (const YAML::Node& balloon_node : node["balloons"]) {
            if (balloons_.size() >= requested_count) {
                break;
            }
            BalloonState b;
            b.id = balloon_node["id"] ? balloon_node["id"].as<int64_t>() : fallback_id;
            b.position = Eigen::Vector3d(balloon_node["pos"][0].as<double>(),
                                         balloon_node["pos"][1].as<double>(),
                                         balloon_node["pos"][2].as<double>());
            b.velocity = Eigen::Vector3d(balloon_node["vel"][0].as<double>(),
                                         balloon_node["vel"][1].as<double>(),
                                         balloon_node["vel"][2].as<double>());
            b.nominal_position = b.position;
            b.nominal_velocity = b.velocity;
            b.initial_position = b.position;
            balloons_.push_back(b);
            ++fallback_id;
        }

        if (balloon_radius_override_ > 0.0) {
            impact_radius_ = balloon_radius_override_;
        } else if (!node["impact_radius"]) {
            ROS_ERROR_STREAM("Motion config has no 'impact_radius': " << path);
            return false;
        } else {
            impact_radius_ = node["impact_radius"].as<double>();
        }
        if (!std::isfinite(impact_radius_) || impact_radius_ <= 0.0) {
            ROS_ERROR_STREAM("Invalid impact_radius=" << impact_radius_
                             << " in " << path << "; expected a positive value.");
            return false;
        }
        // One selected radius drives collision, Gazebo, and RViz geometry.
        sphere_radius_ = impact_radius_;
        rviz_marker_diameter_ = 2.0 * impact_radius_;
        if (node["interception_check_enabled"]) {
            interception_check_enabled_ = node["interception_check_enabled"].as<bool>();
        }
        if (node["motion_mode"]) {
            motion_mode_ = node["motion_mode"].as<std::string>();
            motion_mode_configured_ = true;
        }
        if (node["preset_traj_file"]) {
            preset_traj_file_ = node["preset_traj_file"].as<std::string>();
            if (!preset_traj_file_.empty() && preset_traj_file_.front() != '/') {
                preset_traj_file_ = ros::package::getPath("target_register") + "/" +
                                    preset_traj_file_;
            }
        }
        if (node["keyboard_speed"]) {
            keyboard_max_speed_ = node["keyboard_speed"].as<double>();
        }
        if (node["keyboard_acceleration"]) {
            keyboard_acceleration_ = node["keyboard_acceleration"].as<double>();
        }
        if (node["keyboard_max_speed"]) {
            keyboard_max_speed_ = node["keyboard_max_speed"].as<double>();
        }
        if (node["keyboard_key_timeout_sec"]) {
            keyboard_key_timeout_sec_ = node["keyboard_key_timeout_sec"].as<double>();
        }
        if (node["trajectory_max_points"]) {
            trajectory_max_points_ = node["trajectory_max_points"].as<int>();
        }
        if (node["trajectory_min_distance"]) {
            trajectory_min_distance_ = node["trajectory_min_distance"].as<double>();
        }
        if (node["random_walk_speed"]) {
            random_walk_speed_ = node["random_walk_speed"].as<double>();
        }
        if (node["random_walk_z_scale"]) {
            random_walk_z_scale_ = node["random_walk_z_scale"].as<double>();
        }
        if (node["random_walk_interval"]) {
            random_walk_interval_ = node["random_walk_interval"].as<double>();
        }
        if (node["random_walk_acceleration"]) {
            random_walk_acceleration_ = node["random_walk_acceleration"].as<double>();
        }
        if (node["random_walk_seed"]) {
            random_walk_seed_ = node["random_walk_seed"].as<int>();
        }
        jitter_enabled_ = false;
        jitter_seed_ = 260927;
        jitter_seed_stride_ = 0;
        jitter_low_rms_ = Eigen::Vector3d(0.10, 0.30, 0.19);
        jitter_high_rms_ = Eigen::Vector3d(0.015, 0.04, 0.04);
        jitter_low_frequencies_ = {{0.15, 0.26}};
        jitter_high_frequencies_ = {{1.20, 3.70}};
        if (node["jitter_enabled"]) {
            jitter_enabled_ = node["jitter_enabled"].as<bool>();
        }
        if (node["jitter_seed"]) {
            jitter_seed_ = node["jitter_seed"].as<int>();
        }
        if (node["jitter_seed_stride"]) {
            jitter_seed_stride_ = node["jitter_seed_stride"].as<int>();
        }
        if (!loadJitterVector(node, "jitter_low_rms_m", &jitter_low_rms_) ||
            !loadJitterVector(node, "jitter_high_rms_m", &jitter_high_rms_) ||
            !loadJitterFrequencies(node, "jitter_low_frequencies_hz",
                                   &jitter_low_frequencies_) ||
            !loadJitterFrequencies(node, "jitter_high_frequencies_hz",
                                   &jitter_high_frequencies_)) {
            return false;
        }
        initializeJitter();
        model_names_.clear();
        model_names_.reserve(balloons_.size());
        for (size_t i = 0; i < balloons_.size(); ++i) {
            std::ostringstream oss;
            oss << model_prefix_ << i;
            model_names_.push_back(oss.str());
        }

        ROS_INFO_STREAM("Loaded " << balloons_.size() << " of " << configured_count
                        << " configured balloons, balloon_count=" << balloon_count_
                        << ", effective_count=" << effective_balloon_count
                        << ", radius=" << impact_radius_
                        << ", rviz_marker_diameter=" << rviz_marker_diameter_
                        << ", jitter=" << (jitter_enabled_ ? "enabled" : "disabled")
                        << ", jitter_seed=" << jitter_effective_seed_);
        resetTrajectoryHistory();
        return !balloons_.empty();
    }

    bool loadPresetTrajectoryCsv(const std::string& path) {
        std::ifstream input(path);
        if (!input) {
            ROS_ERROR_STREAM("Failed to open preset trajectory CSV: " << path);
            return false;
        }
        preset_trajectory_points_.clear();
        std::string line;
        std::getline(input, line);  // header
        while (std::getline(input, line)) {
            if (line.empty() || line.front() == '#') continue;
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream row(line);
            PresetTrajectoryPoint point;
            double ax, ay, az, jx, jy, jz;
            if (!(row >> point.time >> point.position.x() >> point.position.y() >>
                  point.position.z() >> point.velocity.x() >> point.velocity.y() >>
                  point.velocity.z() >> ax >> ay >> az >> jx >> jy >> jz) ||
                !std::isfinite(point.time) || !point.position.allFinite() ||
                !point.velocity.allFinite() ||
                (!preset_trajectory_points_.empty() &&
                 point.time <= preset_trajectory_points_.back().time)) {
                ROS_ERROR_STREAM("Invalid preset trajectory CSV row: " << line);
                return false;
            }
            preset_trajectory_points_.push_back(point);
        }
        if (preset_trajectory_points_.size() < 2 ||
            std::abs(preset_trajectory_points_.front().time) > 1e-9) {
            ROS_ERROR("Preset trajectory CSV must contain at least two points and start at t=0.");
            return false;
        }
        preset_duration_ = preset_trajectory_points_.back().time;
        preset_elapsed_time_ = 0.0;
        preset_trajectory_valid_ = true;
        ROS_INFO_STREAM("Loaded " << preset_trajectory_points_.size()
                        << " preset trajectory samples from " << path);
        return true;
    }

    std::string normalizedMotionMode(std::string mode) const {
        std::transform(mode.begin(), mode.end(), mode.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return mode;
    }

    void applyMotionModeConfig() {
        motion_mode_ = normalizedMotionMode(motion_mode_);
        if (!motion_mode_configured_ && keyboard_control_enabled_) {
            motion_mode_ = "keyboard";
        }

        if (motion_mode_ == "static") {
            keyboard_control_enabled_ = false;
            for (auto& b : balloons_) {
                b.nominal_velocity = Eigen::Vector3d::Zero();
                b.velocity = Eigen::Vector3d::Zero();
            }
        } else if (motion_mode_ == "keyboard") {
            keyboard_control_enabled_ = true;
        } else if (motion_mode_ == "velocity" || motion_mode_ == "linear") {
            keyboard_control_enabled_ = false;
            motion_mode_ = "velocity";
        } else if (motion_mode_ == "random_walk") {
            keyboard_control_enabled_ = false;
            if (random_walk_seed_ >= 0) {
                random_walk_rng_.seed(static_cast<std::mt19937::result_type>(
                    random_walk_seed_));
            } else {
                random_walk_rng_.seed(std::random_device{}());
            }
            random_walk_target_velocities_.resize(balloons_.size(), Eigen::Vector3d::Zero());
            for (size_t i = 0; i < balloons_.size(); ++i) {
                random_walk_target_velocities_[i] = randomWalkVelocity();
                balloons_[i].nominal_velocity = random_walk_target_velocities_[i];
                balloons_[i].velocity = random_walk_target_velocities_[i];
            }
        } else if (motion_mode_ == "preset_traj") {
            keyboard_control_enabled_ = false;
            if (preset_traj_file_.empty() || !loadPresetTrajectoryCsv(preset_traj_file_)) {
                ROS_FATAL_STREAM("Failed to initialize preset trajectory from '"
                                 << preset_traj_file_ << "'.");
                ros::shutdown();
            }
        } else {
            ROS_WARN_STREAM("Unknown balloon motion_mode='" << motion_mode_
                            << "', falling back to velocity.");
            keyboard_control_enabled_ = false;
            motion_mode_ = "velocity";
        }

        ROS_INFO_STREAM("Balloon motion mode: " << motion_mode_
                        << ", keyboard_acceleration=" << keyboard_acceleration_
                        << ", keyboard_max_speed=" << keyboard_max_speed_);
    }

    void waitForGazeboServices() {
        ROS_INFO("Waiting for Gazebo services...");
        set_model_state_client_.waitForExistence(ros::Duration(20.0));
        spawn_model_client_.waitForExistence(ros::Duration(20.0));
        delete_model_client_.waitForExistence(ros::Duration(20.0));
        world_properties_client_.waitForExistence(ros::Duration(20.0));
    }

    std::string buildSphereSdf() const {
        std::ostringstream sdf;
        sdf << "<sdf version='1.6'>"
            << "<model name='balloon'>"
            << "<static>true</static>"
            << "<link name='link'>"
            << "<!-- Static visual-only model: motion is commanded with "
            << "/gazebo/set_model_state and interception is handled by proximity check. -->"
            << "<visual name='visual'>"
            << "<geometry><sphere><radius>" << sphere_radius_ << "</radius></sphere></geometry>"
            << "<material>"
            << "<ambient>" << color_r_ << " " << color_g_ << " " << color_b_ << " " << color_a_ << "</ambient>"
            << "<diffuse>" << color_r_ << " " << color_g_ << " " << color_b_ << " " << color_a_ << "</diffuse>"
            << "<emissive>" << 0.1 * color_r_ << " " << 0.1 * color_g_ << " " << 0.1 * color_b_ << " 1</emissive>"
            << "</material>"
            << "</visual>"
            << "</link>"
            << "</model>"
            << "</sdf>";
        return sdf.str();
    }

    void spawnModels() {
        const std::string sdf = buildSphereSdf();
        gazebo_msgs::GetWorldProperties world;
        const bool have_world_models =
            world_properties_client_.call(world) && world.response.success;
        for (size_t i = 0; i < balloons_.size(); ++i) {
            const bool model_exists =
                !have_world_models ||
                std::find(world.response.model_names.begin(),
                          world.response.model_names.end(), model_names_[i]) !=
                    world.response.model_names.end();
            if (respawn_models_ && model_exists) {
                gazebo_msgs::DeleteModel del_req;
                del_req.request.model_name = model_names_[i];
                delete_model_client_.call(del_req);
            }

            gazebo_msgs::SpawnModel req;
            req.request.model_name = model_names_[i];
            req.request.model_xml = sdf;
            req.request.robot_namespace = "";
            req.request.initial_pose.position.x = balloons_[i].position.x();
            req.request.initial_pose.position.y = balloons_[i].position.y();
            req.request.initial_pose.position.z = balloons_[i].position.z();
            req.request.initial_pose.orientation.w = 1.0;
            req.request.reference_frame = world_frame_;

            if (!spawn_model_client_.call(req)) {
                ROS_WARN_STREAM("Failed to call spawn_sdf_model for " << model_names_[i]);
                continue;
            }
            if (!req.response.success) {
                ROS_WARN_STREAM("Spawn model failed for " << model_names_[i] << ": " << req.response.status_message);
            }
        }
    }

    void deleteModels() {
        if (!spawn_in_gazebo_) {
            return;
        }
        for (const auto& model_name : model_names_) {
            gazebo_msgs::DeleteModel req;
            req.request.model_name = model_name;
            delete_model_client_.call(req);
        }
    }

    Eigen::Vector3d jitterToWorld(const Eigen::Vector3d& local) const {
        const double c = std::cos(jitter_frame_yaw_);
        const double s = std::sin(jitter_frame_yaw_);
        return Eigen::Vector3d(c * local.x() - s * local.y(),
                               s * local.x() + c * local.y(), local.z());
    }

    void applyJitterToBalloons(bool base_motion_enabled) {
        if (!jitter_enabled_ || jitter_phases_.size() != balloons_.size()) {
            for (auto& balloon : balloons_) {
                balloon.position = balloon.nominal_position;
                balloon.velocity = base_motion_enabled
                                       ? balloon.nominal_velocity
                                       : Eigen::Vector3d::Zero();
            }
            return;
        }

        for (std::size_t i = 0; i < balloons_.size(); ++i) {
            auto& balloon = balloons_[i];
            if (balloon.intercepted) {
                continue;
            }
            Eigen::Vector3d local_offset = Eigen::Vector3d::Zero();
            Eigen::Vector3d local_velocity = Eigen::Vector3d::Zero();
            for (std::size_t component = 0; component < 2; ++component) {
                const double low_omega =
                    2.0 * M_PI * jitter_low_frequencies_[component];
                const double high_omega =
                    2.0 * M_PI * jitter_high_frequencies_[component];
                for (int axis = 0; axis < 3; ++axis) {
                    const double low_phase =
                        low_omega * jitter_elapsed_time_ +
                        jitter_phases_[i].low[component](axis);
                    const double high_phase =
                        high_omega * jitter_elapsed_time_ +
                        jitter_phases_[i].high[component](axis);
                    // With two independent sine components of equal amplitude,
                    // each configured value is the resulting band RMS.
                    local_offset(axis) +=
                        jitter_low_rms_(axis) * std::sin(low_phase) +
                        jitter_high_rms_(axis) * std::sin(high_phase);
                    local_velocity(axis) +=
                        jitter_low_rms_(axis) * low_omega * std::cos(low_phase) +
                        jitter_high_rms_(axis) * high_omega * std::cos(high_phase);
                }
            }
            balloon.position =
                balloon.nominal_position + jitterToWorld(local_offset);
            balloon.velocity =
                (base_motion_enabled ? balloon.nominal_velocity
                                     : Eigen::Vector3d::Zero()) +
                jitterToWorld(local_velocity);
        }
    }

    void updateBalloonMotion(double dt, bool base_motion_enabled) {
        jitter_elapsed_time_ += std::max(0.0, dt);
        if (!base_motion_enabled || motion_mode_ == "static") {
            applyJitterToBalloons(false);
            appendTrajectoryPoints();
            return;
        }
        if (motion_mode_ == "preset_traj") {
            updatePresetTrajectoryMotion(dt);
            applyJitterToBalloons(true);
            appendTrajectoryPoints();
            return;
        }
        const Eigen::Vector3d keyboard_acceleration = getKeyboardAcceleration();
        const bool stop_requested = keyboard_stop_requested_.exchange(false);

        if (motion_mode_ == "random_walk") {
            random_walk_timer_ -= dt;
            if (random_walk_timer_ <= 0.0) {
                random_walk_timer_ = random_walk_interval_;
                for (size_t i = 0; i < balloons_.size(); ++i) {
                    if (balloons_[i].intercepted) {
                        continue;
                    }
                    random_walk_target_velocities_[i] = randomWalkVelocity();
                }
            }
        }

        for (size_t i = 0; i < balloons_.size(); ++i) {
            auto& b = balloons_[i];
            if (b.intercepted) {
                continue;
            }
            if (motion_mode_ == "random_walk") {
                // Smoothly steer actual velocity toward target velocity.
                const Eigen::Vector3d vel_error =
                    random_walk_target_velocities_[i] - b.nominal_velocity;
                const double error_norm = vel_error.norm();
                const double max_delta = random_walk_acceleration_ * dt;
                if (error_norm <= max_delta || error_norm <= 1e-9) {
                    b.nominal_velocity = random_walk_target_velocities_[i];
                } else {
                    b.nominal_velocity += vel_error.normalized() * max_delta;
                }
                // Clamp to max speed in case acceleration overshoots.
                const double speed = b.nominal_velocity.norm();
                if (speed > random_walk_speed_ && random_walk_speed_ > 0.0) {
                    b.nominal_velocity *= random_walk_speed_ / speed;
                }
            } else if (keyboard_control_enabled_) {
                if (stop_requested) {
                    b.nominal_velocity = Eigen::Vector3d::Zero();
                }
                if (keyboard_acceleration.squaredNorm() > 1e-12) {
                    b.nominal_velocity += keyboard_acceleration * dt;
                } else {
                    const double speed = b.nominal_velocity.norm();
                    const double decel = keyboard_acceleration_ * dt;
                    if (speed <= decel || speed <= 1e-6) {
                        b.nominal_velocity = Eigen::Vector3d::Zero();
                    } else {
                        b.nominal_velocity -=
                            b.nominal_velocity.normalized() * decel;
                    }
                }
                const double speed = b.nominal_velocity.norm();
                if (keyboard_max_speed_ > 0.0 && speed > keyboard_max_speed_) {
                    b.nominal_velocity *= keyboard_max_speed_ / speed;
                }
            }
            b.nominal_position += b.nominal_velocity * dt;
        }
        applyJitterToBalloons(true);
        appendTrajectoryPoints();
    }

    void updatePresetTrajectoryMotion(double dt) {
        if (!preset_trajectory_valid_) {
            return;
        }
        preset_elapsed_time_ += std::max(0.0, dt);
        const bool finished = preset_elapsed_time_ >= preset_duration_;
        double trajectory_time = std::min(preset_elapsed_time_, preset_duration_);
        if (preset_end_behavior_ == "loop") {
            trajectory_time = std::fmod(preset_elapsed_time_, preset_duration_);
        }

        const auto upper = std::upper_bound(
            preset_trajectory_points_.begin(), preset_trajectory_points_.end(),
            trajectory_time, [](double time, const PresetTrajectoryPoint& point) {
                return time < point.time;
            });
        const std::size_t right = std::min<std::size_t>(
            std::distance(preset_trajectory_points_.begin(), upper),
            preset_trajectory_points_.size() - 1);
        const std::size_t left = right == 0 ? 0 : right - 1;
        const auto& p0 = preset_trajectory_points_[left];
        const auto& p1 = preset_trajectory_points_[right];
        const double alpha = p1.time > p0.time
                                 ? (trajectory_time - p0.time) / (p1.time - p0.time)
                                 : 0.0;
        const Eigen::Vector3d relative_position =
            (1.0 - alpha) * p0.position + alpha * p1.position;
        const Eigen::Vector3d relative_velocity =
            (1.0 - alpha) * p0.velocity + alpha * p1.velocity;
        for (auto& balloon : balloons_) {
            if (balloon.intercepted) {
                continue;
            }
            balloon.nominal_position =
                balloon.initial_position + relative_position;
            if (preset_end_behavior_ == "hold" && finished) {
                balloon.nominal_velocity.setZero();
            } else {
                balloon.nominal_velocity = relative_velocity;
            }
        }
    }

    void setBalloonPosition(BalloonState& b, const Eigen::Vector3d& position) {
        b.nominal_position = position;
        b.position = position;
        b.initial_position = position;
    }

    Eigen::Vector3d randomWalkVelocity() {
        std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * M_PI);
        std::uniform_real_distribution<double> z_dist(-1.0, 1.0);
        const double angle = angle_dist(random_walk_rng_);
        const double vx = std::cos(angle);
        const double vy = std::sin(angle);
        const double vz = z_dist(random_walk_rng_) * random_walk_z_scale_;
        Eigen::Vector3d vel(vx, vy, vz);
        vel.normalize();
        vel *= random_walk_speed_;
        return vel;
    }

    Eigen::Vector3d getKeyboardAcceleration() {
        if (!keyboard_control_enabled_) {
            return Eigen::Vector3d::Zero();
        }
        std::lock_guard<std::mutex> lock(keyboard_mutex_);
        const ros::Time now = ros::Time::now();
        Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
        if (isKeyboardKeyActiveLocked(kKeyPosX, now)) {
            acceleration.x() += keyboard_acceleration_;
        }
        if (isKeyboardKeyActiveLocked(kKeyNegX, now)) {
            acceleration.x() -= keyboard_acceleration_;
        }
        if (isKeyboardKeyActiveLocked(kKeyPosY, now)) {
            acceleration.y() += keyboard_acceleration_;
        }
        if (isKeyboardKeyActiveLocked(kKeyNegY, now)) {
            acceleration.y() -= keyboard_acceleration_;
        }
        if (isKeyboardKeyActiveLocked(kKeyPosZ, now)) {
            acceleration.z() += keyboard_acceleration_;
        }
        if (isKeyboardKeyActiveLocked(kKeyNegZ, now)) {
            acceleration.z() -= keyboard_acceleration_;
        }
        return acceleration;
    }

    bool isKeyboardKeyActiveLocked(size_t key_index, const ros::Time& now) const {
        if (key_index >= keyboard_key_times_.size() ||
            keyboard_key_times_[key_index].isZero()) {
            return false;
        }
        return (now - keyboard_key_times_[key_index]).toSec() <=
               keyboard_key_timeout_sec_;
    }

    void activateKeyboardKey(size_t key_index, const std::string& label) {
        if (key_index >= keyboard_key_times_.size()) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(keyboard_mutex_);
            keyboard_key_times_[key_index] = ros::Time::now();
        }
        ROS_INFO_STREAM_THROTTLE(0.2, "Balloon keyboard acceleration: " << label);
    }

    void stopKeyboardMotion() {
        {
            std::lock_guard<std::mutex> lock(keyboard_mutex_);
            for (auto& key_time : keyboard_key_times_) {
                key_time = ros::Time();
            }
        }
        keyboard_stop_requested_ = true;
        ROS_INFO_THROTTLE(0.2, "Balloon keyboard motion stopped.");
    }

    bool readByteWithTimeout(char* value, int timeout_ms) const {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(STDIN_FILENO, &read_fds);

        timeval timeout;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;

        const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready <= 0) {
            return false;
        }
        return read(STDIN_FILENO, value, 1) == 1;
    }

    void startKeyboardThread() {
        if (!isatty(STDIN_FILENO)) {
            ROS_WARN("Balloon keyboard control requested, but stdin is not a TTY.");
            return;
        }

        if (tcgetattr(STDIN_FILENO, &saved_terminal_attrs_) != 0) {
            ROS_WARN("Failed to read terminal settings for balloon keyboard control.");
            return;
        }

        termios raw_attrs = saved_terminal_attrs_;
        raw_attrs.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw_attrs.c_cc[VMIN] = 0;
        raw_attrs.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw_attrs) != 0) {
            ROS_WARN("Failed to enable raw terminal mode for balloon keyboard control.");
            return;
        }

        keyboard_raw_mode_active_ = true;
        stop_keyboard_thread_ = false;
        keyboard_thread_ = std::thread(&GazeboBalloonNode::keyboardLoop, this);
        ROS_INFO("Balloon keyboard control enabled: W/S -> +/-X acceleration, A/D -> +/-Y acceleration, Up/Down -> +/-Z acceleration, Space -> stop.");
    }

    void stopKeyboardThread() {
        stop_keyboard_thread_ = true;
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        if (keyboard_raw_mode_active_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &saved_terminal_attrs_);
            keyboard_raw_mode_active_ = false;
        }
    }

    void keyboardLoop() {
        while (ros::ok() && !stop_keyboard_thread_) {
            char key = 0;
            if (!readByteWithTimeout(&key, 100)) {
                continue;
            }

            if (key == '\033') {
                char seq1 = 0;
                char seq2 = 0;
                if (readByteWithTimeout(&seq1, 20) &&
                    readByteWithTimeout(&seq2, 20) &&
                    seq1 == '[') {
                    if (seq2 == 'A') {
                        activateKeyboardKey(kKeyPosZ, "+Z");
                    } else if (seq2 == 'B') {
                        activateKeyboardKey(kKeyNegZ, "-Z");
                    }
                }
                continue;
            }

            switch (std::tolower(static_cast<unsigned char>(key))) {
                case 'w':
                    activateKeyboardKey(kKeyPosX, "+X");
                    break;
                case 's':
                    activateKeyboardKey(kKeyNegX, "-X");
                    break;
                case 'a':
                    activateKeyboardKey(kKeyPosY, "+Y");
                    break;
                case 'd':
                    activateKeyboardKey(kKeyNegY, "-Y");
                    break;
                case ' ':
                    stopKeyboardMotion();
                    break;
                default:
                    break;
            }
        }
    }

    void publishMassPoints() {
        swarm_msgs::MassPoints msg;
        if (!target_active_) {
            mass_points_pub_.publish(msg);
            return;
        }
        for (const auto& b : balloons_) {
            if (b.intercepted) {
                continue;
            }
            swarm_msgs::MassPoint point;
            point.position.x = b.position.x();
            point.position.y = b.position.y();
            point.position.z = b.position.z();
            const Eigen::Vector3d effective_velocity =
                (motion_started_ ||
                 (jitter_enabled_ &&
                  (!spawn_relative_to_mav_ || relative_initialized_)))
                    ? b.velocity
                    : Eigen::Vector3d::Zero();
            point.velocity.x = effective_velocity.x();
            point.velocity.y = effective_velocity.y();
            point.velocity.z = effective_velocity.z();
            point.volume = b.id;
            msg.points.push_back(point);
        }
        mass_points_pub_.publish(msg);
    }

    void publishImpactMarkers() {
        visualization_msgs::MarkerArray marker_array;
        const ros::Time now = ros::Time::now();
        for (size_t i = 0; i < balloons_.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.stamp = now;
            marker.header.frame_id = world_frame_;
            marker.ns = "balloons";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = (!target_active_ || balloons_[i].intercepted)
                                ? visualization_msgs::Marker::DELETE
                                : visualization_msgs::Marker::ADD;
            marker.pose.position.x = balloons_[i].position.x();
            marker.pose.position.y = balloons_[i].position.y();
            marker.pose.position.z = balloons_[i].position.z();
            marker.pose.orientation.w = 1.0;
            marker.scale.x = rviz_marker_diameter_;
            marker.scale.y = rviz_marker_diameter_;
            marker.scale.z = rviz_marker_diameter_;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = rviz_marker_alpha_;
            marker.lifetime = ros::Duration(0.2);
            marker_array.markers.push_back(marker);
        }
        impact_marker_pub_.publish(marker_array);
    }

    void publishIdMarkers() {
        visualization_msgs::MarkerArray marker_array;
        const ros::Time now = ros::Time::now();
        for (std::size_t i = 0; i < balloons_.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.stamp = now;
            marker.header.frame_id = world_frame_;
            marker.ns = "balloon_ids";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = (!target_active_ || balloons_[i].intercepted)
                                ? visualization_msgs::Marker::DELETE
                                : visualization_msgs::Marker::ADD;
            marker.pose.position.x = balloons_[i].position.x();
            marker.pose.position.y = balloons_[i].position.y();
            marker.pose.position.z = balloons_[i].position.z() + sphere_radius_ + 1.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.z = id_marker_text_height_;
            marker.color.r = 0.0;
            marker.color.g = 0.9;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker.text = std::to_string(balloons_[i].id);
            marker.lifetime = ros::Duration(0.2);
            marker_array.markers.push_back(marker);
        }
        id_marker_pub_.publish(marker_array);
    }

    bool hasAnyInterceptedBalloon() const {
        if (!target_active_) {
            return false;
        }
        for (const auto& b : balloons_) {
            if (b.intercepted) {
                return true;
            }
        }
        return false;
    }

    void publishInterceptionStatus() {
        std_msgs::Bool msg;
        msg.data = hasAnyInterceptedBalloon();
        interception_status_pub_.publish(msg);
    }

    void resetTrajectoryHistory() {
        trajectory_history_.clear();
        trajectory_history_.resize(balloons_.size());
    }

    void resetImpactHistory() {
        has_previous_impact_sample_ = false;
        previous_balloon_positions_.clear();
    }

    void appendTrajectoryPoints() {
        const std::size_t n = std::min(balloons_.size(), trajectory_history_.size());
        for (std::size_t i = 0; i < n; ++i) {
            if (balloons_[i].intercepted) {
                continue;
            }
            auto& history = trajectory_history_[i];
            const auto& pos = balloons_[i].position;
            if (!history.empty() &&
                (pos - history.back()).norm() < trajectory_min_distance_) {
                continue;
            }
            history.push_back(pos);
            if (static_cast<int>(history.size()) > trajectory_max_points_) {
                history.erase(history.begin(),
                              history.end() - trajectory_max_points_);
            }
        }
    }

    void publishTrajectoryMarkers() {
        const ros::Time now = ros::Time::now();
        if (!last_trajectory_publish_time_.isZero() &&
            (now - last_trajectory_publish_time_) < trajectory_publish_interval_) {
            return;
        }
        last_trajectory_publish_time_ = now;

        visualization_msgs::MarkerArray marker_array;

        const std::size_t n = std::min(balloons_.size(), trajectory_history_.size());
        for (std::size_t i = 0; i < n; ++i) {
            const auto& history = trajectory_history_[i];
            visualization_msgs::Marker marker;
            marker.header.stamp = now;
            marker.header.frame_id = world_frame_;
            marker.ns = "balloon_trajectories";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = history.empty()
                                ? visualization_msgs::Marker::DELETE
                                : visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.08;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.8;
            marker.lifetime = ros::Duration(0);
            marker.points.reserve(history.size());
            for (const auto& pt : history) {
                geometry_msgs::Point p;
                p.x = pt.x();
                p.y = pt.y();
                p.z = pt.z();
                marker.points.push_back(p);
            }
            marker_array.markers.push_back(marker);
        }
        trajectory_marker_pub_.publish(marker_array);
    }

    void deleteAllVisualizationMarkers() {
        visualization_msgs::MarkerArray markers;
        for (const std::string& marker_namespace :
             {"balloons", "balloon_ids", "balloon_trajectories"}) {
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = world_frame_;
            marker.ns = marker_namespace;
            marker.action = visualization_msgs::Marker::DELETEALL;
            markers.markers.push_back(marker);
        }
        impact_marker_pub_.publish(markers);
        id_marker_pub_.publish(markers);
        trajectory_marker_pub_.publish(markers);
    }

    bool markBalloonIntercepted(size_t balloon_idx,
                                double dist,
                                const std::string& reason,
                                std::string* message_out = nullptr) {
        if (balloon_idx >= balloons_.size()) {
            if (message_out != nullptr) {
                *message_out = "balloon index out of range";
            }
            return false;
        }
        auto& balloon = balloons_[balloon_idx];
        if (balloon.intercepted) {
            if (message_out != nullptr) {
                *message_out = "balloon already intercepted";
            }
            return false;
        }

        balloon.intercepted = true;
        if (spawn_in_gazebo_) {
            gazebo_msgs::DeleteModel req;
            req.request.model_name = model_names_[balloon_idx];
            if (!delete_model_client_.call(req) || !req.response.success) {
                ROS_WARN_STREAM("Failed to delete intercepted balloon model "
                                << model_names_[balloon_idx] << ": "
                                << req.response.status_message);
            }
        }

        std::ostringstream oss;
        oss << "reason=" << reason
            << " name=" << model_names_[balloon_idx]
            << " dist=" << dist
            << " balloon_radius=" << impact_radius_
            << " uav_collision_radius=" << uav_collision_radius_
            << " collision_radius=" << (impact_radius_ + uav_collision_radius_)
            << " uav=[" << impact_uav_pos_.x() << ", " << impact_uav_pos_.y()
            << ", " << impact_uav_pos_.z() << "]"
            << " balloon=[" << balloon.position.x() << ", " << balloon.position.y() << ", "
            << balloon.position.z() << "]";
        const std::string message = oss.str();
        ROS_INFO_STREAM("Balloon intercepted: " << message);
        publishInterceptionStatus();
        publishInterceptionEvent(balloon, true, reason);
        if (message_out != nullptr) {
            *message_out = message;
        }
        return true;
    }

    void publishInterceptionEvent(const BalloonState& balloon,
                                  bool success,
                                  const std::string& reason) {
        swarm_msgs::InterceptionEvent event;
        event.header.stamp = ros::Time::now();
        event.header.frame_id = world_frame_;
        event.target_id = balloon.id;
        event.success = success;
        event.reason = reason;
        interception_event_pub_.publish(event);
    }

    bool markClosestBalloonIntercepted(std::string* message_out = nullptr) {
        if (!target_active_) {
            if (message_out != nullptr) {
                *message_out = "no active balloons";
            }
            return false;
        }
        if (!has_mav_pose_) {
            if (message_out != nullptr) {
                *message_out = "uav pose unavailable";
            }
            return false;
        }

        int best_idx = -1;
        double best_dist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < balloons_.size(); ++i) {
            if (balloons_[i].intercepted) {
                continue;
            }
            const double dist = (mav_pos_ - balloons_[i].position).norm();
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx < 0) {
            if (message_out != nullptr) {
                *message_out = "no active balloons";
            }
            return false;
        }

        return markBalloonIntercepted(static_cast<size_t>(best_idx),
                                      best_dist,
                                      "service",
                                      message_out);
    }

    void syncModelsToGazebo() {
        if (!target_active_) {
            return;
        }
        for (size_t i = 0; i < balloons_.size(); ++i) {
            if (balloons_[i].intercepted) {
                continue;
            }

            gazebo_msgs::SetModelState req;
            req.request.model_state.model_name = model_names_[i];
            req.request.model_state.reference_frame = world_frame_;
            req.request.model_state.pose.orientation.w = 1.0;

            const Eigen::Vector3d effective_velocity =
                motion_started_ ? balloons_[i].velocity : Eigen::Vector3d::Zero();
            req.request.model_state.pose.position.x = balloons_[i].position.x();
            req.request.model_state.pose.position.y = balloons_[i].position.y();
            req.request.model_state.pose.position.z = balloons_[i].position.z();
            req.request.model_state.twist.linear.x = effective_velocity.x();
            req.request.model_state.twist.linear.y = effective_velocity.y();
            req.request.model_state.twist.linear.z = effective_velocity.z();

            if (!set_model_state_client_.call(req)) {
                ROS_WARN_THROTTLE(1.0, "Failed to call /gazebo/set_model_state");
            }
        }
    }

    void localPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        has_mav_pose_ = true;
        mav_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        const auto& q = msg->pose.orientation;
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        mav_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    void gazeboModelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        const auto it = std::find(msg->name.begin(), msg->name.end(), uav_model_name_);
        if (it == msg->name.end()) {
            ROS_WARN_THROTTLE(2.0, "UAV model '%s' not found in /gazebo/model_states.",
                              uav_model_name_.c_str());
            return;
        }
        const std::size_t index = std::distance(msg->name.begin(), it);
        const auto& position = msg->pose[index].position;
        gazebo_mav_pos_ =
            Eigen::Vector3d(position.x, position.y, position.z);
        const auto& orientation = msg->pose[index].orientation;
        const double siny_cosp =
            2.0 * (orientation.w * orientation.z +
                   orientation.x * orientation.y);
        const double cosy_cosp =
            1.0 - 2.0 * (orientation.y * orientation.y +
                         orientation.z * orientation.z);
        gazebo_mav_yaw_ = std::atan2(siny_cosp, cosy_cosp);
        has_gazebo_mav_pose_ = true;
    }

    void updateInterceptionMask() {
        if (!target_active_ || !interception_check_enabled_ ||
            !has_mav_pose_ || impact_radius_ <= 0.0) {
            resetImpactHistory();
            return;
        }

        // Balloon states are expressed in the Gazebo world frame. Use the UAV
        // model pose from that same frame when available; MAVROS local pose can
        // have a translated origin and would produce incorrect distances.
        impact_uav_pos_ = has_gazebo_mav_pose_ ? gazebo_mav_pos_ : mav_pos_;
        const double collision_radius = impact_radius_ + uav_collision_radius_;
        const bool history_valid =
            has_previous_impact_sample_ &&
            previous_balloon_positions_.size() == balloons_.size();
        std::vector<Eigen::Vector3d> current_balloon_positions;
        current_balloon_positions.reserve(balloons_.size());
        for (const auto& balloon : balloons_) {
            current_balloon_positions.push_back(balloon.position);
        }

        for (size_t i = 0; i < balloons_.size(); ++i) {
            auto& b = balloons_[i];
            if (b.intercepted) {
                continue;
            }

            const Eigen::Vector3d current_relative = impact_uav_pos_ - b.position;
            double min_dist = current_relative.norm();
            if (history_valid) {
                const Eigen::Vector3d previous_relative =
                    previous_impact_uav_pos_ - previous_balloon_positions_[i];
                const Eigen::Vector3d relative_delta =
                    current_relative - previous_relative;
                const double delta_squared = relative_delta.squaredNorm();
                if (delta_squared > 1e-12) {
                    const double alpha = std::max(
                        0.0, std::min(1.0, -previous_relative.dot(relative_delta) /
                                                   delta_squared));
                    min_dist = std::min(
                        min_dist,
                        (previous_relative + alpha * relative_delta).norm());
                } else {
                    min_dist = std::min(min_dist, previous_relative.norm());
                }
            }

            if (min_dist <= collision_radius) {
                std::string message;
                markBalloonIntercepted(i, min_dist, "swept_proximity", &message);
                break;
            }
        }
        previous_impact_uav_pos_ = impact_uav_pos_;
        previous_balloon_positions_ = std::move(current_balloon_positions);
        has_previous_impact_sample_ = true;
    }

    bool interceptClosestCb(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& resp) {
        std::string message;
        resp.success = markClosestBalloonIntercepted(&message);
        resp.message = message;
        return true;
    }

    bool resetTrial(bool advance_sequence, std::string* message) {
        motion_started_ = false;
        motion_trigger_received_ = false;
        relative_initialized_ = false;
        resetImpactHistory();
        preset_elapsed_time_ = 0.0;
        random_walk_timer_ = 0.0;
        keyboard_stop_requested_ = true;
        {
            std::lock_guard<std::mutex> lock(keyboard_mutex_);
            for (auto& key_time : keyboard_key_times_) {
                key_time = ros::Time();
            }
        }

        // Remove models and markers using the old configuration before a
        // sequence step is allowed to change their names or count.
        target_active_ = false;
        resetImpactHistory();
        deleteModels();
        deleteAllVisualizationMarkers();
        publishInterceptionStatus();
        publishMassPoints();

        if (advance_sequence && !selectNextTrial()) {
            resetTrajectoryHistory();
            *message = "trial sequence complete; balloons cleared";
            ROS_INFO_STREAM("Balloon automatic reset: " << *message);
            return true;
        }

        if (!loadBalloonConfig(motion_file_)) {
            *message = "failed to reload balloon motion config";
            return false;
        }
        if (force_disable_interception_check_) {
            interception_check_enabled_ = false;
        }
        applyMotionModeConfig();
        target_active_ = true;

        // Convert a UAV-relative YAML position before exposing the new trial
        // through Gazebo, truth topics, or RViz. If the vehicle pose is not
        // available yet, the main loop performs this same guarded conversion
        // later; relative_initialized_ guarantees it happens only once.
        if (spawn_relative_to_mav_) {
            maybeInitializeRelativeSpawn();
        }

        if (spawn_in_gazebo_) {
            spawnModels();
            syncModelsToGazebo();
        }
        publishInterceptionStatus();
        publishMassPoints();
        publishImpactMarkers();
        publishIdMarkers();
        publishTrajectoryMarkers();

        std::ostringstream oss;
        oss << "balloons reset from " << motion_file_;
        if (!trial_config_files_.empty()) {
            oss << " (trial " << (trial_index_ + 1) << "/"
                << trial_config_files_.size() << ")";
        }
        *message = oss.str();
        ROS_INFO_STREAM("Balloon reset_trial: " << *message);
        return true;
    }

    bool resetTrialCb(std_srvs::Trigger::Request&,
                      std_srvs::Trigger::Response& resp) {
        resp.success = resetTrial(false, &resp.message);
        if (!resp.success) {
            ROS_ERROR_STREAM("Balloon reset_trial failed: " << resp.message);
        }
        return true;
    }

    bool clearTrialCb(std_srvs::Trigger::Request&,
                      std_srvs::Trigger::Response& resp) {
        motion_started_ = false;
        motion_trigger_received_ = false;
        relative_initialized_ = false;
        keyboard_stop_requested_ = true;
        target_active_ = false;
        resetImpactHistory();
        for (auto& b : balloons_) {
            b.intercepted = true;
        }
        deleteModels();
        deleteAllVisualizationMarkers();
        publishInterceptionStatus();
        publishMassPoints();
        publishImpactMarkers();
        publishIdMarkers();
        publishTrajectoryMarkers();
        resp.success = true;
        resp.message = "balloons cleared";
        ROS_INFO_STREAM("Balloon clear_trial: " << resp.message);
        return true;
    }

    void maybeInitializeRelativeSpawn() {
        if (!has_mav_pose_ || relative_initialized_) {
            return;
        }

        // Balloon positions and spawned models live in the Gazebo world
        // frame. Prefer the vehicle pose from that same frame: MAVROS local
        // yaw may differ by 90 degrees after ENU/NED estimator initialization,
        // which otherwise places a nominally forward target beside the camera.
        const Eigen::Vector3d anchor =
            has_gazebo_mav_pose_ ? gazebo_mav_pos_ : mav_pos_;
        const double anchor_yaw =
            has_gazebo_mav_pose_ ? gazebo_mav_yaw_ : mav_yaw_;
        const Eigen::Vector3d forward(
            std::cos(anchor_yaw), std::sin(anchor_yaw), 0.0);
        const Eigen::Vector3d right(
            -std::sin(anchor_yaw), std::cos(anchor_yaw), 0.0);
        const double mav_height_for_relative_gate =
            relative_place_use_abs_uav_z_ ? std::abs(anchor.z()) : anchor.z();
        if (mav_height_for_relative_gate < relative_place_min_uav_z_) {
            if (relative_follow_before_min_z_) {
                for (auto& b : balloons_) {
                    if (!b.intercepted) {
                        const auto& p = b.initial_position;
                        const Eigen::Vector3d follow_position =
                            anchor + p.x() * forward + p.y() * right +
                            Eigen::Vector3d(0.0, 0.0, p.z());
                        // Keep the YAML-relative offset in initial_position;
                        // this branch may run repeatedly before the altitude gate.
                        b.nominal_position = follow_position;
                        b.position = follow_position;
                    }
                }
            }
            return;
        }

        for (auto& b : balloons_) {
            if (!b.intercepted) {
                const auto& p = b.initial_position;
                setBalloonPosition(b, anchor + p.x() * forward + p.y() * right +
                                   Eigen::Vector3d(0.0, 0.0, p.z()));
            }
        }
        jitter_frame_yaw_ = anchor_yaw;
        relative_initialized_ = true;
        const double yaw_deg = anchor_yaw * 180.0 / std::acos(-1.0);
        ROS_INFO_STREAM("Balloon relative placement initialized from YAML pos [forward,right,up], yaw_deg=" << yaw_deg
                        << " anchor=[" << anchor.x() << ", " << anchor.y() << ", " << anchor.z() << "]");
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle global_nh_;
    ros::Publisher mass_points_pub_;
    ros::Publisher interception_status_pub_;
    ros::Publisher interception_event_pub_;
    ros::Publisher impact_marker_pub_;
    ros::Publisher id_marker_pub_;
    ros::Subscriber local_pos_sub_;
    ros::Subscriber gazebo_model_states_sub_;
    ros::Subscriber motion_trigger_sub_;
    ros::Subscriber fsm_state_sub_;
    ros::ServiceServer intercept_closest_srv_;
    ros::ServiceServer reset_trial_srv_;
    ros::ServiceServer clear_trial_srv_;
    ros::ServiceClient set_model_state_client_;
    ros::ServiceClient spawn_model_client_;
    ros::ServiceClient delete_model_client_;
    ros::ServiceClient world_properties_client_;

    std::vector<BalloonState> balloons_;
    std::vector<std::string> model_names_;
    Eigen::Vector3d mav_pos_;
    bool has_mav_pose_;
    Eigen::Vector3d gazebo_mav_pos_{Eigen::Vector3d::Zero()};
    bool has_gazebo_mav_pose_;
    double gazebo_mav_yaw_;

    std::string model_prefix_;
    std::string world_frame_;
    std::string uav_model_name_;
    std::string motion_trigger_topic_;
    std::string motion_file_;
    double motion_start_delay_sec_;
    double motion_start_min_uav_z_;
    double balloon_radius_override_;
    double sphere_radius_{1.50};
    double rviz_marker_diameter_{3.0};
    double rviz_marker_alpha_{0.8};
    double id_marker_text_height_{2.0};
    double hide_z_;
    double impact_radius_;
    double uav_collision_radius_;
    bool interception_check_enabled_;
    bool force_disable_interception_check_{false};
    bool target_active_;
    bool spawn_on_start_;
    bool auto_advance_on_idle_;
    double color_r_;
    double color_g_;
    double color_b_;
    double color_a_;
    bool spawn_in_gazebo_;
    bool respawn_models_;
    double relative_place_min_uav_z_;
    bool relative_place_use_abs_uav_z_;
    bool relative_follow_before_min_z_;
    bool spawn_relative_to_mav_{true};
    bool relative_initialized_;
    int balloon_count_;
    double mav_yaw_;
    bool motion_started_;
    bool motion_trigger_received_;
    bool motion_start_use_abs_uav_z_;
    std::string fsm_state_topic_;
    std::string trial_sequence_file_;
    std::string sequence_policy_{"loop"};
    std::string last_fsm_state_;
    std::vector<std::string> trial_config_files_;
    std::size_t trial_index_{0};
    std::string preset_traj_file_;
    std::string preset_end_behavior_{"hold"};
    std::vector<PresetTrajectoryPoint> preset_trajectory_points_;
    double preset_elapsed_time_;
    double preset_duration_;
    bool preset_trajectory_valid_;
    std::string motion_mode_;
    bool motion_mode_configured_;
    bool keyboard_control_enabled_;
    double keyboard_speed_;
    double keyboard_acceleration_{1.0};
    double keyboard_max_speed_{3.0};
    double keyboard_key_timeout_sec_{0.25};
    std::atomic<bool> keyboard_stop_requested_{false};
    Eigen::Vector3d keyboard_velocity_;
    std::array<ros::Time, 6> keyboard_key_times_{};
    std::mutex keyboard_mutex_;
    std::thread keyboard_thread_;
    std::atomic<bool> stop_keyboard_thread_;
    bool keyboard_raw_mode_active_;
    termios saved_terminal_attrs_;

    ros::Publisher trajectory_marker_pub_;
    std::vector<std::vector<Eigen::Vector3d>> trajectory_history_;
    int trajectory_max_points_{1000};
    double trajectory_min_distance_{0.05};
    ros::Time last_trajectory_publish_time_;
    ros::Duration trajectory_publish_interval_{0.2};  // 5 Hz max

    double random_walk_speed_{3.0};
    double random_walk_z_scale_{0.3};
    double random_walk_interval_{2.0};
    double random_walk_acceleration_{1.5};
    int random_walk_seed_{-1};
    double random_walk_timer_{0.0};
    std::mt19937 random_walk_rng_;

    // Field-calibrated two-band target jitter. Vectors use the balloon's
    // spawn-relative [forward, right, up] frame and specify per-axis RMS.
    bool jitter_enabled_{false};
    int jitter_seed_{260927};
    int jitter_seed_stride_{0};
    int jitter_effective_seed_{260927};
    std::uint64_t jitter_generation_{0};
    Eigen::Vector3d jitter_low_rms_{0.10, 0.30, 0.19};
    Eigen::Vector3d jitter_high_rms_{0.015, 0.04, 0.04};
    std::array<double, 2> jitter_low_frequencies_{{0.15, 0.26}};
    std::array<double, 2> jitter_high_frequencies_{{1.20, 3.70}};
    std::vector<BalloonJitterPhases> jitter_phases_;
    double jitter_elapsed_time_{0.0};
    double jitter_frame_yaw_{0.0};
    std::vector<Eigen::Vector3d> random_walk_target_velocities_;
    bool has_previous_impact_sample_{false};
    Eigen::Vector3d impact_uav_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d previous_impact_uav_pos_{Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> previous_balloon_positions_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_balloon_node");
    GazeboBalloonNode node;
    node.spin();
    return 0;
}

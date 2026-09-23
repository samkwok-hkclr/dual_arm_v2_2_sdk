#include "control_api.h"
#include "gripper_config.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>
#include <fstream>
#include <cerrno>
#include <cstring>

#include <yaml-cpp/yaml.h>

#include "robot_platform_utils/cpp/include/config_loader.h"
#include "robot_platform_utils/cpp/include/cuarm_message_handler.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/cuarm_udp.h"
#include "robot_platform_utils/cpp/include/time_utils.h"

namespace dual_arm_v2_2_sdk {
namespace {

constexpr int kBufferSize = 4096;
constexpr int kAckTimeoutMs = 200;
constexpr float kMinimumProfileTime = 0.01F;

float curve_fit_time(uint16_t radio) {
    // 0..100 maps to 10 ms..1.01 s; a larger value produces a smoother fit.
    return kMinimumProfileTime + static_cast<float>(radio) / 100.0F;
}

float filter_cutoff_frequency(uint16_t radio) {
    // Logarithmic mapping gives perceptually smoother adjustment across 20..1 Hz.
    constexpr float kMaximumCutoffHz = 20.0F;
    constexpr float kMinimumCutoffHz = 1.0F;
    const float normalized = static_cast<float>(radio) / 999.0F;
    return kMaximumCutoffHz *
           std::pow(kMinimumCutoffHz / kMaximumCutoffHz, normalized);
}

}  // namespace

struct ControlApi::Impl {
    YAML::Node config;
    PanelCommand command{};
    PlannerState state{};
    std::unique_ptr<CuarmUdp<PlannerState, PanelCommand>> udp;
    std::thread receive_thread;
    std::atomic<bool> shutdown{false};
    std::atomic<bool> received_state{false};
    std::atomic<uint64_t> state_generation{0};
    mutable std::mutex state_mutex;
    std::mutex command_mutex;
    std::vector<int> joint_sizes;
    std::vector<std::string> gripper_names;
    std::vector<std::vector<float>> max_acceleration;
    uint8_t previous_trajectory_mode = 255;
    bool previous_follow = false;

    // explicit Impl(const std::string& prefix) {
    //     config = loadYamlConfig(prefix + "/config.yaml");
    //     initialize_command();

    //     const auto panel = config["panel"];
    //     const auto rt = config["rt_control"];
    //     udp = std::make_unique<CuarmUdp<PlannerState, PanelCommand>>(
    //         panel["ip"].as<std::string>(), panel["port"].as<int>(),
    //         rt["ip"].as<std::string>(), rt["port"].as<int>(),
    //         &CuarmMessageHandler::unpack_planner_state,
    //         &CuarmMessageHandler::pack_panel_command, kBufferSize);

    //     receive_thread = std::thread(&Impl::receive_loop, this);
    // }

    explicit Impl(const std::string& prefix) {
        std::cerr << "[Impl] === begin construct Impl ===" << std::endl;

        const std::string config_path = prefix + "/config.yaml";
        std::cerr << "[Impl] loading config from: " << config_path << std::endl;

        // 先做一次文件存在性/可读性检查，避免后续 YAML 报错信息不清
        {
            std::ifstream probe(config_path);
            if (!probe.good()) {
                std::cerr << "[Impl][FATAL] cannot open config file: " << config_path
                        << " errno=" << errno << " (" << std::strerror(errno) << ")"
                        << std::endl;
                throw std::runtime_error("config file not readable: " + config_path);
            }
            std::cerr << "[Impl] config file is readable, size check..."
                    << std::endl;
        }

        try {
            config = loadYamlConfig(config_path);
        } catch (const std::exception& e) {
            std::cerr << "[Impl][FATAL] loadYamlConfig threw: " << e.what()
                    << std::endl;
            throw;
        }

        if (!config || config.IsNull()) {
            std::cerr << "[Impl][FATAL] config is null after load" << std::endl;
            throw std::runtime_error("config is null");
        }
        std::cerr << "[Impl] config loaded OK" << std::endl;

        if (config["robot"]) {
            // std::cerr << "[Impl] === robot node dump begin ===\n"
            //         << YAML::Dump(config["robot"])
            //         << "\n[Impl] === robot node dump end ===" << std::endl;
            std::cerr << "[Impl] config['robot'] is okay" << std::endl;
        } else {
            std::cerr << "[Impl][FATAL] config['robot'] is missing" << std::endl;
        }

        initialize_command();
        std::cerr << "[Impl] initialize_command() done" << std::endl;

        auto require = [&](const char* name) -> YAML::Node {
            YAML::Node n = config[name];
            if (!n || n.IsNull()) {
                std::cerr << "[Impl][FATAL] missing top-level key: " << name
                        << std::endl;
                throw std::runtime_error(std::string("missing key: ") + name);
            }
            std::cerr << "[Impl] found top-level key: " << name << std::endl;
            return n;
        };

        auto get_str = [&](const YAML::Node& parent, const char* key,
                        const char* ctx) -> std::string {
            const YAML::Node n = parent[key];           // 同样拷一份更安全
            if (!n || n.IsNull()) {
                std::cerr << "[Impl][FATAL] missing key '" << key << "' under "
                        << ctx << std::endl;
                throw std::runtime_error(std::string(ctx) + "." + key + " missing");
            }
            try {
                std::string v = n.as<std::string>();
                std::cerr << "[Impl] " << ctx << "." << key << " = " << v << std::endl;
                return v;
            } catch (const std::exception& e) {
                std::cerr << "[Impl][FATAL] " << ctx << "." << key
                        << " bad conversion: " << e.what() << std::endl;
                throw;
            }
        };

        auto get_int = [&](const YAML::Node& parent, const char* key,
                        const char* ctx) -> int {
            const YAML::Node n = parent[key];
            if (!n || n.IsNull()) {
                std::cerr << "[Impl][FATAL] missing key '" << key << "' under "
                        << ctx << std::endl;
                throw std::runtime_error(std::string(ctx) + "." + key + " missing");
            }
            try {
                int v = n.as<int>();
                std::cerr << "[Impl] " << ctx << "." << key << " = " << v << std::endl;
                return v;
            } catch (const std::exception& e) {
                std::cerr << "[Impl][FATAL] " << ctx << "." << key
                        << " bad conversion: " << e.what() << std::endl;
                throw;
            }
        };

        const YAML::Node panel = require("panel");
        const YAML::Node rt    = require("rt_control");

        const std::string panel_ip = get_str(panel, "ip", "panel");
        const int         panel_port = get_int(panel, "port", "panel");
        const std::string rt_ip    = get_str(rt, "ip", "rt_control");
        const int         rt_port  = get_int(rt, "port", "rt_control");

        std::cerr << "[Impl] creating CuarmUdp ..." << std::endl;
        udp = std::make_unique<CuarmUdp<PlannerState, PanelCommand>>(
            panel_ip, panel_port,
            rt_ip,    rt_port,
            &CuarmMessageHandler::unpack_planner_state,
            &CuarmMessageHandler::pack_panel_command,
            kBufferSize);
        std::cerr << "[Impl] CuarmUdp created OK" << std::endl;

        std::cerr << "[Impl] rt_control endpoint: " << rt_ip << ":" << rt_port
                << std::endl;

        std::cerr << "[Impl] starting receive thread ..." << std::endl;
        receive_thread = std::thread(&Impl::receive_loop, this);
        std::cerr << "[Impl] === Impl construct finished ===" << std::endl;
    }

    ~Impl() {
        shutdown.store(true);
        if (receive_thread.joinable()) receive_thread.join();
        if (udp) udp->close();
    }

    void initialize_command() {
        const auto arms = config["robot"]["arm"];
        command.connection_state = ConnectionState::kRemote;
        // The planner starts with received_sequence_id == 0. Starting at 1
        // prevents an unsolicited startup state from being mistaken for the
        // acknowledgement of our first command.
        command.sequence_id = 1;
        command.need_setting_update = true;
        command.simulation = config["rt_control"]["hardware_simulation"].as<bool>();
        command.motion_type = MotionControl::kJoint;
        command.target_type = ControlType::kPosition;
        command.actuator_mode = ControlType::kPosition;
        command.interpolation_type = InterpolationMethod::kQuintic;
        command.interpolation_speed_ratio = 1.0F;
        command.InterpolationAccTime = config["panel"]["general"]["acc_time"].as<float>();
        command.InterpolationConstVelTime = config["panel"]["general"]["vel_time"].as<float>();
        command.arm_target_mode = PanelTargetMode::kSinglePoint;
        command.ArmSize = static_cast<uint8_t>(arms.size());

        for (int arm = 0; arm < command.ArmSize; ++arm) {
            const auto node = arms[arm];
            const int count = node["joint_size"].as<int>();
            joint_sizes.push_back(count);
            max_acceleration.emplace_back(count, 0.0F);
            command.JointSize[arm] = static_cast<uint8_t>(count);
            const std::string robot_name = node["name"].as<std::string>() + "-" + node["type"].as<std::string>();
            std::strncpy(command.RobotName[arm], robot_name.c_str(), MAX_ROBOTNAME_SIZE - 1);
            for (int joint = 0; joint < count; ++joint) {
                command.enable_joint[arm][joint] = node["enable_joint"][joint].as<bool>();
                command.arm_soft_limit_position[arm][joint][0] =
                    node["safety"]["joint_soft_limit"]["position"][0][joint].as<float>() * static_cast<float>(M_PI / 180.0);
                command.arm_soft_limit_position[arm][joint][1] =
                    node["safety"]["joint_soft_limit"]["position"][1][joint].as<float>() * static_cast<float>(M_PI / 180.0);
                const float max_velocity = node["safety"]["joint_soft_limit"]["velocity"][joint].as<float>() * static_cast<float>(M_PI / 180.0);
                command.arm_soft_limit_velocity[arm][joint][0] = -max_velocity;
                command.arm_soft_limit_velocity[arm][joint][1] = max_velocity;
                if (node["safety"]["joint_soft_limit"]["acceleration"])
                    max_acceleration[arm][joint] = node["safety"]["joint_soft_limit"]["acceleration"][joint].as<float>() * static_cast<float>(M_PI / 180.0);
            }
        }

        const auto grippers = config["robot"]["gripper"];
        command.GripperSize = grippers ? static_cast<uint8_t>(grippers.size()) : 0;
        for (int gripper = 0; gripper < command.GripperSize; ++gripper)
        {
            gripper_names.push_back(grippers[gripper]["name"].as<std::string>());
            command.GripperJointSize[gripper] =
                grippers[gripper]["joint_size"].as<uint8_t>();
            command.GripperSpeed[gripper] = CUARM_GRIPPER_DEFAULT_SPEED;
            command.GripperForce[gripper] = CUARM_GRIPPER_DEFAULT_FORCE;
        }
    }

    void receive_loop() {
        fprintf(stderr, "[recv] thread started, shutdown=%d\n", (int)shutdown.load());
        while (!shutdown.load()) {
            PlannerState incoming{};
            if (udp->receive(&incoming, 10000)) {
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    state = incoming;
                }
                received_state.store(true);
                state_generation.fetch_add(1);
            }
        }
        fprintf(stderr, "[recv] loop exit, shutdown=%d\n", (int)shutdown.load());
    }

    std::vector<int> group_indices(Group group) const {
        switch (group) {
            case Group::ALL: return {0, 1, 2};
            case Group::LEFT_ARM: return {0};
            case Group::RIGHT_ARM: return {2};
            case Group::HEAD: return {1};
            case Group::ELEVATOR: return {};
        }
        return {};
    }

    size_t expected_joint_count(Group group) const {
        size_t result = 0;
        for (int arm : group_indices(group)) {
            if (arm < 0 || arm >= static_cast<int>(joint_sizes.size())) return 0;
            result += static_cast<size_t>(joint_sizes[arm]);
        }
        return result;
    }

    int gripper_index(Group group) const {
        if (command.GripperSize == 0) return -1;

        const char* target_name = nullptr;
        if (group == Group::LEFT_ARM) target_name = "gripperL";
        else if (group == Group::RIGHT_ARM) target_name = "gripperR";
        else return -1;

        for (int i = 0; i < static_cast<int>(gripper_names.size()); ++i) {
            if (gripper_names[i] == target_name) return i;
        }

        if (command.GripperSize == 1) {
            if (group == Group::LEFT_ARM && CUARM_GRIPPER_ARM_INDEX == 0) return 0;
            if (group == Group::RIGHT_ARM && CUARM_GRIPPER_ARM_INDEX == 2) return 0;
        }
        return -1;
    }

    PlannerState state_snapshot() const {
        std::lock_guard<std::mutex> lock(state_mutex);
        return state;
    }

    void synchronize_sequence_with_planner() {
        if (!received_state.load()) return;
        const uint32_t received_sequence = state_snapshot().received_sequence_id;
        if (command.sequence_id <= received_sequence)
            command.sequence_id = received_sequence + 1;
    }

    bool send_once() {
        try {
            command.send_timestamp = get_time_now();
            udp->send(&command);
            ++command.sequence_id;
            return true;
        } catch (...) {
            return false;
        }
    }

    bool wait_for_ack(uint32_t sequence, uint64_t generation_before_send,
                      int timeout_ms = kAckTimeoutMs) const {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            if (state_generation.load() > generation_before_send &&
                state_snapshot().received_sequence_id >= sequence)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }

    RetCode send_with_ack(int timeout_ms = kAckTimeoutMs) {
        // rt_control can outlive this SDK object/process. Continue after the
        // sequence number it already accepted, rather than restarting at 1
        // and having every packet rejected as stale.
        synchronize_sequence_with_planner();
        const uint32_t sequence = command.sequence_id;
        const uint64_t generation_before_send = state_generation.load();
        if (!send_once()) return RetCode::SEND_FAILED;
        return wait_for_ack(sequence, generation_before_send, timeout_ms)
                   ? RetCode::SUCCESS
                   : RetCode::TIMEOUT;
    }

    void hold_all_groups_at_current_position() {
        if (!received_state.load()) return;
        const PlannerState snapshot = state_snapshot();
        for (int arm = 0; arm < command.ArmSize; ++arm)
            for (int joint = 0; joint < joint_sizes[arm]; ++joint)
                command.JointCmd[arm][joint] = snapshot.JointPos[arm][joint];
        for (int gripper = 0;
             gripper < command.GripperSize && gripper < snapshot.GripperSize;
             ++gripper)
            for (int joint = 0;
                 joint < command.GripperJointSize[gripper]; ++joint)
                command.GripperJointCmd[gripper][joint] =
                    snapshot.GripperJointPos[gripper][joint];
    }

    RetCode apply_settings() {
        command.need_setting_update = true;
        RetCode result = send_with_ack();
        if (result != RetCode::SUCCESS) return result;

        // The settings packet makes rt_control refresh feedback without
        // applying a motion command. Use that feedback as the first enabled
        // position target, so enabling can never fall through to zero-filled
        // JointCmd values.
        hold_all_groups_at_current_position();
        command.need_setting_update = false;
        return send_with_ack();
    }

    RetCode ensure_initialized() {
        return command.need_setting_update ? apply_settings()
                                           : RetCode::SUCCESS;
    }

    void hold_unselected_groups(const std::vector<int>& selected) {
        if (!received_state.load()) return;
        const PlannerState snapshot = state_snapshot();
        for (int arm = 0; arm < command.ArmSize; ++arm) {
            if (std::find(selected.begin(), selected.end(), arm) != selected.end()) continue;
            for (int joint = 0; joint < joint_sizes[arm]; ++joint)
                command.JointCmd[arm][joint] = snapshot.JointPos[arm][joint];
        }
    }
};

ControlApi::ControlApi(const std::string& config_prefix_path)
    : impl_(std::make_unique<Impl>(config_prefix_path)) {}

ControlApi::~ControlApi() = default;

RetCode ControlApi::set_group(Group group, bool enable) {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    const auto indices = impl_->group_indices(group);
    if (indices.empty()) return RetCode::CONTROLLER_ERROR;
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            impl_->command.enable_joint[arm][joint] = enable;
    return impl_->apply_settings();
}

std::tuple<RetCode, std::vector<State>> ControlApi::get_group_state(Group group) const {
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || !impl_->received_state.load()) return {RetCode::RECEIVE_FAILED, {}};
    const PlannerState snapshot = impl_->state_snapshot();
    std::vector<State> result;
    for (int arm : indices) {
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint) {
            if (snapshot.system_state == SystemState::kError)
                result.push_back(State::ERROR);
            else
                result.push_back(snapshot.enabled_joint[arm][joint] ? State::OPERATIONAL : State::SHUTDOWN);
        }
    }
    return {RetCode::SUCCESS, result};
}

RetCode ControlApi::move_joint(Group group, const std::vector<float>& joints,
                               bool follow, uint8_t trajectory_mode, uint16_t radio) {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || joints.size() != impl_->expected_joint_count(group))
        return RetCode::CONTROLLER_ERROR;
    if (follow && (trajectory_mode > 2 ||
                   (trajectory_mode == 1 && radio > 100) ||
                   (trajectory_mode == 2 && radio > 999)))
        return RetCode::CONTROLLER_ERROR;
    if (!impl_->received_state.load()) return RetCode::RECEIVE_FAILED;

    impl_->command.motion_type = MotionControl::kJoint;
    impl_->command.target_type = ControlType::kPosition;
    impl_->command.actuator_mode = ControlType::kPosition;
    impl_->command.interpolation_speed_ratio = 1.0F;
    impl_->command.InterpolationConstVelTime = 0.0F;

    if (!follow) {
        impl_->command.arm_target_mode = PanelTargetMode::kSinglePoint;
        impl_->command.interpolation_type = InterpolationMethod::kQuintic;
        impl_->command.InterpolationAccTime =
            impl_->config["panel"]["general"]["acc_time"].as<float>();
    } else if (trajectory_mode == 0) {
        impl_->command.arm_target_mode = PanelTargetMode::kSinglePoint;
        impl_->command.interpolation_type = InterpolationMethod::kDirect;
        impl_->command.InterpolationAccTime = kMinimumProfileTime;
    } else if (trajectory_mode == 1) {
        impl_->command.arm_target_mode = PanelTargetMode::kSinglePoint;
        impl_->command.interpolation_type = InterpolationMethod::kQuintic;
        impl_->command.InterpolationAccTime = curve_fit_time(radio);
    } else {
        impl_->command.arm_target_mode = PanelTargetMode::kRawSinglePoint;
        impl_->command.interpolation_type = InterpolationMethod::kNone;
        // In filter mode this legacy wire field carries cutoff frequency in Hz.
        impl_->command.NoneInterpolationSaturationRatio = filter_cutoff_frequency(radio);
        // Disable the additional time-to-target clamp; velocity limiting remains active.
        impl_->command.InterpolationAccTime = 0.0F;
    }

    impl_->command.reset_interpolation =
        impl_->previous_follow != follow || impl_->previous_trajectory_mode != trajectory_mode;
    impl_->hold_unselected_groups(indices);
    size_t source = 0;
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            impl_->command.JointCmd[arm][joint] = joints[source++];

    impl_->synchronize_sequence_with_planner();
    const uint32_t sequence = impl_->command.sequence_id;
    const uint64_t generation_before_send = impl_->state_generation.load();
    if (!impl_->send_once()) return RetCode::SEND_FAILED;
    impl_->command.reset_interpolation = false;
    impl_->previous_follow = follow;
    impl_->previous_trajectory_mode = trajectory_mode;

    // High-follow calls must remain non-blocking so the caller can maintain <=10 ms cadence.
    if (follow) return RetCode::SUCCESS;
    return impl_->wait_for_ack(sequence, generation_before_send)
               ? RetCode::SUCCESS
               : RetCode::TIMEOUT;
}

std::tuple<RetCode, std::vector<float>> ControlApi::get_joint(Group group) const {
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || !impl_->received_state.load()) return {RetCode::RECEIVE_FAILED, {}};
    const PlannerState snapshot = impl_->state_snapshot();
    std::vector<float> result;
    result.reserve(impl_->expected_joint_count(group));
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            result.push_back(snapshot.JointPos[arm][joint]);
    return {RetCode::SUCCESS, result};
}

std::tuple<RetCode, std::vector<float>>
ControlApi::get_joint_velocity(Group group) const {
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || !impl_->received_state.load())
        return {RetCode::RECEIVE_FAILED, {}};

    const PlannerState snapshot = impl_->state_snapshot();
    std::vector<float> result;
    result.reserve(impl_->expected_joint_count(group));
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            result.push_back(snapshot.JointVel[arm][joint]);
    return {RetCode::SUCCESS, result};
}

std::vector<float> ControlApi::set_joint_max_speed(Group group, const std::vector<float>& limits) {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || limits.size() != impl_->expected_joint_count(group)) return {};
    size_t source = 0;
    for (int arm : indices) {
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint) {
            const float value = std::abs(limits[source++]);
            impl_->command.arm_soft_limit_velocity[arm][joint][0] = -value;
            impl_->command.arm_soft_limit_velocity[arm][joint][1] = value;
        }
    }
    return impl_->apply_settings() == RetCode::SUCCESS ? limits : std::vector<float>{};
}

std::vector<float> ControlApi::set_joint_max_acc(Group group, const std::vector<float>& limits) {
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || limits.size() != impl_->expected_joint_count(group)) return {};
    size_t source = 0;
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            impl_->max_acceleration[arm][joint] = std::abs(limits[source++]);
    return limits;
}

std::vector<float> ControlApi::get_joint_max_limit(Group group) const {
    const auto indices = impl_->group_indices(group);
    if (indices.empty()) return {};
    std::vector<float> result;
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            result.push_back(impl_->command.arm_soft_limit_position[arm][joint][1]);
    return result;
}

std::vector<float> ControlApi::get_joint_min_limit(Group group) const {
    const auto indices = impl_->group_indices(group);
    if (indices.empty()) return {};
    std::vector<float> result;
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            result.push_back(impl_->command.arm_soft_limit_position[arm][joint][0]);
    return result;
}

RetCode ControlApi::set_gripper_config(Group group, uint16_t speed,
                                       uint16_t force, bool block,
                                       int timeout) {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    const int gripper = impl_->gripper_index(group);
    if (gripper < 0 || speed < CUARM_GRIPPER_SPEED_MIN ||
        speed > CUARM_GRIPPER_SPEED_MAX ||
        force < CUARM_GRIPPER_FORCE_MIN ||
        force > CUARM_GRIPPER_FORCE_MAX ||
        (block && timeout <= 0))
        return RetCode::CONTROLLER_ERROR;
    if (!impl_->received_state.load()) return RetCode::RECEIVE_FAILED;

    impl_->command.GripperSpeed[gripper] = speed;
    impl_->command.GripperForce[gripper] = force;
    const RetCode initialization = impl_->ensure_initialized();
    if (initialization != RetCode::SUCCESS) return initialization;
    if (block)
        return impl_->send_with_ack(timeout * 1000);

    impl_->synchronize_sequence_with_planner();
    return impl_->send_once() ? RetCode::SUCCESS : RetCode::SEND_FAILED;
}

RetCode ControlApi::set_gripper_position(Group group, uint16_t position,
                                         bool block, int timeout) {
    std::unique_lock<std::mutex> lock(impl_->command_mutex);
    const int gripper = impl_->gripper_index(group);
    if (gripper < 0 || position < CUARM_GRIPPER_POSITION_MIN ||
        position > CUARM_GRIPPER_POSITION_MAX ||
        (block && timeout <= 0))
        return RetCode::CONTROLLER_ERROR;
    if (!impl_->received_state.load()) return RetCode::RECEIVE_FAILED;

    const RetCode initialization = impl_->ensure_initialized();
    if (initialization != RetCode::SUCCESS) return initialization;

    impl_->command.GripperJointCmd[gripper][0] =
        static_cast<float>(position) /
        static_cast<float>(CUARM_GRIPPER_POSITION_MAX);
    const uint32_t gripper_command =
        ++impl_->command.GripperCommandId[gripper];

    impl_->synchronize_sequence_with_planner();
    const uint32_t sequence = impl_->command.sequence_id;
    const uint64_t generation_before_send = impl_->state_generation.load();
    if (!impl_->send_once()) return RetCode::SEND_FAILED;
    if (!block) return RetCode::SUCCESS;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(timeout);
    if (!impl_->wait_for_ack(
            sequence, generation_before_send,
            std::min(kAckTimeoutMs, std::max(1, timeout * 1000))))
        return RetCode::TIMEOUT;

    while (std::chrono::steady_clock::now() < deadline) {
        const PlannerState snapshot = impl_->state_snapshot();
        if (snapshot.GripperCommandId[gripper] >= gripper_command) {
            if (snapshot.GripperStatus[gripper] == 2)
                return RetCode::CONTROLLER_ERROR;
            if (snapshot.GripperStatus[gripper] == 1)
                return RetCode::SUCCESS;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return RetCode::TIMEOUT;
}

std::tuple<RetCode, std::unordered_map<std::string, uint16_t>>
ControlApi::get_gripper_state(Group group) const {
    const int gripper = impl_->gripper_index(group);
    if (gripper < 0) return {RetCode::CONTROLLER_ERROR, {}};
    if (!impl_->received_state.load()) return {RetCode::RECEIVE_FAILED, {}};

    const PlannerState snapshot = impl_->state_snapshot();
    if (gripper >= snapshot.GripperSize)
        return {RetCode::RECEIVE_FAILED, {}};
    if (snapshot.GripperStatus[gripper] == 2)
        return {RetCode::CONTROLLER_ERROR, {}};

    const float normalized = std::clamp(
        snapshot.GripperJointPos[gripper][0], 0.0F, 1.0F);
    const uint16_t position = static_cast<uint16_t>(std::lround(
        normalized * static_cast<float>(CUARM_GRIPPER_POSITION_MAX)));
    return {RetCode::SUCCESS,
            {{"position", position},
             {"force", snapshot.GripperForce[gripper]},
             {"speed", snapshot.GripperSpeed[gripper]}}};
}

}  // namespace dual_arm_v2_2_sdk

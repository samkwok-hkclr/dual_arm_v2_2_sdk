/**
 * @file test_dual_arm_100hz_follow.cpp
 * @brief 左右臂各自 100 Hz 高跟随读写测试
 *
 * 对齐 ros2_control JointGroupBase 的调用方式：
 *   read : get_joint / get_joint_velocity / get_group_state
 *   write: move_joint(...) 或 set_joint_target(...) + flush_joint_command(...)
 *
 * 轨迹：从当前姿态平滑偏移到目标（相对起始角 +deg）；到位后双臂同时回零。
 *
 * 用法（在 build/bin/ 目录下）：
 *   ./test_dual_arm_100hz_follow              # 交互：left -> right -> both
 *   ./test_dual_arm_100hz_follow left
 *   ./test_dual_arm_100hz_follow both --left 20 --right 30
 *   ./test_dual_arm_100hz_follow all --left 10,20,30,40,50,60,70 --right 20
 *   ./test_dual_arm_100hz_follow both-flush   # set_joint_target + flush @100 Hz
 *
 * 目标偏移（可选，单位 deg，相对 stage 起始姿态）：
 *   --left  <deg>              左臂各轴相同偏移
 *   --left  <d1,d2,...,dN>     左臂逐轴偏移（N = 关节数，通常 7）
 *   --right <deg> | <d1,...>   右臂同上
 *   未指定时：left 模式默认 J1..J7 为 +10..70；其余模式默认各轴 +20。
 */

#include "control_api.h"
#include "example_common.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using dual_arm_v2_2_sdk::ControlApi;
using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;
using example::default_config_path;
using example::kDegToRad;
using example::kPi;
using example::kRadToDeg;
using example::ret_code_name;
using example::wait_for_enter;
using example::wait_for_joint_state;

namespace {

constexpr int kControlHz = 100;
constexpr int kPeriodMs = 1000 / kControlHz;
constexpr float kDurationSeconds = 8.0F;
constexpr float kLeftAmplitudeDeg = 20.0F;
constexpr float kRightAmplitudeDeg = 20.0F;
// left-only mode: per-joint ramp targets (J1..J7) in deg.
constexpr std::array<float, 7> kLeftPerJointAmplitudeDeg = {
    10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F};
constexpr float kMaxSpeedDegS = 30.0F;
constexpr float kGuardVelocityDegS = 35.0F;
constexpr float kReachFraction = 0.90F;
constexpr int kDebugEveryN = 50;  // 100 Hz 下约 0.5 s 打一行
constexpr int kHoldAtTargetSeconds = 2;
constexpr int kReturnTimeoutSeconds = 20;
constexpr uint8_t kTrajectoryMode = 2;
constexpr uint16_t kRadio = 999;

struct Sample {
    float cmd_offset_deg = 0.0F;
    float mean_act_offset_deg = 0.0F;
    float mean_lag_deg = 0.0F;
    float mean_abs_vel_deg_s = 0.0F;
    std::vector<float> act_offset_deg;
};

struct ArmStats {
    Sample last;
    double sum_abs_lag = 0.0;
    double sum_abs_vel = 0.0;
    float max_abs_vel = 0.0F;
    float max_abs_lag = 0.0F;
    int samples = 0;
    int read_fail = 0;
    int write_fail = 0;
    float t_cmd_90 = -1.0F;
    float t_act_90 = -1.0F;
    double sum_cycle_ms = 0.0;
    int cycles = 0;
};

struct ArmContext {
    Group group = Group::LEFT_ARM;
    const char* name = "LEFT";
    float amplitude_deg = 0.0F;
    std::vector<float> amplitude_deg_per_joint;
    std::vector<float> start;
    std::vector<float> cmd;
    std::vector<float> pos;
    std::vector<float> vel;
    ArmStats stats;
};

struct TargetOverrides {
    std::optional<std::vector<float>> left_deg;
    std::optional<std::vector<float>> right_deg;
};

struct ParsedArgs {
    std::string mode;
    TargetOverrides targets;
    bool show_help = false;
};

std::vector<float> default_left_offsets_deg(const std::string& stage) {
    if (stage == "left") {
        return {kLeftPerJointAmplitudeDeg.begin(), kLeftPerJointAmplitudeDeg.end()};
    }
    return {kLeftAmplitudeDeg};
}

std::vector<float> default_right_offsets_deg(const std::string& /*stage*/) {
    return {kRightAmplitudeDeg};
}

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [mode] [--left <spec>] [--right <spec>]\n"
              << "  mode: left | right | both | both-burst | both-thread | both-flush | all\n"
              << "        (default: interactive left -> right -> both)\n"
              << "  --left, --right: offset in deg from stage start pose.\n"
              << "        One number  -> same offset on every joint.\n"
              << "        Comma list  -> per-joint offsets (count must match arm).\n"
              << "  Defaults when omitted:\n"
              << "        left mode:  L +[10,20,30,40,50,60,70] deg\n"
              << "        other modes: L/R +20 deg/joint\n"
              << "  Examples:\n"
              << "    " << prog << " left --left 15\n"
              << "    " << prog << " both --left 20 --right 30\n"
              << "    " << prog << " all --left 10,20,30,40,50,60,70 --right 20\n"
              << "    " << prog << " both-flush --left 20 --right 20\n";
}

bool parse_offset_spec(const std::string& text, std::vector<float>& out,
                       std::string& err) {
    out.clear();
    std::string token;
    for (char ch : text) {
        if (ch == ',') {
            if (token.empty()) {
                err = "empty value in offset list '" + text + "'";
                return false;
            }
            try {
                out.push_back(std::stof(token));
            } catch (const std::exception&) {
                err = "invalid number '" + token + "' in '" + text + "'";
                return false;
            }
            token.clear();
        } else if (ch != ' ') {
            token.push_back(ch);
        }
    }
    if (token.empty()) {
        err = "empty offset spec";
        return false;
    }
    try {
        out.push_back(std::stof(token));
    } catch (const std::exception&) {
        err = "invalid number '" + token + "' in '" + text + "'";
        return false;
    }
    return true;
}

bool parse_args(int argc, char** argv, ParsedArgs& out, std::string& err) {
    out = ParsedArgs{};
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            out.show_help = true;
            return true;
        }
        if (arg == "--left" || arg == "-l") {
            if (i + 1 >= argc) {
                err = arg + " requires a value";
                return false;
            }
            std::vector<float> values;
            if (!parse_offset_spec(argv[++i], values, err)) return false;
            out.targets.left_deg = std::move(values);
            continue;
        }
        if (arg == "--right" || arg == "-r") {
            if (i + 1 >= argc) {
                err = arg + " requires a value";
                return false;
            }
            std::vector<float> values;
            if (!parse_offset_spec(argv[++i], values, err)) return false;
            out.targets.right_deg = std::move(values);
            continue;
        }
        if (!arg.empty() && arg[0] == '-') {
            err = "unknown option '" + arg + "'";
            return false;
        }
        if (!out.mode.empty()) {
            err = "unexpected argument '" + arg + "'";
            return false;
        }
        out.mode = arg;
    }
    return true;
}

bool resolve_offsets(const std::optional<std::vector<float>>& user,
                     const std::vector<float>& stage_default,
                     std::size_t joint_count, std::vector<float>& out,
                     std::string& err, const char* arm_label) {
    const std::vector<float>& src =
        user ? *user : stage_default;
    if (src.empty()) {
        err = std::string(arm_label) + " offset list is empty";
        return false;
    }
    if (src.size() == 1) {
        out.assign(joint_count, src.front());
        return true;
    }
    if (src.size() != joint_count) {
        err = std::string(arm_label) + " offset count must be 1 or " +
              std::to_string(joint_count) + ", got " + std::to_string(src.size());
        return false;
    }
    out = src;
    return true;
}

void apply_offsets_to_arm(ArmContext& arm, const std::vector<float>& offsets_deg) {
    const bool uniform = std::all_of(
        offsets_deg.begin(), offsets_deg.end(),
        [&](float v) { return std::abs(v - offsets_deg.front()) < 1e-4F; });
    if (uniform) {
        arm.amplitude_deg = offsets_deg.front();
        arm.amplitude_deg_per_joint.clear();
    } else {
        arm.amplitude_deg_per_joint = offsets_deg;
        arm.amplitude_deg =
            *std::max_element(offsets_deg.begin(), offsets_deg.end());
    }
}

std::string format_deg_list(const std::vector<float>& values) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        oss << values[i];
        if (i + 1 < values.size()) oss << ", ";
    }
    oss << "]";
    return oss.str();
}

std::string format_offset_desc(const std::vector<float>& offsets_deg) {
    if (offsets_deg.empty()) return "no offset";
    const bool uniform = std::all_of(
        offsets_deg.begin(), offsets_deg.end(),
        [&](float v) { return std::abs(v - offsets_deg.front()) < 1e-4F; });
    if (uniform) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(0) << "+" << offsets_deg.front()
            << " deg/joint";
        return oss.str();
    }
    return "+deg " + format_deg_list(offsets_deg);
}

std::vector<float> offset_target(const std::vector<float>& start, float amplitude_deg,
                                 float scale) {
    std::vector<float> out = start;
    const float delta = amplitude_deg * kDegToRad * scale;
    for (float& joint : out) joint += delta;
    return out;
}

std::vector<float> offset_target_per_joint(const std::vector<float>& start,
                                           const std::vector<float>& amplitude_deg,
                                           float scale) {
    std::vector<float> out = start;
    const std::size_t n = std::min(out.size(), amplitude_deg.size());
    for (std::size_t i = 0; i < n; ++i)
        out[i] += amplitude_deg[i] * kDegToRad * scale;
    return out;
}

void print_pose(const char* label, const std::vector<float>& joints) {
    std::cout << "  " << label << " (deg): [";
    std::cout << std::fixed << std::setprecision(2);
    for (std::size_t i = 0; i < joints.size(); ++i) {
        std::cout << joints[i] * kRadToDeg;
        if (i + 1 < joints.size()) std::cout << ", ";
    }
    std::cout << "]\n";
}

std::string format_arm_target_label(const ArmContext& arm) {
    if (!arm.amplitude_deg_per_joint.empty())
        return "target +deg " + format_deg_list(arm.amplitude_deg_per_joint);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(0)
        << "target +" << arm.amplitude_deg << " deg/joint";
    return oss.str();
}

std::string describe_stage_motion(const std::string& name, bool use_left,
                                  bool use_right,
                                  const std::vector<float>& left_offsets,
                                  const std::vector<float>& right_offsets) {
    std::ostringstream oss;
    oss << "8 s high-follow: ";
    if (name == "left") {
        oss << "@100 Hz left " << format_offset_desc(left_offsets)
            << "; right/head held.";
    } else if (name == "right") {
        oss << "@100 Hz right " << format_offset_desc(right_offsets)
            << "; left/head held.";
    } else if (name == "both") {
        oss << "staggered move_joint (left, 10 ms gap, right); L "
            << format_offset_desc(left_offsets) << ", R "
            << format_offset_desc(right_offsets) << " (~50 Hz/arm).";
    } else if (name == "both-burst") {
        oss << "burst left then right in one cycle; second UDP packet usually "
               "dropped (first arm tracks); L "
            << format_offset_desc(left_offsets) << ", R "
            << format_offset_desc(right_offsets) << ".";
    } else if (name == "both-thread") {
        oss << "dual-thread 50 Hz each; L " << format_offset_desc(left_offsets)
            << ", R " << format_offset_desc(right_offsets) << ".";
    } else if (name == "both-flush") {
        oss << "@100 Hz set_joint_target + flush_joint_command; L "
            << format_offset_desc(left_offsets) << ", R "
            << format_offset_desc(right_offsets) << ".";
    } else if (name == "all") {
        oss << "@100 Hz Group::ALL; L " << format_offset_desc(left_offsets)
            << ", R " << format_offset_desc(right_offsets) << " in one packet.";
    } else {
        if (use_left) oss << "L " << format_offset_desc(left_offsets);
        if (use_left && use_right) oss << "; ";
        if (use_right) oss << "R " << format_offset_desc(right_offsets);
    }
    return oss.str();
}

bool enable_arm(ControlApi& api, Group group, bool enable) {
    const RetCode code = api.set_group(group, enable);
    std::cout << "set_group(" << example::group_name(group) << ", "
              << (enable ? "enable" : "disable") << "): "
              << ret_code_name(code) << '\n';
    return code == RetCode::SUCCESS;
}

bool configure_speed(ControlApi& api, Group group, std::size_t joint_count) {
    const std::vector<float> speed(joint_count, kMaxSpeedDegS * kDegToRad);
    const auto applied = api.set_joint_max_speed(group, speed);
    if (applied.size() != speed.size()) {
        std::cerr << "set_joint_max_speed(" << example::group_name(group)
                  << ") failed\n";
        return false;
    }
    return true;
}

void read_arm(ControlApi& api, ArmContext& arm) {
    auto [pos_ret, positions] = api.get_joint(arm.group);
    auto [vel_ret, velocities] = api.get_joint_velocity(arm.group);
    auto [st_ret, states] = api.get_group_state(arm.group);
    (void)states;

    if (pos_ret != RetCode::SUCCESS || vel_ret != RetCode::SUCCESS ||
        st_ret != RetCode::SUCCESS) {
        ++arm.stats.read_fail;
        return;
    }
    arm.pos = std::move(positions);
    arm.vel = std::move(velocities);
}

void write_arm(ControlApi& api, ArmContext& arm) {
    const RetCode ret = api.move_joint(arm.group, arm.cmd, true, kTrajectoryMode,
                                       kRadio);
    if (ret != RetCode::SUCCESS) ++arm.stats.write_fail;
}

bool flush_coalesced_targets(ControlApi& api, const ArmContext& left,
                               const ArmContext& right, ArmStats& left_stats,
                               ArmStats& right_stats) {
    RetCode ret = api.set_joint_target(Group::LEFT_ARM, left.cmd);
    if (ret != RetCode::SUCCESS) {
        ++left_stats.write_fail;
        ++right_stats.write_fail;
        return false;
    }
    ret = api.set_joint_target(Group::RIGHT_ARM, right.cmd);
    if (ret != RetCode::SUCCESS) {
        ++left_stats.write_fail;
        ++right_stats.write_fail;
        return false;
    }
    ret = api.flush_joint_command(true, kTrajectoryMode, kRadio);
    if (ret != RetCode::SUCCESS) {
        ++left_stats.write_fail;
        ++right_stats.write_fail;
        return false;
    }
    return true;
}

void update_stats(ArmContext& arm, float elapsed) {
    if (arm.pos.empty() || arm.cmd.empty() || arm.start.empty()) return;

    const std::size_t n = std::min({arm.pos.size(), arm.cmd.size(), arm.start.size()});
    Sample s;
    s.act_offset_deg.resize(n);
    float sum_act_offset = 0.0F;
    float sum_lag = 0.0F;
    float sum_vel = 0.0F;
    for (std::size_t i = 0; i < n; ++i) {
        const float cmd_off = (arm.cmd[i] - arm.start[i]) * kRadToDeg;
        const float act_off = (arm.pos[i] - arm.start[i]) * kRadToDeg;
        s.act_offset_deg[i] = act_off;
        s.cmd_offset_deg = cmd_off;
        sum_act_offset += act_off;
        sum_lag += std::abs(cmd_off - act_off);
        if (i < arm.vel.size()) sum_vel += std::abs(arm.vel[i] * kRadToDeg);
    }
    const float count = static_cast<float>(n);
    s.mean_act_offset_deg = sum_act_offset / count;
    s.mean_lag_deg = sum_lag / count;
    s.mean_abs_vel_deg_s = n <= arm.vel.size() ? sum_vel / count : 0.0F;
    arm.stats.last = std::move(s);

    const float target_90 = arm.amplitude_deg * kReachFraction;
    if (arm.stats.t_cmd_90 < 0.0F && arm.stats.last.cmd_offset_deg >= target_90)
        arm.stats.t_cmd_90 = elapsed;
    if (arm.stats.t_act_90 < 0.0F && arm.stats.last.mean_act_offset_deg >= target_90)
        arm.stats.t_act_90 = elapsed;

    if (elapsed < 0.3F) return;
    arm.stats.sum_abs_lag += arm.stats.last.mean_lag_deg;
    arm.stats.sum_abs_vel += arm.stats.last.mean_abs_vel_deg_s;
    arm.stats.max_abs_vel =
        std::max(arm.stats.max_abs_vel, arm.stats.last.mean_abs_vel_deg_s);
    arm.stats.max_abs_lag =
        std::max(arm.stats.max_abs_lag, arm.stats.last.mean_lag_deg);
    ++arm.stats.samples;
}

bool over_velocity(const ArmContext& arm) {
    for (float v : arm.vel) {
        if (std::abs(v) > kGuardVelocityDegS * kDegToRad) return true;
    }
    return false;
}

float progress_ratio(const ArmContext& arm) {
    if (arm.amplitude_deg <= 1e-6F) return 0.0F;
    return arm.stats.last.mean_act_offset_deg / arm.amplitude_deg;
}

void print_live_line(const char* stage, float elapsed, double cycle_ms,
                     const ArmContext* left, const ArmContext* right) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "[" << stage << "] t=" << elapsed << "s cycle=" << cycle_ms << "ms";
    auto append = [&](const ArmContext* arm) {
        if (!arm) return;
        oss << "\n  " << arm->name << " " << format_arm_target_label(*arm)
            << std::setprecision(2)
            << "  progress=" << (100.0F * progress_ratio(*arm)) << "%"
            << "  cmd=" << arm->stats.last.cmd_offset_deg
            << "  act=" << arm->stats.last.mean_act_offset_deg
            << "  lag=" << arm->stats.last.mean_lag_deg
            << "  vel=" << arm->stats.last.mean_abs_vel_deg_s
            << "\n      d=" << format_deg_list(arm->stats.last.act_offset_deg);
    };
    append(left);
    append(right);
    std::cout << oss.str() << '\n';
}

void print_summary(const char* stage, const ArmContext* left,
                   const ArmContext* right) {
    std::cout << "\n======== " << stage << " summary (all joints) ========\n";
    auto row = [](const ArmContext* arm) {
        if (!arm) return;
        const ArmStats& s = arm->stats;
        const float mean_lag =
            s.samples > 0 ? static_cast<float>(s.sum_abs_lag / s.samples) : 0.0F;
        const float mean_vel =
            s.samples > 0 ? static_cast<float>(s.sum_abs_vel / s.samples) : 0.0F;
        const float mean_cycle =
            s.cycles > 0 ? static_cast<float>(s.sum_cycle_ms / s.cycles) : 0.0F;
        const float final_progress = 100.0F * progress_ratio(*arm);
        std::cout << std::fixed << std::setprecision(2)
                  << "  " << arm->name << " " << format_arm_target_label(*arm)
                  << "  final_progress=" << final_progress << "%"
                  << "  (mean act / max target; per-joint mode is approximate)\n"
                  << "       mean|lag|=" << mean_lag << " deg"
                  << "  max|lag|=" << s.max_abs_lag << " deg"
                  << "  mean|vel|=" << mean_vel << " deg/s"
                  << "  peak|vel|=" << s.max_abs_vel << " deg/s\n"
                  << "       t_cmd_90=" << s.t_cmd_90 << " s"
                  << "  t_act_90=" << s.t_act_90 << " s"
                  << "  cycle=" << mean_cycle << " ms"
                  << "  read_fail=" << s.read_fail
                  << "  write_fail=" << s.write_fail << '\n'
                  << "       final d="
                  << format_deg_list(s.last.act_offset_deg) << '\n';
    };
    row(left);
    row(right);
    if (left && right && left->stats.samples > 0 && right->stats.samples > 0) {
        const float left_progress = progress_ratio(*left);
        const float right_progress = progress_ratio(*right);
        std::cout << std::setprecision(2)
                  << "  progress L/R = "
                  << (right_progress > 1e-3F ? left_progress / right_progress
                                             : 0.0F)
                  << "   (<< 1 means left finished less of its move)\n";
        const float left_lag_norm =
            left->stats.max_abs_lag / std::max(left->amplitude_deg, 1e-3F);
        const float right_lag_norm =
            right->stats.max_abs_lag / std::max(right->amplitude_deg, 1e-3F);
        std::cout << "  lag/amp  L/R = "
                  << (right_lag_norm > 1e-3F ? left_lag_norm / right_lag_norm
                                             : 0.0F)
                  << "   (>> 1 means left lags more than amplitude explains)\n";
    }
    std::cout << "==========================================\n";
}

bool wait_until_pose(ControlApi& api, Group group,
                     const std::vector<float>& target) {
    constexpr float kTol = 0.5F * kDegToRad;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(kReturnTimeoutSeconds);
    int stable = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        auto [code, pos] = api.get_joint(group);
        if (code == RetCode::SUCCESS && pos.size() == target.size()) {
            float max_err = 0.0F;
            for (std::size_t i = 0; i < pos.size(); ++i)
                max_err = std::max(max_err, std::abs(pos[i] - target[i]));
            if (max_err <= kTol) {
                if (++stable >= 8) return true;
            } else {
                stable = 0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

bool go_home_both(ControlApi& api, const std::vector<float>& left_template,
                  const std::vector<float>& right_template,
                  const std::vector<float>& head_now) {
    std::cout << "Both arms going home (all arm joints -> 0, head unchanged)...\n";
    const std::vector<float> left_zero(left_template.size(), 0.0F);
    const std::vector<float> right_zero(right_template.size(), 0.0F);
    std::vector<float> all;
    all.insert(all.end(), left_zero.begin(), left_zero.end());
    all.insert(all.end(), head_now.begin(), head_now.end());
    all.insert(all.end(), right_zero.begin(), right_zero.end());
    if (api.move_joint(Group::ALL, all, false) != RetCode::SUCCESS) {
        std::cerr << "go-home move_joint(ALL) failed\n";
        return false;
    }
    const bool left_ok = wait_until_pose(api, Group::LEFT_ARM, left_zero);
    const bool right_ok = wait_until_pose(api, Group::RIGHT_ARM, right_zero);
    if (!left_ok || !right_ok) {
        std::cerr << "Timed out going home "
                  << "(left=" << (left_ok ? "ok" : "timeout")
                  << ", right=" << (right_ok ? "ok" : "timeout") << ")\n";
        return false;
    }
    std::cout << "Home reached.\n";
    return true;
}

bool hold_current(ControlApi& api, bool use_left, bool use_right) {
    if (use_left) {
        auto [code, pos] = api.get_joint(Group::LEFT_ARM);
        if (code == RetCode::SUCCESS)
            api.move_joint(Group::LEFT_ARM, pos, true, 0, 0);
    }
    if (use_right) {
        auto [code, pos] = api.get_joint(Group::RIGHT_ARM);
        if (code == RetCode::SUCCESS)
            api.move_joint(Group::RIGHT_ARM, pos, true, 0, 0);
    }
    return true;
}

enum class DriveMode {
    Sequential,
    SequentialBurst,
    Threaded,
    CoalescedFlush,
    GroupAll
};

bool run_stage(ControlApi& api, const char* stage, DriveMode mode,
               bool use_left, bool use_right,
               const std::vector<float>& left_start,
               const std::vector<float>& right_start,
               const std::vector<float>& head_now,
               const std::vector<float>& left_offsets_deg,
               const std::vector<float>& right_offsets_deg) {
    ArmContext left;
    left.group = Group::LEFT_ARM;
    left.name = "L";
    left.start = left_start;
    left.cmd = left_start;
    if (use_left) apply_offsets_to_arm(left, left_offsets_deg);

    ArmContext right;
    right.group = Group::RIGHT_ARM;
    right.name = "R";
    right.start = right_start;
    right.cmd = right_start;
    if (use_right) apply_offsets_to_arm(right, right_offsets_deg);

    std::atomic<bool> stop{false};
    std::atomic<bool> safety{false};
    std::mutex print_mutex;
    int debug_tick = 0;

    auto fill_cmd = [](ArmContext& arm, float progress) {
        const float smooth = 0.5F - 0.5F * std::cos(kPi * progress);
        if (!arm.amplitude_deg_per_joint.empty()) {
            arm.cmd = offset_target_per_joint(arm.start, arm.amplitude_deg_per_joint,
                                              smooth);
        } else {
            arm.cmd = offset_target(arm.start, arm.amplitude_deg, smooth);
        }
    };

    auto tick_one = [&](ArmContext& arm, float elapsed, float progress) {
        read_arm(api, arm);
        if (over_velocity(arm)) {
            safety.store(true);
            return;
        }
        fill_cmd(arm, progress);
        write_arm(api, arm);
        update_stats(arm, elapsed);
    };

    std::cout << "\n==> " << stage << "  " << kDurationSeconds << " s,";
    if (use_left) {
        std::cout << " L " << format_offset_desc(left_offsets_deg) << ',';
    } else {
        std::cout << " L held,";
    }
    if (use_right) {
        std::cout << " R " << format_offset_desc(right_offsets_deg) << ',';
    } else {
        std::cout << " R held,";
    }
    std::cout << " follow=true mode=" << static_cast<int>(kTrajectoryMode)
              << " radio=" << kRadio << '\n';
    if (mode == DriveMode::Sequential && use_left && use_right) {
        std::cout << "  Write pattern: left, wait 10ms, right, wait 10ms.\n"
                     "  rt_control only receives 1 UDP packet / 10ms; this gap "
                     "lets both packets be applied.\n"
                     "  Each single-arm move_joint() still hold_unselected_groups() "
                     "the other arm to current pose.\n";
    } else if (mode == DriveMode::SequentialBurst && use_left && use_right) {
        std::cout << "  Write pattern: left then right in the same 0.1ms "
                     "(BURST). The second packet is usually dropped, so only "
                     "the first arm moves.\n";
    } else if (mode == DriveMode::Threaded) {
        std::cout << "  Two threads, 50 Hz each, right delayed by 10ms.\n";
    } else if (mode == DriveMode::CoalescedFlush) {
        std::cout << "  Each 10 ms: set_joint_target(L), set_joint_target(R), "
                     "flush_joint_command().\n"
                     "  Unselected groups (head) held from feedback at flush.\n";
    } else if (mode == DriveMode::GroupAll) {
        std::cout << "  One move_joint(ALL) per 10ms; both arm targets in the "
                     "same packet.\n";
    }

    if (mode == DriveMode::CoalescedFlush) api.reset_joint_target_cache();

    const auto start_time = std::chrono::steady_clock::now();
    auto next_wakeup = start_time;
    const int thread_period_ms =
        (mode == DriveMode::Threaded) ? (2 * kPeriodMs) : kPeriodMs;

    if (mode == DriveMode::Threaded && use_left && use_right) {
        auto worker = [&](ArmContext& arm, int delay_ms) {
            if (delay_ms > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            auto next = std::chrono::steady_clock::now();
            while (!stop.load() && !safety.load()) {
                const auto now = std::chrono::steady_clock::now();
                const float elapsed =
                    std::chrono::duration<float>(now - start_time).count();
                if (elapsed >= kDurationSeconds) break;
                const float progress =
                    std::min(elapsed / kDurationSeconds, 1.0F);
                const auto cycle_begin = std::chrono::steady_clock::now();
                tick_one(arm, elapsed, progress);
                const double cycle_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - cycle_begin)
                        .count();
                arm.stats.sum_cycle_ms += cycle_ms;
                ++arm.stats.cycles;
                next += std::chrono::milliseconds(thread_period_ms);
                std::this_thread::sleep_until(next);
            }
        };

        std::thread left_thread(worker, std::ref(left), 0);
        std::thread right_thread(worker, std::ref(right), kPeriodMs);

        while (!safety.load()) {
            const float elapsed = std::chrono::duration<float>(
                                      std::chrono::steady_clock::now() -
                                      start_time)
                                      .count();
            if (elapsed >= kDurationSeconds) break;
            if (++debug_tick >= kDebugEveryN) {
                debug_tick = 0;
                std::lock_guard<std::mutex> lock(print_mutex);
                print_live_line(stage, elapsed, thread_period_ms, &left, &right);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(kPeriodMs));
        }
        stop.store(true);
        left_thread.join();
        right_thread.join();
    } else {
        while (!safety.load()) {
            const auto now = std::chrono::steady_clock::now();
            const float elapsed =
                std::chrono::duration<float>(now - start_time).count();
            if (elapsed >= kDurationSeconds) break;
            const float progress = std::min(elapsed / kDurationSeconds, 1.0F);
            const auto cycle_begin = std::chrono::steady_clock::now();

            if (mode == DriveMode::CoalescedFlush && use_left && use_right) {
                read_arm(api, left);
                read_arm(api, right);
                if (over_velocity(left) || over_velocity(right)) {
                    safety.store(true);
                    break;
                }
                fill_cmd(left, progress);
                fill_cmd(right, progress);
                flush_coalesced_targets(api, left, right, left.stats, right.stats);
                update_stats(left, elapsed);
                update_stats(right, elapsed);
                const double cycle_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - cycle_begin)
                        .count();
                left.stats.sum_cycle_ms += cycle_ms;
                right.stats.sum_cycle_ms += cycle_ms;
                ++left.stats.cycles;
                ++right.stats.cycles;
                if (++debug_tick >= kDebugEveryN) {
                    debug_tick = 0;
                    print_live_line(stage, elapsed, cycle_ms, &left, &right);
                }
                next_wakeup += std::chrono::milliseconds(kPeriodMs);
                std::this_thread::sleep_until(next_wakeup);
            } else if (mode == DriveMode::GroupAll && use_left && use_right) {
                read_arm(api, left);
                read_arm(api, right);
                if (over_velocity(left) || over_velocity(right)) {
                    safety.store(true);
                    break;
                }
                fill_cmd(left, progress);
                fill_cmd(right, progress);
                std::vector<float> all;
                all.insert(all.end(), left.cmd.begin(), left.cmd.end());
                all.insert(all.end(), head_now.begin(), head_now.end());
                all.insert(all.end(), right.cmd.begin(), right.cmd.end());
                const RetCode ret =
                    api.move_joint(Group::ALL, all, true, kTrajectoryMode,
                                   kRadio);
                if (ret != RetCode::SUCCESS) {
                    ++left.stats.write_fail;
                    ++right.stats.write_fail;
                }
                update_stats(left, elapsed);
                update_stats(right, elapsed);
                if (use_left) {
                    left.stats.sum_cycle_ms +=
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - cycle_begin)
                            .count();
                    ++left.stats.cycles;
                }
                if (use_right) {
                    right.stats.sum_cycle_ms +=
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - cycle_begin)
                            .count();
                    ++right.stats.cycles;
                }
                if (++debug_tick >= kDebugEveryN) {
                    debug_tick = 0;
                    print_live_line(stage, elapsed, kPeriodMs, &left, &right);
                }
                next_wakeup += std::chrono::milliseconds(kPeriodMs);
                std::this_thread::sleep_until(next_wakeup);
            } else if (use_left && use_right &&
                       mode == DriveMode::Sequential) {
                // 10ms between the two independent move_joint() calls so
                // rt_control can receive both. Left is written first; the
                // following right packet hold_unselected_groups() the left
                // arm back to its current pose — this is the "left slow,
                // right normal" overwrite.
                read_arm(api, left);
                read_arm(api, right);
                if (over_velocity(left) || over_velocity(right)) {
                    safety.store(true);
                    break;
                }
                fill_cmd(left, progress);
                write_arm(api, left);
                update_stats(left, elapsed);
                left.stats.sum_cycle_ms +=
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - cycle_begin)
                        .count();
                ++left.stats.cycles;

                next_wakeup += std::chrono::milliseconds(kPeriodMs);
                std::this_thread::sleep_until(next_wakeup);
                if (safety.load()) break;

                const auto right_now = std::chrono::steady_clock::now();
                const float right_elapsed =
                    std::chrono::duration<float>(right_now - start_time).count();
                if (right_elapsed >= kDurationSeconds) break;
                const float right_progress =
                    std::min(right_elapsed / kDurationSeconds, 1.0F);
                const auto right_cycle_begin = std::chrono::steady_clock::now();
                read_arm(api, left);
                read_arm(api, right);
                if (over_velocity(left) || over_velocity(right)) {
                    safety.store(true);
                    break;
                }
                fill_cmd(right, right_progress);
                write_arm(api, right);
                update_stats(left, right_elapsed);
                update_stats(right, right_elapsed);
                right.stats.sum_cycle_ms +=
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - right_cycle_begin)
                        .count();
                ++right.stats.cycles;

                if (++debug_tick >= (kDebugEveryN / 2)) {
                    debug_tick = 0;
                    print_live_line(stage, right_elapsed, 2 * kPeriodMs,
                                    &left, &right);
                }
                next_wakeup += std::chrono::milliseconds(kPeriodMs);
                std::this_thread::sleep_until(next_wakeup);
            } else {
                if (use_left) read_arm(api, left);
                if (use_right) read_arm(api, right);
                if ((use_left && over_velocity(left)) ||
                    (use_right && over_velocity(right))) {
                    safety.store(true);
                    break;
                }
                if (use_left) {
                    fill_cmd(left, progress);
                    write_arm(api, left);
                    update_stats(left, elapsed);
                }
                if (use_right) {
                    fill_cmd(right, progress);
                    write_arm(api, right);
                    update_stats(right, elapsed);
                }

                const double cycle_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - cycle_begin)
                        .count();
                if (use_left) {
                    left.stats.sum_cycle_ms += cycle_ms;
                    ++left.stats.cycles;
                }
                if (use_right) {
                    right.stats.sum_cycle_ms += cycle_ms;
                    ++right.stats.cycles;
                }

                if (++debug_tick >= kDebugEveryN) {
                    debug_tick = 0;
                    print_live_line(stage, elapsed, cycle_ms,
                                    use_left ? &left : nullptr,
                                    use_right ? &right : nullptr);
                }

                next_wakeup += std::chrono::milliseconds(kPeriodMs);
                std::this_thread::sleep_until(next_wakeup);
            }
        }
    }

    if (safety.load()) {
        std::cerr << "\nSAFETY STOP: joint velocity exceeded "
                  << kGuardVelocityDegS << " deg/s. Holding current pose.\n";
        hold_current(api, use_left, use_right);
        print_summary(stage, use_left ? &left : nullptr,
                      use_right ? &right : nullptr);
        return false;
    }

    print_summary(stage, use_left ? &left : nullptr,
                  use_right ? &right : nullptr);

    std::cout << "Hold " << kHoldAtTargetSeconds
              << "s at the offset pose, then both arms go home together.\n";
    std::this_thread::sleep_for(std::chrono::seconds(kHoldAtTargetSeconds));
    return go_home_both(api, left_start, right_start, head_now);
}

}  // namespace

int main(int argc, char** argv) {
    ParsedArgs args;
    std::string parse_err;
    if (!parse_args(argc, argv, args, parse_err)) {
        std::cerr << parse_err << '\n';
        print_usage(argv[0]);
        return 1;
    }
    if (args.show_help) {
        print_usage(argv[0]);
        return 0;
    }

    const bool interactive = args.mode.empty();
    std::array<std::string, 5> stages{};
    int stage_count = 0;
    auto add = [&](const std::string& name) {
        if (stage_count < static_cast<int>(stages.size()))
            stages[static_cast<std::size_t>(stage_count++)] = name;
    };

    if (interactive) {
        add("left");
        add("right");
        add("both");
    } else if (args.mode == "left" || args.mode == "right" ||
               args.mode == "both" || args.mode == "both-burst" ||
               args.mode == "both-thread" || args.mode == "both-flush" ||
               args.mode == "all") {
        add(args.mode);
    } else {
        std::cerr << "Unknown mode '" << args.mode << "'\n";
        print_usage(argv[0]);
        return 1;
    }

    try {
        ControlApi api(default_config_path());
        std::vector<float> left_now;
        std::vector<float> right_now;
        std::vector<float> head_now;
        std::cout << "Waiting for rt_control state...\n";
        if (!wait_for_joint_state(api, Group::LEFT_ARM, left_now) ||
            !wait_for_joint_state(api, Group::RIGHT_ARM, right_now) ||
            !wait_for_joint_state(api, Group::HEAD, head_now)) {
            std::cerr << "No rt_control state within 3 seconds.\n";
            return 1;
        }

        std::cout << std::fixed << std::setprecision(2)
                  << "Current pose:\n";
        print_pose("LEFT ", left_now);
        print_pose("RIGHT", right_now);

        if (args.targets.left_deg) {
            std::cout << "Left target override: "
                      << format_offset_desc(*args.targets.left_deg) << '\n';
        }
        if (args.targets.right_deg) {
            std::cout << "Right target override: "
                      << format_offset_desc(*args.targets.right_deg) << '\n';
        }

        if (!enable_arm(api, Group::LEFT_ARM, true) ||
            !enable_arm(api, Group::RIGHT_ARM, true))
            return 1;
        if (!configure_speed(api, Group::LEFT_ARM, left_now.size()) ||
            !configure_speed(api, Group::RIGHT_ARM, right_now.size()))
            return 1;

        if (!wait_for_enter(
                "First go home (both arms to 0) for a known start pose before the "
                "high-follow test."))
            return 0;
        if (!go_home_both(api, left_now, right_now, head_now)) return 1;

        for (int i = 0; i < stage_count; ++i) {
            const std::string& name = stages[static_cast<std::size_t>(i)];
            bool use_left = false;
            bool use_right = false;
            DriveMode mode = DriveMode::Sequential;
            const char* label = name.c_str();

            if (name == "left") {
                use_left = true;
                label = "LEFT only";
            } else if (name == "right") {
                use_right = true;
                label = "RIGHT only";
            } else if (name == "both") {
                use_left = true;
                use_right = true;
                label = "BOTH staggered (left, 10ms, right, 10ms)";
            } else if (name == "both-burst") {
                use_left = true;
                use_right = true;
                mode = DriveMode::SequentialBurst;
                label = "BOTH burst (left+right in 0.1ms, second packet dropped)";
            } else if (name == "both-thread") {
                use_left = true;
                use_right = true;
                mode = DriveMode::Threaded;
                label = "BOTH threaded (50 Hz each, right delayed 10ms)";
            } else if (name == "both-flush") {
                use_left = true;
                use_right = true;
                mode = DriveMode::CoalescedFlush;
                label = "BOTH coalesced (set_joint_target + flush @100 Hz)";
            } else if (name == "all") {
                use_left = true;
                use_right = true;
                mode = DriveMode::GroupAll;
                label = "BOTH via Group::ALL (control)";
            }

            std::vector<float> left_offsets;
            std::vector<float> right_offsets;
            if (use_left &&
                !resolve_offsets(args.targets.left_deg,
                                 default_left_offsets_deg(name), left_now.size(),
                                 left_offsets, parse_err, "left")) {
                std::cerr << parse_err << '\n';
                return 1;
            }
            if (use_right &&
                !resolve_offsets(args.targets.right_deg,
                                 default_right_offsets_deg(name),
                                 right_now.size(), right_offsets, parse_err,
                                 "right")) {
                std::cerr << parse_err << '\n';
                return 1;
            }

            std::ostringstream prompt;
            prompt << "Stage " << (i + 1) << "/" << stage_count << ": " << label
                   << '\n'
                   << describe_stage_motion(name, use_left, use_right,
                                            left_offsets, right_offsets)
                   << "\nThen hold 2 s and both arms go home together.";
            if (!wait_for_enter(prompt.str().c_str())) return 0;

            if (!wait_for_joint_state(api, Group::LEFT_ARM, left_now) ||
                !wait_for_joint_state(api, Group::RIGHT_ARM, right_now) ||
                !wait_for_joint_state(api, Group::HEAD, head_now)) {
                std::cerr << "Failed to refresh start pose.\n";
                return 1;
            }

            if (!run_stage(api, label, mode, use_left, use_right, left_now,
                           right_now, head_now, left_offsets, right_offsets))
                return 1;
        }

        std::cout << "\nDone. Watch live `progress` / per-joint `d=` during each stage.\n"
                     "Use --left / --right to override per-arm offset targets (deg).\n"
                     "Mode reference:\n"
                     "  left        : left arm only; default L +[10..70] deg.\n"
                     "  right       : right arm only; default R +20 deg/joint.\n"
                     "  both        : staggered packets; left slows (hold overwrite).\n"
                     "  both-burst  : second packet often dropped; first arm tracks.\n"
                     "  both-thread : two threads @50 Hz each, right delayed 10 ms.\n"
                     "  both-flush  : set_joint_target + flush @100 Hz (SDK coalesce).\n"
                     "  all         : both arms sync via Group::ALL @100 Hz.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal SDK error: " << error.what() << '\n';
        return 1;
    }
}

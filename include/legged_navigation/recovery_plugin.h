#ifndef LEGGED_NAVIGATION_RECOVERY_PLUGIN_H
#define LEGGED_NAVIGATION_RECOVERY_PLUGIN_H

#include <cstdint>
#include <string>
#include <string_view>

#include "legged_navigation/relocalization_supervisor.h"

namespace legged_navigation {

// 插件边界：LNR 对 BT 中枢只输出 Advice（候选动作 + 授权建议 + 原因码），
// 不持有执行权。字段语义对齐 fork-spatial-runtime 契约
// （docs/contracts/spatial_state.md 映射表）。
enum class PluginAuthorityLevel : std::uint8_t {
    Evidence,
    Advice,
};

// 平台扩展位：跨平台（四足/轮式/人形/无人机）与环境上下文（电梯/楼梯）
// 的开放槽位；契约字段之外、不影响状态机语义。
struct PlatformHints {
    std::string gait;          // 运动方式提示（如 "trot"、"wheel"）
    std::string environment;   // 环境上下文（如 "elevator"、"stairs"）
    std::uint32_t vendor_flags{0};  // 厂商/平台扩展位

    bool operator==(const PlatformHints&) const = default;
};

// 输入 = 契约 LocalizationHealth 镜像 + 底层传感器标志。
// localization_mode: 0 Normal3D / 1 Degraded2D / 2 Relocalizing /
//                    3 Verifying / 4 Hold（见契约 LocalizationMode）。
struct RecoveryInput {
    std::int64_t timestamp_ms{};
    std::uint8_t localization_mode{0};
    double quality{1.0};
    double pose_jump_m{0.0};
    double alignment_fitness{0.0};
    std::uint32_t healthy_frames{0};
    bool stance_ok{true};
    bool tracking_valid{true};
    bool descriptor_candidate_found{false};
    bool alignment_complete{false};
    bool alignment_success{false};
    PlatformHints hints;
};

// 契约 ExecutionAuthority 镜像：Tracking→Full，恢复中→Suspended，
// SafeStop→Revoked；退化通行模式（外部在输入层给到 Degraded2D）→Restricted。
enum class RecoveryAuthority : std::uint8_t {
    Full,
    Restricted,
    Suspended,
    Revoked,
};

// 契约 ReasonCode 子集：恢复域只用这三个原因码 + None。
enum class RecoveryReason : std::uint8_t {
    None,
    LocalizationLost,
    LocalizationJump,
    RelocalizationTimeout,
};

struct RecoveryOutput {
    RecoveryState previous{RecoveryState::Tracking};
    RecoveryState current{RecoveryState::Tracking};
    RecoveryAction action{RecoveryAction::None};
    RecoveryAuthority authority{RecoveryAuthority::Full};
    RecoveryReason reason{RecoveryReason::None};
    PlatformHints hints;

    [[nodiscard]] bool state_changed() const noexcept {
        return previous != current;
    }
};

// 恢复插件：薄适配层，把契约形状输入喂给 RelocalizationSupervisor，
// 输出映射到执行授权与原因码。状态机本体不变。
class RecoveryPlugin {
public:
    static constexpr std::string_view plugin_id = "lnr-recovery";
    static constexpr std::string_view interface_version = "recovery.v1";
    static constexpr PluginAuthorityLevel authority_level =
        PluginAuthorityLevel::Advice;

    explicit RecoveryPlugin(RecoveryConfig config = {});

    RecoveryOutput update(const RecoveryInput& input);
    void reset(std::int64_t timestamp_ms = 0);

    [[nodiscard]] RecoveryState state() const noexcept;
    [[nodiscard]] static std::string_view name(RecoveryAuthority authority) noexcept;
    // Same snake_case names as fork-spatial-runtime contract::name(ReasonCode).
    [[nodiscard]] static std::string_view name(RecoveryReason reason) noexcept;

private:
    RelocalizationSupervisor supervisor_;
    RecoveryConfig config_;
};

}  // namespace legged_navigation

#endif

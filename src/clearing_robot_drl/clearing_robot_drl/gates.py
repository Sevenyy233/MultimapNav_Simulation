"""独立评估历史的四组可配置收敛判据。"""

import math


METRICS = (
    "reward_mean",
    "success_rate",
    "collision_rate",
    "entropy_mean",
    "yaw_oscillation_amplitude",
    "speed_change_mean",
)


def _usable(entry):
    """! @brief 核对本次评估已完整结束且所有指标有限。"""
    return entry.get("complete") and all(
        isinstance(entry.get(key), (int, float)) and math.isfinite(entry[key])
        for key in METRICS
    )


def convergence(history, window=3, min_success=.9, max_collision=.05,
                reward_relative=.05, success_spread=.03, entropy_drop=.05,
                entropy_spread=.10, entropy_jump=.10, max_oscillation=.04,
                max_speed_change=.1, collision_spread=.03,
                oscillation_spread=.02, speed_change_spread=.03,
                reward_floor=1e-6, reward_trend=.015, quality_trend_fraction=.45,
                entropy_upper_bound=.8):
    """! @brief 在连续完整的最近评估周期上检查四组收敛门限。
    @param history 独立评估指标序列。较早的有效评估提供熵基线。
    @param window 最近连续评估周期数，至少为 2。
    @return 各门限结果及窗口证据；缺失或失败评估会推迟判定。
    """
    valid = [entry for entry in history if _usable(entry)]
    recent = history[-window:] if window >= 2 else []
    ready = (window >= 2 and len(history) >= window + 1 and
             all(_usable(entry) for entry in recent) and
             any(_usable(entry) for entry in history[:-window]))
    result = {
        "ready": ready,
        "valid_evaluations": len(valid),
        "window": window,
        "reward": False,
        "success": False,
        "entropy": False,
        "safety_smoothness": False,
        "converged": False,
    }
    if not ready:
        return result

    def values(key):
        """! @brief 返回连续窗口中的一列有限指标。"""
        return [entry[key] for entry in recent]

    def span(key):
        """! @brief 返回连续窗口内该指标的最大值与最小值之差。"""
        series = values(key)
        return max(series) - min(series)

    def trend(key):
        """! @brief 计算最小二乘趋势斜率，单位为每评估周期的指标变化。"""
        midpoint = (window - 1) / 2
        return (sum((index - midpoint) * value for index, value in enumerate(values(key))) /
                sum((index - midpoint) ** 2 for index in range(window)))

    reward_mean = sum(values("reward_mean")) / window
    reward_scale = max(abs(reward_mean), reward_floor)
    relative = span("reward_mean") / reward_scale
    reward_slope = trend("reward_mean") / reward_scale
    entropy_delta = valid[0]["entropy_mean"] - sum(values("entropy_mean")) / window
    entropy_series = values("entropy_mean")
    entropy_max_jump = max(abs(right - left) for left, right in zip(entropy_series, entropy_series[1:]))
    result.update(
        reward_relative_range=relative,
        reward_slope_per_evaluation=reward_slope,
        success_range=span("success_rate"),
        entropy_decrease=entropy_delta,
        entropy_range=span("entropy_mean"),
        entropy_max_jump=entropy_max_jump,
        collision_range=span("collision_rate"),
        oscillation_range=span("yaw_oscillation_amplitude"),
        speed_change_range=span("speed_change_mean"),
        collision_slope=trend("collision_rate"),
        oscillation_slope=trend("yaw_oscillation_amplitude"),
        speed_change_slope=trend("speed_change_mean"),
    )
    result["reward"] = relative < reward_relative and reward_slope <= reward_trend
    result["success"] = (
        span("success_rate") < success_spread and min(values("success_rate")) >= min_success
    )
    result["entropy"] = (
        entropy_delta >= entropy_drop and max(entropy_series) <= entropy_upper_bound and
        span("entropy_mean") <= entropy_spread and entropy_max_jump <= entropy_jump
    )
    result["safety_smoothness"] = all(
        max(values(key)) <= maximum and span(key) <= spread and
        abs(trend(key)) <= spread * quality_trend_fraction
        for key, maximum, spread in (
            ("collision_rate", max_collision, collision_spread),
            ("yaw_oscillation_amplitude", max_oscillation, oscillation_spread),
            ("speed_change_mean", max_speed_change, speed_change_spread),
        )
    )
    result["converged"] = all(
        result[key] for key in ("reward", "success", "entropy", "safety_smoothness")
    )
    return result

"""以命令确认和实际实体运动组成仿真复位停稳门槛。"""
from dataclasses import dataclass
import math


@dataclass
class StopGate:
    """! @brief 连续仿真时钟内同时确认零执行指令及实体停稳。"""

    request_stamp: float
    dwell: float = .35
    stable_since: float | None = None
    last_sample: float | None = None

    def check(self, sim_stamp, applied_stamp, applied, linear, angular):
        """! @brief 更新一次新鲜样本（线速度 m/s、角速度 rad/s）。
        @return 停稳、当前判定原因。零指令反馈不能代替实测速度。
        """
        if self.last_sample is not None and sim_stamp <= self.last_sample:
            return False, 'waiting_for_sim_time'
        if self.last_sample is not None and sim_stamp - self.last_sample > .4:
            self.stable_since = None
        self.last_sample = sim_stamp
        if not (sim_stamp > self.request_stamp and applied_stamp is not None and
                applied_stamp > self.request_stamp and abs(sim_stamp - applied_stamp) < .45):
            self.stable_since = None
            return False, 'no_fresh_applied_feedback'
        if any(not math.isfinite(value) or abs(value) >= .015 for value in applied):
            self.stable_since = None
            return False, 'drive_command_nonzero'
        if not (math.hypot(*linear) < .035 and math.hypot(*angular) < .04):
            self.stable_since = None
            return False, 'gazebo_body_in_motion'
        if self.stable_since is None:
            self.stable_since = sim_stamp
        ready = sim_stamp - self.stable_since >= self.dwell
        return ready, 'stopped' if ready else 'holding_still'

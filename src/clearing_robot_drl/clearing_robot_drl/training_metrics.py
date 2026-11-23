"""训练指标写入 TensorBoard；累计步数与运行 CSV 一致。"""
import math


class TrainingMetrics:
    """! @brief 维护可追加的事件流，调用方负责生命周期及已有日志高水位检查。"""
    def __init__(self, log_dir, flush_every=50):
        """! @brief 创建事件文件；恢复训练时保留已有文件，不清理旧数据。
        @param log_dir 本次运行的 TensorBoard 日志目录。
        @param flush_every 每隔多少次记录调用主动刷盘。
        """
        if not isinstance(flush_every, int) or flush_every < 1:
            raise ValueError('flush_every 必须是正整数')
        try:
            from torch.utils.tensorboard import SummaryWriter
        except ImportError as exc:
            raise RuntimeError('TensorBoard 不可用；请在运行环境安装 tensorboard') from exc
        self.writer = SummaryWriter(log_dir=str(log_dir), flush_secs=30)
        self.flush_every = flush_every
        self.pending = 0
        self.last_step = -1
        self.closed = False

    def _begin(self, step):
        """! @brief 确认步数在当前会话内单调，允许同一步对应多项事件。"""
        if self.closed:
            raise RuntimeError('TensorBoard writer 已关闭')
        if isinstance(step, bool) or not isinstance(step, int) or step < 0 or step < self.last_step:
            raise ValueError('累计训练步数必须非负且不回退')
        self.last_step = step

    def _scalar(self, tag, value, step):
        """! @brief 可用且有限的数值才写入标量。"""
        if value is None or value == '':
            return
        try:
            number = float(value)
        except (TypeError, ValueError):
            return
        if math.isfinite(number):
            self.writer.add_scalar(tag, number, step)

    def _finish(self):
        """! @brief 定期主动刷盘，减少异常退出时丢失的事件。"""
        self.pending += 1
        if self.pending >= self.flush_every:
            self.flush()

    def step(self, step, entropy=None, losses=None):
        """! @brief 记录当前动作熵及实际执行过的优化损失和温度。
        @param step 累计环境交互步数。
        @param entropy 当前动作分布微分熵的单次估计，可缺省。
        @param losses agent.update 返回的 critic、actor、alpha，可缺省。
        """
        self._begin(step)
        self._scalar('policy/entropy_sample', entropy, step)
        if losses is not None:
            for key, tag in (('critic', 'loss/critic'), ('actor', 'loss/actor'),
                             ('alpha_loss', 'loss/temperature'), ('alpha', 'policy/temperature')):
                self._scalar(tag, losses.get(key), step)
        self._finish()

    def episode(self, step, row):
        """! @brief 记录回合回报、结果与误差；运行错误单独标记。
        @param step 累计环境交互步数，允许多个零动作回合共用一步。
        @param row 回合 CSV 使用的字典，包含 outcome、return 等字段。
        """
        self._begin(step)
        self._scalar('episode/reward', row.get('return'), step)
        self._scalar('episode/entropy_mean', row.get('entropy_mean'), step)
        self._scalar('episode/duration_s', row.get('duration'), step)
        self._scalar('episode/length_steps', row.get('steps'), step)
        outcome = str(row.get('outcome', 'unknown'))
        for name, value in (('success', outcome == 'success'), ('collision', outcome == 'collision'),
                            ('timeout', outcome in ('timeout', 'planning_timeout')),
                            ('error', outcome == 'error')):
            self._scalar('episode/' + name, int(value), step)
        self.writer.add_text('episode/outcome', outcome, step)
        self._finish()

    def reset(self, step, detail):
        """! @brief 记录复位持续时间和原因，不将错误描述当成数值。
        @param step 累计环境交互步数。
        @param detail 已写入 resets.jsonl 的复位记录字典。
        """
        self._begin(step)
        prefix = 'reset/' + detail.get('stage', 'stop_gate')
        self._scalar(prefix+'/wall_duration_s', detail.get('wall_wait'), step)
        self._scalar(prefix+'/sim_duration_s', detail.get('sim_wait'), step)
        result = str(detail.get('result', 'unknown'))
        failed = bool(detail.get('timeout', False)) or result in ('retry', 'exhausted', 'waiting')
        self._scalar(prefix+'/failure', int(failed), step)
        reason = str(detail.get('reason') or result)
        limit = str(detail.get('timeout_limit', ''))
        self.writer.add_text(prefix+'/reason', reason + (f' ({limit})' if limit else ''), step)
        self._finish()

    def rates(self, step, metrics, prefix='navigation'):
        """! @brief 写入按有效回合计算的导航比率。
        @param step 对应累计交互步数。
        @param metrics 本次训练累计统计或独立评估统计字典。
        @param prefix 区分训练导航和独立评估的命名空间。
        """
        self._begin(step)
        for name in ('success_rate', 'collision_rate', 'timeout_rate', 'planning_timeout_rate', 'valid'):
            self._scalar(prefix+'/'+name, metrics.get(name), step)
        self._finish()

    def flush(self):
        """! @brief 刷新待写入事件。"""
        if not self.closed:
            self.writer.flush()
            self.pending = 0

    def close(self):
        """! @brief 刷新并关闭本次事件流；重复调用安全。"""
        if not self.closed:
            self.flush()
            self.writer.close()
            self.closed = True

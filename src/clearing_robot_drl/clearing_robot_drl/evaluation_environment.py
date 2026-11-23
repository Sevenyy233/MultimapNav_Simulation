"""按需启动和回收独立的 Fast DDS 与无界面评估环境。"""
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def _free_port(socket_type):
    """! @brief 在回环地址选择当前可用的临时端口。"""
    with socket.socket(socket.AF_INET, socket_type) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def _stop_group(process, grace=8.0):
    """! @brief 仅向本对象创建的进程组发送终止信号并回收子进程。"""
    if process is None:
        return
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            # 优先由 roslaunch 关闭它管理的节点及嵌套导航进程。
            process.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            pass
    for sig, delay in ((signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + delay
        while time.monotonic() < deadline:
            process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(.1)
    process.poll()


class EvaluationEnvironment:
    """! @brief 管理一个快照评估专用的发现服务、仿真器及评估进程。"""
    def __init__(self, scene, log_directory, run_id, startup_timeout=300., status=None):
        """! @brief 配置独立评估及可选的终端进度记录函数 status。"""
        self.scene = scene
        self.log_directory = Path(log_directory)
        self.run_id = run_id
        self.startup_timeout = startup_timeout
        self.status = status
        self.discovery = None
        self.stack = None
        self.worker = None
        self.files = []
        self.environment = None

    def _launch(self, command, name, env):
        """! @brief 将子进程置入独立进程组并把输出保存到独立日志。"""
        stream = (self.log_directory / (self.run_id + '-' + name + '.log')).open('w', encoding='utf-8')
        self.files.append(stream)
        return subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)

    def __enter__(self):
        """! @brief 启动隔离服务，等待本次启动日志中的两个就绪标志。"""
        try:
            discovery_port = _free_port(socket.SOCK_DGRAM)
            gazebo_port = _free_port(socket.SOCK_STREAM)
            env = os.environ.copy()
            env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
            env['ROS_DISCOVERY_SERVER'] = f'127.0.0.1:{discovery_port}'
            env['ROS_LOCALHOST_ONLY'] = '1'
            env['GAZEBO_MASTER_URI'] = f'http://127.0.0.1:{gazebo_port}'
            env['PYTHONUNBUFFERED'] = '1'
            self.environment = env
            server_env = env.copy()
            server_env.pop('ROS_DISCOVERY_SERVER', None)
            self.discovery = self._launch(['fast-discovery-server', '--server-id', '0',
                '--udp-address', '127.0.0.1', '--udp-port', str(discovery_port)], 'discovery', server_env)
            # 启动即失败必须在创建仿真进程之前暴露。
            time.sleep(.5)
            if self.discovery.poll() is not None:
                raise RuntimeError('Fast DDS 发现服务启动失败；请查看 discovery 日志')
            self.stack = self._launch(['ros2', 'launch', 'clearing_robot_plugins', 'bringup.launch.py',
                f'scene:={self.scene}', 'map_mode:=gridmap', 'controller:=training', 'gui:=false', 'use_rviz:=false'], 'bringup', env)
            log = self.log_directory / (self.run_id + '-bringup.log')
            deadline = time.monotonic() + self.startup_timeout
            report_at = time.monotonic() + 15.
            pending = {'Clearing robot ready', '导航模式已就绪'}
            position = 0
            tail = ''
            with log.open('r', encoding='utf-8', errors='replace') as reader:
                while pending:
                    if self.discovery.poll() is not None or self.stack.poll() is not None:
                        raise RuntimeError('评估发现服务或仿真进程提前退出；请查看启动日志')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('评估环境启动超时，缺少就绪标志: '+', '.join(sorted(pending)))
                    reader.seek(position)
                    chunk = reader.read()
                    position = reader.tell()
                    lines = tail + chunk
                    pending = {marker for marker in pending if marker not in lines}
                    tail = lines[-128:]
                    if pending:
                        if self.status and time.monotonic() >= report_at:
                            self.status('独立评估仿真等待就绪: '+', '.join(sorted(pending)))
                            report_at = time.monotonic() + 15.
                        time.sleep(.25)
            return self
        except BaseException:
            self.close()
            raise

    def run(self, command, timeout):
        """! @brief 在隔离环境运行评估，并在启动进程退出或超时时报错。"""
        self.worker = self._launch(command, 'worker', self.environment)
        started = time.monotonic()
        deadline = started + timeout
        report_at = started + 15.
        while self.worker.poll() is None:
            if self.discovery.poll() is not None or self.stack.poll() is not None:
                raise RuntimeError('评估期间发现服务或仿真进程退出')
            if time.monotonic() >= deadline:
                raise TimeoutError('评估工作进程超时')
            if self.status and time.monotonic() >= report_at:
                path = self.log_directory / (self.run_id + '-worker.log')
                with path.open('rb') as stream:
                    stream.seek(max(0, path.stat().st_size-4096))
                    lines = stream.read().decode('utf-8', errors='replace').strip().splitlines()
                marker = lines[-1] if lines else '等待首条评估日志'
                self.status(f'独立评估运行 {time.monotonic()-started:.0f}s: {marker}')
                report_at = time.monotonic() + 15.
            time.sleep(.25)
        return self.worker.returncode

    def close(self):
        """! @brief 按评估、仿真、发现服务次序清理本次拥有的所有进程组。"""
        for process in (self.worker, self.stack, self.discovery):
            _stop_group(process)
        self.worker = self.stack = self.discovery = None
        for stream in self.files:
            stream.close()
        self.files.clear()

    def __exit__(self, exc_type, exc, traceback):
        self.close()
        return False

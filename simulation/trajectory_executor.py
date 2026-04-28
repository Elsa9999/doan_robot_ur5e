"""
simulation/trajectory_executor.py — Bộ Thực thi Quỹ đạo (Trajectory Player).

NGUYÊN LÝ HOẠT ĐỘNG:
- Nhận vào một đối tượng JointTrajectory (chuỗi N điểm tọa độ khớp theo thời gian).
- Mỗi vòng lặp vật lý (1/240 giây), đọc ra 1 điểm trên quỹ đạo tại thời điểm t.
- Đẩy 6 góc khớp tại điểm đó xuống cho PyBullet thực thi.
- Giống như kim đĩa than lướt trên rãnh nhạc — đọc từng nốt nhạc và phát liên tục.
"""
import time


class TrajectoryExecutor:
    """Bộ phát quỹ đạo — chạy đồng bộ với vòng lặp vật lý 240Hz của PyBullet."""
    def __init__(self, env, bridge=None):
        self._env           = env
        self._bridge        = bridge
        self._traj          = None
        self._t0            = None
        self._running       = False
        self._done_callback = None
        self._speed_scale   = 1.0

    # ─── Public API ───────────────────────────────────────────────────────────

    def execute(self, trajectory, done_callback=None, speed_scale: float = 1.0):
        """Bắt đầu chạy trajectory. Thực thi đồng bộ với simulation steps."""
        self._traj          = trajectory
        self._t             = 0.0
        self._running       = True
        self._done_callback = done_callback
        self._speed_scale   = max(0.1, min(2.0, speed_scale))

    def stop(self):
        """Dừng ngay, giữ nguyên vị trí hiện tại."""
        self._running = False
        self._traj    = None

    def set_speed(self, speed_scale: float):
        """Thay đổi tốc độ real-time (0.1 → 2.0)."""
        self._speed_scale = max(0.1, min(2.0, speed_scale))

    @property
    def is_running(self) -> bool:
        return self._running

    # ─── Update (gọi mỗi simulation step) ────────────────────────────────────

    def update(self) -> dict:
        """
        Hàm này được gọi MỖI BƯỚC vật lý (240 lần/giây).
        Tại mỗi bước:
        1. Tăng biến thời gian t lên 1/240 giây.
        2. Đọc ra tọa độ 6 khớp tại thời điểm t trên quỹ đạo.
        3. Đẩy tọa độ đó xuống cho PyBullet di chuyển robot.
        4. Kiểm tra xem đã chạy hết quỹ đạo chưa.
        """
        if not self._running or self._traj is None:
            return {'running': False, 'progress': 0.0, 't': 0.0, 'q': None}

        self._t += (1.0 / 240.0) * self._speed_scale
        t = self._t
        point = self._traj.get_point(t)

        # Set joint với max_velocity cao hơn manual vì trajectory đã smooth
        self._env.set_joint_positions(point['q'], max_velocity=3.0)

        progress = min(t / self._traj.duration, 1.0)

        if self._traj.is_done(t):
            self._running = False
            if self._done_callback:
                self._done_callback()

        return {
            'running':  self._running,
            'progress': progress,
            't':        t,
            'q':        point['q']
        }

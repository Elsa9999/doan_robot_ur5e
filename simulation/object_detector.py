"""
simulation/object_detector.py — Camera ảo (Object Detector) dùng Raycast.

NGUYÊN LÝ HOẠT ĐỘNG:
- Thay vì dùng camera thực + thuật toán thị giác (OpenCV), ta dùng PyBullet Raycast.
- Raycast = Bắn 1 tia laser ảo từ đầu robot thẳng xuống mặt bàn.
- Nếu tia chạm vào vật thể → Trả về tọa độ (X, Y, Z) chính xác của vật.
- Ngoài ra, lớp này còn tính sẵn các tư thế tiếp cận (Approach), gắp (Pick), nhấc (Lift).
"""
import pybullet as p


class ObjectDetector:
    """
    Lớp phát hiện vật thể bằng Raycast (tia laser ảo) và
    tính toán các tư thế gắp/thả cho robot.
    """

    def __init__(self, env):
        self._env     = env
        self._ray_ids = []   # debug line IDs ray hiện tại (max 1)

    # ─── Pose query ───────────────────────────────────────────────────────────

    def get_object_pose(self, object_id: int) -> dict:
        """Lấy vị trí + orientation vật trực tiếp từ PyBullet."""
        pos, orn = p.getBasePositionAndOrientation(object_id)
        euler    = p.getEulerFromQuaternion(orn)
        return {
            'pos':   list(pos),
            'orn':   list(orn),
            'euler': list(euler)
        }

    # ─── Raycast ──────────────────────────────────────────────────────────────

    def raycast_detect(self, ee_pos: list, max_dist: float = 0.5) -> dict:
        """
        Bắn tia laser ảo từ vị trí End-Effector thẳng xuống (theo trục Z âm).
        - Tia dài tối đa 0.5m (nửa mét).
        - Nếu tia chạm vật thể → Vẽ tia xanh + trả về tọa độ.
        - Nếu không chạm gì → Vẽ tia đỏ + trả về None.
        """
        ray_start = list(ee_pos)
        ray_end   = [ee_pos[0], ee_pos[1], ee_pos[2] - max_dist]

        result        = p.rayTest(ray_start, ray_end)[0]
        hit_object_id = result[0]
        hit_pos       = list(result[3]) if result[3] else ray_end

        hit = hit_object_id > 0
        self._draw_ray(ray_start, ray_end,
                       color=[0, 1, 0] if hit else [1, 0, 0])

        if not hit:
            return None

        return {
            'object_id': hit_object_id,
            'hit_pos':   hit_pos,
            'distance':  ee_pos[2] - hit_pos[2]
        }

    def _draw_ray(self, start, end, color):
        """Xóa ray cũ, vẽ mới."""
        for lid in self._ray_ids:
            try:
                p.removeUserDebugItem(lid)
            except Exception:
                pass
        self._ray_ids.clear()
        lid = p.addUserDebugLine(start, end, color, lineWidth=1)
        self._ray_ids.append(lid)

    def clear_ray(self):
        for lid in self._ray_ids:
            try: p.removeUserDebugItem(lid)
            except: pass
        self._ray_ids.clear()

    # ─── Pose computation ─────────────────────────────────────────────────────

    def compute_pick_poses(self,
                           object_pos: list,
                           approach_height: float = 0.15,
                           pick_clearance: float  = 0.01) -> dict:
        """
        Tính 3 tư thế cho quy trình Gắp:
        1. Approach: Bay trên đầu vật 15cm (an toàn, không va chạm).
        2. Pick: Hạ xuống sát vật (cách 1cm để giác hút chạm).
        3. Lift: Nhấc vật lên cao 20cm để di chuyển an toàn.
        """
        x, y, z = object_pos
        return {
            'approach': [x, y, z + approach_height],
            'pick':     [x, y, z + pick_clearance],
            'lift':     [x, y, z + 0.20],
        }

    def compute_place_poses(self,
                            bin_center: list,
                            place_height: float = 0.15) -> dict:
        """
        Tính 2 tư thế cho quy trình Thả:
        1. Above bin: Bay trên miệng thùng 15cm.
        2. Place: Hạ xuống lòng thùng 8cm rồi nhả.
        """
        x, y, z = bin_center
        return {
            'above_bin': [x, y, z + place_height],
            'place':     [x, y, z + 0.08],
        }

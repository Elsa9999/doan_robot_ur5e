"""
simulation/manual_controller.py — Bộ Điều khiển Bằng tay (Manual Jog Controller).

CHỨC NĂNG:
- Cho phép người dùng điều khiển robot bằng bàn phím trong cửa sổ PyBullet.
- Hỗ trợ 2 chế độ:
  1. Joint Mode: Bấm Q/W, A/S, Z/X... để xoay từng khớp tăng/giảm 0.05 rad.
  2. Cartesian Mode: Bấm phím mũi tên để dịch chuyển XYZ tăng/giảm 0.01m.
- Tự động kiểm tra vùng làm việc an toàn (WorkspaceValidator) trước khi di chuyển.

LƯU Ý: File này chỉ dùng khi chạy PyBullet trực tiếp (không qua HMI).
Khi chạy HMI (python -m hmi.app), việc điều khiển bằng tay sẽ do các Panel GUI đảm nhận.
"""
import pybullet as p
import copy
from kinematics.forward_kinematics import forward_kinematics
from kinematics.inverse_kinematics import inverse_kinematics
from kinematics.workspace_validator import WorkspaceValidator
from simulation.environment import UR5eEnvironment, HOME_POSE
from utils.transforms import local_to_world, world_to_local

# Giới hạn góc quay tối đa cho từng khớp (đơn vị: radian)
# Khớp 1,2,4,5,6: ±360° (±6.28 rad). Khớp 3 (khuỷu): ±180° (±3.14 rad).
JOINT_LIMITS = {
    'lower': [-6.28, -6.28, -3.14, -6.28, -6.28, -6.28],
    'upper': [ 6.28,  6.28,  3.14,  6.28,  6.28,  6.28]
}


class ManualController:
    """Bộ điều khiển bằng bàn phím cho robot trong cửa sổ PyBullet 3D."""
    def __init__(self, env: UR5eEnvironment):
        self._env        = env              # Môi trường vật lý PyBullet
        self._q_current  = list(HOME_POSE)  # 6 góc khớp hiện tại (bắt đầu từ Home)
        self._mode       = 'joint'          # Chế độ hiện tại: 'joint' hoặc 'cartesian'
        self._step_joint = 0.05             # Bước nhảy mỗi lần bấm phím (Joint mode): 0.05 rad ≈ 2.86°
        self._step_cart  = 0.01             # Bước nhảy mỗi lần bấm phím (Cartesian mode): 0.01m = 1cm
        self._running    = True             # Cờ cho vòng lặp chính
        self._validator  = WorkspaceValidator()  # Bộ kiểm tra vùng làm việc an toàn

        self._text_ids = []                 # ID các dòng text debug trên màn hình 3D
        self._keys = self._define_keys()    # Bản đồ phím bàn phím → hành động
        self._setup_debug_text()            # Tạo text hiển thị trên cửa sổ PyBullet

    # ─── Cấu hình phím bấm ────────────────────────────────────────────────────
    def _define_keys(self) -> dict:
        """Ánh xạ mã phím → tên hành động. Ví dụ: phím 'Q' → xoay khớp 0 giảm."""
        return {
            # Chế độ Joint: Q/W (khớp 1), A/S (khớp 2), Z/X (khớp 3)...
            ord('q'): 'joint_0_minus',
            ord('w'): 'joint_0_plus',
            ord('a'): 'joint_1_minus',
            ord('s'): 'joint_1_plus',
            ord('z'): 'joint_2_minus',
            ord('x'): 'joint_2_plus',
            ord('e'): 'joint_3_minus',
            ord('r'): 'joint_3_plus',
            ord('d'): 'joint_4_minus',
            ord('f'): 'joint_4_plus',
            ord('c'): 'joint_5_minus',
            ord('v'): 'joint_5_plus',
            # Chế độ Cartesian: Phím mũi tên (XY), PageUp/Down (Z)
            p.B3G_UP_ARROW:    'cart_x_plus',
            p.B3G_DOWN_ARROW:  'cart_x_minus',
            p.B3G_LEFT_ARROW:  'cart_y_plus',
            p.B3G_RIGHT_ARROW: 'cart_y_minus',
            p.B3G_PAGE_UP:     'cart_z_plus',
            p.B3G_PAGE_DOWN:   'cart_z_minus'
        }

    # ─── Hiển thị thông tin trên màn hình 3D ──────────────────────────────────
    def _setup_debug_text(self):
        """Tạo 4 dòng text debug hiển thị trực tiếp trên cửa sổ PyBullet 3D."""
        self._text_ids = [
            p.addUserDebugText("MODE: JOINT", [-0.8, -0.6, 1.4], textColorRGB=[1, 1, 0], textSize=1.2),
            p.addUserDebugText("J1:0 J2:0 J3:0 J4:0 J5:0 J6:0", [-0.8, -0.6, 1.3], textColorRGB=[1, 1, 1], textSize=1.0),
            p.addUserDebugText("EE: x=0 y=0 z=0", [-0.8, -0.6, 1.2], textColorRGB=[0, 1, 1], textSize=1.0),
            p.addUserDebugText("ENTER=toggle MODE, SPACE=home, F1=reset", [-0.8, -0.6, 1.1], textColorRGB=[0.8, 0.8, 0.8], textSize=0.8)
        ]
        self._update_debug_text()

    def _update_debug_text(self):
        """Cập nhật nội dung 4 dòng text trên màn hình 3D theo trạng thái hiện tại."""
        q = self._q_current
        fk_result = forward_kinematics(q)  # Tính tọa độ XYZ từ 6 góc khớp
        pos = fk_result['position']
        ok, reason = self._validator.is_valid_ee(pos)  # Kiểm tra vùng an toàn
        ws_status = "OK" if ok else f"WARN: {reason[:20]}"

        # Dòng 1: Chế độ + trạng thái vùng làm việc
        txt1 = f"MODE: {self._mode.upper()} | WS: {ws_status}"
        # Dòng 2: 6 góc khớp hiện tại
        txt2 = f"J1:{q[0]:.2f} J2:{q[1]:.2f} J3:{q[2]:.2f} J4:{q[3]:.2f} J5:{q[4]:.2f} J6:{q[5]:.2f}"
        # Dòng 3: Tọa độ XYZ của End-Effector
        txt3 = f"EE: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}"
        # Dòng 4: Hướng dẫn phím tắt
        txt4 = "ENTER=toggle mode   SPACE=home   F1=reset"

        # Cập nhật text bằng replaceItemUniqueId (không tạo mới, chỉ thay nội dung)
        p.addUserDebugText(txt1, [-0.8, -0.6, 1.4], textColorRGB=[1, 1, 0], textSize=1.2, replaceItemUniqueId=self._text_ids[0])
        p.addUserDebugText(txt2, [-0.8, -0.6, 1.3], textColorRGB=[1, 1, 1], textSize=1.0, replaceItemUniqueId=self._text_ids[1])
        p.addUserDebugText(txt3, [-0.8, -0.6, 1.2], textColorRGB=[0, 1, 1], textSize=1.0, replaceItemUniqueId=self._text_ids[2])
        p.addUserDebugText(txt4, [-0.8, -0.6, 1.1], textColorRGB=[0.8, 0.8, 0.8], textSize=0.8, replaceItemUniqueId=self._text_ids[3])

    # ─── Giới hạn góc quay ────────────────────────────────────────────────────
    def _clamp_joints(self, q) -> list:
        """Kẹp cứng 6 góc khớp vào giới hạn vật lý của mô-tơ (tránh xoay quá mức)."""
        q_clamped = []
        for i in range(6):
            val = q[i]
            lo, hi = JOINT_LIMITS['lower'][i], JOINT_LIMITS['upper'][i]
            if val < lo:
                print(f"[WARN] Joint {i+1} clamped to lower ({lo})")
                val = lo
            elif val > hi:
                print(f"[WARN] Joint {i+1} clamped to upper ({hi})")
                val = hi
            q_clamped.append(val)
        return q_clamped

    def _apply_joints(self, q):
        """Áp dụng 6 góc khớp mới xuống PyBullet (sau khi kiểm tra an toàn)."""
        # Kiểm tra vùng làm việc TRƯỚC KHI di chuyển
        fk_res = forward_kinematics(q)
        w_pos = local_to_world(fk_res['position'])
        ok, reason = self._validator.is_valid_ee(w_pos)
        if not ok:
            print(f"[CTRL] Blocked by workspace: {reason}")
            return  # Từ chối di chuyển nếu ra ngoài vùng an toàn

        self._q_current = self._clamp_joints(q)     # Kẹp góc vào giới hạn
        self._env.set_joint_positions(self._q_current)  # Gửi lệnh xuống PyBullet
        self._env.step(5)                             # Chạy 5 bước vật lý để robot di chuyển
        self._update_debug_text()                     # Cập nhật text trên màn hình

    # ─── Xử lý chế độ Joint ──────────────────────────────────────────────────
    def handle_joint_mode(self, action: str):
        """Xử lý phím bấm trong chế độ Joint (xoay từng khớp)."""
        if not action.startswith('joint_'): return
        parts = action.split('_')           # Ví dụ: 'joint_0_plus' → ['joint', '0', 'plus']
        idx = int(parts[1])                 # Số thứ tự khớp (0-5)
        direction = 1 if parts[2] == 'plus' else -1  # Hướng xoay (+/-)
        q_new = copy.copy(self._q_current)  # Copy để không ảnh hưởng trạng thái cũ
        q_new[idx] += direction * self._step_joint  # Xoay thêm 0.05 rad
        self._apply_joints(q_new)

    # ─── Xử lý chế độ Cartesian ──────────────────────────────────────────────
    def handle_cartesian_mode(self, action: str):
        """Xử lý phím bấm trong chế độ Cartesian (dịch chuyển XYZ)."""
        if not action.startswith('cart_'): return
        fk_res = forward_kinematics(self._q_current)  # Tính tọa độ XYZ hiện tại
        pos, euler = local_to_world(fk_res['position'], fk_res['euler'])  # Chuyển sang hệ World
        pos_list = list(pos)
        parts = action.split('_')           # Ví dụ: 'cart_x_plus' → ['cart', 'x', 'plus']
        axis  = parts[1]                    # Trục di chuyển: 'x', 'y', hoặc 'z'
        direction = 1 if parts[2] == 'plus' else -1
        pos_list[{'x': 0, 'y': 1, 'z': 2}[axis]] += direction * self._step_cart  # Dịch 1cm

        # Kiểm tra vùng an toàn trước khi gọi IK
        ok, reason = self._validator.is_valid_ee(pos_list)
        if not ok:
            print(f"[CTRL] Blocked: {reason}")
            return

        # Chuyển về hệ Local rồi gọi IK để tính 6 góc khớp tương ứng
        l_pos, l_eul = world_to_local(pos_list, euler)
        res = inverse_kinematics(l_pos, l_eul, q_current=self._q_current)
        best = res['best']
        if best is not None:
            self._apply_joints(best)  # Áp dụng nghiệm IK tốt nhất
        else:
            print(f"[CTRL] IK failed at pos: {pos_list}")  # IK vô nghiệm (ngoài tầm)

    # ─── Lệnh đặc biệt ──────────────────────────────────────────────────────
    def go_home(self):
        """Đưa robot về tư thế Home (tư thế nghỉ chuẩn công nghiệp)."""
        self._q_current = list(HOME_POSE)
        self._env.set_joint_positions(self._q_current)
        self._env.step(10)
        self._update_debug_text()
        print("[CTRL] Go home")

    def toggle_mode(self):
        """Chuyển đổi giữa chế độ Joint ↔ Cartesian."""
        self._mode = 'cartesian' if self._mode == 'joint' else 'joint'
        print(f"[CTRL] Mode: {self._mode.upper()}")
        self._update_debug_text()

    # ─── Vòng lặp xử lý phím bấm (gọi mỗi frame) ───────────────────────────
    def process_keys(self):
        """Đọc phím bấm từ PyBullet, phân loại và xử lý hành động tương ứng."""
        keys = p.getKeyboardEvents()  # Đọc tất cả phím đang được bấm
        for key, state in keys.items():
            if state & p.KEY_WAS_TRIGGERED:  # Chỉ xử lý khi phím vừa được nhấn (không giữ)
                if key == ord(' '):           # SPACE → Về Home
                    self.go_home()
                elif key == p.B3G_RETURN:     # ENTER → Đổi chế độ Joint/Cartesian
                    self.toggle_mode()
                elif key == p.B3G_F1:         # F1 → Reset toàn bộ môi trường
                    self._env.reset()
                    self.go_home()
                elif key in self._keys:       # Phím điều khiển → Xử lý theo chế độ
                    action = self._keys[key]
                    if self._mode == 'joint' and action.startswith('joint_'):
                        self.handle_joint_mode(action)
                    elif self._mode == 'cartesian' and action.startswith('cart_'):
                        self.handle_cartesian_mode(action)
        return self._running

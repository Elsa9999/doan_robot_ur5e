"""
utils/transforms.py — Chuyển đổi Hệ tọa độ giữa Robot và PyBullet.

VẤN ĐỀ:
- Hệ tọa độ DH (toán học) của robot UR5e: X hướng trước, Y hướng trái.
- Hệ tọa độ PyBullet (mô phỏng): Robot bị xoay 180° (PI) quanh trục Z,
  và đế robot được đặt trên mặt bàn ở độ cao Z = 0.42m.

GIẢI PHÁP:
- Mỗi khi gửi tọa độ từ FK/IK ra giao diện/PyBullet → Gọi local_to_world().
- Mỗi khi nhận tọa độ từ PyBullet gửi vào IK → Gọi world_to_local().
"""
import math

# ──────────────────────────────────────────────────────────────────────────────
# BIẾN ĐỔI TỌA ĐỘ: Local (FK/IK) → World (PyBullet)
# ──────────────────────────────────────────────────────────────────────────────
def local_to_world(pos: list, euler: list = None) -> tuple:
    """
    Chuyển tọa độ từ hệ DH (toán học) sang hệ PyBullet (mô phỏng).
    - Đảo dấu X và Y (do robot bị xoay 180° quanh Z).
    - Cộng thêm 0.42m vào Z (do đế robot đặt trên bàn cao 0.42m).
    """
    w_pos = [-pos[0], -pos[1], pos[2] + 0.42]
    if euler is not None:
        # Trừ PI khỏi góc Yaw để bù xoay 180°
        w_eul = [euler[0], euler[1], euler[2] - math.pi]
        return w_pos, w_eul
    return w_pos

# ──────────────────────────────────────────────────────────────────────────────
# BIẾN ĐỔI TỌA ĐỘ: World (PyBullet) → Local (FK/IK)
# ──────────────────────────────────────────────────────────────────────────────
def world_to_local(w_pos: list, w_euler: list = None) -> tuple:
    """
    Chuyển tọa độ từ hệ PyBullet (mô phỏng) về hệ DH (toán học).
    Phép biến đổi ngược lại: Đảo dấu XY, trừ 0.42m khỏi Z, cộng PI vào Yaw.
    """
    l_pos = [-w_pos[0], -w_pos[1], w_pos[2] - 0.42]
    if w_euler is not None:
        l_eul = [w_euler[0], w_euler[1], w_euler[2] + math.pi]
        return l_pos, l_eul
    return l_pos

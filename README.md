# 🤖 UR5e Pick & Place — Mô phỏng & Điều khiển Robot Công nghiệp

> Đồ án mô phỏng robot UR5e 6 bậc tự do với 3 chế độ vận hành:  
> **Manual** (tay) · **Auto** (máy trạng thái FSM) · **AI** (học tăng cường SAC)

---

## 🚀 Cài đặt & Chạy

```bash
# 1. Clone repo
git clone https://github.com/Elsa9999/doan_robot_ur5e.git
cd doan_robot_ur5e

# 2. Cài thư viện
pip install -r requirements.txt

# 3. Chạy giao diện
python -m hmi.app
```

---

## 📁 Chức năng từng file

### 📂 `kinematics/` — Lõi toán học (Động học Robot)

| File | Chức năng |
|------|-----------|
| `forward_kinematics.py` | **Động học Thuận (FK):** Nhập 6 góc khớp → Tính ra tọa độ XYZ + góc xoay của mũi kẹp. Dùng phép nhân chuỗi 6 ma trận DH 4×4. Bảng DH được đọc tự động từ file URDF. |
| `inverse_kinematics.py` | **Động học Nghịch (IK):** Nhập tọa độ XYZ mong muốn → Tính ra 6 góc khớp cần xoay. Hệ thống Hybrid: Lớp 1 giải tích (8 nghiệm, <1ms) + Lớp 2 số học L-BFGS-B (dự phòng). |
| `trajectory.py` | **Quy hoạch quỹ đạo:** Tạo đường đi mượt mà từ điểm A → B bằng biên dạng vận tốc hình thang (Tăng tốc → Đều ga → Giảm tốc). Hỗ trợ nội suy Joint Space và Cartesian Space. |
| `workspace_validator.py` | **Cảnh sát vùng cấm:** Kiểm tra tọa độ EE có nằm trong vùng an toàn không. Ngăn robot đập tay xuống bàn, vươn quá xa, hoặc đâm vào thùng rác. |

---

### 📂 `simulation/` — Thế giới vật lý 3D (PyBullet)

| File | Chức năng |
|------|-----------|
| `environment.py` | **Môi trường chính:** Dựng bàn, robot, thùng rác, cục mút trong PyBullet. Chứa hàm điều khiển mô-tơ (`set_joint_positions`), hàm di chuyển Cartesian (`move_ee_cartesian`), hàm gắp/nhả (`activate_gripper`). |
| `gripper.py` | **Giác hút chân không:** Mô phỏng giác hút bằng PyBullet Constraint (JOINT_FIXED). Khi hút = dán cứng vật vào đầu kẹp. Khi nhả = xóa ràng buộc, vật rơi tự do. |
| `pick_place_sm.py` | **Máy trạng thái FSM (chế độ Auto):** 11 trạng thái: IDLE → DETECT → APPROACH → DESCEND → PICK → LIFT → MOVE_TO_BIN → PLACE → RELEASE → RETREAT → DONE. Tự động gắp vật và thả vào thùng. |
| `object_detector.py` | **Camera ảo:** Dùng Raycast (bắn tia laser ảo) từ đầu robot xuống mặt bàn để dò vị trí vật thể. Tính sẵn các tư thế tiếp cận (Approach), gắp (Pick), nhấc (Lift). |
| `trajectory_executor.py` | **Bộ phát quỹ đạo:** Nhận vào JointTrajectory, mỗi bước vật lý (1/240s) đọc ra 1 điểm trên quỹ đạo và bơm xuống PyBullet. Giống kim đĩa than lướt trên rãnh nhạc. |
| `manual_controller.py` | **Điều khiển bằng bàn phím:** Dùng khi chạy PyBullet trực tiếp (không qua HMI). Hỗ trợ Joint Mode (Q/W/A/S...) và Cartesian Mode (phím mũi tên). |

---

### 📂 `hmi/` — Giao diện người dùng (PyQt5)

| File | Chức năng |
|------|-----------|
| `app.py` | **Điểm khởi chạy:** Import PyTorch trước PyQt5 (fix WinError 1114), tạo Splash Screen, khởi động SimBridge trong thread nền, mở MainWindow. |
| `main_window.py` | **Cửa sổ chính:** Ghép 4 tab điều khiển (Manual, Trajectory, Auto, AI) + Status Panel + Log Panel. Timer cập nhật giao diện mỗi 50ms (20 FPS). |
| `sim_bridge.py` | **Cầu nối HMI ↔ PyBullet:** Chạy trên Thread riêng ở 240Hz. Nhận lệnh từ GUI qua command_queue, thực thi trên PyBullet, trả kết quả qua state_queue. Quản lý 3 chế độ: Manual, Auto (FSM), AI (SAC). |

#### 📂 `hmi/widgets/` — Các bảng điều khiển con

| File | Chức năng |
|------|-----------|
| `joint_panel.py` | 6 thanh trượt (slider) điều khiển từng góc khớp riêng biệt. |
| `cartesian_panel.py` | Nhập tọa độ X, Y, Z + góc Roll, Pitch, Yaw → Bấm "Go To Pose" để di chuyển. Có nút Jog ±1cm theo từng trục. |
| `trajectory_panel.py` | Tạo danh sách waypoints, chọn kiểu nội suy (Joint/Cartesian), điều chỉnh tốc độ, bấm "Execute" để chạy quỹ đạo. |
| `auto_panel.py` | Bấm Start/Stop chế độ Auto FSM. Hiển thị trạng thái 11 bước + số chu kỳ đã hoàn thành. |
| `ai_panel.py` | Bấm Start/Stop chế độ AI (SAC). Hiển thị số lần gắp thành công. |
| `status_panel.py` | Hiển thị real-time: tọa độ XYZ, 6 góc khớp, trạng thái gripper, chế độ hoạt động. |
| `log_panel.py` | Console log hiển thị mọi sự kiện: IK thành công, lỗi, lệnh gửi, AI reward... |

---

### 📂 `utils/` — Tiện ích

| File | Chức năng |
|------|-----------|
| `transforms.py` | Chuyển đổi hệ tọa độ giữa DH (toán học) và PyBullet (mô phỏng). Robot bị xoay 180° + đặt trên bàn cao 0.42m nên cần biến đổi qua lại. |

---

### 📂 `urdf/` — Bản vẽ 3D Robot

| File | Chức năng |
|------|-----------|
| `ur5e_final.urdf` | File XML mô tả cấu trúc robot UR5e: 6 khớp, 7 link, kích thước, khối lượng. Được đọc bởi PyBullet để render 3D và bởi FK để trích xuất bảng DH. |
| `meshes/ur5e/` | Thư mục chứa file 3D (.stl, .dae) cho từng link của robot (vai, bắp tay, cẳng tay, cổ tay). |

---

### 📂 Gốc — Huấn luyện AI & Kiểm thử

| File | Chức năng |
|------|-----------|
| `train_17d_grasp.py` | Huấn luyện AI giai đoạn 1: Học gắp vật (SAC, 3M bước, ~1 tiếng). |
| `train_17d_place.py` | Huấn luyện AI giai đoạn 2: Học gắp + thả vào thùng (Transfer Learning, 5.5M bước, ~2.5 tiếng). |
| `requirements.txt` | Danh sách thư viện cần cài: pybullet, numpy, scipy, PyQt5, torch, stable-baselines3, gymnasium. |

---

## 🧪 Kiểm chứng Động học

```bash
# Kiểm chứng Động học Thuận (3 test cases, sai số < 5mm = PASS)
python kinematics/forward_kinematics.py

# Kiểm chứng Động học Nghịch (4 test cases, Round-Trip Verification, sai số < 1mm = PASS)
python kinematics/inverse_kinematics.py
```

---

## 📚 Tài liệu tham khảo

1. **Hawkins, K. P. (2013).** *Analytic Inverse Kinematics for the Universal Robots UR-5/UR-10 Arms.* Georgia Institute of Technology.
2. **Andersen, R. S. (2018).** *Kinematics of a UR5.* Aalborg University.
3. **Haarnoja, T., et al. (2018).** *Soft Actor-Critic: Off-Policy Maximum Entropy Deep RL with a Stochastic Actor.* ICML 2018.
4. **Universal Robots (2024).** *UR5e Technical Specifications & URDF.*

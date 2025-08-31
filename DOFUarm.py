import os  # Thư viện thao tác với hệ thống tệp và đường dẫn
import json  # Thư viện để đọc/ghi file JSON
import math  # Thư viện toán học tiêu chuẩn (hàm lượng giác, hằng số pi, ...)
import struct  # Thư viện đóng gói/giải nén dữ liệu nhị phân tương tự ngôn ngữ C
from typing import Tuple, Optional  # Hỗ trợ khai báo kiểu dữ liệu tĩnh cho biến/hàm

import numpy as np  # Thư viện tính toán ma trận, đại số tuyến tính hiệu suất cao
import serial  # Thư viện pyserial để giao tiếp cổng nối tiếp (UART/COM)

# Debug: in ra đường dẫn (path) của module serial được import để đảm bảo dùng đúng phiên bản pyserial
print("✅ Imported serial from:", getattr(serial, "__file__", "⚠️ Not found"))


class UArm3DOF:
    """Lớp (class) điều khiển cánh tay robot UArm 3 bậc tự do (3‑DOF).

    Chức năng chính:
    * Quản lý kết nối UART tới vi điều khiển điều khiển robot.
    * Cung cấp hàm động học thuận và nghịch.
    * Xây dựng và gửi gói lệnh (packet) theo giao thức tự định nghĩa.
    """

    # ==== HẰNG SỐ GIAO THỨC UART ==== #
    PACKET_HEADER = bytes([0xAA, 0x55])  # 2 byte đầu cố định đánh dấu header gói tin

    # ------------------------------------------------------------------
    #                   Khởi tạo & cấu hình mặc định
    # ------------------------------------------------------------------
    def __init__(
        self,
        port: str = "COM6",  # Tên cổng serial mặc định trên Windows
        baud: int = 115200,  # Baudrate mặc định (tốc độ truyền)
        config_path: str | os.PathLike[str] = "config.json",  # File lưu cấu hình góc Home
    ) -> None:
        """Hàm khởi tạo lớp.

        Parameters
        ----------
        port, baud : Thông số serial.
        config_path : Đường dẫn tới file `config.json`.
        """
        # Kiểm tra pyserial đã được cài đặt đầy đủ hay chưa
        if not hasattr(serial, "Serial"):
            raise ImportError("❌ pyserial chưa được cài đặt. Chạy: pip install pyserial")

        # --- Serial ----------------------------------------------------
        self.serial = serial.Serial(port, baud, timeout=1)  # Mở cổng serial với timeout 1 giây
        print(f"🔌 Serial port {port} @ {baud}bps opened")  # Thông báo đã mở cổng

        # Bộ đếm transition 2 byte (quay vòng từ 0..65535) cho mỗi gói tin gửi
        self.transition_counter: int = 0

        # --- Các tham số cơ học của robot (mm) ------------------------
        self.l1 = 130.0 + 40.0  # Chiều dài khâu 1 (từ base tới khớp thứ 2) + 40.0 nếu có đế đựng mạch điện
        self.l2 = 140.0  # Chiều dài khâu 2
        self.l3 = 140.0  # Chiều dài khâu 3
        self.l4 = 35.0   # Chiều dài "end‑effector" (pince / nam châm)

        # Lưu giá trị góc motor lần trước (dùng để bù theta3 khi cần)
        self.theta1_m_prev: float = 999.0  # 999 đánh dấu giá trị "chưa khởi tạo"
        self.theta2_m_prev: float = 999.0
        self.theta3_m_prev: float = 999.0

        # --- Home Joint (đọc từ config.json) ---------------------------
        self._config_path = os.fspath(config_path)  # Chuẩn hoá path về chuỗi thuần
        self.home_joint1: int = 0  # Mặc định 0 nếu chưa có file
        self.home_joint2: int = 0
        self.home_joint3: int = 0
        self._load_home_config()  # Gọi hàm nội bộ để nạp file cấu hình

    # ------------------------------------------------------------------
    #                       Đọc / ghi config.json
    # ------------------------------------------------------------------
    def _load_home_config(self) -> None:
        """Đọc file config và cập nhật 3 biến `home_jointX`."""
        data = self._read_config_file(self._config_path)  # Đọc JSON (trả về dict hoặc None)
        if data is None:  # Nếu không tìm thấy hoặc lỗi file
            print("⚠️ Dùng Home (0,0,0) mặc định")
            return  # Thoát, giữ nguyên 0,0,0

        try:
            # Lấy giá trị Joint1/2/3, ép về int (nếu không có mặc định 0)
            self.home_joint1 = int(data.get("Joint1", 0))
            self.home_joint2 = int(data.get("Joint2", 0))
            self.home_joint3 = int(data.get("Joint3", 0))
            print(
                f"✅ Đã nạp Home Joint từ {self._config_path}: "
                f"({self.home_joint1}, {self.home_joint2}, {self.home_joint3})"
            )
        except (TypeError, ValueError):  # Bắt lỗi giá trị sai kiểu
            print("❌ Trường JointX trong config không hợp lệ – dùng (0,0,0)")
            self.home_joint1 = self.home_joint2 = self.home_joint3 = 0

    def save_home_config(self) -> None:
        """Ghi lại 3 Home Joint hiện tại vào `config.json`."""
        data = {
            "Joint1": self.home_joint1,
            "Joint2": self.home_joint2,
            "Joint3": self.home_joint3,
        }
        try:
            with open(self._config_path, "w", encoding="utf-8") as f:  # Mở file ghi UTF‑8
                json.dump(data, f, ensure_ascii=False, indent=2)  # Ghi đẹp, có thụt lề
            print(f"✅ Đã lưu Home Joint vào {self._config_path}")
        except OSError as e:
            print("❌ Lỗi ghi config:", e)

    def set_home(self, joint1: int, joint2: int, joint3: int, save: bool = False) -> None:
        """Cập nhật giá trị Home Joint và tuỳ chọn lưu xuống file."""
        self.home_joint1, self.home_joint2, self.home_joint3 = joint1, joint2, joint3
        print(f"🔧 Đã set Home: ({joint1}, {joint2}, {joint3})")
        if save:
            self.save_home_config()  # Lưu file nếu tham số save = True

    # -------------------- Static helper -------------------------------
    @staticmethod
    def _read_config_file(path: str) -> Optional[dict]:
        """Hàm tiện ích đọc file JSON, trả về dict hoặc None."""
        if not os.path.exists(path):  # File không tồn tại
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)  # Trả về dữ liệu dict
        except (json.JSONDecodeError, OSError):  # Lỗi cú pháp JSON hoặc OS
            return None

    # ------------------------------------------------------------------
    #                     Tiện ích toán học (statics)
    # ------------------------------------------------------------------
    @staticmethod
    def deg_to_rad(deg: float) -> float:
        return deg * math.pi / 180.0  # Chuyển độ sang radian

    @staticmethod
    def rad_to_deg(rad: float) -> float:
        return rad * 180.0 / math.pi  # Chuyển radian sang độ

    @staticmethod
    def _dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """Tạo ma trận biến đổi Denavit‑Hartenberg (4×4)."""
        return np.array(
            [
                [
                    math.cos(theta),
                    -math.sin(theta) * math.cos(alpha),
                    math.sin(theta) * math.sin(alpha),
                    a * math.cos(theta),
                ],
                [
                    math.sin(theta),
                    math.cos(theta) * math.cos(alpha),
                    -math.cos(theta) * math.sin(alpha),
                    a * math.sin(theta),
                ],
                [0, math.sin(alpha), math.cos(alpha), d],
                [0, 0, 0, 1],
            ]
        )

    # ------------------------------------------------------------------
    #        Thuật toán động học thuận / nghịch (giữ nguyên logic)
    # ------------------------------------------------------------------
    def forward_kinematics(self, q1: float, q2: float, q3: float) -> Tuple[float, float, float]:
        """Tính toạ độ (x,y,z) đầu công cụ từ 3 góc khớp (độ)."""
        # Chuyển sang radian và căn chỉnh hệ toạ độ theo mô hình DH
        q1_r = self.deg_to_rad(q1)
        q2_r = self.deg_to_rad(q2 + 90)  # +90 để trùng khớp DH
        q3_r = self.deg_to_rad(q3 - 90)  # -90 để trùng khớp DH

        # Tạo các ma trận biến đổi DH cho từng khớp
        A1 = self._dh_transform(q1_r, self.l1, 0, math.pi / 2)
        A2 = self._dh_transform(q2_r, 0, self.l2, 0)
        A3 = self._dh_transform(q3_r, 0, self.l3, 0)

        T03 = A1 @ A2 @ A3  # Phép nhân ma trận (numpy @) => ma trận tổng
        O3 = T03[:3, 3]  # Toạ độ khớp thứ 3 (trước end‑effector)

        # Tính toạ độ điểm cuối (O4) bằng cách cộng thêm l4 theo hướng q1
        O4 = np.array(
            [
                O3[0] + self.l4 * math.cos(q1_r),
                O3[1] + self.l4 * math.sin(q1_r),
                O3[2],
            ]
        )
        return tuple(O4.tolist())  # Trả về (x,y,z) dạng tuple Python

    def inverse_kinematics(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Tính 3 góc khớp (độ) từ toạ độ (x,y,z) mong muốn."""
        # Góc khớp 1 dựa trên arctan2 của y/x
        theta1 = math.atan2(y, x)

        # Tính toạ độ khớp 3 (sau khi trừ phần l4) so với gốc toạ độ base
        x3 = x - self.l4 * math.cos(theta1)
        y3 = y - self.l4 * math.sin(theta1)
        z3 = z - self.l1
        r = math.sqrt(x3 ** 2 + y3 ** 2 + z3 ** 2)  # Khoảng cách từ khớp 2 đến khớp 3

        # Sử dụng định lý cos để tìm theta3 (góc khớp thứ 3)
        cos_t3 = (r ** 2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3)
        cos_t3 = max(-1.0, min(1.0, cos_t3))  # Giới hạn [-1,1] tránh lỗi math.acos
        theta3 = -math.acos(cos_t3)  # Lấy nghiệm gập khuỷu xuống (negative)

        # Tính theta2 dựa vào hình học tam giác phẳng
        alpha = math.asin(z3 / r)
        beta = -math.atan2(self.l3 * math.sin(theta3), self.l2 + self.l3 * math.cos(theta3))
        theta2 = alpha + beta

        # Chuyển sang độ và điều chỉnh lại offset ±90° cho hệ robot
        q1_deg = self.rad_to_deg(theta1)
        q2_deg = self.rad_to_deg(theta2) - 90.0
        q3_deg = self.rad_to_deg(theta3) + 90.0
        return q1_deg, q2_deg, q3_deg

    # ------------------------------------------------------------------
    #                    Gói tin UART (CRC, transition)
    # ------------------------------------------------------------------
    def _build_packet(self, cmd: int, option: int, payload: bytes) -> bytes:
        """Tạo gói tin theo định dạng: Header + Length + Cmd + Option + Trans + Payload + CRC."""
        # Chèn bộ đếm transition (2 byte, big‑endian) vào gói tin
        trans = self.transition_counter & 0xFFFF
        self.transition_counter = (self.transition_counter + 1) & 0xFFFF  # Quay vòng
        trans_bytes = struct.pack(">H", trans)  # '>H' = unsigned short big‑endian

        # Tính trường length (không tính 2 byte header và 1 byte length tự thân)
        length = 1 + 1 + 2 + len(payload) + 1  # CMD + Option + Trans + Payload + CRC

        pkt = bytearray(self.PACKET_HEADER)  # Bắt đầu bằng header 0xAA55
        pkt.append(length)  # Thêm byte length
        pkt.append(cmd & 0xFF)  # Byte lệnh
        pkt.append(option & 0xFF)  # Byte tuỳ chọn (0=send,1=recv,...)
        pkt.extend(trans_bytes)  # 2 byte transition counter
        pkt.extend(payload)  # Payload dữ liệu

        # Tính CRC đơn giản bằng XOR tất cả byte sau byte length
        crc = 0
        for b in pkt[3:]:  # Bắt đầu từ byte thứ 3 (bỏ header + length)
            crc ^= b
        pkt.append(crc)
        return bytes(pkt)  # Trả về bytes không thể thay đổi

    def _send(self, data: bytes) -> None:
        """Gửi bytes ra cổng serial và in log hex."""
        if not self.serial.is_open:
            raise serial.SerialException("Serial port chưa mở!")
        self.serial.write(data)  # Gửi dữ liệu
        print("📤 Sent", data.hex(" "))  # In ở dạng hex, cách nhau bởi khoảng trắng

    # ------------------------ Public helpers --------------------------
    def send_packet(self, cmd: int, option: int = 0x00, payload: bytes | None = None):
        """Helper: tự động build + send packet."""
        if payload is None:
            payload = b""
        self._send(self._build_packet(cmd, option, payload))

    # --------------------- Hàm trung tâm (motor) ----------------------
    def send_motor_angles(self, q1_deg: float, q2_deg: float, q3_deg: float):
        """Gửi 3 góc motor (độ) tới firmware."""
        # Biến đổi góc theo quy ước firmware (lật dấu q2,q3)
        theta1_m = float(q1_deg)
        theta2_m = float(-q2_deg)
        theta3_m = float(-q3_deg)

        # Nếu có sự khác biệt đáng kể so với lần trước, cộng bù theta3 = theta3 + theta2
        if not (
            math.isclose(theta1_m, self.theta1_m_prev)
            and math.isclose(theta2_m, self.theta2_m_prev)
            and math.isclose(theta3_m, self.theta3_m_prev)
        ):
            theta3_m += theta2_m

        # Lưu lại để so lần gửi tiếp theo
        self.theta1_m_prev = theta1_m
        self.theta2_m_prev = theta2_m
        self.theta3_m_prev = theta3_m

        # Đóng gói payload định dạng Little‑endian float32 (<fff)
        payload = struct.pack("<fff", theta1_m, theta2_m, theta3_m)
        self.send_packet(cmd=0x05, payload=payload)

    # ------------------------- LỆNH ĐIỀU KHIỂN -----------------------
    def send_go_home(
        self,
        joint1: Optional[int] = None,
        joint2: Optional[int] = None,
        joint3: Optional[int] = None,
    ) -> None:
        """Gửi lệnh về vị trí Home.

        Nếu bỏ qua tham số → dùng giá trị đã nạp từ file config.
        Vẫn có thể truyền tay 3 giá trị để ghi đè.
        """
        if joint1 is None or joint2 is None or joint3 is None:
            # Lấy giá trị đã lưu trước đó
            joint1, joint2, joint3 = self.home_joint1, self.home_joint2, self.home_joint3
        payload = struct.pack(">hhh", joint1, joint2, joint3)  # Big‑endian int16
        self.send_packet(cmd=0x03, payload=payload)

    def send_emergency(self):
        """Kích hoạt trạng thái dừng khẩn trên firmware."""
        self.send_packet(cmd=0x02)

    def send_electromagnet(self, on: bool = True):
        """Bật/tắt nam châm điện ở đầu gắp."""
        self.send_packet(cmd=0x04, payload=bytes([0x01 if on else 0x00]))

    # ------------------------------------------------------------------
    #                               Cleanup
    # ------------------------------------------------------------------
    def close(self):
        """Đóng cổng serial an toàn."""
        if self.serial.is_open:
            self.serial.close()
            print("🔌 Đã đóng cổng serial")

    # Cho phép dùng câu lệnh with UArm3DOF() as arm: ...
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

from DOFUarm import UArm3DOF
import time

def main():
    """Demo điều khiển UArm 3DOF.

    • Tự động nạp Home Joint từ `config.json`.
    • Không cần khai báo thủ công HomeX/HomeY/HomeZ.
    """
    arm = UArm3DOF(port="COM3")  # Serial và config đã được nạp bên trong

    try:
        time.sleep(5)  # Đợi MCU sẵn sàng

        # ─────────────────── Về vị trí Home ───────────────────
        arm.send_go_home()  # Dùng góc trong `config.json`

        time.sleep(2)
        arm.send_electromagnet(False)  # Mở gripper

        # ─────────────────── Pick ───────────────────
        time.sleep(2)
        q1, q2, q3 = arm.inverse_kinematics(170, 80, 15)  # X, Y, Z (mm)
        arm.send_motor_angles(q1, q2, q3)

        time.sleep(2)
        arm.send_electromagnet(True)  # Kẹp

        # ─────────────────── Place ───────────────────
        time.sleep(2)
        q1, q2, q3 = arm.inverse_kinematics(170, 0, 100)
        arm.send_motor_angles(q1, q2, q3)

        time.sleep(2)
        q1, q2, q3 = arm.inverse_kinematics(170, 0, 270)
        arm.send_motor_angles(q1, q2, q3)

        # ─────────────────── Trả về ───────────────────
        time.sleep(10)
        q1, q2, q3 = arm.inverse_kinematics(170, 0, 15)
        arm.send_motor_angles(q1, q2, q3)

        time.sleep(2)
        arm.send_electromagnet(False)  # Thả gripper

        # ─────────────────── Về Home lần cuối ───────────────────
        time.sleep(5)
        arm.send_go_home()

        time.sleep(2)
        arm.send_emergency()

    finally:
        arm.close()


if __name__ == "__main__":
    main()

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
from DOFUarm import UArm3DOF

class ArmControlGUI:
    """
    Giao diện đồ họa (GUI) được cải tiến để điều khiển UArm3DOF mượt mà hơn,
    sử dụng kỹ thuật nội suy tuyến tính để làm mịn dữ liệu đầu vào.
    """

    def __init__(self, root):
        self.root = root
        self.root.title("UArm Joystick Control (Smoothed & Fixed)")

        self.arm: UArm3DOF | None = None
        self.is_connected = False
        self.control_thread = None
        self.stop_thread = False

        # --- Biến trạng thái để làm mịn chuyển động ---
        self.goal_x = 170.0
        self.goal_y = 0.0
        self.goal_z = 100.0
        self.commanded_x = self.goal_x
        self.commanded_y = self.goal_y
        self.commanded_z = self.goal_z
        
        self.SMOOTHING_FACTOR = 0.08 
        self.UPDATE_RATE = 20 # 20Hz

        self.X_LIMITS = (100, 250)
        self.Y_LIMITS = (-150, 150)
        self.Z_LIMITS = (10, 270)

        self._setup_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

    def _setup_widgets(self):
        """Khởi tạo và sắp xếp các thành phần trên giao diện."""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="10")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E))
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, sticky=tk.W)
        self.com_port_entry = ttk.Entry(conn_frame)
        self.com_port_entry.insert(0, "COM3")
        self.com_port_entry.grid(row=0, column=1, padx=5)

        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.grid(row=0, column=3, padx=10)

        control_frame = ttk.LabelFrame(main_frame, text="Manual Control (Cartesian)", padding="10")
        control_frame.grid(row=1, column=0, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Thanh trượt X
        ttk.Label(control_frame, text="X:").grid(row=0, column=0)
        self.x_slider = ttk.Scale(control_frame, from_=self.X_LIMITS[0], to=self.X_LIMITS[1], orient=tk.HORIZONTAL, command=self._on_slider_change)
        self.x_slider.set(self.goal_x)
        self.x_slider.grid(row=0, column=1, sticky="ew")
        self.x_label = ttk.Label(control_frame, text=f"{self.goal_x:.1f} mm")
        self.x_label.grid(row=0, column=2, padx=5)

        # Thanh trượt Y
        ttk.Label(control_frame, text="Y:").grid(row=1, column=0)
        self.y_slider = ttk.Scale(control_frame, from_=self.Y_LIMITS[0], to=self.Y_LIMITS[1], orient=tk.HORIZONTAL, command=self._on_slider_change)
        self.y_slider.set(self.goal_y)
        self.y_slider.grid(row=1, column=1, sticky="ew")
        self.y_label = ttk.Label(control_frame, text=f"{self.goal_y:.1f} mm")
        self.y_label.grid(row=1, column=2, padx=5)

        # Thanh trượt Z
        ttk.Label(control_frame, text="Z:").grid(row=2, column=0)
        self.z_slider = ttk.Scale(control_frame, from_=self.Z_LIMITS[0], to=self.Z_LIMITS[1], orient=tk.HORIZONTAL, command=self._on_slider_change)
        self.z_slider.set(self.goal_z)
        self.z_slider.grid(row=2, column=1, sticky="ew")
        self.z_label = ttk.Label(control_frame, text=f"{self.goal_z:.1f} mm")
        self.z_label.grid(row=2, column=2, padx=5)

        control_frame.columnconfigure(1, weight=1)

        action_frame = ttk.LabelFrame(main_frame, text="Actions", padding="10")
        action_frame.grid(row=1, column=1, pady=10, padx=10, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.home_button = ttk.Button(action_frame, text="Go Home", command=self._go_home)
        self.home_button.pack(fill=tk.X, pady=5)
        
        self.gripper_button = ttk.Button(action_frame, text="Toggle Gripper (OFF)", command=self._toggle_gripper)
        self.gripper_button.pack(fill=tk.X, pady=5)
        self.gripper_state = False

        self.emergency_button = ttk.Button(action_frame, text="Emergency Stop", command=self._emergency_stop)
        self.emergency_button.pack(fill=tk.X, pady=5)

    def _toggle_connection(self):
        if not self.is_connected:
            port = self.com_port_entry.get()
            try:
                self.arm = UArm3DOF(port=port)
                self.is_connected = True
                self.status_label.config(text="Status: Connected", foreground="green")
                self.connect_button.config(text="Disconnect")
                
                self.stop_thread = False
                self.control_thread = threading.Thread(target=self._arm_control_loop, daemon=True)
                self.control_thread.start()
                
            except Exception as e:
                messagebox.showerror("Connection Error", f"Failed to connect to {port}:\n{e}")
        else:
            self.stop_thread = True
            if self.control_thread: self.control_thread.join()
            if self.arm: self.arm.close()
            self.is_connected = False
            self.status_label.config(text="Status: Disconnected", foreground="red")
            self.connect_button.config(text="Connect")

    # ===================================================================
    #   SỬA LỖI NẰM Ở HÀM DƯỚI ĐÂY
    # ===================================================================
    def _on_slider_change(self, _=None):
        """
        Cập nhật tọa độ 'đích' từ thanh trượt.
        Hàm này được thiết kế để chạy an toàn ngay cả khi các widget label
        chưa được khởi tạo xong.
        """
        # Kiểm tra xem widget đã tồn tại chưa trước khi truy cập bằng hasattr()
        if hasattr(self, 'x_slider') and hasattr(self, 'x_label'):
            self.goal_x = self.x_slider.get()
            self.x_label.config(text=f"{self.goal_x:.1f} mm")
        
        if hasattr(self, 'y_slider') and hasattr(self, 'y_label'):
            self.goal_y = self.y_slider.get()
            self.y_label.config(text=f"{self.goal_y:.1f} mm")

        if hasattr(self, 'z_slider') and hasattr(self, 'z_label'):
            self.goal_z = self.z_slider.get()
            self.z_label.config(text=f"{self.goal_z:.1f} mm")

    def _arm_control_loop(self):
        """Vòng lặp điều khiển chính, có thêm logic làm mịn."""
        print("Control thread started.")
        while not self.stop_thread:
            start_time = time.perf_counter()
            if self.arm and self.is_connected:
                try:
                    self.commanded_x += (self.goal_x - self.commanded_x) * self.SMOOTHING_FACTOR
                    self.commanded_y += (self.goal_y - self.commanded_y) * self.SMOOTHING_FACTOR
                    self.commanded_z += (self.goal_z - self.commanded_z) * self.SMOOTHING_FACTOR

                    q1, q2, q3 = self.arm.inverse_kinematics(self.commanded_x, self.commanded_y, self.commanded_z)
                    self.arm.send_motor_angles(q1, q2, q3)

                except ValueError: pass 
                except Exception as e: print(f"Send Error: {e}")
            
            elapsed_time = time.perf_counter() - start_time
            sleep_time = (1.0 / self.UPDATE_RATE) - elapsed_time
            if sleep_time > 0: time.sleep(sleep_time)
        
        print("Control thread stopped.")

    def _go_home(self):
        if self.arm and self.is_connected:
            self.arm.send_go_home()
            home_x, home_y, home_z = 170, 0, 100 
            self.goal_x = self.commanded_x = home_x
            self.goal_y = self.commanded_y = home_y
            self.goal_z = self.commanded_z = home_z
            self.x_slider.set(home_x)
            self.y_slider.set(home_y)
            self.z_slider.set(home_z)
            self._on_slider_change()

    def _toggle_gripper(self):
        if self.arm and self.is_connected:
            self.gripper_state = not self.gripper_state
            self.arm.send_electromagnet(self.gripper_state)
            state_text = "ON" if self.gripper_state else "OFF"
            self.gripper_button.config(text=f"Toggle Gripper ({state_text})")

    def _emergency_stop(self):
        if self.arm and self.is_connected:
            self.arm.send_emergency()

    def _on_closing(self):
        if self.is_connected: self._toggle_connection()
        self.root.destroy()

if __name__ == "__main__":
    try:
        from DOFUarm import UArm3DOF
    except ImportError:
        messagebox.showerror("Import Error", "Không tìm thấy file DOFUarm.py. Hãy chắc chắn nó nằm cùng thư mục.")
        exit()

    root = tk.Tk()
    app = ArmControlGUI(root)
    root.mainloop()
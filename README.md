# Cánh tay Robot 3 bậc tự do điều khiển thời gian thực với FreeRTOS và Giao diện Python

![Python](https://upload.wikimedia.org/wikipedia/commons/1/16/Blue_Python_3.10%2B_Shield_Badge.svg)
![Arduino](https://img.shields.io/badge/Arduino%20(C++)-orange)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-red)
![License](https://img.shields.io/badge/License-MIT-green)

## Mô tả dự án

Đây là đồ án môn học "Lập trình hệ thống", nghiên cứu, thiết kế và chế tạo một hệ thống điều khiển hoàn chỉnh cho cánh tay robot 3 bậc tự do (3-DOF). Điểm cốt lõi của dự án là giải quyết vấn đề **blocking (chặn)** trong các vi điều khiển truyền thống bằng cách áp dụng **Hệ điều hành thời gian thực FreeRTOS**, kết hợp với một giao diện điều khiển (GUI) trực quan được xây dựng bằng Python.

Hệ thống cho phép người dùng điều khiển robot theo tọa độ không gian (X, Y, Z) một cách mượt mà, ổn định và có khả năng phản hồi tức thì với các lệnh quan trọng như dừng khẩn cấp.

## Hình ảnh & Video Demo

*(Đề xuất: Bạn có thể quay một video ngắn thể hiện các chức năng và chuyển thành file GIF để chèn vào đây)*

| Giao diện điều khiển (GUI) | Mô hình Robot thực tế |
| :---: | :---: |
| ![Giao diện điều khiển](https://i.ibb.co/fGqVTk16/image.png) | ![Mô hình Robot](https://i.ibb.co/MkqVgsRj/aa.png) |

## Tính năng chính

- **Điều khiển thời gian thực:** Phản hồi tức thì với lệnh của người dùng, không có độ trễ.
- **Firmware đa nhiệm (FreeRTOS):** Các tác vụ như nhận lệnh, điều khiển động cơ, gửi trạng thái chạy song song, đảm bảo hệ thống không bao giờ bị "đơ".
- **Giao diện Python trực quan:** Cho phép điều khiển robot dễ dàng thông qua các thanh trượt và nút nhấn.
- **Điều khiển theo tọa độ (Động học nghịch):** Tích hợp thuật toán Inverse Kinematics, cho phép ra lệnh cho robot di chuyển đến một điểm (X, Y, Z) cụ thể.
- **Chuyển động mượt mà:** Áp dụng thuật toán nội suy trên GUI để biến các thao tác đột ngột của người dùng thành chuyển động ổn định cho robot.
- **An toàn:** Tích hợp chức năng Homing (về gốc) và Dừng khẩn cấp (Emergency Stop) hoạt động tức thì.
- **Giao thức truyền thông tin cậy:** Xây dựng giao thức gói tin riêng qua UART có kiểm tra lỗi CRC.

## Kiến trúc hệ thống

Hệ thống được thiết kế theo mô hình Client-Server:
- **Client (PC):** Ứng dụng Python với giao diện Tkinter, chịu trách nhiệm tính toán động học và gửi lệnh.
- **Server (Vi điều khiển):** Firmware FreeRTOS trên LGT8F328P, nhận lệnh, quản lý các tác vụ và điều khiển phần cứng.

![Sơ đồ khối hệ thống](https://i.ibb.co/n4n268H/bbbb.png)

### Phần cứng
- **Vi điều khiển:** LGT8F328P (Tương thích Arduino, hiệu năng cao).
- **Cơ cấu chấp hành:** 3 Động cơ bước NEMA 17 và 1 Servo SG90.
- **Driver:** 3x A4988 với chế độ vi bước 1/16.
- **Cảm biến:** 3x Công tắc hành trình (Limit Switch) cho quá trình Homing.

### Firmware (FreeRTOS)
Chương trình được chia thành các tác vụ độc lập để đảm bảo khả năng đáp ứng:
- `Task_SerialCommander`: Luôn lắng nghe lệnh từ PC.
- `Task_StepperRunner`: Chạy nền liên tục để điều khiển động cơ một cách mượt mà.

### Phần mềm PC (Python GUI)
- **Ngôn ngữ:** Python.
- **Thư viện:**
  - `Tkinter`: Xây dựng giao diện đồ họa.
  - `PySerial`: Giao tiếp với cổng COM.
  - `NumPy`: Hỗ trợ tính toán động học.
- **Điểm nổi bật:**
  - **Đa luồng (Multi-threading):** Giao diện không bị treo khi gửi lệnh liên tục.
  - **Làm mịn chuyển động:** Sử dụng thuật toán nội suy để robot di chuyển ổn định.

## Giao thức truyền thông
Hệ thống sử dụng một giao thức gói tin nhị phân tùy chỉnh qua UART để đảm bảo tính toàn vẹn dữ liệu.

| Trường | Số byte | Mô tả |
| :--- | :---: | :--- |
| Header 1 | 1 | Byte bắt đầu cố định (`0xAA`) |
| Header 2 | 1 | Byte bắt đầu cố định (`0x55`) |
| Length | 1 | Độ dài của phần Body |
| Command | 1 | Mã lệnh (ví dụ: `0x03` Homing, `0x05` Move) |
| Option | 1 | Tùy chọn cho lệnh |
| Transition ID | 2 | ID để khớp yêu cầu và phản hồi |
| Payload | N | Dữ liệu của lệnh (tọa độ, góc quay) |
| CRC | 1 | Checksum XOR để kiểm tra lỗi |

## Hướng dẫn sử dụng

### Yêu cầu
- **Phần cứng:**
  - Cánh tay robot đã được lắp ráp đúng theo thiết kế.
  - Mạch điều khiển với LGT8F328P.
  - Cáp USB để kết nối với máy tính.
- **Phần mềm:**
  - [Arduino IDE](https://www.arduino.cc/en/software).
  - [Python 3.8+](https://www.python.org/downloads/).
  - Các thư viện Python: `pyserial`, `numpy`.
    ```bash
    pip install pyserial numpy
    ```

### Các bước thực hiện

#### Bước 1: Nạp Firmware cho Vi điều khiển

1.  Mở file `Codenaparuno.cpp` bằng phần mềm Arduino IDE.
2.  **Cài đặt Board LGT8F328P:**
    - Vào `File > Preferences`.
    - Trong ô "Additional Boards Manager URLs", dán link sau: `https://raw.githubusercontent.com/dbuezas/lgt8fx/master/package_lgt8fx_index.json`.
    - Vào `Tools > Board > Boards Manager`, tìm "LGT8F" và cài đặt.
    - Chi tiết hướng dẫn ở link: https://drive.google.com/file/d/1qtqUgPM83gQCf6iKPLxniyYX50QsaeeN/view?usp=sharing
3.  **Chọn Board và Cổng COM:**
    - `Tools > Board`: Chọn "LGT8F328P".
    - `Tools > Port`: Chọn đúng cổng COM mà robot đang kết nối.
4.  Nhấn nút **Upload** để nạp chương trình.


#### Bước 2: Chạy ứng dụng điều khiển Python

1.  Mở terminal hoặc command prompt.
2.  Di chuyển đến thư mục chứa dự án.
3.  Chạy file GUI chính:
    ```bash
    python "ai_studio_code (2).py"
    ```
    *(Lưu ý: Nếu tên file có khoảng trắng, hãy đặt trong dấu ngoặc kép)*

#### Bước 3: Vận hành Robot

1.  Trên giao diện phần mềm, nhập đúng **tên cổng COM** mà robot đang sử dụng.
2.  Nhấn nút **"Connect"**. Dòng trạng thái sẽ chuyển sang "Status: Connected" màu xanh lá.
3.  **QUAN TRỌNG:** Luôn nhấn nút **"Go Home"** đầu tiên sau khi kết nối để robot tự động về vị trí gốc.
4.  Sử dụng các thanh trượt X, Y, Z để điều khiển robot di chuyển.
5.  Sử dụng các nút "Toggle Gripper" và "Emergency Stop" cho các hành động tương ứng.
6.  Nhấn **"Disconnect"** để kết thúc phiên làm việc.

## Cấu trúc thư mục
```
.
├── ai_studio_code (2).py  # File ứng dụng GUI Python chính
├── Codenaparuno.cpp       # File firmware FreeRTOS cho vi điều khiển
├── config.json            # File cấu hình vị trí Home của robot
├── DOFUarm.py             # Lớp Python driver, chứa logic động học và giao thức
├── Main.py                # File demo đơn giản, không có GUI
└── README.md              # File hướng dẫn này
```

## Tác giả
- **Nguyễn Long Thiên Thuận** – 22050096
- **Nguyễn Trọng Hiếu** - 22050095

## Giảng viên hướng dẫn
- **Lê Duy Hùng**

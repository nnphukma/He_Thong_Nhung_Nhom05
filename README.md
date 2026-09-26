<=============================================================================================================================================================================>
                                                  NỘI DUNG ĐỀ TÀI HỆ THỐNG NHÚNG
<=============================================================================================================================================================================>

* Đề tài:	Bệ cân bằng tự động 2 trục (gimbal) dùng MPU6050
  
* Đo góc nghiêng bằng MPU6050 (gia tốc + con quay), kết hợp bằng bộ lọc bù (complementary filter); điều khiển 2 servo giữ mặt trên luôn nằm ngang khi đế bị nghiêng; hiển thị OLED.
  
* Phần cứng chính: STM32, MPU6050, 2 servo SG90/MG90, khung gimbal (in 3D/bìa cứng), OLED, nguồn servo riêng
  
* Ngoại vi & Kiến thức: HAL I2C, lấy mẫu định kỳ bằng Timer, complementary filter, PWM servo, điều khiển P/PI
  
* Sinh viên 1: Đọc MPU6050 200 Hz theo ngắt Timer hoặc chân INT; hiệu chuẩn bias con quay
  
* Sinh viên 2: Ước lượng góc: từ gia tốc, tích phân gyro, bộ lọc bù; so sánh 3 cách
  
* Sinh viên 3: Điều khiển servo PWM theo µs, bộ điều khiển bù góc, giới hạn tốc độ servo
  
* Sinh viên 4: Cơ khí + nguồn; OLED / UART vẽ đồ thị góc; đánh giá đáp ứng khi lắc đế; báo cáo
  
* Demo/ Tiêu chí đạt: Nghiêng đế ±30°, mặt trên giữ ngang trong ±3°; đồ thị so sánh góc thô và góc sau lọc
  
* Mở rộng: Thay bộ lọc bù bằng Kalman 1D

<=============================================================================================================================================================================>
                                                  CONFIG IDE + TOOLCHAIN
<=============================================================================================================================================================================>
STM32 CMake Base Project

Base project dùng chung cho STM32F103C8T6, được xây dựng với VS Code + CMake + Ninja + ARM GNU Toolchain + OpenOCD.

Mục tiêu của repository là cung cấp một cấu trúc project STM32 thống nhất để các thành viên trong nhóm có thể clone, build, flash và debug mà không phải thiết lập lại project từ đầu.

1. Tổng quan

Project hiện sử dụng:

MCU: STM32F103C8T6 / STM32F103C8Tx

IDE: Visual Studio Code

Build system: CMake

Build generator: Ninja

Compiler: arm-none-eabi-gcc

Debugger: arm-none-eabi-gdb

Programmer / Debug server: OpenOCD

Debug interface: ST-LINK / SWD

Thư viện STM32: STM32CubeF1 HAL

Sinh mã: STM32CubeMX

CPU: ARM Cortex-M3

System clock: 72 MHz

2. Mục tiêu của Base Project

Project được thiết kế để:

Có một cấu trúc STM32 thống nhất cho cả nhóm.

Build bằng CMake + Ninja.

Viết code và debug trực tiếp trong VS Code.

Dùng OpenOCD để flash và debug thông qua ST-LINK.

Nhấn F5 để build, flash và bắt đầu debug.

Có task riêng để Flash Only khi không cần debug.

Tách code ứng dụng và code driver phần cứng.

Tự động phát hiện các file driver mới trong Hardware/Src.

Có thể clone repository và sử dụng lại cho các project STM32F103 khác.

3. Cấu trúc thư mục

STM32_Cmake_Base/
├── .vscode/
│   ├── launch.json
│   └── tasks.json
│
├── Core/
│   ├── Inc/
│   └── Src/
│
├── Hardware/
│   ├── Inc/
│   └── Src/
│
├── Drivers/
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/
│
├── cmake/
│   ├── stm32cubemx/
│   ├── gcc-arm-none-eabi.cmake
│   └── stm-clang.cmake
│
├── build/                    # Sinh tự động, không commit
│
├── CMakeLists.txt
├── CMakePresets.json
├── STM32_Cmake_Base.ioc
├── STM32F103XX_FLASH.ld
├── startup_stm32f103xb.s
├── .gitignore
└── README.md

Chức năng của từng thư mục

Thư mục

Chức năng

Core/Inc

Header của ứng dụng và code do CubeMX sinh

Core/Src

Source của ứng dụng và code do CubeMX sinh

Hardware/Inc

Header của các driver phần cứng tự viết

Hardware/Src

Source của các driver phần cứng tự viết

Drivers/

CMSIS và STM32 HAL

cmake/

Cấu hình CMake và các file CMake của CubeMX

.vscode/

Cấu hình build, flash và debug dùng chung

build/

File build sinh ra ở máy local

4. Cấu trúc Hardware

Các driver phần cứng tự viết được đặt trong hai thư mục:

Hardware/
├── Inc/
│   ├── mpu6050.h
│   ├── oled.h
│   └── servo.h
│
└── Src/
    ├── mpu6050.c
    ├── oled.c
    └── servo.c

Trong code có thể include trực tiếp:

#include "mpu6050.h"

Không cần viết đường dẫn tương đối:

#include "../Hardware/Inc/mpu6050.h"

CMake tự động phát hiện driver

CMakeLists.txt sử dụng:

file(GLOB HARDWARE_SOURCES
    CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/Hardware/Src/*.c
)

Vì vậy khi thêm driver mới:

Hardware/Inc/new_driver.h
Hardware/Src/new_driver.c

thì thông thường không cần sửa CMakeLists.txt.

CMake sẽ tự phát hiện file .c mới và đưa nó vào quá trình build.

Hardware/Inc được khai báo là include directory nên các header trong đó có thể được include trực tiếp.

5. Phần mềm cần cài đặt

Trước khi sử dụng project, máy tính cần có các công cụ sau:

ARM GNU Toolchain

Kiểm tra:

arm-none-eabi-gcc --version
arm-none-eabi-gdb --version

CMake

cmake --version

Ninja

ninja --version

OpenOCD

openocd --version

Git

git --version

Extension VS Code

Khuyến nghị cài:

C/C++

CMake Tools

Cortex-Debug

6. Cấu hình project

Mở thư mục project bằng VS Code:

STM32_Cmake_Base/

Project cung cấp hai CMake preset:

Debug
Release

Có thể cấu hình Debug bằng PowerShell:

cmake --preset Debug

7. Build project

Build Debug

cmake --build --preset Debug

Build Release

cmake --build --preset Release

Build ở chế độ verbose

cmake --build --preset Debug --verbose

File ELF của Debug nằm tại:

build/Debug/STM32_Cmake_Base.elf

8. Build bằng VS Code

Project có sẵn các task trong:

.vscode/tasks.json

Quy trình thông thường:

Sửa code → Build → Flash / Debug

Cấu hình build sử dụng CMake preset Debug.

9. Flash Only

Project có task riêng để chỉ flash firmware mà không mở phiên debug.

Quy trình:

OpenOCD → GDB → Load ELF → Reset → Detach

Chế độ này phù hợp khi chỉ muốn nạp firmware vào STM32 và chạy chương trình bình thường.

10. Debug bằng F5

Nhấn:

F5

Cortex-Debug sẽ tự động khởi động OpenOCD dựa trên:

.vscode/launch.json

Chuỗi debug:

ST-LINK → SWD → OpenOCD → Cortex-M3

Debugger được cấu hình để chạy đến:

main()

Không cần mở OpenOCD thủ công trước khi nhấn F5.

11. Cấu hình OpenOCD

Project sử dụng:

interface/stlink.cfg
target/stm32f1x.cfg

và chọn transport:

transport select hla_swd

TCL port sử dụng:

6667

Port này được chọn để tránh xung đột với các phần mềm khác có thể sử dụng port mặc định.

12. STM32CubeMX

File cấu hình CubeMX:

STM32_Cmake_Base.ioc

Khi cần thay đổi peripheral hoặc clock:

Mở file .ioc bằng STM32CubeMX.

Thực hiện thay đổi cần thiết.

Generate code.

Kiểm tra các file thay đổi.

Build lại bằng CMake.

Lưu ý

Code driver phần cứng tự viết nên đặt tại:

Hardware/Inc
Hardware/Src

Không nên đặt driver tự viết trực tiếp vào các thư mục do CubeMX quản lý nếu không có lý do đặc biệt.

13. Cấu hình Clock

Cấu hình clock hiện tại:

HSE 8 MHz → PLL ×9 → SYSCLK 72 MHz
                         ↓
                    AHB 72 MHz
                         ↓
                    APB1 36 MHz
                    APB2 72 MHz

14. Git

Repository sử dụng Git để quản lý source code và chia sẻ trong nhóm.

Sau khi clone:

git clone <repository-url>
cd STM32_Cmake_Base

Cấu hình và build:

cmake --preset Debug
cmake --build --preset Debug

Thư mục:

build/

được loại khỏi Git thông qua .gitignore.

Mỗi thành viên sẽ tự tạo build directory trên máy của mình.

15. Quy trình làm việc của nhóm

Quy trình cơ bản:

Pull code mới nhất
→ Configure / Build
→ Viết hoặc sửa code
→ Test trên phần cứng
→ Commit
→ Push

Khi tạo driver phần cứng mới:

Tạo .h → Hardware/Inc
Tạo .c → Hardware/Src
→ #include "driver.h"
→ Build

Không cần sửa CMakeLists.txt cho mỗi driver mới.

16. Phần cứng đã kiểm tra

Base project đã được kiểm tra với:

STM32F103C8T6

ST-LINK V2

SWD

OpenOCD

Cortex-Debug

LED onboard PC13

Các chức năng đã kiểm tra:

Build bằng CMake + Ninja

Flash bằng OpenOCD

Debug bằng Cortex-Debug

F5 từ VS Code

Dừng tại main()

Breakpoint

Step code

Continue

Chạy chương trình trên STM32
<img width="3664" height="128" alt="image" src="https://github.com/user-attachments/assets/7642d0ce-a797-4463-8655-a036bca2013d" />


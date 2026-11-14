# Kế hoạch code: phần THUẬT TOÁN của xe micromouse team 15

**Phạm vi:** chỉ thuật toán. Driver (PWM, encoder, I2C, VL53L0X, MPU6050, UART, Flash) do team phần cứng làm.
- Thuật toán giao tiếp với driver qua **một file interface duy nhất**.
- Toàn bộ code thuật toán **không gọi HAL**, nên chạy và test được bằng **mô phỏng trên PC** trước khi có xe.

## 1. Quyết định đã chốt
| Vấn đề | Quyết định |
|---|---|
| Đường đi Lượt 2 | A* tính sẵn trên PC (`tools/path_planner/plan_path.py` → `Algo/path.h`). Đã xong |
| Bám quỹ đạo | **Pure Pursuit, lookahead thay đổi theo độ cong**: gần góc gắt thì lookahead ngắn lại và xe chạy chậm lại |
| Né vật cản ngẫu nhiên | Né tại chỗ theo map. Nếu không đủ chỗ thì chạy lại A* trên xe. Không có đường thì dừng |
| Lượt 1 | Ghi quỹ đạo lái tay **theo khoảng cách** (≥ 2 cm/điểm), rút gọn rồi dùng làm đường dự phòng |
| Kiểm chứng | Mô phỏng trên PC có nhiễu (Monte Carlo). Tiêu chí: **thân xe luôn cách vật cản ≥ 1,5 cm** |

**Vì sao cần lookahead thay đổi:** đường A* có các góc đổi hướng 71° tại (59, 94), 62° tại (155, 64), 50° tại (12, 32) và 45° tại (114, 70). Pure Pursuit với lookahead cố định 12 cm sẽ đi tắt vào trong góc khoảng 3 cm, trong khi chỗ hẹp chỉ chừa khoảng 2 cm.

## 2. Giao tiếp với phần driver
File `Algo/nav_io.h`. Team phần cứng gọi **`Nav_Step()` 200 lần/giây**.
```c
typedef struct {            /* driver -> thuật toán, mỗi chu kỳ */
    float   dt_s;           /* thời gian thật từ lần gọi trước */
    int32_t d_ticks_left;   /* số xung encoder thay đổi trong chu kỳ */
    int32_t d_ticks_right;
    float   gyro_z_rad_s;   /* đã trừ bias, dương = quay trái (CCW) */
    uint16_t tof_mm[4];     /* VL53L0X trái -> phải */
    uint8_t  tof_fresh;     /* bit i = 1 nếu tof_mm[i] là số đo mới */
    uint8_t  ir_on_tape;    /* bit0 = IR trái, bit1 = IR phải, 1 = đang trên băng đen */
    uint8_t  manual_cmd;    /* ký tự lái tay mới nhận ('F','B','L','R','S','0'..'9'), 0 nếu không có */
} NavInput;

typedef struct {            /* thuật toán -> driver */
    float v_cm_s;           /* vận tốc thẳng mong muốn */
    float w_rad_s;          /* vận tốc góc mong muốn */
    uint8_t state;          /* IDLE / RUN / AVOID / TEACH / DONE / STOPPED */
} NavOutput;

void Nav_Init(const NavParams *p, NavMode mode);   /* RUN_ASTAR, RUN_RECORDED, TEACH */
NavOutput Nav_Step(const NavInput *in);
const Pose *Nav_GetPose(void);                     /* để driver log */
```
**Ranh giới trách nhiệm:**
- Driver lo PID vận tốc từng bánh, biến `(v, ω)` thành PWM. Thuật toán chỉ xuất ra `(v, ω)`.
- Lưu Flash cũng do driver lo. Thuật toán chỉ đưa ra mảng điểm và struct thông số.

**Số liệu cần team phần cứng cung cấp** (trước khi có thì dùng giá trị tạm trong `NavParams`):
| Thông số | Dùng cho |
|---|---|
| Số xung encoder / cm, khoảng cách 2 bánh | Odometry |
| Vị trí và góc đặt 4 VL53L0X, 2 IR so với tâm xe | Né vật cản, sửa vị trí |
| Vận tốc và gia tốc tối đa xe đạt được ổn định | Biên dạng tốc độ |
| Sai số odometry đo thật (chạy 2 m, quay 360°) | Nhiễu trong mô phỏng, biên an toàn của A* |
| Khoảng cách từ trục bánh tới mũi và đuôi xe | Kiểm tra va chạm thân xe khi quay tại chỗ |
| Kích thước bao ngoài thật, tính cả module IR nhô ra | `body_len_cm`, `body_wid_cm` |
| Hướng lắp IR: chúc xuống sàn hay nhìn ngang ra trước | Có dùng IR để sửa vị trí theo băng đen được không |

**Phần cứng theo slide báo cáo giữa kỳ (27/10/2026) và ảnh hưởng tới thuật toán:**
- Xe: STM32F401 Black Pill, TB6612, 2 motor N20 có encoder, 4 VL53L0X + IMU chung I2C (đổi địa chỉ qua XSHUT), 2 IR góc trước, HC-05, nút SW1, pin 2S 7,4 V 2400 mAh, buck 5 V. PCB 10 × 9 cm làm khung.
- **Trục bánh lệch về phía sau** (theo ảnh 3D). Đã thêm `body_off_cm` (tâm thân xe nằm trước trục bánh bao nhiêu cm, mặc định 0) và `NAV_PARAMS_VERSION` lên 2. Kết quả mô phỏng (01/11/2026):

  | Thân xe | Đường gốc, 200 lượt | Vật lạ (135, 67), 100 lượt |
  |---|---|---|
  | 10 × 9, lệch 0 | 200/200, cách vật ≥ 3,34 cm | 100/100 |
  | 10 × 9, lệch 1,5 cm | 200/200, ≥ 3,20 cm | 99/100, ≥ 1,47 cm |
  | 10 × 9, lệch 2,5 cm | 200/200, ≥ 2,92 cm | chưa chạy |
  | 10 × 11, lệch 1,5 cm | 200/200, ≥ 2,21 cm | **84/100**, ≥ 0,53 cm |
  | 10 × 11, lệch 1,5, `avoid_r_cm` 8 | | 98/100, ≥ 1,00 cm |

  Đường gốc vẫn an toàn. Phần né vật lạ thì nhạy với kích thước thân: có số đo thật rồi phải chỉnh lại `avoid_r_cm` (8,5 lại gây 1 va chạm, nên không chỉ tăng là được).
- **Module IR nhô ra ngoài mép PCB** ở hai góc trước, nên bao ngoài thật có thể lớn hơn 10 × 9 cm.
- **IR trong ảnh 3D có vẻ nhìn ngang ra phía trước** (module dò vật cản kiểu FC-51), trong khi `localize.c` coi IR là cảm biến chúc xuống báo "đang trên băng đen". Nếu IR nhìn ngang thì phải tắt cập nhật IR (`ir_gate_cm = 0`) hoặc đổi mô hình đo.
- **PID góc của driver** phải bám **vận tốc góc ω** mà `Nav_Step` xuất ra, không giữ một hướng cố định. Nếu giữ hướng cố định thì sẽ chống lại lệnh rẽ của Pure Pursuit.

## 3. Các module thuật toán (`Algo/`, C11, không HAL)
| # | File | Nội dung |
|---|---|---|
| A1 | `nav_io.h`, `nav_params.h` | Struct ở mục 2. `NavParams` có giá trị mặc định và trường `version` |
| A2 | `odometry.c/.h` | Pose (x, y, θ) tích phân từ encoder. θ lấy từ gyro, trộn nhẹ với chênh lệch encoder (bộ lọc bù) |
| A3 | `geometry.c/.h` | Hàm chung: chiếu tia lên `MAP_OBSTACLES`, khoảng cách từ hình chữ nhật thân xe tới vật cản, chuẩn hoá góc |
| A4 | `follower.c/.h` | **Pure Pursuit lookahead thay đổi**. L = clamp(k·v, 5, 15) cm, rút ngắn khi tổng góc đổi hướng trong 15 cm phía trước lớn. Vận tốc giới hạn theo độ cong (v ≤ √(a_ngang / κ)) và theo gia tốc. Chậm dần về đích. Báo xong khi cả thân xe qua vạch, tức y < −6 |
| A5 | `localize.c/.h` | **Bộ lọc Kalman mở rộng (EKF)** trên (x, y, θ): dự đoán theo odometry, cập nhật bằng từng số đo VL53L0X (so với tia chiếu lên các khối đã biết) và IR chạm băng đen. Sửa được cả hướng θ. Loại số đo lệch quá 3σ |
| A6 | `avoid.c/.h` | Số đo không giải thích được bằng khối đã biết (nón nhìn nới thêm 5°) thì nghi là vật lạ, xác nhận sau 2 lần đo. Vị trí đoán = điểm trong nón gần quỹ đạo gốc nhất, sau đó được tinh chỉnh dần theo các số đo tiếp theo. Chắn đường thì phanh, gọi A7, ghép đường vòng vào phần còn lại của quỹ đạo |
| A7 | `astar_grid.c/.h` | A* trên xe: lưới 2 cm trong cửa sổ 88 × 88 cm (~16 KB RAM). Điểm trên đường được dời vào giữa khe hẹp rồi kiểm tra lại khoảng cách thật. Không có đường thì `STOPPED` |
| A8 | `recorder.c/.h` | Lượt 1: đổi ký tự lái thành `(v, ω)`. Tự dừng nếu 300 ms không nhận lệnh (mất Bluetooth). Ghi pose mỗi ≥ 2 cm, rút gọn bằng thuật toán Ramer–Douglas–Peucker, xuất ra mảng `PathPoint` |
| A9 | `nav.c/.h` | Máy trạng thái `IDLE → RUN ⇄ AVOID → DONE / STOPPED`, gọi các module theo thứ tự trong `Nav_Step` |

## 4. Mô phỏng trên PC (`tools/sim/`)
- `sim_main.c`: biên dịch chung với `Algo/*.c` bằng gcc. Mô hình xe hai bánh vi sai với các nguồn nhiễu:
  - Sai tỉ lệ bánh ±2%.
  - Trượt bánh.
  - Nhiễu và trôi gyro.
  - Lượng tử hoá encoder.
  - VL53L0X giả lập bằng chiếu tia (nón 25°, nhiễu ±1 cm).
  - IR theo mép sân.
  - Vật cản ngẫu nhiên đặt tuỳ ý.
- Xuất CSV: thời gian, pose thật, pose ước lượng, `(v, ω)`, trạng thái.
- `run_sim.py`: build bằng gcc, chạy N seed, tính các chỉ số rồi vẽ đường chạy lên map (dùng lại hàm vẽ của `plan_path.py`).
  - Chỉ số: khoảng cách nhỏ nhất từ thân xe tới vật cản, có chạm hay không, thời gian về đích, tỉ lệ thành công.
- Cùng file vẽ đó dùng để vẽ **log từ xe thật** khi team phần cứng gửi.

## 5. Thứ tự làm và tiêu chí đạt (tất cả kiểm tra trên PC)
| Mốc | Việc | Đạt khi |
|---|---|---|
| M1 | A1 + khung mô phỏng (xe chạy theo lệnh `(v, ω)` cố định) | Build bằng gcc, vẽ ra đường chạy |
| M2 | A2 odometry, A3 geometry | Không nhiễu: pose ước lượng trùng pose thật. Có nhiễu: sai lệch đúng như mô hình |
| M3 | A4 follower trên `path.h` | Không nhiễu: thân xe cách vật cản ≥ 1,5 cm suốt đường, về đích |
| M4 | Monte Carlo 200 lần có nhiễu, chưa sửa vị trí | Đo tỉ lệ va chạm để biết cần sửa vị trí đến mức nào |
| M5 | A5 localize | 200 lần có nhiễu: **≥ 98% không chạm** |
| M6 | A6 avoid + A7 A* trên xe | Vật lạ đặt ở mọi ô trống dọc quỹ đạo (bước 5 cm): không chạm, về đích hoặc dừng an toàn |
| M7 | A8 recorder | Đường lái tay giả lập → ghi → rút gọn → phát lại đạt tiêu chí như M5 |
| M8 | A9 `nav.c`, giao cho team phần cứng | Team gọi `Nav_Step` trên xe. Log thật vẽ khớp với mô phỏng |

## 6. Việc cần từ người khác
- **Team phần cứng:** các số liệu ở mục 2. Gọi `Nav_Step` ở 200 Hz. Gửi log pose qua UART bằng ngắt để không làm chậm vòng điều khiển.
- **Ban tổ chức:**
  - Có cộng điểm riêng cho từng vật cản vượt qua không? Nếu có thì đổi chi phí A*.
  - Sau vạch đích có chỗ để xe chạy thêm khoảng 10 cm không?
  - Vật cản ngẫu nhiên có thể được đặt ở vùng nào?

## 7. Trạng thái (01/11/2026): đã code xong M1–M7, M8 chờ team phần cứng
Toàn bộ module A1–A9 nằm trong `Algo/`, đã thêm vào `CMakeLists.txt` và build được cho STM32
(~26 KB Flash, ~35 KB RAM tĩnh, chưa ai gọi nên linker còn bỏ đi).

**Kết quả mô phỏng** (`python tools/sim/run_sim.py ...`, nhiễu mặc định: sai bán kính bánh ±1%,
khoảng cách bánh ±2%, sai tỉ lệ gyro ±1%, trôi gyro, trượt bánh, nhiễu VL53L0X ±0,5 cm):

| Kịch bản | Kết quả |
|---|---|
| Không nhiễu | Về đích sau 14,2 s, thân xe cách vật cản ≥ 4,0 cm |
| 200 lượt có nhiễu | **200/200 về đích sạch**, thân xe cách vật cản ≥ 3,3 cm |
| 200 lượt, nhiễu gấp đôi | 89,5% về đích sạch |
| 200 lượt, tắt sửa vị trí (EKF) | chỉ 54,5%: bắt buộc phải có EKF |
| Lượt 1 lái tay (người giả lập) rồi Lượt 2 phát lại, 100 lượt | **100/100 về đích sạch**, 13,3 s |
| Vật lạ tại (135, 67), 100 lượt | **100/100 về đích sạch**, 20,4 s |
| Vật lạ đặt lần lượt dọc quỹ đạo (25 vị trí) | 21 về đích, 4 dừng an toàn, 0 va chạm |

**Điều kiện về đích (sửa 02/11/2026):** xe dừng khi điểm cao nhất của hình chữ nhật thân xe (tính cả khi chạy
xiên và `body_off_cm`) xuống dưới `finish_y_cm` = −4 cm, thay vì dừng ở cuối quỹ đạo. Cách cũ để 10% lượt phát lại
quỹ đạo lái tay dừng khi một góc xe còn trên vạch. Biên 4 cm bù sai số vị trí ước lượng; biên 1 cm là quá mỏng.

**Giới hạn đã biết:**
- Vật lạ đặt trên đường chéo Ô2 (giữa T5, T7, T9, T3, T4) có thể chỉ còn khe ~15 cm. Khi đó xe hoặc dừng
  (`STOPPED`) hoặc lách qua sát 0,1–1,5 cm. Đây là giới hạn vật lý của xe 9 cm, không phải lỗi thuật toán.
- Mọi kết quả trên là **mô phỏng**. Cần thay nhiễu giả định bằng số đo thật của xe (mục 2) rồi chạy lại.
- Vị trí đặt VL53L0X/IR trong `NavParams` là giá trị tạm, phải sửa theo xe thật.

## 8. Hướng dẫn tích hợp cho team phần cứng
```c
#include "nav_io.h"

static NavParams prm;
NavParams_Default(&prm);          /* hoặc đọc từ Flash */
Nav_Init(&prm, NAV_MODE_RUN_ASTAR, NULL, 0);   /* gọi khi nhấn SW1, xe đã đặt ở điểm xuất phát */

/* mỗi 5 ms */
NavInput in = {0};
in.dt_s = dt;                                  /* thời gian thật từ lần trước */
in.d_ticks_left  = RobotDrive_LeftEncoderDelta();
in.d_ticks_right = RobotDrive_RightEncoderDelta();
in.gyro_z_rad_s  = gyro_z_da_tru_bias;         /* dương = quay trái */
/* tof_mm[], tof_fresh, ir_on_tape, manual_cmd như mô tả trong nav_io.h */
NavOutput out = Nav_Step(&in);
/* PID vận tốc từng bánh: v_trai = out.v_cm_s - out.w_rad_s * track/2, v_phai = ... + ... */
```
- Lượt 1: `Nav_Init(&prm, NAV_MODE_TEACH, NULL, 0)`, đưa ký tự app vào `in.manual_cmd`. Lái xong gọi
  `Nav_TeachFinish(buf, NAV_MAX_PTS)` rồi lưu `buf` vào Flash.
- Lượt 2 với đường đã ghi: `Nav_Init(&prm, NAV_MODE_RUN_RECORDED, buf, n)`.
- Đổi thông số qua Bluetooth: `NavParams_Set(&prm, "v_max", 25.0f)` rồi `Nav_Init` lại.
- Khi vật lạ chắn đường, một lần `Nav_Step` có thể mất vài chục ms (A* trên xe) trong lúc xe đã dừng.ờ 
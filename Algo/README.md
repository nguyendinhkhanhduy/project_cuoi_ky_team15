# Algo – thuật toán dẫn đường

C11 thuần, **không include HAL**. Nhờ vậy cùng một code chạy được trên STM32 và trong mô phỏng `tools/sim`.

- **Đơn vị:** cm, cm/s, rad.
- **Toạ độ sân:** gốc ở góc dưới-trái, x sang phải (0–180), y lên trên (0–120). θ đo từ trục x, dương khi quay trái.
- **Vạch:** vạch xuất phát ở y = 0, x 0–60; vạch đích ở y = 0, x 120–180.

## Thư mục con

| Thư mục | File | Vai trò |
|---|---|---|
| `core/` | `nav.c`, `nav_io.h`, `nav_params.*` | Giao tiếp với App (`Nav_Init`, `Nav_Step`), máy trạng thái, thông số chỉnh được |
| `common/` | `geometry.*`, `nav_path.*` | Hình học (khoảng cách, chiếu tia, va chạm thân xe); quỹ đạo gấp khúc tra theo độ dài cung |
| `localization/` | `odometry.*`, `localize.*` | Odometry (encoder + gyro, bộ lọc bù); EKF cập nhật bằng ToF so với map và IR so với băng đen |
| `planning/` | `nav_map.*`, `avoid.*`, `astar_grid.*`, `path.h` | Map tĩnh + vật lạ; phát hiện vật lạ và lập đường vòng; A* trên xe; quỹ đạo A* sinh sẵn |
| `control/` | `follower.*` | Pure Pursuit, lookahead thay đổi theo góc cua, giới hạn tốc độ, điều kiện về đích |
| `teach/` | `recorder.*` | Lượt 1: ký tự lái → (v, ω), ghi quỹ đạo, rút gọn RDP |

Include viết phẳng (`#include "geometry.h"`), nên mọi thư mục con đều phải có trong include path.

## Luồng mỗi chu kỳ (`Nav_Step`, 200 Hz)

```
Odo_Update → Loc_Predict → Loc_Update (chỉ map tĩnh)
  → TEACH: Rec_Command + Rec_Track
  → RUN / AVOID / EDGE: phản xạ biên (IR) → phát hiện vật lạ (ToF / IR) → BRAKE → Avoid_Plan → Fol_Step
```

**Trạng thái:** `IDLE`, `RUN`, `BRAKE`, `AVOID`, `TEACH`, `DONE`, `STOPPED`, `EDGE`.
`EDGE` được thêm ở cuối enum để không đổi số của các trạng thái cũ.

## Quy tắc quan trọng

- **EKF chỉ định vị theo vật cản đã biết** (`NavMap_KnownRects`). Vật lạ chỉ dùng để né.
- **Phát hiện vật lạ:** ToF đo ngắn hơn khoảng cách map dự đoán quá `avoid_margin_cm`, xác nhận sau `avoid_confirm` lần. Khi so với map, nón quét được nới thêm 5–8° để bù sai số hướng.
- **Danh sách vật lạ:** tối đa 3. Khi đầy thì ghi đè vật cũ nhất.
- **A* trên xe:**
  - Thử lưới 2 cm (cửa sổ 88 cm), rồi lưới 3 cm (cửa sổ 132 cm).
  - Sau khi tìm được đường: dời điểm vào giữa khe hẹp, rồi nối tắt với khoảng cách tối thiểu 9,5 cm.
  - Không có đường với `r_hard_cm` thì thử lại với `r_tight_cm` (lách khe chỉ đủ chạy thẳng).
- **Về đích:** điểm cao nhất của thân xe xuống dưới `finish_y_cm` (−4 cm). Ngoài ra kết thúc khi đã hết quỹ đạo và thân xe đã qua vạch, hoặc khi đã quá cuối quỹ đạo 15 cm.
- **Phản xạ biên** (`ir_mode = 0`): IR thấy băng đen thì xe lùi và xoay ra xa vạch. Lặp quá 6 lần mà không tiến được 10 cm thì chuyển sang `STOPPED`.
- **Thông số:** nằm trong `NavParams` (`core/nav_params.c`), chỉnh theo tên bằng `NavParams_Set`/`Get`. Đổi bố cục struct thì phải tăng `NAV_PARAMS_VERSION`.
- **`planning/path.h`:** do `tools/path_planner/plan_path.py` sinh ra, không sửa tay. Chỉ `nav_map.c` được include file này.

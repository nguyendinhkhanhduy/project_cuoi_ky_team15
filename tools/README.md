# tools – công cụ trên PC

Cần Python 3 (numpy, scipy, matplotlib) và gcc trên PATH. Chạy các lệnh từ thư mục `robot_firmware/`.

| File | Việc |
|---|---|
| `path_planner/plan_path.py` | A* offline lưới 1 cm, 8 hướng, chi phí `1 + 4p²` khi sát vật (R_HARD 7,5, R_SOFT 15 cm), làm mượt → `Algo/planning/path.h` + `out/path.png` |
| `sim/sim_main.c` | Xe và sân ảo có nhiễu (bánh, trượt, gyro, encoder, ToF, IR) gọi `Nav_Step`; in ra `RESULT ...` |
| `sim/run_sim.py` | Build `Algo/**/*.c` + `sim_main.c` bằng gcc, chạy nhiều lượt, thống kê, vẽ hình vào `sim/out/` |
| `sim/plot_log.py` | Vẽ log xe thật (`$log 1`) lên sân |

## Lệnh

```sh
python tools/path_planner/plan_path.py           # sửa map / R_HARD / R_SOFT trong script rồi chạy lại
python tools/sim/run_sim.py --runs 200           # Monte Carlo
python tools/sim/run_sim.py --obstacle 135,67    # thêm vật lạ (lặp được)
python tools/sim/run_sim.py --mode teach         # Lượt 1 (người lái giả lập) rồi Lượt 2
python tools/sim/run_sim.py --sweep              # vật lạ đặt lần lượt dọc quỹ đạo
python tools/sim/run_sim.py --noise 2 --set odo_sigma_frac=0.06   # đổi nhiễu / thông số
python tools/sim/run_sim.py --gif                # ảnh động
python tools/sim/plot_log.py log.txt --sim tools/sim/out/run_noise0.csv
```

**Tiêu chí "về đích sạch":** trạng thái `DONE` và thân xe luôn cách vật cản ít nhất 1,5 cm.
Build không in gì nghĩa là không có cảnh báo (`-Wall -Wextra`).

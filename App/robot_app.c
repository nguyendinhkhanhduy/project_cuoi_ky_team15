#include "robot_app.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "bench.h"
#include "board_config.h"
#include "flash_store.h"
#include "inputs.h"
#include "motor.h"
#include "mpu6050.h"
#include "nav_io.h"
#include "nav_path.h"
#include "tof.h"
#include "uart_io.h"
#include "wheel_ctrl.h"

#define CONTROL_PERIOD_MS  5U      /* Nav_Step 200 lần/giây, như thiết kế trong docs/ke_hoach_code.md */
#define GYRO_CAL_SAMPLES   200U    /* ~1 s, xe phải đứng yên */
#define DEBOUNCE_MS        25U
#define LONG_PRESS_MS      1000U
#define DRIVE_Q_LEN        8U
#define LINE_LEN           48U
#define MAX_CTRL_GAP_MS    50U     /* = DT_MAX_S của nav.c: chậm hơn thì Nav_Step cắt dt, odometry sai */
#define LOG_EVERY          20U     /* log pose mỗi 20 chu kỳ = 100 ms (~40 byte, 9600 baud chịu được) */
#define STREAM_MS          200U
#define STORE_LAYOUT       1U
#define STORE_VERSION      ((NAV_PARAMS_VERSION << 16) | STORE_LAYOUT)

typedef enum { APP_IDLE, APP_TEACH, APP_RUN, APP_BENCH } AppState;

/* Mọi thứ cần giữ qua lần tắt nguồn nằm chung một struct để lưu Flash một lần. */
typedef struct {
    NavParams prm;
    WheelGains gains;
    uint16_t rec_n;            /* quỹ đạo Lượt 1 */
    uint16_t reserved;
    NavPoint rec[NAV_MAX_PTS];
} AppStore;

static AppStore cfg;
static AppState app = APP_IDLE;
static WheelCtrl wheel;
static bool imu_ok;
static uint8_t tof_mask;
static uint16_t tof_mm[TOF_COUNT];
static uint32_t last_ctrl_ms;

static char drive_q[DRIVE_Q_LEN];
static uint8_t dq_head, dq_tail;
static char idle_speed;    /* lệnh tốc độ '0'..'9'/'q' nhận lúc chưa chạy, áp dụng khi bắt đầu Lượt 1 */
static char line[LINE_LEN];
static uint8_t line_len;

static bool log_on;        /* $log 1: gửi pose mỗi 100 ms khi chạy */
static uint8_t log_div;
static NavState last_nav_state = NAV_IDLE;
static bool stream_on;     /* $stream 1: in cảm biến mỗi 200 ms khi đứng yên */
static uint32_t stream_ms;
static int32_t stream_l, stream_r;

/* ---------------------------------------------------------------- Hàng đợi lệnh lái 1 ký tự */
static void dq_flush(void) { dq_tail = dq_head; }

static void dq_push(char c)
{
    uint8_t next = (uint8_t)((dq_head + 1U) % DRIVE_Q_LEN);
    if (next == dq_tail) dq_tail = (uint8_t)((dq_tail + 1U) % DRIVE_Q_LEN);  /* đầy: bỏ lệnh cũ nhất */
    drive_q[dq_head] = c;
    dq_head = next;
}

static char dq_pop(void)
{
    if (dq_tail == dq_head) return 0;
    char c = drive_q[dq_tail];
    dq_tail = (uint8_t)((dq_tail + 1U) % DRIVE_Q_LEN);
    return c;
}

static bool is_speed_cmd(char c) { return (c >= '0' && c <= '9') || c == 'q'; }

static bool is_drive_cmd(char c)
{
    return c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'G' || c == 'I' || c == 'H' || c == 'J';
}

/* ---------------------------------------------------------------- Chuyển trạng thái */
static void stop_motors(void)
{
    Motor_Stop();
    Motor_Enable(false);
    Wheel_Reset(&wheel);
}

static void begin_motion(AppState s)
{
    /* Bỏ phần encoder tích luỹ lúc đứng yên để chu kỳ đầu không có bước nhảy. */
    (void)Encoder_LeftDelta();
    (void)Encoder_RightDelta();
    Wheel_Reset(&wheel);
    dq_flush();
    stream_on = false;
    log_div = 0;
    last_ctrl_ms = HAL_GetTick();
    app = s;
    Motor_Enable(true);
}

static void start_nav(NavMode mode, const NavPoint *path, uint16_t n)
{
    Nav_Init(&cfg.prm, mode, path, n);
    last_nav_state = Nav_GetState();
    begin_motion((mode == NAV_MODE_TEACH) ? APP_TEACH : APP_RUN);
}

static void go_idle(const char *why)
{
    stop_motors();
    Bench_Abort();
    app = APP_IDLE;
    dq_flush();  /* ký tự lái còn sót không được tự khởi động lại Lượt 1 */
    if (why) Uart_Write(why);
}

static void save_config(void)
{
    Uart_Write("Luu Flash...\r\n");
    Uart_Flush(300);  /* lúc xoá Flash CPU bị treo ~1-2 s, UART không gửi được */
    bool ok = Store_Save(&cfg, sizeof(cfg), STORE_VERSION);
    Uart_Write(ok ? "Luu Flash: OK\r\n" : "Luu Flash: LOI (firmware vuot 128 KB hoac loi ghi)\r\n");
}

static void finish_teach(void)
{
    cfg.rec_n = Nav_TeachFinish(cfg.rec, NAV_MAX_PTS);
    go_idle(NULL);
    Uart_Printf("Luot 1 xong: %u diem\r\n", (unsigned)cfg.rec_n);
    save_config();  /* để tắt nguồn giữa hai lượt không mất quỹ đạo */
}

/* ---------------------------------------------------------------- SW1 */
static bool btn_raw, btn_stable;
static bool btn_consumed;  /* lần nhấn hiện tại đã được dùng (dừng khẩn cấp, kết thúc Lượt 1, giữ từ lúc bật) */
static uint32_t btn_change_ms, btn_down_ms;

/* Trả về: 1 = vừa nhấn xuống, 2 = vừa thả (nhấn ngắn), 3 = vừa thả (nhấn giữ), 0 = không có gì. */
static int button_event(uint32_t now)
{
    bool down = Sw1_IsDown();
    if (down != btn_raw) {
        btn_raw = down;
        btn_change_ms = now;
    }
    if (btn_stable == btn_raw || (now - btn_change_ms) < DEBOUNCE_MS) return 0;
    btn_stable = btn_raw;
    if (btn_stable) {
        btn_down_ms = now;
        btn_consumed = false;
        return 1;
    }
    if (btn_consumed) return 0;
    return ((now - btn_down_ms) >= LONG_PRESS_MS) ? 3 : 2;
}

static void handle_button(uint32_t now)
{
    int ev = button_event(now);
    if (ev == 0) return;

    if ((app == APP_RUN || app == APP_BENCH) && ev == 1) {
        btn_consumed = true;  /* lúc thả ra không được khởi động Lượt 2 */
        go_idle("Dung khan cap (SW1)\r\n");
    } else if (app == APP_TEACH && ev == 1) {
        btn_consumed = true;
        finish_teach();
    } else if (app == APP_IDLE && ev == 2) {
        Uart_Write("Luot 2: quy dao A*\r\n");
        start_nav(NAV_MODE_RUN_ASTAR, NULL, 0);
    } else if (app == APP_IDLE && ev == 3) {
        if (cfg.rec_n >= 2) {
            Uart_Write("Luot 2: quy dao da ghi\r\n");
            start_nav(NAV_MODE_RUN_RECORDED, cfg.rec, cfg.rec_n);
        } else {
            Uart_Write("Chua co quy dao ghi, chay A*\r\n");
            start_nav(NAV_MODE_RUN_ASTAR, NULL, 0);
        }
    }
}

/* ---------------------------------------------------------------- Đọc cảm biến chung */
/* Gyro đã trừ bias; mất IMU thì lấy vận tốc góc từ encoder. */
static float read_gyro(float dl_cm, float dr_cm, float dt)
{
    float g;
    if (imu_ok && !MPU6050_ReadYawRate(&g)) {
        imu_ok = false;  /* mỗi lần đọc lỗi tốn 10 ms timeout: thôi dùng gyro cho tới khi khởi động lại */
        Uart_Write("Loi IMU: chuyen sang goc tu encoder\r\n");
    }
    if (!imu_ok) g = (dr_cm - dl_cm) / cfg.prm.track_cm / dt;
    return g;
}

static const char *state_name(NavState s)
{
    static const char *const names[] = {"IDLE", "RUN", "BRAKE", "AVOID", "TEACH", "DONE", "STOPPED", "EDGE"};
    return ((unsigned)s < sizeof(names) / sizeof(names[0])) ? names[s] : "?";
}

/* ---------------------------------------------------------------- Vòng điều khiển */
static void log_step(const NavOutput *out)
{
    if (out->state != last_nav_state) {
        Uart_Printf("E,%lu,%s\r\n", (unsigned long)HAL_GetTick(), state_name(out->state));
        last_nav_state = out->state;
    }
    if (!log_on || ++log_div < LOG_EVERY) return;
    log_div = 0;
    const Pose *p = Nav_GetPose();
    const NavDebug *d = Nav_GetDebug();
    /* Số nguyên (newlib-nano không in %f): x, y theo mm, theta theo mrad, v theo mm/s. */
    Uart_Printf("L,%lu,%d,%d,%d,%d,%d,%u\r\n", (unsigned long)HAL_GetTick(), (int)(p->x * 10.0f),
                (int)(p->y * 10.0f), (int)(p->th * 1000.0f), (int)(out->v_cm_s * 10.0f), (int)out->state,
                (unsigned)d->unknown_count);
}

static void control_step(float dt, char cmd)
{
    NavInput in;
    memset(&in, 0, sizeof(in));
    in.dt_s = dt;
    in.d_ticks_left = Encoder_LeftDelta();
    in.d_ticks_right = Encoder_RightDelta();
    float dl = (float)in.d_ticks_left / cfg.prm.ticks_per_cm;
    float dr = (float)in.d_ticks_right / cfg.prm.ticks_per_cm;
    float gyro = read_gyro(dl, dr, dt);
    Uart_Service();

    float duty_l = 0.0f, duty_r = 0.0f;
    if (app == APP_BENCH) {
        if (!Bench_Step(dt, (int16_t)in.d_ticks_left, (int16_t)in.d_ticks_right, gyro, imu_ok, &cfg.prm,
                        &cfg.gains, &wheel, &duty_l, &duty_r)) {
            go_idle(NULL);
            return;
        }
        Motor_SetDuty(duty_l, duty_r);
        return;
    }

    in.gyro_z_rad_s = gyro;
    in.tof_fresh = ToF_Poll(tof_mm);
    Uart_Service();
    for (int i = 0; i < NAV_TOF_COUNT && i < TOF_COUNT; i++) in.tof_mm[i] = tof_mm[i];
    in.ir_on_tape = Ir_OnTapeMask();
    in.manual_cmd = cmd;

    NavState before = Nav_GetState();
    NavOutput out = Nav_Step(&in);
    if (before == NAV_BRAKE) {
        /* Lần gọi này có thể đã chạy A* trên xe (vài chục tới >100 ms) lúc xe đứng yên: tính chu kỳ
         * sau từ bây giờ, nếu không MAX_CTRL_GAP_MS sẽ dừng xe đúng lúc bắt đầu né. */
        last_ctrl_ms = HAL_GetTick();
    }
    log_step(&out);
    if (out.state == NAV_DONE && app == APP_RUN) {
        go_idle("Ve dich\r\n");
        return;
    }
    if (out.state == NAV_STOPPED) {
        go_idle("Dung: khong co duong an toan\r\n");
        return;
    }
    Wheel_Step(&wheel, &cfg.gains, out.v_cm_s, out.w_rad_s, cfg.prm.track_cm, dl / dt, dr / dt, gyro, dt,
               &duty_l, &duty_r);
    Motor_SetDuty(duty_l, duty_r);
}

/* In cảm biến định kỳ khi đứng yên ($stream 1): kiểm tra chiều encoder, dấu gyro, ToF, IR. */
static void stream_step(uint32_t now)
{
    stream_l += Encoder_LeftDelta();
    stream_r += Encoder_RightDelta();
    if (!stream_on || (now - stream_ms) < STREAM_MS) return;
    stream_ms = now;
    float g = 0.0f;
    if (imu_ok && !MPU6050_ReadYawRate(&g)) imu_ok = false;
    (void)ToF_Poll(tof_mm);
    char f[16];
    Uart_Printf("S enc %ld %ld | gyro %s dps | tof %u %u %u %u | ir %u\r\n", (long)stream_l, (long)stream_r,
                Uart_FmtFloat(f, g * 57.29578f), tof_mm[0], tof_mm[1], tof_mm[2], tof_mm[3], Ir_OnTapeMask());
}

/* ---------------------------------------------------------------- Lệnh Bluetooth "$..." */
static int get_param(const char *name, float *v)
{
    if (!strcmp(name, "wheel_kff")) { *v = cfg.gains.kff; return 0; }
    if (!strcmp(name, "wheel_kp")) { *v = cfg.gains.kp; return 0; }
    if (!strcmp(name, "wheel_ki")) { *v = cfg.gains.ki; return 0; }
    if (!strcmp(name, "wheel_imax")) { *v = cfg.gains.i_max; return 0; }
    if (!strcmp(name, "wheel_kw")) { *v = cfg.gains.kw; return 0; }
    return NavParams_Get(&cfg.prm, name, v);
}

static int set_param(const char *name, float v)
{
    if (!strcmp(name, "wheel_kff")) { cfg.gains.kff = v; return 0; }
    if (!strcmp(name, "wheel_kp")) { cfg.gains.kp = v; return 0; }
    if (!strcmp(name, "wheel_ki")) { cfg.gains.ki = v; return 0; }
    if (!strcmp(name, "wheel_imax")) { cfg.gains.i_max = v; return 0; }
    if (!strcmp(name, "wheel_kw")) { cfg.gains.kw = v; return 0; }
    return NavParams_Set(&cfg.prm, name, v);
}

static void print_param(const char *name)
{
    float v;
    char f[16];
    if (get_param(name, &v) == 0) Uart_Printf("%s=%s\r\n", name, Uart_FmtFloat(f, v));
    else Uart_Printf("Khong co thong so '%s'\r\n", name);
    Uart_Flush(200);  /* danh sách dài hơn bộ đệm gửi: chờ từng dòng (chỉ khi đứng yên) */
}

static void cmd_help(void)
{
    static const char *const lines[] = {
        "$status | $get [ten] | $set ten gia_tri | $save | $load | $defaults\r\n",
        "$stream 0|1 (in cam bien) | $log 0|1 (log pose khi chay) | $stop\r\n",
        "$motor L% R% [ms] | $straight cm [v] | $spin do [w] | $calib dist cm_that\r\n",
        "Thong so banh: wheel_kff wheel_kp wheel_ki wheel_imax wheel_kw\r\n",
    };
    for (unsigned i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
        Uart_Write(lines[i]);
        Uart_Flush(200);
    }
}

static void cmd_status(void)
{
    static const char *const app_names[] = {"IDLE", "TEACH", "RUN", "BENCH"};
    Uart_Printf("App %s | Nav %s | IMU %s | ToF 0x%X | quy dao ghi %u diem | log %u\r\n", app_names[app],
                state_name(Nav_GetState()), imu_ok ? "OK" : "LOI", tof_mask, (unsigned)cfg.rec_n,
                (unsigned)log_on);
}

static void exec_line(char *s)
{
    char *argv[4] = {0};
    int argc = 0;
    for (char *tok = strtok(s, " \t"); tok && argc < 4; tok = strtok(NULL, " \t")) argv[argc++] = tok;
    if (argc == 0) return;
    const char *c = argv[0];

    /* Đang chạy: chỉ cho dừng và bật/tắt log. */
    if (!strcmp(c, "stop")) { go_idle("Dung (lenh)\r\n"); return; }
    if (!strcmp(c, "log") && argc >= 2) { log_on = atoi(argv[1]) != 0; return; }
    if (app != APP_IDLE) { Uart_Write("Dang chay: chi nhan $stop, $log\r\n"); return; }

    if (!strcmp(c, "help")) {
        cmd_help();
    } else if (!strcmp(c, "status")) {
        cmd_status();
    } else if (!strcmp(c, "get")) {
        if (argc >= 2) {
            print_param(argv[1]);
        } else {
            for (int i = 0; i < NavParams_Count(); i++) print_param(NavParams_NameAt(i));
            const char *wheel_names[] = {"wheel_kff", "wheel_kp", "wheel_ki", "wheel_imax", "wheel_kw"};
            for (unsigned i = 0; i < 5U; i++) print_param(wheel_names[i]);
        }
    } else if (!strcmp(c, "set") && argc >= 3) {
        if (set_param(argv[1], strtof(argv[2], NULL)) == 0) print_param(argv[1]);
        else Uart_Printf("Khong dat duoc '%s'\r\n", argv[1]);
    } else if (!strcmp(c, "save")) {
        save_config();
    } else if (!strcmp(c, "load")) {
        Uart_Write(Store_Load(&cfg, sizeof(cfg), STORE_VERSION) ? "Doc Flash: OK\r\n"
                                                                 : "Doc Flash: khong co du lieu hop le\r\n");
    } else if (!strcmp(c, "defaults")) {
        NavParams_Default(&cfg.prm);
        Wheel_DefaultGains(&cfg.gains);
        Uart_Write("Da ve mac dinh (chua luu, go $save)\r\n");
    } else if (!strcmp(c, "stream") && argc >= 2) {
        stream_on = atoi(argv[1]) != 0;
        stream_l = stream_r = 0;
    } else if (!strcmp(c, "motor") && argc >= 3) {
        uint32_t ms = (argc >= 4) ? (uint32_t)atoi(argv[3]) : 1000U;
        Bench_StartMotor(strtof(argv[1], NULL) * 0.01f, strtof(argv[2], NULL) * 0.01f, ms);
        begin_motion(APP_BENCH);
    } else if (!strcmp(c, "straight") && argc >= 2) {
        Bench_StartStraight(strtof(argv[1], NULL), (argc >= 3) ? strtof(argv[2], NULL) : 20.0f);
        begin_motion(APP_BENCH);
    } else if (!strcmp(c, "spin") && argc >= 2) {
        Bench_StartSpin(strtof(argv[1], NULL), (argc >= 3) ? strtof(argv[2], NULL) : 1.5f);
        begin_motion(APP_BENCH);
    } else if (!strcmp(c, "calib") && argc >= 3 && !strcmp(argv[1], "dist")) {
        float enc = fabsf(Bench_TakeLastStraightCm()), real = strtof(argv[2], NULL);
        if (enc > 1.0f && real > 1.0f) {
            /* encoder báo enc cm khi xe đi real cm: số xung/cm thật = cũ * enc / real */
            set_param("ticks_per_cm", cfg.prm.ticks_per_cm * enc / real);
            print_param("ticks_per_cm");
            Uart_Write("Go $save de luu\r\n");
        } else {
            Uart_Write("Chay $straight truoc\r\n");
        }
    } else {
        Uart_Write("Lenh khong hop le, go $help\r\n");
    }
}

/* Ký tự từ HC-05: dòng bắt đầu bằng '$' là lệnh (terminal), ký tự đơn khác là lệnh lái của app RC
 * (app RC không bao giờ gửi '$'). */
static void route_rx(void)
{
    int ch;
    while ((ch = Uart_Getc()) >= 0) {
        char c = (char)ch;
        if (line_len == 0 && c != '$') {
            if (c != '\r' && c != '\n') dq_push(c);
            continue;
        }
        if (c == '\r' || c == '\n') {
            line[line_len] = 0;
            exec_line(line + 1);  /* bỏ '$' */
            line_len = 0;
        } else if (line_len < LINE_LEN - 1U) {
            line[line_len++] = c;
        }
    }
}

/* ---------------------------------------------------------------- API */
void RobotApp_Init(void)
{
    Motor_Init();  /* STBY thấp: động cơ tắt trong lúc khởi động */
    bool from_flash = Store_Load(&cfg, sizeof(cfg), STORE_VERSION);
    if (!from_flash) {
        memset(&cfg, 0, sizeof(cfg));
        NavParams_Default(&cfg.prm);
        Wheel_DefaultGains(&cfg.gains);
    }
    Wheel_Reset(&wheel);
    for (int i = 0; i < TOF_COUNT; i++) tof_mm[i] = TOF_NO_READING;

    Uart_Write("\r\n== Khoi dong, giu xe dung yen ==\r\n");
    Uart_Flush(200);
    imu_ok = MPU6050_Init() && MPU6050_CalibrateGyro(GYRO_CAL_SAMPLES);
    tof_mask = ToF_InitAll();
    Uart_Printf("IMU %s, ToF 0x%X/0x%X, cau hinh %s, quy dao ghi %u diem. Go $help\r\n", imu_ok ? "OK" : "LOI",
                tof_mask, (1U << TOF_COUNT) - 1U, from_flash ? "tu Flash" : "mac dinh", (unsigned)cfg.rec_n);

    btn_raw = btn_stable = Sw1_IsDown();
    btn_consumed = btn_stable;  /* đang giữ SW1 lúc bật nguồn: lần thả này không tính */
    btn_down_ms = HAL_GetTick();
    last_ctrl_ms = HAL_GetTick();
    app = APP_IDLE;
}

void RobotApp_Task(void)
{
    uint32_t now = HAL_GetTick();
    Uart_Service();
    route_rx();
    handle_button(now);

    if (app == APP_IDLE) {
        stream_step(now);
        char c = dq_pop();
        if (is_speed_cmd(c)) {
            idle_speed = c;
        } else if (is_drive_cmd(c)) {
            start_nav(NAV_MODE_TEACH, NULL, 0);
            if (idle_speed) control_step(CONTROL_PERIOD_MS * 0.001f, idle_speed);
            control_step(CONTROL_PERIOD_MS * 0.001f, c);
        }
        return;
    }

    uint32_t elapsed = now - last_ctrl_ms;
    if (elapsed < CONTROL_PERIOD_MS) return;
    last_ctrl_ms = now;
    if (elapsed > MAX_CTRL_GAP_MS) {
        go_idle("Loi: vong dieu khien qua cham (I2C treo?)\r\n");
        return;
    }
    control_step((float)elapsed * 0.001f, (app == APP_TEACH) ? dq_pop() : 0);
}

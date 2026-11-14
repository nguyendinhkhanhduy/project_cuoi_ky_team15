/* Tự sinh bởi tools/path_planner/plan_path.py - KHÔNG sửa tay, chạy lại script. */
/* Toạ độ cm, gốc ở góc dưới-trái sân, x sang phải, y lên trên. */
#ifndef PATH_H
#define PATH_H

typedef struct {
    float x_cm;
    float y_cm;
} PathPoint;

typedef struct {
    float x_min;
    float y_min;
    float x_max;
    float y_max;
} MapRect;

#define MAP_FIELD_W_CM          180.0f
#define MAP_FIELD_H_CM          120.0f
#define PATH_START_HEADING_DEG  90.0f
#define PATH_CLEARANCE_MIN_CM   8.7f
#define PATH_LENGTH_CM          292.1f
#define PATH_POINT_COUNT        69U
#define MAP_OBSTACLE_COUNT      18U
#define MAP_BLOCK_COUNT         17U  /* MAP_OBSTACLES[0..n) là khối thật, sau đó là vùng cấm */

/* Chỉ include file này từ MỘT file .c (mảng static const). */
static const PathPoint PATH_POINTS[PATH_POINT_COUNT] = {
    {  30.00f,    6.00f},
    {  27.30f,   10.10f},
    {  24.60f,   14.20f},
    {  21.90f,   18.30f},
    {  19.20f,   22.40f},
    {  16.50f,   26.50f},
    {  14.50f,   29.50f},
    {  12.50f,   32.50f},
    {  13.50f,   35.83f},
    {  14.50f,   39.17f},
    {  15.50f,   42.50f},
    {  17.50f,   44.50f},
    {  17.50f,   45.50f},
    {  20.70f,   49.10f},
    {  23.90f,   52.70f},
    {  27.10f,   56.30f},
    {  30.30f,   59.90f},
    {  33.50f,   63.50f},
    {  36.70f,   67.10f},
    {  39.90f,   70.70f},
    {  43.10f,   74.30f},
    {  46.30f,   77.90f},
    {  49.50f,   81.50f},
    {  51.50f,   84.75f},
    {  53.50f,   88.00f},
    {  55.50f,   91.25f},
    {  57.50f,   94.50f},
    {  61.25f,   93.25f},
    {  65.00f,   92.00f},
    {  68.75f,   90.75f},
    {  72.50f,   89.50f},
    {  74.50f,   87.50f},
    {  78.50f,   85.50f},
    {  82.79f,   84.50f},
    {  87.07f,   83.50f},
    {  91.36f,   82.50f},
    {  95.64f,   81.50f},
    {  99.93f,   80.50f},
    { 104.21f,   79.50f},
    { 108.50f,   78.50f},
    { 111.50f,   74.50f},
    { 114.50f,   70.50f},
    { 119.06f,   69.83f},
    { 123.61f,   69.17f},
    { 128.17f,   68.50f},
    { 132.72f,   67.83f},
    { 137.28f,   67.17f},
    { 141.83f,   66.50f},
    { 146.39f,   65.83f},
    { 150.94f,   65.17f},
    { 155.50f,   64.50f},
    { 157.10f,   59.90f},
    { 158.70f,   55.30f},
    { 160.30f,   50.70f},
    { 161.90f,   46.10f},
    { 163.50f,   41.50f},
    { 162.25f,   37.75f},
    { 161.00f,   34.00f},
    { 159.75f,   30.25f},
    { 158.50f,   26.50f},
    { 159.50f,   22.30f},
    { 160.50f,   18.10f},
    { 161.50f,   13.90f},
    { 162.50f,    9.70f},
    { 163.50f,    5.50f},
    { 163.50f,    1.62f},
    { 163.50f,   -2.25f},
    { 163.50f,   -6.12f},
    { 163.50f,  -10.00f},
};

static const MapRect MAP_OBSTACLES[MAP_OBSTACLE_COUNT] = {
    {   0.0f,   10.0f,   10.0f,   20.0f}, /* T1 */
    {  50.0f,   20.0f,   60.0f,   30.0f}, /* T2 */
    {  25.0f,   30.0f,   35.0f,   40.0f}, /* T3 */
    {  50.0f,   40.0f,   60.0f,   50.0f}, /* T4 */
    {   0.0f,   50.0f,   10.0f,   60.0f}, /* T5 */
    {   0.0f,   70.0f,   10.0f,   80.0f}, /* T6 */
    {  30.0f,   85.0f,   40.0f,   95.0f}, /* T7 */
    {  40.0f,  110.0f,   50.0f,  120.0f}, /* T8 */
    {  60.0f,   70.0f,   70.0f,   80.0f}, /* T9 */
    {  70.0f,   60.0f,   80.0f,   70.0f}, /* T10 */
    {  80.0f,   95.0f,   90.0f,  105.0f}, /* T11 */
    {  90.0f,   60.0f,  100.0f,   70.0f}, /* T12 */
    { 110.0f,  110.0f,  120.0f,  120.0f}, /* T13 */
    { 120.0f,   80.0f,  130.0f,   90.0f}, /* T14 */
    { 140.0f,   40.0f,  150.0f,   50.0f}, /* T15 */
    { 140.0f,   10.0f,  150.0f,   20.0f}, /* T16 */
    { 170.0f,   20.0f,  180.0f,   30.0f}, /* T17 */
    {  60.0f,    0.0f,  120.0f,   60.0f}, /* FORBIDDEN */
};

#endif /* PATH_H */

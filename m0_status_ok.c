#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * 配套说明：
 * - K230端 main_status.py 连续3帧确认后只发送一个ASCII字节：'1'、'2'、'3'或'4'。
 * - 本文件的 DecodeCommand() 与该协议直接对应。
 * - K230端每次上电只发送一次，因此整车每次上电只执行一次任务。
 * - M0到K230的 A/F/B/E 回传为可选调试功能；只做单向控制时可不接M0 PA0到K230 IO33。
 */

/*
 * MSPM0G3507 + K230 + JY60
 *
 * K230发送单字节命令：
 *   '1' 或 0x01：直行1米，然后左转90度
 *   '2' 或 0x02：直行1米，然后右转90度
 *   '3' 或 0x03：直行2米，然后左转90度
 *   '4' 或 0x04：直行2米，然后右转90度
 *
 * 一次任务完成后停车，重新等待下一条命令。
 * 忙碌期间收到新命令会忽略，并向K230返回字符'B'。
 *
 * MSPM0引脚（来自当前SysConfig）：
 *   电机：PB8 IN1，PB9 IN2，PA31 ENA，PB2 IN3，PB3 IN4，PB16 ENB
 *   编码器：PA26 左A，PA27 左B，PA24 右A，PA25 右B
 *   K230 UART0：PA1 RX，PA0 TX，115200 8N1
 *   JY60 UART1：PA18 RX，PA8 TX，建议9600 8N1
 *
 * 接线：
 *   K230 TX -> MSPM0 PA1
 *   K230 RX <- MSPM0 PA0（若只发送命令，可以不接）
 *   K230 GND -- MSPM0 GND
 *
 *   JY60 TX -> MSPM0 PA18
 *   JY60 RX <- MSPM0 PA8（通常可不接）
 *   JY60 GND -- MSPM0 GND
 */

/*==================================================
 * 系统参数
 *==================================================*/

#ifndef CPUCLK_FREQ
#define CPUCLK_FREQ                         32000000UL
#endif

#define SYSTICK_FREQUENCY_HZ                20000U
#define PWM_PERIOD_COUNT                    100U
#define ENCODER_CONTROL_INTERVAL_TICKS      400U   /* 20ms */
#define HEADING_CONTROL_INTERVAL_TICKS      200U   /* 10ms */
#define SYSTICK_TICKS_PER_MS                20U

/*==================================================
 * UART实例
 *==================================================*/

#ifndef K230_UART_INST
#error "K230_UART_INST未定义：请确认SysConfig中的UART名称为K230_UART"
#endif

#ifndef UART_0_INST
#error "UART_1_INST未定义：请确认JY60串口名称为UART_1"
#endif

#define JY60_UART_INST                      UART_0_INST

/*==================================================
 * 编码器与距离参数
 *==================================================*/

#define LEFT_COUNTS_PER_REV                 740L
#define RIGHT_COUNTS_PER_REV                740L

/* 轮胎直径，单位mm。实测距离不准时优先修改这个值。 */
#define WHEEL_DIAMETER_MM                   65L

/* 距离总校准系数：1000=1.000倍，走短则调大，走长则调小。 */
#define DISTANCE_CALIBRATION_X1000          1000L

#define DISTANCE_1M_MM                      1000U
#define DISTANCE_2M_MM                      2000U
#define DISTANCE_SLOW_ZONE_MM               150U
#define DISTANCE_FINAL_ZONE_MM              50U
#define DISTANCE_SLOW_PWM                   32
#define DISTANCE_FINAL_PWM                  27

/*==================================================
 * 直线电机参数
 *==================================================*/

#define MOTOR_BASE_PWM                      40
#define MOTOR_MIN_PWM                       24
#define MOTOR_MAX_PWM                       56
#define STARTUP_PWM_STEP                    2
#define PWM_CHANGE_LIMIT                    1
#define PID_START_STABLE_CYCLES             15U
#define TOTAL_CORRECTION_LIMIT_X100         1000L

/*==================================================
 * 电机映射与方向
 *==================================================*/

#define MOTOR_CHANNELS_SWAPPED              0
#define DRIVER_A_FORWARD_REVERSE            0
#define DRIVER_B_FORWARD_REVERSE            0

/*==================================================
 * 编码器差速PID
 *==================================================*/

#define PID_INITIAL_KP_X1000                550L
#define PID_INITIAL_KI_X1000                8L
#define PID_INITIAL_KD_X1000                80L
#define PID_MAX_CORRECTION_X100             800L
#define PID_CORRECTION_STEP_X100            100L
#define PID_ERROR_DEADBAND_X100             10L
#define PID_INTEGRAL_LIMIT_X100             15000L
#define INITIAL_TARGET_DIFFERENCE_X100      35L

/*==================================================
 * 直线航向PD
 *==================================================*/

#define HEADING_NEAR_ZONE_X100              100L
#define HEADING_KP_NEAR_X100                100L
#define HEADING_KP_FAR_X100                 180L
#define HEADING_KD_X100                     15L
#define HEADING_MAX_CORRECTION_X100         700L
#define HEADING_CORRECTION_STEP_X100        150L
#define HEADING_MIN_ACTIVE_X100             100L
#define HEADING_DEADBAND_ENTER_X100         15L
#define HEADING_DEADBAND_EXIT_X100          30L
#define GYRO_DEADBAND_X100                  20L

/*==================================================
 * JY60在线判定
 *==================================================*/

#define JY60_OFFLINE_TIMEOUT_MS             300U
#define JY60_REQUIRED_ANGLE_FRAMES          8U
#define COMMAND_WAIT_JY60_TIMEOUT_MS        5000U

/*==================================================
 * 90度转向参数
 *==================================================*/

/* 左转时JY60的Yaw是否增加。根据你此前“左转Z轴为正”先设为1。 */
#define JY60_LEFT_TURN_YAW_INCREASES        1

#define TURN_ANGLE_X100                     9000L
#define TURN_FAST_PWM                       32
#define TURN_MEDIUM_PWM                     28
#define TURN_SLOW_PWM                       24
#define TURN_CREEP_PWM                      21

#define TURN_FAST_ZONE_X100                 2500L  /* 25度外 */
#define TURN_MEDIUM_ZONE_X100               800L   /* 8度外 */
#define TURN_SLOW_ZONE_X100                 250L   /* 2.5度外 */
#define TURN_TOLERANCE_X100                 100L   /* ±1度 */
#define TURN_RATE_LIMIT_X100                250L   /* 2.5度/秒 */
#define TURN_STABLE_CYCLES                  10U    /* 100ms */
#define TURN_TIMEOUT_MS                     8000U
#define STRAIGHT_STOP_SETTLE_MS             250U
#define TURN_STOP_SETTLE_MS                 300U

/*==================================================
 * 引脚
 *==================================================*/

#define DRIVER_A_IN1_PORT                   GPIOB
#define DRIVER_A_IN1_PIN                    DL_GPIO_PIN_8
#define DRIVER_A_IN2_PORT                   GPIOB
#define DRIVER_A_IN2_PIN                    DL_GPIO_PIN_9
#define DRIVER_A_EN_PORT                    GPIOA
#define DRIVER_A_EN_PIN                     DL_GPIO_PIN_31

#define DRIVER_B_IN3_PORT                   GPIOB
#define DRIVER_B_IN3_PIN                    DL_GPIO_PIN_2
#define DRIVER_B_IN4_PORT                   GPIOB
#define DRIVER_B_IN4_PIN                    DL_GPIO_PIN_3
#define DRIVER_B_EN_PORT                    GPIOB
#define DRIVER_B_EN_PIN                     DL_GPIO_PIN_16

#define LEFT_ENCODER_A_PIN                  DL_GPIO_PIN_26
#define LEFT_ENCODER_B_PIN                  DL_GPIO_PIN_27
#define RIGHT_ENCODER_A_PIN                 DL_GPIO_PIN_24
#define RIGHT_ENCODER_B_PIN                 DL_GPIO_PIN_25

/*==================================================
 * 状态定义
 *==================================================*/

typedef enum {
    MISSION_IDLE = 0,
    MISSION_WAIT_JY60,
    MISSION_STRAIGHT,
    MISSION_STRAIGHT_SETTLE,
    MISSION_TURN,
    MISSION_TURN_SETTLE,
    MISSION_ERROR
} MissionState;

typedef enum {
    TURN_NONE = 0,
    TURN_LEFT,
    TURN_RIGHT
} TurnDirection;

/*
 * 直线控制状态：
 * 0 停止
 * 2 缓启动
 * 3 等待轮速稳定
 * 4 编码器PID + JY60
 * 5 仅编码器PID
 * 900 SysTick错误
 */
volatile uint32_t g_run_state = 0U;
volatile MissionState g_mission_state = MISSION_IDLE;
volatile TurnDirection g_mission_turn = TURN_NONE;
volatile uint8_t g_last_command = 0U;
volatile uint32_t g_mission_error_code = 0U;

/*==================================================
 * 系统变量
 *==================================================*/

volatile uint32_t g_system_ms = 0U;
volatile uint32_t g_systick_heartbeat = 0U;

/*==================================================
 * 编码器变量
 *==================================================*/

volatile uint32_t g_left_speed_count = 0U;
volatile uint32_t g_right_speed_count = 0U;
volatile uint32_t g_left_encoder_total = 0U;
volatile uint32_t g_right_encoder_total = 0U;
volatile uint32_t g_left_invalid_count = 0U;
volatile uint32_t g_right_invalid_count = 0U;
volatile int32_t g_left_speed_filtered_x100 = 0;
volatile int32_t g_right_speed_filtered_x100 = 0;
volatile int32_t g_left_speed_normalized_x100 = 0;
volatile int32_t g_right_speed_normalized_x100 = 0;
volatile int32_t g_measured_left_minus_right_x100 = 0;
volatile int32_t g_left_rpm_x100 = 0;
volatile int32_t g_right_rpm_x100 = 0;

/*==================================================
 * 距离任务变量
 *==================================================*/

volatile uint32_t g_mission_distance_mm = 0U;
volatile uint32_t g_distance_target_count = 0U;
volatile uint32_t g_distance_travel_count = 0U;
volatile uint32_t g_distance_left_delta = 0U;
volatile uint32_t g_distance_right_delta = 0U;
static uint32_t g_distance_start_left = 0U;
static uint32_t g_distance_start_right = 0U;

/*==================================================
 * PID变量
 *==================================================*/

volatile int32_t g_pid_kp_x1000 = PID_INITIAL_KP_X1000;
volatile int32_t g_pid_ki_x1000 = PID_INITIAL_KI_X1000;
volatile int32_t g_pid_kd_x1000 = PID_INITIAL_KD_X1000;
volatile int32_t g_target_left_minus_right_x100 = INITIAL_TARGET_DIFFERENCE_X100;
volatile int32_t g_pid_max_correction_x100 = PID_MAX_CORRECTION_X100;
volatile uint32_t g_pid_reset_command = 0U;
volatile uint32_t g_capture_heading_command = 0U;

volatile int32_t g_pid_error_x100 = 0;
volatile int32_t g_pid_previous_error_x100 = 0;
volatile int32_t g_pid_derivative_x100 = 0;
volatile int32_t g_pid_integral_x100 = 0;
volatile int32_t g_pid_p_x100 = 0;
volatile int32_t g_pid_i_x100 = 0;
volatile int32_t g_pid_d_x100 = 0;
volatile int32_t g_pid_raw_correction_x100 = 0;
volatile int32_t g_pid_correction_x100 = 0;

/*==================================================
 * JY60变量
 *==================================================*/

volatile uint32_t g_jy60_yaw_reverse = 0U;
volatile uint32_t g_jy60_gyro_reverse = 0U;
volatile uint32_t g_heading_output_reverse = 0U;
volatile int32_t g_manual_heading_correction_x100 = 0;

volatile int32_t g_jy60_yaw_deg_x100 = 0;
volatile int32_t g_jy60_yaw_filtered_x100 = 0;
volatile int32_t g_jy60_gyro_z_dps_x100 = 0;
volatile int32_t g_jy60_gyro_filtered_x100 = 0;
volatile int32_t g_target_yaw_deg_x100 = 0;
volatile int32_t g_heading_error_x100 = 0;
volatile int32_t g_heading_p_x100 = 0;
volatile int32_t g_heading_d_x100 = 0;
volatile int32_t g_heading_raw_correction_x100 = 0;
volatile int32_t g_heading_correction_x100 = 0;
volatile uint32_t g_heading_deadband_active = 1U;
volatile uint32_t g_jy60_online = 0U;
volatile uint32_t g_jy60_gyro_online = 0U;
volatile uint32_t g_heading_target_valid = 0U;
volatile uint32_t g_jy60_angle_frame_count = 0U;
volatile uint32_t g_jy60_gyro_frame_count = 0U;
volatile uint32_t g_jy60_checksum_errors = 0U;
volatile uint32_t g_jy60_last_angle_ms = 0U;
volatile uint32_t g_jy60_last_gyro_ms = 0U;
volatile uint32_t g_heading_update_count = 0U;

/*==================================================
 * 转向变量
 *==================================================*/

volatile int32_t g_turn_target_yaw_x100 = 0;
volatile int32_t g_turn_error_x100 = 0;
volatile int32_t g_turn_gyro_x100 = 0;
volatile int32_t g_turn_pwm = 0;
volatile uint32_t g_turn_stable_count = 0U;
volatile uint32_t g_turn_start_ms = 0U;
volatile uint32_t g_turn_direction_output = 0U;

/*==================================================
 * PWM变量
 *==================================================*/

volatile int32_t g_current_base_pwm = 0;
volatile int32_t g_total_correction_x100 = 0;
volatile int32_t g_left_target_pwm = 0;
volatile int32_t g_right_target_pwm = 0;
volatile int32_t g_left_pwm = 0;
volatile int32_t g_right_pwm = 0;
volatile int32_t g_driver_a_pwm = 0;
volatile int32_t g_driver_b_pwm = 0;
volatile uint32_t g_motor_output_enabled = 0U;

/*==================================================
 * 中断内部变量
 *==================================================*/

static volatile uint32_t g_pwm_phase = 0U;
static volatile uint32_t g_encoder_control_tick = 0U;
static volatile uint32_t g_heading_control_tick = 0U;
static volatile uint32_t g_encoder_control_due = 0U;
static volatile uint32_t g_heading_control_due = 0U;
static volatile uint32_t g_ms_divider = 0U;
static volatile uint32_t g_left_period_count = 0U;
static volatile uint32_t g_right_period_count = 0U;
static volatile uint32_t g_left_previous_state = 0U;
static volatile uint32_t g_right_previous_state = 0U;
static uint32_t g_pid_stable_cycle_count = 0U;

/*==================================================
 * 通信与任务内部变量
 *==================================================*/

static uint32_t g_command_wait_start_ms = 0U;
static uint32_t g_state_start_ms = 0U;
static uint8_t g_k230_last_rx = 0U;
static uint32_t g_k230_rx_count = 0U;
static uint32_t g_k230_valid_command_count = 0U;

/*==================================================
 * JY60内部变量
 *==================================================*/

static uint8_t g_jy60_frame[11];
static uint32_t g_jy60_frame_index = 0U;
static uint32_t g_last_used_angle_frame = 0U;
static uint32_t g_last_used_gyro_frame = 0U;

static const int8_t g_quadrature_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

/*==================================================
 * 工具函数
 *==================================================*/

static int32_t ClampInt32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int32_t AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t MoveToward(int32_t current, int32_t target, int32_t step)
{
    if (current < target) {
        current += step;
        if (current > target) current = target;
    } else if (current > target) {
        current -= step;
        if (current < target) current = target;
    }
    return current;
}

static int32_t NormalizeAngleX100(int32_t angle)
{
    while (angle > 18000L) angle -= 36000L;
    while (angle < -18000L) angle += 36000L;
    return angle;
}

static int32_t WrapAngleErrorX100(int32_t error)
{
    return NormalizeAngleX100(error);
}

static int32_t CorrectionX100ToPWM(int32_t value)
{
    if (value >= 0) return (value + 50L) / 100L;
    return (value - 50L) / 100L;
}

static uint32_t DistanceMmToNormalizedCount(uint32_t distanceMm)
{
    uint64_t numerator;
    uint64_t denominator;
    uint64_t count;

    /* 周长 = 直径 * pi，pi取3.14159。 */
    numerator = (uint64_t)distanceMm *
                (uint64_t)RIGHT_COUNTS_PER_REV *
                100000ULL *
                (uint64_t)DISTANCE_CALIBRATION_X1000;

    denominator = (uint64_t)WHEEL_DIAMETER_MM *
                  314159ULL *
                  1000ULL;

    count = (numerator + denominator / 2ULL) / denominator;

    if (count > 0xFFFFFFFFULL) count = 0xFFFFFFFFULL;
    return (uint32_t)count;
}

/*==================================================
 * K230串口
 *==================================================*/

static void K230_SendByte(uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(K230_UART_INST, data);
}

static void K230_SendAccepted(uint8_t command)
{
    K230_SendByte((uint8_t)'A');
    K230_SendByte((uint8_t)('0' + command));
}

static void K230_SendFinished(uint8_t command)
{
    K230_SendByte((uint8_t)'F');
    K230_SendByte((uint8_t)('0' + command));
}

static void K230_SendError(uint8_t errorCode)
{
    K230_SendByte((uint8_t)'E');
    K230_SendByte((uint8_t)('0' + errorCode));
}

/*==================================================
 * 电机GPIO与方向
 *==================================================*/

static void DriverA_Enable(void)
{
    DL_GPIO_setPins(DRIVER_A_EN_PORT, DRIVER_A_EN_PIN);
}

static void DriverA_Disable(void)
{
    DL_GPIO_clearPins(DRIVER_A_EN_PORT, DRIVER_A_EN_PIN);
}

static void DriverB_Enable(void)
{
    DL_GPIO_setPins(DRIVER_B_EN_PORT, DRIVER_B_EN_PIN);
}

static void DriverB_Disable(void)
{
    DL_GPIO_clearPins(DRIVER_B_EN_PORT, DRIVER_B_EN_PIN);
}

static void Motor_GPIO_Init(void)
{
    DL_GPIO_clearPins(GPIOA, DRIVER_A_EN_PIN);
    DL_GPIO_clearPins(
        GPIOB,
        DRIVER_A_IN1_PIN | DRIVER_A_IN2_PIN |
        DRIVER_B_IN3_PIN | DRIVER_B_IN4_PIN |
        DRIVER_B_EN_PIN
    );

    DL_GPIO_enableOutput(GPIOA, DRIVER_A_EN_PIN);
    DL_GPIO_enableOutput(
        GPIOB,
        DRIVER_A_IN1_PIN | DRIVER_A_IN2_PIN |
        DRIVER_B_IN3_PIN | DRIVER_B_IN4_PIN |
        DRIVER_B_EN_PIN
    );
}

static void DriverA_SetDirection(uint32_t forward)
{
    uint32_t useIn1;

    useIn1 = forward ^ (uint32_t)DRIVER_A_FORWARD_REVERSE;

    if (useIn1 != 0U) {
        DL_GPIO_clearPins(DRIVER_A_IN2_PORT, DRIVER_A_IN2_PIN);
        DL_GPIO_setPins(DRIVER_A_IN1_PORT, DRIVER_A_IN1_PIN);
    } else {
        DL_GPIO_clearPins(DRIVER_A_IN1_PORT, DRIVER_A_IN1_PIN);
        DL_GPIO_setPins(DRIVER_A_IN2_PORT, DRIVER_A_IN2_PIN);
    }
}

static void DriverB_SetDirection(uint32_t forward)
{
    uint32_t useIn3;

    useIn3 = forward ^ (uint32_t)DRIVER_B_FORWARD_REVERSE;

    if (useIn3 != 0U) {
        DL_GPIO_clearPins(DRIVER_B_IN4_PORT, DRIVER_B_IN4_PIN);
        DL_GPIO_setPins(DRIVER_B_IN3_PORT, DRIVER_B_IN3_PIN);
    } else {
        DL_GPIO_clearPins(DRIVER_B_IN3_PORT, DRIVER_B_IN3_PIN);
        DL_GPIO_setPins(DRIVER_B_IN4_PORT, DRIVER_B_IN4_PIN);
    }
}

static void Motor_SetPhysicalDirections(uint32_t leftForward, uint32_t rightForward)
{
    DriverA_Disable();
    DriverB_Disable();

#if MOTOR_CHANNELS_SWAPPED == 0
    DriverA_SetDirection(leftForward);
    DriverB_SetDirection(rightForward);
#else
    DriverA_SetDirection(rightForward);
    DriverB_SetDirection(leftForward);
#endif
}

static void Motor_SetForwardDirection(void)
{
    Motor_SetPhysicalDirections(1U, 1U);
}

static void Motor_SetLeftTurnDirection(void)
{
    /* 左轮后退，右轮前进。 */
    Motor_SetPhysicalDirections(0U, 1U);
}

static void Motor_SetRightTurnDirection(void)
{
    /* 左轮前进，右轮后退。 */
    Motor_SetPhysicalDirections(1U, 0U);
}

static void Motor_UpdateDriverPWM(void)
{
#if MOTOR_CHANNELS_SWAPPED == 0
    g_driver_a_pwm = g_left_pwm;
    g_driver_b_pwm = g_right_pwm;
#else
    g_driver_a_pwm = g_right_pwm;
    g_driver_b_pwm = g_left_pwm;
#endif

    g_driver_a_pwm = ClampInt32(g_driver_a_pwm, 0, 100);
    g_driver_b_pwm = ClampInt32(g_driver_b_pwm, 0, 100);
}

static void Motor_Stop(void)
{
    g_motor_output_enabled = 0U;
    g_current_base_pwm = 0;
    g_left_target_pwm = 0;
    g_right_target_pwm = 0;
    g_left_pwm = 0;
    g_right_pwm = 0;
    g_driver_a_pwm = 0;
    g_driver_b_pwm = 0;
    g_run_state = 0U;
    DriverA_Disable();
    DriverB_Disable();
}

static void Motor_ApplyTurnPWM(int32_t pwm)
{
    pwm = ClampInt32(pwm, 0, 100);
    g_left_pwm = pwm;
    g_right_pwm = pwm;
    Motor_UpdateDriverPWM();
    g_motor_output_enabled = 1U;
}

/*==================================================
 * 编码器
 *==================================================*/

static uint32_t Encoder_ReadLeftState(uint32_t gpioValue)
{
    uint32_t a = ((gpioValue & LEFT_ENCODER_A_PIN) != 0U) ? 1U : 0U;
    uint32_t b = ((gpioValue & LEFT_ENCODER_B_PIN) != 0U) ? 1U : 0U;
    return (a << 1U) | b;
}

static uint32_t Encoder_ReadRightState(uint32_t gpioValue)
{
    uint32_t a = ((gpioValue & RIGHT_ENCODER_A_PIN) != 0U) ? 1U : 0U;
    uint32_t b = ((gpioValue & RIGHT_ENCODER_B_PIN) != 0U) ? 1U : 0U;
    return (a << 1U) | b;
}

static void Encoder_Initialize(void)
{
    uint32_t gpioValue;

    gpioValue = DL_GPIO_readPins(
        GPIOA,
        LEFT_ENCODER_A_PIN | LEFT_ENCODER_B_PIN |
        RIGHT_ENCODER_A_PIN | RIGHT_ENCODER_B_PIN
    );

    g_left_previous_state = Encoder_ReadLeftState(gpioValue);
    g_right_previous_state = Encoder_ReadRightState(gpioValue);
    g_left_period_count = 0U;
    g_right_period_count = 0U;
}

static void Encoder_UpdateSpeed20ms(void)
{
    uint32_t leftCount;
    uint32_t rightCount;

    __disable_irq();
    leftCount = g_left_period_count;
    rightCount = g_right_period_count;
    g_left_period_count = 0U;
    g_right_period_count = 0U;
    g_encoder_control_due = 0U;
    __enable_irq();

    g_left_speed_count = leftCount;
    g_right_speed_count = rightCount;

    g_left_speed_filtered_x100 =
        (g_left_speed_filtered_x100 + ((int32_t)leftCount * 100L)) / 2L;
    g_right_speed_filtered_x100 =
        (g_right_speed_filtered_x100 + ((int32_t)rightCount * 100L)) / 2L;

    g_left_speed_normalized_x100 =
        (int32_t)(((int64_t)g_left_speed_filtered_x100 * RIGHT_COUNTS_PER_REV) /
                  LEFT_COUNTS_PER_REV);
    g_right_speed_normalized_x100 = g_right_speed_filtered_x100;
    g_measured_left_minus_right_x100 =
        g_left_speed_normalized_x100 - g_right_speed_normalized_x100;

    g_left_rpm_x100 =
        (int32_t)(((int64_t)g_left_speed_filtered_x100 * 3000LL) /
                  LEFT_COUNTS_PER_REV);
    g_right_rpm_x100 =
        (int32_t)(((int64_t)g_right_speed_filtered_x100 * 3000LL) /
                  RIGHT_COUNTS_PER_REV);
}

static uint32_t Mission_UpdateTravelCount(void)
{
    uint32_t leftDelta;
    uint32_t rightDelta;
    uint32_t leftNormalized;
    uint32_t average;

    leftDelta = g_left_encoder_total - g_distance_start_left;
    rightDelta = g_right_encoder_total - g_distance_start_right;

    leftNormalized = (uint32_t)(
        ((uint64_t)leftDelta * (uint64_t)RIGHT_COUNTS_PER_REV) /
        (uint64_t)LEFT_COUNTS_PER_REV
    );

    average = (leftNormalized + rightDelta) / 2U;

    g_distance_left_delta = leftDelta;
    g_distance_right_delta = rightDelta;
    g_distance_travel_count = average;

    return average;
}

/*==================================================
 * 编码器PID
 *==================================================*/

static void Straight_PID_Reset(void)
{
    g_pid_error_x100 = 0;
    g_pid_previous_error_x100 = 0;
    g_pid_derivative_x100 = 0;
    g_pid_integral_x100 = 0;
    g_pid_p_x100 = 0;
    g_pid_i_x100 = 0;
    g_pid_d_x100 = 0;
    g_pid_raw_correction_x100 = 0;
    g_pid_correction_x100 = 0;
}

static int32_t Straight_PID_Update(void)
{
    int32_t error;
    int32_t derivative;
    int32_t oldIntegral;
    int32_t candidateIntegral;
    int32_t pOutput;
    int32_t iOutput;
    int32_t dOutput;
    int32_t rawOutput;
    int32_t limitedOutput;
    int32_t maxCorrection;

    error = g_target_left_minus_right_x100 -
            g_measured_left_minus_right_x100;

    if (AbsInt32(error) <= PID_ERROR_DEADBAND_X100) error = 0;

    if (((error > 0) && (g_pid_previous_error_x100 < 0)) ||
        ((error < 0) && (g_pid_previous_error_x100 > 0))) {
        g_pid_integral_x100 = 0;
    }

    derivative = error - g_pid_previous_error_x100;
    oldIntegral = g_pid_integral_x100;
    candidateIntegral = ClampInt32(
        oldIntegral + error,
        -PID_INTEGRAL_LIMIT_X100,
        PID_INTEGRAL_LIMIT_X100
    );

    pOutput = (g_pid_kp_x1000 * error) / 1000L;
    iOutput = (g_pid_ki_x1000 * candidateIntegral) / 1000L;
    dOutput = (g_pid_kd_x1000 * derivative) / 1000L;
    rawOutput = pOutput + iOutput + dOutput;

    maxCorrection = AbsInt32(g_pid_max_correction_x100);
    if (maxCorrection > 1600) maxCorrection = 1600;

    limitedOutput = ClampInt32(rawOutput, -maxCorrection, maxCorrection);

    if (((rawOutput > maxCorrection) && (error > 0)) ||
        ((rawOutput < -maxCorrection) && (error < 0))) {
        candidateIntegral = oldIntegral;
        iOutput = (g_pid_ki_x1000 * candidateIntegral) / 1000L;
        rawOutput = pOutput + iOutput + dOutput;
        limitedOutput = ClampInt32(rawOutput, -maxCorrection, maxCorrection);
    }

    if (error == 0) candidateIntegral = (candidateIntegral * 95L) / 100L;

    g_pid_error_x100 = error;
    g_pid_derivative_x100 = derivative;
    g_pid_integral_x100 = candidateIntegral;
    g_pid_p_x100 = pOutput;
    g_pid_i_x100 = iOutput;
    g_pid_d_x100 = dOutput;
    g_pid_raw_correction_x100 = limitedOutput;
    g_pid_correction_x100 = MoveToward(
        g_pid_correction_x100,
        limitedOutput,
        PID_CORRECTION_STEP_X100
    );
    g_pid_previous_error_x100 = error;

    return g_pid_correction_x100;
}

/*==================================================
 * JY60解析
 *==================================================*/

static void JY60_ParseCompleteFrame(void)
{
    uint32_t i;
    uint8_t checksum = 0U;
    int16_t rawValue;
    int32_t newValue;
    int32_t difference;

    for (i = 0U; i < 10U; i++) checksum = (uint8_t)(checksum + g_jy60_frame[i]);

    if (checksum != g_jy60_frame[10]) {
        g_jy60_checksum_errors++;
        return;
    }

    if (g_jy60_frame[1] == 0x52U) {
        rawValue = (int16_t)(((uint16_t)g_jy60_frame[7] << 8U) |
                             g_jy60_frame[6]);
        newValue = (int32_t)(((int64_t)rawValue * 200000LL) / 32768LL);
        g_jy60_gyro_z_dps_x100 = newValue;

        if (g_jy60_gyro_frame_count == 0U)
            g_jy60_gyro_filtered_x100 = newValue;
        else
            g_jy60_gyro_filtered_x100 =
                (g_jy60_gyro_filtered_x100 + newValue) / 2L;

        g_jy60_last_gyro_ms = g_system_ms;
        g_jy60_gyro_frame_count++;
    }

    if (g_jy60_frame[1] == 0x53U) {
        rawValue = (int16_t)(((uint16_t)g_jy60_frame[7] << 8U) |
                             g_jy60_frame[6]);
        newValue = (int32_t)(((int64_t)rawValue * 18000LL) / 32768LL);
        g_jy60_yaw_deg_x100 = newValue;

        if (g_jy60_angle_frame_count == 0U) {
            g_jy60_yaw_filtered_x100 = newValue;
        } else {
            difference = WrapAngleErrorX100(newValue - g_jy60_yaw_filtered_x100);
            g_jy60_yaw_filtered_x100 += difference / 2L;
            g_jy60_yaw_filtered_x100 =
                NormalizeAngleX100(g_jy60_yaw_filtered_x100);
        }

        g_jy60_last_angle_ms = g_system_ms;
        g_jy60_angle_frame_count++;
    }
}

static void JY60_InputByte(uint8_t byte)
{
    if (g_jy60_frame_index == 0U) {
        if (byte == 0x55U) {
            g_jy60_frame[0] = byte;
            g_jy60_frame_index = 1U;
        }
        return;
    }

    if ((g_jy60_frame_index == 1U) && (byte == 0x55U)) {
        g_jy60_frame[0] = 0x55U;
        g_jy60_frame_index = 1U;
        return;
    }

    g_jy60_frame[g_jy60_frame_index++] = byte;

    if (g_jy60_frame_index >= 11U) {
        JY60_ParseCompleteFrame();
        g_jy60_frame_index = 0U;
    }
}

static void JY60_ProcessUART(void)
{
    uint8_t byte;
    while (DL_UART_Main_receiveDataCheck(JY60_UART_INST, &byte)) {
        JY60_InputByte(byte);
    }
}

static void JY60_UpdateOnlineState(void)
{
    uint32_t now = g_system_ms;

    g_jy60_online =
        ((g_jy60_angle_frame_count > 0U) &&
         ((uint32_t)(now - g_jy60_last_angle_ms) <= JY60_OFFLINE_TIMEOUT_MS)) ?
        1U : 0U;

    g_jy60_gyro_online =
        ((g_jy60_gyro_frame_count > 0U) &&
         ((uint32_t)(now - g_jy60_last_gyro_ms) <= JY60_OFFLINE_TIMEOUT_MS)) ?
        1U : 0U;
}

static void JY60_CaptureTargetHeading(void)
{
    g_target_yaw_deg_x100 = g_jy60_yaw_filtered_x100;
    g_heading_error_x100 = 0;
    g_heading_p_x100 = 0;
    g_heading_d_x100 = 0;
    g_heading_raw_correction_x100 = 0;
    g_heading_correction_x100 = 0;
    g_heading_deadband_active = 1U;
    g_heading_target_valid = 1U;
    g_last_used_angle_frame = g_jy60_angle_frame_count;
    g_last_used_gyro_frame = g_jy60_gyro_frame_count;
}

/*==================================================
 * 直线航向控制
 *==================================================*/

static void HeadingController_Update10ms(void)
{
    int32_t error;
    int32_t gyroRate;
    int32_t absoluteError;
    int32_t kp;
    int32_t rawCorrection;
    uint32_t hasNewData = 0U;

    __disable_irq();
    g_heading_control_due = 0U;
    __enable_irq();

    JY60_UpdateOnlineState();

    if (g_mission_state != MISSION_STRAIGHT) return;

    if (g_capture_heading_command != 0U) {
        if (g_jy60_online != 0U) JY60_CaptureTargetHeading();
        g_capture_heading_command = 0U;
    }

    if (g_manual_heading_correction_x100 != 0) {
        g_heading_raw_correction_x100 = ClampInt32(
            g_manual_heading_correction_x100,
            -HEADING_MAX_CORRECTION_X100,
            HEADING_MAX_CORRECTION_X100
        );
        g_heading_correction_x100 = MoveToward(
            g_heading_correction_x100,
            g_heading_raw_correction_x100,
            HEADING_CORRECTION_STEP_X100
        );
        g_heading_update_count++;
        return;
    }

    if ((g_jy60_online == 0U) || (g_heading_target_valid == 0U)) {
        g_heading_error_x100 = 0;
        g_heading_p_x100 = 0;
        g_heading_d_x100 = 0;
        g_heading_raw_correction_x100 = 0;
        g_heading_correction_x100 = MoveToward(
            g_heading_correction_x100,
            0,
            HEADING_CORRECTION_STEP_X100
        );
        return;
    }

    if (g_jy60_angle_frame_count != g_last_used_angle_frame) {
        g_last_used_angle_frame = g_jy60_angle_frame_count;
        hasNewData = 1U;
    }

    if (g_jy60_gyro_frame_count != g_last_used_gyro_frame) {
        g_last_used_gyro_frame = g_jy60_gyro_frame_count;
        hasNewData = 1U;
    }

    if (hasNewData != 0U) {
        error = WrapAngleErrorX100(
            g_jy60_yaw_filtered_x100 - g_target_yaw_deg_x100
        );
        gyroRate = g_jy60_gyro_filtered_x100;

        if (g_jy60_yaw_reverse != 0U) error = -error;
        if (g_jy60_gyro_reverse != 0U) gyroRate = -gyroRate;

        absoluteError = AbsInt32(error);

        if (g_heading_deadband_active != 0U) {
            if (absoluteError >= HEADING_DEADBAND_EXIT_X100)
                g_heading_deadband_active = 0U;
        } else {
            if (absoluteError <= HEADING_DEADBAND_ENTER_X100)
                g_heading_deadband_active = 1U;
        }

        if (g_heading_deadband_active != 0U) error = 0;

        if ((g_jy60_gyro_online == 0U) ||
            (AbsInt32(gyroRate) <= GYRO_DEADBAND_X100)) {
            gyroRate = 0;
        }

        g_heading_error_x100 = error;
        kp = (AbsInt32(error) <= HEADING_NEAR_ZONE_X100) ?
             HEADING_KP_NEAR_X100 : HEADING_KP_FAR_X100;

        g_heading_p_x100 = (kp * error) / 100L;
        g_heading_d_x100 = (HEADING_KD_X100 * gyroRate) / 100L;
        rawCorrection = g_heading_p_x100 + g_heading_d_x100;

        if (g_heading_output_reverse != 0U) rawCorrection = -rawCorrection;

        rawCorrection = ClampInt32(
            rawCorrection,
            -HEADING_MAX_CORRECTION_X100,
            HEADING_MAX_CORRECTION_X100
        );

        if ((error != 0) &&
            (AbsInt32(rawCorrection) < HEADING_MIN_ACTIVE_X100)) {
            rawCorrection = (rawCorrection >= 0) ?
                HEADING_MIN_ACTIVE_X100 : -HEADING_MIN_ACTIVE_X100;
        }

        g_heading_raw_correction_x100 = rawCorrection;
        g_heading_update_count++;
    }

    g_heading_correction_x100 = MoveToward(
        g_heading_correction_x100,
        g_heading_raw_correction_x100,
        HEADING_CORRECTION_STEP_X100
    );
}

/*==================================================
 * 直线PWM
 *==================================================*/

static void Motor_ApplyCorrection(int32_t basePWM,
                                  int32_t correctionX100,
                                  uint32_t startupMode)
{
    int32_t correctionPWM;
    int32_t leftTarget;
    int32_t rightTarget;

    correctionX100 = ClampInt32(
        correctionX100,
        -TOTAL_CORRECTION_LIMIT_X100,
        TOTAL_CORRECTION_LIMIT_X100
    );

    g_total_correction_x100 = correctionX100;
    correctionPWM = CorrectionX100ToPWM(correctionX100);

    /* 正修正：左轮快、右轮慢，小车向右修正。 */
    leftTarget = basePWM + correctionPWM;
    rightTarget = basePWM - correctionPWM;

    if (startupMode != 0U) {
        leftTarget = ClampInt32(leftTarget, 0, MOTOR_MAX_PWM);
        rightTarget = ClampInt32(rightTarget, 0, MOTOR_MAX_PWM);
    } else {
        leftTarget = ClampInt32(leftTarget, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
        rightTarget = ClampInt32(rightTarget, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    }

    g_left_target_pwm = leftTarget;
    g_right_target_pwm = rightTarget;
    g_left_pwm = MoveToward(g_left_pwm, g_left_target_pwm, PWM_CHANGE_LIMIT);
    g_right_pwm = MoveToward(g_right_pwm, g_right_target_pwm, PWM_CHANGE_LIMIT);
    Motor_UpdateDriverPWM();
}

static void StartMotorRamp(void)
{
    Straight_PID_Reset();
    g_current_base_pwm = 0;
    g_left_pwm = 0;
    g_right_pwm = 0;
    g_pid_stable_cycle_count = 0U;
    g_motor_output_enabled = 1U;
    g_run_state = 2U;
}

static void EncoderController_Update20ms(void)
{
    Encoder_UpdateSpeed20ms();
    JY60_UpdateOnlineState();

    if (g_pid_reset_command != 0U) {
        Straight_PID_Reset();
        g_pid_reset_command = 0U;
    }

    if (g_mission_state != MISSION_STRAIGHT) return;

    if (g_run_state == 2U) {
        if (g_current_base_pwm < MOTOR_BASE_PWM) {
            g_current_base_pwm += STARTUP_PWM_STEP;
            if (g_current_base_pwm > MOTOR_BASE_PWM)
                g_current_base_pwm = MOTOR_BASE_PWM;
        }

        g_pid_correction_x100 = MoveToward(
            g_pid_correction_x100,
            0,
            PID_CORRECTION_STEP_X100
        );

        if (g_current_base_pwm >= MOTOR_BASE_PWM) {
            g_pid_stable_cycle_count = 0U;
            Straight_PID_Reset();
            g_run_state = 3U;
        }
        return;
    }

    if (g_run_state == 3U) {
        g_pid_stable_cycle_count++;
        if (g_pid_stable_cycle_count >= PID_START_STABLE_CYCLES) {
            Straight_PID_Reset();
            g_run_state = ((g_jy60_online != 0U) &&
                           (g_heading_target_valid != 0U)) ? 4U : 5U;
        }
        return;
    }

    if ((g_run_state == 4U) || (g_run_state == 5U)) {
        Straight_PID_Update();
        g_run_state = ((g_jy60_online != 0U) &&
                       (g_heading_target_valid != 0U)) ? 4U : 5U;
    }
}

static void Control_ApplyCurrentOutput(void)
{
    int32_t basePWM;
    int32_t totalCorrection;
    uint32_t startupMode;

    if ((g_mission_state != MISSION_STRAIGHT) ||
        (g_motor_output_enabled == 0U)) {
        return;
    }

    if (g_run_state == 2U) {
        basePWM = g_current_base_pwm;
        startupMode = 1U;
    } else {
        uint32_t remainingCount;
        uint32_t slowZoneCount;
        uint32_t finalZoneCount;

        basePWM = MOTOR_BASE_PWM;
        startupMode = 0U;

        remainingCount =
            (g_distance_travel_count < g_distance_target_count) ?
            (g_distance_target_count - g_distance_travel_count) : 0U;

        slowZoneCount =
            DistanceMmToNormalizedCount(DISTANCE_SLOW_ZONE_MM);
        finalZoneCount =
            DistanceMmToNormalizedCount(DISTANCE_FINAL_ZONE_MM);

        if (remainingCount <= finalZoneCount) {
            basePWM = DISTANCE_FINAL_PWM;
        } else if (remainingCount <= slowZoneCount) {
            basePWM = DISTANCE_SLOW_PWM;
        }
    }

    totalCorrection = g_pid_correction_x100;

    if (((g_jy60_online != 0U) && (g_heading_target_valid != 0U)) ||
        (g_manual_heading_correction_x100 != 0)) {
        totalCorrection += g_heading_correction_x100;
    }

    Motor_ApplyCorrection(basePWM, totalCorrection, startupMode);
}

/*==================================================
 * 任务状态机
 *==================================================*/

static void Mission_Fail(uint8_t errorCode)
{
    Motor_Stop();
    g_mission_error_code = errorCode;
    g_mission_state = MISSION_ERROR;
    K230_SendError(errorCode);
}

static void Mission_BeginStraight(void)
{
    Motor_Stop();
    Motor_SetForwardDirection();

    g_distance_start_left = g_left_encoder_total;
    g_distance_start_right = g_right_encoder_total;
    g_distance_left_delta = 0U;
    g_distance_right_delta = 0U;
    g_distance_travel_count = 0U;
    g_distance_target_count =
        DistanceMmToNormalizedCount(g_mission_distance_mm);

    Straight_PID_Reset();
    g_left_speed_filtered_x100 = 0;
    g_right_speed_filtered_x100 = 0;

    if (g_jy60_online != 0U) {
        JY60_CaptureTargetHeading();
    } else {
        g_heading_target_valid = 0U;
    }

    StartMotorRamp();
    g_mission_state = MISSION_STRAIGHT;
    g_state_start_ms = g_system_ms;
}

static void Mission_BeginTurn(void)
{
    int32_t delta;

    Motor_Stop();
    JY60_UpdateOnlineState();

    if (g_jy60_online == 0U) {
        Mission_Fail(2U);  /* 转弯前JY60离线 */
        return;
    }

#if JY60_LEFT_TURN_YAW_INCREASES != 0
    delta = (g_mission_turn == TURN_LEFT) ? TURN_ANGLE_X100 : -TURN_ANGLE_X100;
#else
    delta = (g_mission_turn == TURN_LEFT) ? -TURN_ANGLE_X100 : TURN_ANGLE_X100;
#endif

    /* 以直线目标朝向为基准，保证最终相对转90度。 */
    g_turn_target_yaw_x100 = NormalizeAngleX100(
        g_target_yaw_deg_x100 + delta
    );

    g_turn_error_x100 = WrapAngleErrorX100(
        g_turn_target_yaw_x100 - g_jy60_yaw_filtered_x100
    );
    g_turn_gyro_x100 = g_jy60_gyro_filtered_x100;
    g_turn_pwm = 0;
    g_turn_stable_count = 0U;
    g_turn_start_ms = g_system_ms;
    g_turn_direction_output = 0U;
    g_heading_target_valid = 0U;
    g_mission_state = MISSION_TURN;
}

static void Mission_Finish(void)
{
    uint8_t command = g_last_command;

    Motor_Stop();
    g_mission_state = MISSION_IDLE;
    g_mission_turn = TURN_NONE;
    g_mission_distance_mm = 0U;
    g_heading_target_valid = 0U;
    K230_SendFinished(command);
}

static void TurnController_Update10ms(void)
{
    int32_t error;
    int32_t gyroRate;
    int32_t absoluteError;
    int32_t pwm;
    uint32_t turnLeft;

    JY60_UpdateOnlineState();

    if (g_mission_state != MISSION_TURN) return;

    if (g_jy60_online == 0U) {
        Mission_Fail(3U);  /* 转弯过程中JY60掉线 */
        return;
    }

    error = WrapAngleErrorX100(
        g_turn_target_yaw_x100 - g_jy60_yaw_filtered_x100
    );
    gyroRate = g_jy60_gyro_filtered_x100;

    if (g_jy60_gyro_reverse != 0U) gyroRate = -gyroRate;

    absoluteError = AbsInt32(error);
    g_turn_error_x100 = error;
    g_turn_gyro_x100 = gyroRate;

    if (absoluteError <= TURN_TOLERANCE_X100) {
        Motor_Stop();
        g_turn_pwm = 0;

        if ((g_jy60_gyro_online != 0U) &&
            (AbsInt32(gyroRate) <= TURN_RATE_LIMIT_X100)) {
            g_turn_stable_count++;
        } else {
            g_turn_stable_count = 0U;
        }

        if (g_turn_stable_count >= TURN_STABLE_CYCLES) {
            g_mission_state = MISSION_TURN_SETTLE;
            g_state_start_ms = g_system_ms;
        }
        return;
    }

    g_turn_stable_count = 0U;

    if (absoluteError > TURN_FAST_ZONE_X100)
        pwm = TURN_FAST_PWM;
    else if (absoluteError > TURN_MEDIUM_ZONE_X100)
        pwm = TURN_MEDIUM_PWM;
    else if (absoluteError > TURN_SLOW_ZONE_X100)
        pwm = TURN_SLOW_PWM;
    else
        pwm = TURN_CREEP_PWM;

    /* 角速度很大且已靠近目标时，提前降一级，减少过冲。 */
    if ((absoluteError < TURN_MEDIUM_ZONE_X100) &&
        (AbsInt32(gyroRate) > 1500L)) {
        pwm -= 3;
        if (pwm < TURN_CREEP_PWM) pwm = TURN_CREEP_PWM;
    }

#if JY60_LEFT_TURN_YAW_INCREASES != 0
    turnLeft = (error > 0) ? 1U : 0U;
#else
    turnLeft = (error < 0) ? 1U : 0U;
#endif

    if (turnLeft != 0U) {
        if (g_turn_direction_output != 1U) {
            Motor_Stop();
            Motor_SetLeftTurnDirection();
            g_turn_direction_output = 1U;
        }
    } else {
        if (g_turn_direction_output != 2U) {
            Motor_Stop();
            Motor_SetRightTurnDirection();
            g_turn_direction_output = 2U;
        }
    }

    g_turn_pwm = pwm;
    Motor_ApplyTurnPWM(pwm);
}

static void Mission_Update20ms(void)
{
    uint32_t travelCount;

    JY60_UpdateOnlineState();

    switch (g_mission_state) {
        case MISSION_IDLE:
            Motor_Stop();
            break;

        case MISSION_WAIT_JY60:
            Motor_Stop();

            if ((g_jy60_online != 0U) &&
                (g_jy60_angle_frame_count >= JY60_REQUIRED_ANGLE_FRAMES)) {
                Mission_BeginStraight();
            } else if ((uint32_t)(g_system_ms - g_command_wait_start_ms) >=
                       COMMAND_WAIT_JY60_TIMEOUT_MS) {
                Mission_Fail(1U);  /* 等待JY60超时 */
            }
            break;

        case MISSION_STRAIGHT:
            travelCount = Mission_UpdateTravelCount();

            if (travelCount >= g_distance_target_count) {
                Motor_Stop();
                g_mission_state = MISSION_STRAIGHT_SETTLE;
                g_state_start_ms = g_system_ms;
            }
            break;

        case MISSION_STRAIGHT_SETTLE:
            Motor_Stop();
            if ((uint32_t)(g_system_ms - g_state_start_ms) >=
                STRAIGHT_STOP_SETTLE_MS) {
                Mission_BeginTurn();
            }
            break;

        case MISSION_TURN:
            if ((uint32_t)(g_system_ms - g_turn_start_ms) >= TURN_TIMEOUT_MS) {
                Mission_Fail(4U);  /* 转弯超时 */
            }
            break;

        case MISSION_TURN_SETTLE:
            Motor_Stop();
            if ((uint32_t)(g_system_ms - g_state_start_ms) >=
                TURN_STOP_SETTLE_MS) {
                Mission_Finish();
            }
            break;

        case MISSION_ERROR:
            Motor_Stop();
            /* 报错后允许下一条有效命令重新启动。 */
            break;

        default:
            Mission_Fail(5U);
            break;
    }
}

static uint8_t DecodeCommand(uint8_t byte)
{
    if ((byte >= (uint8_t)'1') && (byte <= (uint8_t)'4'))
        return (uint8_t)(byte - (uint8_t)'0');

    if ((byte >= 1U) && (byte <= 4U)) return byte;

    return 0U;
}

static void Mission_AcceptCommand(uint8_t command)
{
    if ((g_mission_state != MISSION_IDLE) &&
        (g_mission_state != MISSION_ERROR)) {
        K230_SendByte((uint8_t)'B');
        return;
    }

    g_last_command = command;
    g_mission_error_code = 0U;

    switch (command) {
        case 1U:
            g_mission_distance_mm = DISTANCE_1M_MM;
            g_mission_turn = TURN_LEFT;
            break;

        case 2U:
            g_mission_distance_mm = DISTANCE_1M_MM;
            g_mission_turn = TURN_RIGHT;
            break;

        case 3U:
            g_mission_distance_mm = DISTANCE_2M_MM;
            g_mission_turn = TURN_LEFT;
            break;

        case 4U:
            g_mission_distance_mm = DISTANCE_2M_MM;
            g_mission_turn = TURN_RIGHT;
            break;

        default:
            return;
    }

    Motor_Stop();
    g_heading_target_valid = 0U;
    g_command_wait_start_ms = g_system_ms;
    g_mission_state = MISSION_WAIT_JY60;
    g_k230_valid_command_count++;
    K230_SendAccepted(command);
}

static void K230_ProcessUART(void)
{
    uint8_t byte;
    uint8_t command;

    while (DL_UART_Main_receiveDataCheck(K230_UART_INST, &byte)) {
        g_k230_last_rx = byte;
        g_k230_rx_count++;
        command = DecodeCommand(byte);

        if (command != 0U) Mission_AcceptCommand(command);
    }
}

/*==================================================
 * SysTick
 *==================================================*/

void SysTick_Handler(void)
{
    uint32_t gpioValue;
    uint32_t leftCurrentState;
    uint32_t rightCurrentState;
    uint32_t tableIndex;
    int32_t step;

    g_systick_heartbeat++;

    g_ms_divider++;
    if (g_ms_divider >= SYSTICK_TICKS_PER_MS) {
        g_ms_divider = 0U;
        g_system_ms++;
    }

    gpioValue = DL_GPIO_readPins(
        GPIOA,
        LEFT_ENCODER_A_PIN | LEFT_ENCODER_B_PIN |
        RIGHT_ENCODER_A_PIN | RIGHT_ENCODER_B_PIN
    );

    leftCurrentState = Encoder_ReadLeftState(gpioValue);
    rightCurrentState = Encoder_ReadRightState(gpioValue);

    if (leftCurrentState != g_left_previous_state) {
        tableIndex = (g_left_previous_state << 2U) | leftCurrentState;
        step = g_quadrature_table[tableIndex];

        if (step != 0) {
            g_left_period_count++;
            g_left_encoder_total++;
        } else {
            g_left_invalid_count++;
        }
        g_left_previous_state = leftCurrentState;
    }

    if (rightCurrentState != g_right_previous_state) {
        tableIndex = (g_right_previous_state << 2U) | rightCurrentState;
        step = g_quadrature_table[tableIndex];

        if (step != 0) {
            g_right_period_count++;
            g_right_encoder_total++;
        } else {
            g_right_invalid_count++;
        }
        g_right_previous_state = rightCurrentState;
    }

    g_pwm_phase++;
    if (g_pwm_phase >= PWM_PERIOD_COUNT) g_pwm_phase = 0U;

    if (g_motor_output_enabled == 0U) {
        DriverA_Disable();
        DriverB_Disable();
    } else {
        if (g_pwm_phase < (uint32_t)g_driver_a_pwm)
            DriverA_Enable();
        else
            DriverA_Disable();

        if (g_pwm_phase < (uint32_t)g_driver_b_pwm)
            DriverB_Enable();
        else
            DriverB_Disable();
    }

    g_heading_control_tick++;
    if (g_heading_control_tick >= HEADING_CONTROL_INTERVAL_TICKS) {
        g_heading_control_tick = 0U;
        g_heading_control_due = 1U;
    }

    g_encoder_control_tick++;
    if (g_encoder_control_tick >= ENCODER_CONTROL_INTERVAL_TICKS) {
        g_encoder_control_tick = 0U;
        g_encoder_control_due = 1U;
    }
}

/*==================================================
 * 主函数
 *==================================================*/

int main(void)
{
    uint32_t systickResult;
    uint32_t controlUpdated;

    SYSCFG_DL_init();
    Motor_GPIO_Init();
    Motor_SetForwardDirection();
    Encoder_Initialize();
    Motor_Stop();
    Straight_PID_Reset();

    systickResult = SysTick_Config(CPUCLK_FREQ / SYSTICK_FREQUENCY_HZ);

    if (systickResult != 0U) {
        g_run_state = 900U;
        Motor_Stop();
        while (1) __WFI();
    }

    g_mission_state = MISSION_IDLE;

    while (1) {
        K230_ProcessUART();
        JY60_ProcessUART();
        controlUpdated = 0U;

        if (g_heading_control_due != 0U) {
            if (g_mission_state == MISSION_TURN) {
                __disable_irq();
                g_heading_control_due = 0U;
                __enable_irq();
                TurnController_Update10ms();
            } else {
                HeadingController_Update10ms();
            }
            controlUpdated = 1U;
        }

        if (g_encoder_control_due != 0U) {
            EncoderController_Update20ms();
            Mission_Update20ms();
            controlUpdated = 1U;
        }

        if (controlUpdated != 0U) {
            Control_ApplyCurrentOutput();
        }

        __WFI();
    }
}

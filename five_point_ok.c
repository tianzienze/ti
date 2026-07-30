#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * MSPM0G3507：K230钢球一维位置UART接收测试
 *
 * 作用：
 * 1. 只测试K230 -> M0连续通信，不控制电机、不控制舵机；
 * 2. 接收K230发送的ASCII帧：
 *      B,1,-37\n   表示位置有效，钢球位置为-37mm
 *      B,0,0\n     表示当前未识别到钢球/位置无效
 * 3. 在CCS / Keil调试器Expressions窗口观察全局变量；
 * 4. 统计每秒收到的完整有效协议帧数量，验证约28Hz连续通信。
 *
 * 接线保持原成功版本不变：
 *   K230 IO32 / UART3_TXD -> MSPM0 PA1 / K230_UART_RX
 *   K230 IO33 / UART3_RXD <- MSPM0 PA0 / K230_UART_TX（本测试可不接）
 *   K230 GND              -- MSPM0 GND
 *
 * UART：
 *   115200 baud，8N1
 *
 * 使用要求：
 * - SysConfig中的UART名称仍为K230_UART；
 * - PA1配置为RX，PA0配置为TX，115200 8N1；
 * - 本文件应作为测试工程唯一的main.c；
 * - 不要同时编译旧任务状态机main.c，否则会出现重复main或重复SysTick_Handler。
 */

#ifndef K230_UART_INST
#error "K230_UART_INST未定义：请确认SysConfig中的UART名称为K230_UART"
#endif

#ifndef CPUCLK_FREQ
#define CPUCLK_FREQ 32000000UL
#endif

#define RX_LINE_BUFFER_SIZE       24U
#define LINK_TIMEOUT_MS           200U
#define POSITION_MIN_MM           (-125)
#define POSITION_MAX_MM           125

/* ============================================================
 * 调试器重点观察的变量
 * ============================================================ */

/* 当前最近一帧的位置状态。 */
volatile uint32_t g_ball_position_valid = 0U;
volatile int32_t  g_ball_position_mm = 0;

/* 最近一次valid=1时的位置。丢球后仍保留，便于观察，但控制时不能继续盲用。 */
volatile int32_t  g_last_valid_position_mm = 0;

/* 收发与解析统计。 */
volatile uint8_t  g_k230_last_rx_byte = 0U;
volatile uint32_t g_k230_rx_byte_count = 0U;
volatile uint32_t g_k230_complete_line_count = 0U;
volatile uint32_t g_k230_parsed_frame_count = 0U;
volatile uint32_t g_k230_malformed_frame_count = 0U;
volatile uint32_t g_k230_overflow_count = 0U;

volatile uint32_t g_ball_valid_frame_count = 0U;
volatile uint32_t g_ball_invalid_frame_count = 0U;

/* 最近一秒收到多少个“成功解析的协议帧”。正常预计约28帧/秒。 */
volatile uint32_t g_k230_frames_per_second = 0U;

/* 链路在线状态：200ms内收到过正确协议帧则为1。 */
volatile uint32_t g_k230_link_online = 0U;
volatile uint32_t g_last_frame_age_ms = 0xFFFFFFFFU;

/* 每收到并解析一帧就加1，可用于设置调试器断点或观察变化。 */
volatile uint32_t g_position_update_count = 0U;

/* 最近一行原始文本与解析结果。 */
volatile char     g_last_frame_text[RX_LINE_BUFFER_SIZE];
volatile uint32_t g_last_frame_length = 0U;
volatile uint32_t g_last_frame_parse_ok = 0U;

/* 主程序和时间基准。 */
volatile uint32_t g_system_ms = 0U;
volatile uint32_t g_main_loop_count = 0U;

/* ============================================================
 * 内部接收状态
 * ============================================================ */

static char g_rx_line[RX_LINE_BUFFER_SIZE];
static uint32_t g_rx_line_index = 0U;

static volatile uint32_t g_last_good_frame_ms = 0U;
static volatile uint32_t g_frames_this_second = 0U;
static volatile uint32_t g_second_divider_ms = 0U;
static volatile uint32_t g_has_received_good_frame = 0U;

/* ============================================================
 * 工具函数
 * ============================================================ */

static void CopyLastFrame(const char *text, uint32_t length)
{
    uint32_t i;
    uint32_t copyLength = length;

    if (copyLength >= RX_LINE_BUFFER_SIZE) {
        copyLength = RX_LINE_BUFFER_SIZE - 1U;
    }

    for (i = 0U; i < copyLength; i++) {
        g_last_frame_text[i] = text[i];
    }

    g_last_frame_text[copyLength] = '\0';
    g_last_frame_length = copyLength;
}

/*
 * 从text[index]开始解析一个有符号十进制整数。
 * 成功返回1，并要求数字必须一直解析到字符串末尾。
 */
static uint32_t ParseSignedInteger(
    const char *text,
    uint32_t length,
    uint32_t startIndex,
    int32_t *valueOut
)
{
    uint32_t index = startIndex;
    uint32_t digitCount = 0U;
    int32_t sign = 1;
    int32_t value = 0;

    if ((text == 0) || (valueOut == 0) || (index >= length)) {
        return 0U;
    }

    if (text[index] == '-') {
        sign = -1;
        index++;
    } else if (text[index] == '+') {
        index++;
    }

    if (index >= length) {
        return 0U;
    }

    while (index < length) {
        char character = text[index];

        if ((character < '0') || (character > '9')) {
            return 0U;
        }

        value = value * 10 + (int32_t)(character - '0');
        digitCount++;

        /* 防止异常长数字溢出；实际位置只允许±125mm。 */
        if (value > 10000) {
            return 0U;
        }

        index++;
    }

    if (digitCount == 0U) {
        return 0U;
    }

    *valueOut = value * sign;
    return 1U;
}

/*
 * 严格解析：
 *   B,1,-37
 *   B,0,0
 *
 * 返回1表示协议格式正确。
 */
static uint32_t ParseBallFrame(
    const char *frame,
    uint32_t length,
    uint32_t *positionValidOut,
    int32_t *positionMmOut
)
{
    uint32_t positionValid;
    int32_t positionMm;

    if ((frame == 0) ||
        (positionValidOut == 0) ||
        (positionMmOut == 0)) {
        return 0U;
    }

    /* 最短帧是B,0,0，共5个字符。 */
    if (length < 5U) {
        return 0U;
    }

    if ((frame[0] != 'B') ||
        (frame[1] != ',') ||
        (frame[3] != ',')) {
        return 0U;
    }

    if (frame[2] == '0') {
        positionValid = 0U;
    } else if (frame[2] == '1') {
        positionValid = 1U;
    } else {
        return 0U;
    }

    if (ParseSignedInteger(
            frame,
            length,
            4U,
            &positionMm
        ) == 0U) {
        return 0U;
    }

    if (positionValid == 0U) {
        /* K230的无效帧必须严格是B,0,0。 */
        if (positionMm != 0) {
            return 0U;
        }
    } else {
        /* 当前物理凹槽范围为-125mm到+125mm。 */
        if ((positionMm < POSITION_MIN_MM) ||
            (positionMm > POSITION_MAX_MM)) {
            return 0U;
        }
    }

    *positionValidOut = positionValid;
    *positionMmOut = positionMm;
    return 1U;
}

static void HandleCompleteLine(const char *line, uint32_t length)
{
    uint32_t positionValid = 0U;
    int32_t positionMm = 0;

    g_k230_complete_line_count++;
    CopyLastFrame(line, length);

    if (ParseBallFrame(
            line,
            length,
            &positionValid,
            &positionMm
        ) == 0U) {
        g_last_frame_parse_ok = 0U;
        g_k230_malformed_frame_count++;
        return;
    }

    g_last_frame_parse_ok = 1U;
    g_k230_parsed_frame_count++;
    g_frames_this_second++;
    g_position_update_count++;

    g_ball_position_valid = positionValid;

    if (positionValid != 0U) {
        g_ball_position_mm = positionMm;
        g_last_valid_position_mm = positionMm;
        g_ball_valid_frame_count++;
    } else {
        /*
         * 这里把当前显示值置0，但必须结合g_ball_position_valid判断。
         * valid=0时的0绝不能当作“钢球真的在中心”。
         */
        g_ball_position_mm = 0;
        g_ball_invalid_frame_count++;
    }

    g_last_good_frame_ms = g_system_ms;
    g_has_received_good_frame = 1U;
    g_last_frame_age_ms = 0U;
    g_k230_link_online = 1U;
}

static void K230_InputByte(uint8_t byte)
{
    g_k230_last_rx_byte = byte;
    g_k230_rx_byte_count++;

    /* 兼容CRLF：忽略\r，以\n作为唯一帧结束符。 */
    if (byte == (uint8_t)'\r') {
        return;
    }

    if (byte == (uint8_t)'\n') {
        if (g_rx_line_index > 0U) {
            g_rx_line[g_rx_line_index] = '\0';
            HandleCompleteLine(g_rx_line, g_rx_line_index);
        }

        g_rx_line_index = 0U;
        return;
    }

    if (g_rx_line_index < (RX_LINE_BUFFER_SIZE - 1U)) {
        g_rx_line[g_rx_line_index++] = (char)byte;
    } else {
        /*
         * 一行超过缓冲区：丢弃当前行并重新同步。
         * 后续收到\n后会从下一行重新开始。
         */
        g_rx_line_index = 0U;
        g_k230_overflow_count++;
    }
}

static void K230_ProcessUART(void)
{
    uint8_t byte;

    while (DL_UART_Main_receiveDataCheck(K230_UART_INST, &byte)) {
        K230_InputByte(byte);
    }
}

/* ============================================================
 * 1ms时间基准与接收频率统计
 * ============================================================ */

void SysTick_Handler(void)
{
    g_system_ms++;

    if (g_has_received_good_frame != 0U) {
        g_last_frame_age_ms = g_system_ms - g_last_good_frame_ms;

        if (g_last_frame_age_ms > LINK_TIMEOUT_MS) {
            g_k230_link_online = 0U;
        } else {
            g_k230_link_online = 1U;
        }
    } else {
        g_last_frame_age_ms = 0xFFFFFFFFU;
        g_k230_link_online = 0U;
    }

    g_second_divider_ms++;

    if (g_second_divider_ms >= 1000U) {
        g_second_divider_ms = 0U;
        g_k230_frames_per_second = g_frames_this_second;
        g_frames_this_second = 0U;
    }
}

static void StartSysTick1ms(void)
{
    SysTick->LOAD = (CPUCLK_FREQ / 1000U) - 1U;
    SysTick->VAL = 0U;
    SysTick->CTRL =
        SysTick_CTRL_CLKSOURCE_Msk |
        SysTick_CTRL_TICKINT_Msk |
        SysTick_CTRL_ENABLE_Msk;
}

/* ============================================================
 * 主函数
 * ============================================================ */

int main(void)
{
    SYSCFG_DL_init();
    StartSysTick1ms();

    /*
     * 不启用电机、编码器、JY60或舵机。
     * 本测试只轮询K230_UART并更新上面的volatile调试变量。
     */
    while (1) {
        K230_ProcessUART();
        g_main_loop_count++;
    }
}

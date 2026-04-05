#pragma once

#include "hal/interface.hh"

#include "util/font6x8.h"

namespace bsp::screen
{
    template <typename T>
    concept ScreenConcept = requires(T screen) {
        { screen.init() } -> std::same_as<void>;                            // 初始化函数，返回 void
        { screen.Width } -> std::convertible_to<uint16_t>;                  // 屏幕宽度，单位像素
        { screen.Height } -> std::convertible_to<uint16_t>;                 // 屏幕高度，单位像素
        { screen.transaction() } -> std::same_as<hal::i2c::I2CTransaction>; // 提供给 I2C Handler 使用的 Transaction 结构
        { screen.clear() } -> std::same_as<void>;                           // 清屏函数，返回 void
        { screen.draw_pixel(0, 0, Color::Black) } -> std::same_as<void>;    // 画点函数，参数为坐标和颜色，返回 void
    };

    /**
     * @brief 向屏幕上绘制位图
     *
     * @tparam T 屏幕类型，必须满足 ScreenConcept
     * @param screen 屏幕对象
     * @param x 位图的左上角 x 坐标
     * @param y 位图的左上角 y 坐标
     * @param w 位图的宽度（像素）
     * @param h 位图的高度（像素）
     * @param bitmap 位图数据，按行扫描，每行占用 (w + 7) / 8 字节
     */
    template <ScreenConcept T>
    void draw_bitmap(T &screen, int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *bitmap)
    {
        // 计算输入位图每行占用的字节数（处理非 8 倍数宽度）
        int16_t byteWidth = (w + 7) / 8;

        for (int16_t j = 0; j < h; j++) {
            for (int16_t i = 0; i < w; i++) {
                // 检查输入位图在 (i, j) 处的像素是否为 1
                // 假设输入位图是常见的水平扫描（如 Image2Lcd 生成的格式）
                if (bitmap[j * byteWidth + (i / 8)] & (0x80 >> (i % 8))) {
                    screen.draw_pixel(x + i, y + j, Color::White);
                } else {
                    screen.draw_pixel(x + i, y + j, Color::Black);
                }
            }
        }
    }

    /**
     * @brief 向屏幕上绘制 ASCII 字符
     *
     * @tparam T 屏幕类型，必须满足 ScreenConcept
     * @param screen 屏幕对象
     * @param x 字符的左上角 x 坐标
     * @param y 字符的左上角 y 坐标
     * @param c 字符，必须是 32-126 的可打印字符
     * @param color 显示颜色，true 为屏幕显示颜色，false 为屏幕背景色
     */
    template <ScreenConcept T>
    void draw_char(T &screen, int16_t x, int16_t y, char c, bool color)
    {
        if (c < 32 || c > 126) return; // 过滤非打印字符

        uint8_t index = c;

        // 遍历字符的 6 列
        for (uint8_t i = 0; i < 6; i++) {
            uint8_t column_data = font6x8_ascii[index][i];
            // 遍历每一列的 8 个像素 (从下往上或从上往下，由字库决定)
            for (uint8_t j = 0; j < 8; j++) {
                if (column_data & (1 << j)) {
                    screen.draw_pixel(x + i, y + j, color);
                }
            }
        }
    }

    /**
     * @brief 向屏幕上绘制字符串
     *
     * @tparam T 屏幕类型，必须满足 ScreenConcept
     * @param screen 屏幕对象
     * @param x 字符串的左上角 x 坐标
     * @param y 字符串的左上角 y 坐标
     * @param str 字符串
     * @param color 显示颜色，true 为屏幕显示颜色，false 为屏幕背景色
     */
    template <ScreenConcept T>
    void draw_string(T &screen, int16_t x, int16_t y, const char *str, bool color)
    {
        while (*str) {
            draw_char(screen, x, y, *str++, color);
            x += 6;                       // 字符宽度为 6
            if (x + 6 >= T::Width) break; // 简单的边界检查
        }
    }
} // namespace bsp::screen

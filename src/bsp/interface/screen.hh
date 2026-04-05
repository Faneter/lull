#pragma once

#include "hal/interface.hh"

#include "util/font6x8.h"

namespace bsp::screen
{
    template <typename T>
    concept ScreenConcept = requires(T screen) {
        { screen.init() } -> std::same_as<void>;
        { screen.Width } -> std::convertible_to<uint16_t>;
        { screen.Height } -> std::convertible_to<uint16_t>;
        { screen.transaction() } -> std::same_as<hal::i2c::I2CTransaction>;
        { screen.clear() } -> std::same_as<void>;
        { screen.draw_pixel(0, 0, Color::Black) } -> std::same_as<void>;
        { screen.draw_bitmap(0, 0, 16, 16, nullptr) } -> std::same_as<void>;
    };

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

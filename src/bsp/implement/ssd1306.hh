#pragma once

#include <cstring>

#include "hal/i2c.hh"
#include "util/font6x8.h"

namespace bsp::screen
{
    enum AddressingMode {
        Horizontal = 0b00,
        Vertical   = 0b01,
        Page       = 0b10,
        INVALID    = 0b11
    };

    enum Color {
        White = 0,
        Black = 1
    };

    enum WrapType {
        NoWrap,
        WrapDisplay,
        WrapCoord
    };

    template <typename i2c_bus, size_t Address, size_t Width, size_t Height>
        requires hal::i2c::HasI2CHandleConcept<i2c_bus>
    class SSD1306
    {
    public:
        static inline constexpr size_t Buffer_Size = Width * Height / 8;

        hal::i2c::I2CTransaction transaction()
        {
            setColumnAddressScope(0, Width - 1);
            setPageAddressScope(0, (Height / 8) - 1);
            return _transaction;
        }

        SSD1306()
        {
        }

        void init()
        {
            switch_oled(false);
            setMemoryAddressingMode(Horizontal);

            setColumnAddressScope(0, Width - 1);      // 0 到 127
            setPageAddressScope(0, (Height / 8) - 1); // 0 到 7 (针对 64 线屏幕)

            i2c_bus::template write_mem<hal::Mode::Normal>(
                static_cast<uint16_t>(Address << 1),
                0x40, // 数据寄存器
                1,
                buffer,
                Buffer_Size);

            write_command(0x40); //--set start line address - CHECK

            setContrast(0xFF);

            flip(false);

            setInverse(false);

            setMultiplexRatio(0x3F);

            setDisplayOn(true); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content

            write_command(0xD3); //-set display offset - CHECK
            write_command(0x00);

            setOscillatorFrequency(0xf0);

            setPrecharge(0x22);

            setCOMPinsHardwareConfiguration(0x12);

            adjustVcomDeselectLevel(0x20);

            chargePump(true);

            switch_oled(true);
        }

        void clear()
        {
            std::memset(buffer, 0x00, Buffer_Size);
        }

        /**
         * @brief 在缓冲区指定位置画点
         */
        void draw_pixel(int16_t x, int16_t y, bool color)
        {
            if (x < 0 || x >= Width || y < 0 || y >= Height) return;

            // 水平模式下，缓冲区的组织方式依然是：
            // 第 0 字节 = 第 0 列的第 0-7 行
            // 第 1 字节 = 第 1 列的第 0-7 行
            if (color) {
                buffer[x + (y / 8) * Width] |= (1 << (y % 8));
            } else {
                buffer[x + (y / 8) * Width] &= ~(1 << (y % 8));
            }
        }

        /**
         * @brief 将位图写入缓冲区
         * @param x, y 起始坐标
         * @param w, h 位图宽高
         * @param bitmap 位图数组
         */
        void draw_bitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *bitmap)
        {
            // 计算输入位图每行占用的字节数（处理非 8 倍数宽度）
            int16_t byteWidth = (w + 7) / 8;

            for (int16_t j = 0; j < h; j++) {
                for (int16_t i = 0; i < w; i++) {
                    // 检查输入位图在 (i, j) 处的像素是否为 1
                    // 假设输入位图是常见的水平扫描（如 Image2Lcd 生成的格式）
                    if (bitmap[j * byteWidth + (i / 8)] & (0x80 >> (i % 8))) {
                        draw_pixel(x + i, y + j, true);
                    }
                }
            }
        }

        /**
         * @brief 显示单个字符 (适配 6x8 字库)
         */
        void draw_char(int16_t x, int16_t y, char c, bool color)
        {
            if (c < 32 || c > 126) return; // 过滤非打印字符

            uint8_t index = c;

            // 遍历字符的 6 列
            for (uint8_t i = 0; i < 6; i++) {
                uint8_t column_data = font6x8_ascii[index][i];
                // 遍历每一列的 8 个像素 (从下往上或从上往下，由字库决定)
                for (uint8_t j = 0; j < 8; j++) {
                    if (column_data & (1 << j)) {
                        draw_pixel(x + i, y + j, color);
                    }
                }
            }
        }

        /**
         * @brief 显示字符串
         */
        void draw_string(int16_t x, int16_t y, const char *str, bool color)
        {
            while (*str) {
                draw_char(x, y, *str++, color);
                x += 6;                    // 字符宽度为 6
                if (x + 6 >= Width) break; // 简单的边界检查
            }
        }

    private:
        uint8_t buffer[Buffer_Size];
        hal::i2c::I2CTransaction _transaction = {
            .type          = hal::i2c::TransactionType::WriteMem,
            .dev_addr      = static_cast<uint16_t>(Address << 1),
            .mem_addr      = 0x40,
            .mem_addr_size = 1,
            .data_ptr      = buffer,
            .size          = Buffer_Size,
            .context       = this,
            .user_callback = [](hal::i2c::I2CTransaction *transaction) {},
        };

        void write_command(uint8_t command)
        {
            i2c_bus::template write_mem<hal::Mode::Normal>(static_cast<uint16_t>(Address << 1), 0x00, 1, &command, 1);
        }

        void switch_oled(bool state)
        {
            if (state) {
                write_command(0xAF);
            } else {
                write_command(0xAE);
            }
        }

        void setOscillatorFrequency(uint8_t frequency)
        {
            write_command(0xd5);
            write_command(frequency);
        }

        void setMultiplexRatio(uint8_t ratio)
        {
            write_command(0xa8);
            write_command(ratio);
        }

        void setInverse(bool inverse)
        {
            auto cmd = (uint8_t)(inverse ? 0xA7 : 0xA6);
            write_command(cmd);
        }

        void chargePump(bool chargeOn)
        {
            write_command(0x8D);
            if (chargeOn) {
                write_command(0x14);
            } else {
                write_command(0x10);
            }
        }

        void setContrast(uint8_t contrast)
        {
            write_command(0x81);
            write_command(contrast);
        }

        void setPrecharge(uint8_t precharge)
        {
            write_command(0xd9);
            write_command(precharge);
        }

        void setCOMPinsHardwareConfiguration(uint8_t val)
        {
            write_command(0xda);
            write_command(static_cast<uint8_t>(0b00110010 & val));
        }

        void adjustVcomDeselectLevel(uint8_t level)
        {
            write_command(0xdb);
            write_command(level);
        }

        void setDisplayOn(bool state)
        {
            auto cmd = (uint8_t)(state ? 0xA4 : 0xA5);
            write_command(cmd);
        }

        void setMemoryAddressingMode(AddressingMode mode)
        {
            write_command(0x20);
            write_command(mode);
        }

        void setColumnAddressScope(uint8_t lower, uint8_t upper)
        {
            write_command(0x21);
            write_command(lower);
            write_command(upper);
        }

        void setPageAddressScope(uint8_t lower, uint8_t upper)
        {
            write_command(0x22);
            write_command(lower);
            write_command(upper);
        }

        void flip(bool enable)
        {
            if (enable) {
                write_command(0xA0); // 左右翻转
                write_command(0xC0); // 上下翻转
            } else {
                write_command(0xA1);
                write_command(0xC8);
            }
        }
    };
} // namespace bsp::screen

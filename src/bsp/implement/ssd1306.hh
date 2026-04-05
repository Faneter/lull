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

    template <typename i2c_bus>
        requires hal::i2c::HasI2CHandleConcept<i2c_bus>
    class SSD1306
    {
    public:
        static inline constexpr uint16_t Address     = (0x3C << 1);
        static inline constexpr uint16_t Width       = 128;
        static inline constexpr uint16_t Height      = 64;
        static inline constexpr uint16_t Buffer_Size = Width * Height / 8;

        hal::i2c::I2CTransaction transaction()
        {
            setColumnAddressScope(0, Width - 1);
            setPageAddressScope(0, (Height / 8) - 1);
            return _transaction;
        }

        void init()
        {
            switch_oled(false);
            setMemoryAddressingMode(Horizontal);

            setColumnAddressScope(0, Width - 1);      // 0 到 127
            setPageAddressScope(0, (Height / 8) - 1); // 0 到 7 (针对 64 线屏幕)

            i2c_bus::template write_mem<hal::Mode::Normal>(
                Address,
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
         *
         * @param x 水平坐标，范围 [0, Width-1]
         * @param y 垂直坐标，范围 [0, Height-1]
         * @param color 颜色，true 为点亮，false 为熄灭
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

    private:
        uint8_t buffer[Buffer_Size];
        hal::i2c::I2CTransaction _transaction = {
            .type          = hal::i2c::TransactionType::WriteMem,
            .dev_addr      = Address,
            .mem_addr      = 0x40,
            .mem_addr_size = 1,
            .data_ptr      = buffer,
            .size          = Buffer_Size,
            .context       = this,
            .user_callback = [](hal::i2c::I2CTransaction *transaction) {},
        };

        /**
         * @brief 向 OLED 发送命令
         *
         * @param command 命令字节
         */
        void write_command(uint8_t command)
        {
            i2c_bus::template write_mem<hal::Mode::Normal>(Address, 0x00, 1, &command, 1);
        }

        /**
         * @brief 开关 OLED 显示
         *
         * @param state 显示状态，true 为开，false 为关
         */
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

        /**
         * @brief 是否翻转显示（即左右和上下翻转）
         *
         * @param enable 翻转使能，true 为翻转，false 为正常
         */
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

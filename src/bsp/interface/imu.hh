#pragma once

#include "interface.hh"

namespace bsp
{
    namespace imu
    {
        struct IMURawData {
            struct Accel {
                int16_t x, y, z;
            };

            struct Gyro {
                int16_t x, y, z;
            };

            Accel accel;
            Gyro gyro;
            int16_t temperature;
        };

        struct IMUData {
            struct Accel {
                float x, y, z;
            };

            struct Gyro {
                float x, y, z;
            };

            Accel accel;
            Gyro gyro;
            float temperature;
        };

        template <typename T>
        concept IMUConcept = requires(T imu) {
            // 初始化函数，返回 hal::Status
            { imu.init() } -> std::same_as<hal::Status>;

            // 提供给 I2C Handler 使用的 Transaction 结构
            { imu.transaction() } -> std::same_as<hal::i2c::I2CTransaction>;

            // 数据获取接口
            { imu.data() } -> std::same_as<IMUData>;
            { imu.raw_data() } -> std::same_as<IMURawData>;

            // 逻辑处理接口
            { imu.update() } -> std::same_as<void>;
            { imu.has_new_data() } -> std::same_as<bool>;

            // 零偏配置
            { imu.offset_data } -> std::same_as<IMUData &>;
        };
    } // namespace imu
} // namespace bsp

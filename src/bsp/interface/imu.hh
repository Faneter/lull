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
    } // namespace imu
} // namespace bsp

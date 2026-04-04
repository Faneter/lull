#pragma once

#include <i2c.hh>

#include "interface/imu.hh"

namespace bsp
{
    namespace imu
    {
        enum class MPU6050Reg : uint8_t {
            ADDRESS      = (0x68 << 1),
            WHO_AM_I     = 0x75,
            PWR_MGMT_1   = 0x6B,
            GYRO_CONFIG  = 0x1B,
            ACCEL_CONFIG = 0x1C,
            ACCEL_XOUT_H = 0x3B,
            GYRO_XOUT_H  = 0x43,
        };

        template <typename i2c_bus>
            requires hal::i2c::HasI2CHandleConcept<i2c_bus>
        class MPU6050
        {
        public:
            inline IMURawData RawData()
            {
                return raw_data;
            }

            inline IMUData Data()
            {
                return data;
            }

            inline hal::i2c::I2CTransaction i2c_transaction()
            {
                return transaction;
            }

            hal::Status init()
            {
                uint8_t check = 0;
                uint8_t data;
                i2c_bus::template read_mem<hal::Mode::Normal>(
                    static_cast<uint16_t>(MPU6050Reg::ADDRESS),
                    static_cast<uint16_t>(MPU6050Reg::WHO_AM_I),
                    1, &check, 1, 1000);

                if (check == 0x68) {
                    // 唤醒传感器：将 PWR_MGMT_1 的 SLEEP 位（第6位）清零
                    data = 0;
                    i2c_bus::template write_mem<hal::Mode::Normal>(
                        static_cast<uint16_t>(MPU6050Reg::ADDRESS),
                        static_cast<uint16_t>(MPU6050Reg::PWR_MGMT_1),
                        1, &data, 1, 1000);

                    // 配置陀螺仪量程 (默认 ±250°/s)
                    data = 0x00; // FS_SEL = 0
                    i2c_bus::template write_mem<hal::Mode::Normal>(
                        static_cast<uint16_t>(MPU6050Reg::ADDRESS),
                        static_cast<uint16_t>(MPU6050Reg::GYRO_CONFIG),
                        1, &data, 1, 1000);

                    // 配置加速度计量程 (默认 ±2g)
                    data = 0x00; // AFS_SEL = 0
                    i2c_bus::template write_mem<hal::Mode::Normal>(
                        static_cast<uint16_t>(MPU6050Reg::ADDRESS),
                        static_cast<uint16_t>(MPU6050Reg::ACCEL_CONFIG),
                        1, &data, 1, 1000);

                    return hal::Status::Ready;
                } else {
                    return hal::Status::Error;
                }
            }

            void update()
            {
                data.accel.x     = static_cast<float>(raw_data.accel.x) / 16384.0f - _offset_data.accel.x;
                data.accel.y     = static_cast<float>(raw_data.accel.y) / 16384.0f - _offset_data.accel.y;
                data.accel.z     = static_cast<float>(raw_data.accel.z) / 16384.0f - _offset_data.accel.z;
                data.gyro.x      = static_cast<float>(raw_data.gyro.x) / 131.0f - _offset_data.gyro.x;
                data.gyro.y      = static_cast<float>(raw_data.gyro.y) / 131.0f - _offset_data.gyro.y;
                data.gyro.z      = static_cast<float>(raw_data.gyro.z) / 131.0f - _offset_data.gyro.z;
                data.temperature = (static_cast<float>(raw_data.temperature) / 340.0) + 36.53;
            }

            void set_offset(IMUData offset)
            {
                _offset_data = offset;
            }

            bool has_new_data()
            {
                if (_new_data_flag) {
                    _new_data_flag = false;
                    return true;
                }
                return false;
            }

        private:
            IMURawData raw_data;
            IMUData data;
            IMUData _offset_data = {};
            uint8_t buffer[14];
            hal::i2c::I2CTransaction transaction = {
                .type          = hal::i2c::TransactionType::ReadMem,
                .dev_addr      = static_cast<uint16_t>(MPU6050Reg::ADDRESS),
                .mem_addr      = static_cast<uint16_t>(MPU6050Reg::ACCEL_XOUT_H),
                .mem_addr_size = 1,
                .data_ptr      = buffer,
                .size          = 14,
                .context       = this,
                .user_callback = transaction_callback,
            };
            volatile bool _new_data_flag = false;

            static void transaction_callback(hal::i2c::I2CTransaction *transaction)
            {
                MPU6050 *instance              = static_cast<MPU6050 *>(transaction->context);
                instance->raw_data.accel.x     = (int16_t)(transaction->data_ptr[0] << 8 | transaction->data_ptr[1]);
                instance->raw_data.accel.y     = (int16_t)(transaction->data_ptr[2] << 8 | transaction->data_ptr[3]);
                instance->raw_data.accel.z     = (int16_t)(transaction->data_ptr[4] << 8 | transaction->data_ptr[5]);
                instance->raw_data.temperature = (int16_t)(transaction->data_ptr[6] << 8 | transaction->data_ptr[7]);
                instance->raw_data.gyro.x      = (int16_t)(transaction->data_ptr[8] << 8 | transaction->data_ptr[9]);
                instance->raw_data.gyro.y      = (int16_t)(transaction->data_ptr[10] << 8 | transaction->data_ptr[11]);
                instance->raw_data.gyro.z      = (int16_t)(transaction->data_ptr[12] << 8 | transaction->data_ptr[13]);

                instance->_new_data_flag = true;
            }
        };
    } // namespace imu
} // namespace bsp
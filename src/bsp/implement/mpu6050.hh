#pragma once

#include "hal/i2c.hh"

#include "bsp/interface/imu.hh"

namespace bsp
{
    namespace imu
    {
        enum MPU6050Reg : uint8_t {
            ADDRESS      = (0x68 << 1),
            WHO_AM_I     = 0x75,
            PWR_MGMT_1   = 0x6B,
            GYRO_CONFIG  = 0x1B,
            ACCEL_CONFIG = 0x1C,
            ACCEL_XOUT_H = 0x3B,
            GYRO_XOUT_H  = 0x43,
        };

        class MPU6050
        {
        public:
            MPU6050(hal::i2c::AbstractHandler &handler) : i2cHandler(handler)
            {
            }

            IMUData data()
            {
                return _data;
            }

            IMURawData raw_data()
            {
                return _raw_data;
            }

            void set_offset(IMUData offset)
            {
                _offset_data = offset;
            }

            hal::Status init()
            {
                auto status = hal::Status::Error;

                uint8_t check = 0;
                uint8_t data;
                status = i2cHandler.execute(hal::i2c::I2CTransaction{
                    .type          = hal::i2c::TransactionType::ReadMem,
                    .dev_addr      = ADDRESS,
                    .mem_addr      = WHO_AM_I,
                    .mem_addr_size = 1,
                    .data_ptr      = &check,
                    .size          = 1,
                });

                if (check == 0x68) {
                    // 唤醒传感器：将 PWR_MGMT_1 的 SLEEP 位（第6位）清零
                    data   = 0;
                    status = i2cHandler.execute(hal::i2c::I2CTransaction{
                        .type          = hal::i2c::TransactionType::WriteMem,
                        .dev_addr      = ADDRESS,
                        .mem_addr      = PWR_MGMT_1,
                        .mem_addr_size = 1,
                        .data_ptr      = &data,
                        .size          = 1,
                    });

                    // 配置陀螺仪量程 (默认 ±250°/s)
                    data   = 0x00; // FS_SEL = 0
                    status = i2cHandler.execute(hal::i2c::I2CTransaction{
                        .type          = hal::i2c::TransactionType::WriteMem,
                        .dev_addr      = ADDRESS,
                        .mem_addr      = GYRO_CONFIG,
                        .mem_addr_size = 1,
                        .data_ptr      = &data,
                        .size          = 1,
                    });

                    // 配置加速度计量程 (默认 ±2g)
                    data   = 0x00; // AFS_SEL = 0
                    status = i2cHandler.execute(hal::i2c::I2CTransaction{
                        .type          = hal::i2c::TransactionType::WriteMem,
                        .dev_addr      = ADDRESS,
                        .mem_addr      = ACCEL_CONFIG,
                        .mem_addr_size = 1,
                        .data_ptr      = &data,
                        .size          = 1,
                    });
                }

                return status;
            }

            void update()
            {
                i2cHandler.async_execute(_transaction);
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
            hal::i2c::AbstractHandler &i2cHandler;

            uint8_t buffer[14];
            IMURawData _raw_data;
            IMUData _data;
            IMUData _offset_data                  = {};
            hal::i2c::I2CTransaction _transaction = {
                .type          = hal::i2c::TransactionType::ReadMem,
                .dev_addr      = ADDRESS,
                .mem_addr      = ACCEL_XOUT_H,
                .mem_addr_size = 1,
                .data_ptr      = buffer,
                .size          = 14,
                .context       = this,
                .user_callback = transaction_callback,
            };
            volatile bool _new_data_flag = false;

            void calculate_data()
            {
                _data.accel.x     = static_cast<float>(_raw_data.accel.x) / 16384.0f - _offset_data.accel.x;
                _data.accel.y     = static_cast<float>(_raw_data.accel.y) / 16384.0f - _offset_data.accel.y;
                _data.accel.z     = static_cast<float>(_raw_data.accel.z) / 16384.0f - _offset_data.accel.z;
                _data.gyro.x      = static_cast<float>(_raw_data.gyro.x) / 131.0f - _offset_data.gyro.x;
                _data.gyro.y      = static_cast<float>(_raw_data.gyro.y) / 131.0f - _offset_data.gyro.y;
                _data.gyro.z      = static_cast<float>(_raw_data.gyro.z) / 131.0f - _offset_data.gyro.z;
                _data.temperature = (static_cast<float>(_raw_data.temperature) / 340.0) + 36.53;
            }

            static void transaction_callback(hal::i2c::I2CTransaction *transaction)
            {
                MPU6050 *instance               = static_cast<MPU6050 *>(transaction->context);
                instance->_raw_data.accel.x     = (int16_t)(transaction->data_ptr[0] << 8 | transaction->data_ptr[1]);
                instance->_raw_data.accel.y     = (int16_t)(transaction->data_ptr[2] << 8 | transaction->data_ptr[3]);
                instance->_raw_data.accel.z     = (int16_t)(transaction->data_ptr[4] << 8 | transaction->data_ptr[5]);
                instance->_raw_data.temperature = (int16_t)(transaction->data_ptr[6] << 8 | transaction->data_ptr[7]);
                instance->_raw_data.gyro.x      = (int16_t)(transaction->data_ptr[8] << 8 | transaction->data_ptr[9]);
                instance->_raw_data.gyro.y      = (int16_t)(transaction->data_ptr[10] << 8 | transaction->data_ptr[11]);
                instance->_raw_data.gyro.z      = (int16_t)(transaction->data_ptr[12] << 8 | transaction->data_ptr[13]);

                instance->calculate_data();

                instance->_new_data_flag = true;
            }
        };
    } // namespace imu
} // namespace bsp
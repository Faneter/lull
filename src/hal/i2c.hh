#pragma once

#include "interface.hh"
#include <concepts>
#include <cstddef>
#include <utility>
#include <etl/queue.h>

#ifdef HAL_I2C_MODULE_ENABLED

namespace hal
{
    using I2CHandler = I2C_HandleTypeDef *;

    template <I2CHandler _handle>
    class I2C
    {
    public:
        static constexpr I2CHandler handle()
        { return _handle; }

        /**
         * @brief 发送数据到指定地址的从机
         */
        template <Mode mode>
        static inline Status transmit(uint16_t dev_addr, uint8_t *p_data, uint16_t size, uint32_t timeout = 50)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(HAL_I2C_Master_Transmit(_handle, dev_addr, p_data, size, timeout));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(HAL_I2C_Master_Transmit_IT(_handle, dev_addr, p_data, size));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(HAL_I2C_Master_Transmit_DMA(_handle, dev_addr, p_data, size));
        }

        /**
         * @brief 从指定地址的从机接收数据
         */
        template <Mode mode>
        static inline Status receive(uint16_t dev_addr, uint8_t *p_data, uint16_t size, uint32_t timeout = 50)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(HAL_I2C_Master_Receive(_handle, dev_addr, p_data, size, timeout));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(HAL_I2C_Master_Receive_IT(_handle, dev_addr, p_data, size));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(HAL_I2C_Master_Receive_DMA(_handle, dev_addr, p_data, size));
        }

        /**
         * @brief 写入从机寄存器 (Mem Write)
         */
        template <Mode mode>
        static inline Status write_mem(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout = 50)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(HAL_I2C_Mem_Write(_handle, dev_addr, mem_addr, mem_addr_size, p_data, size, timeout));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(HAL_I2C_Mem_Write_IT(_handle, dev_addr, mem_addr, mem_addr_size, p_data, size));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(HAL_I2C_Mem_Write_DMA(_handle, dev_addr, mem_addr, mem_addr_size, p_data, size));
        }

        /**
         * @brief 读取从机寄存器 (Mem Read)
         */
        template <Mode mode>
        static inline Status read_mem(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout = 50)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(HAL_I2C_Mem_Read(_handle, dev_addr, mem_addr, mem_addr_size, p_data, size, timeout));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(HAL_I2C_Mem_Read_IT(_handle, dev_addr, mem_addr, mem_addr_size, p_data, size));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(HAL_I2C_Mem_Read_DMA(_handle, dev_addr, mem_addr, mem_addr_size, p_data, size));
        }
    };

    namespace i2c
    {
        template <typename T>
        concept HasI2CHandleConcept = requires {
            { T::handle() } -> std::same_as<I2CHandler>;
        };

        enum class TransactionType {
            Transmit,
            Receive,
            WriteMem,
            ReadMem
        };

        struct I2CTransaction {
            TransactionType type;
            uint16_t dev_addr;
            uint16_t mem_addr; // 如果是 Mem 模式使用
            uint16_t mem_addr_size;
            uint8_t *data_ptr; // 数据指针
            uint16_t size;
            void *context;
            void (*user_callback)(I2CTransaction *); // 执行完后的回调
        };

        /**
         * @brief I2C 基础回调处理器
         * 用于处理传输完成后的逻辑（主要用于 DMA 和 IT 模式）
         */
        template <HasI2CHandleConcept i2c_bus, Mode mode>
        struct BaseHandler {
            etl::queue<I2CTransaction, 8> rx_queue;
            bool is_rx_busy = false;

            void (*_on_tx_complete)()              = nullptr;
            void (*_on_rx_complete)()              = nullptr;
            void (*_on_error)(uint32_t error_code) = nullptr;

            Status read(I2CTransaction transaction)
            {
                auto status = Status::Ready;

                if (transaction.type == TransactionType::ReadMem) {
                    status = i2c_bus::template read_mem<Mode::Normal>(transaction.dev_addr, transaction.mem_addr,
                                                                      transaction.mem_addr_size, transaction.data_ptr,
                                                                      transaction.size);

                    transaction.user_callback(&transaction);
                } else if (transaction.type == TransactionType::Receive) {
                    status = i2c_bus::template receive<Mode::Normal>(transaction.dev_addr, transaction.data_ptr, transaction.size);

                    transaction.user_callback(&transaction);
                }

                return status;
            }

            void async_read(I2CTransaction transaction)
            {
                rx_queue.push(transaction);
                rx_schedule_next(); // 尝试启动
            }

            Status rx_schedule_next()
            {
                if (is_rx_busy || rx_queue.empty())
                    return Status::Busy;

                auto status = Status::Ready;

                I2CTransaction &task = rx_queue.front();
                is_rx_busy           = true;

                if (task.type == TransactionType::ReadMem)
                    status = i2c_bus::template read_mem<mode>(task.dev_addr, task.mem_addr, task.mem_addr_size, task.data_ptr, task.size);
                else if (task.type == TransactionType::Receive)
                    status = i2c_bus::template receive<mode>(task.dev_addr, task.data_ptr, task.size);

                if (status != Status::Ready) {
                    is_rx_busy = false;
                    rx_queue.pop();
                    rx_schedule_next();
                }

                return status;
            }

            void callback_tx(I2CHandler hi2c)
            {
                if (hi2c == i2c_bus::handle() && _on_tx_complete) {
                    _on_tx_complete();
                }
            }

            void callback_rx(I2CHandler hi2c)
            {
                if (hi2c != i2c_bus::handle()) return;

                I2CTransaction &task = rx_queue.front(); // 任务完成
                rx_queue.pop();

                is_rx_busy = false;

                // 向你的 component 层分发数据
                if (task.user_callback)
                    task.user_callback(&task);

                if (_on_rx_complete)
                    _on_rx_complete();

                // 继续下一个任务
                rx_schedule_next();
            }

            void callback_error(I2CHandler hi2c)
            {
                if (hi2c == i2c_bus::handle() && _on_error) {
                    _on_error(HAL_I2C_GetError(hi2c));
                }
            }
        };
    } // namespace i2c

    namespace internal
    {
        // 静态分发适配器
        template <typename Handler>
        concept I2CCallableConcept = requires(Handler h, I2CHandler hi2c) {
            { h.callback_tx(hi2c) } -> std::same_as<void>;
            { h.callback_rx(hi2c) } -> std::same_as<void>;
        };

        template <typename Handler>
        concept I2CErrorCallableConcept = requires(Handler h, I2CHandler hi2c) {
            { h.callback_error(hi2c) } -> std::same_as<void>;
        };

        void execute_i2c_tx_callbacks(I2CHandler hi2c, auto &&...handlers)
        {
            (handlers.callback_tx(hi2c), ...);
        }

        void execute_i2c_rx_callbacks(I2CHandler hi2c, auto &&...handlers)
        {
            (handlers.callback_rx(hi2c), ...);
        }

        void execute_i2c_error_callbacks(I2CHandler hi2c, auto &&...handlers)
        {
            (handlers.callback_error(hi2c), ...);
        }
    } // namespace internal

// 回调生成宏
#define GENERATE_I2C_MASTER_TX_COMPLETE_CALLBACK(...)               \
    void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)      \
    {                                                               \
        hal::internal::execute_i2c_tx_callbacks(hi2c, __VA_ARGS__); \
    }

#define GENERATE_I2C_MASTER_RX_COMPLETE_CALLBACK(...)               \
    void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)      \
    {                                                               \
        hal::internal::execute_i2c_rx_callbacks(hi2c, __VA_ARGS__); \
    }

#define GENERATE_I2C_MEM_TX_COMPLETE_CALLBACK(...)                  \
    void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)         \
    {                                                               \
        hal::internal::execute_i2c_tx_callbacks(hi2c, __VA_ARGS__); \
    }

#define GENERATE_I2C_MEM_RX_COMPLETE_CALLBACK(...)                  \
    void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)         \
    {                                                               \
        hal::internal::execute_i2c_rx_callbacks(hi2c, __VA_ARGS__); \
    }

#define GENERATE_I2C_ERROR_CALLBACK(...)                               \
    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)                \
    {                                                                  \
        hal::internal::execute_i2c_error_callbacks(hi2c, __VA_ARGS__); \
    }
} // namespace hal

#endif
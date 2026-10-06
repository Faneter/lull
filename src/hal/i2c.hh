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
            void (*user_callback)(I2CTransaction *) = nullptr; // 执行完后的回调
        };

        struct AbstractHandler {
            virtual Status execute(I2CTransaction transaction)     = 0;
            virtual void async_execute(I2CTransaction transaction) = 0;
            virtual void callback_tx(I2CHandler hi2c)              = 0;
            virtual void callback_rx(I2CHandler hi2c)              = 0;
            virtual void callback_error(I2CHandler hi2c)           = 0;
        };

        /**
         * @brief I2C 基础回调处理器
         * 用于处理传输完成后的逻辑（主要用于 DMA 和 IT 模式）
         */
        template <HasI2CHandleConcept i2c_bus, Mode mode>
        struct BaseHandler : AbstractHandler {
            etl::queue<I2CTransaction, 8> queue;
            bool is_busy = false;

            void (*_on_tx_complete)()              = nullptr;
            void (*_on_rx_complete)()              = nullptr;
            void (*_on_error)(uint32_t error_code) = nullptr;

            Status execute(I2CTransaction transaction) override
            {
                auto status = Status::Ready;

                if (transaction.type == TransactionType::ReadMem) {
                    status = i2c_bus::template read_mem<Mode::Normal>(transaction.dev_addr, transaction.mem_addr,
                                                                      transaction.mem_addr_size, transaction.data_ptr,
                                                                      transaction.size);
                } else if (transaction.type == TransactionType::Receive) {
                    status = i2c_bus::template receive<Mode::Normal>(transaction.dev_addr, transaction.data_ptr, transaction.size);
                } else if (transaction.type == TransactionType::WriteMem) {
                    status = i2c_bus::template write_mem<Mode::Normal>(transaction.dev_addr, transaction.mem_addr,
                                                                       transaction.mem_addr_size, transaction.data_ptr,
                                                                       transaction.size);
                } else if (transaction.type == TransactionType::Transmit) {
                    status = i2c_bus::template transmit<Mode::Normal>(transaction.dev_addr, transaction.data_ptr, transaction.size);
                }

                if (transaction.user_callback)
                    transaction.user_callback(&transaction);

                return status;
            }

            void async_execute(I2CTransaction transaction) override
            {
                queue.push(transaction);
                schedule_next(); // 尝试启动
            }

            Status schedule_next()
            {
                if (is_busy || queue.empty())
                    return Status::Busy;

                auto status = Status::Ready;

                I2CTransaction &task = queue.front();
                is_busy              = true;

                if (task.type == TransactionType::ReadMem)
                    status = i2c_bus::template read_mem<mode>(task.dev_addr, task.mem_addr, task.mem_addr_size, task.data_ptr, task.size);
                else if (task.type == TransactionType::Receive)
                    status = i2c_bus::template receive<mode>(task.dev_addr, task.data_ptr, task.size);
                else if (task.type == TransactionType::WriteMem)
                    status = i2c_bus::template write_mem<mode>(task.dev_addr, task.mem_addr, task.mem_addr_size, task.data_ptr, task.size);
                else if (task.type == TransactionType::Transmit)
                    status = i2c_bus::template transmit<mode>(task.dev_addr, task.data_ptr, task.size);

                if (status != Status::Ready) {
                    // 注意：下发失败的帧会被直接丢弃，而且不会回调 user_callback，
                    //       调用方无从得知（上层会以为这一帧发出去了）。
                    //       需要感知失败请挂 _on_error，或在业务侧自己看 HAL 状态
                    //       （例如 HAL_I2C_GetError / hi2c.State）。
                    is_busy = false;
                    queue.pop();
                    schedule_next();
                }

                return status;
            }

            void callback_tx(I2CHandler hi2c) override
            {
                if (hi2c != i2c_bus::handle()) return;

                if (finish() && _on_tx_complete)
                    _on_tx_complete();

                // 继续下一个任务
                schedule_next();
            }

            void callback_rx(I2CHandler hi2c) override
            {
                if (hi2c != i2c_bus::handle()) return;

                if (finish() && _on_rx_complete)
                    _on_rx_complete();

                // 继续下一个任务
                schedule_next();
            }

            void callback_error(I2CHandler hi2c) override
            {
                if (hi2c == i2c_bus::handle() && _on_error) {
                    _on_error(HAL_I2C_GetError(hi2c));
                }
            }

        private:
            /**
             * @brief 结束队首任务：先拷贝再出队，避免 pop 之后继续引用已出队元素
             *
             * 原来的写法是取 queue.front() 的引用、pop() 之后再通过该引用调用
             * user_callback —— 那已经是一个生命期结束的对象。因为 I2CTransaction
             * 是平凡类型、出队并不会擦除内存，-O0 下侥幸能跑；优化一开就可能出错。
             * 这里改成先拷到局部变量再出队。
             *
             * 另外先判空再取 front()：HAL 在部分错误路径上会先给 ErrorCallback、
             * 再给完成回调，第二个回调不应该在一个已经空了的队列上取 front()。
             *
             * @return 队列非空（即确实完成了一个任务）返回 true
             */
            bool finish()
            {
                if (queue.empty()) return false;

                I2CTransaction task = queue.front();
                queue.pop();

                is_busy = false;

                // 向你的 component 层分发数据
                if (task.user_callback)
                    task.user_callback(&task);

                return true;
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
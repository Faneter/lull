#pragma once

#include "interface.hh"

#include <concepts>
#include <cstddef>
#include <utility>
#include <etl/queue.h>

#ifdef HAL_SPI_MODULE_ENABLED

namespace hal
{
    using SPIHandler = SPI_HandleTypeDef *;

    /**
     * @brief SPI 总线静态封装
     *
     * 只负责把「句柄 + 模式」翻译成对应的 HAL 调用，不持有任何运行时状态，
     * 所有成员均为静态函数，因此可以直接当类型使用（零开销）。
     *
     * @tparam _handle SPI 句柄，例如 &hspi1
     */
    template <SPIHandler _handle>
    class SPI
    {
    public:
        static constexpr SPIHandler handle()
        { return _handle; }

        /**
         * @brief 发送数据（只写，MISO 上的数据被丢弃）
         *
         * @tparam mode    传输模式：Normal (阻塞)、It (中断)、Dma (DMA)
         * @param p_data   待发送数据指针
         * @param size     待发送字节数
         * @param timeout  超时时间 (毫秒)，仅在 mode == Mode::Normal 时生效，默认 50ms
         * @return Status  HAL 状态：Ready(OK) / Error / Busy / Timeout
         */
        template <Mode mode>
        static inline Status transmit(const uint8_t *p_data, uint16_t size, uint32_t timeout = 50)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(HAL_SPI_Transmit(_handle, p_data, size, timeout));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(HAL_SPI_Transmit_IT(_handle, p_data, size));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(HAL_SPI_Transmit_DMA(_handle, p_data, size));
        }

        /**
         * @brief 接收数据（只读）
         *
         * @tparam mode    传输模式：Normal (阻塞)、It (中断)、Dma (DMA)
         * @param p_data   接收缓冲区指针
         * @param size     待接收字节数
         * @param timeout  超时时间 (毫秒)，仅在 mode == Mode::Normal 时生效，默认 50ms
         * @return Status  HAL 状态
         */
        template <Mode mode>
        static inline Status receive(uint8_t *p_data, uint16_t size, uint32_t timeout = 50)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(HAL_SPI_Receive(_handle, p_data, size, timeout));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(HAL_SPI_Receive_IT(_handle, p_data, size));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(HAL_SPI_Receive_DMA(_handle, p_data, size));
        }

        /**
         * @brief 全双工收发：发送 p_tx_data 的同时把 MISO 上的数据写入 p_rx_data
         *
         * 这是 SPI 最常用的「写命令 + 读回显」方式，W25QXX 之类的 Flash 全靠它。
         *
         * @tparam mode      传输模式：Normal (阻塞)、It (中断)、Dma (DMA)
         * @param p_tx_data  发送缓冲区指针
         * @param p_rx_data  接收缓冲区指针
         * @param size       传输字节数
         * @param timeout    超时时间 (毫秒)，仅在 mode == Mode::Normal 时生效，默认 50ms
         * @return Status    HAL 状态
         */
        template <Mode mode>
        static inline Status transmit_receive(const uint8_t *p_tx_data, uint8_t *p_rx_data,
                                              uint16_t size, uint32_t timeout = 50)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(
                    HAL_SPI_TransmitReceive(_handle, p_tx_data, p_rx_data, size, timeout));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(
                    HAL_SPI_TransmitReceive_IT(_handle, p_tx_data, p_rx_data, size));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(
                    HAL_SPI_TransmitReceive_DMA(_handle, p_tx_data, p_rx_data, size));
        }
    };

    namespace spi
    {
        template <typename T>
        concept HasSPIHandleConcept = requires {
            { T::handle() } -> std::same_as<SPIHandler>;
        };

        enum class TransactionType {
            Transmit,
            Receive,
            TransmitReceive
        };

        struct SPITransaction {
            TransactionType type;
            const uint8_t *tx_data_ptr; // Tx / TxRx 使用，可为 nullptr
            uint8_t *rx_data_ptr;       // Rx / TxRx 使用，可为 nullptr
            uint16_t size;
            void *context;
            void (*user_callback)(SPITransaction *) = nullptr; // 执行完后的回调
        };

        struct AbstractHandler {
            virtual Status execute(SPITransaction transaction)     = 0;
            virtual void async_execute(SPITransaction transaction) = 0;
            virtual void callback_tx(SPIHandler hspi)              = 0;
            virtual void callback_rx(SPIHandler hspi)              = 0;
            virtual void callback_txrx(SPIHandler hspi)            = 0;
            virtual void callback_error(SPIHandler hspi)           = 0;
        };

        /**
         * @brief SPI 基础回调处理器
         * 用于处理传输完成后的逻辑（主要用于 DMA 和 IT 模式），
         * 内部维护一个先进先出的任务队列，保证同一时刻只有一个传输在跑。
         */
        template <HasSPIHandleConcept spi_bus, Mode mode>
        struct BaseHandler : AbstractHandler {
            etl::queue<SPITransaction, 8> queue;
            bool is_busy = false;

            void (*_on_tx_complete)()              = nullptr;
            void (*_on_rx_complete)()              = nullptr;
            void (*_on_txrx_complete)()            = nullptr;
            void (*_on_error)(uint32_t error_code) = nullptr;

            /**
             * @brief 阻塞式执行一个传输任务（配合 Mode::Normal 使用）
             */
            Status execute(SPITransaction transaction) override
            {
                auto status = Status::Ready;

                if (transaction.type == TransactionType::Transmit)
                    status = spi_bus::template transmit<Mode::Normal>(transaction.tx_data_ptr, transaction.size);
                else if (transaction.type == TransactionType::Receive)
                    status = spi_bus::template receive<Mode::Normal>(transaction.rx_data_ptr, transaction.size);
                else if (transaction.type == TransactionType::TransmitReceive)
                    status = spi_bus::template transmit_receive<Mode::Normal>(
                        transaction.tx_data_ptr, transaction.rx_data_ptr, transaction.size);

                if (transaction.user_callback)
                    transaction.user_callback(&transaction);

                return status;
            }

            /**
             * @brief 把任务排入队列并尝试启动（异步，配合 It / Dma 模式使用）
             */
            void async_execute(SPITransaction transaction) override
            {
                queue.push(transaction);
                schedule_next(); // 尝试启动
            }

            /**
             * @brief 若总线空闲则取出队首任务下发；下发失败的任务直接丢弃并继续下一个
             */
            Status schedule_next()
            {
                if (is_busy || queue.empty())
                    return Status::Busy;

                auto status = Status::Ready;

                SPITransaction &task = queue.front();
                is_busy              = true;

                if (task.type == TransactionType::Transmit)
                    status = spi_bus::template transmit<mode>(task.tx_data_ptr, task.size);
                else if (task.type == TransactionType::Receive)
                    status = spi_bus::template receive<mode>(task.rx_data_ptr, task.size);
                else if (task.type == TransactionType::TransmitReceive)
                    status = spi_bus::template transmit_receive<mode>(
                        task.tx_data_ptr, task.rx_data_ptr, task.size);

                if (status != Status::Ready) {
                    is_busy = false;
                    queue.pop();
                    schedule_next();
                }

                return status;
            }

            void callback_tx(SPIHandler hspi) override
            {
                if (hspi != spi_bus::handle()) return;

                finish();

                if (_on_tx_complete)
                    _on_tx_complete();

                // 继续下一个任务
                schedule_next();
            }

            void callback_rx(SPIHandler hspi) override
            {
                if (hspi != spi_bus::handle()) return;

                finish();

                if (_on_rx_complete)
                    _on_rx_complete();

                // 继续下一个任务
                schedule_next();
            }

            void callback_txrx(SPIHandler hspi) override
            {
                if (hspi != spi_bus::handle()) return;

                finish();

                if (_on_txrx_complete)
                    _on_txrx_complete();

                // 继续下一个任务
                schedule_next();
            }

            /**
             * @brief 错误回调
             *
             * 注意：这里只上报错误码，不主动出队。
             * HAL 在部分错误路径上会先调 ErrorCallback 再调 TxCplt/RxCplt，
             * 若在此处 pop 会导致完成回调二次出队、队列错位。
             * 需要恢复总线时请在 _on_error 里处理（DeInit / 重新 Init）。
             */
            void callback_error(SPIHandler hspi) override
            {
                if (hspi == spi_bus::handle() && _on_error) {
                    _on_error(HAL_SPI_GetError(hspi));
                }
            }

        private:
            /**
             * @brief 结束队首任务：先拷贝再出队，避免 pop 之后继续引用已出队元素
             */
            void finish()
            {
                if (queue.empty()) return;

                SPITransaction task = queue.front();
                queue.pop();

                is_busy = false;

                if (task.user_callback)
                    task.user_callback(&task);
            }
        };
    } // namespace spi

    namespace internal
    {
        // 静态分发适配器
        template <typename Handler>
        concept SPICallableConcept = requires(Handler h, SPIHandler hspi) {
            { h.callback_tx(hspi) } -> std::same_as<void>;
            { h.callback_rx(hspi) } -> std::same_as<void>;
            { h.callback_txrx(hspi) } -> std::same_as<void>;
        };

        template <typename Handler>
        concept SPIErrorCallableConcept = requires(Handler h, SPIHandler hspi) {
            { h.callback_error(hspi) } -> std::same_as<void>;
        };

        void execute_spi_tx_callbacks(SPIHandler hspi, auto &&...handlers)
        {
            (handlers.callback_tx(hspi), ...);
        }

        void execute_spi_rx_callbacks(SPIHandler hspi, auto &&...handlers)
        {
            (handlers.callback_rx(hspi), ...);
        }

        void execute_spi_txrx_callbacks(SPIHandler hspi, auto &&...handlers)
        {
            (handlers.callback_txrx(hspi), ...);
        }

        void execute_spi_error_callbacks(SPIHandler hspi, auto &&...handlers)
        {
            (handlers.callback_error(hspi), ...);
        }
    } // namespace internal

    // 回调生成宏（在 .cc 中展开一次，用于把 HAL 的弱回调转发给 handler）
#define GENERATE_SPI_TX_COMPLETE_CALLBACK(...)                      \
    void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)            \
    {                                                               \
        hal::internal::execute_spi_tx_callbacks(hspi, __VA_ARGS__); \
    }

#define GENERATE_SPI_RX_COMPLETE_CALLBACK(...)                      \
    void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)            \
    {                                                               \
        hal::internal::execute_spi_rx_callbacks(hspi, __VA_ARGS__); \
    }

#define GENERATE_SPI_TX_RX_COMPLETE_CALLBACK(...)                     \
    void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)            \
    {                                                                 \
        hal::internal::execute_spi_txrx_callbacks(hspi, __VA_ARGS__); \
    }

#define GENERATE_SPI_ERROR_CALLBACK(...)                               \
    void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)                \
    {                                                                  \
        hal::internal::execute_spi_error_callbacks(hspi, __VA_ARGS__); \
    }
} // namespace hal

#endif

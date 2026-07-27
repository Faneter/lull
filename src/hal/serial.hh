#pragma once

#include "interface.hh"
#include <concepts>
#include <cstddef>
#include <utility>

#ifdef HAL_UART_MODULE_ENABLED

namespace hal
{
    using UartHandler = UART_HandleTypeDef *;

    template <UartHandler _handle>
    class Serial
    {
    public:
        static constexpr UartHandler handle()
        { return _handle; }

        // Send
        /**
         * @brief 通过指定模式发送指定长度的字节缓冲区数据（底层基础接口）
         *
         * @tparam mode    发送模式：支持 Normal (阻塞)、It (中断)、Dma (DMA)
         * @param bytes    指向待发送字节缓冲区的指针
         * @param size     待发送的字节数量
         * @param timeout  发送超时时间 (毫秒)，仅在 mode == Mode::Normal 时生效，默认 50ms
         * @return Status  发送状态：返回 HAL 状态枚举 (HAL_OK, HAL_BUSY, HAL_TIMEOUT, HAL_ERROR)
         */
        template <Mode mode>
        static inline Status send(const uint8_t bytes[], uint16_t size,
                                  uint32_t timeout = 50)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(
                    HAL_UART_Transmit(_handle, bytes, size, timeout));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(
                    HAL_UART_Transmit_IT(_handle, bytes, size));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(
                    HAL_UART_Transmit_DMA(_handle, bytes, size));
        }
        /**
         * @brief 发送定长 C 风格原生字节数组（自动在编译期推导数组长度）
         *
         * @details 利用 C++ 模板数组引用特性，直接传入数组即可，无需显式指定 size，
         *          规避了指针退化带来的越界风险。
         *
         * @tparam mode    发送模式：支持 Normal (阻塞)、It (中断)、Dma (DMA)
         * @tparam length  编译期自动推导出的数组长度 (无需手动传递)
         * @param bytes    对定长字节数组的引用
         * @param timeout  发送超时时间 (毫秒)，仅在阻塞模式下生效，默认 50ms
         * @return Status  发送状态
         */
        template <Mode mode, size_t length>
        static inline Status send(const uint8_t (&bytes)[length],
                                  uint32_t timeout = 50)
        {
            return send<mode>(bytes, length, timeout);
        }
        /**
         * @brief 直接向串口发送字符串字面量或字符数组（自动推导长度并剔除结尾 '\0'）
         *
         * @details 该函数重载用于简化字符串发送。会自动将 `const char[]` 转换为 `const uint8_t*`，
         *          并在发送时自动扣除 1 字节的空终止符 `\0`。
         *
         * @tparam mode    发送模式：支持 Normal (阻塞)、It (中断)、Dma (DMA)
         * @tparam length  编译期自动推导出的字符串数组长度 (包含 '\0')
         * @param str      对字符数组/字符串字面量的引用
         * @param timeout  发送超时时间 (毫秒)，仅在阻塞模式下生效，默认 50ms
         * @return Status  发送状态
         */
        template <Mode mode, size_t length>
        static inline Status send(const char (&str)[length], uint32_t timeout = 50)
        {
            // 将 char 数组引用强转为 uint8_t 指针，length - 1 可用于剔除结尾的 '\0'（根据业务需求决定）
            return send<mode>(reinterpret_cast<const uint8_t *>(str), length - 1, timeout);
        }

        // Receive
        template <Mode mode>
        static inline Status receive(uint8_t bytes[], uint16_t size,
                                     uint32_t timeout = 50)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(
                    HAL_UART_Receive(_handle, bytes, size, timeout));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(HAL_UART_Receive_IT(_handle, bytes, size));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(HAL_UART_Receive_DMA(_handle, bytes, size));
        }
        template <Mode mode, size_t length>
        static inline Status receive(uint8_t (&bytes)[length],
                                     uint32_t timeout = 50)
        {
            return receive<mode>(bytes, length, timeout);
        }

        // Receive with idle
        template <Mode mode>
        static inline Status receive_idle(uint8_t bytes[], uint16_t size)
        {
            if constexpr (mode == Mode::It)
                return static_cast<Status>(
                    HAL_UARTEx_ReceiveToIdle_IT(_handle, bytes, size));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(
                    HAL_UARTEx_ReceiveToIdle_DMA(_handle, bytes, size));
        }
        template <Mode mode, size_t length>
        static inline Status receive_idle(uint8_t (&bytes)[length])
        {
            return receive_idle<mode>(bytes, length);
        }

        static inline Status receive_idle(uint8_t bytes[], uint16_t &receive_size,
                                          uint16_t max_size, uint32_t timeout = 50)
        {
            return static_cast<Status>(HAL_UARTEx_ReceiveToIdle(
                _handle, bytes, max_size, &receive_size, timeout));
        }
        template <size_t length>
        static inline Status receive_idle(uint8_t (&bytes)[length],
                                          uint16_t &receive_size,
                                          uint32_t timeout = 50)
        {
            return receive_idle(bytes, receive_size, length, timeout);
        }

        template <Mode mode>
        static inline void hello_world()
        {
            send<mode>("Hello World!\n");
        }
    };

    namespace serial
    {
        template <typename T>
        concept HasSerialHandleConcept = requires {
            { T::handle() } -> std::same_as<UartHandler>;
        };

        template <HasSerialHandleConcept serial, Mode mode, size_t BufferSize = 128>
        struct BaseHandler {
            uint8_t buffer[BufferSize];

            void (*_on_data_ready)(uint8_t *bytes, uint16_t size) = nullptr;

            void start()
            { serial::template receive_idle<mode>(buffer); }

            void callback(UartHandler huart, uint16_t size)
            {
                if (huart == serial::handle()) {
                    if (size > 0 && _on_data_ready)
                        _on_data_ready(buffer, size);
                    start();
                }
            }
        };
    } // namespace serial

    namespace internal
    {
        template <typename Handler>
        concept SerialStaticCallableConcept =
            requires(Handler, UartHandler huart, uint16_t size) {
                { Handler::callback(huart, size) } -> std::same_as<void>;
            };
        template <typename Handler>
        concept SerialNormalCallableConcept =
            requires(Handler handler, UartHandler huart, uint16_t size) {
                { handler.callback(huart, size) } -> std::same_as<void>;
            };
        template <typename Handler>
        concept SerialDirectCallableConcept =
            requires(Handler handler, UartHandler huart, uint16_t size) {
                { handler(huart, size) } -> std::same_as<void>;
            };
        template <typename Handlers>
        concept SerialDirectCallableNoArgsConcept = requires(Handlers handler) {
            { handler() } -> std::same_as<void>;
        };

        void call_serial_callback(UartHandler huart, uint16_t size,
                                  SerialStaticCallableConcept auto const &handler)
        {
            handler.callback(huart, size);
        }
        void call_serial_callback(UartHandler huart, uint16_t size,
                                  SerialNormalCallableConcept auto &&handlers)
        {
            handlers.callback(huart, size);
        }
        void call_serial_callback(UartHandler huart, uint16_t size,
                                  SerialDirectCallableConcept auto &&handlers)
        {
            handlers(huart, size);
        }
        void call_serial_callback(UartHandler, uint16_t,
                                  SerialDirectCallableNoArgsConcept auto &&handlers)
        {
            handlers();
        }

        void execute_serial_callbacks(UartHandler huart, uint16_t size,
                                      auto &&...handlers)
        {
            (call_serial_callback(huart, size,
                                  std::forward<decltype(handlers)>(handlers)),
             ...);
        }
    } // namespace internal

#define GENERATE_UART_RX_EVENT_CALLBACK(...)                               \
    void HAL_UARTEx_RxEventCallback(hal::UartHandler huart, uint16_t size) \
    {                                                                      \
        hal::internal::execute_serial_callbacks(huart, size, __VA_ARGS__); \
    }
#define GENERATE_UART_RX_COMPLETE_CALLBACK(...)                         \
    void HAL_UARTEx_RxCpltCallback(hal::UartHandler huart)              \
    {                                                                   \
        hal::internal::execute_serial_callbacks(huart, 0, __VA_ARGS__); \
    }

} // namespace hal

#endif
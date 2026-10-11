#pragma once

#include "interface.hh"
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#include <etl/format_spec.h>
#include <etl/string.h>
#include <etl/to_string.h>

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

        // Print
        /**
         * @brief 把若干参数依次格式化后一次性发送（ETL 版 "printf"）
         *
         * 不使用变参、不依赖 printf/vsnprintf：每个参数都交给 etl::to_string 处理。
         * 整数格式化（进制 / 宽度 / 填充 / 大小写）的代码在用到 etl::string 的工程里
         * 本来就会链接，所以额外 flash 开销接近 0；而且参数类型是编译期检查的，
         * 不存在 printf 那种 "%d" 配 float 的错配。
         *
         * 传一个 etl::format_spec 可以改变它**后面**那些值的格式（类似 iostream 的操纵器）：
         * @code
           // 十进制：ID: 239
           hal::Serial<&huart1>::print("ID: ", 239, "\n");

           // 两位大写十六进制：ID: EF
           hal::Serial<&huart1>::print("ID: ",
                                       etl::format_spec().hex().width(2).fill('0').upper_case(true),
                                       0xEF, "\n");
         * @endcode
         *
         * @tparam mode     发送模式，默认 Normal（阻塞、整行一次发完，不会被 HAL_BUSY 丢掉）。
         *                  用 It / Dma 时请自行保证不重入，否则上一笔没发完会静默丢数据
         * @tparam Capacity 行缓冲区容量（栈上分配 Capacity 字节）；超出部分截断
         * @param args      任意个可格式化参数：整数 / 浮点 / bool / C 字符串 /
         *                  etl 字符串 / etl::string_view / etl::format_spec
         * @return Status   发送状态；没有任何内容时不发送，直接返回 Ready
         *
         * @note 单个参数格式化后的长度上限为 32 字符（FieldCapacity），超出会截断
         */
        template <Mode mode = Mode::Normal, size_t Capacity = 128, typename... Args>
        static inline Status print(Args &&...args)
        {
            etl::string<Capacity> line;
            etl::format_spec format;

            (append_argument(line, format, std::forward<Args>(args)), ...);

            if (line.empty())
                return Status::Ready;

            return send<mode>(reinterpret_cast<const uint8_t *>(line.c_str()),
                              static_cast<uint16_t>(line.size()), 100);
        }

    private:
        /** 单个参数格式化后的长度上限：64 位整数（20 位）加符号和填充都够 */
        static constexpr size_t FieldCapacity = 32;

        /**
         * @brief 把一个参数追加到行缓冲区
         *
         * 分四种情况：format_spec 当"操纵器"、char 按字符、C 字符串按文本，
         * 其余（整数 / 浮点 / bool / etl 字符串 / string_view）交给 etl::to_string。
         */
        template <size_t Capacity, typename T>
        static inline void append_argument(etl::string<Capacity> &line,
                                           etl::format_spec &format, T &&value)
        {
            using value_type = std::decay_t<T>;

            if constexpr (std::same_as<value_type, etl::format_spec>) {
                format = value;
            } else if constexpr (std::same_as<value_type, char>) {
                if (!line.full())
                    line.push_back(value);
            } else if constexpr (std::convertible_to<value_type, const char *>) {
                const char *text = value;
                while (*text != '\0' && !line.full())
                    line.push_back(*text++);
            } else {
                // 先格式化到定长小缓冲，再逐字符拷进行缓冲：
                // 行缓冲永远不会被越界写（ETL 的断言在 Release 下是关掉的）
                etl::string<FieldCapacity> field;
                etl::to_string(value, field, format, false);

                for (size_t i = 0; i < field.size() && !line.full(); ++i)
                    line.push_back(field[i]);
            }
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
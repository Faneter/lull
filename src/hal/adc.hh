#pragma once

#include "interface.hh"

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <utility>

#ifdef HAL_ADC_MODULE_ENABLED

namespace hal
{
    using ADCHandler = ADC_HandleTypeDef *;

    /**
     * @brief ADC 静态封装
     *
     * 只把「句柄 + 模式」翻译成对应的 HAL 调用，不持有任何运行时状态。
     * 单通道 + 单次转换（CubeMX 默认配置）下最常用的就是
     * read<Mode::Normal>(value)，内部完成 start -> poll -> get -> stop。
     *
     * @tparam _handle ADC 句柄，例如 &hadc1
     */
    template <ADCHandler _handle>
    class ADC
    {
    public:
        static constexpr ADCHandler handle()
        { return _handle; }

        /** 12 位 ADC 的满量程原始值 */
        static inline constexpr uint16_t FullScale = 4095;

        /**
         * @brief 配置规则通道（通道号 / 采样时间 / 排序）
         *
         * 采样时间必须与信号源阻抗匹配：源阻抗越大，需要的采样周期越多。
         * 例如 10K 分压的源阻抗约 5K，至少要用 ADC_SAMPLETIME_55CYCLES_5，
         * 用默认的 1.5 周期（125ns @12MHz）采样保持电容充不满，读数会偏低且跳动。
         */
        static inline Status configure_channel(uint32_t channel,
                                               uint32_t sampling_time,
                                               uint32_t rank = ADC_REGULAR_RANK_1)
        {
            ADC_ChannelConfTypeDef config = {};
            config.Channel                = channel;
            config.Rank                   = rank;
            config.SamplingTime           = sampling_time;

            return static_cast<Status>(HAL_ADC_ConfigChannel(_handle, &config));
        }

        /**
         * @brief 启动转换
         *
         * @tparam mode   Normal (阻塞轮询)、It (中断)、Dma (DMA)
         * @param buffer  仅 Dma 模式使用的结果缓冲区
         * @param length  仅 Dma 模式使用的通道数
         */
        template <Mode mode>
        static inline Status start(uint32_t *buffer = nullptr, uint32_t length = 0)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(HAL_ADC_Start(_handle));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(HAL_ADC_Start_IT(_handle));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(HAL_ADC_Start_DMA(_handle, buffer, length));
        }

        template <Mode mode>
        static inline Status stop()
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(HAL_ADC_Stop(_handle));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(HAL_ADC_Stop_IT(_handle));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(HAL_ADC_Stop_DMA(_handle));
        }

        /** 阻塞等待本次转换结束 */
        static inline Status poll(uint32_t timeout = 10)
        { return static_cast<Status>(HAL_ADC_PollForConversion(_handle, timeout)); }

        /** 读取最近一次转换结果（原始值） */
        static inline uint16_t value()
        { return static_cast<uint16_t>(HAL_ADC_GetValue(_handle)); }

        /** 读取错误标志 */
        static inline uint32_t error()
        { return HAL_ADC_GetError(_handle); }

        /**
         * @brief 单次采样：start -> poll -> value -> stop
         *
         * @param result  原始值输出
         * @param timeout 等待转换完成的超时 (毫秒)
         * @return Status 成功返回 Ready，失败返回 HAL 的错误码映射
         */
        template <Mode mode>
        static inline Status read(uint16_t &result, uint32_t timeout = 10)
        {
            auto status = start<mode>();
            if (status != Status::Ready) return status;

            status = poll(timeout);
            stop<mode>();

            if (status != Status::Ready) return status;

            result = value();

            return Status::Ready;
        }

        /**
         * @brief 连续采样 times 次并取平均，抑制随机噪声
         */
        template <Mode mode>
        static inline Status read_average(uint16_t &result, uint8_t times, uint32_t timeout = 10)
        {
            if (times == 0) return Status::Error;

            uint32_t sum = 0;

            for (uint8_t i = 0; i < times; ++i) {
                uint16_t sample   = 0;
                const auto status = read<mode>(sample, timeout);
                if (status != Status::Ready) return status;

                sum += sample;
            }

            result = static_cast<uint16_t>(sum / times);

            return Status::Ready;
        }

        /** 原始值 -> 比例 (0.0 ~ 1.0) */
        static inline float to_ratio(uint16_t raw)
        { return static_cast<float>(raw) / static_cast<float>(FullScale); }

        /** 原始值 -> 电压，reference 为 VDDA（通常 3.3V） */
        static inline float to_voltage(uint16_t raw, float reference = 3.3f)
        { return to_ratio(raw) * reference; }
    };

    namespace adc
    {
        template <typename T>
        concept HasADCHandleConcept = requires {
            { T::handle() } -> std::same_as<ADCHandler>;
        };

        /**
         * @brief ADC 中断 / DMA 模式的完成处理器
         *
         * ADC 的转换没有数据指针，结果直接从寄存器取，所以这里不需要任务队列，
         * 只保留最近一次转换结果和一个「有新数据」标志。
         */
        template <HasADCHandleConcept adc_bus, Mode mode = Mode::It>
        struct BaseHandler {
            uint16_t value = 0;

            void (*_on_conversion_complete)(uint16_t value) = nullptr;
            void (*_on_error)(uint32_t error_code)          = nullptr;

            /**
             * @brief 启动一次转换
             *
             * 连续采样需要在 CubeMX 里把 ContinuousConvMode 打开；
             * 单次模式则每次取数前都要调用一次。
             */
            void start(uint32_t *buffer = nullptr, uint32_t length = 0)
            {
                new_data = false;
                adc_bus::template start<mode>(buffer, length);
            }

            void stop()
            { adc_bus::template stop<mode>(); }

            bool has_new_data()
            {
                if (new_data) {
                    new_data = false;
                    return true;
                }
                return false;
            }

            uint16_t data() const
            { return value; }

            void callback(ADCHandler hadc)
            {
                if (hadc != adc_bus::handle()) return;

                value    = adc_bus::value();
                new_data = true;

                if (_on_conversion_complete) _on_conversion_complete(value);
            }

            void callback_error(ADCHandler hadc)
            {
                if (hadc == adc_bus::handle() && _on_error) {
                    _on_error(HAL_ADC_GetError(hadc));
                }
            }

        private:
            volatile bool new_data = false;
        };
    } // namespace adc

    namespace internal
    {
        // 静态分发适配器
        template <typename Handler>
        concept ADCCallableConcept = requires(Handler h, ADCHandler hadc) {
            { h.callback(hadc) } -> std::same_as<void>;
        };

        template <typename Handler>
        concept ADCErrorCallableConcept = requires(Handler h, ADCHandler hadc) {
            { h.callback_error(hadc) } -> std::same_as<void>;
        };

        void execute_adc_conv_callbacks(ADCHandler hadc, auto &&...handlers)
        {
            (handlers.callback(hadc), ...);
        }

        void execute_adc_error_callbacks(ADCHandler hadc, auto &&...handlers)
        {
            (handlers.callback_error(hadc), ...);
        }
    } // namespace internal

    // 回调生成宏（在 .cc 中展开一次，用于把 HAL 的弱回调转发给 handler）
#define GENERATE_ADC_CONV_COMPLETE_CALLBACK(...)                      \
    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)            \
    {                                                                 \
        hal::internal::execute_adc_conv_callbacks(hadc, __VA_ARGS__); \
    }

#define GENERATE_ADC_ERROR_CALLBACK(...)                               \
    void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)                \
    {                                                                  \
        hal::internal::execute_adc_error_callbacks(hadc, __VA_ARGS__); \
    }
} // namespace hal

#endif

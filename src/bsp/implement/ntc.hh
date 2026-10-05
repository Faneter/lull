#pragma once

#include "hal/adc.hh"

#include <cmath>
#include <cstdint>

namespace bsp
{
    namespace temperature
    {
        /**
         * @brief NTC 热敏电阻的电气与转换参数
         *
         * 注意：本结构体要作为模板非类型参数使用，因此必须是 structural type
         * —— 所有非静态数据成员都是 public、没有 mutable 成员、成员本身也都是
         * structural type。改动时请保持这个性质（不要加私有成员/构造函数）。
         *
         * 用指派初始化器只改其中几项即可（必须按下面的声明顺序书写）：
         *
         * @code
           constexpr bsp::temperature::Config config = {
               .sampling_time      = ADC_SAMPLETIME_55CYCLES_5,
               .divider_resistance = 4700.0f,
               .beta               = 3435.0f,
               .ntc_to_ground      = false,
           };
           @endcode
         */
        struct Config {
            // ---- ADC 侧 ----
            uint32_t channel       = ADC_CHANNEL_0;              // 规则通道
            uint32_t sampling_time = ADC_SAMPLETIME_239CYCLES_5; // 见 NTC::init()
            uint8_t samples        = 16;                         // 每次读数取平均的次数

            // ---- 传感器侧 ----
            float reference_voltage  = 3.3f;     // 参考电压（VDDA）
            uint16_t full_scale      = 4095;     // 满量程原始值（12 位）
            float divider_resistance = 10000.0f; // 与 NTC 串联的固定电阻
            float nominal_resistance = 10000.0f; // 标称温度下的 NTC 阻值
            float nominal_celsius    = 25.0f;    // 标称温度
            float beta               = 3950.0f;  // B 值
            bool ntc_to_ground       = true;     // true: NTC 接地、固定电阻上拉到 VCC

            // ---- 有效区间（超出即认为传感器异常，供 valid() 判断）----
            float minimum_celsius = -55.0f;
            float maximum_celsius = 150.0f;
        };

        /**
         * @brief NTC 热敏电阻驱动（电阻分压 + Beta 参数方程）
         *
         * 换算链条：原始值 -> 分压电压 -> NTC 阻值 -> 摄氏温度
         *
         * 配置全部在编译期确定并内联进代码，实例本身只保存最近一次采样的结果，
         * 因此没有任何运行时的配置开销。
         *
         * 两种分压拓扑都支持，由 Config::ntc_to_ground 选择：
         *   - true （默认）：固定电阻上拉到 VCC、NTC 接地
         *                    温度升高 -> 阻值下降 -> 电压升高
         *   - false        ：NTC 接 VCC、固定电阻下拉到 GND
         *                    温度升高 -> 阻值下降 -> 电压降低
         *
         * 若实测「用手加热温度反而下降」，把 ntc_to_ground 取反即可。
         *
         * @tparam adc_bus ADC 总线类型，例如 hal::ADC<&hadc1>
         * @tparam _config 传感器与 ADC 的配置，见 Config
         * @tparam _mode   采样模式，默认阻塞（Mode::Normal）
         *
         * @code
           constexpr bsp::temperature::Config config = {
               .beta          = 3435.0f,
               .ntc_to_ground = false,
           };

           using ntc1 = bsp::temperature::NTC<hal::ADC<&hadc1>, config>;

           ntc1 thermistor;

           void init() { thermistor.init(); }

           void run()
           {
               thermistor.sample();
               if (thermistor.valid()) {
                   float celsius = thermistor.celsius();
               }
           }
           @endcode
         */
        template <typename adc_bus, Config _config = Config{}, hal::Mode _mode = hal::Mode::Normal>
            requires hal::adc::HasADCHandleConcept<adc_bus>
        class NTC
        {
        public:
            static inline constexpr float KelvinZeroCelsius = 273.15f;

            /** 编译期配置，可直接读取，例如 NTC<...>::config.beta */
            static inline constexpr Config config = _config;

            NTC() = default;

            /**
             * @brief 按配置初始化 ADC 通道
             *
             * 主要是把采样时间调到与源阻抗匹配。热敏分压的源阻抗约等于
             * 「固定电阻 // 标称阻值」（10K 分压时约 5K），1.5 个采样周期在
             * 12MHz ADC 时钟下只有 125ns，采样保持电容来不及充满，读数会偏低
             * 而且跳动，所以默认给的是 239.5 周期。
             *
             * 更彻底的做法是直接在 CubeMX 里改掉 Sampling Time 再重新生成。
             *
             * @return Status HAL_ADC_ConfigChannel 的结果
             */
            hal::Status init()
            {
                return adc_bus::configure_channel(config.channel, config.sampling_time);
            }

            /**
             * @brief 采样一次并换算成摄氏度（内部阻塞 samples 次转换）
             *
             * @return Status 采样失败时返回 HAL 的错误码映射，此时 valid() 为 false
             */
            hal::Status sample()
            {
                uint16_t raw = 0;

                const auto status = adc_bus::template read_average<_mode>(raw, config.samples);
                if (status != hal::Status::Ready) {
                    _valid = false;
                    return status;
                }

                _raw = raw;
                update(raw);

                return hal::Status::Ready;
            }

            /** 最近一次采样的原始值 */
            uint16_t raw() const
            { return _raw; }

            /** 最近一次换算出的 NTC 阻值（欧姆） */
            float resistance() const
            { return _resistance; }

            /** 最近一次换算出的温度（摄氏度） */
            float celsius() const
            { return _celsius; }

            /**
             * @brief 最近一次结果是否可信
             *
             * 采样失败，或者算出的温度跑出 Config 的
             * [minimum_celsius, maximum_celsius]（例如传感器断开、短路）时为 false。
             */
            bool valid() const
            { return _valid; }

        private:
            uint16_t _raw     = 0;
            float _resistance = 0.0f;
            float _celsius    = 0.0f;
            bool _valid       = false;

            /**
             * @brief 把原始值换算成 NTC 阻值
             */
            static float to_resistance(uint16_t raw)
            {
                // 掐掉两端：raw = 0 会让 ln() 没有定义，raw = 满量程会让分母为 0
                if (raw < 1U) raw = 1U;
                if (raw >= config.full_scale) raw = static_cast<uint16_t>(config.full_scale - 1U);

                const float voltage = static_cast<float>(raw) * config.reference_voltage /
                                      static_cast<float>(config.full_scale);

                if (config.ntc_to_ground) {
                    // NTC 接地、固定电阻上拉到 VCC
                    return config.divider_resistance * voltage /
                           (config.reference_voltage - voltage);
                }

                // NTC 接 VCC、固定电阻下拉到 GND
                return config.divider_resistance * (config.reference_voltage - voltage) / voltage;
            }

            /**
             * @brief 把阻值换算成摄氏度（Beta 参数方程）
             *
             * 1/T = 1/T0 + ln(R / R0) / B
             */
            static float to_celsius(float resistance)
            {
                const float nominal_kelvin = config.nominal_celsius + KelvinZeroCelsius;

                const float inv_kelvin =
                    1.0f / nominal_kelvin +
                    std::log(resistance / config.nominal_resistance) / config.beta;

                return 1.0f / inv_kelvin - KelvinZeroCelsius;
            }

            void update(uint16_t raw)
            {
                _resistance = to_resistance(raw);
                _celsius    = to_celsius(_resistance);

                // 与 NaN 的比较恒为 false，所以这一句顺便把 NaN / Inf 也判为无效
                _valid = (_celsius >= config.minimum_celsius && _celsius <= config.maximum_celsius);
            }
        };
    } // namespace temperature
} // namespace bsp

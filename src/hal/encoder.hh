#pragma once

#include "interface.hh"

#include <concepts>
#include <cstdint>

namespace hal
{
    namespace encoder
    {
        /**
         * @brief 计数方向
         */
        enum class Direction : int8_t {
            Down = -1,
            Stop = 0,
            Up   = 1
        };

        constexpr inline float TwoPi = 6.2831853f;

        /**
         * @brief 编码器统一接口
         *
         * 硬件编码器（定时器 Encoder Mode，见 hal::Encoder）与软件编码器
         * （例如 GPIO 中断里对 A/B 相做四倍频计数）都实现这一组函数，
         * 上层代码只依赖本 concept，不必关心底层到底怎么计数。
         *
         * 一个软件编码器长这样：
         *
         * @code
           class SoftEncoder
           {
           public:
               hal::Status init();                    // 配置 GPIO / 中断
               hal::Status start();                   // 开始计数
               hal::Status stop();
               void update();                         // 采样一次，搬进 ISR 累计的计数
               int32_t position() const;
               int32_t delta() const;
               void set_position(int32_t position);
               hal::encoder::Direction direction() const;
           };

           static_assert(hal::encoder::EncoderConcept<SoftEncoder>);
           @endcode
         */
        template <typename T>
        concept EncoderConcept = requires(T e, int32_t position) {
            { e.init() } -> std::same_as<hal::Status>;
            { e.start() } -> std::same_as<hal::Status>;
            { e.stop() } -> std::same_as<hal::Status>;

            { e.update() } -> std::same_as<void>;               // 采样一次
            { e.position() } -> std::same_as<int32_t>;          // 累计位置
            { e.delta() } -> std::same_as<int32_t>;             // 相对上一次采样的增量
            { e.set_position(position) } -> std::same_as<void>; // 设置当前位置
            { e.direction() } -> std::same_as<Direction>;       // 最近一次增量的方向
        };

        /**
         * @brief 把「会回绕的原始计数」累积成有符号位置
         *
         * 硬件编码器读到的定时器计数是 0 ~ ARR 的无符号值（16 位定时器即 0 ~ 65535），
         * 软件编码器如果把 A/B 相四倍频结果存在 0 ~ 3 的 2 位状态里也一样会回绕。
         * 两者都用这个累积器把回绕抵消掉，得到可以正负无限增长的 int32 位置。
         *
         * 前提：两次 update() 之间转过的脉冲数不能超过 modulus / 2，否则方向会被判反。
         * 例如 16 位定时器 + 1000 线编码器四倍频（4000 计数/圈），
         * 采样间隔内不允许转过 8 圈以上。
         */
        class Accumulator
        {
        public:
            /**
             * @brief 以当前原始计数为基准，把位置清零
             */
            void reset(int32_t raw)
            {
                set_position(raw, 0);
            }

            /**
             * @brief 以当前原始计数为基准，把位置设成指定值
             */
            void set_position(int32_t raw, int32_t position)
            {
                _last      = raw;
                _position  = position;
                _delta     = 0;
                _direction = Direction::Stop;
            }

            /**
             * @brief 喂入一个新的原始计数
             *
             * @param raw     当前原始计数
             * @param modulus 回绕模数（计数范围 0 ~ modulus - 1），传 0 表示不回绕
             * @return int32_t 相对上一次的增量
             */
            int32_t update(int32_t raw, int32_t modulus)
            {
                int32_t step = raw - _last;

                if (modulus > 0) {
                    const int32_t half = modulus / 2;
                    if (step > half)
                        step -= modulus; // 正向跨过零点
                    else if (step < -half)
                        step += modulus; // 反向跨过零点
                }

                _delta = step;
                _position += step;
                _last = raw;

                if (step > 0)
                    _direction = Direction::Up;
                else if (step < 0)
                    _direction = Direction::Down;
                else
                    _direction = Direction::Stop;

                return step;
            }

            /** 累计位置（带符号） */
            int32_t position() const
            { return _position; }

            /** 最近一次 update() 的增量 */
            int32_t delta() const
            { return _delta; }

            /** 最近一次增量的方向 */
            Direction direction() const
            { return _direction; }

        private:
            int32_t _position    = 0;
            int32_t _last        = 0;
            int32_t _delta       = 0;
            Direction _direction = Direction::Stop;
        };

        // -------------------------------------------------------------------
        // 硬件 / 软件编码器通用的模板函数
        //
        // 这些函数只依赖 EncoderConcept，两种实现都能直接用。
        // -------------------------------------------------------------------

        /**
         * @brief 采样一次并返回本次增量
         */
        template <EncoderConcept T>
        int32_t read(T &encoder)
        {
            encoder.update();
            return encoder.delta();
        }

        /**
         * @brief 采样一次并判断有没有动过
         */
        template <EncoderConcept T>
        bool has_moved(T &encoder)
        {
            encoder.update();
            return encoder.delta() != 0;
        }

        /**
         * @brief 把当前位置当作零点
         */
        template <EncoderConcept T>
        void reset(T &encoder)
        {
            encoder.set_position(0);
        }

        /**
         * @brief 本次增量换算成角度（度）
         *
         * @param pulses_per_revolution 编码器每圈脉冲数（注意是否已经四倍频）
         */
        template <EncoderConcept T>
        float delta_degrees(T &encoder, uint16_t pulses_per_revolution)
        {
            return static_cast<float>(encoder.delta()) * 360.0f /
                   static_cast<float>(pulses_per_revolution);
        }

        /**
         * @brief 本次增量换算成角度（弧度）
         */
        template <EncoderConcept T>
        float delta_radians(T &encoder, uint16_t pulses_per_revolution)
        {
            return static_cast<float>(encoder.delta()) * TwoPi /
                   static_cast<float>(pulses_per_revolution);
        }

        /**
         * @brief 角速度（度 / 秒）
         *
         * @param elapsed_ms 距上一次采样的时间，单位毫秒
         */
        template <EncoderConcept T>
        float degrees_per_second(T &encoder, uint16_t pulses_per_revolution, uint32_t elapsed_ms)
        {
            if (elapsed_ms == 0) return 0.0f;

            return delta_degrees(encoder, pulses_per_revolution) * 1000.0f /
                   static_cast<float>(elapsed_ms);
        }

        /**
         * @brief 本次增量换算成直线位移
         *
         * @param per_pulse 每个脉冲对应的位移（例如丝杆导程 / 每圈脉冲数）
         */
        template <EncoderConcept T>
        float delta_distance(T &encoder, float per_pulse)
        {
            return static_cast<float>(encoder.delta()) * per_pulse;
        }
    } // namespace encoder
} // namespace hal

// ---------------------------------------------------------------------------
// 硬件编码器：定时器 Encoder Mode
//
// 放在 HAL_TIM_MODULE_ENABLED 里（TimHandler 依赖 TIM 模块），
// 而上面的 interface 层刻意留在外面：软件编码器不需要 TIM 模块也能用。
// ---------------------------------------------------------------------------
#ifdef HAL_TIM_MODULE_ENABLED

#include "hal/timer.hh"

namespace hal
{
    /**
     * @brief 硬件编码器（定时器 Encoder Mode）
     *
     * 定时器本身（GPIO、EncoderMode、ARR）由 CubeMX 生成的 MX_TIMx_Init() 负责，
     * 这里只做启停、读计数，以及把 16 位计数累积成 int32 位置。
     *
     * @tparam _handle 工作在 Encoder Mode 的定时器句柄，例如 &htim3
     *
     * @code
       using encoder1 = hal::Encoder<&htim3>;

       encoder1 encoder;

       void init()
       {
           encoder.init();            // 检查确实配成了 Encoder Mode
           encoder.start();
       }

       void loop()
       {
           encoder.update();                                   // 采样
           float rpm = hal::encoder::delta_degrees(encoder, 1000) / 360.0f * 60.0f;
       }
       @endcode
     */
    template <TimHandler _handle>
    class Encoder
    {
    public:
        static constexpr TimHandler handle()
        { return _handle; }

        /**
         * @brief 自检：确认定时器工作在 Encoder Mode，并把累积器对齐到当前计数
         *
         * 如果 CubeMX 里忘了把 Combined Channels 设成 Encoder Mode，
         * 这里会返回 Status::Error，而不是在后面读出莫名其妙的数值。
         */
        hal::Status init()
        {
            const uint32_t mode = _handle->Instance->SMCR & TIM_SMCR_SMS;

            if (mode != TIM_ENCODERMODE_TI1 && mode != TIM_ENCODERMODE_TI2 &&
                mode != TIM_ENCODERMODE_TI12) {
                return Status::Error;
            }

            _accumulator.reset(raw());

            return Status::Ready;
        }

        /**
         * @brief 启动编码器接口
         *
         * @tparam mode   Normal（轮询）/ It（溢出中断）/ Dma
         * @param p_data1 / p_data2 / length  仅 Dma 模式使用；F1 的编码器 DMA 接口
         *                                    要求两个缓冲区，但计数本身在定时器内部完成，
         *                                    一般用不到 Dma。
         */
        template <Mode mode = Mode::Normal>
        hal::Status start(uint32_t *p_data1 = nullptr, uint32_t *p_data2 = nullptr, uint16_t length = 0)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(HAL_TIM_Encoder_Start(_handle, TIM_CHANNEL_ALL));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(HAL_TIM_Encoder_Start_IT(_handle, TIM_CHANNEL_ALL));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(
                    HAL_TIM_Encoder_Start_DMA(_handle, TIM_CHANNEL_ALL, p_data1, p_data2, length));
        }

        template <Mode mode = Mode::Normal>
        hal::Status stop()
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(HAL_TIM_Encoder_Stop(_handle, TIM_CHANNEL_ALL));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(HAL_TIM_Encoder_Stop_IT(_handle, TIM_CHANNEL_ALL));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(HAL_TIM_Encoder_Stop_DMA(_handle, TIM_CHANNEL_ALL));
        }

        /**
         * @brief 采样一次，把定时器计数累积进位置
         */
        void update()
        {
            _accumulator.update(raw(), modulus());
        }

        /** 累计位置（带符号，可正可负、不受计数器回绕影响） */
        int32_t position() const
        { return _accumulator.position(); }

        /** 最近一次 update() 的增量 */
        int32_t delta() const
        { return _accumulator.delta(); }

        /** 最近一次增量的方向 */
        encoder::Direction direction() const
        { return _accumulator.direction(); }

        /**
         * @brief 设置当前位置（会以当前计数为基准重新对齐）
         */
        void set_position(int32_t position)
        {
            _accumulator.set_position(raw(), position);
        }

        /** 定时器原始计数（0 ~ period） */
        int32_t raw() const
        { return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(_handle)); }

        /** 直接写定时器计数（一般只在自检/调试时用） */
        void set_raw(int32_t value)
        { __HAL_TIM_SET_COUNTER(_handle, static_cast<uint32_t>(value)); }

        /** 计数上限（ARR） */
        uint32_t period() const
        { return _handle->Init.Period; }

        /** 一个完整回绕周期的计数个数（ARR + 1） */
        int32_t modulus() const
        { return static_cast<int32_t>(_handle->Init.Period) + 1; }

        /** 硬件计数方向（CR1 的 DIR 位） */
        bool counting_down() const
        { return __HAL_TIM_IS_TIM_COUNTING_DOWN(_handle) != 0; }

    private:
        encoder::Accumulator _accumulator = {};
    };
} // namespace hal

#endif

#pragma once

#include "interface.hh"
#include <cstdint>
#include <algorithm>

#ifdef HAL_TIM_MODULE_ENABLED

namespace hal
{

    namespace pwm
    {
        constexpr inline uint32_t channel1 = 0x00000000U;
        constexpr inline uint32_t channel2 = 0x00000004U;
        constexpr inline uint32_t channel3 = 0x00000008U;
        constexpr inline uint32_t channel4 = 0x0000000CU;
        constexpr inline uint32_t channel5 = 0x00000010U;
        constexpr inline uint32_t channel6 = 0x00000014U;

        template <size_t index>
        constexpr inline uint32_t get_channel_from_index()
        {
            return (index - 1) << 2;
        }
        template <uint32_t channel>
        constexpr inline size_t get_index_from_channel()
        {
            return (channel >> 2) + 1;
        }

        template <typename PWM>
        concept PwmConcept = requires { typename PWM::pwm_token; };
    } // namespace pwm

    template <TimHandler _handle, uint32_t _channel>
    class PWM
    {
    public:
        static constexpr TimHandler handle()
        { return _handle; }

        template <Mode mode>
        static inline Status start(uint32_t *buffer = nullptr, uint16_t length = 0)
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(
                    HAL_TIM_PWM_Start(_handle, _channel));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(
                    HAL_TIM_PWM_Start_IT(_handle, _channel));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(
                    HAL_TIM_PWM_Start_DMA(_handle, _channel, buffer, length));
        }

        template <Mode mode>
        static inline Status stop()
        {
            if constexpr (mode == Mode::Normal)
                return static_cast<Status>(
                    HAL_TIM_PWM_Stop(_handle, _channel));
            else if constexpr (mode == Mode::It)
                return static_cast<Status>(
                    HAL_TIM_PWM_Stop_IT(_handle, _channel));
            else if constexpr (mode == Mode::Dma)
                return static_cast<Status>(
                    HAL_TIM_PWM_Stop_DMA(_handle, _channel));
        }

        static inline void set_pwm(uint32_t pwm)
        { compare_register() = pwm; }

        static inline void set_ratio(float ratio)
        { set_pwm(static_cast<uint32_t>(std::clamp(ratio, 0.0f, 1.0f) * (_handle->Init.Period))); }

        static inline uint32_t period()
        { return _handle->Init.Period; }

        struct pwm_token {
        };

    private:
        // capture/compare register
        static inline constexpr volatile uint32_t &compare_register()
        {
            auto address = &_handle->Instance->CCR1;
            return *(address + pwm::get_index_from_channel<_channel>() - 1);
        }
    };

    namespace pwm
    {
        template <typename T>
        concept HasPwmHandleConcept = requires {
            { T::handle() } -> std::same_as<TimHandler>;
        };

        template <HasPwmHandleConcept P, Mode mode = Mode::It>
        struct BaseHandler {
            void (*_on_pulse_finished)(void (*stop)()) = nullptr;

            virtual void start()
            {
                P::template start<mode>();
            }

            static void stop()
            {
                P::template stop<mode>();
            }

            void callback(TimHandler htim)
            {
                if (_on_pulse_finished && htim == P::handle()) {
                    _on_pulse_finished(stop);
                }
            }
        };
    } // namespace pwm

    namespace internal
    {
        template <typename Handlers>
        concept PwmStaticCallableConcept = requires(Handlers, TimHandler htim) {
            { Handlers::callback(htim) } -> std::same_as<void>;
        };
        template <typename Handler>
        concept PwmNormalCallableConcept = requires(Handler handler, TimHandler htim) {
            { handler.callback(htim) } -> std::same_as<void>;
        };
        template <typename Handler>
        concept PwmDirectCallableConcept = requires(Handler handler, TimHandler htim) {
            { handler(htim) } -> std::same_as<void>;
        };
        template <typename Handlers>
        concept PwmDirectCallableNoArgsConcept = requires(Handlers handler) {
            { handler() } -> std::same_as<void>;
        };

        void call_pwm_callback(TimHandler htim, PwmStaticCallableConcept auto const &handler)
        {
            handler.callback(htim);
        }
        void call_pwm_callback(TimHandler htim, PwmNormalCallableConcept auto &&handler)
        {
            handler.callback(htim);
        }
        void call_pwm_callback(TimHandler htim, PwmDirectCallableConcept auto &&handler)
        {
            handler(htim);
        }
        void call_pwm_callback(TimHandler, PwmDirectCallableNoArgsConcept auto &&handler)
        {
            handler();
        }

        void execute_pwm_callbacks(TimHandler htim, auto &&...handlers)
        {
            (call_pwm_callback(htim, std::forward<decltype(handlers)>(handlers)), ...);
        }
    } // namespace internal
} // namespace hal

#define GENERATE_PWM_PLUSE_FINISHED_CALLBACK(...)                   \
    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) \
    {                                                               \
        hal::internal::execute_pwm_callbacks(htim, __VA_ARGS__);    \
    }

#endif
#pragma once

#include "interface.hh"

#include <cassert>
#include <concepts>
#include <cstdint>
#include <utility>

namespace hal
{
    using TimHandler = TIM_HandleTypeDef *;

    namespace time
    {
        inline void delay(uint32_t ms)
        {
            uint32_t start = ::uwTick;
            uint32_t wait  = ms - 1;
            if (wait < HAL_MAX_DELAY) wait += static_cast<uint32_t>(::uwTickFreq);
            while (::uwTick - start < wait);
        }
    }

#ifdef HAL_TIM_MODULE_ENABLED

    namespace time
    {
        template <TimHandler _handle>
        inline void delay(uint32_t tick)
        {
            _handle->Instance->CNT = 0;
            HAL_TIM_Base_Start(_handle);
            while (_handle->Instance->CNT < tick);
            HAL_TIM_Base_Stop(_handle);
        }
    }

    template <TimHandler _handle>
    class Timer
    {
    public:
        using Callback = void (*)(void);

        static constexpr TimHandler handle()
        { return _handle; }

        static inline void start()
        { HAL_TIM_Base_Start_IT(_handle); }

        static inline void stop()
        { HAL_TIM_Base_Stop_IT(_handle); }
    };

    namespace timer
    {
        template <typename T>
        concept HasTimerHandleConcept = requires {
            { T::handle() } -> std::same_as<TimHandler>;
        };
        template <HasTimerHandleConcept tim, size_t tick = 1>
        struct BaseHandler {
            size_t _tick = 0;

            void (*_on_elapsed)() = nullptr;

            void callback(TimHandler htim)
            {
                if (htim == tim::handle()) {
                    if (_on_elapsed && ++_tick >= tick) {
                        _on_elapsed();
                        _tick = 0;
                    }
                }
            }
        };
    }

    namespace internal
    {

        template <typename Handlers>
        concept TimerStaticCallableConcept = requires(Handlers, TimHandler htim) {
            { Handlers::callback(htim) } -> std::same_as<void>;
        };
        template <typename Handler>
        concept TimerNormalCallableConcept = requires(Handler handler, TimHandler htim) {
            { handler.callback(htim) } -> std::same_as<void>;
        };
        template <typename Handler>
        concept TimerDirectCallableConcept = requires(Handler handler, TimHandler htim) {
            { handler(htim) } -> std::same_as<void>;
        };
        template <typename Handlers>
        concept TimerDirectCallableNoArgsConcept = requires(Handlers handler) {
            { handler() } -> std::same_as<void>;
        };

        void call_timer_callback(TimHandler htim, TimerStaticCallableConcept auto const &handler)
        {
            handler.callback(htim);
        }
        void call_timer_callback(TimHandler htim, TimerNormalCallableConcept auto &&handler)
        {
            handler.callback(htim);
        }
        void call_timer_callback(TimHandler htim, TimerDirectCallableConcept auto &&handler)
        {
            handler(htim);
        }
        void call_timer_callback(TimHandler, TimerDirectCallableNoArgsConcept auto &&handler)
        {
            handler();
        }

        void execute_timer_callbacks(TimHandler htim, auto &&...handlers)
        {
            (call_timer_callback(htim, std::forward<decltype(handlers)>(handlers)), ...);
        }
    }

#define GENERATE_TIM_PERIOD_ELAPSED_CALLBACK(...)                  \
    void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)        \
    {                                                              \
        hal::internal::execute_timer_callbacks(htim, __VA_ARGS__); \
    }

#endif
}
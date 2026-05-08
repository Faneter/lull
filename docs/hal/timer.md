# Timer 部分

与 `Timer` 相关的代码在仓库 `src/hal/timer.hh` 文件内。

## 简要说明

请提前在 `STM32CubeMX` 中配置好所要使用的定时器，然后导入 `timer.hh` 文件。

例如要使用 `TIM2` 进行定时中断：

```cpp
using Tim2 = hal::Timer<&htim2>;
Tim2::start();
```

## 设计特点

- **编译期绑定**：定时器句柄通过非类型模板参数在编译期绑定，避免运行时查找开销。
- **类型安全**：每个定时器都是独立的类型，配合 `HasTimerHandleConcept` 概念可在编译期约束定时器相关操作。
- **回调多态**：支持四种不同签名的回调形式（静态成员、实例成员、直接调用带参/无参），通过 C++20 概念自动匹配。
- **宏辅助生成**：提供 `GENERATE_TIM_PERIOD_ELAPSED_CALLBACK` 宏，一键生成 HAL 所需的 `TIM_PeriodElapsedCallback` 函数。

## API

### 延时函数

#### `void delay(uint32_t ms)`

基于 `uwTick` 的忙等待延时，不依赖任何定时器外设。

```cpp
hal::time::delay(100);  // 延时 100ms
```

> **注意**：此函数会阻塞 CPU，且精度受 `uwTick` 频率（通常为 1ms）限制，不适合高精度延时。

#### `void delay<TimHandler _handle>(uint32_t tick)`

基于指定定时器计数器的忙等待延时，精度取决于定时器时钟配置。

```cpp
hal::time::delay<&htim2>(1000);  // 等待 TIM2 计数器增加 1000 个 tick
```

> **注意**：此函数同样阻塞 CPU，但可通过配置定时器分频获得更高精度。

### 定时器类

#### `Timer<TimHandler _handle>`

定时器包装类，提供启动/停止中断定时器的接口。

##### `static TimHandler handle()`

返回绑定的定时器句柄。

```cpp
auto h = Tim2::handle();  // 返回 &htim2
```

##### `static void start()`

以中断模式启动定时器。

```cpp
Tim2::start();
```

##### `static void stop()`

停止中断模式的定时器。

```cpp
Tim2::stop();
```

### 定时器中断处理器

#### `BaseHandler<HasTimerHandleConcept tim, size_t tick = 1>`

定时器中断的基础处理器，支持按指定 tick 间隔触发回调。

| 模板参数 | 说明                                   |
| -------- | -------------------------------------- |
| `tim`    | 定时器类型，如 `Tim2`                  |
| `tick`   | 触发回调所需的定时器中断次数，默认为 1 |

```cpp
hal::timer::BaseHandler<Tim2, 10> handler;  // 每 10 次中断触发一次
handler._on_elapsed = []() {
    // 定时任务
};
```

> **注意**：需要在 `TIM_PeriodElapsedCallback` 中调用 `handler.callback(htim)`。

### 回调分发机制

`internal` 命名空间提供了一套回调分发机制，支持以下四种回调形式：

| 概念                               | 要求                       | 示例                        |
| ---------------------------------- | -------------------------- | --------------------------- |
| `TimerStaticCallableConcept`       | 静态 `callback(htim)` 方法 | `MyHandler::callback(htim)` |
| `TimerNormalCallableConcept`       | 实例 `callback(htim)` 方法 | `handler.callback(htim)`    |
| `TimerDirectCallableConcept`       | 函数调用运算符带参数       | `handler(htim)`             |
| `TimerDirectCallableNoArgsConcept` | 函数调用运算符无参数       | `handler()`                 |

### 宏

#### `GENERATE_TIM_PERIOD_ELAPSED_CALLBACK(...)`

生成 HAL 定时器周期中断回调函数，自动将中断分发到指定的处理器。

```cpp
// 定义处理器
hal::timer::BaseHandler<Tim2> tim2_handler;

// 生成回调函数
GENERATE_TIM_PERIOD_ELAPSED_CALLBACK(tim2_handler);
```

> 展开后等价于：
>
> ```cpp
> void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
> {
>     hal::internal::execute_timer_callbacks(htim, tim2_handler);
> }
> ```

## 完整示例

```cpp
#include "hal/timer.hh"

// 1. 定义定时器类型别名（假设 CubeMX 已生成 htim2）
using Tim2 = hal::Timer<&htim2>;

// 2. 定义中断处理器
hal::timer::BaseHandler<Tim2, 100> tim2_handler;  // 每 100 次中断触发

void setup()
{
    // 设置回调函数
    tim2_handler._on_elapsed = []() {
        // 每 100 个定时周期执行一次
        hal::gpio::PA<5>::toggle();  // 翻转 LED
    };

    // 启动定时器中断
    Tim2::start();
}

// 3. 生成 HAL 回调函数（通常在 main.c 对应的 cpp 文件中）
GENERATE_TIM_PERIOD_ELAPSED_CALLBACK(tim2_handler);
```

### 多定时器管理

```cpp
using Tim2 = hal::Timer<&htim2>;
using Tim3 = hal::Timer<&htim3>;

hal::timer::BaseHandler<Tim2> tim2_handler;
hal::timer::BaseHandler<Tim3, 50> tim3_handler;

// 同时管理多个定时器的中断
GENERATE_TIM_PERIOD_ELAPSED_CALLBACK(tim2_handler, tim3_handler);
```

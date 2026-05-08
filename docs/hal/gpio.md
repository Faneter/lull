# GPIO 部分

与 `GPIO` 相关的代码在仓库 `src/hal/gpio.hh` 文件内。

## 简要说明

请提前在 `STM32CubeMX` 中配置好所要操控的 `GPIO` 引脚，然后导入 `gpio.hh` 文件。

例如要对引脚 `PA5` 进行输出：

```cpp
hal::gpio::PA<5>::toggle();
```

## 设计特点

- **编译期计算**：引脚地址和掩码均在编译期通过模板元编程计算，零运行时开销。
- **类型安全**：每个引脚都是独立的类型（如 `PA<5>` 与 `PB<5>` 类型不同），可在编译期防止引脚混淆。
- **条件编译**：仅包含当前芯片实际存在的 GPIO 端口（`A` ~ `H`），避免访问不存在的寄存器。

## API

### 单引脚操作

以下方法均为 `static` 方法，可直接通过类型调用，无需实例化对象。

#### `void set()`

将引脚设为高电平。

```cpp
hal::gpio::PA<5>::set();
```

#### `void reset()`

将引脚设为低电平。

```cpp
hal::gpio::PA<5>::reset();
```

#### `void toggle()`

翻转引脚的电平状态。

```cpp
hal::gpio::PA<5>::toggle();
```

#### `Status status()`

获取引脚当前的电平状态。

##### 返回值

返回 `hal::gpio::internal::GPIO::Status` 枚举类型：

| 枚举值          | 含义   |
| --------------- | ------ |
| `Status::SET`   | 高电平 |
| `Status::RESET` | 低电平 |

```cpp
auto s = hal::gpio::PA<5>::status();
if (s == hal::gpio::PA<5>::Status::SET) {
    // 引脚为高电平
}
```

### 多引脚批量操作

支持通过 C++20 可变参数模板对多个引脚进行批量操作，所有操作按顺序执行。

#### `void set(GpioConcept auto&... gpio)`

同时将多个引脚设为高电平。

```cpp
hal::gpio::set(hal::gpio::PA<5>{}, hal::gpio::PB<0>{});
```

#### `void reset(GpioConcept auto&... gpio)`

同时将多个引脚设为低电平。

```cpp
hal::gpio::reset(hal::gpio::PA<5>{}, hal::gpio::PB<0>{});
```

#### `void toggle(GpioConcept auto&... gpio)`

同时翻转多个引脚的电平状态。

```cpp
hal::gpio::toggle(hal::gpio::PA<5>{}, hal::gpio::PB<0>{});
```

> **注意**：批量操作函数接受的是对象引用，需要构造临时对象传入（如 `hal::gpio::PA<5>{}`）。

## 支持的端口

| 端口别名  | 条件宏       | 说明                     |
| --------- | ------------ | ------------------------ |
| `PA<pin>` | `GPIOA_BASE` | GPIO 端口 A，引脚 0 ~ 15 |
| `PB<pin>` | `GPIOB_BASE` | GPIO 端口 B，引脚 0 ~ 15 |
| `PC<pin>` | `GPIOC_BASE` | GPIO 端口 C，引脚 0 ~ 15 |
| `PD<pin>` | `GPIOD_BASE` | GPIO 端口 D，引脚 0 ~ 15 |
| `PE<pin>` | `GPIOE_BASE` | GPIO 端口 E，引脚 0 ~ 15 |
| `PF<pin>` | `GPIOF_BASE` | GPIO 端口 F，引脚 0 ~ 15 |
| `PG<pin>` | `GPIOG_BASE` | GPIO 端口 G，引脚 0 ~ 15 |
| `PH<pin>` | `GPIOH_BASE` | GPIO 端口 H，引脚 0 ~ 15 |

> 引脚号 `pin` 必须在 `[0, 15]` 范围内，否则将在编译期触发 `static_assert` 错误。

## 完整示例

```cpp
#include "hal/gpio.hh"

void led_blink()
{
    using namespace hal::gpio;

    // 点亮 LED（假设接在 PA5）
    PA<5>::set();

    // 熄灭 LED
    PA<5>::reset();

    // 翻转 LED 状态
    PA<5>::toggle();

    // 读取按键状态（假设接在 PB0）
    auto key_status = PB<0>::status();

    // 批量操作：同时点亮 PA5 和 PB0
    set(PA<5>{}, PB<0>{});
}
```

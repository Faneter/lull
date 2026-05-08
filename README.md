# lull

本项目旨在利用 `C++20` 特性重构 `STM32` 的 `HAL` 库。

本项目名称 `lull`，其英文解释如下：

> a short period of calm in which little happens

表达了我对熬夜、忙碌、混沌的厌恶。

## 特性

- **编译期绑定**：外设句柄通过非类型模板参数在编译期绑定，零运行时开销。
- **类型安全**：每个外设实例都是独立的类型，配合 C++20 概念（Concept）在编译期约束接口。
- **模式多态**：支持 `Normal` / `It`（中断）/ `Dma` 三种传输模式，通过 `if constexpr` 在编译期分发。
- **回调灵活**：定时器、串口、I2C、PWM 等模块均支持多种回调签名，自动匹配最合适的调用方式。
- **零成本抽象**：所有包装均为 `static inline`，不引入额外的内存占用或运行时开销。

## 项目结构

```
lull/
├── third_party/      # 第三方库头文件
│   ├── Mahony/       # Mahony 姿态解算滤波器
│   └── etl/          # Embedded Template Library
├── src/
│   ├── hal/          # HAL 封装层（核心）
│   │   ├── gpio.hh   # GPIO 引脚操作
│   │   ├── timer.hh  # 定时器与延时
│   │   ├── i2c.hh    # I2C 通信
│   │   ├── serial.hh # UART 串口通信
│   │   ├── pwm.hh    # PWM 输出
│   │   └── interface.hh  # 公共接口定义
│   ├── bsp/          # 板级支持包
│   │   ├── interface/    # 抽象接口（IMU、Screen 等）
│   │   └── implement/    # 具体实现（MPU6050、SSD1306 等）
│   └── util/         # 工具库
│       ├── font6x8.h     # 6x8 ASCII 字库
│       └── math/         # 数学/滤波算法
└── docs/             # 文档
    └── hal/          # HAL 模块文档
```

## 快速开始

1. 在 `STM32CubeMX` 中配置好所需外设，生成代码。
2. 将本项目的 `third_party/` 和 `src/` 目录加入编译路径。
3. 在业务代码中包含对应的头文件：

```cpp
#include "hal/gpio.hh"
#include "hal/timer.hh"

// GPIO 翻转
hal::gpio::PA<5>::toggle();

// 定时器延时
hal::time::delay(100);  // 100ms

// 启动定时器中断
using Tim2 = hal::Timer<&htim2>;
Tim2::start();
```

## 模块文档

| 模块   | 文件                | 文档                                   |
| ------ | ------------------- | -------------------------------------- |
| GPIO   | `src/hal/gpio.hh`   | [docs/hal/gpio.md](docs/hal/gpio.md)   |
| Timer  | `src/hal/timer.hh`  | [docs/hal/timer.md](docs/hal/timer.md) |
| I2C    | `src/hal/i2c.hh`    | 待完善                                 |
| Serial | `src/hal/serial.hh` | 待完善                                 |
| PWM    | `src/hal/pwm.hh`    | 待完善                                 |

## 待办

1. CAN 通信的封装
2. SPI 通信的封装
3. 内存布局优化
4. 完善注释与文档

## 致谢

1. [`creeper5820` 的 `stm32_hal` 项目](https://github.com/creeper5820/stm32_hal/)
   本项目在其代码基础上进一步完善，打造了底层 `HAL` 库。

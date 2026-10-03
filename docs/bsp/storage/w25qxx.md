# W25QXX 部分

与 `W25QXX` 相关的代码在仓库 `src/bsp/implement/w25qxx.hh` 文件内，总线与片选分别复用 `src/hal/spi.hh` 与 `src/hal/gpio.hh`。

## 简要说明

请提前在 `STM32CubeMX` 中把所用的 `SPI` 配置为**主机全双工**（`Full-Duplex Master`）、`NSS = Software`，并额外配置一个普通推挽输出引脚作为片选 `CS`，然后导入 `w25qxx.hh`。

驱动是模板类，需要把 SPI 总线类型与片选引脚类型作为模板参数传入。除了 `select()` 一类的内部静态函数，**其余接口都是普通成员函数，必须实例化对象后调用**：

```cpp
using w25_cs = hal::gpio::PB<0>;
using w25qxx = bsp::storage::W25QXX<hal::SPI<&hspi1>, w25_cs>;

w25qxx flash;

flash.init();
flash.erase_sector(0x000000);
flash.write(0x000000, data, sizeof(data));
flash.read(0x000000, buffer, sizeof(buffer));
```

## 设计特点

- **编译期绑定**：SPI 句柄与片选引脚都通过模板参数在编译期绑定，运行期没有任何查表或虚调用开销。
- **双概念约束**：模板受 `hal::spi::HasSPIHandleConcept<spi_bus>` 与 `hal::gpio::GpioConcept<cs_pin>` 两个 C++20 概念约束，传错类型会在编译期直接报错。
- **全部阻塞**：所有操作都是 `hal::Mode::Normal`。SPI 位速率在 MHz 级，一页 256 字节只要零点几毫秒，比引入异步状态机简单得多，也避开了「异步传输还没结束就得释放片选」的问题。
- **读操作分块**：`read()` / `fast_read()` 内部按 `ChunkSize` 分块，单次阻塞的超时预算只有几十微秒，避免大块读取撞上 `TransferTimeout`。
- **错误可传播**：所有接口返回 `hal::Status`，`wait_while_busy()` 能区分「器件已空闲」与「等待超时」，而不会把超时伪装成成功。
- **无全局可变状态**：只保留记录 JEDEC ID 的几个成员变量，因此同一 SPI 上挂多片 Flash 时可以各自实例化，互不干扰。

## 硬件与 CubeMX 配置

| 配置项                | 建议值               | 说明                                                    |
| --------------------- | -------------------- | ------------------------------------------------------- |
| `Mode`                | `Full-Duplex Master` | 驱动使用 `transmit` / `transmit_receive`                |
| `Direction`           | `2 Lines`            | 标准 4 线 SPI                                           |
| `Data Size`           | `8 Bits`             | 指令、地址、数据都按字节存取                            |
| `First Bit`           | `MSB First`          | W25QXX 要求高位先出                                     |
| `CLK Polarity`        | `Low`                | 对应模式 0（W25QXX 同时支持模式 0 / 模式 3）            |
| `CLK Phase`           | `1 Edge`             | 同上                                                    |
| `NSS`                 | `Software`           | 片选由本驱动的 `cs_pin` 控制                            |
| `Baud Rate Prescaler` | `4` ~ `16`           | 72MHz APB2 下即 18MHz ~ 4.5MHz；`0x03` 读指令上限 50MHz |
| `CS` 引脚             | `GPIO_Output` 推挽   | 低有效，空闲时应保持高电平                              |

> **注意**：`CubeMX` 生成的 `MX_GPIO_Init()` 一般会把 `CS` 初始化为**低电平**，而 `CS` 是低有效。所以要么尽早调用 `flash.init()`（它做的第一件事就是拉高 `CS`），要么在 GPIO 初始化后手动把该引脚置高。

> **注意**：`W25QXX` 只实现了 24 位地址，可直接寻址 **16MB**。`capacity_from_code()` 虽然能识别 `0x19`（32MB），但 `W25Q256` 这类 256Mbit 器件超过 16MB 的部分需要 4 字节地址模式，本驱动尚未实现。

## API

### 模板参数

#### `W25QXX<spi_bus, cs_pin>`

| 模板参数  | 说明                                          |
| --------- | --------------------------------------------- |
| `spi_bus` | SPI 总线类型，如 `hal::SPI<&hspi1>`           |
| `cs_pin`  | 片选引脚类型，如 `hal::gpio::PB<0>`（低有效） |

```cpp
using flash_type = bsp::storage::W25QXX<hal::SPI<&hspi1>, hal::gpio::PB<0>>;
flash_type flash;
```

### 常量

| 常量               | 值       | 说明                                       |
| ------------------ | -------- | ------------------------------------------ |
| `PageSize`         | `256`    | 页大小（字节）                             |
| `SectorSize`       | `4096`   | 扇区大小（字节），擦除的最小单位           |
| `BlockSize`        | `65536`  | 块大小（字节），`erase_block()` 的单位     |
| `ChunkSize`        | `64`     | 读操作分块大小，决定栈占用（默认 64 字节） |
| `TransferTimeout`  | `100`    | 单次阻塞传输超时（毫秒）                   |
| `EraseTimeout`     | `10000`  | 扇区 / 块擦除与页编程等待超时（毫秒）      |
| `ChipEraseTimeout` | `300000` | 整片擦除等待超时（毫秒）                   |

```cpp
uint32_t pages = flash_type::PageSize;   // 编译期常量，不占空间
```

### 指令集

`bsp::storage::Instruction` 枚举，覆盖数据手册中的常用指令：

| 枚举值             | 指令 | 说明                 |
| ------------------ | ---- | -------------------- |
| `WriteEnable`      | 0x06 | 写使能               |
| `WriteDisable`     | 0x04 | 写禁止               |
| `ReadStatusReg1`   | 0x05 | 读状态寄存器 1       |
| `ReadStatusReg2`   | 0x35 | 读状态寄存器 2       |
| `WriteStatusReg`   | 0x01 | 写状态寄存器         |
| `PageProgram`      | 0x02 | 页编程               |
| `SectorErase`      | 0x20 | 扇区擦除（4KB）      |
| `BlockErase32K`    | 0x52 | 块擦除（32KB）       |
| `BlockErase64K`    | 0xD8 | 块擦除（64KB）       |
| `ChipErase`        | 0xC7 | 整片擦除             |
| `ReadData`         | 0x03 | 读数据               |
| `FastRead`         | 0x0B | 快速读               |
| `ReadJedecId`      | 0x9F | 读 JEDEC ID          |
| `ReadUniqueId`     | 0x4B | 读 64bit 唯一 ID     |
| `PowerDown`        | 0xB9 | 掉电                 |
| `ReleasePowerDown` | 0xAB | 释放掉电 / 读器件 ID |
| `EnableReset`      | 0x66 | 复位使能             |
| `ResetDevice`      | 0x99 | 执行复位             |

### 状态位

`bsp::storage::StatusReg1Bit` 枚举：

| 枚举值                | 值   | 含义                   |
| --------------------- | ---- | ---------------------- |
| `BusyBit`             | 0x01 | WIP：擦除 / 编程进行中 |
| `WriteEnableLatchBit` | 0x02 | WEL：写使能锁存        |

### 构造与初始化

#### `hal::Status init()`

释放片选，读取 JEDEC ID，并记录厂商、存储类型与容量。

```cpp
if (flash.init() != hal::Status::Ready) {
    // 器件无应答（MISO 悬空、片选接错、供电异常……）
}
```

| 返回值          | 含义                                             |
| --------------- | ------------------------------------------------ |
| `Status::Ready` | 读到合法 ID                                      |
| `Status::Error` | 传输失败，或厂商 ID 为 `0x00` / `0xFF`（无应答） |

> **注意**：`init()` 不会自动复位器件，也不会改写状态寄存器。若器件处于异常状态，可先调用 `reset()`。

### 器件信息

| 接口                      | 返回值     | 说明                           |
| ------------------------- | ---------- | ------------------------------ |
| `manufacturer_id() const` | `uint8_t`  | 厂商 ID，Winbond 为 `0xEF`     |
| `memory_type() const`     | `uint8_t`  | 存储类型，W25Q 系列为 `0x40`   |
| `capacity_code() const`   | `uint8_t`  | 容量码，如 `0x17` 表示 8MB     |
| `capacity() const`        | `uint32_t` | 总容量（字节），未识别时为 `0` |
| `page_count() const`      | `uint32_t` | 页总数                         |
| `sector_count() const`    | `uint32_t` | 扇区总数                       |

```cpp
// 例如 W25Q64：EF 40 17
uint8_t manufacturer = flash.manufacturer_id();  // 0xEF
uint32_t total       = flash.capacity();         // 8 * 1024 * 1024
```

### 状态与写使能

#### `uint8_t read_status()`

读取状态寄存器 1，可配合 `BusyBit` / `WriteEnableLatchBit` 使用。

```cpp
if (flash.read_status() & bsp::storage::BusyBit) {
    // 器件正忙
}
```

> **注意**：传输失败时返回 `0`，若需要严格的错误处理请使用 `wait_while_busy()`。

#### `uint8_t read_status2()`

读取状态寄存器 2（含 QE、SRP 等位）。

#### `bool is_busy()`

`read_status()` 的便捷封装，仅判断 `BusyBit`。

#### `hal::Status wait_while_busy(uint32_t timeout = EraseTimeout)`

忙等待直到器件空闲（擦除 / 编程结束）。

```cpp
flash.erase_sector(0x000000);   // 内部已调用 wait_while_busy
flash.wait_while_busy(2000);    // 或手动等待，2s 超时
```

| 返回值            | 含义           |
| ----------------- | -------------- |
| `Status::Ready`   | 器件已空闲     |
| `Status::Timeout` | 超过 `timeout` |
| 其它              | SPI 传输失败   |

#### `hal::Status write_enable()`

发送写使能指令（0x06），置位 WEL。

#### `hal::Status write_disable()`

发送写禁止指令（0x04），清零 WEL。

> **注意**：每次页编程与擦除之前都必须写使能，驱动内部已自动完成；WEL 在操作结束后由器件自动清零，一般无需手动调用。

### 数据读取

#### `hal::Status read(uint32_t address, uint8_t *buffer, uint32_t size)`

从任意地址连续读取，使用 `ReadData`（0x03）指令，内部按 `ChunkSize` 分块。

```cpp
uint8_t buffer[128] = {0};
flash.read(0x000100, buffer, sizeof(buffer));
```

#### `hal::Status fast_read(uint32_t address, uint8_t *buffer, uint32_t size)`

使用 `FastRead`（0x0B）指令，多一个空字节，适合较高时钟频率。

> **注意**：两个函数都不检查地址是否越界，`buffer` 为 `nullptr` 或 `size` 为 `0` 时返回 `Status::Error`。

### 数据写入

NOR Flash 的写入本质是把 `1` 写成 `0`，因此：

> **重要**：写入前目标区域**必须已擦除**，否则结果是新旧数据的按位与。覆盖写请先调用 `erase_sector()`。

#### `hal::Status page_program(uint32_t address, const uint8_t *data, uint16_t size)`

页编程，`size` 必须在 `1 ~ PageSize` 之间，且**不能跨页**（跨页时器件会回卷覆盖页首数据，因此驱动会直接返回 `Status::Error`）。

```cpp
flash.page_program(0x000000, data, 128);   // 一次写 128 字节
flash.page_program(0x000080, data, 128);   // 跨页请拆开
```

#### `hal::Status write(uint32_t address, const uint8_t *data, uint32_t size)`

连续写入：内部自动按页拆分，每页写完后等待器件空闲，因此可以跨越任意页 / 扇区边界。

```cpp
flash.erase_sector(0x000000);
flash.write(0x000000, payload, sizeof(payload));   // 自动分页，无需手动对齐
```

### 擦除

| 接口                                 | 单位 | 地址对齐 |
| ------------------------------------ | ---- | -------- |
| `hal::Status erase_sector(uint32_t)` | 4KB  | 4KB      |
| `hal::Status erase_block(uint32_t)`  | 64KB | 64KB     |
| `hal::Status erase_chip()`           | 整片 | —        |

```cpp
flash.erase_sector(0x001000);   // 擦除 0x001000 ~ 0x001FFF
flash.erase_block(0x010000);    // 擦除 0x010000 ~ 0x01FFFF
flash.erase_chip();             // 耗时较长（16MB 典型值约 40s）
```

> **注意**：擦除地址未对齐时器件的行为是实现相关的（会忽略低位），驱动不做额外检查，请自行保证对齐。

> **注意**：整片擦除可能远超看门狗复位时间，使用前请先喂狗策略或关闭看门狗。

### 复位与低功耗

| 接口                               | 说明                                    |
| ---------------------------------- | --------------------------------------- |
| `hal::Status reset()`              | 软复位（0x66 + 0x99），随后等待器件空闲 |
| `hal::Status power_down()`         | 进入掉电模式（0xB9），功耗最低          |
| `hal::Status release_power_down()` | 释放掉电（0xAB），需等待 `tRES1` 后访问 |

```cpp
flash.power_down();          // 长时间不使用时降低功耗
flash.release_power_down();  // 唤醒（建议之后稍作延时再发指令）
```

### 器件识别

#### `hal::Status read_jedec_id(uint8_t id[3])`

读取 JEDEC ID，`id[0]` 厂商、`id[1]` 存储类型、`id[2]` 容量码。

```cpp
uint8_t jedec[3] = {0};
flash.read_jedec_id(jedec);
// Winbond W25Q64: {0xEF, 0x40, 0x17}
```

### 辅助函数

#### `constexpr uint32_t capacity_from_code(uint8_t code)`

由容量码换算容量（字节），`init()` 内部使用：

| 容量码 | 容量 | 型号     |
| ------ | ---- | -------- |
| `0x14` | 1MB  | W25Q80   |
| `0x15` | 2MB  | W25Q16   |
| `0x16` | 4MB  | W25Q32   |
| `0x17` | 8MB  | W25Q64   |
| `0x18` | 16MB | W25Q128  |
| `0x19` | 32MB | W25Q256* |

```cpp
uint32_t bytes = bsp::storage::capacity_from_code(0x17);   // 8388608
```

> **注意**：`*` 表示 256Mbit 器件超出 24 位寻址范围，本驱动只能访问其低 16MB。

## 完整示例

```cpp
#include "bsp/implement/w25qxx.hh"
#include "hal/gpio.hh"
#include "hal/spi.hh"

#include "spi.h"   // CubeMX 生成：extern SPI_HandleTypeDef hspi1;

// 1. 绑定 SPI 总线与片选引脚
using w25_cs = hal::gpio::PB<0>;
using w25qxx = bsp::storage::W25QXX<hal::SPI<&hspi1>, w25_cs>;

w25qxx flash;

void storage_demo()
{
    // 2. 识别器件
    if (flash.init() != hal::Status::Ready) {
        return;   // 器件无应答
    }

    uint8_t jedec[3] = {0};
    flash.read_jedec_id(jedec);
    // jedec = {0xEF, 0x40, 0x17} -> Winbond 8MB

    // 3. 擦除 -> 写入 -> 回读校验
    const uint8_t payload[] = "Hello Flash";
    constexpr uint32_t address = 0x000000;

    flash.erase_sector(address);
    flash.write(address, payload, sizeof(payload));

    uint8_t buffer[sizeof(payload)] = {0};
    flash.read(address, buffer, sizeof(buffer));
    // buffer == "Hello Flash"
}

void storage_sleep()
{
    // 4. 长时间不用时进入掉电模式
    flash.power_down();
    flash.release_power_down();
}
```

### 参数保存 / 读取

把结构体当成一整块数据写入，注意 `erase_sector()` 会清掉整个 4KB：

```cpp
struct Config {
    uint32_t magic;
    float    kp, ki, kd;
};

void save_config(const Config &config)
{
    flash.erase_sector(0x000000);
    flash.write(0x000000, reinterpret_cast<const uint8_t *>(&config), sizeof(Config));
}

bool load_config(Config &config)
{
    flash.read(0x000000, reinterpret_cast<uint8_t *>(&config), sizeof(Config));
    return config.magic == 0x5A5AA5A5;
}
```

## 时序与超时参考

下表为 W25Q64 量级的典型值，具体以所用型号的数据手册为准：

| 操作            | 指令 | 典型耗时           | 驱动使用的超时            |
| --------------- | ---- | ------------------ | ------------------------- |
| 页编程（256B）  | 0x02 | 0.7 ms             | `EraseTimeout` (10s)      |
| 扇区擦除（4KB） | 0x20 | 45 ms              | `EraseTimeout` (10s)      |
| 块擦除（64KB）  | 0xD8 | 150 ms             | `EraseTimeout` (10s)      |
| 整片擦除（8MB） | 0xC7 | 20 s               | `ChipEraseTimeout` (300s) |
| 读 64B          | 0x03 | 约 0.11 ms @4.5MHz | `TransferTimeout` (100ms) |

## 注意事项

- **片选空闲电平**：`CS` 低有效，上电或 `MX_GPIO_Init()` 之后应尽快拉高（调用 `flash.init()` 即可）。
- **先擦后写**：NOR Flash 只能 `1 -> 0`，不擦除直接写会得到新旧数据的按位与结果。
- **页边界**：单次 `page_program()` 不能跨页；需要跨页时用 `write()`，它内部会拆分。
- **擦除对齐**：`erase_sector()` 需 4KB 对齐，`erase_block()` 需 64KB 对齐。
- **阻塞特性**：`wait_while_busy()` 是纯忙等待，会占满 CPU 并持续发起 SPI 轮询；在 RTOS 下建议把它放到低优先级任务里，或自行改造为带 `osDelay` 的版本。
- **与异步 SPI 混用**：本驱动全部使用阻塞模式，因此不要在 `hal::spi::BaseHandler` 还有 `W25QXX` 任务在飞的时候调用它的接口，否则会破坏「一次只有一个传输」的前提。若确实需要 DMA 页写入，注意片选必须等到完成回调里再释放。

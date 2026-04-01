# GPIO 部分

与`GPIO`相关的代码在仓库`src/hal/gpio.hh`文件内。

## 简要说明

请提前在`STM32CubeMX`中配置好所要操控的`GPIO`引脚，然后导入`gpio.hh`文件。


例如要对引脚`PA5`进行输出：

```cpp
hal::gpio::PA<5>::toggle();
```

## API

### `void set()`

将引脚设为高电平。

### `void reset()`

将引脚设为低电平。

### `void toggle()`

翻转引脚的电平。

### `Status status()`

获取引脚电平的状态。

#### 返回值

返回`hal::Status`类型，这是一个枚举，其中包括`SET`和`RESET`，
分别指代高电平和低电平。

#pragma once

#include <cstdint>

#include "hal/gpio.hh"
#include "hal/spi.hh"

namespace bsp
{
    namespace storage
    {
        /**
         * @brief W25QXX 系列 SPI NOR Flash 指令集
         */
        enum Instruction : uint8_t {
            WriteEnable      = 0x06,
            WriteDisable     = 0x04,
            ReadStatusReg1   = 0x05,
            ReadStatusReg2   = 0x35,
            WriteStatusReg   = 0x01,
            PageProgram      = 0x02,
            SectorErase      = 0x20, // 4KB
            BlockErase32K    = 0x52,
            BlockErase64K    = 0xD8,
            ChipErase        = 0xC7,
            ReadData         = 0x03,
            FastRead         = 0x0B,
            ReadJedecId      = 0x9F,
            ReadUniqueId     = 0x4B,
            PowerDown        = 0xB9,
            ReleasePowerDown = 0xAB,
            EnableReset      = 0x66,
            ResetDevice      = 0x99,
        };

        /**
         * @brief 状态寄存器 1 的位定义
         */
        enum StatusReg1Bit : uint8_t {
            BusyBit             = 0x01, // WIP：擦除 / 编程进行中
            WriteEnableLatchBit = 0x02, // WEL：写使能锁存
        };

        /**
         * @brief 由 JEDEC ID 第 3 字节（容量码）换算总容量（字节）
         *
         * 0x14 -> 1MB (8Mbit)    0x15 -> 2MB (16Mbit)   0x16 -> 4MB (32Mbit)
         * 0x17 -> 8MB (64Mbit)   0x18 -> 16MB (128Mbit) 0x19 -> 32MB (256Mbit)
         */
        constexpr uint32_t capacity_from_code(uint8_t code)
        {
            return (code >= 0x14U && code <= 0x19U)
                       ? (1UL << (code - 0x14U)) * 1024UL * 1024UL
                       : 0UL;
        }

        /**
         * @brief W25QXX 系列 SPI NOR Flash 驱动
         *
         * 硬件要求：SPI 主机 + 软件 NSS，片选由本驱动通过 cs_pin 控制（低有效）。
         *
         * 全部操作都是阻塞式（Mode::Normal）：SPI 位速率在 MHz 级，一页 256 字节
         * 只需零点几毫秒，比引入异步状态机简单得多，也避开了「异步传输还没结束
         * 就要释放片选」的问题。
         *
         * @tparam spi_bus SPI 总线类型，例如 hal::SPI<&hspi1>
         * @tparam cs_pin  片选引脚类型，例如 hal::gpio::PB<0>
         *
         * @code
           auto flash = bsp::storage::W25QXX<hal::SPI<&hspi1>, hal::gpio::PB<0>>();

           flash.init();
           flash.erase_sector(0x000000);
           flash.write(0x000000, data, sizeof(data));
           flash.read(0x000000, buffer, sizeof(buffer));
           @endcode
         */
        template <typename spi_bus, typename cs_pin>
            requires hal::spi::HasSPIHandleConcept<spi_bus> && hal::gpio::GpioConcept<cs_pin>
        class W25QXX
        {
        public:
            static inline constexpr uint16_t PageSize   = 256;
            static inline constexpr uint32_t SectorSize = 4 * 1024;
            static inline constexpr uint32_t BlockSize  = 64 * 1024;

            static inline constexpr uint16_t ChunkSize        = 64;     // 读操作分块大小（决定栈占用）
            static inline constexpr uint32_t TransferTimeout  = 100;    // 单次阻塞传输超时 (ms)
            static inline constexpr uint32_t EraseTimeout     = 10000;  // 扇区 / 块擦除等待 (ms)
            static inline constexpr uint32_t ChipEraseTimeout = 300000; // 整片擦除等待 (ms)

            /**
             * @brief 初始化：释放片选，读取 JEDEC ID 并记录厂商、型号与容量
             *
             * @return Status 读到合法 ID 返回 Ready，器件无应答返回 Error
             */
            hal::Status init()
            {
                deselect();

                uint8_t id[3] = {0};
                auto status   = read_jedec_id(id);
                if (status != hal::Status::Ready) return status;

                _manufacturer_id = id[0];
                _memory_type     = id[1];
                _capacity_code   = id[2];
                _capacity        = capacity_from_code(id[2]);

                // 全 0 / 全 1 说明 MISO 悬空或器件没有应答
                if (_manufacturer_id == 0x00U || _manufacturer_id == 0xFFU) {
                    _capacity = 0;
                    return hal::Status::Error;
                }

                return hal::Status::Ready;
            }

            // ---- 器件信息 ----
            uint8_t manufacturer_id() const
            { return _manufacturer_id; }

            uint8_t memory_type() const
            { return _memory_type; }

            uint8_t capacity_code() const
            { return _capacity_code; }

            uint32_t capacity() const
            { return _capacity; } // 字节

            uint32_t page_count() const
            { return _capacity / PageSize; }

            uint32_t sector_count() const
            { return _capacity / SectorSize; }

            // ---- 状态与写使能 ----
            /**
             * @brief 读取状态寄存器 1（传输失败时返回 0）
             */
            uint8_t read_status()
            {
                uint8_t status = 0;
                read_register(ReadStatusReg1, status);
                return status;
            }

            /**
             * @brief 读取状态寄存器 2（传输失败时返回 0）
             */
            uint8_t read_status2()
            {
                uint8_t status = 0;
                read_register(ReadStatusReg2, status);
                return status;
            }

            bool is_busy()
            { return (read_status() & BusyBit) != 0; }

            /**
             * @brief 阻塞等待器件空闲（擦除 / 编程结束）
             */
            hal::Status wait_while_busy(uint32_t timeout = EraseTimeout)
            {
                const uint32_t start = HAL_GetTick();

                for (;;) {
                    uint8_t status = 0;
                    auto transfer  = read_register(ReadStatusReg1, status);
                    if (transfer != hal::Status::Ready) return transfer;

                    if ((status & BusyBit) == 0) return hal::Status::Ready;
                    if (HAL_GetTick() - start > timeout) return hal::Status::Timeout;
                }
            }

            hal::Status write_enable()
            { return command(WriteEnable); }

            hal::Status write_disable()
            { return command(WriteDisable); }

            // ---- 数据读取 ----
            /**
             * @brief 从任意地址连续读取数据（指令 0x03，内部自动分块）
             */
            hal::Status read(uint32_t address, uint8_t *buffer, uint32_t size)
            { return read_impl(ReadData, address, buffer, size, 0); }

            /**
             * @brief 快速读取（指令 0x0B，多一个空字节，适合高时钟频率）
             */
            hal::Status fast_read(uint32_t address, uint8_t *buffer, uint32_t size)
            { return read_impl(FastRead, address, buffer, size, 1); }

            // ---- 数据写入 ----
            /**
             * @brief 页编程：必须在同一页内（跨页会回卷覆盖页首数据）
             *
             * @param address 起始地址
             * @param data    待写入数据
             * @param size    字节数，1 ~ PageSize
             */
            hal::Status page_program(uint32_t address, const uint8_t *data, uint16_t size)
            {
                if (data == nullptr || size == 0 || size > PageSize) return hal::Status::Error;
                if ((address / PageSize) != ((address + size - 1) / PageSize)) return hal::Status::Error;

                auto status = write_enable();
                if (status != hal::Status::Ready) return status;

                const uint8_t cmd[4] = {
                    PageProgram,
                    static_cast<uint8_t>(address >> 16),
                    static_cast<uint8_t>(address >> 8),
                    static_cast<uint8_t>(address),
                };

                select();
                status = spi_bus::template transmit<hal::Mode::Normal>(cmd, 4, TransferTimeout);
                if (status == hal::Status::Ready)
                    status = spi_bus::template transmit<hal::Mode::Normal>(data, size, TransferTimeout);
                deselect();

                if (status != hal::Status::Ready) return status;
                return wait_while_busy();
            }

            /**
             * @brief 连续写入：内部自动按页拆分，并在每页写完后等待器件空闲
             *
             * 注意：NOR Flash 只能把 1 写成 0，写入前目标区域必须已擦除，
             * 否则结果是新旧数据的按位与。覆盖写请先 erase_sector()。
             */
            hal::Status write(uint32_t address, const uint8_t *data, uint32_t size)
            {
                if (data == nullptr) return hal::Status::Error;

                while (size > 0) {
                    uint16_t chunk = static_cast<uint16_t>(PageSize - (address % PageSize));
                    if (chunk > size) chunk = static_cast<uint16_t>(size);

                    auto status = page_program(address, data, chunk);
                    if (status != hal::Status::Ready) return status;

                    address += chunk;
                    data += chunk;
                    size -= chunk;
                }

                return hal::Status::Ready;
            }

            // ---- 擦除 ----
            /**
             * @brief 擦除一个扇区（4KB），address 需 4KB 对齐
             */
            hal::Status erase_sector(uint32_t address)
            { return erase_with_address(SectorErase, address, EraseTimeout); }

            /**
             * @brief 擦除一个 64KB 块，address 需 64KB 对齐
             */
            hal::Status erase_block(uint32_t address)
            { return erase_with_address(BlockErase64K, address, EraseTimeout); }

            /**
             * @brief 整片擦除（耗时较长，16MB 典型值约 40s）
             */
            hal::Status erase_chip()
            {
                auto status = write_enable();
                if (status != hal::Status::Ready) return status;

                status = command(ChipErase);
                if (status != hal::Status::Ready) return status;

                return wait_while_busy(ChipEraseTimeout);
            }

            // ---- 复位与低功耗 ----
            /**
             * @brief 软复位（0x66 + 0x99）
             */
            hal::Status reset()
            {
                auto status = command(EnableReset);
                if (status != hal::Status::Ready) return status;

                status = command(ResetDevice);
                if (status != hal::Status::Ready) return status;

                return wait_while_busy(EraseTimeout);
            }

            hal::Status power_down()
            { return command(PowerDown); }

            hal::Status release_power_down()
            { return command(ReleasePowerDown); }

            // ---- 器件识别 ----
            /**
             * @brief 读取 JEDEC ID（厂商 + 存储类型 + 容量码）
             */
            hal::Status read_jedec_id(uint8_t id[3])
            {
                if (id == nullptr) return hal::Status::Error;

                const uint8_t tx[4] = {ReadJedecId, 0x00, 0x00, 0x00};
                uint8_t rx[4]       = {0};

                select();
                auto status = spi_bus::template transmit_receive<hal::Mode::Normal>(
                    tx, rx, sizeof(tx), TransferTimeout);
                deselect();

                if (status != hal::Status::Ready) return status;

                id[0] = rx[1];
                id[1] = rx[2];
                id[2] = rx[3];

                return hal::Status::Ready;
            }

        private:
            uint8_t _manufacturer_id = 0;
            uint8_t _memory_type     = 0;
            uint8_t _capacity_code   = 0;
            uint32_t _capacity       = 0;

            // 片选低有效
            static void select()
            { cs_pin::reset(); }

            static void deselect()
            { cs_pin::set(); }

            /**
             * @brief 只发一个字节的指令（0x06 / 0x04 / 0xC7 / 0xB9 …）
             */
            static hal::Status command(uint8_t instruction)
            {
                select();
                auto status = spi_bus::template transmit<hal::Mode::Normal>(
                    &instruction, 1, TransferTimeout);
                deselect();

                return status;
            }

            /**
             * @brief 先发指令、再发 24 位地址的指令（擦除类）
             */
            static hal::Status command_with_address(uint8_t instruction, uint32_t address)
            {
                const uint8_t cmd[4] = {
                    instruction,
                    static_cast<uint8_t>(address >> 16),
                    static_cast<uint8_t>(address >> 8),
                    static_cast<uint8_t>(address),
                };

                select();
                auto status = spi_bus::template transmit<hal::Mode::Normal>(
                    cmd, sizeof(cmd), TransferTimeout);
                deselect();

                return status;
            }

            /**
             * @brief 读寄存器的公共实现：指令 + 1 字节回读
             */
            static hal::Status read_register(uint8_t instruction, uint8_t &value)
            {
                const uint8_t dummy = 0x00;

                select();
                auto status = spi_bus::template transmit<hal::Mode::Normal>(
                    &instruction, 1, TransferTimeout);
                if (status == hal::Status::Ready)
                    status = spi_bus::template transmit_receive<hal::Mode::Normal>(
                        &dummy, &value, 1, TransferTimeout);
                deselect();

                return status;
            }

            /**
             * @brief 读操作的公共实现
             *
             * @param instruction 0x03 或 0x0B
             * @param dummy_size  0x0B 需要 1 个空字节，0x03 不需要
             */
            hal::Status read_impl(uint8_t instruction, uint32_t address, uint8_t *buffer,
                                  uint32_t size, uint8_t dummy_size)
            {
                if (buffer == nullptr || size == 0) return hal::Status::Error;

                const uint8_t cmd[5] = {
                    instruction,
                    static_cast<uint8_t>(address >> 16),
                    static_cast<uint8_t>(address >> 8),
                    static_cast<uint8_t>(address),
                    0x00, // FastRead 的空字节
                };

                select();
                auto status = spi_bus::template transmit<hal::Mode::Normal>(
                    cmd, static_cast<uint16_t>(4 + dummy_size), TransferTimeout);
                if (status == hal::Status::Ready)
                    status = receive_chunks(buffer, size);
                deselect();

                return status;
            }

            /**
             * @brief 分块全双工接收
             *
             * 这里用显式的 dummy 发送缓冲区走 transmit_receive，而不是调用
             * hal::SPI::receive()：HAL 在「主机 + 2 线」模式下会把它转发成
             * HAL_SPI_TransmitReceive(hspi, pData, pData, ...)
             * （stm32f1xx_hal_spi.c:973），即把接收缓冲区的旧内容当发送数据。
             * 对 0x03 读指令虽然无害，但语义不清晰，这里显式发 0x00。
             *
             * 分块还能避免「一次读很多字节导致阻塞超时」：ChunkSize 字节在
             * MHz 级 SPI 上只需几十微秒，远小于 TransferTimeout。
             */
            hal::Status receive_chunks(uint8_t *buffer, uint32_t size)
            {
                uint8_t dummy[ChunkSize] = {};
                uint32_t offset          = 0;

                while (offset < size) {
                    const uint32_t remaining = size - offset;
                    const uint16_t chunk     = static_cast<uint16_t>(
                        remaining < ChunkSize ? remaining : ChunkSize);

                    auto status = spi_bus::template transmit_receive<hal::Mode::Normal>(
                        dummy, buffer + offset, chunk, TransferTimeout);
                    if (status != hal::Status::Ready) return status;

                    offset += chunk;
                }

                return hal::Status::Ready;
            }

            /**
             * @brief 擦除类指令的公共实现：写使能 -> 指令 + 地址 -> 等待空闲
             */
            hal::Status erase_with_address(uint8_t instruction, uint32_t address, uint32_t timeout)
            {
                auto status = write_enable();
                if (status != hal::Status::Ready) return status;

                status = command_with_address(instruction, address);
                if (status != hal::Status::Ready) return status;

                return wait_while_busy(timeout);
            }
        };
    } // namespace storage
} // namespace bsp

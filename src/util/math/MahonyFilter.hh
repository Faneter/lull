#pragma once

#include "Mahony/MahonyAHRS.h"
#include "bsp/interface/imu.hh" // 包含你的 IMUData 结构
#include <numbers>

namespace util::math
{

    class MahonyFilter
    {
    public:
        /**
         * @param sample_freq 采样频率 (Hz)，例如 1000.0f
         * @param kp 比例增益，决定加速度计修正权重
         * @param ki 积分增益，决定消除偏移的速度
         */
        MahonyFilter(float kp = 0.5f, float ki = 0.0f)
        {
            // 注意：原库中 twoKpDef 等是宏，如果需要动态改，
            // 建议修改 MahonyAHRS.h 暴露 setGains 接口，或者直接在封装里处理
        }

        /**
         * @brief 输入 IMU 物理量并更新姿态
         * @param imu 已经过 update() 转换和 Offset 修正的物理量数据
         */
        void update(const bsp::imu::IMUData &imu, float dt)
        {
            // 1. 单位转换：Mahony 算法必须使用弧度制 rad/s
            // constexpr float deg2rad = std::numbers::pi_v<float> / 180.0f;

            // float gx = imu.gyro.x * deg2rad;
            // float gy = imu.gyro.y * deg2rad;
            // float gz = imu.gyro.z * deg2rad;

            // 2. 核心更新逻辑 (6轴模式)
            // 注意：如果加速度计 G 值单位不是 m/s^2 而是 g，算法同样适用（内部会单位化）
            _mahony.set_dt(dt);
            _mahony.updateIMU(imu.gyro.x, imu.gyro.y, imu.gyro.z, imu.accel.x, imu.accel.y, imu.accel.z);
        }

        /**
         * @brief 获取当前姿态（度）
         */
        void get_angles(float &pitch, float &roll, float &yaw)
        {
            pitch = _mahony.getPitch();
            roll  = _mahony.getRoll();
            yaw   = _mahony.getYaw();
        }

        // 提供重置接口，在机器人校准或启动时使用
        void reset()
        {
            _mahony = Mahony(); // 重新实例化或调用初始化
        }

    private:
        Mahony _mahony;
    };

} // namespace util::math
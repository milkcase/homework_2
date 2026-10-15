#ifndef GM6020_H
#define GM6020_H

#include <stdint.h>

// GM6020 直流无刷电机 CAN 协议封装类
//
// 控制(电流模式, 需固件 >=1.0.11.2，并在 Assistant >=2.7 开启电流环):
//   标识符 0x1FE (ID 1~4) / 0x2FE (ID 5~7), 标准帧, 8 字节
//   每电机 2 字节 int16 电流给定, 范围 -16384 ~ +16384 对应 -3 A ~ +3 A
//
// 反馈(电机 -> 主控, 1kHz):
//   标识符 0x204 + ID (ID=1 -> 0x205, ID=2 -> 0x206, ...)
//   DATA[0..1] 机械角度 0~8191 (8192 = 一圈)
//   DATA[2..3] 转速 rpm (int16)
//   DATA[4..5] 实际转矩电流 (int16)
//   DATA[6]    电机温度 (°C)
//   DATA[7]    保留
class Gm6020 {
public:
    // id: 电机拨码开关设置的 ID (1~7), 对应 Bit[2:0]
    explicit Gm6020(uint8_t id);
    ~Gm6020() = default;

    // 反馈帧 / 控制帧标识符
    uint32_t rxId() const;   // 0x204 + id
    uint32_t txId() const;   // 0x1FE(id<=4) / 0x2FE(id>=5)

    // ---- 反馈 ----
    float angle() const;     // 累计机械角度 (rad), 支持多圈连续
    float vel() const;       // 转速 (rad/s)
    float current() const;   // 实际转矩电流 (A, 近似)
    float temp() const;      // 电机温度 (°C)
    int16_t velRpm() const;  // 原始转速 (rpm)

    // ---- 控制 ----
    void setCurrent(int16_t raw); // 原始电流给定，内部限幅 +/-16384
    int16_t currentCommand() const; // 原始电流给定；current() 返回反馈电流 A

    // ---- 协议编解码 ----
    void decode(const uint8_t *data);   // 解析 8 字节反馈帧
    void encode(uint8_t *data) const;   // 将本电机电流给定写入 8 字节控制帧对应位置

private:
    uint8_t  id_;
    int16_t  current_command_;

    uint16_t raw_angle_;      // 0~8191
    int16_t  raw_vel_rpm_;
    int16_t  raw_current_;
    uint8_t  raw_temp_;

    int32_t  full_turns_;     // 累计整圈数 (正转 +1, 反转 -1)
    uint16_t last_raw_angle_; // 上一拍原始角度, 用于跨零检测
    bool angle_initialized_;
};

#endif // GM6020_H

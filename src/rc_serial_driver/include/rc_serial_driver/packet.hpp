// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rc_serial_driver
{
//     struct SendPacket
//     {
//         // uint8_t header = 43;
//         // float chasis_motor01;
//         // float chasis_motor02;
//         // float chasis_motor03;
//         // float chasis_motor04;
//         // uint16_t checksum = 0; // This is for the CRC checksum. Remember to sizeof(SendPacket) represent the real length of the data

//         uint8_t header = 43;
//         float chasis_vx;
//         float chasis_vy;
//         float chasis_w;
//         float x_ball;
//         float y_ball;
//         uint8_t vaccum;
//         uint8_t tail = 43;
//         // uint16_t checksum =
//         // This is for the CRC checksum. Remember to sizeof(SendPacket) represent the real length of the data

//     } __attribute__((packed));



    // struct SendPacket {
    //   uint8_t header = 43;
    //   float cmd_vx = 0.0f;
    //   float cmd_vy = 0.0f;
    //   float desire_yaw = 0.0f;
    //   float measure_yaw = 0.0f;
    //   float x_dot = 0.0f;
    //   float y_dot = 0.0f;
    //   uint8_t d = 0;
    //   uint8_t tail = 43;
    // } __attribute__((packed));

    struct SendPacket
    {
    //   uint8_t header = 0x5A;
    uint8_t header = 43;
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float desire_yaw;
    float measure_yaw;
    float x_dot = 0.0f;
    float y_dot = 0.0f;
    //    0 默认
    //    1 取球动2006
    //    2 取球动小米
    uint8_t arm_mode;
    //   气泵开关
    uint8_t gass_mode;
    uint8_t tail = 43;
    // uint16_t checksum

    } __attribute__((packed));

    struct ReceivePacket
    {
        uint8_t header = 0x5A;
        uint8_t detect_color : 1; // 0-red 1-blue
        bool reset_tracker : 1;
        uint8_t reserved : 6;
        float roll;
        float pitch;
        float yaw;
        float aim_x;
        float aim_y;
        float aim_z;
        uint16_t checksum = 0;
    } __attribute__((packed));

    inline ReceivePacket fromVector(const std::vector<uint8_t> &data)
    {
        ReceivePacket packet;
        std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
        return packet;
    }

    inline std::vector<uint8_t> toVector(const SendPacket &data)
    {
        std::vector<uint8_t> packet(sizeof(SendPacket));
        std::copy(
            reinterpret_cast<const uint8_t *>(&data),
            reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
        return packet;
    }

}

#endif
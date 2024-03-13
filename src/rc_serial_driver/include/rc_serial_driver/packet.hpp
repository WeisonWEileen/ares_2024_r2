// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rc_serial_driver
{
    struct SendPacket
    {
        uint8_t header = 0x5A;
        uint8_t chasis_motor01;
        uint8_t chasis_motor02;
        uint8_t chasis_motor03;
        uint8_t chasis_motor04;
    } __attribute__((packed));

// inline ReceivePacket fromVector(const std::vector<uint8_t> &data)
// {
//     ReceivePacket packet;
//     std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
//     return packet;
// }

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
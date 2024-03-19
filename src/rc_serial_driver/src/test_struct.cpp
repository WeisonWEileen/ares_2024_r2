#include <stdint.h>
#include "crc.hpp"
// #include <packet.hpp>
#include <vector>

inline std::vector<uint8_t> toVector(const SendPacket &data)
{
    std::vector<uint8_t> packet(sizeof(SendPacket));
    std::copy(
        reinterpret_cast<const uint8_t *>(&data),
        reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
    return packet;
}


struct SendPacket
{
    uint8_t header = 0x5A;
    uint8_t chasis_motor01;
    uint8_t chasis_motor02;
    uint8_t chasis_motor03;
    uint8_t chasis_motor04;
} __attribute__((packed));

int main()
{
    SendPacket packet;
    packet.chasis_motor01 = 100;
    packet.chasis_motor02 = 100;
    packet.chasis_motor03 = 100;
    packet.chasis_motor04 = 100;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    std::vector<uint8_t> data = toVector(packet);
    return 0;
}
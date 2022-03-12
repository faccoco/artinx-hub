#pragma once

#include "Utility.hpp"
#include <cstdint>

struct GimbalFdbPacket {
    static constexpr uint16_t id = 0x0A;
    float yaw, pitch;
    GimbalFdbPacket(float yaw, float pitch) : yaw(yaw), pitch(pitch) {}
    explicit GimbalFdbPacket(std::array<uint8_t, 1024>& buffer) {
        uint8_t yawRawData[4], pitchRawData[4];
        memcpy(yawRawData, buffer.data() + 6, 4);
        memcpy(pitchRawData, buffer.data() + 10, 4);
        yaw = *reinterpret_cast<float*>(yawRawData);
        pitch = *reinterpret_cast<float*>(pitchRawData);
    }
    static GimbalFdbPacket receive(std::array<uint8_t, 1024>& buffer) {
        uint8_t yawRawData[4], pitchRawData[4];
        memcpy(yawRawData, buffer.data() + 6, 4);
        memcpy(pitchRawData, buffer.data() + 10, 4);
        float yawData = *reinterpret_cast<float*>(yawRawData), pitchData = *reinterpret_cast<float*>(pitchRawData);
        return { yawData, pitchData };
    }
};

struct GimbalSetPacket {
    static constexpr size_t size = 17;
    static constexpr uint16_t id = 0x0F;
    float yaw, pitch;
    uint8_t isFire;
    std::array<uint8_t, size> buffer;

    GimbalSetPacket(float yaw, float pitch, bool isFire) : yaw{ yaw }, pitch{ pitch }, isFire{ isFire }, buffer{} {
        serialize();
    }

    void serialize() {
        auto* yawData = reinterpret_cast<uint8_t*>(&yaw);
        auto* pitchData = reinterpret_cast<uint8_t*>(&pitch);
        buffer = { // Header: magicNumber, dataLen(2 float and 1 uint8_t), seq,seq, pre_calculated crc8, id
                   0xA5,
                   9,
                   0,
                   0,
                   77,
                   id,
                   yawData[0],
                   yawData[1],
                   yawData[2],
                   yawData[3],
                   pitchData[0],
                   pitchData[1],
                   pitchData[2],
                   pitchData[3],
                   isFire
        };
        uint16_t crc16Result = Crc::Get_CRC16_Check_Sum(buffer.data(), 15, Crc::CRC16_INIT);
        buffer[15] = static_cast<uint8_t>(crc16Result);
        buffer[16] = static_cast<uint8_t>(crc16Result >> 8);
    }
};

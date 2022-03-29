#pragma once

#include "Utility.hpp"
#include <cstdint>

struct FdbPacket {
    static constexpr uint16_t id = 0x0A;
    float yaw, pitch, downYaw, downPitch, bulletSpeed;
    uint8_t color, shooterId;
    explicit FdbPacket(std::array<uint8_t, 1024>& buffer) {
        uint8_t rawFloatData[4];
        memcpy(rawFloatData, buffer.data() + 6, 4);
        yaw = *reinterpret_cast<float*>(rawFloatData);
        memcpy(rawFloatData, buffer.data() + 10, 4);
        pitch = *reinterpret_cast<float*>(rawFloatData);
        memcpy(rawFloatData, buffer.data() + 14, 4);
        downYaw = *reinterpret_cast<float*>(rawFloatData);
        memcpy(rawFloatData, buffer.data() + 18, 4);
        downPitch = *reinterpret_cast<float*>(rawFloatData);
        color = buffer[22];
        shooterId = buffer[23];
        memcpy(rawFloatData, buffer.data() + 24, 4);
        bulletSpeed = *reinterpret_cast<float*>(rawFloatData);
    }
};

struct GimbalSetPacket {
    static constexpr size_t size = 26;
    static constexpr uint16_t id = 0x0F;
    float yaw, pitch, downYaw{}, downPitch{};
    uint8_t isFire, downIsFire{};
    std::array<uint8_t, size> buffer;

    GimbalSetPacket(float yaw, float pitch, bool isFire, float downYaw = 0.0f, float downPitch = 0.0f, bool downIsFire = false)
        : yaw{ yaw }, pitch{ pitch }, isFire{ isFire }, downYaw{ downYaw }, downPitch{ downPitch },
          downIsFire{ downIsFire }, buffer{} {
        serialize();
    }

    void serialize() {
        // Header: magicNumber, dataLen(2 float and 1 uint8_t), seq,seq, pre_calculated crc8, id
        buffer = { 0xA5, 18, 0, 0 };
        buffer[4] = Crc::Get_CRC8_Check_Sum(buffer.data(), 4, Crc::CRC8_INIT);
        buffer[5] = id;
        memcpy(buffer.data() + 6, &yaw, 4);
        memcpy(buffer.data() + 10, &pitch, 4);
        memcpy(buffer.data() + 14, &downYaw, 4);
        memcpy(buffer.data() + 18, &downPitch, 4);
        buffer[19] = isFire;
        buffer[20] = downIsFire;
        uint16_t crc16Result = Crc::Get_CRC16_Check_Sum(buffer.data(), 15, Crc::CRC16_INIT);
        memcpy(buffer.data() + 21, &crc16Result, 2);
    }
};

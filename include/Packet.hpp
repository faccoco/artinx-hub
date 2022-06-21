#pragma once

#include "PacketHelper.hpp"
#include <cstdint>

struct FdbPacket {
    static constexpr uint16_t id = 0x0A;
    float yaw, pitch, downYaw, downPitch, bulletSpeed, speedX, speedY;
    uint8_t color, shooterId, energyMode;
    explicit FdbPacket(std::array<uint8_t, 1024>& buffer) {
        PacketReader<1024> reader(buffer);
        yaw = reader.readCompressedFloat(-4.0f, 0.0005f);
        pitch = reader.readCompressedFloat(-4.0f, 0.0005f);
        downYaw = reader.readCompressedFloat(-4.0f, 0.0005f);
        downPitch = reader.readCompressedFloat(-4.0f, 0.0005f);
        speedX = reader.readCompressedFloat(-20.0f, 0.01f);
        speedY = reader.readCompressedFloat(-20.0f, 0.01f);
        const auto mask = reader.read();
        color = mask & 1;
        shooterId = (mask >> 1) & 1;
        energyMode = (mask >> 2) & 1;
        bulletSpeed = reader.readCompressedFloat(-1.0f, 0.005f);
    }
};

struct GimbalSetPacket {
    static constexpr uint16_t id = 0x0F;
    PacketBuffer<9, id> buffer{};

    GimbalSetPacket(float yaw, float pitch, bool isFire, float downYaw = 0.0f, float downPitch = 0.0f, bool downIsFire = false) {
        buffer.serialize(yaw, -4.0f, 0.0005f);
        buffer.serialize(pitch, -4.0f, 0.0005f);
        buffer.serialize(downYaw, -4.0f, 0.0005f);
        buffer.serialize(downPitch, -4.0f, 0.0005f);
        buffer.serialize(static_cast<uint8_t>(static_cast<uint8_t>(isFire) | (static_cast<uint8_t>(downIsFire) << 1)));
        buffer.serializeCrc16();
    }
};

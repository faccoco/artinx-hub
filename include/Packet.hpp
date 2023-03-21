#pragma once

#include "AsyncSerial/BufferedAsyncSerial.h"
#include "Crc.hpp"
#include "PacketHelper.hpp"

#include <cstddef>
#include <cstdint>
#include <iterator>

struct FdbPacket final {
    static constexpr uint16_t id = 0x0A;
    float yaw, pitch, downYaw, downPitch, bulletSpeed, speedX, speedY;
    uint8_t color, shooterId, energyMode, outpostMode;
    float capEnergy, chasisPower;
    uint16_t shootDelayTime;  // ms
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
        outpostMode = (mask >> 4) & 1;
        bulletSpeed = reader.readCompressedFloat(-1.0f, 0.005f);
        capEnergy = reader.readCompressedFloat(-1.0f, 0.1f);
        chasisPower = reader.readCompressedFloat(-1.0f, 0.01f);
        shootDelayTime = reader.read<uint16_t>();
    }
};

struct GimbalSetPacket final {
    static constexpr uint16_t id = 0x0F;
    struct Info {
        float yaw, pitch;
        bool isFire;
    };

    Info up{}, down{};
    uint8_t hasTargets{};
    PacketBuffer<9, id> buffer{};

    void setUpTarget(float yaw, float pitch, bool isFire) {
        up = { yaw, pitch, isFire };
    }

    void setDownTarget(float yaw, float pitch, bool isFire) {
        down = { yaw, pitch, isFire };
    }

    void setHasTargetBits(uint8_t targetBits) {
        hasTargets = targetBits;
    }

    void serialize() {
        buffer = {};
        buffer.serialize(up.yaw, -4.0f, 0.0005f);
        buffer.serialize(up.pitch, -4.0f, 0.0005f);
        buffer.serialize(down.yaw, -4.0f, 0.0005f);
        buffer.serialize(down.pitch, -4.0f, 0.0005f);
        buffer.serialize(
            static_cast<uint8_t>(static_cast<uint8_t>(up.isFire) | (static_cast<uint8_t>(down.isFire) << 1) | (hasTargets << 2)));
        buffer.serializeCrc16();
    }
};

struct SingleBotPos final {
    uint16_t botID;
    float x;
    float y;
};

struct RadarPositionPacket final {
    constexpr static uint8_t headerSOF = 0xA5;
    constexpr static uint8_t headerSeq = 10;
    constexpr static uint8_t headerCRC8 = Crc::getHeaderCRC8(4);
    constexpr static uint16_t cmdId = 0x0303;

    BufferedAsyncSerial::Ptr& ptr;
    RadarPositionPacket(BufferedAsyncSerial::Ptr& ptr) : ptr(ptr) {}
    void sendPos(std::iterator_traits<std::vector<SingleBotPos>>& data, size_t num) {
        //        ptr->write();
    }
};

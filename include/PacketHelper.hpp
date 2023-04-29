#pragma once
#include "Crc.hpp"
#include "Hub.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

template <int bufferSize>
class PacketReader {
    const std::array<uint8_t, bufferSize>& buffer;
    int now = 6;
    std::array<uint8_t, 8> rawData{};

public:
    explicit PacketReader(const std::array<uint8_t, bufferSize>& buffer) : buffer(buffer) {}

    template <typename T>
    T read() {
        memcpy(rawData.data(), buffer.data() + now, sizeof(T));
        now += sizeof(T);
        return *reinterpret_cast<T*>(rawData.data());
    }

    uint8_t read() {
        return buffer[now++];
    }

    float readCompressedFloat(float min, float precision) {
        uint32_t raw = read<uint16_t>();
        return static_cast<float>(raw) * precision + min;
    }
};

template <int bodySize, int id>
class PacketBuffer {
    // Header: magicNumber, dataLen, seq, seq, crc8, id

public:
    int now = 6;
    std::array<uint8_t, bodySize + 8> buffer{ 0xA5, bodySize, 0, 0, Crc::getHeaderCRC8(bodySize), id };

    template <typename T>
    void serialize(T data) {
        memcpy(buffer.data() + now, &data, sizeof(T));
        now += sizeof(T);
    }

    void serialize(uint8_t data) {
        buffer[now++] = data;
    }

    void serialize(float data, float min, float precision) {
        assert(data >= min);
        assert(data <= min + 65535 * precision);
        uint16_t fixed = static_cast<uint16_t>((data - min) / precision);
        serialize(fixed);
    }

    void serializeCrc16() {
        uint16_t crc16Result = Crc::Get_CRC16_Check_Sum(buffer.data(), bodySize + 6, Crc::CRC16_INIT);
        serialize<uint16_t>(crc16Result);
    }

    [[nodiscard]] constexpr int size() const {
        return bodySize + 8;
    }

    void copyToSendBuffer(void* dest) {
        memcpy(dest, buffer.data(), size());
    }
};

template <int dataLength, int cmdId>
struct JudgeSystemPacketBuffer {
    size_t index = 7;
    std::array<uint8_t, dataLength + 9> buffer{ 0xA5, dataLength & 0x00FF, dataLength >> 8, 1, 0, cmdId & 0x00FF, cmdId >> 8 };

    JudgeSystemPacketBuffer() {
        uint16_t tmp = dataLength;
        memcpy(buffer.data() + 1, &tmp, 2);
        buffer[4] = Crc::Get_CRC8_Check_Sum(buffer.data(), 4, Crc::CRC8_INIT);
        tmp = cmdId;
        memcpy(buffer.data() + 5, &tmp, 2);
    };

    template <typename T>
    void serialize(T data) {
        std::memcpy(buffer.data() + index, &data, sizeof(T));
        index += sizeof(T);
    }

    void serialize(uint8_t data) {
        buffer[index++] = data;
    }

    void serializeCrc16() {
        uint16_t crc16Result = Crc::Get_CRC16_Check_Sum(buffer.data(), dataLength + 7, Crc::CRC16_INIT);
        serialize(crc16Result);
    }

    [[nodiscard]] constexpr int size() const {
        return dataLength + 13;
    }

    void clear() {
        index = 7;
    }
};

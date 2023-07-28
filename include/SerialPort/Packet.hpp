#pragma once

#include "AsyncSerial/BufferedAsyncSerial.h"
#include "PacketHelper.hpp"
#include "SerialPort/Crc.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>

struct MapData final {
    uint16_t botID;
    float x;
    float y;
};

struct MapMessage final {
    JudgeSystemPacketBuffer<14, 0x0305> buffer;
    char* data() {
        return reinterpret_cast<char*>(buffer.buffer.data());
    }

    [[nodiscard]] constexpr size_t size() const {
        return buffer.buffer.size();
    }

    void enBuffer(const MapData& src) {
        buffer.serialize(src.botID);
        buffer.serialize(src.x);
        buffer.serialize(src.y);
        buffer.index += 4;
        buffer.serializeCrc16();
    }

    void copyToBuffer(uint8_t* dest) {
        std::memcpy(dest, data(), size());
        clear();
    }

    void clear() {
        buffer.clear();
    }
};

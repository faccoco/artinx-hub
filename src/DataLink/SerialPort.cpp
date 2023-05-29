#include "SuppressWarningBegin.hpp"

#include "AsyncSerial/BufferedAsyncSerial.h"
#include "Crc.hpp"

#include "SuppressWarningEnd.hpp"

#include <functional>

template <typename RecvPacket, typename SendPacket>
class SerialPort {

public:
    SerialPort(const std::string& devPath, uint32_t baudRate)
        : mSerialPort(std::make_unique<BufferedAsyncSerial>()), mCheckingHeader(false), mSendBufferLen(0) {
        mSerialPort->open(devPath, baudRate);
        packetSend.serialize();
        mThread = std::thread(startListen);
    }

    constexpr static uint8_t RecvHeader = 0xA5;
    constexpr static size_t HeaderLen = 5;
    constexpr static size_t RecvBufferLen = 1024;
    constexpr static size_t SendBufferLen = 1024;

    BufferedAsyncSerial::Ptr mSerialPort;
    std::thread mThread;

    bool started = false;

    uint16_t mExpectedLen;
    std::array<uint8_t, RecvBufferLen> mPacketBuffer;
    uint32_t mPacketLen;
    std::array<uint8_t, HeaderLen> mHeaderBuffer;
    uint32_t mHeaderLen;
    bool mCheckingHeader;
    std::array<uint8_t, SendBufferLen> mSendBuffer;
    size_t mSendBufferLen;

    SendPacket packetSend{};
    std::function<void(uint16_t id)> handlePacket;  //TODO
    std::function<void()> startListen;              //TODO

    void receive() {
        if(!started)
            return;
        std::vector<char> vec = mSerialPort->read();
        mPacketLen = 0;
        for(uint8_t data : vec) {
            if(mPacketLen < RecvBufferLen) {
                mPacketBuffer[mPacketLen++] = data;
                if(mPacketLen == mExpectedLen && Crc::VerifyCrc16CheckSum(mPacketBuffer.data(), mPacketLen)) {
                    handlePacket(mPacketBuffer[5]);  // mPacketBuffer[5] ==> Protocol id
                }
            }

            if(mCheckingHeader) {
                mHeaderBuffer[mHeaderLen++] = data;
                if(mHeaderLen == HeaderLen) {
                    mCheckingHeader = false;
                    if(Crc::VerifyCrc8CheckSum(mHeaderBuffer.data(), mHeaderLen)) {
                        mExpectedLen = mHeaderBuffer[1] + 8;
                        std::copy(mHeaderBuffer.begin(), mHeaderBuffer.end(), mPacketBuffer.begin());
                        mPacketLen = 5;
                    }
                    mHeaderLen = 0;
                }
            }

            if(data == RecvHeader) {
                mCheckingHeader = true;
                mHeaderLen = 0;
                mHeaderBuffer[mHeaderLen++] = data;
            }
        }
    }

    void sendPacket() {
        packetSend.serialize();
        packetSend.buffer.copyToSendBuffer(mSendBuffer.data() + mSendBufferLen);
        mSerialPort->write(reinterpret_cast<char*>(mSendBuffer.data()), mSendBufferLen);
        mSendBufferLen += packetSend.buffer.size();

        if(mSendBufferLen > SendBufferLen)
            mSendBufferLen = 0;
        if(mSendBufferLen == 0)
            return;
        mSendBufferLen = 0;
    }

    ~SerialPort() {
        mSerialPort.release()->close();
        mThread.detach();
    }
};

#pragma once
#include "DetectedArmor.hpp"
#include "Hub.hpp"
#include "SuppressWarningBegin.hpp"

#include "AsyncSerial/BufferedAsyncSerial.h"
#include "SerialPort/Crc.hpp"
#include "Timer.hpp"
#include "Utility.hpp"

#include "SuppressWarningEnd.hpp"

#include <functional>
#include <thread>

template <typename RecvPacket, typename SendPacket>
class SerialPort {
public:
    SerialPort(const std::string& devPath, uint32_t baudRate, const std::function<void(const RecvPacket& recvPacket)>& recvCB,
               const std::function<void()>& setBeforeSend)
        : mSerialPort(std::make_unique<BufferedAsyncSerial>()), mPacketLen(0), mCheckingHeader(false), mSendBufferLen(0),
          mDevPath(devPath), mBaudRate(baudRate), mRecvCallback(recvCB), mSetBeforeSend(setBeforeSend) {
        mSerialPort->open(mDevPath, mBaudRate);
        mSendPacket.serialize();
        std::thread{ [this]() {
            while(globalStatus == RunStatus::running) {
                receive();
                std::this_thread::sleep_for(0.75ms);
                mSetBeforeSend();
                send();
            }
        } }.detach();
    }

    constexpr static uint8_t RecvHeader = 0xA5;
    constexpr static size_t HeaderLen = 5;
    constexpr static size_t RecvBufferLen = 1024;
    constexpr static size_t SendBufferLen = 1024;

    BufferedAsyncSerial::Ptr mSerialPort;

    bool mStarted = false;

    uint16_t mExpectedLen;
    std::array<uint8_t, RecvBufferLen> mPacketBuffer;
    uint32_t mPacketLen;
    std::array<uint8_t, HeaderLen> mHeaderBuffer;
    uint32_t mHeaderLen;
    bool mCheckingHeader;
    std::array<uint8_t, SendBufferLen> mSendBuffer;
    size_t mSendBufferLen;

    std::optional<TimePoint> mLastReceivedTime;
    const std::string mDevPath;
    const uint32_t mBaudRate;

    std::mutex mPacketMutex;
    SendPacket mSendPacket{};
    std::function<void(const RecvPacket& recvPacket)> mRecvCallback;
    std::function<void()> mSetBeforeSend;
    int32_t mRecordeCnt = 0;

    inline uint8_t tfRobotType(RobotType robotType) {
        switch(robotType) {
            case Sentry:
                return 7;  // 规则中哨兵ID 7
                break;
            case Outpost:
                return 10;  // 规则中前哨站ID 10
                break;
            case Base:
                return 11;  // 规则中基地ID 11
                break;
            default:
                return static_cast<uint8_t>(robotType);  // 其它与规则ID一致
                break;
        }
    }

    void receive() {
        if(!mStarted)
            return;
        mRecordeCnt++;
        std::vector<char> vec = mSerialPort->read();
        if(mRecordeCnt == 100) {
            HubLogger::visualLog(fmt::format("SerialPort Recive : {} byte information", vec.size()));
            mRecordeCnt = 0;
        }
        for(uint8_t data : vec) {
            if(mPacketLen < RecvBufferLen) {
                mPacketBuffer[mPacketLen++] = data;
                if(mPacketLen == mExpectedLen && Crc::VerifyCrc16CheckSum(mPacketBuffer.data(), mPacketLen)) {
                    if(mPacketBuffer[5] == RecvPacket::id) {  // mPacketBuffer[5] ==> Protocol id
                        mRecvCallback(RecvPacket(mPacketBuffer));
                        mLastReceivedTime = Clock::now();
                    }
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
        if(mLastReceivedTime.has_value() && Clock::now() - mLastReceivedTime.value() > std::chrono::seconds(1)) {
            HubLogger::visualLog("SerialPort: hasn't received from serial for 1s, restart serial port");
            mLastReceivedTime.reset();
            mSerialPort->close();
            mSerialPort->open(mDevPath, mBaudRate);
        }
    }

    void send() {
        {
            std::lock_guard lock{ mPacketMutex };
            mSendPacket.serialize();
            mSendPacket.buffer.copyToSendBuffer(mSendBuffer.data() + mSendBufferLen);
        }
        mSendBufferLen += mSendPacket.buffer.size();
        if(mSendBufferLen > SendBufferLen)
            mSendBufferLen = 0;
        if(mSendBufferLen == 0)
            return;
        mSerialPort->write(reinterpret_cast<char*>(mSendBuffer.data()), mSendBufferLen);
        mSendBufferLen = 0;
    }

    ~SerialPort() {
        mSerialPort.release()->close();
    }
};

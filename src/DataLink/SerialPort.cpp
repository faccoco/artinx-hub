#include "AsyncSerial/BufferedAsyncSerial.h"
#include "BlackBoard.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include "Packet.hpp"
#include "PostureData.hpp"
#include "Utility.hpp"
#include <boost/circular_buffer.hpp>
#include <caf/event_based_actor.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <fmt/format.h>

struct SerialPortSettings final {
    std::string devPath;
    uint32_t baudRate;
    double headHeightOffset;
};

template <class Inspector>
bool inspect(Inspector& f, SerialPortSettings& x) {
    return f.object(x).fields(f.field("devPath", x.devPath), f.field("baudRate", x.baudRate),
                              f.field("headHeightOffset", x.headHeightOffset));
}

class SerialPort final : public HubHelper<caf::event_based_actor, SerialPortSettings, update_head_atom, update_posture_atom> {
    constexpr static size_t bufferLen = 1024;
    constexpr static size_t headerLen = 5;

    BufferedAsyncSerial::Ptr mSerialPort;
    std::thread mThread;

    Identifier mKey;

    bool started = false;

    uint16_t mExpectedLen;
    std::array<uint8_t, bufferLen> mPacketBuffer;
    size_t mPacketLen;
    std::array<uint8_t, headerLen> mHeaderBuffer;
    size_t mHeaderLen;
    bool mCheckingHeader;

    void receive() {
        if (!started) return;
        std::vector<char> vec = mSerialPort->read();
#ifdef ARTINXHUB_DEBUG
        for (auto v : vec) std::cout << v << " ";
        std::cout << std::endl;
#endif
        for(uint8_t data : vec) {
            if(mPacketLen < bufferLen) {
                mPacketBuffer[mPacketLen++] = data;
                if(mPacketLen == mExpectedLen && Crc::VerifyCrc16CheckSum(mPacketBuffer.data(), mPacketLen)) {
                    handlePacket(mPacketBuffer[5]);
                }
            }

            if(mCheckingHeader) {
                mHeaderBuffer[mHeaderLen++] = data;
                if(mHeaderLen == 5) {
                    mCheckingHeader = false;
                    if(Crc::VerifyCrc8CheckSum(mHeaderBuffer.data(), mHeaderLen)) {
                        mExpectedLen = mHeaderBuffer[1] + 8;
                        std::copy(mHeaderBuffer.begin(), mHeaderBuffer.end(), mPacketBuffer.begin());
                        mPacketLen = 5;
                    }
                    mHeaderLen = 0;
                }
            }

            if(data == 0xA5) {
                mCheckingHeader = true;
                mHeaderLen = 0;
                mHeaderBuffer[mHeaderLen++] = data;
            }
        }
    }

    void handlePacket(uint16_t id) {
        switch(id) {
            case(GimbalFdbPacket::id): {
                GimbalFdbPacket fdb(mPacketBuffer);
//                std::cout << fdb.yaw << " " << fdb.pitch << std::endl;
//                CAF_LOG_INFO(fmt::format("{}, {}", fdb.yaw, fdb.pitch));
                fdb.yaw = (fdb.yaw < 0) ? fdb.yaw += 6.2831852 : fdb.yaw;
                //TODO: transform to -Pi ~ Pi
                const HeadInfo info {
                    SynchronizedClock::instance().now(),
                    decltype(HeadInfo::transform) {
                        glm::lookAtRH(
                            glm::dvec3{ 0.0, mConfig.headHeightOffset, 0.0 },
                            glm::dvec3{
                                std::cos(fdb.yaw - glm::half_pi<double>()) * std::cos(fdb.pitch),
                                mConfig.headHeightOffset + std::sin(fdb.pitch),
                                std::sin(fdb.yaw - glm::half_pi<double>()) * std::cos(fdb.pitch)
                            },
                            glm::dvec3{ 0.0, 1.0, 0.0 })
                    },
                    0.0, 0.0
                };
                BlackBoard::instance().updateSync(mKey, info);
                PostureData posture;
                posture.lastUpdate = SynchronizedClock::instance().now();
                posture.postureOfRobot =
                    Transform<FrameOfReference::Ground, FrameOfReference::Robot>{ glm::identity<glm::dmat4>() };
                posture.angularAccelerationOfRobot =
                    Vector<UnitType::AngularAcceleration, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
                posture.angularVelocityOfRobot =
                    Vector<UnitType::AngularVelocity, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
                posture.linearAccelerationOfRobot =
                    Vector<UnitType::LinearAcceleration, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
                posture.linearVelocityOfRobot =
                    Vector<UnitType::LinearVelocity, FrameOfReference::Ground>{ glm::zero<glm::dvec3>() };
                BlackBoard::instance().updateSync(mKey, posture);
                sendAll(update_posture_atom_v, mKey);
                sendAll(update_head_atom_v, mKey);
                break;
            }
        }
    }

public:
    SerialPort(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mSerialPort(std::make_unique<BufferedAsyncSerial>()), mKey{ typeid(SerialPort).hash_code() },
          mCheckingHeader(false) {
        const auto [devPath, baudRate, offset] = mConfig;
        mSerialPort->open(devPath, baudRate);
        mThread = std::thread{ [this]() {
            while(globalStatus == RunStatus::running) {
                receive();
            }
        } };
    }

    ~SerialPort() override {
        mSerialPort.release()->close();
        mThread.detach();
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    started = true;
                },
                 [this](set_target_info_atom, double yawAngle, double pitchAngle, bool isFire) {
                     HubLogger::print(fmt::format("yaw: {} pitch: {}", (yawAngle > 3.1415926) ? yawAngle - 6.2831852 : yawAngle, pitchAngle), "serial_out", 500);
//                     CAF_LOG_INFO(fmt::format("yaw: {} pitch: {}", yawAngle, pitchAngle));
                     GimbalSetPacket gimbalSetPacket{ static_cast<float>(yawAngle), static_cast<float>(pitchAngle), isFire };
                     mSerialPort->write(reinterpret_cast<const char*>(gimbalSetPacket.buffer.data()), GimbalSetPacket::size);
                 } };
    }
};

HUB_REGISTER_CLASS(SerialPort);

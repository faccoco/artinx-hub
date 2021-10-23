#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedTarget.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <cstdlib>
#include <fmt/format.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <queue>

static constexpr double zNear = 0.5;
static constexpr double zFar = 50.0;

struct ArmorLocatorTesterSettings final {
    uint32_t count;
    double fov;
    uint32_t imageWidth, imageHeight;
    double length, width, height;
    double noiseStd;
    double maxError;  // distance(expected,error) / distance(expected,origin)
};
template <class Inspector>
bool inspect(Inspector& f, ArmorLocatorTesterSettings& x) {
    return f.object(x).fields(f.field("count", x.count), f.field("fov", x.fov), f.field("imageWidth", x.imageWidth),
                              f.field("imageHeight", x.imageHeight), f.field("length", x.length), f.field("width", x.width),
                              f.field("height", x.height), f.field("noiseStd", x.noiseStd), f.field("maxError", x.maxError));
}

// NOTICE: ArmorLocator Only
class ArmorLocatorTester final
    : public HubHelper<caf::event_based_actor, ArmorLocatorTesterSettings, armor_detect_available_atom> {
    Identifier mKey;
    glm::dmat4 mMat;
    std::queue<Point<UnitType::Distance, FrameOfReference::Gun>> mExpected{};
    uint32_t mCount = 0;
    double mMaxError = 0.0;

    void next() {
        const auto center = glm::linearRand(glm::dvec3{ -mConfig.width, -mConfig.height, -mConfig.length },
                                            glm::dvec3{ mConfig.width, mConfig.height, -zNear });
        const auto yaw = glm::linearRand(0.1, 0.9) * glm::pi<double>();
        const auto pitch = glm::linearRand(-0.45, 0.45) * glm::pi<double>();

        const auto forward = glm::dvec3{ std::cos(yaw) * std::cos(pitch), std::sin(pitch), std::sin(yaw) * std::cos(pitch) };
        const auto up = glm::normalize(glm::dvec3{ glm::linearRand(-0.2, 0.2), 1.0, glm::linearRand(-0.2, 0.2) });
        const auto horizonal = glm::cross(forward, up);
        const auto vertical = glm::cross(horizonal, forward);

        const auto offset = widthOfSmallArmor / 8;

        const auto generateNoise = [&] {
            return glm::clamp(glm::gaussRand(0.0, mConfig.noiseStd), -mConfig.noiseStd * 3.0, mConfig.noiseStd * 3.0);
        };

        const auto generateRotatedRect = [&](const glm::dvec3& vecX) {
            const auto off1 = vecX * ((widthOfSmallArmor + offset) * 0.5);
            const auto off2 = vecX * ((widthOfSmallArmor - offset) * 0.5);
            const auto off3 = vertical * (heightOfSmallArmor * 0.5);
            const auto off4 = vertical * (heightOfSmallArmor * -0.5);

            const glm::dvec3 corners[4] = { center + off1 + off3, center + off2 + off3, center + off1 + off4,
                                            center + off2 + off4 };

            std::vector<cv::Point2f> pts;
            pts.reserve(4);

            for(auto& pos : corners) {
                const auto projected = mMat * glm::dvec4{ pos, 1.0 };
                const auto posX = projected.x / projected.w + mConfig.imageWidth * 0.5 + generateNoise();
                const auto posY = projected.y / projected.w + mConfig.imageHeight * 0.5 + generateNoise();
                pts.push_back({ static_cast<float>(posX), static_cast<float>(posY) });
            }

            return cv::minAreaRect(pts);
        };

        mExpected.push(decltype(mExpected)::value_type{ center });

        DetectedArmorsOfCar armors;
        armors.roi = cv::Rect{ 0, 0, static_cast<int>(mConfig.imageWidth), static_cast<int>(mConfig.imageHeight) };
        armors.armors.push_back({ generateRotatedRect(horizonal), generateRotatedRect(-horizonal) });

        DetectedArmorArray res;
        res.cameraInfo = { { Transform<FrameOfReference::Gun, FrameOfReference::Camera, true>{ glm::identity<glm::dmat4>() } },
                           mConfig.fov,
                           mConfig.imageWidth,
                           mConfig.imageHeight };
        res.armors.push_back(std::move(armors));

        BlackBoard::instance().updateSync(mKey, std::move(res));
        sendAll(armor_detect_available_atom_v, mKey);
    }

public:
    ArmorLocatorTester(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ArmorLocatorTester).hash_code() }, mMat{
              glm::perspectiveFovRH(mConfig.fov, static_cast<double>(mConfig.imageWidth),
                                    static_cast<double>(mConfig.imageHeight), zNear, zFar)
          } {}
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
                    for(uint32_t idx = 0; idx < mConfig.count; ++idx)
                        next();
                },
                 [&](detect_available_atom, Identifier key) {
                     const auto solved = BlackBoard::instance().get<DetectedTargetArray>(key).value().targets.front().center;

                     const auto expected = mExpected.front();
                     mExpected.pop();

                     const auto expectedRaw = expected.raw();
                     const auto solvedRaw = solved.raw();

                     const auto error = distance(expected, solved).val / glm::length(expected.raw());  // relative error
                     const auto message =
                         fmt::format("Error: {:.1f}% Expected {:.2f} {:.2f} {:.2f} Solved {:.2f} {:.2f} {:.2f}", error * 100.0,
                                     expectedRaw.x, expectedRaw.y, expectedRaw.z, solvedRaw.x, solvedRaw.y, solvedRaw.z);
                     if(error < mConfig.maxError)
                         CAF_LOG_INFO(message);
                     else
                         CAF_LOG_ERROR(message);

                     mMaxError = std::max(mMaxError, error);
                     ++mCount;

                     if(mCount >= mConfig.count) {
                         if(mMaxError < mConfig.maxError)
                             CAF_LOG_INFO("Test passed");
                         else
                             CAF_LOG_ERROR("Test failed");
                         terminateSystem(*this, mMaxError < mConfig.maxError);
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorLocatorTester);

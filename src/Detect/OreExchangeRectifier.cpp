#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <algorithm>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <limits>
#include <opencv2/barcode.hpp>
struct OreExchangeRectifierSettings final {
    std::string sr_prototxt;
    std::string sr_caffemodel;
    int validDetectionRequired;
    double permissibleAngleRangeOfError;
    double deltaTime;
    double movingRate;
};
template <class Inspector>
bool inspect(Inspector& f, OreExchangeRectifierSettings& x) {
    return f.object(x).fields(
        f.field("sr_prototxt", x.sr_prototxt).invariant([](const std::string& sr_prototxt) {
            return fs::exists(sr_prototxt) && fs::is_character_file(sr_prototxt);
        }),
        f.field("sr_caffemodel", x.sr_caffemodel).invariant([](const std::string& sr_caffemodel) {
            return fs::exists(sr_caffemodel) && fs::is_regular_file(sr_caffemodel);
        }),
        f.field("validDetectionRequired", x.validDetectionRequired).invariant([](const int& validDetectionRequired) {
            return validDetectionRequired > 0;
        }),
        f.field("permissibleAngleRangeOfError", x.permissibleAngleRangeOfError)
            .invariant([](const double& permissibleAngleRangeOfError) { return permissibleAngleRangeOfError > 0; }),
        f.field("deltaTime", x.deltaTime).invariant([](const double& deltaTime) { return deltaTime > 0; }),
        f.field("movingRate", x.movingRate).invariant([](const double& movingRate) { return movingRate > 0; }));
}
class OreExchangeRectifier final
    : public HubHelper<caf::event_based_actor, OreExchangeRectifierSettings, ore_detect_available_atom> {
    Identifier mKey;
    enum class AutomataStates { OFF, FINDING, LEARNING, BACK_FINDING, RECTIFYING };
    AutomataStates mAutomataState{ AutomataStates::OFF };
    cv::barcode::BarcodeDetector mBarcodeDetector{ mConfig.sr_prototxt, mConfig.sr_caffemodel };
    cv::Rect oreBackSurfaceDetect(const cv::Mat& image) const {
        cv::Mat corners;
        if(!mBarcodeDetector.detect(image, corners))
            return {};
        CAF_ASSERT(corners.cols == 4 && corners.channels() == 2);
        std::vector<cv::Rect> result(corners.rows);
        for(int i = 0; i < corners.rows; ++i) {
            std::vector<cv::Point2f> points(4);
            for(int j = 0; j < 4; ++j) {
                points[j] = { corners.at<cv::Vec2f>(i, j)[0], corners.at<cv::Vec2f>(i, j)[1] };
            }
            result[i] = cv::minAreaRect(points).boundingRect();
        }
        return *std::max_element(result.begin(), result.end(),
                                 [](const cv::Rect& rect1, const cv::Rect& rect2) { return rect1.area() < rect2.area(); });
    }

    void off() {
        mValidDetectionCount = 0;
        mMaxHeight = std::numeric_limits<int32_t>::min();
        mAngularVelocity = -mConfig.movingRate;
        mPrevTheta = 0;  // reset to initial value.
    }
    int mValidDetectionCount;  // 0
    void finds(const cv::Rect& rect) {
        send(+mConfig.movingRate);
        if(rect.empty()) {
            mValidDetectionCount = 0;
            return;
        }
        mValidDetectionCount++;
        // only when a consecutive image sequence is found to have bar code, do we go to the next state.
        if(mValidDetectionCount >= mConfig.validDetectionRequired) {
            mValidDetectionCount = 0;  // reuse for validMisDetectionCount.
            mAutomataState = AutomataStates::LEARNING;
        }
    }
    int mMaxHeight;  // std::numeric_limits<int32_t>::min()
    void learns(const cv::Rect& rect) {
        auto& validMisDetectionCount = mValidDetectionCount;
        const auto& validMisDetectionRequired = mConfig.validDetectionRequired;
        send(+mConfig.movingRate);
        if(rect.empty()) {
            validMisDetectionCount++;
            // only when a consecutive image sequence is found not to have bar code, do we go to the next state.
            if(validMisDetectionCount >= validMisDetectionRequired) {
                validMisDetectionCount = 0;  // reuse for mValidDetectionCount.
                mAutomataState = AutomataStates::BACK_FINDING;
            }
        } else {
            validMisDetectionCount = 0;
            mMaxHeight = std::max((int)mMaxHeight, rect.height);
        }
    }
    void backFinds(const cv::Rect& rect) {
        send(-mConfig.movingRate);
        if(rect.empty()) {
            mValidDetectionCount = 0;
            return;
        }
        mValidDetectionCount++;
        // only when a consecutive image sequence is found to have bar code, do we go to the next state.
        if(mValidDetectionCount >= mConfig.validDetectionRequired) {
            mValidDetectionCount = 0;
            mAutomataState = AutomataStates::RECTIFYING;
        }
    }
    double mAngularVelocity;  //-1
    double mPrevTheta;        // 0
    void rectifies(const cv::Rect& rect) {
        send(mAngularVelocity);
        if(rect.empty()) {
            return;  // If some frames don`t find the bar code, it does not matter.
        } else {
            const double theta = glm::acos(rect.height / mMaxHeight);  // we cannot know its direction.
            const double deltaTheta = theta - mPrevTheta;
            mPrevTheta = theta;
            if(theta < mConfig.permissibleAngleRangeOfError)
                mValidDetectionCount++;
            if(mValidDetectionCount >= mConfig.validDetectionRequired) {
                send(-90);
                mAutomataState = AutomataStates::OFF;
                return;
            }
            // if theta is declining, then the sign of velocity is correct.
            // if theta is increasing, then the sign of velocity is wrong.
            mAngularVelocity =
                glm::sign(mAngularVelocity) * (-deltaTheta / mConfig.deltaTime);  // it seems velocity is reasonable.
        }
    }
    // TODO: let ore_detect_available_atom (SerialPort.cpp) handle it.
    void send(double pitch) {
        sendAll(ore_detect_available_atom_v, pitch);
    }

public:
    OreExchangeRectifier(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(OreExchangeRectifier).hash_code() } {}
    caf::behavior make_behavior() override {
        return {
            [this](start_atom) {},
            [&](ore_instructions_atom, bool y /*may be useful*/, Identifier key) {
                off();  // initialize the values for member variable.
                // situation 1: OFF rectifier is waked up.
                // situation 2: ON or other state, someone wants to stop it when some exceptions may be observed.
                mAutomataState = (mAutomataState == AutomataStates::OFF ? AutomataStates::FINDING : AutomataStates::OFF);
            },
            [&](image_frame_atom, Identifier key) {
                if(mAutomataState == AutomataStates::OFF) {
                    off();
                    return;
                }
                const auto rect =
                    oreBackSurfaceDetect(BlackBoard::instance().get<CameraFrame>(key).value().frame);  // safe and need not copy
                switch(mAutomataState) {
                    case AutomataStates::FINDING: {
                        finds(rect);
                        return;
                    }
                    case AutomataStates::LEARNING: {
                        learns(rect);
                        return;
                    }
                    case AutomataStates::BACK_FINDING: {
                        backFinds(rect);
                        return;
                    }
                    case AutomataStates::RECTIFYING: {
                        rectifies(rect);
                        return;
                    }
                    default: {
                        CAF_RAISE_ERROR("Invalid automata state. ");
                    }
                }
            },
        };
    }
};
HUB_REGISTER_CLASS(OreExchangeRectifier);

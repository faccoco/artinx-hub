#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedArmor.hpp"
#include "DetectedCar.hpp"
#include "Hub.hpp"
#include <atomic>
#include <caf/event_based_actor.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/barcode.hpp>
#include <algorithm>
#include <cstdint>
struct OreExchangeRectifierSettings final {
    std::string sr_prototxt;
    std::string sr_caffemodel;
    int validDetectionRequired;
    double permissibleAngleRangeOfError;
    double deltaTime;
};
template <class Inspector>
bool inspect(Inspector& f, OreExchangeRectifierSettings& x) {
    return f.object(x).fields(f.field("sr_prototxt", x.sr_prototxt).invariant([](const std::string& sr_prototxt) {
        return fs::exists(sr_prototxt) && fs::is_character_file(sr_prototxt);
    }),
                              f.field("sr_caffemodel", x.sr_caffemodel).invariant([](const std::string& sr_caffemodel) {
                                  return fs::exists(sr_caffemodel) && fs::is_regular_file(sr_caffemodel);
                              }), f.field("validDetectionRequired", x.validDetectionRequired).invariant([](const int& validDetectionRequired){
                                  return validDetectionRequired>0;
                              }));
}
class OreExchangeRectifier final
    : public HubHelper<caf::event_based_actor, OreExchangeRectifierSettings, ore_detect_available_atom> {
    Identifier mKey;
    enum AutomataStates { OFF, FINDING, LEARNING, BACK_FINDING, RECTIFYING };
    std::atomic<int> mAutomataState{ OFF };  // uses atomic int to avoid concurrent modifications to mAutomataState.
    cv::barcode::BarcodeDetector mBarcodeDetector{mConfig.sr_prototxt, mConfig.sr_caffemodel};
//    struct OreBarCode{
//        cv::Rect location{};
//        OreBarCode()= default;
//        explicit OreBarCode(cv::Rect&& rect) noexcept : location(rect){}
//    };
    cv::Rect oreBackSurfaceDetect(const cv::Mat& image) const {
        cv::Mat corners;
        if(!mBarcodeDetector.detect(image, corners))
            return {};
        CV_Assert(corners.cols==4 && corners.channels()==2);
        std::vector<cv::Rect> result(corners.rows);
        for (int i = 0; i < corners.rows; ++i) {
            cv::Point2f points[4]{};
            for (int j = 0; j < 4; ++j) {
                points[j] = {corners.at<cv::Vec2f>(i, j)[0], corners.at<cv::Vec2f>(i, j)[1]};
            }
            //TODO, how to choose 3 points? If just first 3, why not reduce the points array size to 3?
            result[i] = cv::RotatedRect{points[0], points[1], points[2]}.boundingRect();
        }
        return *std::max_element(result.begin(), result.end(), [](const cv::Rect& rect1, const cv::Rect& rect2){
            return rect1.area()<rect2.area();
        });
    }
    cv::Mat preProcessImage(const cv::Mat& image) const{
        //TODO
        return image;
    }

    void off(){
        mValidDetectionCount = 0;
        mMaxHeight = INT32_MIN;
    }
    std::atomic<int> mValidDetectionCount{0};
    void finds(const cv::Rect& rect){
        send(+1.0);
        if(rect.empty()) {
            mValidDetectionCount = 0;
            return;
        }
        mValidDetectionCount++;
        // only when a consecutive image sequence is found to have bar code, do we go to the next state.
        if(mValidDetectionCount >= mConfig.validDetectionRequired) {
            mValidDetectionCount = 0; //reuse for validMisDetectionCount.
            mAutomataState = LEARNING;
        }
    }
    std::atomic<int> mMaxHeight{INT32_MIN};
    void learns(const cv::Rect& rect){
        auto& validMisDetectionCount = mValidDetectionCount;
        const auto& validMisDetectionRequired = mConfig.validDetectionRequired;
        send(+1.0);
        if(rect.empty()){
            validMisDetectionCount++;
            // only when a consecutive image sequence is found not to have bar code, do we go to the next state.
            if(validMisDetectionCount >= validMisDetectionRequired) {
                validMisDetectionCount = 0; //reuse for mValidDetectionCount.
                mAutomataState = BACK_FINDING;
            }
        }else{
            validMisDetectionCount = 0;
            mMaxHeight = std::max((int)mMaxHeight, rect.height);
        }
    }
    void backFinds(const cv::Rect& rect){
        send(-1.0);
        if(rect.empty()) {
            mValidDetectionCount = 0;
            return;
        }
        mValidDetectionCount++;
        // only when a consecutive image sequence is found to have bar code, do we go to the next state.
        if(mValidDetectionCount >= mConfig.validDetectionRequired) {
            mValidDetectionCount = 0;
            mAutomataState = RECTIFYING;
        }
    }
    void rectifies(const cv::Rect& rect){
        static double angularVelocity = -1; //? send velocity or send displacement ?
        static double prevTheta = 0;
        send(angularVelocity);
        if(rect.empty()) {
            return;
        }else{
            const double theta = glm::acos(rect.height/ mMaxHeight); //we cannot know its direction.
            const double deltaTheta = theta - prevTheta;
            prevTheta = theta;
            if(theta<mConfig.permissibleAngleRangeOfError)
                mValidDetectionCount++;
            if(mValidDetectionCount>=mConfig.validDetectionRequired){
                send(-90);
                mAutomataState = OFF;
            }
            //if theta is declining, then the sign of velocity is correct.
            //if theta is increasing, then the sign of velocity is wrong.
            angularVelocity = glm::sign(angularVelocity) * (-deltaTheta/mConfig.deltaTime);  //it seems velocity is reasonable.
        }
    }
public:
    OreExchangeRectifier(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(OreExchangeRectifier).hash_code() } {}
    //TODO: let ore_detect_available_atom (SerialPort.cpp) handle it.
    void send(double pitch){
        sendAll(ore_detect_available_atom, pitch);
    }
    caf::behavior make_behavior() override {
        return {
            [this](start_atom) {},
            [&](ore_instructions_atom, bool/*may be useful*/, Identifier key) { mAutomataState = (mAutomataState == OFF ? FINDING : OFF); },
            [&](image_frame_atom, Identifier key) {
                if(mAutomataState==OFF){
                    off();
                    return;
                }
                const auto rect =
                    oreBackSurfaceDetect(preProcessImage(BlackBoard::instance().get<CameraFrame>(key).value().frame));
                switch(mAutomataState) {
                    case FINDING: {
                        finds(rect);
                        return;
                    }
                    case LEARNING: {
                        learns(rect);
                        return;
                    }
                    case BACK_FINDING: {
                        backFinds(rect);
                        return;
                    }
                    case RECTIFYING: {
                        rectifies(rect);
                        return;
                    }
                    default: {
                        throw std::runtime_error("Invalid automata state. ");
                    }
                }
            },
        };
    }
};
HUB_REGISTER_CLASS(OreExchangeRectifier);

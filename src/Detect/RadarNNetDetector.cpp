#ifdef ARTINX_RADAR
#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "RadarInfo.hpp"

#include "SuppressWarningBegin.hpp"

#include <caf/event_based_actor.hpp>
#include <infer.hpp>
#include <yolo.hpp>

#include "SuppressWarningEnd.hpp"

#include <caf/actor_config.hpp>
#include <exception>
#include <opencv2/core.hpp>
#include <optional>
#include <string>

struct RadarNNetDetectorSettings final {
    std::string modelPath;
    double carConfidence = 0.7;
    double armorConfidence = 0.7;
    uchar greenThreshold;
    bool selfRed;
};

template <typename Inspector>
bool inspect(Inspector& f, RadarNNetDetectorSettings& x) {
    return f.object(x).fields(f.field("modelPath", x.modelPath), f.field("carConfidence", x.carConfidence),
                              f.field("armorConfidence", x.armorConfidence), f.field("greenThreshold", x.greenThreshold),
                              f.field("selfRed", x.selfRed));
}

class RadarNNetDetector final
    : public HubHelper<caf::event_based_actor, RadarNNetDetectorSettings, bot_locate_request_atom, image_frame_atom> {
    Identifier mKey;
    std::shared_ptr<yolo::Infer> model;
    std::vector<yolo::Box*> mCars, mArmors;

    void debugView(const std::string_view& name, const cv::Mat& src, const std::function<void(cv::Mat&)>& func) {

        const auto hash = std::hash<std::string_view>{}(name);
        const Identifier newKey{ mKey.val ^ hash };

        cv::Mat res;
        src.copyTo(res);
        if(res.depth() == CV_8U)
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 255 });
        else
            cv::putText(res, name.data(), { 0, 20 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{ 0, 255, 0 });

        func(res);

        CameraFrame frame;
        frame.frame = std::move(res);

        sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(newKey, std::move(frame), name));
    }

    constexpr static bool comparer(const yolo::Box* l, const yolo::Box* r) {
        if(l->left == r->left)
            return l->top < r->top;
        return l->left < r->left;
    };

    constexpr static bool bound(const yolo::Box* armor, const yolo::Box* car) {
        return armor->left > car->left && armor->right < car->right && armor->top > car->top && armor->bottom < car->bottom;
    };

    constexpr static bool cross(const yolo::Box* l, const yolo::Box* r) {
        return l->right > r->left || l->bottom > r->top;
    }

    static bool invalidBox(const yolo::Box& src, const cv::Mat frame) {
        return src.left < 0 || src.right < 0 || src.left > src.right || src.top > src.bottom || src.right > frame.cols ||
            src.bottom > frame.rows;
    }

    Color identifyColor(const cv::Mat& frame, const yolo::Box* car) {
        auto roiFrame = frame(cv::Rect{ static_cast<int>(car->left), static_cast<int>(car->top),
                                        static_cast<int>(car->right - car->left), static_cast<int>(car->bottom - car->top) });
        size_t bCount = 0, rCount = 0;
        for(int i = 0; i < roiFrame.rows; ++i) {
            uchar* ptr = roiFrame.ptr(i);
            for(int j = 0; j < roiFrame.cols; ++j) {
                if(ptr[1] >= mConfig.greenThreshold) {
                    continue;
                } else if(ptr[0] > ptr[2]) {
                    ++bCount;
                } else {
                    ++rCount;
                }
                ptr += 3;
            }
        }
        return bCount > rCount ? Color::Blue : Color::Red;
    }

    void identifyCars(DetectedBots& res) {
        std::vector<std::pair<yolo::Box*, std::vector<yolo::Box*>>> carsGroup;
        std::sort(mArmors.begin(), mArmors.end(), comparer);
        for(size_t i = 0; i < mCars.size(); ++i) {
            carsGroup.emplace_back(mCars[i], std::vector<yolo::Box*>{});
            for(size_t j = 0; j < mArmors.size(); ++j) {
                if(bound(mArmors[j], mCars[i])) {
                    carsGroup[i].second.push_back(mArmors[j]);
                }
            }
        }
        for(const auto& group : carsGroup) {
            res.botBoxes.push_back(*group.first);
            if(group.second.empty()) {
                switch(identifyColor(res.frame.frame, group.first)) {
                    case Color::Red:
                        res.botBoxes.back().class_label = 19;
                        break;
                    case Color::Blue:
                        res.botBoxes.back().class_label = 10;
                        break;
                    default:
                        continue;
                }
            } else {
                if(group.second.size() == 1) {
                    res.botBoxes.back().class_label = group.second[0]->class_label;
                } else {
                    int label = group.second[0]->class_label;
                    double maxConfidence = group.second[0]->confidence;
                    for(const auto armor : group.second) {
                        if(label != armor->class_label && maxConfidence < armor->class_label) {
                            label = armor->class_label;
                            maxConfidence = armor->confidence;
                        }
                    }
                    res.botBoxes.back().class_label = label;
                }
            }
        }
    }

    DetectedBots detect(const CameraFrame& frame) {
        auto srcFrame = yolo::Image(frame.frame.data, frame.frame.cols, frame.frame.rows);
        auto inferRes = model->forward(srcFrame);
        cv::Mat debug;
        frame.frame.copyTo(debug);
        DetectedBots result;
        result.frame = frame;
        std::vector<std::string> pointsName;
        mCars.clear();
        mArmors.clear();
        for(auto& atom : inferRes) {
            if(invalidBox(atom, frame.frame))
                continue;
            if(atom.class_label == 0) {
                if(atom.confidence < mConfig.carConfidence) {
                    continue;
                } else {
                    mCars.push_back(&atom);
                }
            } else if(atom.class_label == 1 || atom.class_label > 20) {
                continue;
            } else {
                if(atom.confidence < mConfig.armorConfidence) {
                    continue;
                } else {
                    mArmors.push_back(&atom);
                }
            }
            static const auto isCar = [](const yolo::Box& obj) { return obj.class_label == 0; };
            static const auto parseArmor = [](const yolo::Box& armor) -> std::pair<Color, int> {
                if(armor.class_label > 1 && armor.class_label <= 10)
                    return { Color::Blue, armor.class_label - 1 };
                else if(armor.class_label > 10 && armor.class_label <= 19)
                    return { Color::Red, armor.class_label - 10 };
                return { Color::Negative, 0 };
            };
            cv::rectangle(debug, cv::Point2f(atom.left, atom.top), cv::Point2f(atom.right, atom.bottom), cv::Scalar(0, 255, 0),
                          1);
            if(isCar(atom)) {
                cv::putText(debug, std::to_string(atom.class_label), { static_cast<int>(atom.left), static_cast<int>(atom.top) },
                            cv::FONT_HERSHEY_SIMPLEX, 1, { 0, 255, 255 });
            } else {
                auto info = parseArmor(atom);
                if(info.first != Color::Negative)
                    cv::putText(debug, std::to_string(info.second), { static_cast<int>(atom.left), static_cast<int>(atom.top) },
                                cv::FONT_HERSHEY_COMPLEX, 1,
                                info.first == Color::Red ? cv::Scalar{ 150, 150, 255 } : cv::Scalar{ 255, 150, 150 });
            }
        }
        debugView("Detected", debug, [](auto&) {});
        identifyCars(result);
        return result;
    }

    void removeSelfColor(DetectedBots& origin) {
        std::vector<yolo::Box> selected;
        constexpr auto isRed = [](yolo::Box& box) { return box.class_label > 10 && box.class_label <= 19; };
        constexpr auto isBlue = [](yolo::Box& box) { return box.class_label > 1 && box.class_label <= 10; };
        if(mConfig.selfRed) {
            for(auto& box : origin.botBoxes)
                if(isRed(box))
                    selected.push_back(std::move(box));
        } else {
            for(auto& box : origin.botBoxes)
                if(isBlue(box))
                    selected.push_back(std::move(box));
        }
        std::swap(selected, origin.botBoxes);
    }

public:
    RadarNNetDetector(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) } {
        if(!fs::exists(mConfig.modelPath)) {
            const auto err = "Model not exist at: " + mConfig.modelPath;
            logError(err);
            throw std::runtime_error(err.c_str());
        }
        try {
            model = yolo::load(mConfig.modelPath, yolo::Type::V7);
        } catch(std::exception& e) {
            logError(e.what());
        }
    }

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [this](image_frame_atom, Identifier key) {
                     if(auto data = BlackBoard::instance().get<CameraFrame, std::string_view>(key)) {
                         const auto& [frame, name] = data.value();
                         auto result = detect(frame);
                         removeSelfColor(result);
                         HubLogger::watch("Detected bots:", result.botBoxes.size());
                         ACTOR_PROTOCOL_CHECK(bot_locate_request_atom, TypedIdentifier<DetectedBots>);
                         sendAll(bot_locate_request_atom_v, BlackBoard::instance().updateSync(mKey, std::move(result)));
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(RadarNNetDetector);
#endif

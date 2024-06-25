//
// Created by zitto on 24-4-18.
//
#include "BlackBoard.hpp"
#include "DetectedArmor.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"

#include <filesystem>
#include <fmt/format.h>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

#include <unistd.h>
#include <fstream>

struct ArmorLabelerSettings final {
    std::string savePath;
    int saveInterval;
    int startId;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorLabelerSettings& x) {
    return f.object(x).fields(f.field("savePath", x.savePath), f.field("saveInterval", x.saveInterval).fallback(1),
                              f.field("startId", x.startId).fallback(-1));
}

class ArmorLabeler final : public HubHelper<caf::event_based_actor, ArmorLabelerSettings> {
    Identifier mKey;
    int mCurrentId;

public:
    ArmorLabeler(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) } {
        mCurrentId = mConfig.startId == -1 ? getMaxFileId(mConfig.savePath) + 1 : mConfig.startId;
    }

    // get max file id
    static int getMaxFileId(const std::string& folderPath) {
        int maxId = 0;
        for(const auto& entry : fs::directory_iterator(folderPath)) {
            const auto& path = entry.path();
            if(entry.is_regular_file()) {
                std::string filename = path.stem().string();
                try {
                    int id = std::stoi(filename);
                    maxId = std::max(maxId, id);
                } catch(const std::invalid_argument& e) {
                    logError("Invalid filename format");
                }
            }
        }

        return maxId;
    }

    static void writeFile(const cv::Mat& src, const std::string& label, const std::string& savePath, const int newId) {
        char newJpgFilename[256], newTextFilename[256];
        sprintf(newJpgFilename, "%s/%08d.jpg", savePath.c_str(), newId);
        sprintf(newTextFilename, "%s/%08d.txt", savePath.c_str(), newId);

        std::ofstream textFile(newTextFilename);
        textFile << label << "\n";
        textFile.close();

        cv::imwrite(newJpgFilename, src);
    }

    caf::behavior make_behavior() override {
        return { [](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); },
                 [&](armor_detect_available_atom, Identifier key) {
                     ACTOR_PROTOCOL_CHECK(armor_detect_available_atom, TypedIdentifier<DetectedArmorArray>);
                     ACTOR_EXCEPTION_PROBE();

                     auto res = BlackBoard::instance().get<DetectedArmorArray>(key).value();

                     cv::Mat showImg;
                     res.frame.frame.copyTo(showImg);

                     double width = showImg.size().width;
                     double height = showImg.size().height;

                     if(res.armors.empty()) {
                         return;
                     }

                     std::string label;

                     for(const auto& armor : res.armors) {
                         auto id = armor.robotType;
                         int color = static_cast<int>(armor.robotColor);
                         auto robotTag = static_cast<int>(color) * 9 + static_cast<int>(id);

                         auto armorPoints = armor.light4Point;

                         label += fmt::format("{} {} {} {} {} {} {} {} {}\n", robotTag, armorPoints[0].x / width,
                                              armorPoints[0].y / height, armorPoints[1].x / width, armorPoints[1].y / height,
                                              armorPoints[2].x / width, armorPoints[2].y / height, armorPoints[3].x / width,
                                              armorPoints[3].y / height);
                     }
                     writeFile(showImg, label, mConfig.savePath, mCurrentId);
                     sleep(mConfig.saveInterval);
                     mCurrentId++;
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorLabeler);
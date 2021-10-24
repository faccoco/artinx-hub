#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/blocking_actor.hpp>
#include <caf/event_based_actor.hpp>
#include <caf/exit_reason.hpp>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>
#include <httplib.h>
#include <opencv2/opencv.hpp>
#include <optional>
#ifdef ARTINXHUB_WINDOWS
#define NOMINMAX
#include <Windows.h>
#endif

class HttpServer final : public HubHelper<caf::event_based_actor, void> {
    httplib::Server mServer;
    cv::Mat mImage;
    uint64_t mImageFrom = 0;
    std::unordered_map<uint64_t, bool> mFilter; // false -> being filtered
    std::mutex mMutex;
    std::thread mListener;

    std::streambuf* clogBuffer;
    std::stringstream logStream;

    void modifyParameter(std::string path, std::string value) {}
    std::string generateParameterJson() {
        return "hello world!    clock " + std::to_string(::clock());
    }

    std::optional<std::vector<uchar>> generateImageData(std::string path) {
        if(mImage.empty() || !mFilter[mImageFrom])
            return std::nullopt;
        std::unique_lock<std::mutex> guard{ mMutex };
        auto img = mImage;
        guard.unlock();
        std::vector<uchar> data;
        if(!cv::imencode(".jpg", img, data))
            return std::nullopt;
        return data;
    }
    std::string generateStatusJson() {
        return "hello world!    clock " + std::to_string(::clock());
    }
    std::string generateProfileJson() {
        return "hello world!    clock " + std::to_string(::clock());
    }
    std::string generateFilterJson() {
        nlohmann::json result = nlohmann::json::array();
        for (auto &v : mFilter) {
            result.push_back({std::to_string(v.first), v.second});
        }
        return result.dump();
    }

public:
    HttpServer(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, clogBuffer(std::clog.rdbuf()) {

        std::clog.rdbuf(logStream.rdbuf());

        mServer.set_mount_point("/pages", "./pages");

        mServer.Get("/status", [this](const httplib::Request&, httplib::Response& res) {
            res.set_content(generateStatusJson(), "text/plain");
        });
        mServer.Get("/profile", [this](const httplib::Request&, httplib::Response& res) {
            res.set_content(generateProfileJson(), "text/plain");
        });
        mServer.Get("/parameters", [this](const httplib::Request& req, httplib::Response& res) {
            res.set_content(generateParameterJson(), "text/plain");
        });
        mServer.Get("/imgs", [this](const httplib::Request& req, httplib::Response& res) {
            if(auto img = generateImageData("")) {
                const auto& data = img.value();
                res.set_content(reinterpret_cast<const char*>(data.data()), data.size(), "blob");
            }
        });
        mServer.Get("/log", [this](const httplib::Request& req, httplib::Response& res) {
            res.set_content(logStream.str(), "text/plain");
            logStream.str("");
        });
        mServer.Post("/filter", [this](const httplib::Request& req, httplib::Response& res) {
            if (req.body.empty()) {
                res.set_content(generateFilterJson(), "text/plain");
                return;
            }
            auto j = nlohmann::json::parse(req.body);
            for (auto &[k, v] : j.items()) {
                mFilter[std::stoull(k)] = v;
            }
            res.set_content("", "text/plain");
        });
        mServer.Get("/exit", [this](const httplib::Request& req, httplib::Response& res) {
            mServer.stop();
            terminateSystem(*this, true);
        });
        mListener = std::thread{ [this] { mServer.listen("localhost", 8080); } };
    }
    ~HttpServer() override {
        std::clog.rdbuf(clogBuffer);
        mListener.detach();
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
#if defined(ARTINXHUB_WINDOWS)
                    ShellExecuteA(nullptr, "open", "http://127.0.0.1:8080/pages/Main.html", nullptr, nullptr, SW_SHOWNORMAL);
#elif defined(ARTINXHUB_LINUX)
                    ::system("xdg-open http://127.0.0.1:8080/pages/index.html");
#endif
                },
                 [this](image_frame_atom, Identifier key) {
                     std::lock_guard<std::mutex> guard{ mMutex };
                     mImage = BlackBoard::instance().get<CameraFrame>(key).value().frame;
                     mImageFrom = key.val;
                     if (mFilter.find(key.val) == mFilter.cend()) {
                        mFilter[key.val] = true;
                     }
                 } };
    }
};

HUB_REGISTER_CLASS(HttpServer);

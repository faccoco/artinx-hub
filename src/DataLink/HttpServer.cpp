#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Utility.hpp"
#include <caf/blocking_actor.hpp>
#include <caf/event_based_actor.hpp>
#include <caf/exit_reason.hpp>
#include <cstdint>
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#ifdef ARTINXHUB_WINDOWS
#define NOMINMAX
#include <Windows.h>
#endif

struct ImageWithFilter {
    cv::Mat image;
    bool isEnable{ true };
};

class HttpServer final : public HubHelper<caf::event_based_actor, void> {
    httplib::Server mServer;
    std::unordered_map<uint64_t, ImageWithFilter> mImage;
    std::mutex mMutex;
    std::thread mListener;

    std::streambuf* cLogBuffer;
    std::stringstream logStream;

    void modifyParameter(std::string path, std::string value) {}
    std::string generateParameterJson() {
        return "hello world!    clock " + std::to_string(::clock());
    }

    std::optional<std::vector<uchar>> generateImageData(const std::string& path) {
        if(path.empty())
            return std::nullopt;

        uint64_t id;
        try {
            id = std::stoull(path);
        } catch(std::exception& e) {
            std::clog << e.what() << std::endl;
            return std::nullopt;
        }

        std::unique_lock<std::mutex> guard{ mMutex };
        if(mImage.find(id) == mImage.end() || !mImage[id].isEnable)
            return std::nullopt;
        auto img = mImage[id].image;
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
        std::lock_guard<std::mutex> guard{ mMutex };
        for(const auto& v : mImage) {
            result.push_back({ std::to_string(v.first), v.second.isEnable });
        }
        return result.dump();
    }

public:
    HttpServer(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config }, cLogBuffer{ std::clog.rdbuf() } {

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
        mServer.Get(R"(/img/(\d+)/.*)", [this](const httplib::Request& req, httplib::Response& res) {
            if(auto img = generateImageData(req.matches[1])) {
                const auto& data = img.value();
                res.set_content(reinterpret_cast<const char*>(data.data()), data.size(), "blob");
            }
        });
        mServer.Get("/log", [this](const httplib::Request& req, httplib::Response& res) {
            res.set_content(logStream.str(), "text/plain");
            logStream.str("");
        });
        mServer.Post("/filter", [this](const httplib::Request& req, httplib::Response& res) {
            if(req.body.empty()) {
                res.set_content(generateFilterJson(), "text/plain");
                return;
            }
            auto j = nlohmann::json::parse(req.body);
            std::lock_guard<std::mutex> guard{ mMutex };
            for(auto& [k, v] : j.items()) {
                mImage[std::stoull(k)].isEnable = v;
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
        std::clog.rdbuf(cLogBuffer);
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
                     mImage[key.val].image = BlackBoard::instance().get<CameraFrame>(key).value().frame;
                 } };
    }
};

HUB_REGISTER_CLASS(HttpServer);

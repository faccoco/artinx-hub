#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include <caf/actor_ostream.hpp>
#include <caf/blocking_actor.hpp>
#include <caf/event_based_actor.hpp>
#include <caf/exit_reason.hpp>
#include <cstdint>
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
    std::mutex mMutex;
    std::thread mListener;

    void modifyParameter(std::string path, std::string value) {}
    std::string generateParameterJson() {
        return "hello world!    clock " + std::to_string(::clock());
    }

    std::optional<std::vector<uchar>> generateImageData(std::string path) {
        if(mImage.empty())
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

public:
    HttpServer(caf::actor_config& base, const HubConfig& config) : HubHelper{ base, config } {
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
        mServer.Get("/exit", [this](const httplib::Request& req, httplib::Response& res) {
            mServer.stop();

            std::exit(0);
        });

        mListener = std::thread{ [this] { mServer.listen("127.0.0.1", 8080); } };
    }
    ~HttpServer() {
        mListener.detach();
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) {
#if defined(ARTINXHUB_WINDOWS)
                    ShellExecuteA(nullptr, "open", "http://127.0.0.1:8080/pages/Main.html", nullptr, nullptr, SW_SHOWNORMAL);
#elif defined(ARTINXHUB_LINUX)
                    ::system("xdg-open http://127.0.0.1:8080/pages/Main.html");
#endif
                },
                 [this](const cv::Mat& img) {
                     std::lock_guard<std::mutex> guard{ mMutex };
                     mImage = img;
                 } };
    }
};

HUB_REGISTER_CLASS(HttpServer);

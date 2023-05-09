#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "RadarInfo.hpp"
#include "SuppressWarningBegin.hpp"
#include "Utility.hpp"

#include <caf/blocking_actor.hpp>
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <fmt/core.h>
#include <functional>
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

// #define CPPHTTPLIB_SEND_FLAGS 0x4000

#ifdef ARTINXHUB_WINDOWS
#define NOMINMAX
#include <Windows.h>
#endif

#ifdef ARTINXHUB_LINUX
#include <net/if.h>
#include <sys/ioctl.h>
#endif

#include "SuppressWarningEnd.hpp"

struct ImageWithFilter {
    std::string_view name;
    cv::Mat image;
    bool isEnable = true;
};

struct HttpServerSettings final {
    bool enableRadar;
    uint32_t radarPointsNum;
};

template <typename Inspector>
bool inspect(Inspector& f, HttpServerSettings& x) {
    return f.object(x).fields(f.field("enableRadar", x.enableRadar).fallback(false),
                              f.field("radarPointsNum", x.radarPointsNum).fallback(6));
}

class HttpServer final : public HubHelper<caf::event_based_actor, HttpServerSettings, radar_locate_request_atom> {
    httplib::Server mServer;
    std::unordered_map<uint64_t, ImageWithFilter> mImage;
    std::mutex mMutex;
    std::thread mListener;

    std::string mhostIpAddress;
    std::streambuf* mClogBuffer;

#ifdef ARTINX_RADAR
    uint64_t radarKey;
    CameraInfo radarCameraInfo;
    std::reference_wrapper<RadarTransform> radarTrans;
#endif

    Identifier mKey;

#if defined(ARTINXHUB_LINUX)
#define ETH_NAME "wlp0s20f3"
    std::string getHostIpAddress() {
        int sockFd;
        struct sockaddr_in sockIn;
        struct ifreq ifReq;

        sockFd = socket(AF_INET, SOCK_DGRAM, 0);
        if(sockFd != -1) {
            strncpy(ifReq.ifr_name, ETH_NAME, IFNAMSIZ);   // Interface name
            if(ioctl(sockFd, SIOCGIFADDR, &ifReq) == 0) {  // SIOCGIFADDR obtain interface address
                memcpy(&sockIn, &ifReq.ifr_addr, sizeof(ifReq.ifr_addr));
                return inet_ntoa(sockIn.sin_addr);
            }
        }
        return "127.0.0.1";
    }
#endif

    std::optional<std::vector<uchar>> generateImageData(const std::string& path) {
        if(path.empty())
            return std::nullopt;

        uint64_t id;
        try {
            id = std::stoull(path);
        } catch(std::exception&) {
            return std::nullopt;
        }

        std::unique_lock guard{ mMutex };
        if(!mImage.count(id) || !mImage[id].isEnable)
            return std::nullopt;
        auto img = mImage[id].image;
        guard.unlock();

        std::vector<uchar> data;
        if(!cv::imencode(".jpg", img, data))
            return std::nullopt;
        return data;
    }

    std::string generateFilterJson() {
        nlohmann::json result = nlohmann::json::array();
        std::lock_guard<std::mutex> guard{ mMutex };
        for(const auto& v : mImage)
            result.push_back({ v.second.name.data() + std::string("-") + std::to_string(v.first), v.second.isEnable });
        return result.dump();
    }

public:
#ifdef ARTINX_RADAR
    HttpServer(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mClogBuffer{ std::clog.rdbuf() }, radarTrans(RadarTransform::Instance()),
          mKey{ generateKey(this) } {
#else
    HttpServer(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mClogBuffer{ std::clog.rdbuf() }, mKey{ generateKey(this) } {
#endif
              using json = nlohmann::json;
#if defined(ARTINXHUB_WINDOWS)
    mhostIpAddress = "127.0.0.1";
#elif defined(ARTINXHUB_LINUX)
        mhostIpAddress = getHostIpAddress();
#endif
    // std::clog.rdbuf(mLogStream.rdbuf());

    mServer.set_mount_point("/pages", "./pages");

    mServer.Get("/status", [](const httplib::Request&, httplib::Response& res) {
        res.set_content("hello world!    clock " + std::to_string(::clock()), "text/plain");
    });
    //        mServer.Get("/profile", [this](const httplib::Request&, httplib::Response& res) {
    //            res.set_content("hello world!    clock " + std::to_string(::clock());, "text/plain");
    //        });
    //        mServer.Get("/parameters", [this](const httplib::Request& req, httplib::Response& res) {
    //            res.set_content("hello world!    clock " + std::to_string(::clock()), "text/plain");
    //        });
    mServer.Get(R"(/img/.*?(\d+).*)", [this](const httplib::Request& req, httplib::Response& res) {
        if(req.matches.empty())
            return;
        const auto&& path = req.matches[1].str();
        res.set_content_provider(
            "multipart/x-mixed-replace;boundary=MJP",
            [this, path](size_t, httplib::DataSink& sink) {
                if(const auto&& img = generateImageData(path)) {
                    auto& vec = img.value();
                    sink.os << "--MJP\r\n"
                               "Content-Type: image/jpeg\r\n"
                               "Content-Length: "
                            << vec.size() << "\r\n\r\n";
                    sink.os.write(reinterpret_cast<const char*>(vec.data()), static_cast<long>(vec.size()));
                }
                return true;
            },
            [](bool) {});
    });

    mServer.Get("/watch", [](const httplib::Request&, httplib::Response& res) {
        res.set_content(nlohmann::json(HubLogger::watches).dump(), "application/json");
    });

    static bool filterInit = false;
    mServer.Post("/filter", [this](const httplib::Request& req, httplib::Response& res) {
        if(!filterInit && req.body.empty()) {
            res.set_content(generateFilterJson(), "text/plain");
            filterInit = true;
            return;
        } else if(req.body.empty()) {
            res.set_content("{}", "text/plain");
        } else {
            auto reqJson = json::parse(req.body);
            std::lock_guard guard{ mMutex };
            for(const auto& [str, val] : reqJson.items()) {
                uint64_t key = std::stoull(str.substr(str.find('-') + 1));
                mImage[key].isEnable = val;
            }
            res.set_content("{}", "text/plain");
        }
    });

#ifdef ARTINX_RADAR
    mServer.Get(R"(/img/RadarCenter)", [this](const httplib::Request& req, httplib::Response& res) {
        res.set_content_provider("multipart/x-mixed-replace;boundary=MJP",
                                 [this](size_t, httplib::DataSink& sink) {
                                     if(const auto&& img = generateImageData(std::to_string(radarKey))) {
                                         auto& vec = img.value();
                                         sink.os << "--MJP\r\n"
                                                    "Content-Type: image/jpeg\r\n"
                                                    "Content-Length: "
                                                 << vec.size() << "\r\n\r\n";
                                         sink.os.write(reinterpret_cast<const char*>(vec.data()), static_cast<long>(vec.size()));
                                     }
                                     return true;
                                 },
                                 [](bool){});
    });
    if(mConfig.enableRadar) {
        mServer.Get("/radar", [](const httplib::Request&, httplib::Response& res) {
            res.set_content(json("true").dump(), "application/json");
        });

        //            mServer.Get("/log", [this](const httplib::Request&, httplib::Response& res) {
        //                std::string send("trans mat:\n");
        //                for(int i = 0; i < 4; ++i) {
        //                    for(int j = 0; j < 4; ++j)
        //                        send += std::to_string(radarSuccessTransform.trans[i][j]) + " ";
        //                    send += "\n";
        //                }
        //                send += "rotate mat:\n";
        //                for(int i = 0; i < 3; ++i) {
        //                    for(int j = 0; j < 3; ++j)
        //                        send += std::to_string(radarSuccessTransform.rotate[i][j]) + " ";
        //                    send += "\n";
        //                }
        //                res.set_content(json(send).dump(), "text/plain");
        //            });

        mServer.Post("/radar_points", [this](const httplib::Request& req, httplib::Response& res) {
            auto allPoints = json::parse(req.body);
            RadarCameraPoints data;
            data.info = radarCameraInfo;
            for(uint32_t i = 0; i < mConfig.radarPointsNum; ++i)
                data.points.emplace_back(static_cast<int>(allPoints[i]["x"]), static_cast<int>(allPoints[i]["y"]));
            sendAll(radar_locate_request_atom_v, BlackBoard::instance().updateSync(mKey, std::move(data)));
            res.set_content(json(json("success")).dump(), "text/plain");
        });
    }
#endif

    mServer.Get("/exit", [this](const httplib::Request&, httplib::Response&) {
        mServer.stop();
        terminateSystem(*this, true);
    });

    mListener = std::thread{ [this] { mServer.listen(mhostIpAddress.c_str(), 5630); } };
} ~HttpServer() override {
    std::clog.rdbuf(mClogBuffer);
    mListener.detach();
}
caf::behavior make_behavior() override {
    return { [this](start_atom) {
                ACTOR_PROTOCOL_CHECK(start_atom);
                [[maybe_unused]] const auto res =
#if defined(ARTINXHUB_WINDOWS)
                    ShellExecuteA(nullptr, "open", "http://localhost:5630/pages/index.html", nullptr, nullptr, SW_SHOWNORMAL);
#elif defined(ARTINXHUB_LINUX)
                        ::system(fmt::format("xdg-open http://{}:5630/pages/index.html", mhostIpAddress).c_str());
#else
                    0;
#endif
            },
             [this](image_frame_atom, Identifier key) {
                 ACTOR_PROTOCOL_CHECK(image_frame_atom, TypedIdentifier<CameraFrame, std::string_view>);
                 std::lock_guard<std::mutex> guard{ mMutex };
                 auto data = BlackBoard::instance().get<CameraFrame, std::string_view>(key);
                 auto [cameraFrame, name] = data.value();
#ifdef ARTINX_RADAR
                 if(name == "RadarCenter")
                     radarKey = key.val;
                 radarCameraInfo = cameraFrame.info;
#endif
                 mImage[key.val] = { name, cameraFrame.frame, true };
             }
#ifdef ARTINX_RADAR
             ,
             [](radar_locate_succeed_atom) {
                 // TODO
             }
#endif
    };
}
}
;

HUB_REGISTER_CLASS(HttpServer);

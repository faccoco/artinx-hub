#include "BlackBoard.hpp"
#include "DataDesc.hpp"
#include "DetectedTarget.hpp"
#include "ExceptionProbe.hpp"
#include "HeadInfo.hpp"
#include "Hub.hpp"
#include <caf/event_based_actor.hpp>
#include <cstdint>
#include <glm/glm.hpp>
#include <opencv2/video/tracking.hpp>

struct ArmorPredictorSettings final {
    double maxDistance;
    double step;
    double fixedDelay;
};

template <class Inspector>
bool inspect(Inspector& f, ArmorPredictorSettings& x) {
    return f.object(x).fields(f.field("maxDistance", x.maxDistance), f.field("step", x.step),
                              f.field("fixedDelay", x.fixedDelay));
}

class Filter final {
    cv::KalmanFilter mKF{ 2, 1, 0, CV_64F };
    double mLastValue;

public:
    Filter(const double value) : mLastValue{ value } {
        mKF.transitionMatrix = (cv::Mat_<double>(2, 2) << 1, 1, 0, 1);
        setIdentity(mKF.measurementMatrix);
        setIdentity(mKF.processNoiseCov, cv::Scalar::all(1e-5));
        setIdentity(mKF.measurementNoiseCov, cv::Scalar::all(1e-1));
        setIdentity(mKF.errorCovPost, cv::Scalar::all(1));

        mKF.statePost = (cv::Mat_<double>(2, 1) << value, 0);
    }
    std::pair<double, double> update(const double value, const int32_t dt, const double predictTime) {
        for(int32_t idx = 0; idx < dt; ++idx)
            mKF.predict();
        mKF.correct((cv::Mat_<double>(1, 1) << value));
        mLastValue = value;
        return { mKF.statePost.at<double>(0) + mKF.statePost.at<double>(1) * predictTime, mKF.statePost.at<double>(1) };
    }
};

class SpaceFilter final {
    Filter mX, mY, mZ;
    glm::dvec3 mLast;

public:
    explicit SpaceFilter(const glm::dvec3& pos) : mX{ pos.x }, mY{ pos.y }, mZ{ pos.z }, mLast{ pos } {}
    double distance(const glm::dvec3& pos) const noexcept {
        return glm::distance(mLast, pos);
    }
    std::pair<glm::dvec3, glm::dvec3> update(const glm::dvec3& pos, const int32_t dt, const double predictTime) {
        mLast = pos;
        const auto [px, vx] = mX.update(pos.x, dt, predictTime);
        const auto [py, vy] = mY.update(pos.y, dt, predictTime);
        const auto [pz, vz] = mZ.update(pos.z, dt, predictTime);
        return { { px, py, pz }, { vx, vy, vz } };
    }
};

static std::vector<uint32_t> solveKM(uint32_t n, uint32_t m, const std::vector<double>& w) {
    const auto size = std::max(n, m) + 1;
    std::vector<double> lh(size), rh(size), slack(size);
    std::vector<uint32_t> pair(size), pre(size);
    std::vector<bool> flag(size);

    const auto reset = [](auto& c, auto value) { std::fill(c.begin() + 1, c.end(), value); };

    const auto aug = [&](uint32_t s) {
        reset(flag, false);
        reset(pre, 0);
        reset(slack, 1e9);
        pair[0] = s;
        uint32_t u = 0;
        do {
            uint32_t v = pair[u], nxt;
            double minh = 1e9;
            flag[u] = true;
            for(uint32_t i = 1; i <= m; ++i)
                if(!flag[i]) {
                    const auto delta = lh[v] + rh[i] - w[(v - 1) * n + i - 1];
                    if(delta < slack[i])
                        slack[i] = delta, pre[i] = u;
                    if(minh > slack[i])
                        minh = slack[i], nxt = i;
                }
            for(uint32_t i = 0; i <= m; ++i)
                if(flag[i])
                    lh[pair[i]] -= minh, rh[i] += minh;
                else
                    slack[i] -= minh;
            u = nxt;
        } while(pair[u]);
        while(u) {
            int p = pre[u];
            pair[u] = pair[p];
            u = p;
        }
    };

    for(int i = 1; i <= n; ++i) {
        double maxh = 0;
        for(int j = 1; j <= m; ++j)
            maxh = std::fmax(maxh, w[(i - 1) * n + j - 1]);
        lh[i] = maxh;
    }
    reset(rh, 0.0);
    reset(pair, 0);
    for(int i = 1; i <= n; ++i)
        aug(i);
    std::vector<uint32_t> res(m);
    for(uint32_t idx = 0; idx < m; ++idx)
        res[idx] = pair[idx + 1] - 1;

    return res;
}

class ArmorPredictor final : public HubHelper<caf::event_based_actor, ArmorPredictorSettings, detect_available_atom> {
    Identifier mKey;
    TimePoint mLastUpdate{ 0s };
    int64_t mAccumulatedTime = 0, mTimeStep = 100'000'000;
    std::vector<SpaceFilter> mTargets;
    static constexpr double timeScale = static_cast<double>(Clock::period::den) / static_cast<double>(Clock::period::num);

public:
    ArmorPredictor(caf::actor_config& base, const HubConfig& config)
        : HubHelper{ base, config }, mKey{ typeid(ArmorPredictor).hash_code() } {
        mTimeStep = static_cast<int64_t>(mConfig.step * timeScale);
    }
    caf::behavior make_behavior() override {
        return { [this](start_atom) {},
                 [&](detect_available_atom, Identifier key) {
                     ACTOR_EXCEPTION_PROBE();

                     auto res = BlackBoard::instance().get<DetectedTargetArray>(key).value();

                     if(res.lastUpdate - mLastUpdate < 1s)
                         mAccumulatedTime += (res.lastUpdate - mLastUpdate).count();
                     else {
                         mTargets.clear();
                         mAccumulatedTime = 0;
                     }

                     mLastUpdate = res.lastUpdate;

                     std::vector<double> w;
                     w.reserve(mTargets.size() * res.targets.size());
                     for(auto& old : mTargets)
                         for(auto& cur : res.targets) {
                             const auto dist = old.distance(cur.center.raw());
                             if(dist < mConfig.maxDistance)
                                 w.push_back(1e7 - dist);
                             else
                                 w.push_back(0.0);
                         }

                     const auto match = solveKM(mTargets.size(), res.targets.size(), w);

                     const auto dt = static_cast<int32_t>(mAccumulatedTime / mTimeStep);
                     mAccumulatedTime -= dt * mTimeStep;

                     const auto predictTime =
                         (static_cast<double>(mAccumulatedTime + (SynchronizedClock::instance().now() - mLastUpdate).count()) /
                              timeScale +
                          mConfig.fixedDelay) /
                         mConfig.step;

                     std::vector<bool> use(mTargets.size());
                     for(uint32_t idx = 0; idx < res.targets.size(); ++idx) {
                         auto& cur = res.targets[idx];
                         if(match[idx] != std::numeric_limits<uint32_t>::max()) {
                             auto& old = mTargets[match[idx]];
                             const auto [p, v] = old.update(cur.center.raw(), dt, predictTime);
                             cur.center = decltype(cur.center){ p };
                             cur.velocity = decltype(cur.velocity){ v };
                             use[match[idx]] = true;
                         } else
                             mTargets.push_back(SpaceFilter{ cur.center.raw() });
                     }
                     for(int32_t idx = static_cast<int32_t>(use.size()) - 1; idx >= 0; --idx)
                         if(!use[idx])
                             mTargets.erase(mTargets.cbegin() + idx);

                     BlackBoard::instance().updateSync(mKey, std::move(res));
                     sendAll(detect_available_atom_v, mKey);
                 } };
    }
};

HUB_REGISTER_CLASS(ArmorPredictor);

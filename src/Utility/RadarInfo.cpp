#ifdef ARTINX_RADAR
#include "RadarInfo.hpp"
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <shared_mutex>

void RadarTransform::setReady() {
    flag.store(true, std::memory_order_release);
}

bool RadarTransform::isReady() {
    return flag.load(std::memory_order_acquire);
}

glm::dmat4 RadarTransform::load() {
    std::shared_lock lock(mutex);
    return trans;
}

void RadarTransform::store(const glm::dmat4& rhs) {
    std::lock_guard guard(mutex);
    trans = rhs;
}

RadarTransform& RadarTransform::instant() {
    static RadarTransform instance;
    return instance;
}

void RadarPerspectiveTransform::setReady() {
    flag.store(true, std::memory_order_release);
}

bool RadarPerspectiveTransform::isReady() {
    return flag.load(std::memory_order_acquire);
}

cv::Mat RadarPerspectiveTransform::load() {
    std::shared_lock lock(mutex);
    return trans;
}

void RadarPerspectiveTransform::store(const cv::Mat& rhs) {
    std::lock_guard guard(mutex);
    trans = rhs;
}

RadarPerspectiveTransform& RadarPerspectiveTransform::instant() {
    static RadarPerspectiveTransform instance;
    return instance;
}
#endif

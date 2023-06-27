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
    std::shared_lock<std::shared_mutex> lock(mutex, std::defer_lock);
    return trans;
}

void RadarTransform::store(const glm::dmat4& rhs) {
    std::lock_guard<std::shared_mutex> guard(mutex);
    trans = rhs;
}

RadarTransform& RadarTransform::instant() {
    static RadarTransform instance;
    return instance;
}

void RadarPerspectTransform::setReady() {
    flag.store(true, std::memory_order_release);
}

bool RadarPerspectTransform::isReady() {
    return flag.load(std::memory_order_acquire);
}

cv::Mat RadarPerspectTransform::load() {
    std::shared_lock<std::shared_mutex> lock(mutex, std::defer_lock);
    lock.lock();
    return trans;
}

void RadarPerspectTransform::store(const cv::Mat& rhs) {
    std::lock_guard<std::shared_mutex> guard(mutex);
    trans = rhs;
}

RadarPerspectTransform& RadarPerspectTransform::instant() {
    static RadarPerspectTransform instance;
    return instance;
}
#endif
